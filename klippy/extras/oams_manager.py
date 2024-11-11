import logging
import time
from functools import partial
from collections import deque

PAUSE_DISTANCE = 60
F1S_CURRENT_SAMPLES = 3

class OAMSManager:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.filament_groups = {}
        self.oams = {}
        self._initialize_oams()
        self._initialize_filament_groups()
        
        # runout variables
        self.current_group = None
        self.current_spool = None
        self.runout_position = None
        self.runout_after_position = None
        self.monitor_spool_timer = None
        self.monitor_pause_timer = None
        self.monitor_load_next_spool_timer = None
        self.f1s_current_max = config.getfloat("f1s_current_max", 0.7)
        self.f1s_current_max_samples = config.getint("f1s_current_max_samples", F1S_CURRENT_SAMPLES)
        self.f1s_current_samples = deque(maxlen=self.f1s_current_max_samples)
        self.monitor_timers = []  # Initialize monitor_timers
        
        self.reload_before_toolhead_distance = config.getfloat("reload_before_toolhead_distance", 0.0)
        
        self.register_commands()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    # these are available to the gcode
    def get_status(self, eventtime):
        return {"current_group": self.current_group}
        
    def handle_ready(self):
        self.current_group, current_oam, current_spool_idx = self.determine_current_loaded_group()
        if current_oam is not None and current_spool_idx is not None:
            self.current_spool = (current_oam, current_spool_idx)
        else:
            self.current_spool = None
        # start a monitor to check if the spool has not runout
        reactor = self.printer.get_reactor()
        self.monitor_spool_timer = reactor.register_timer(self._monitor_spool, reactor.NOW)

    def _log_status(self, is_printing):
        logging.info(
            """
    OAMS: print status = %s, current_group = %s, current_spool_oam = %s, current_spool_idx = %s, ran_out = %s, runout_position = %s, traveled_after_runout = %s 
            """ 
            % (is_printing, 
                self.current_group, 
                None if self.current_spool is None else self.current_spool[0].name,
                None if self.current_spool is None else self.current_spool[1],
                self.current_spool is not None and not bool(self.current_spool[0].hub_hes_value[self.current_spool[1]]), 
                self.runout_position, self.runout_after_position))
    
    def _pause_before_coasting(self, eventtime, initial_position, pause_distance):
        extruder = self.printer.lookup_object("extruder")
        current_position = extruder.last_position
        traveled_distance = current_position - initial_position
        if traveled_distance >= pause_distance:
            logging.info("OAMS: Pause complete, coasting the follower.")
            self.current_spool[0].set_oams_follower(0, 1)
            self._register_load_next_spool_timer(eventtime, pause_distance)
            return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def _register_pause_timer(self, eventtime, pause_distance):
        logging.info("OAMS: Filament runout detected, pausing for %s mm before coasting the follower." % pause_distance)
        extruder = self.printer.lookup_object("extruder")
        initial_position = extruder.last_position
        self.monitor_pause_timer = self.printer.get_reactor().register_timer(
            lambda et: self._pause_before_coasting(et, initial_position, pause_distance), eventtime)
    
    def _load_next_spool(self, eventtime, pause_distance):
        extruder = self.printer.lookup_object("extruder")
        if self.runout_position is None:
            self.runout_position = extruder.last_position
            logging.info("OAMS: Runout position set to %s" % self.runout_position)
        else:
            self.runout_after_position = extruder.last_position - self.runout_position
            logging.info("OAMS: Traveled after runout: %s" % self.runout_after_position)
            if self.runout_after_position + pause_distance + self.reload_before_toolhead_distance > self.current_spool[0].filament_path_length / 1.14:
                logging.info("OAMS: Loading next spool in the filament group.")
                for (oam, bay_index) in self.filament_groups[self.current_group].bays:
                    if oam.is_bay_ready(bay_index):
                        success, message = oam.load_spool(bay_index)
                        if success:
                            logging.info("OAMS: Successfully loaded spool in bay %s of OAM %s" % (bay_index, oam.name))
                            self.current_spool = (oam, bay_index)
                            self.runout_position = None
                            self.runout_after_position = None
                            self._register_monitor_spool_timer()
                            return self.printer.get_reactor().NEVER
                        else:
                            logging.error("OAMS: Failed to load spool: %s" % message)
                            raise Exception(message)
                self._pause_print()
        return eventtime + 1.0
    
    def _register_load_next_spool_timer(self, eventtime, pause_distance):
        logging.info("OAMS: Registering timer to load next spool.")
        self.monitor_load_next_spool_timer = self.printer.get_reactor().register_timer(
            lambda et: self._load_next_spool(et, pause_distance), eventtime)
    
    def _pause_print(self):
        logging.info("OAMS: No spool available, pausing the print.")
        gcode = self.printer.lookup_object("gcode")
        message = "Print has been paused due to filament runout on group " + self.current_group
        gcode.run_script("M118 " + message)
        gcode.run_script("M114 " + message)
        gcode.run_script("PAUSE")
        self.current_group = None
        self.current_spool = None
        self.runout_position = None
        self.runout_after_position = None
        self._register_monitor_spool_timer()
    
    def _monitor_spool(self, eventtime):
        idle_timeout = self.printer.lookup_object("idle_timeout")
        is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
    
        if is_printing and \
            self.current_group is not None and \
            self.current_spool is not None and \
            not bool(self.current_spool[0].hub_hes_value[self.current_spool[1]]):
            self._register_pause_timer(eventtime, PAUSE_DISTANCE)
            return self.printer.get_reactor().NEVER
    
        return eventtime + 1.0
    
    def _register_monitor_spool_timer(self):
        reactor = self.printer.get_reactor()
        self.monitor_spool_timer = reactor.register_timer(self._monitor_spool, reactor.NOW)

    def _initialize_oams(self):
        for (name, oam) in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self):
        for (name, group) in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self):
        for group_name, group in self.filament_groups.items():
            for (oam, bay_index) in group.bays:
                if oam.is_bay_loaded(bay_index):
                    return group_name, oam, bay_index
        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "OAMSM_UNLOAD_FILAMENT",
            self.cmd_UNLOAD_FILAMENT,
            desc=self.cmd_UNLOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_LOAD_FILAMENT",
            self.cmd_LOAD_FILAMENT,
            desc=self.cmd_LOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "OAMSM_FOLLOWER",
            self.cmd_FOLLOWER,
            desc=self.cmd_FOLLOWER_help,
        )
        
        gcode.register_command(
            "OAMSM_CURRENT_LOADED_GROUP",
            self.cmd_CURRENT_LOADED_GROUP,
            desc=self.cmd_CURRENT_LOADED_GROUP_help,
        )
    
    cmd_CURRENT_LOADED_GROUP_help = "Get the current loaded group"
    def cmd_CURRENT_LOADED_GROUP(self, gcmd):
        group_name, _, _ = self.determine_current_loaded_group()
        if group_name is not None:
            gcmd.respond_info(group_name)
        else:
            gcmd.respond_info("No group is currently loaded")
        return  # Add return statement
    
    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        if enable is None:
            gcmd.respond_info("Missing ENABLE parameter")
            return  # Add return statement
        direction = gcmd.get_int('DIRECTION')
        if direction is None:
            gcmd.respond_info("Missing DIRECTION parameter")
            return  # Add return statement
        loaded = False
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                oam.set_oams_follower(enable, direction)
                loaded = True
        if not loaded:
            gcmd.respond_info("No spool is currently loaded")
        return  # Add return statement
    
    def is_printer_loaded(self):
        # determine if any of the oams has a spool loaded
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                return True
        return False
    
    cmd_UNLOAD_FILAMENT_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD_FILAMENT(self, gcmd):
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                oam.cmd_OAMS_UNLOAD_SPOOL(gcmd)
                self.current_group = None
                self.current_spool = None
                return  # Add return statement
        gcmd.respond_info("No spool is loaded in any of the OAMS")
        self.current_group = None
        return  # Add return statement
        
    cmd_LOAD_FILAMENT_help = "Load a spool from an specific group"
    def cmd_LOAD_FILAMENT(self, gcmd):
        if self.is_printer_loaded():
            gcmd.respond_info("Printer is already loaded with a spool")
            return  # Add return statement
        group_name = gcmd.get('GROUP')
        for (oam, bay_index) in self.filament_groups[group_name].bays:
            if oam.is_bay_ready(bay_index):
                success, message = oam.load_spool(bay_index)
                if success:
                    self.current_group = group_name
                    self.current_spool = (oam, bay_index)
                    gcmd.respond_info(message)
                    self.current_group = group_name
                    return  # Add return statement
                else:
                    raise gcmd.error(message)
        gcmd.respond_info("No spool available for group " + group_name)
        return  # Add return statement
        
    def _pause_printer_message(self, message):
        logging.info("OAMS: %s" % (message))
        gcode = self.printer.lookup_object("gcode")
        message = "Print has been paused: " + message
        gcode.run_script("M118 " + message)
        gcode.run_script("M114 " + message)
        gcode.run_script("PAUSE")
        
    def _monitor_unload_current(self, eventtime):
        if self.current_group is not None:
            oams = self.filament_groups[self.current_group].get_oams()
            i = oams.get_current()
            self.f1s_current_samples.append(i)
            # find average of the last samples
            avg = sum(self.f1s_current_samples) / len(self.f1s_current_samples)
            if avg > self.f1s_current_max:
                logging.info("OAMS: Current exceeded threshold, pausing print.")
                # we also want to flash the leds on the corresponding OAMS and spool bay
                oams.set_led_error(oams.current_spool, 1)
                self._pause_printer_message("Printer paused because the first stage feeder current exceeded threshold of %s" % self.f1s_current_max)
                self.stop_all_monitors()
                return self.printer.get_reactor().NEVER
        return eventtime + 1.0
    
    def start_monitors(self):
        self.monitor_timers = []
        reactor = self.printer.get_reactor()
        self.monitor_timers.append(reactor.register_timer(self._monitor_unload_current, reactor.NOW))
    
    def stop_monitors(self):
        for timer in self.monitor_timers:
            self.printer.get_reactor().unregister_timer(timer)
        self.monitor_timers = []

def load_config(config):
    return OAMSManager(config)
