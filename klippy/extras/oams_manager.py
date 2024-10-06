import logging

class OAMSManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.filament_groups = {}
        self.oams = {}
        self.config = config
        self.printer.add_object("oams_manager", self)
        self._initialize_oams()
        self._initialize_filament_groups()
        
        # runout variable
        self.current_group = None
        self.current_spool = None
        self.runout_position = None
        self.runout_after_position = None
        
        self.register_commands()
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        
    def handle_ready(self):
        self.current_group, current_oam, current_spool_idx = self.determine_current_loaded_group()
        if current_oam is not None and current_spool_idx is not None:
            self.current_spool = (current_oam, current_spool_idx)
        else:
            self.current_spool = None
        # start a monitor to check if the spool has not runout
        reactor = self.printer.get_reactor()
        self.monitor_spool_timer = reactor.register_timer(self._monitor_spool, reactor.NOW)
        
    def _monitor_spool(self, eventtime):
        # Determine "printing" status
        idle_timeout = self.printer.lookup_object("idle_timeout")
        is_printing = idle_timeout.get_status(eventtime)["state"] == "Printing"
        is_printing = True

        logging.info(
        """
OAMS: print status = %s, current_group = %s, current_spool_oam = %s, current_spool_idx = %d, ran_out = %s, runout_position = %s, traveled_after_runout = %s 
        """ 
        % (is_printing, 
            self.current_group, 
            None if self.current_spool is None else self.current_spool[0].name,
            None if self.current_spool is None else self.current_spool[1],
            self.current_spool is not None and not bool(self.current_spool[0].hub_hes_value[self.current_spool[1]]), 
            self.runout_position, self.runout_after_position))
        
        if is_printing and \
            self.current_group is not None and \
            self.current_spool is not None and \
            not bool(self.current_spool[0].hub_hes_value[self.current_spool[1]]):
                
            extruder = self.printer.lookup_object("extruder")

            # we must stop the follower
            self.current_group[0].set_oams_follower(0,1)
            
            if self.runout_position is None:
                self.runout_position = extruder.last_position
            else:
                self.runout_after_position = extruder.last_position - self.runout_position
                if self.runout_after_position > self.current_spool[0].filament_path_length / 1.14: # fudge factor calculated from the encoder and a 1meter extrusion, the actual value is 1140 clicks to a meter
                    for (oam, bay_index) in self.filament_groups[self.current_group].bays:
                        if oam.is_bay_ready(bay_index):
                            success, message = oam.load_spool(bay_index)
                            if success:
                                return eventtime + 1.0
                            else:
                                raise Exception(message)
                    # no spool is available, pause the print
                    # call gcode PAUSE macro from here
                    self.gcode = self.printer.lookup_object("gcode")
                    message = "Print has been paused due to filament runout on group " + self.current_group
                    self.gcode.run_script("M118 " + message)
                    self.gcode.run_script("M114 " + message)
                    self.gcode.run_script("PAUSE")
                    
                    self.current_group = None
                    self.current_spool = None
                    self.runout_position = None
                    self.runout_after_position = None
                    
        return eventtime + 1.0
            
            

    def _initialize_oams(self):
        for (name, oam) in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self):
        for (name, group) in self.printer.lookup_objects(module="filament_group"):
            name = name.split()[-1]
            logging.info(f"OAMS: Adding group {name}")
            self.filament_groups[name] = group
    
    # todo this is a mistake as it won't return bu the first index
    def determine_current_loaded_group(self):
        for group_name, group in self.filament_groups.items():
            for (oam, bay_index) in group.bays:
                if oam.is_bay_loaded(bay_index):
                    return group_name, oam, bay_index
        return None, None, None
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "UNLOAD_FILAMENT",
            self.cmd_UNLOAD_FILAMENT,
            desc=self.cmd_UNLOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "LOAD_FILAMENT",
            self.cmd_LOAD_FILAMENT,
            desc=self.cmd_LOAD_FILAMENT_help,
        )
        
        gcode.register_command(
            "FOLLOWER",
            self.cmd_FOLLOWER,
            desc=self.cmd_FOLLOWER_help,
        )
        
        gcode.register_command(
            "CURRENT_LOADED_GROUP",
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
        
    cmd_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_FOLLOWER(self, gcmd):
        enable = gcmd.get_int('ENABLE')
        if enable is None:
            raise gcmd.respond_error("Missing ENABLE parameter")
        direction = gcmd.get_int('DIRECTION')
        if direction is None:
            raise gcmd.respond_error("Missing DIRECTION parameter")
        for _, oam in self.oams.items():
            if oam.current_spool is not None:
                oam.set_oams_follower(enable, direction)
                return
            raise gcmd.respond_error("No spool is currently loaded")
    
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
                return
        gcmd.respond_info("No spool is loaded in any of the OAMS")
        self.current_group = None
        
    cmd_LOAD_FILAMENT_help = "Load a spool from an specific group"
    def cmd_LOAD_FILAMENT(self, gcmd):
        if self.is_printer_loaded():
            gcmd.error("Printer is already loaded with a spool")
            return
        group_name = gcmd.get('GROUP')
        for (oam, bay_index) in self.filament_groups[group_name].bays:
            if oam.is_bay_ready(bay_index):
                success, message = oam.load_spool(bay_index)
                if success:
                    gcmd.respond_info(message)
                    self.current_group = group_name
                    return
                else:
                    raise gcmd.error(message)
        gcmd.respond_info("No spool available for group " + group_name)
        

def load_config_prefix(config):
    return OAMSManager(config)

def load_config(config):
    return OAMSManager(config)
