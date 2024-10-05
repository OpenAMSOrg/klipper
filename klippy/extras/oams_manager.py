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
        self.current_group = None
        self.register_commands()
        # add handle on ready
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        
    def handle_ready(self):
        self.current_group = self.determine_current_loaded_group()

    def _initialize_oams(self):
        for (name, oam) in self.printer.lookup_objects(module="oams"):
            self.oams[name] = oam
        
    def _initialize_filament_groups(self):
        for (name, group) in self.printer.lookup_objects(module="filament_group"):
            self.filament_groups[name] = group
    
    def determine_current_loaded_group(self):
        for group_name, group in self.filament_groups.items():
            for (oam, bay_index) in group.bays:
                if oam.is_bay_loaded(bay_index):
                    return group_name
        return None 
        
    def register_commands(self):
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "UNLOAD",
            self.cmd_UNLOAD,
            desc=self.cmd_UNLOAD_help,
        )
        
        gcode.register_command(
            "LOAD_GROUP",
            self.cmd_LOAD_GROUP,
            desc=self.cmd_LOAD_GROUP_help,
        )
        
        gcode.register_command(
            "ENABLE_FOLLOWER",
            self.cmd_ENABLE_FOLLOWER,
            desc=self.cmd_ENABLE_FOLLOWER_help,
        )
        
        gcode.register_command(
            "CURRENT_LOADED_GROUP",
            self.cmd_CURRENT_LOADED_GROUP,
            desc=self.cmd_CURRENT_LOADED_GROUP_help,
        )
    
    cmd_CURRENT_LOADED_GROUP_help = "Get the current loaded group"
    def cmd_CURRENT_LOADED_GROUP(self, gcmd):
        group_name = self.determine_current_loaded_group()
        if group_name is not None:
            gcmd.respond_info(group_name)
        else:
            gcmd.respond_info("No group is currently loaded")
        
    cmd_ENABLE_FOLLOWER_help = "Enable the follower on whatever OAMS is current loaded"
    def cmd_ENABLE_FOLLOWER(self, gcmd):
        enable = gcmd.get('ENABLE')
        if enable is None:
            raise gcmd.respond_error("Missing ENABLE parameter")
        direction = gcmd.get('DIRECTION')
        if direction is None:
            raise gcmd.respond_error("Missing DIRECTION parameter")
        for oam in self.oams:
            if oam.current_spool is not None:
                oam.cmd_OAMS_ENABLE_FOLLOWER(enable, direction)
                return
            raise gcmd.respond_error("No spool is currently loaded")
    
    def is_printer_loaded(self):
        # determine if any of the oams has a spool loaded
        for oam in self.oams:
            if oam.current_spool is not None:
                return True
        return False
    
    cmd_UNLOAD_help = "Unload a spool from any of the OAMS if any is loaded"
    def cmd_UNLOAD(self, gcmd):
        for oam in self.oams:
            if oam.current_spool is not None:
                oam.cmd_OAMS_UNLOAD_SPOOL()
                self.current_group = None
                return
        gcmd.respond_info("No spool is loaded in any of the OAMS")
        self.current_group = None
        
    cmd_LOAD_GROUP_help = "Load a spool from an specific group"
    def cmd_LOAD_GROUP(self, gcmd):
        if self.is_printer_loaded():
            gcmd.respond_error("Printer is already loaded with a spool")
            return
        group_name = gcmd.get('GROUP')
        if self.determine_current_loaded_group() == group_name:
            gcmd.respond_error("Group is already loaded")
            return
        for (oam, bay_index), _ in self.filament_groups[group_name].bays.items():
            if oam.is_bay_loaded(bay_index):
                success, message = oam.load_spool(bay_index)
                if success:
                    gcmd.respond_info(message)
                    self.current_group = group_name
                else:
                    raise gcmd.respond_error(message)

def load_config_prefix(config):
    return OAMSManager(config)

def load_config(config):
    return OAMSManager(config)
