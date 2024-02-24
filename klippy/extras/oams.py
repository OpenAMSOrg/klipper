# OAMS
#
# Copyright (C) 2023-2024 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from .pulse_counter import FrequencyCounter
import logging

DEFAULT_FILAMENT_DIAMETER = 1.75
DEFAULT_CLICKS_PER_ROTATION = 4

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 10
REPORT_TIME = 0.050
RANGE_CHECK_COUNT = 4

PIN_MIN_TIME = 0.01  # anything less than 0.01 is not reliable
RESEND_HOST_TIME = 0.05 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 0.5

dc_motor_table_forward = None
dc_motor_table_backward = None

BLDC_FORWARD = 1
BLDC_REVERSE = 0
BLDC_PWM_VALUE = 0
BLDC_CYCLE_TIME = 0.00002


from statemachine import StateMachine, State


class OAMS(StateMachine):

    unloaded = State('Unloaded', initial=True)
    loading = State('Loading')
    loaded_fw = State('Loaded Forward')
    loaded_bw = State('Loaded Backward')
    unloading = State('Unloading')
    error = State('Error')
    
    resume_loaded = (
        loaded_fw.to(loaded_fw) |
        loaded_bw.to(loaded_fw) |
        unloaded.to(unloaded)
    )
    
    change_filament = (
        loaded_fw.to(loaded_bw) |
        loaded_bw.to(loaded_bw) |
        unloaded.to(unloaded)
    )
    
    determine_state_unloaded = (
        loaded_fw.to(unloaded) |
        loaded_bw.to(unloaded) |
        unloaded.to(unloaded)
    )

    determined_state_loaded = (
        unloaded.to(loaded_fw)
    )
    loading_end = (
        loading.to(loaded_fw)
    )
    unloading_end = (
        unloading.to(unloaded)
    )
    retract = (
        loaded_fw.to(loaded_bw) |
        loaded_bw.to(loaded_bw) |
        unloaded.to(unloaded) 
    )
    extrude = (
        loaded_bw.to(loaded_fw) |
        loaded_fw.to(loaded_fw) |
        unloaded.to(unloaded)
    )

    # gcode commands
    load_stop_unloaded = (
        loading.to(unloaded)
    )
    load_stop_loaded = (
         loading.to(loaded_fw)
    )
    load_spool = (
        unloaded.to(loading)
    )
    unload_spool = (
        loaded_fw.to(unloading) | loaded_bw.to(unloading)
    )

    # errors
    too_many_spools_loaded = (
        unloaded.to(error)
    )

    no_spool_error = (
        unloaded.to(error)
    )

    overload_error = (
        loading.to(error) | unloading.to(error) |
        loaded_fw.to(error) | loaded_bw.to(error)
    )
    
    def stats(self, eventtime):
        return (False, """
OAMS: state id: %s current spool: %s filament buffer adc: %s bldc state: %s fs motor state: %s fs 1 switch: %s fs 2 switch: %s fs 3 switch: %s fs 4 switch: %s hub 1 switch: %s hub 2 switch: %s hub 3 switch: %s hub 4 switch: %s

""" 
                % (self.current_state.id, 
                   self.current_spool, 
                   self.filament_pressure_sensor.last_value,
                   self.bldc_state,
                   self.f1_state,
                   self.f1s_switches[0].on,
                   self.f1s_switches[1].on,
                   self.f1s_switches[2].on,
                   self.f1s_switches[3].on,
                   self.hub_switches[0].on,
                   self.hub_switches[1].on,
                   self.hub_switches[2].on,
                   self.hub_switches[3].on,
                   ))
        
    def get_status(self, eventtime):
        return {'state': self.current_state.id,
                'current_spool': self.current_spool,
                'filament_pressure_sensor': self.filament_pressure_sensor.last_value,
                'bldc_state': self.bldc_state,
                'f1_state': self.f1_state,
                'f1_1_switch': self.f1s_switches[0].on,
                'f1_2_switch': self.f1s_switches[1].on,
                'f1_3_switch': self.f1s_switches[2].on,
                'f1_4_switch': self.f1s_switches[3].on,
                'hub_1_switch': self.hub_switches[0].on,
                'hub_2_switch': self.hub_switches[1].on,
                'hub_3_switch': self.hub_switches[2].on,
                'hub_4_switch': self.hub_switches[3].on,
                }
    
    def __init__(self, config) -> None:

        # State variables
        self.config = config
        self.current_spool = None
        self.state_trigger = None
        self.bldc_state = "STOP"
        self.f1_state = "STOP"
        
        self.tach = BLDCTachometer(config)

        self.printer = config.get_printer()
        
        self.board_revision = config.get('board_revision', '1.1')
        global dc_motor_table_forward, dc_motor_table_backward
        if self.board_revision == '1.1':
            dc_motor_table_forward = {0:0b00, 1:0b11, 2:0b01, 3:0b10}
            dc_motor_table_backward = {0:0b00, 1:0b10, 2:0b01, 3:0b11}
        elif self.board_revision == '1.4':
            dc_motor_table_forward = {0:0b00, 1:0b01, 2:0b10, 3:0b11}
            dc_motor_table_backward = {0:0b00, 1:0b01, 2:0b10, 3:0b11}
            
        self.load_slow_clicks = config.getint('load_slow_clicks', 900, minval=0)
        self.encoder = OAMSEncoder(self.printer, self.config, self._encoder_callback)
        hub_switch_pin_names = [x.strip() for x in config.get('hub_switch_pins').split(',')]
        
        self.last_rewind_time = None
        self.last_rewind_value = None
        self.bldc_rewind_speed = None
        def _current_sensor_callback(read_time, read_value):
            # TODO: add overload current check
            pass
            # if self.current_state.id == "unloading":
            #     k = 0.001
            #     self.bldc_rewind_speed += k
            #     logging.info("current_test\t%s\t%s\t%s" %(read_time, self.bldc_rewind_speed, read_value))
            
        self.current_sensor = Adc(self.printer, config,"oams:PC5", callback=_current_sensor_callback)
        
        # this is not reliable as of yet
        def _toolhead_filament_switch_callback(state):
            #logging.info("TOOLHEAD SWITCH: state %s value %s" % (self.current_state.id, self.filament_pressure_sensor.last_value))
            if self.current_state.id == 'loading':
               #and self.filament_pressure_sensor.last_value > self.schmitt_trigger_upper:
               self.bldc_stop()
               self.f1_stop()
               self.loading_end()
            pass
        
        self.toolhead_filament_switch = OAMSDigitalSwitch(config, config.get('toolhead_filament_switch'),_toolhead_filament_switch_callback)
        
        self.hub_switches = [
            AdcHesSwitch(self.printer, config,
                         x,idx,
                         config.get('hub_on_value_type'),
                         config.get('hub_on_value'),
                         callback=self.state_change_hub_switch_callback,
                         log_info=False) 
            for idx, x in enumerate(hub_switch_pin_names)]
        
        self.f1s_switch_pin_names = [x.strip() for x in config.get('f1s_switch_pins').split(',')]
        self.f1s_switches = [
            AdcHesSwitch(self.printer, config,
                         x,idx,
                         config.get('f1s_on_value_type'),
                         config.get('f1s_on_value'),
                         callback=self.state_change_f1s_switch_callback,
                         log_info=False) 
            for idx, x in enumerate(self.f1s_switch_pin_names)]
        
        led_pwm = config.getboolean('led_pwm', False)
        led_hardware_pwm = config.getboolean('led_hardware_pwm', False)
        led_cycle_time = config.getfloat('led_cycle_time', 0.100)
        self.led_start_value = config.getfloat('led_start_value', 0., minval=0., maxval=1.)

        self.led_white_pin_names = [x.strip() for x in config.get('led_white_pins').split(',')]
        self.led_white = [
            PrinterOutputPin(config, x, led_pwm, led_hardware_pwm, led_cycle_time)
            for idx, x in enumerate(self.led_white_pin_names)]
        self.led_red_pin_names = [x.strip() for x in config.get('led_red_pins').split(',')]
        self.led_red = [
            PrinterOutputPin(config, x, led_pwm, led_hardware_pwm, led_cycle_time)
            for idx, x in enumerate(self.led_red_pin_names)]
        
        # set up multi pin for first stage feeders
        m_driver_select_pin_name = config.get('dc_motor_driver_select_pin')
        m_select_pin_name = config.get('dc_motor_select_pin')
        #m_disable_pin_name = config.get('dc_motor_disable')
        m_pwm_a = config.get('dc_motor_pwm_a')
        m_pwm_b = config.get('dc_motor_pwm_b')
        m_is_pwm = config.getboolean('dc_motor_pwm', False)
        m_is_hardware_pwm = config.getboolean('dc_motor_hardware_pwm', False)
        m_cycle_time = config.getfloat('dc_motor_cycle_time', 0.100)

        self.m_driver_select_pin = PrinterOutputPin(config, m_driver_select_pin_name, False, False, False)
        self.m_select_pin = PrinterOutputPin(config, m_select_pin_name, False, False, False)
        #self.m_disable_pin = PrinterOutputPin(config, m_disable_pin_name, False, False, False)
        self.m_pwm_a_pin = PrinterOutputPin(config, m_pwm_a, m_is_pwm, m_is_hardware_pwm, m_cycle_time)
        self.m_pwm_b_pin = PrinterOutputPin(config, m_pwm_b, m_is_pwm, m_is_hardware_pwm, m_cycle_time)
        
        # set up bldc motor
        bldc_pwm_pin_name = config.get('bldc_pwm_pin')
        bldc_en_pin_name = config.get('bldc_en_pin')
        bldc_dir_pin_name = config.get('bldc_dir_pin')
        bldc_reset_pin_name = config.get('bldc_reset_pin')

        self.bldc_nreset_pin = PrinterOutputPin(config, bldc_reset_pin_name, False, False, False)
        self.bldc_en_pin = PrinterOutputPin(config, bldc_en_pin_name, False, False, False)  
        self.bldc_dir_pin = PrinterOutputPin(config, bldc_dir_pin_name, False, False, False)
        self.bldc_pwm_pin = PrinterOutputPin(config, bldc_pwm_pin_name, True, True, cycle_time=1/20000.0, shutdown_value=1.0)

        # set up filament pressure sensor
        self.filament_pressure_sensor = FilamentPressureSensor(config, callback=self.filament_pressure_sensor_callback)
        self.schmitt_trigger_upper = config.getfloat('pressure_sensor_bldc_schmitt_trigger_upper', 0.7)
        self.schmitt_trigger_lower = config.getfloat('pressure_sensor_bldc_schmitt_trigger_lower', 0.3)
        
        self.schmitt_trigger_reverse = config.getfloat('pressure_sensor_bldc_schmitt_trigger_reverse_lower', 0.2)
        
        if self.schmitt_trigger_upper < self.schmitt_trigger_lower:
            raise config.error("Schmitt trigger upper must be greater than lower")
        elif self.schmitt_trigger_upper == self.schmitt_trigger_lower:
            raise config.error("Schmitt trigger upper must be greater than lower")

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("LOAD_SPOOL", self.cmd_LOAD_SPOOL,
                                desc=self.cmd_LOAD_SPOOL_help)
        gcode.register_command("LOAD_STOP", self.cmd_LOAD_STOP,
                               desc=self.cmd_LOAD_STOP_help)
        gcode.register_command("UNLOAD_SPOOL", self.cmd_UNLOAD_SPOOL)
        gcode.register_command("LOAD_STATS", self.cmd_LOAD_STATS,)
        gcode.register_command("LOAD_CLICKS", self.cmd_LOAD_CLICKS,)
        gcode.register_command("OAMS_LOADED", self.cmd_OAMS_LOADED,)
        gcode.register_command("OAMS_CHANGE_FILAMENT", self.cmd_OAMS_CHANGE_FILAMENT)
        
        # self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # self.printer.register_event_handler("idle_timeout:ready", self.handle_ready)
        # self.printer.register_event_handler("idle_timeout:idle", self.handle_ready)
        
        self.printer.register_event_handler("g1:extrude", self.schmitt_trigger_forward)
        self.printer.register_event_handler("g1:retract", self.schmitt_trigger_backward)
        self.hard_stop = False
        
        self.printer.add_object('oams', self)
        
        self.bldc_run_reset = 0
        
        super().__init__()
        
    def cmd_OAMS_LOADED(self, gcmd):
        self.resume_loaded()
        gcmd.respond_info("OAMS: Loaded, state %s" % self.current_state.id)
        
    def cmd_OAMS_CHANGE_FILAMENT(self, gcmd):
        self.change_filament()
        gcmd.respond_info("OAMS: Change Filament, state %s" % self.current_state.id)
        
    def cmd_LOAD_CLICKS(self, gcmd):
        value = gcmd.get_int('SET', None, minval=0)
        if value is None:
            raise gcmd.error("Missing SET parameter")
        self.load_slow_clicks = value
        gcmd.respond_info("Load clicks set to %s" % self.load_slow_clicks)
        
        
    def _determine_state(self):
        # determine the state of the system LOADED or UNLOADED
        current_spool = None
        for idx in range(0, len(self.hub_switches)):
            if self.hub_switches[idx].on and self.f1s_switches[idx].on:
                if current_spool == None:
                    current_spool = idx
                else:
                    self.too_many_spools_loaded()
        if current_spool is not None:
            self.current_spool = current_spool
            self.determined_state_loaded()
        else:
            self.determine_state_unloaded()
    
    def schmitt_trigger_forward(self):
        #logging.info("g1:extrude")
        self.extrude()
        #logging.info("OAMS_Event: Extrude, state %s" % self.current_state.id)

            
    def schmitt_trigger_backward(self):
        #logging.info("g1:retract")
        self.retract()
        #logging.info("OAMS_Event: Retract, state %s" % self.current_state.id)

        
    def bldc_stop(self):
        if self.bldc_state == "STOP":
            return
        self.bldc_state = "STOP"
        self.bldc_en_pin.set_pin(0.0) # BRK on
        self.bldc_pwm_pin.set_pin(1.0, cycle_time=BLDC_CYCLE_TIME) # will stop the motor this is a reset effectively
        # self.bldc_pwm_pin.set_pin(0.0, cycle_time=BLDC_CYCLE_TIME)
        # self.bldc_pwm_pin.set_pin(1.0, cycle_time=BLDC_CYCLE_TIME)
        self.bldc_nreset_pin.set_pin(0.0)
        self.bldc_dir_pin.set_pin(BLDC_REVERSE)
        self.bldc_dir_pin.set_pin(BLDC_FORWARD)
        #self.bldc_en_pin.set_pin(1.0) # BRK off
        #self.bldc_en_pin.set_pin(0.0) # BRK on
        
    def bldc_coast(self):
        self.bldc_en_pin.set_pin(1.0) # BRK off (active low)
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + 0.101)
    
    
    # the callback is a function that runs when the motor
    # is confirmed to have turned on
    def bldc_run(self, value, forward=True, reset=False, callback=None):
        logging.info("bldc run %s" % value)
        pwm_value = 1.0 - value
        rpm = self.tach.get_rpm()
        logging.info("rpm: %s" % rpm) 
        if forward:
            self.bldc_dir_pin.set_pin(BLDC_FORWARD)
            self.bldc_state = "RUN_FORWARD"
        else:
            self.bldc_dir_pin.set_pin(BLDC_REVERSE)
            self.bldc_state = "RUN_BACKWARD"
        if reset:
            self.bldc_nreset_pin.set_pin(0.0)
            reactor = self.printer.get_reactor()
            
            task_timer = None
            def _future_task(eventtime):
                self.bldc_nreset_pin.set_pin(1.0)
                self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
                self.bldc_en_pin.set_pin(1.0) # BRK off
                inner_task_timer = None
                
                def __future_task(eventtime):
                    # here we wait another 0.1 then make sure the motor is running
                    # if the motor is not running do not unregister the task
                    if self.tach.get_rpm() > 0:
                        reactor.unregister_timer(task_timer)
                        if callback is not None:
                            callback(eventtime + 1)
                    else:
                        self.bldc_nreset_pin.set_pin(0.0)
                    reactor.unregister_timer(inner_task_timer)
                    return eventtime + 1
                inner_task_timer = reactor.register_timer(__future_task, reactor.monotonic() + 0.101)
                return eventtime + 1
            task_timer = reactor.register_timer(_future_task, reactor.monotonic() + 0.3)
            
            
            # # create a loop to reset and run the motor
            # def __future_task(eventtime):
            #     self.bldc_nreset_pin.set_pin(1.0)
            #     reactor.unregister_timer(task_timer)
            #     another_task_timer = None
            #     def ___future_task(eventtime):
            #         logging.info("setting the value to %s" % pwm_value)
            #         self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
            #         self.bldc_en_pin.set_pin(1.0)
            #         reactor.unregister_timer(another_task_timer)
            #         return eventtime + 1
            #     another_task_timer = reactor.register_timer(___future_task, reactor.monotonic() + 0.301)
            #     return eventtime + 1
            # task_timer = reactor.register_timer(__future_task, reactor.monotonic() + 0.301) # 0.301 second of a delay
        else:
            self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
            self.bldc_en_pin.set_pin(1.0)


    
    def f1_enable(self, spool_idx, forward=True):
        fn = None
        if forward:
            fn = dc_motor_table_forward
        else:
            fn = dc_motor_table_backward
        mux_val = fn[spool_idx]
        motor_driver_select_bit = mux_val & 0b01
        motor_select_bit = (mux_val & 0b10) >> 1
        
        self.m_driver_select_pin.set_pin(motor_driver_select_bit, time_ahead=0.100)
        self.m_select_pin.set_pin(motor_select_bit, time_ahead=0.100)
        # this is approximately the max_time for the mux to be set
        # at 3v3 the max rise time is 33ns or 0.000033ms hence 0.0001ms would be more than sufficient
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + .101)
        
        if forward:
            self.m_pwm_a_pin.set_pin(1.0)
            self.m_pwm_b_pin.set_pin(0.0)
            self.f1_state = "RUN_FORWARD"
        else:
            self.m_pwm_a_pin.set_pin(0.0)
            self.m_pwm_b_pin.set_pin(1.0)
            self.f1_state = "RUN_BACKWARD"
        
        #self.m_disable_pin.set_pin(0.0)
    
    def f1_stop(self):
        self.f1_state = "STOP"
        #self.m_disable_pin.set_pin(1.0)
        self.m_pwm_a_pin.set_pin(0.0)
        self.m_pwm_b_pin.set_pin(0.0)
    
    bldc_entry_pwm = 0.40
    bldc_slow_pwm = 0.50
    def _encoder_callback(self, clicks):
        clicks = abs(clicks)
        logging.info("number of clicks %s" % clicks)
        if self.current_state.id == 'loading' and clicks > self.load_slow_clicks + 50:
            logging.info("loading entry speed by encoder callback")
            self.bldc_run(self.bldc_entry_pwm, reset=False)
        elif self.current_state.id == 'loading' and clicks > self.load_slow_clicks:
            logging.info("loading slowdown by encoder callback")
            self.bldc_run(self.bldc_slow_pwm, reset=False)
    

    CHANGE_FILAMENT_TRIGGER_SPEED = 0.45
    PRINTING_TRIGGER_SPEED = 0.45

    def filament_pressure_sensor_callback(self, read_time, read_value):
        
        #logging.info("OAMS: Filament Pressure Sensor: %s, state: %s, u: %s, l: %s" % (read_value, self.current_state.id, self.schmitt_trigger_upper, self.schmitt_trigger_lower))
        # here we define the schmitt trigger for the bldc motor
        if self.current_state.id == 'loading' and read_value > self.schmitt_trigger_upper:
            logging.info("OAMS: loading end by filament pressure sensor")
            self.loading_end()
            self.bldc_stop()
            self.f1_stop()
        elif self.current_state.id == 'loaded_fw' and read_value < self.schmitt_trigger_lower and self.bldc_state == "STOP":
            #logging.info("running forward")
            self.bldc_run(self.PRINTING_TRIGGER_SPEED, forward=True, reset=True)
        elif self.current_state.id == 'loaded_fw' and read_value > self.schmitt_trigger_upper and self.bldc_state == "RUN_FORWARD":
            #logging.info("stopping")
            self.bldc_stop()
        elif self.current_state.id == 'loaded_bw' and read_value < self.schmitt_trigger_reverse and self.bldc_state == "RUN_BACKWARD":
            #logging.info("stopping")
            self.bldc_stop()
        elif self.current_state.id == 'loaded_bw' and read_value > self.schmitt_trigger_reverse and self.bldc_state == "STOP":
            logging.info("OAMS: running backward")
            # first reverse the f1 stage dc motor
            #self.f1_enable(self.current_spool, forward=False)
            #logging.info("OAMS: end of running backward")
            self.bldc_run(self.CHANGE_FILAMENT_TRIGGER_SPEED, forward=False, reset=True)
            #self.f1_stop()
            
    # TODO: implement a timeout while waiting and a way
    #       to handle errors within the AMS to set the status so they are recoverable
    def _wait_till_done(self, fn_done):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not fn_done():
            print_time = toolhead.get_last_move_time()
            eventtime = reactor.pause(eventtime + 1.0)
            
    cmd_LOAD_STATS_help = "Query load statistics"
    def cmd_LOAD_STATS(self, gcmd):
        gcmd.respond_info("Load statistics: %s %s %s" % (self.current_state.id, self.current_spool, self.hard_stop))

    cmd_LOAD_SPOOL_help = "Load a spool of filament"
    def cmd_LOAD_SPOOL(self, gcmd):
        ams_idx = gcmd.get_int('AMS', 0, minval=0, maxval=3)
        spool_idx = gcmd.get_int('SPOOL', 0, minval=0, maxval=3)

        if ams_idx is None  or spool_idx is None:
            raise gcmd.error("Missing AMS or SPOOL parameter")
        
        if self.current_state.id != 'unloaded':
            raise gcmd.error("Filament already loaded, please unload filament first")

        if not self.f1s_switches[spool_idx].on:
            raise gcmd.error("Spool %s is not inserted, please make sure the filament is inserted into the feeder" % spool_idx)
        
        self.current_spool = spool_idx
        
        self.load_spool()
        def _f1_enable_callback(eventtime):
            self.f1_enable(spool_idx, forward=True)
            return eventtime + 1
        self.bldc_run(1.0, reset=True, callback=_f1_enable_callback)
        self._wait_till_done(lambda: self.hard_stop or self.printer.is_shutdown() or self.current_state.id == 'loaded_fw')


    cmd_LOAD_STOP_help = "Stop a spool of filament"
    def cmd_LOAD_STOP(self, gcmd):
        self.hard_stop = True
        self.f1_stop()
        self.bldc_stop()
        self._determine_state()
        self.hard_stop = False


    cmd_UNLOAD_SPOOL_help = "Unload currently loaded spool"
    def cmd_UNLOAD_SPOOL(self, gcmd):
        self.retract()
        #spool_idx = 0

        # determine which spool is loaded
        spool_idx = None
        for idx, x in enumerate(self.hub_switches):
            if x.on:
                spool_idx = idx
                break
        if spool_idx is None:
            gcmd.respond_info("No spool is currently loaded")
            return
        
        #logging.info("Spool index: %s", spool_idx)
        self.current_spool = spool_idx
        
        # run bldc motor
        self.unload_spool()
        self.bldc_rewind_speed = 0.75
        self.encoder.reset_clicks()
        def _f1_enable_callback(eventtime):
            self.f1_enable(spool_idx, forward=False)
            return eventtime + 1
        self.bldc_run(0.5, forward=False, reset=True, callback=_f1_enable_callback)
        # reactor = self.printer.get_reactor()
        # reactor.pause(reactor.monotonic() + 0.101)
        self._wait_till_done(lambda: self.hard_stop  or self.printer.is_shutdown() or self.current_state.id == 'unloaded')
        
        # once done we want to momentarily forward the f1 motor to make sure the gearbox is in neutral
        self.f1_enable(spool_idx, forward=True)
        # schedule a timer to stop the motor 0.2s
        reactor = self.printer.get_reactor()
        turn_off_motor_timer = None
        self.f1_done = False
        def _turn_off_f1_motor(eventtime):
            logging.info("MOTOR CONTROL: stopping motor")
            self.f1_stop()
            reactor.unregister_timer(turn_off_motor_timer)
            self.f1_done = True
            return eventtime + 1
        turn_off_motor_timer = reactor.register_timer(_turn_off_f1_motor, reactor.monotonic() + 0.1) # 0.1 second of a delay
        # have to wait
        self._wait_till_done(lambda: self.hard_stop  or self.printer.is_shutdown() or self.f1_done)
        


    def state_change_hub_switch_callback(self, idx, on):
        if self.current_state.id == 'loading' and idx == self.current_spool and on:
            # we must schedule this sometime in the future so the filament reaches the bldc motor
            reactor = self.printer.get_reactor()
            turn_off_motor_timer = None
            def _turn_off_f1_motor(eventtime):
                logging.info("MOTOR CONTROL: stopping motor")
                self.f1_stop()
                reactor.unregister_timer(turn_off_motor_timer)
                return eventtime + 1
            turn_off_motor_timer = reactor.register_timer(_turn_off_f1_motor, reactor.monotonic() + 2.0) # 2 second of a delay
            self.encoder.reset_clicks()
        if self.current_state.id == 'unloading' and idx == self.current_spool and not on:
            # schedule a timer to stop the motor 0.2s
            reactor = self.printer.get_reactor()
            task_timer = None
            def __future_task(eventtime):
                self.unloading_end()
                self.f1_stop()
                self.bldc_stop()
                self.last_rewind_time = None
                self.last_rewind_value = None
                self.bldc_rewind_speed = None
                reactor.unregister_timer(task_timer)
                return eventtime + 1
            task_timer = reactor.register_timer(__future_task, reactor.monotonic() + 0.2) # 0.2 second of a delay
            
        if self.current_state.id == 'unloaded':
            self._determine_state()
    
    def state_change_f1s_switch_callback(self, idx, on):
        self.led_white[idx].set_pin(on)
        if self.current_state.id == 'unloaded':
            self._determine_state()

class  OAMSDigitalSwitch:
    def __init__(self, config, switch_pin, callback):
        printer = config.get_printer()
        buttons = printer.load_object(config, 'buttons')
        buttons.register_buttons([switch_pin], self._button_handler)
        self.callback = callback
        self.last_state = None
    def _button_handler(self, eventtime, state):
        self.last_state = state
        self.callback(state)

class PrinterOutputPin:
    def __init__(self, config, pin_name, is_pwm, hardware_pwm, cycle_time=None, shutdown_value=None):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.is_pwm = is_pwm
        self.cycle_time = cycle_time
        if self.is_pwm:
            self.mcu_pin = ppins.setup_pin('pwm', pin_name)
            self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
            self.scale = config.getfloat('scale', 1., above=0.)
            self.last_cycle_time = self.default_cycle_time = cycle_time
        else:
            self.mcu_pin = ppins.setup_pin('digital_out', pin_name)
            self.scale = 1.
            self.last_cycle_time = self.default_cycle_time = 0.
        self.last_print_time = 0.
        self.reactor = self.printer.get_reactor()
        self.resend_timer = None
        self.resend_interval = 0.
        max_mcu_duration = config.getfloat('maximum_mcu_duration', 0.,
                                            minval=0.100,
                                            maxval=MAX_SCHEDULE_TIME)
        self.mcu_pin.setup_max_duration(max_mcu_duration)
        if max_mcu_duration:
            self.resend_interval = max_mcu_duration - RESEND_HOST_TIME

        self.last_value = config.getfloat(
            'value', 0., minval=0., maxval=self.scale) / self.scale
        
        if shutdown_value == None:
            self.shutdown_value = 0.
        else:
            self.shutdown_value = shutdown_value
        
        self.mcu_pin.setup_start_value(self.last_value, self.shutdown_value)

    def get_status(self, eventtime):
        return {'value': self.last_value}
    def _set_pin(self, print_time, value, cycle_time, is_resend=False):
        #logging.info("PWM: %s", self.is_pwm)
        # if value == self.last_value and cycle_time == self.last_cycle_time:
        #     if not is_resend:
        #         return
        print_time = max(print_time, self.last_print_time + PIN_MIN_TIME)
        
        if self.is_pwm:
            #logging.info("got here pwm with values %s %s %s", print_time, value, cycle_time)
            self.mcu_pin.set_pwm(print_time, value) #, cycle_time)
        else:
            self.mcu_pin.set_digital(print_time, value)
        self.last_value = value
        self.last_cycle_time = cycle_time
        self.last_print_time = print_time
        if self.resend_interval and self.resend_timer is None:
            self.resend_timer = self.reactor.register_timer(
                self._resend_current_val, self.reactor.NOW)
        
    def set_pin(self, value, cycle_time = None, time_ahead = 0.100):
        if cycle_time is None:
            cycle_time = self.default_cycle_time
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu_pin.get_mcu().estimated_print_time(curtime)
        print_time = est_print_time + time_ahead
        self._set_pin(print_time, value, cycle_time)
        return print_time

    def _resend_current_val(self, eventtime):
        if self.last_value == self.shutdown_value:
            self.reactor.unregister_timer(self.resend_timer)
            self.resend_timer = None
            return self.reactor.NEVER

        systime = self.reactor.monotonic()
        print_time = self.mcu_pin.get_mcu().estimated_print_time(systime)
        time_diff = (self.last_print_time + self.resend_interval) - print_time
        if time_diff > 0.:
            # Reschedule for resend time
            return systime + time_diff
        self._set_pin(print_time + PIN_MIN_TIME,
                      self.last_value, self.last_cycle_time, True)
        return systime + self.resend_interval

class Adc:
    def __init__(self, printer, config, sensor_pin, callback = None):
        self.callback = callback
        ppins = printer.lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', sensor_pin)
        self.mcu_adc.setup_adc_callback(REPORT_TIME, self.adc_callback)
        query_adc = config.get_printer().load_object(config, 'query_adc')
        self.hes_switch_name = config.get_name()
        query_adc.register_adc(config.get_name(), self.mcu_adc)
        self.setup_minmax(0, 1)
        
    def get_report_time_delta(self):
        return REPORT_TIME
    
    def adc_callback(self, read_time, read_value):
        if self.callback != None:
            self.callback(read_time, read_value)
        
    def setup_minmax(self, min_val, max_val):
        self.mcu_adc.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT,
                                  minval=min_val, maxval=max_val,
                                  range_check_count=RANGE_CHECK_COUNT)


class AdcHesSwitch:
    def __init__(self, printer, config, sensor_pin, idx, on_value_type, on_value, callback = None, log_info=False):
        self.log_info = log_info
        self.idx = idx
        self.callback = callback
        ppins = printer.lookup_object('pins')
        self.mcu_adc = ppins.setup_pin('adc', sensor_pin)
        self.on_value_type = on_value_type
        self.on_value = float(on_value)
        self.mcu_adc.setup_adc_callback(REPORT_TIME, self.adc_callback)
        query_adc = config.get_printer().load_object(config, 'query_adc')
        self.hes_switch_name = config.get_name()
        query_adc.register_adc(config.get_name(), self.mcu_adc)
        self.setup_minmax(0, 1)
        self.on = False

    def get_report_time_delta(self):
        return REPORT_TIME
    def adc_callback(self, read_time, read_value):
        on = False
        if self.on_value_type == 'above':            
            if read_value > self.on_value:
                on = True
        elif self.on_value_type == 'below':
            if read_value < self.on_value:
                on = True
        if self.on != on:
            self.on = on
            # change of states must call the callback
            if self.callback is not None:
                self.callback(self.idx, on)

        if self.log_info:
            logging.info("ADC HES Switch (%s): %s (%s)", self.hes_switch_name , on, read_value)
        #logging.info("ADC HES Switch (%s): %s", self.hes_switch_name , on)
        #self.temperature_callback(read_time + SAMPLE_COUNT * SAMPLE_TIME, temp)
    def setup_minmax(self, min_val, max_val):
        self.mcu_adc.setup_minmax(SAMPLE_TIME, SAMPLE_COUNT,
                                  minval=min_val, maxval=max_val,
                                  range_check_count=RANGE_CHECK_COUNT)
        

class OAMSEncoder:
    def __init__(self, printer, config, callback=None):
        self.callback = callback
        self.cps = 0 # clicks per second
        self.clicks = 0
        self.filament_diameter = config.getfloat('filament_diameter', DEFAULT_FILAMENT_DIAMETER, above=0.)
        self.reactor = printer.get_reactor()
        buttons = printer.load_object(config, "buttons")
        # Register rotary encoder
        encoder_pins = config.get('encoder_pins', None)
        encoder_steps_per_detent = config.getchoice('encoder_steps_per_detent',
                                                    {2: 2, 4: 4}, 4)
        if encoder_pins is not None:
            try:
                pin1, pin2 = encoder_pins.split(',')
            except:
                raise config.error("Unable to parse encoder_pins")
            buttons.register_rotary_encoder(pin1.strip(), pin2.strip(),
                                            self.encoder_cw_callback,
                                            self.encoder_ccw_callback,
                                            encoder_steps_per_detent)
        self.last_encoder_cw_eventtime = 0
        self.last_encoder_ccw_eventtime = 0

        # register commands
        self.gcode = printer.lookup_object('gcode')
        self.gcode.register_command('OAMS_QUERY_CLICKS', self.cmd_QUERY_CLICKS,
                                    desc=self.cmd_QUERY_CLICKS_help)

    # Rotary encoder callbacks
    def encoder_cw_callback(self, eventtime):
        self.clicks += 1
        self.last_encoder_cw_eventtime = eventtime
        if self.callback is not None:
            self.callback(self.clicks)

    def encoder_ccw_callback(self, eventtime):
        self.clicks -= 1
        self.last_encoder_ccw_eventtime = eventtime
        if self.callback is not None:
            self.callback(self.clicks)

    def get_clicks(self):
        return self.clicks
    
    def reset_clicks(self):
        self.clicks = 0
    
    cmd_QUERY_CLICKS_help = "Query number of clicks since last restart"
    def cmd_QUERY_CLICKS(self, gcmd):
        self.gcode.respond_info(
            "Current number of clicks since last restart: %i" % self.clicks)
    

class BLDCTachometer:
    def __init__(self, config):
        printer = config.get_printer()
        self._freq_counter = None

        pin = config.get('bldc_tach_pin', None)
        if pin is not None:
            # pulses per revolution
            self.ppr = config.getint('tachometer_ppr', 2, minval=1)
            poll_time = config.getfloat('tachometer_poll_interval',
                                        0.0015, above=0.)
            sample_time = 1.
            self._freq_counter = FrequencyCounter(
                printer, pin, sample_time, poll_time)
            
    def get_rpm(self):
        rpm = None
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30. / self.ppr
        return rpm

    def get_status(self, eventtime):
        if self._freq_counter is not None:
            rpm = self._freq_counter.get_frequency() * 30. / self.ppr
        else:
            rpm = None
        return {'rpm': rpm}
    
class FilamentPressureSensor:
    def __init__(self, config, callback=None):
        self.callback = callback
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self._pin = config.get('pressure_sensor_pin')
        self._sample_count = config.getint('pressure_sensor_sample_count', 5)
        self._sample_time = config.getfloat('pressure_sensor_sample_time', 0.005)
        self._report_time = config.getfloat('pressure_sensor_report_time', 0.050)

        # printer objects
        self.ppins = self.adc = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.ppins = self.printer.lookup_object('pins')
        self.adc = self.ppins.setup_pin('adc', self._pin)
        self.adc.setup_minmax(self._sample_time, self._sample_count)
        self.adc.setup_adc_callback(self._report_time, self._adc_callback)
        
        self.last_value = 0.0

        #self.step_pin = self.printer.lookup_object('output_pin step_pin')

        self.filament_sensor_timer = self.reactor.register_timer(
            self.filament_sensor_timer_cb)


    # Initialization
    def handle_ready(self):
        self.reactor.update_timer(self.filament_sensor_timer,
                                  self.reactor.NOW)

    def filament_sensor_timer_cb(self, eventtime):
        return eventtime + 1

    def _adc_callback(self, read_time, read_value):
        self.last_value = read_value
        if self.callback is not None:
            self.callback(read_time, read_value)

MIN_DERIV_TIME = 0.1
class ControlPID:
    def __init__(self, Kp, Kd, Ki, sp):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.sp = sp

        self.min_deriv_time = MIN_DERIV_TIME
        self.integ_max = 1.0
        
        self.prev_value = 0.0
        self.prev_time = 0.0
        self.prev_deriv = 0.0
        self.prev_integ = 0.0

    def update_pid(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def current_update(self, read_time, value):
        time_diff = read_time - self.prev_time
        # Calculate change of pressure
        diff = value - self.prev_value
        
        deriv = None
        if time_diff >= self.min_deriv_time:
            deriv = diff / time_diff
        else:
            deriv = (self.prev_deriv * (self.min_deriv_time-time_diff)
                          + diff) / self.min_deriv_time
            
        # Calculate accumulated pressure "error"
        err = self.sp - value
        integ = self.prev_integ + err * time_diff
        integ = max(-1.0, min(self.integ_max, integ))

        # Calculate output
        co = self.Kp*err + self.Ki*integ - self.Kd*deriv
        
        #logging.info("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%f",
        #    pressure, read_time, pressure_diff, pressure_deriv, pressure_err, pressure_integ, co)

        bounded_co = max(-1.0, min(1, co))

        # Store state for next measurement
        self.prev_value = value
        self.prev_time = read_time
        self.prev_deriv = deriv
        if co == bounded_co:
            self.prev_integ = integ

        return bounded_co

def load_config_prefix(config):
    return OAMS(config)


# from statemachine.contrib.diagram import DotGraphMachine
# graph = DotGraphMachine(OAMS)
# dot = graph()
# dot.write_png("/home/pi/printer_data/config/m.png")
