# OAMS
#
# Copyright (C) 2023-2024 JR Lomas (discord:knight_rad.iant) <lomas.jr@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# TODO: add error handling, pausing and resuming of the OAMS
# TODO: read current sensor and implement overload error
# TODO: implement change over of spool in filament runout
# TODO: tune and determine the sample count time for encoder

# from gevent import monkey
# monkey.patch_all()

import asyncio

import sqlite3
import time
import threading

def create_db(db_name):
    """Connect to database and create schema, if it doesn't already exist, for storing calibration data."""
    conn = sqlite3.connect(db_name)
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS "hes_internal_hub" (
	"id"	INTEGER NOT NULL,
	"unix_timestamp"	NUMERIC NOT NULL,
	"spool_idx"	INTEGER NOT NULL,
	"adc_value"	REAL NOT NULL,
	"is_on"	INTEGER NOT NULL DEFAULT 0,
	PRIMARY KEY("id" AUTOINCREMENT)
)''')
    c.execute('''CREATE INDEX IF NOT EXISTS "hes_internal_hub_unix_timestamp_spool_idx" ON "hes_internal_hub" (
	"unix_timestamp"	DESC,
	"spool_idx"
)''')
    c.execute('''CREATE TABLE IF NOT EXISTS "spool_status" (
	"spool_idx"	INTEGER NOT NULL,
	"unix_timestamp"	NUMERIC NOT NULL,
	"start_percentage"	REAL NOT NULL DEFAULT 100.0,
	"clicks"	INTEGER NOT NULL DEFAULT 0,
    "type" TEXT NOT NULL DEFAULT "ABS",
	PRIMARY KEY("spool_idx")
)''')
    conn.commit()
    c.close()
    conn.close()




# def get_historical_offset(config):
#     """Get the historical offset from the database."""
#     conn = sqlite3.connect(config.get('database', 'database.db'))
#     c = conn.cursor()
#     res = c.execute('''SELECT MAX(nozzle_run_id) as last_nozzle_run_id FROM nozzle_run''',)
#     nozzle_run_id = res.fetchone()[0]
#     # select number of rows
#     res = c.execute('''SELECT COUNT(*) as count FROM calibration WHERE nozzle_run_id = ?''', (nozzle_run_id,))
#     count = res.fetchone()[0]
#     if count < 10:
#         return None, None
#     samples = 10
#     res  = c.execute('''SELECT offset FROM calibration ORDER BY nozzle_run_id DESC LIMIT ?''',(samples,))
#     offsets = res.fetchall()
#     avg_offset = 0
#     proper_offsets = []
#     for offset in offsets:
#         avg_offset += offset[0]/samples
#         proper_offsets.append(offset[0])
#     stdev_offset = statistics.stdev(proper_offsets)
#     offset_3_sigma = 3*stdev_offset
#     c.close()
#     conn.close()
#     return avg_offset, offset_3_sigma


from statemachine import StateMachine, State
from .pulse_counter import FrequencyCounter
import logging
from enum import Enum

DEFAULT_FILAMENT_DIAMETER = 1.75
DEFAULT_CLICKS_PER_ROTATION = 4

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 10
REPORT_TIME = 0.050
RANGE_CHECK_COUNT = 4

PIN_MIN_TIME = 0.01  # anything less than 0.01 is not reliable
RESEND_HOST_TIME = 0.05 + PIN_MIN_TIME
MAX_SCHEDULE_TIME = 0.5

BLDC_FORWARD = 1
BLDC_REVERSE = 0
BLDC_PWM_VALUE = 0
BLDC_CYCLE_TIME = 0.00002
    
MIN_RESET_COUNT = 5

class BLDCCommandQueue(StateMachine):

    stopped = State('Stopped', initial=True)
    running_forward = State('Forward')
    running_backward = State('Backward')
    coasting = State('Coasting')
    overloaded = State('Overloaded')
    
    cs_stop = (
        running_forward.to(stopped) |
        running_backward.to(stopped) |
        coasting.to(stopped) |
        stopped.to(stopped) |
        overloaded.to(overloaded)
    )
    
    cs_run_forward = (
        running_forward.to(running_forward) |
        stopped.to(running_forward) |
        coasting.to(running_forward)
    )
    
    cs_run_backward = (
        running_backward.to(running_backward) |
        stopped.to(running_backward) |
        coasting.to(running_backward)
    )
    
    cs_overload = (
        running_forward.to(overloaded) |
        running_backward.to(overloaded) |
        overloaded.to(overloaded)
    )
    
    cs_coast = (
        running_forward.to(coasting) |
        running_backward.to(coasting) |
        stopped.to(coasting) |
        coasting.to(coasting)
    )
    
    class BLDCCommand:
        def __init__(self, motor_action, callback):
            self.motor_action = motor_action
            self.callback = callback
    
    def __init__(self, config, tach) -> None:
        # set up bldc motor
        bldc_pwm_pin_name = config.get('bldc_pwm_pin')
        bldc_en_pin_name = config.get('bldc_en_pin')
        bldc_dir_pin_name = config.get('bldc_dir_pin')
        bldc_reset_pin_name = config.get('bldc_reset_pin')

        self.bldc_nreset_pin = PrinterOutputPin(config, bldc_reset_pin_name, False, False, False)
        self.bldc_en_pin = PrinterOutputPin(config, bldc_en_pin_name, False, False, False)  
        self.bldc_dir_pin = PrinterOutputPin(config, bldc_dir_pin_name, False, False, False)
        self.bldc_pwm_pin = PrinterOutputPin(config, bldc_pwm_pin_name, True, True, cycle_time=BLDC_CYCLE_TIME, shutdown_value=1.0)

        self.tach = tach
        
        self.queue = []
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.current_command = None
        self.bldc_reset_count = 0
        self.pwm_value = 0.0
        self.locked = False
        
        super().__init__()
        
    def lock(self):
        self.locked = True
        
    def unlock(self):
        self.locked = False
        
    def empty(self):
        return len(self.queue) == 0
    
    def purge(self):
        self.queue = []
        
    def enqueue(self, motor_action, callback = None):
        if self.locked:
            return
        command = self.BLDCCommand(motor_action, callback)
        self.queue.append(command)
        
    def command_task(self, eventtime):
        while len(self.queue) > 0:
            self.current_command = self.queue.pop(0)
            self.current_command.motor_action()
            if self.current_command.callback is not None:
                self.current_command.callback()
        return eventtime + 0.01
                
    def purge_queue(self):
        self.queue = []
                
    def reset_motor(self):
        self.bldc_nreset_pin.set_pin(0.0)
        self.reactor.pause(self.reactor.monotonic() + .101)
        self.bldc_nreset_pin.set_pin(1.0)
        self.reactor.pause(self.reactor.monotonic() + .101)
        self.bldc_reset_count = 0
        
    def replay_action(self):
        if self.current_state.id == 'running_forward':
            self.run_forward(self.pwm_value, nowait=True)
        elif self.current_state.id == 'running_backward':
            self.run_backward(self.pwm_value, nowait=True)
        elif self.current_state.id == 'coasting':
            self.coast(nowait=True)
        elif self.current_state.id == 'stopped':
            self.stop(nowait=True)
    
    def run_forward(self, pwm_value, nowait = False):
        self.pwm_value = pwm_value
        pwm_value = 1 - pwm_value
        if self.current_state.id == 'running_forward' and self.tach.get_rpm() > 0:
            logging.debug("OAMS: (RUNNING_FORWARD) motor is already running and tach reads %s" % self.tach.get_rpm())
            self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
            self.bldc_reset_count = 0
            return
        elif self.current_state.id == 'running_forward' and self.tach.get_rpm() == 0:
            logging.debug("OAMS: (RUNNING_FORWARD) motor is already running and tach reads 0 reset count %d", self.bldc_reset_count)
            self.bldc_reset_count += 1
            if self.bldc_reset_count >= MIN_RESET_COUNT:
                # the motor is not really running, we must stop it first
                logging.debug("OAMS: (RUNNING_FORWARD) motor is stalled")
                self.stop()
                self.reset_motor()
            else:
                return
        elif self.current_state.id == 'running_backward':
            # we must stop the motor first
            self.stop()
        self.cs_run_forward()
        logging.debug("OAMS: (RUNNING FORWARD) attempting to run forward")
        self.bldc_dir_pin.set_pin(BLDC_FORWARD)
        self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
        self.bldc_en_pin.set_pin(1.0)
        if not nowait:
            self.reactor.pause(self.reactor.monotonic() + .101)
    
    def run_backward(self, pwm_value, nowait = False):
        self.pwm_value = pwm_value
        pwm_value = 1 - pwm_value
        if self.current_state.id == 'running_backward' and self.tach.get_rpm() > 0:
            self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
            self.bldc_reset_count = 0
            return
        elif self.current_state.id == 'running_backward' and self.tach.get_rpm() == 0:
            self.bldc_reset_count += 1
            if self.bldc_reset_count >= MIN_RESET_COUNT:
                # the motor is not really running, we must stop it first
                self.stop()
                self.reset_motor()
            else:
                return
        elif self.current_state.id == 'running_forward':
            self.stop()
        self.cs_run_backward()
        self.bldc_dir_pin.set_pin(BLDC_REVERSE)
        self.bldc_pwm_pin.set_pin(pwm_value, cycle_time=BLDC_CYCLE_TIME)
        self.bldc_en_pin.set_pin(1.0)
        if not nowait:
            self.reactor.pause(self.reactor.monotonic() + .101)
    
    def stop(self, nowait = False):
        if self.current_state.id == 'stopped':
            return
        self.cs_stop()
        self.bldc_en_pin.set_pin(0.0)
        self.bldc_reset_count = 0
        if not nowait:
            self.reactor.pause(self.reactor.monotonic() + .101)
    
    def coast(self, nowait = False):
        if self.current_state.id == 'coasting':
            return
        self.cs_coast()
        self.bldc_en_pin.set_pin(1.0)
        self.bldc_pwm_pin.set_pin(1.0, cycle_time=BLDC_CYCLE_TIME)
        if not nowait:
            self.reactor.pause(self.reactor.monotonic() + .101)


class MonitorCondition():
    def __init__(self,
                 condition_fn, 
                 callback_fn,
                 start_in_time = 0.0, # given in seconds
                 period = None, 
                 end_at_time = None):
        self.start_in_time = start_in_time
        self.period = period
        self.end_at_time = end_at_time
        self.condition_fn = condition_fn
        self.callback_fn = callback_fn
        self.timer_task = None
        self.isCanceled = False
        
    def cancel(self):
        self.isCanceled = True
        self.reactor.unregister_timer(self.timer_task)
        
    def start(self):
        def _task(eventtime):
            if not self.isCanceled and self.condition_fn():
                self.callback_fn()
            if self.period is None or self.isCanceled:
                self.reactor.unregister_timer(self.timer_task)
            return eventtime + 1
        self.timer_task = self.reactor.register_time(self.reactor.monotonic() + self.start_in_time, _task)
            

class OAMSMonitor():
    
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.conditions = []
        
    def add_monitor_condition(self, condition):
        self.conditions.append(condition)
        
    def add_monitor_conditions(self, *conditions):
        for condition in conditions:
            self.add_monitor_condition(condition)
        
    def start(self):
        for condition in self.conditions:
            condition.start()

class OAMSSpool():
    def __init__(self) -> None:
        self.start_percentage = 100.0
        self.type = "ABS"
        self.unload_lower_range = 0.40 #0.45467032967033 # lowest speed for BLDC motor
        self.unload_m = 0.17 #0.1835164835
        self.unload_upper_range = self.unload_lower_range + self.unload_m
        
    def get_percentage(self):
        spool_length = None
        if self.type == "ABS":
            spool_length = 380 # given in meters
        elif self.type == "PLA":
            spool_length = 330 # given in meters
        
        if spool_length is not None:
            return max(self.start_percentage - (2*self.clicks / (330 * 1000))*100,0) # 2 clicks per mm of filament, 330 meters of filament in one spool
        else:
            return None
        
    def get_unload_delay(self):
        upper_range = 0.5
        lower_range = 0.2
        percentage = self.get_percentage()
        if percentage is None:
            return (lower_range + upper_range) / 2
        return lower_range + (lower_range - upper_range) * (1 - (percentage / 100))
        
    def set_clicks(self, clicks):
        self.clicks = clicks
        
    def get_unload_speed(self):
        percentage = self.get_percentage()
        if percentage is None:
            logging.info("OAMS: Unknown material, please set the material type, rewind speed on the f1s motor might not be correct")
            return (self.unload_lower_range + self.unload_upper_range) / 2
        speed = self.unload_lower_range + self.unload_m * (percentage / 100)
        return speed


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
OAMS: state id: %s current spool: %s filament buffer adc: %s bldc state: %s fs motor state: %s fs 1 switch: %s fs 2 switch: %s fs 3 switch: %s fs 4 switch: %s hub 1 switch: %s hub 2 switch: %s hub 3 switch: %s hub 4 switch: %s fast unload: %s current sensor: %.3f follower enabled: %s

""" 
                % (self.current_state.id, 
                   self.current_spool, 
                   self.filament_pressure_sensor.last_value,
                   self.bldc_cmd_queue.current_state.id,
                   self.f1_state,
                   "%s (%.3f)" % (self.f1s_switches[0].on, self.f1s_switches[0].adc_value),
                   "%s (%.3f)" % (self.f1s_switches[1].on, self.f1s_switches[1].adc_value),
                   "%s (%.3f)" % (self.f1s_switches[2].on, self.f1s_switches[2].adc_value),
                   "%s (%.3f)" % (self.f1s_switches[3].on, self.f1s_switches[3].adc_value),
                   "%s (%.3f)" % (self.hub_switches[0].on, self.hub_switches[0].adc_value),
                   "%s (%.3f)" % (self.hub_switches[1].on, self.hub_switches[1].adc_value),
                   "%s (%.3f)" % (self.hub_switches[2].on, self.hub_switches[2].adc_value),
                   "%s (%.3f)" % (self.hub_switches[3].on, self.hub_switches[3].adc_value),
                   self.fast_unload,
                   self.current_sensor_value,
                   self.follower_enable
                   ))
        
    def get_status(self, eventtime):
        return {'state': self.current_state.id,
                'current_spool': self.current_spool,
                'filament_pressure_sensor': self.filament_pressure_sensor.last_value,
                'bldc_state': self.bldc_cmd_queue.current_state.id,
                'f1_state': self.f1_state,
                'f1_1_switch': "%s (%f.3)" % (self.f1s_switches[0].on, self.f1s_switches[0].adc_value),
                'f1_2_switch': "%s (%f.3)" % (self.f1s_switches[1].on, self.f1s_switches[1].adc_value),
                'f1_3_switch': "%s (%f.3)" % (self.f1s_switches[2].on, self.f1s_switches[2].adc_value),
                'f1_4_switch': "%s (%f.3)" % (self.f1s_switches[3].on, self.f1s_switches[3].adc_value),
                'hub_1_switch': "%s (%f.3)" % (self.hub_switches[0].on, self.hub_switches[0].adc_value),
                'hub_2_switch': "%s (%f.3)" % (self.hub_switches[1].on, self.hub_switches[1].adc_value),
                'hub_3_switch': "%s (%f.3)" % (self.hub_switches[2].on, self.hub_switches[2].adc_value),
                'hub_4_switch': "%s (%f.3)" % (self.hub_switches[3].on, self.hub_switches[3].adc_value),
                'fast_unload': self.fast_unload,
                'current_sensor' : self.current_sensor_value,
                'follower_enabled' : self.follower_enable
                }
    
    def __init__(self, config) -> None:

        # State variables
        self.config = config
        self.current_spool = None
        self.state_trigger = None
        self.f1_state = "STOP"
        self.error_state = None
        self.calibrating = False
        self.fast_unload = config.getboolean('fast_unload', False)
        
        # create database if not exists
        self.database_name = config.get('oams_database_name', "oams.db")
        create_db(self.database_name)
        
        self.tach = BLDCTachometer(config)

        self.printer = config.get_printer()
        
        self.board_revision = config.get('board_revision', '1.1')
        
        # revision 1.1 contain erros in the wiring of muxes, hence
        # this table is used to reroute the signals
        if self.board_revision == '1.1':
            self.dc_motor_table_forward = {0:0b00, 1:0b11, 2:0b01, 3:0b10}
            self.dc_motor_table_backward = {0:0b00, 1:0b10, 2:0b01, 3:0b11}
        # revision 1.4 has the muxes wired correctly, hence the binary order on the table
        elif self.board_revision == '1.4':
            self.dc_motor_table_forward = {0:0b00, 1:0b01, 2:0b10, 3:0b11}
            self.dc_motor_table_backward = {0:0b00, 1:0b01, 2:0b10, 3:0b11}
            
        self.load_slow_clicks = config.getint('load_slow_clicks', 900, minval=0)
        self.encoder = OAMSEncoder(self.printer, self.config, None)
                
        hub_switch_pin_names = [x.strip() for x in config.get('hub_switch_pins').split(',')]

        self.rewinding = False
        self.current_sensor_values = []
        self.current_sensor_value = 0.0
        def _current_sensor_callback(read_time, read_value):
            if self.rewinding:
                self.current_sensor_values.append(read_value)
            self.current_sensor_value = read_value
        self.current_sensor_target_value = config.getfloat('current_sensor_target_value', 0.30)
        self.current_sensor = Adc(self.printer, config,"oams:PC5", callback=_current_sensor_callback)
        self.BLDC_PID = ControlPID(-4, 0, -0.5, self.current_sensor_target_value)
        
        # # There is no use for the toolhead filament switch at the moment
        # def _toolhead_filament_switch_callback(state):
        #     logging.info("OAMS: toolhead switch state change to %s value %s" % (self.current_state.id, self.filament_pressure_sensor.last_value))
        #     pass
        
        # self.toolhead_filament_switch = OAMSDigitalSwitch(config, config.get('toolhead_filament_switch'),_toolhead_filament_switch_callback)
        
        self.hub_switches = [
            AdcHesSwitch(self.printer, config,
                         x,idx,
                         config.get('hub_on_value_type'),
                         float(config.get('hub_on_value').split(",")[idx]),
                         callback=None,
                         log_info=False,
                         db_name=self.database_name) 
            for idx, x in enumerate(hub_switch_pin_names)]
        
        self.f1s_switch_pin_names = [x.strip() for x in config.get('f1s_switch_pins').split(',')]
        self.f1s_switches = [
            AdcHesSwitch(self.printer, config,
                         x,idx,
                         config.get('f1s_on_value_type'),
                         config.get('f1s_on_value'),
                         callback=self.state_change_f1s_switch_callback,
                         log_info=False,
                         db_name = self.database_name) 
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
        
        self.reverse_dc_motor_on_unload = config.getboolean('reverse_dc_motor_on_unload', False)

        self.m_driver_select_pin = PrinterOutputPin(config, m_driver_select_pin_name, False, False, False)
        self.m_select_pin = PrinterOutputPin(config, m_select_pin_name, False, False, False)
        #self.m_disable_pin = PrinterOutputPin(config, m_disable_pin_name, False, False, False)
        self.m_pwm_a_pin = PrinterOutputPin(config, m_pwm_a, m_is_pwm, m_is_hardware_pwm, m_cycle_time)
        self.m_pwm_b_pin = PrinterOutputPin(config, m_pwm_b, m_is_pwm, m_is_hardware_pwm, m_cycle_time)
        
        self.bldc_cmd_queue = BLDCCommandQueue(config, self.tach)

        # set up filament pressure sensor
        self.reverse_adc_value = config.getboolean('reverse_adc_value', False)
        self.filament_pressure_sensor = FilamentPressureSensor(config, callback=self.filament_pressure_sensor_callback, reverse=self.reverse_adc_value)
        self.schmitt_trigger_upper = config.getfloat('pressure_sensor_bldc_schmitt_trigger_upper', 0.7)
        self.schmitt_trigger_lower = config.getfloat('pressure_sensor_bldc_schmitt_trigger_lower', 0.3)
        
        # during the load_bw (unloading of the slide) this is the trigger point at which the
        # OAMS will stop the BLDC motor from running, the value does nto have to be the same as the
        # schmitt_trigger_lower, it is recommended to set it to the middle of the slide range (0.5)
        self.schmitt_trigger_reverse = config.getfloat('pressure_sensor_bldc_schmitt_trigger_reverse_lower', 0.5)
        
        if self.schmitt_trigger_upper < self.schmitt_trigger_lower:
            raise config.error("Schmitt trigger upper must be greater than lower")
        elif self.schmitt_trigger_upper == self.schmitt_trigger_lower:
            raise config.error("Schmitt trigger upper must be greater than lower")

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("OAMS_LOAD_SPOOL", self.cmd_OAMS_LOAD_SPOOL,
                                desc=self.cmd_LOAD_SPOOL_help)
        gcode.register_command("OAMS_START_UNLOAD_SPOOL", self.cmd_OAMS_START_UNLOAD_SPOOL)
        gcode.register_command("OAMS_CONTINUE_UNLOAD_SPOOL", self.cmd_OAMS_CONTINUE_UNLOAD_SPOOL)
        gcode.register_command("OAMS_LOAD_STATS", self.cmd_OAMS_LOAD_STATS,)
        gcode.register_command("OAMS_CHANGE_CLICKS", self.cmd_OAMS_CHANGE_CLICKS,)
        gcode.register_command("OAMS_LOADED", self.cmd_OAMS_LOADED,)
        gcode.register_command("OAMS_CHANGE_FILAMENT", self.cmd_OAMS_CHANGE_FILAMENT)
        gcode.register_command("OAMS_CALIBRATE_CLICKS", self.cmd_OAMS_CALIBRATE_CLICKS)
        gcode.register_command("OAMS_CALIBRATE_HUB_HES", self.cmd_OAMS_CALIBRATE_HUB_HES)
        gcode.register_command("OAMS_SPOOL", self.cmd_OAMS_SPOOL)
        
        # these events are given here as a place holder in case some
        # initialization is needed during the startup of the printer for the OAMS
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # self.printer.register_event_handler("idle_timeout:ready", self.handle_ready)
        # self.printer.register_event_handler("idle_timeout:idle", self.handle_ready)
        
        self.printer.register_event_handler("g1:extrude", self.schmitt_trigger_forward)
        self.printer.register_event_handler("g1:retract", self.schmitt_trigger_backward)
        self.hard_stop = False
        
        self.follower_enable = False
        
        self.printer.add_object('oams', self)
        
        # these must be populated from the database
        self.spools = []
        for i in range(4):
            conn = sqlite3.connect(self.database_name)
            c = conn.cursor()
            res = c.execute('''SELECT spool_idx, start_percentage, type, clicks FROM spool_status WHERE spool_idx = ?''', (i,))
            row = res.fetchone()
            spool = OAMSSpool()
            if row is None:
                unix_time = time.time()
                c.execute('''INSERT INTO spool_status (spool_idx, start_percentage, type, unix_timestamp) VALUES (?, ?, ?, ?)''', (i, 100.0, "ABS", unix_time))
                conn.commit()
            else:
                spool.start_percentage = row[1]
                spool.type = row[2]
                spool.set_clicks(row[3])
            
            self.spools.append(spool)
            c.close()
            conn.close()
            
        self.bldc_run_reset = 0
        
        # errors and monitoring conditions
        OAMSError = Enum('OAMS_ERRORS', 
                    ['BLDC_MOTOR_NOT_RUNNING', 
                    'F1S_DC_MOTOR_OVERLOADED', 
                    'TIME_OUT_WHILE_WAITING', 
                    'ENCODER_NOT_CLICKING', 
                    'SLIDE_NOT_UNLOADING', 
                    'SLIDE_NOT_LOADING', 
                    'HES_STILL_ON', 
                    'HES_STILL_OFF', 
                    'ENCODER_READS_TOO_MANY_CLICKS'])
        
        OAMSError.__doc__ = "OAMS Error Codes"
        
        class EncoderNotClicking():
            def __init__(self, encoder):
                self.encoder = encoder
                self.last_clicks = 0
            
            def clicking(self):
                clicks = self.encoder.get_clicks()
                if clicks == self.last_clicks:
                    return False
                self.last_clicks = clicks
                return True
            
        encoderMonitor = EncoderNotClicking(self.encoder)
        
        MonitorConditions = {
            OAMSError.BLDC_MOTOR_NOT_RUNNING :  MonitorCondition(
                                                    lambda: self.tach.get_rpm == 0, 
                                                    lambda: self.handle_error(OAMSError.BLDC_MOTOR_NOT_RUNNING),
                                                    start_in_time = 3.0, # given in seconds
                                                    period = 0.5, 
                                                    end_at_time = None),
            OAMSError.F1S_DC_MOTOR_OVERLOADED : MonitorCondition(
                                                    lambda: self.current_sensor_value > 0.9,
                                                    lambda: self.handle_error(OAMSError.F1S_DC_MOTOR_OVERLOADED),
                                                    start_in_time = 0.0,
                                                    period = None,
                                                    end_at_time = None),
            OAMSError.TIME_OUT_WHILE_WAITING : MonitorCondition(
                                                    lambda: self.hard_stop,
                                                    lambda: self.handle_error(OAMSError.TIME_OUT_WHILE_WAITING),
                                                    start_in_time = 60.0, # timeout in 1 minute
                                                    period = None,
                                                    end_at_time = None),
            OAMSError.ENCODER_NOT_CLICKING : MonitorCondition(
                                                    lambda: not encoderMonitor.clicking(),
                                                    lambda: self.handle_error(OAMSError.ENCODER_NOT_CLICKING),
                                                    start_in_time = 0.0,
                                                    period = 0.5,
                                                    end_at_time = None),
            OAMSError.SLIDE_NOT_UNLOADING : MonitorCondition(
                                                    lambda: self.filament_pressure_sensor.last_value > self.schmitt_trigger_reverse,
                                                    lambda: self.handle_error(OAMSError.SLIDE_NOT_UNLOADING),
                                                    start_in_time = 1.0,
                                                    period = None,
                                                    end_at_time = None),
            OAMSError.SLIDE_NOT_LOADING : MonitorCondition(
                                                    lambda: self.filament_pressure_sensor.last_value < self.schmitt_trigger_upper,
                                                    lambda: self.handle_error(OAMSError.SLIDE_NOT_LOADING),
                                                    start_in_time = 1.0,
                                                    period = None,
                                                    end_at_time = None),
            OAMSError.HES_STILL_ON : MonitorCondition(
                                                    lambda: self.current_spool is not None and self.hub_switches[self.current_spool].on,
                                                    lambda: self.handle_error(OAMSError.HES_STILL_ON),
                                                    # TODO: Must have some way to account for a trigger event instead of a time
                                                    start_in_time = 0.0,
                                                    period = None,
                                                    end_at_time = None),
            OAMSError.HES_STILL_OFF : MonitorCondition(
                                                    lambda: self.current_spool is None or not self.hub_switches[self.current_spool].on,
                                                    lambda: self.handle_error(OAMSError.HES_STILL_OFF),
                                                    start_in_time=0.0,
                                                    period = None,
                                                    end_at_time = None),
            OAMSError.ENCODER_READS_TOO_MANY_CLICKS : MonitorCondition(
                                                    lambda: self.encoder.get_clicks() > self.load_slow_clicks + 200,
                                                    lambda: self.handle_error(OAMSError.ENCODER_READS_TOO_MANY_CLICKS),
                                                    start_in_time=0.0,
                                                    period = None,
                                                    end_at_time = None)       
        }
        
        
        self.loadMonitor = OAMSMonitor(config)
        self.loadMonitor.add_monitor_conditions(MonitorConditions[OAMSError.BLDC_MOTOR_NOT_RUNNING],
                                               MonitorConditions[OAMSError.F1S_DC_MOTOR_OVERLOADED],
                                               MonitorConditions[OAMSError.TIME_OUT_WHILE_WAITING],
                                               MonitorConditions[OAMSError.ENCODER_NOT_CLICKING],
                                               MonitorConditions[OAMSError.HES_STILL_OFF],
                                               MonitorConditions[OAMSError.ENCODER_READS_TOO_MANY_CLICKS])
        
        self.unloadMonitor = OAMSMonitor(config)
        self.unloadMonitor.add_monitor_conditions(MonitorConditions[OAMSError.BLDC_MOTOR_NOT_RUNNING],
                                                 MonitorConditions[OAMSError.F1S_DC_MOTOR_OVERLOADED],
                                                 MonitorConditions[OAMSError.TIME_OUT_WHILE_WAITING],
                                                 MonitorConditions[OAMSError.ENCODER_NOT_CLICKING],
                                                 MonitorConditions[OAMSError.HES_STILL_ON],
                                                 MonitorConditions[OAMSError.ENCODER_READS_TOO_MANY_CLICKS])
        self.unloadSlideMonitor = OAMSMonitor(config)
        self.unloadSlideMonitor.add_monitor_conditions(MonitorConditions[OAMSError.SLIDE_NOT_UNLOADING])
        
        self.loadSlideMonitor = OAMSMonitor(config)
        self.loadSlideMonitor.add_monitor_conditions(MonitorConditions[OAMSError.SLIDE_NOT_LOADING])
        
        super().__init__()
        
    def handle_error(self, error):
        self.error_state = error
        gcmd = self.printer.lookup_object('gcode')
        gcmd.respond_error("OAMS: Error %s" % error)
        self.hard_stop = True # this will make the OAMS get out of M400

    def handle_ready(self):
        self._determine_state()

        # reset HR8833s
        self.f1_stop()
        
        # create a task to read the encoder value and record it to the database
        reactor = self.printer.get_reactor()
        def _record_clicks(eventtime):
            logging.debug("OAMS: recording clicks")
            if self.follower_enable and self.current_spool is not None and not self.calibrating:
                conn = sqlite3.connect(self.database_name)
                unix_time = int(time.time())
                c = conn.cursor()
                clicks = self.encoder.get_clicks()
                self.encoder.reset_clicks() # we are only reading the delta
                type = self.spools[self.current_spool].type
                start_percentage = self.spools[self.current_spool].start_percentage
                c.execute('''INSERT OR REPLACE INTO spool_status(spool_idx, type, start_percentage, unix_timestamp, clicks) 
                             VALUES(?,?,?,?,coalesce((SELECT clicks FROM spool_status WHERE spool_idx = ?),0) + ?)''',
                        (self.current_spool, type, start_percentage, unix_time, self.current_spool, clicks))
                conn.commit()
                res = c.execute('''SELECT  clicks FROM spool_status WHERE spool_idx = ?''', (self.current_spool,))
                clicks = res.fetchone()[0]
                self.spools[self.current_spool].set_clicks(clicks)
                c.close()
                conn.close()
            return eventtime + 10.0 # record every 10 seconds
        self.record_clicks = _record_clicks
        
        reactor.register_timer(_record_clicks, reactor.NOW)
        reactor.register_timer(self.bldc_cmd_queue.command_task, reactor.NOW)
        
        # this makes sure the BLDC nreset is pulled high when the printer first starts
        self.bldc_cmd_queue.enqueue(self.bldc_cmd_queue.reset_motor)
        
        # monitor for BLDC motor not running
        def _monitor_bldc_motor(eventtime):
            if self.current_state.id == 'loading':
                self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.replay_action())
            elif self.current_state.id == "unloading":
                if self.board_revision == '1.4':
                    # here we calculate the BLDC speed based on the current load
                    adjustment = self.BLDC_PID.current_update(eventtime, self.current_sensor_value)
                    pwm = self.spools[self.current_spool].unload_lower_range + adjustment*self.spools[self.current_spool].unload_m
                    if pwm != (1 - self.bldc_cmd_queue.pwm_value):
                        logging.debug("OAMS: (UNLOADING MONITOR) setting pwm to %.3f, process: %.3f" % (pwm, self.current_sensor_value))
                        self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.run_backward(pwm, nowait=True))
                    else:
                        self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.replay_action())
                else:
                    self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.replay_action)
            return eventtime + 0.2
        reactor.register_timer(_monitor_bldc_motor, reactor.NOW)
                
        
    # GCODE commands for the OAMS, all commands are prefixed by OAMS_
    
    
    def cmd_OAMS_SPOOL(self, gcmd):
        ams = gcmd.get_int('AMS', 0, minval=0, maxval=3)
        if ams is None:
            raise gcmd.error("Missing AMS parameter")
        spool = gcmd.get_int('SPOOL', 0, minval=0, maxval=3)
        if spool is None:
            raise gcmd.error("Missing SPOOL parameter")
        
        spool_type = gcmd.get('TYPE', None)
        percentage = gcmd.get_float('PERCENTAGE', None, minval=0.0, maxval=100.0)
        clicks = gcmd.get_int('CLICKS', None, minval=0)
        
        conn = sqlite3.connect(self.database_name)
        c = conn.cursor()
        
        unix_time = int(time.time())
        if spool_type is not None:
            self.spools[spool].type = spool_type
            c.execute('''INSERT OR REPLACE INTO spool_status(spool_idx, type, unix_timestamp) 
                            VALUES(?,?,?)''',
                    (spool, spool_type, unix_time))
            
        if percentage is not None:
            self.spools[spool].start_percentage = percentage
            c.execute('''INSERT OR REPLACE INTO spool_status(spool_idx, start_percentage, unix_timestamp, clicks)
                            VALUES(?,?,?, 0)''',
                    (spool, percentage,unix_time))
        if clicks is not None:
            self.spools[spool].set_clicks(clicks)
            if percentage is None:
                percentage = 100.0
            c.execute('''INSERT OR REPLACE INTO spool_status(spool_idx, clicks, unix_timestamp, start_percentage) 
                            VALUES(?,?,?,?)''',
                    (spool, clicks, unix_time, percentage))
        
        res = c.execute('''SELECT start_percentage, type, clicks FROM spool_status WHERE spool_idx = ?''', (spool,))
        row = res.fetchone()
        self.spools[spool].type = row[1]
        self.spools[spool].start_percentage = row[0]
        self.spools[spool].set_clicks(row[2])
        if row is not None:
            gcmd.respond_info("OAMS: Spool %d (%s) %.3f%%, %d clicks" % (spool, row[1], self.spools[spool].get_percentage(), row[2]))
        else:
            gcmd.respond_info("OAMS: No information about spool in the database!")
        conn.commit()
        c.close()
        conn.close()
        
        
    cmd_LOAD_STATS_help = "Query load statistics"
    def cmd_OAMS_LOAD_STATS(self, gcmd):
        gcmd.respond_info("Load statistics: %s %s %s" % (self.current_state.id, self.current_spool, self.hard_stop))

    def cmd_OAMS_CALIBRATE_HUB_HES(self,gcmd):
        self.calibrating = True
        ams_idx = gcmd.get_int('AMS', 0, minval=0, maxval=3)
        spool_idx = gcmd.get_int('SPOOL', 0, minval=0, maxval=3)
        oams_name = gcmd.get("OAMS_NAME", None)

        if ams_idx is None  or spool_idx is None:
            raise gcmd.error("Missing AMS or SPOOL parameter")
        if self.current_spool == spool_idx:
            raise gcmd.error("Spool %s is already loaded" % spool_idx)
        if not self.f1s_switches[spool_idx].on:
            raise gcmd.error("Spool %s is not inserted, please make sure the filament is inserted into the feeder" % spool_idx)
        if oams_name is None:
            raise gcmd.error("Missing OAMS_NAME parameter, usually this is oams1, oams2, etc...")
        
        reactor = self.printer.get_reactor()

        self.hub_switches[spool_idx].calibrated = False
        self.hub_switches[spool_idx].low_calibration = []
        self.hub_switches[spool_idx].high_calibration = []

        for i in range(3):
            starting_adc_value = self.hub_switches[spool_idx].adc_value
            self.hub_switches[spool_idx].high_calibration.append(starting_adc_value)
            self.f1_enable(spool_idx, forward=True, speed=0.5)
            hysteresis = 0.075
            while(abs(self.hub_switches[spool_idx].adc_value - starting_adc_value) < hysteresis):
                reactor.pause(reactor.monotonic() + 0.01)
            self.f1_stop()
            reactor.pause(reactor.monotonic() + 0.5)
            starting_adc_value = self.hub_switches[spool_idx].adc_value
            self.hub_switches[spool_idx].low_calibration.append(starting_adc_value)
            self.f1_enable(spool_idx, forward=False, speed=0.5)
            while(abs(self.hub_switches[spool_idx].adc_value - starting_adc_value) < hysteresis):
                reactor.pause(reactor.monotonic() + 0.01)
            self.f1_stop()
            reactor.pause(reactor.monotonic() + 0.5)
        average_low = sum(self.hub_switches[spool_idx].low_calibration) / len(self.hub_switches[spool_idx].low_calibration)
        average_high = sum(self.hub_switches[spool_idx].high_calibration) / len(self.hub_switches[spool_idx].high_calibration)
        self.hub_switches[spool_idx].on_value = (average_low + average_high) / 2
        self.hub_switches[spool_idx].calibrated = True
        hub_on_value_type = None
        if average_low < average_high:
            hub_on_value_type = "below"
            self.hub_switches[spool_idx].on_value_type = "below"
        else:
            hub_on_value_type = "above"
            self.hub_switches[spool_idx].on_value_type = "above"

        configfile = self.printer.lookup_object('configfile')

        configfile.set(oams_name, "hub_on_value", "%f,%f,%f,%f" % (self.hub_switches[0].on_value, self.hub_switches[1].on_value, self.hub_switches[2].on_value, self.hub_switches[3].on_value))
        configfile.set(oams_name, "hub_on_value_type", hub_on_value_type)

        gcmd.respond_info("OAMS: Hub switch %s calibrated, switch will be on %s the value %s" % (spool_idx, self.hub_switches[spool_idx].on_value_type ,self.hub_switches[spool_idx].on_value))
        self.calibrating = False

    def _load_spool(self, gcmd, load_speed):
        ams_idx = gcmd.get_int('AMS', None, minval=0, maxval=3)
        spool_idx = gcmd.get_int('SPOOL', None, minval=0, maxval=3)

        if ams_idx is None  or spool_idx is None:
            raise gcmd.error("Missing AMS or SPOOL parameter")
        
        if self.current_state.id != 'unloaded':
            raise gcmd.error("Filament already loaded, please unload filament first")

        if not self.f1s_switches[spool_idx].on:
            raise gcmd.error("Spool %s is not inserted, please make sure the filament is inserted into the feeder" % spool_idx)
        
        reactor = self.printer.get_reactor()
        self.load_spool()
        self.encoder.reset_clicks()
        self.current_spool = spool_idx
        while(not self.bldc_cmd_queue.empty()):
            reactor.pause(reactor.monotonic() + 0.1)
        self.bldc_cmd_queue.stop()
        reactor.pause(reactor.monotonic() + 0.101)
        self.f1_enable(spool_idx, forward=True, speed=1.0)
        self.bldc_cmd_queue.run_forward(0.6)
        while(not self.hub_switches[spool_idx].on):
            reactor.pause(reactor.monotonic() + 0.1)
        reactor.pause(reactor.monotonic() + 1.0)
        self.f1_stop()
        self.bldc_cmd_queue.run_forward(load_speed)
        while(abs(self.encoder.get_clicks()) < self.load_slow_clicks):
            reactor.pause(reactor.monotonic() + 0.01)
        self.bldc_cmd_queue.run_forward(0.6)
        while(self.filament_pressure_sensor.last_value < self.schmitt_trigger_upper):
            reactor.pause(reactor.monotonic() + 0.01)
        self.f1_stop()
        self.bldc_cmd_queue.stop()
        reactor.pause(reactor.monotonic() + 0.101)
        self.loading_end()
        self.follower_enable = True
        if not self.calibrating:
            self.encoder.reset_clicks()

    cmd_LOAD_SPOOL_help = "Load a spool of filament"
    def cmd_OAMS_LOAD_SPOOL(self, gcmd):
        self.error_state = None
        self._load_spool(gcmd, 1.0)
        
    def _run_first_unload_sequence(self, gcmd):
        if self.current_spool is None:
            raise gcmd.error("No spool is currently loaded")
        
        self.rewinding = True
        
        unload_speed = gcmd.get_float('SPEED', None)
        if unload_speed is None:
            unload_speed = self.spools[self.current_spool].get_unload_speed()
        
        # the unload speed should be set to 0.45 + 0.20 * (the weight of the spool in grams) / 1000
        # we must discern the weight of the filament using the current sensor
        reactor = self.printer.get_reactor()
        self.record_clicks(reactor.NOW) # this makes sure to flush the current clicks to the database
        self.follower_enable = False
        self.unload_spool()
        
        # bldc queue lock start
        self.bldc_cmd_queue.lock()
        while(not self.bldc_cmd_queue.empty()):
            reactor.pause(reactor.monotonic() + 0.1)
        self.bldc_cmd_queue.stop()    
        self.encoder.reset_clicks()
        self.f1_enable(self.current_spool, forward=False)
        delay = self.spools[self.current_spool].get_unload_delay()
        reactor.pause(reactor.monotonic() + delay) # the f1s dc motor has some delay getting the spool to start moving because of inertia
        self.bldc_cmd_queue.run_backward(unload_speed, nowait=True)
        self.bldc_cmd_queue.unlock()
         # bldc queue lock end
        
    
    def _start_unload_spool(self, gcmd):
        self.retract()
        if self.fast_unload:       
            self._run_first_unload_sequence(gcmd)
        return
    
    def _continue_unload_spool(self, gcmd):
        if not self.fast_unload:
            self._run_first_unload_sequence(gcmd)

        reactor = self.printer.get_reactor()

        while(self.hub_switches[self.current_spool].on):
            reactor.pause(reactor.monotonic() + 0.01)
        
        # bldc queue lock start    
        self.bldc_cmd_queue.lock()
        while(not self.bldc_cmd_queue.empty()):
            reactor.pause(reactor.monotonic() + 0.01)
        # we must terminate the monitor for the BLDC motor not running
        self.bldc_cmd_queue.stop()
        reactor.pause(reactor.monotonic() + 0.4)
        self.f1_stop()
        reactor.pause(reactor.monotonic() + 0.101)
        self.current_spool = None
        
        if self.reverse_dc_motor_on_unload:
            # once done we want to momentarily forward the f1 motor to make sure the gearbox is in neutral
            self.f1_enable(self.current_spool, forward=True)
            reactor.pause(reactor.monotonic() + 0.1)
            self.f1_stop()
            reactor.pause(reactor.monotonic() + 0.1)
            
        self.unloading_end()
        self.bldc_cmd_queue.unlock()
        # bldc queue lock end
        
        # dump the current sensor values
        logging.debug("OAMS: Current sensor values: %s" % self.current_sensor_values)
        self.current_sensor_values = []
        self.rewinding = False
        return None
        
    cmd_UNLOAD_SPOOL_help = "Start unload sequence currently loaded spool"
    def cmd_OAMS_START_UNLOAD_SPOOL(self, gcmd):
        self.error_state = None
        self._start_unload_spool(gcmd)

    def cmd_OAMS_CONTINUE_UNLOAD_SPOOL(self, gcmd):
        self.error_state = None
        self._continue_unload_spool(gcmd)
        
    # AMS, SPOOL, OAMS_NAME must be specified
    def cmd_OAMS_CALIBRATE_CLICKS(self, gcmd):
        self.calibrating = True
        oams_name = gcmd.get('OAMS_NAME', None)
        if oams_name is None:
            raise gcmd.error("Missing OAMS name, this is usually oams1, oams2, etc.")
        self.encoder.reset_clicks()
        self._load_spool(gcmd, 0.5)
        number_of_clicks = abs(self.encoder.get_clicks())
        gcmd.respond_info("Number of clicks: %s" % number_of_clicks)
        number_of_clicks -= 100
        gcmd.respond_info("Number of clicks to save to configuration: %s" % number_of_clicks)
        self._start_unload_spool(gcmd)
        self._continue_unload_spool(gcmd)
        oams_name = "oams %s" % oams_name
        # Store results for SAVE_CONFIG
        configfile = self.printer.lookup_object('configfile')
        configfile.set(oams_name, 'load_slow_clicks', "%d" % (number_of_clicks,))
        gcmd.respond_info("Done calibrating clicks, output saved to configuration")
        self.calibrating = False
        
    def cmd_OAMS_LOADED(self, gcmd):
        self.resume_loaded()
        gcmd.respond_info("OAMS: Loaded, state %s" % self.current_state.id)
        
    def cmd_OAMS_CHANGE_FILAMENT(self, gcmd):
        self.change_filament()
        reactor = self.printer.get_reactor()
        while(not self.bldc_cmd_queue.empty()):
            reactor.pause(reactor.monotonic() + 0.1)
        if self.fast_unload:
            self.bldc_cmd_queue.coast(nowait=True)
        gcmd.respond_info("OAMS: Change Filament, state %s" % self.current_state.id)
        
    def cmd_OAMS_CHANGE_CLICKS(self, gcmd):
        value = gcmd.get_int('SET', None, minval=0)
        if value is None:
            raise gcmd.error("Missing SET parameter")
        self.load_slow_clicks = value
        gcmd.respond_info("Load clicks set to %s" % self.load_slow_clicks)
        
        
    def _determine_state(self):
        # determine the state of the system LOADED or UNLOADED
        current_spool = None
        reactor = self.printer.get_reactor()
        for idx in range(0, len(self.hub_switches)):
            while(not self.hub_switches[idx].ready):
                reactor.pause(reactor.monotonic() + 0.1)
            while(not self.f1s_switches[idx].ready):
                reactor.pause(reactor.monotonic() + 0.1)

        for idx in range(0, len(self.hub_switches)):
            if self.hub_switches[idx].on and self.f1s_switches[idx].on:
                if current_spool == None:
                    current_spool = idx
                else:
                    self.too_many_spools_loaded()
                    raise Exception("Too many spools loaded")
        if current_spool is not None:
            self.current_spool = current_spool
            self.determined_state_loaded()
            self.follower_enable = True
        else:
            self.current_spool = None
            self.determine_state_unloaded()

    # this method is here to unload the pressure on the slide
    # given a large retract event
    # the G1 command must be emitted by the gcode.py under the G1 command
    # so far we are not using it, however if a retract event
    # larger than 10mm is given to the extruder
    # without issuing OAMS_UNLOADED the extruder will most likely skip steps and bottom out        
    def schmitt_trigger_backward(self):
        logging.debug("g1:retract")
        self.retract()
        logging.debug("OAMS_Event: Retract, state %s" % self.current_state.id)
        
    # correspondingly this method is here to load the pressure on the slide
    # as soon as a positive extrude event occurs
    def schmitt_trigger_forward(self):
        logging.debug("g1:extrude")
        self.extrude()
        logging.debug("OAMS_Event: Extrude, state %s" % self.current_state.id)
    
    def f1_enable(self, spool_idx, forward=True, speed = 1.0, nowait=False):
        mux_dict = None
        if forward:
            mux_dict = self.dc_motor_table_forward
        else:
            mux_dict = self.dc_motor_table_backward
        mux_val = mux_dict[spool_idx]
        motor_driver_select_bit = mux_val & 0b01
        motor_select_bit = (mux_val & 0b10) >> 1
        
        self.m_driver_select_pin.set_pin(motor_driver_select_bit, time_ahead=0.100)
        self.m_select_pin.set_pin(motor_select_bit, time_ahead=0.100)
        # this is approximately the max_time for the mux to be set
        # at 3v3 the max rise time is 33ns or 0.000033ms hence 0.0001ms would be more than sufficient
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + .101)
        
        if forward:
            self.m_pwm_a_pin.set_pin(speed)
            self.m_pwm_b_pin.set_pin(0.0)
            self.f1_state = "RUN_FORWARD"
        else:
            self.m_pwm_a_pin.set_pin(0.0)
            self.m_pwm_b_pin.set_pin(speed)
            self.f1_state = "RUN_BACKWARD"
        
        #self.m_disable_pin.set_pin(0.0)
    
    def f1_stop(self):
        self.f1_state = "STOP"
        #self.m_disable_pin.set_pin(1.0)
        self.m_pwm_a_pin.set_pin(0.0)
        self.m_pwm_b_pin.set_pin(0.0)
    
    # bldc_slow_pwm = 0.50
    # def _encoder_callback(self, clicks):
    #     clicks = abs(clicks)
        # logging.debug("OAMS: number of clicks %s" % clicks)
        # if self.current_state.id == 'loading' and clicks > self.load_slow_clicks:
        #     logging.debug("OAMS: loading slowdown by encoder callback")
        #     self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.run_forward(self.bldc_slow_pwm), None)
    
    PRINTING_TRIGGER_SPEED = 0.45
    def filament_pressure_sensor_callback(self, read_time, read_value):
        
        logging.debug("OAMS: Filament Pressure Sensor: %s, state: %s, u: %s, l: %s" % (read_value, self.current_state.id, self.schmitt_trigger_upper, self.schmitt_trigger_lower))

        if self.follower_enable:
            if self.current_state.id == 'loaded_fw' and read_value < self.schmitt_trigger_lower:
                logging.debug("OAMS: running forward")
                def _run_forward():
                    self.bldc_cmd_queue.run_forward(self.PRINTING_TRIGGER_SPEED, nowait = True)
                self.bldc_cmd_queue.enqueue(_run_forward, None)
            elif self.current_state.id == 'loaded_fw' and read_value > self.schmitt_trigger_upper:
                logging.debug("OAMS: stopping")
                self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.stop(nowait = True), None)
            elif self.current_state.id == 'loaded_bw' and read_value < self.schmitt_trigger_reverse and not self.fast_unload:
                logging.debug("OAMS: stopping")
                self.bldc_cmd_queue.enqueue(lambda: self.bldc_cmd_queue.stop(nowait = True), None)
            elif self.current_state.id == 'loaded_bw' and read_value > self.schmitt_trigger_reverse and not self.fast_unload:
                logging.debug("OAMS: running backward")
                def _run_backward():
                    self.bldc_cmd_queue.run_backward(self.PRINTING_TRIGGER_SPEED, nowait = True)
                self.bldc_cmd_queue.enqueue(_run_backward, None)
    
    def state_change_f1s_switch_callback(self, idx, on, adc_value):
        self.led_white[idx].set_pin(on)
            

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
            self.mcu_pin.set_pwm(print_time, value)
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
    
    def __init__(self, printer, config, sensor_pin, idx, on_value_type, on_value, callback = None, log_info=False, db_name=None):
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
        self.adc_value = 0
        self.calibrated = False
        self.low_calibration = []
        self.high_calibration = []
        self.db_name = db_name
        self.ready = False

    def get_report_time_delta(self):
        return REPORT_TIME
    
    def adc_callback(self, read_time, read_value):
        
        def write_adc(db_name, idx, read_value):
            """Insert data into the database."""
            conn = sqlite3.connect(db_name)
            unix_time = int(time.time())
            c = conn.cursor()
            c.execute('''INSERT INTO hes_internal_hub(unix_timestamp, spool_idx, adc_value) VALUES(?,?,?)''',
                    (unix_time, idx, read_value))
            conn.commit()
            # delete old records, only keep 1000 entries per state and spool
            # get max id#     
            c.execute('''delete from hes_internal_hub
                    where id not in (select id from hes_internal_hub
                    where spool_idx = ? and is_on = 0
                    order by unix_timestamp DESC
                    limit 1000) 
                    or id not in 
                    (select id from hes_internal_hub
                    where spool_idx = ? and is_on = 1
                    order by unix_timestamp DESC
                    limit 1000)''', (idx, idx))
            c.close()
            conn.close()
            
        # if self.db_name is not None:
        #      db_thread = threading.Thread(target=write_adc, args=(self.db_name, self.idx, read_value))
        #      db_thread.start()
        self.adc_value = read_value
        self.ready = True
        
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
                self.callback(self.idx, on, read_value)

        if self.log_info:
            logging.debug("OAMS: ADC HES Switch (%s): %s (%s)", self.hes_switch_name , on, read_value)
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
        
    def get_cps(self):
        return self.cps
        
    
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
    def __init__(self, config, callback=None, reverse = False):
        self.callback = callback
        self.reverse = reverse
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
        if self.reverse:
            read_value = 1.0 - read_value
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
        
        #logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%f",
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

# this code here can be used locally to print the state diagram of the OAMS
# from statemachine.contrib.diagram import DotGraphMachine
# graph = DotGraphMachine(OAMS)
# dot = graph()
# dot.write_png("/home/pi/printer_data/config/m.png")