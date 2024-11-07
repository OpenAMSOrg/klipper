# OAMS Manager for Filament Runout Monitoring

## Overview
The `OAMSManager` manages filament spool loading and unloading for 3D printers, specifically monitoring for filament runouts, handling automated loading of new spools, and registering G-code commands for advanced control over filament operations.

## Configuration
`OAMSManager` requires the following settings in your configuration file:
- Filament groups (grouped collections of spools to manage as a set).
- Reactors and timers for handling spool monitoring and filament runout events.
- Registered event handler for `klippy:ready` to initialize when the printer is ready.

## Commands

### G-code Commands
The following G-code commands are registered by `OAMSManager` and can be executed from the printer's terminal or added to print start/stop scripts.

#### `OAMSM_CURRENT_LOADED_GROUP`
- **Description:** Returns the current loaded filament group, if any.
- **Usage:** `OAMSM_CURRENT_LOADED_GROUP`
- **Example Output:** `Current Group: T0`

#### `OAMSM_FOLLOWER`
- **Description:** Enables or disables the follower motor on the currently loaded spool. It also allows setting the rotation direction for the follower.
- **Parameters:**
  - `ENABLE` (required): `1` to enable, `0` to disable.
  - `DIRECTION` (required): `1` for clockwise, `0` for counterclockwise.
- **Usage:** `OAMSM_FOLLOWER ENABLE=1 DIRECTION=1`

#### `OAMSM_UNLOAD_FILAMENT`
- **Description:** Unloads the current spool from any loaded filament group.
- **Usage:** `OAMSM_UNLOAD_FILAMENT`
- **Example Output:** `Unloading currently loaded spool`

#### `OAMSM_LOAD_FILAMENT`
- **Description:** Loads a spool from a specified filament group.
- **Parameters:**
  - `GROUP` (required): Name of the filament group to load from.
- **Usage:** `OAMSM_LOAD_FILAMENT GROUP=T0`
- **Example Output:** `Loaded spool from T0`


# G-code Commands for OAMS ACE Mainboard

This document provides a list of all available G-code commands for the OAMS ACE mainboard, along with brief descriptions.

## Command List

ALL commands must contain OAMS=<ams_idx> as the parameter to address the correct AMS.

### 1. `OAMS_LOAD_SPOOL`
**Description:** Load a specified spool of filament into the system.  
**Usage:** `OAMS_LOAD_SPOOL SPOOL=<index>`  
**Parameters:**  
- `SPOOL`: Specifies the spool index (0 to 3).  

### 2. `OAMS_UNLOAD_SPOOL`
**Description:** Unload the currently loaded spool of filament.  
**Usage:** `OAMS_UNLOAD_SPOOL`  
**Parameters:** None.  

### 3. `OAMS_FOLLOWER`
**Description:** Enable or disable the follower mechanism and set its direction.  
**Usage:** `OAMS_FOLLOWER ENABLE=<0|1> DIRECTION=<0|1>`  
**Parameters:**  
- `ENABLE`: Enables (1) or disables (0) the follower.
- `DIRECTION`: Sets the direction (0 for reverse, 1 for forward).  

### 4. `OAMS_CALIBRATE_PTFE_LENGTH`
**Description:** Calibrate the length of the PTFE tube for accurate filament measurements.  
**Usage:** `OAMS_CALIBRATE_PTFE_LENGTH SPOOL=<index>`  
**Parameters:**  
- `SPOOL`: The spool index (0 to 3) for which the calibration should be done.  

### 5. `OAMS_CALIBRATE_HUB_HES`
**Description:** Calibrate the range of the hub Hall Effect Sensor (HES) for a specific spool.  
**Usage:** `OAMS_CALIBRATE_HUB_HES SPOOL=<index>`  
**Parameters:**  
- `SPOOL`: Specifies the spool index (0 to 3).  

### 6. `OAMS_PID_SET`
**Description:** Manually set the PID values for filament rewinds.  
**Usage:** `OAMS_PID_SET P=<float> I=<float> D=<float> TARGET=<float>`  
**Parameters:**  
- `P`: Proportional gain value.  
- `I`: Integral gain value.  
- `D`: Derivative gain value.  
- `TARGET`: Target flowrate or performance threshold. 
