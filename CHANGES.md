# Changelog

## alpha (unreleased)

### Added

* #37 Hackflight Parametrization
* #42 ESKF estimates gyrometer bias
* #43 ESKF estimates vertical position and velocity
* #43 Add IMU to the list of available sensors 
* #43 Allow to define the frequency at which sensors correct/update ESKF estimates
* #46 Implement handler for getting Motor values via MSP
* #47 MSP getters for Mosquito version and presence of Position Board
* #48 MSP message and methods to calibrate ESCs
* #50 Check if position board is connected via MSP
* #45 Allow to set rangefinder parameters with an MSP message
* #55 Enable setting LEDs on/off with an MSP message
* #68 Motor pins are set according to the specified Mosquito version
* #70 Transmitter calibration
* #73 Battery monitoring and low battery trigger
* #75 ESKF estimates horizontal positions and velocities as well as accelerometer bias
* #79 MSP message to get PID constants
* #95 MSP message to retrieve flight mission execution status
* #96 MSP message to stop the Mosquito
* #102 Filter OF measures with a LPF
* #102 Express linear velocities in IMU frame
* #102 ESKF tuning and cleanup
* #103 Allow to define MSP messages with ID > 200 that do not acknowledge
* #104 Vertical velocity control when in Altitude Hold mode and outside of the deadband 
* #105 Filter ESKF estimated linear velocities with a LPF

### Changed

* #43 Update file headers
* #75 Refactor tasks in main loop
* #76 Improve Rate and Level controllers 
* #80 Enable clearing EEPROM by sections

### Removed

* #44 Obsolete ESKF helper struct matrices

### Fixed

* #39 Firmware version
* #41 Serial communications with the platform
* #49 Optical Flow connection pin
* #51 Modify parameters at runtime (Mosquito version and Position Board presence)
* #52 ESC calibration procedure
* #77 Indexing of the Optical Flow's covariance matrix
* #77 Inclusion of the Optical Flow in the ESKF sensor array
