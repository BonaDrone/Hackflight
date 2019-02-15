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

### Changed

* #43 Update file headers

### Removed

* #44 Obsolete ESKF helper struct matrices

### Fixed

* #39 Firmware version
* #41 Serial communications with the platform
* #49 Optical Flow connection pin
* #51 Modify parameters at runtime (Mosquito version and Position Board presence)
* #52 ESC calibration procedure
