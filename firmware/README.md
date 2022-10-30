## TODO

 - [x] ADD: torque<->current conversion
 - [ ] ADD: implement CORDIC for trig function calculation
 - [ ] ADD: implement position bound
 - [ ] ADD: implement velocity bound
 - [x] FIX: safety feature, change the control of PWM output to using MOE bit
 - [x] FIX: move current control functions inside `current_controller` file
 - [x] FIX: move position control functions inside `position_controller` file
 - [ ] Add: improve calibration sequence, ramp up current gradually to avoid sudden motor movement.
 - [ ] ADD: implement CAN message protocol
 - [ ] ADD: implement host machine software (Python API)
 - [ ] ADD: implement host machine software (Web GUI)
 - [ ] ADD: reduce motor noise
 - [ ] ADD: safety feature, temperature sense and over-temperature shutdown
 - [ ] ADD: safety feature, heart beat CAN message and watchdog shutdown
 - [ ] REFACTOR: merge encoder direction together with cpr
