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


## Modes of Operation


  // these are closed-loop modes
  MODE_CURRENT              = 0x10U,
  MODE_TORQUE               = 0x11U,
  MODE_VELOCITY             = 0x12U,
  MODE_POSITION             = 0x13U,

  // these are open-loop modes
  MODE_VABC_OVERRIDE        = 0x20U,
  MODE_VALPHABETA_OVERRIDE  = 0x21U,
  MODE_VDQ_OVERRIDE         = 0x22U,
  MODE_IDQ_OVERRIDE         = 0x23U,

`MODE_VABC_OVERRIDE` will not update `v_a_setpoint`, `v_b_setpoint`, and `v_c_setpoint` in the commutation loop, and user can manually override the value.

`MODE_VALPHABETA_OVERRIDE` will not update `v_alpha_setpoint` and `v_beta_setpoint` in the commutation loop, and user can manually override the value. Note that `MODE_CALIBRATION` will also override `v_alpha_setpoint` and `v_beta_setpoint` in the calibration routine to point rotor to a fixed angle.

`MODE_VDQ_OVERRIDE` will not update `v_q_setpoint` and `v_d_setpoint` in the commutation loop, and user can manually override the value.

`MODE_IDQ_OVERRIDE` will not update `i_q_setpoint` and `i_d_setpoint` in the commutation loop, and user can manually override the value. In this mode, the integrator term of the current PID loop will also be disabled. This mode can be used to tune the kp term to transform from `i_q`, `i_d` to `v_q`, `v_d`.

`MODE_CURRENT` is the current closed loop. Current controller will follow the `i_q_setpoint` and `i_d_setpoint`, and position controller will not update these values.



