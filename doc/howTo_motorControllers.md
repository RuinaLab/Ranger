# How to work with motor control boards


### Controller Form:

Each of Ranger's motors has a dedicated motor control board, that runs a 2 kHz controller. The controller is of the form:
`u = uRef - kp*(posErr) kd*(velErr)`

Where:
- `u` = current sent to motor
- `uRef` = nominal current, as expected based on model, computed at a high level.
- `kp` = proportional gain (set at a low level) 
- `posErr` = position error, computed at a high level.
- `kd` = proportional gain (set at a low level) 
- `velErr` = velocity error, computed at a high level.

### Controller I/O

Each controller has three parameters that the high-level controller can adjust:
- `ID_MCXX_COMMAND_CURRENT` = uRef
- `ID_MCXX_STIFFNESS` = posErr
- `ID_MCXX_DAMPNESS` = velErr

Note that the `XX` in each parameter name is the two letter code for the control board of choice. The available options are:
- `FO` = foot-outer
- `FI` = foot-inner
- `H` = hip
- `SI` = steering-inner

When writing high-level code, these values are set using the communications function:
- `set_io_float(ID_MCXX_YYYYYYYY, valueToAssign)`

# How to work with motor control boards


### Controller Form:

Each of Ranger's motors has a dedicated motor control board, that runs a 2 kHz controller. The controller is of the form:
`u = uRef - kp*(posErr) kd*(velErr)`

Where:
- `u` = current sent to motor
- `uRef` = nominal current, as expected based on model, computed at a high level.
- `kp` = proportional gain (set at a low level) 
- `posErr` = position error, computed at a high level.
- `kd` = proportional gain (set at a low level) 
- `velErr` = velocity error, computed at a high level.

### Controller I/O

Each controller has three parameters that the high-level controller can adjust:
- `ID_MCXX_COMMAND_CURRENT` = uRef
- `ID_MCXX_STIFFNESS` = posErr
- `ID_MCXX_DAMPNESS` = velErr

Note that the `XX` in each parameter name is the two letter code for the control board of choice. The available options are:
- `FO` = foot-outer
- `FI` = foot-inner
- `H` = hip
- `SI` = steering-inner

When writing high-level code, these values are set using the communications function:
- `set_io_float(ID_MCXX_YYYYYYYY, valueToAssign)`

