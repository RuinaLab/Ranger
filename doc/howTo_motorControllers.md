# How to work with motor control boards


### Controller Form:

Each of Ranger's motors has a dedicated motor control board, that runs a 2 kHz controller. The controller is of the form:
- `current = refCurrent - kp*angle - kd*rate`

Where:
- `current` = current sent to motor 
- `refCurrent` = nominal current, as expected based on model (set at high level)
- `kp` = proportional gain (set at a high level) 
- `angle` = absolute angle of the joint, measured at low level
- `kd` = proportional gain (set at high level)
- `rate` = absolute angle rate of the joint, measured at a low level
 
### Controller I/O

Each controller has three parameters that the high-level controller can adjust:
- `ID_MCXX_COMMAND_CURRENT` = refCurrent
- `ID_MCXX_STIFFNESS` = kp
- `ID_MCXX_DAMPNESS` = kd

Note that the `XX` in each parameter name is the two letter code for the control board of choice. The available options are:
- `FO` = foot-outer
- `FI` = foot-inner
- `H` = hip
- `SI` = steering-inner

When writing high-level code, these values are set using the communications function:
- `set_io_float(ID_MCXX_YYYYYYYY, valueToAssign)`

# How to track a non-zero reference?!
Answer: do a bit of math in the high-level program
- `u = uNom + kp*(xNom-x) + kd*(vNom-v)`
- `refCurrent` = `uNom + kp*xNom + kd*vNom`