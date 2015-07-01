# How to read sensors and state estimates:
 - `get_io_float(ID_XXXXXX)`  <-- used in all C++ code (controller)  
 - `mb_io_get_float(ID_XXXXXX)`     <-- used in all c code (estimator)
 - `mb_io_get_time(ID_XXXXXX)`     <-- used in all c code (estimator)

The `ID_XXXXXX` is a paramter ID that is obtained from the can_table.csv file, or from the abbreviated list below. 

### Notes:
- Each sensor reading is assigned a time-stamp along with the data. This allows for data-logging, and allows for asynchronous communication.
- This estimator is really just a series of Second-Order [Butterworth Filters](https://en.wikipedia.org/wiki/Butterworth_filter). A simple implementation can be found [here](https://github.com/MatthewPeterKelly/myJavaPkgs/blob/master/mpk_dsc/ButterworthFilter.java). 
- Since the time-stamps are rounded to the nearest millisecond, the filter code uses that as the default sample rate. Any missing data is filled in using a zero-order-hold. The filter should be called about every 2ms, but in practice the time-stamps are anywhere from 1ms-4ms apart.

### Anoop's Estimator:
The code defines a bunch of functions for running Butterworth Filters, as well as piecewise-polynomial approximations to some simple analytic functions.

### Heel-strike detector:
The code computes the mean-value of the contact sensors on the swing foot in the half of the swing before it hits the ground. The threshold is then adjusted based on this "nominal" value. 

The contact sensor is actually an optical sensor that measures the deflection of the foot. There is a complicated geometric relation between the deflection and the load, so it is converted to a boolean value with a threshold. The contact detection works best when the foot is in the horizontal configuration.

### Raw Sensor Data:
- `ID_UI_ANG_RATE_X` - IMU angle rate about hip joint axis - be careful about gyro bias. This is measuring the absolute angular rate of the outer pair of legs.
- `ID_MCH_ANG_RATE` - Rate of change in the angle between the inner and outer legs.
- `ID_MCFI_RIGHT_HEEL_SENSE` - Raw value of the contact sensor. Needs to be thresholded to get a boolean value. Similar parameters for `MCFO/MCFI` and `LEFT/RIGHT`

### Filtered Sensor Data:
- `ID_E_OUTER_ANG_RATE` - Estimate of the absolute rate of the outer legs
- `ID_E_INNER_ANG_RATE` - Estimate of the absolute rate of the inner legs
- `ID_E_H_RATE` - Estimate of the rate of change in the relative angle between the inner and outer legs