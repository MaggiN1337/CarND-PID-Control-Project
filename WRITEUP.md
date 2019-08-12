# CarND-PID-Control-Project
Self-Driving Car Engineer Nanodegree Program

Model Documentation by Marcus Neuert, 12.08.2019

### Project Goals and how I managed them
No. | Criteria | How I solved it 
---|---------|---------------
1|Code compiles|Well, of course it does... Using JetBrains CLion on Windows10 without WebSocket support, so code execution was done on Udacity Workspace
2|Hyperparameters were chosen|By try and error while considering the definition of the three P-I-D-components
3|Vehicle drives a lap without mistake|No tire leaves the drivable portion, because speed is adjusted to turing angle.

### PID algorithm explanation
1. The P component calculates the output in proportion to the CTE. So if the current error is high, the controller will 
react very hard. In case of steering, it will steer very hard so that the vehicle will overshoot, because the angle will
be too big. For that reason, I use the P-component with a factor of 0.3 for the steering PID-Controller.

2. The integral component is for correcting the vehicles bias. The integral will multiply the sum of all previous errors
and therefore gets bigger and bigger. A really small i-parameter is the right choice, otherwise the vehicle is out of 
control very quickly as the i_error has too much impact.

3. The differential parameter will work against the proportional factor, if the error is already decreasing. To give the
differential correction, so the correction over time, a high effect, you can use higher values for Kd.


### Possible improvement for next time
1. Use twiddle for automatic and precise parameter optimization
2. Use a second PID controller for the throttle to reach higher speeds.
