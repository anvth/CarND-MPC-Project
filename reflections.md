# The model

The model used in this project is known as Kinematic Bicycle model. The model is non-linear as it takes changes of heading direction into account. This model also neglects dynamic effects such as inertia, friction and torque.

The model state is:
- px: X-position of the vehicle in the forward direction
- py: Y-position of the vehicle in the lateral direction
- psi: Orientation of the vehicle
- v: Velocity of the vehicle

The actuators of the vehicle are:
- deltaPsi: Steering angle
- a: Acceleration

The update equations for our model used to predict future states are:
- px(t+1) = px(t) + v(t) * cos(psi(t)) * dt
- py(t+1) = py(t) + v(t) * sin(psi(t)) * dt
- psi(t+1) = psi(t) + v(t) / Lf * deltaPsi * dt
- v(t+1) = v(t) + a * dt;

Where dt is the timestep between predictions
      Lf is the distance between the front and the center of gravity of the vehicle, which determines its turning radius.


# Timestep Length and Elapsed Duration (N & dt)

The time T = N * dt defines the prediction horizon. Short prediction horizons lead to more responsive controllers, but are less accurate and can suffer from instabilities. For example, when the values of N and dt were set to 8 and 0.01 respectively, the vehicle started to oscillate on straight fast speeds. Long prediction horizons generally lead to smoother controls but the car fails to achive higher speeds. The drive becomes very conservative.

I finally ended up using 15 for N and 0.12 for dt.


# Polynomial Fitting and MPC Preprocessing

The provided waypoints are transformed into vehicle space and then fitted to a polynomial. The polynomial is fitted on to 3-dimensions. The following formulas were used to transform points to Vehicle's orientation:
- ptsx_vehicle[i] = x * cos(-psi) - y * sin(-psi);
- ptsy_vehicle[i] = x * sin(-psi) + y * cos(-psi);

The polyfit() function is next invoked to fit the transformed waypoints. This essentially draws the path which to vehicle should try to take. The cross track error can be calculated by evaluating polynomial function at px, this is done through polyeval() function. The psi error is calculated from the derivative of the polyfit line.  

With the planned path in vehicle space, the MPC solver is is fed with the state vector, which also includes x/y position and the orientation of the vehicle.

The solve() function in the MPC class has variables based off the state size, actuators and timestamps. All these variables are initially set to zero except the first variable, which is set to the input current state(lines ***-***). Next, I set upper and lower limits for the variables. All variables are set to an upper limit of max negative and lower limit max positive. These limits for delta and "a" are based on the simulated vehicle having max steering angles and maximum throttle or breaking of these values. The delta values are limited between -25 to 25(lines ***-***) and the value "a" is limited between -1 to +1(lines ***-***). Similarly, Constraints are also set. Initially to zeros and constrain their lower and upper limits to input state.


# Model Predictive Control with Latency

In a real world scenario, there is a latency between the actuator calculation (when the model tells the car to perform a steering or acceleration/braking change) and when the car actually performs that action. The simulator here, also induces a latency of 100ms.

This lead to the following effects:
- At high speed, the car would not recognise sharp change is steering angles there by going off-road.
- Any existing oscillation would get amplified.

In order to overcome above effects, I set the timestamp length and elapsed duration to a relatively high value. Also, I increased the penalty on the use of actuators.
