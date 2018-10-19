# C++ Quad Controller 

Miguel Campo

macamporem@gmail.com | 6036673655



### Body rate and roll/pitch control (scenario 2)

- **Implement the code in the function `GenerateMotorCommands()` - COMPLETED**

  To produce individual motor thrust commands we first need to transform input thrust & 3-axis moments into a dimensionless magnitude times kf, where kf is proportionality constant between individual motor thrust and squared rotor velocity.  These are,below, the computed force and the 3 moments in the frame body axis*

    **float** c_bar_x_kf = collThrustCmd;

    **float** p_bar_x_kf = momentCmd.x / (L/sqrtf(2.f));

    **float** q_bar_x_kf = momentCmd.y / (L/sqrtf(2.f));

  Note that the momentum of rotor 1 points downwards.  Thus, the opposite momentum of the quaduptor points upwards (ccw).  If we want a command a very strong upwards momentumin the quadruptor, means momentCmd<<0.  It means we will need higher angular rotor velocity*

    **float** r_bar_x_kf = -momentCmd.z / kappa;

  Now, to produce individual motor thrust commands, we add forces and momentums when the rotor cotributes positively to that force or momentum, and substract them when it contributes negatively.  For example, the front left motor contributes positively to the force and the three momentums, and therefore:

    cmd.desiredThrustsN[0] = (c_bar_x_kf + p_bar_x_kf + q_bar_x_kf + r_bar_x_kf)/4.f;

  Meanwhile, the front right motor contributes negatively to the momentum in the x axis and the z axis:

    cmd.desiredThrustsN[1] = (c_bar_x_kf - p_bar_x_kf + q_bar_x_kf - r_bar_x_kf )/4.f;

   And so on for the motors in the rear right and rear left*

    cmd.desiredThrustsN[2] = (c_bar_x_kf + p_bar_x_kf - q_bar_x_kf - r_bar_x_kf )/4.f ;

    cmd.desiredThrustsN[3] = (c_bar_x_kf - p_bar_x_kf - q_bar_x_kf + r_bar_x_kf )/4.f;

- **implement the code in the function `BodyRateControl()` - COMPLETED**

  The original angular rate equations of for the P controller have the structure

  angular_acceleration = k \* (angular_rate_error)

  Now, we know that

  angular_momentum = angular_acceleration \* moment_of_inertia

  And we also know that the angular rate error is the difference between the rate commanded and the actual angular rate

  Therefore, for each of the body frame axis, the commanded angular momentum can be written as:*

    momentCmd.x = kpPQR.x*Ixx*(pqrCmd.x-pqr.x);

    momentCmd.y = kpPQR.y*Iyy*(pqrCmd.y-pqr.y);

    momentCmd.z = kpPQR.z*Izz*(pqrCmd.z-pqr.z);

- **Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot - COMPLETED**

- **implement the code in the function `RollPitchControl()` - COMPLETED**

  The implementation is based on the ideas discussed in the 3D control notebook.  The only difference is the need to contraint the tilt angle, which we can accomplish with a CONTRAIN statement as shown below

  ​      **float** b_x = R(0,2);

  ​      **float** b_x_c = CONSTRAIN(accelCmd.x/(-1*c), -maxTiltAngle, maxTiltAngle);

  ​      **float** b_x_err = b_x_c - b_x;

  ​      **float** b_x_p_term = kpBank * b_x_err;

  ​	...

  ​      **float** rot_mat11 = R(1,0)/R(2,2);

  ​	...

  ​      pqrCmd.x = rot_mat11 * b_x_p_term + rot_mat12 * b_y_p_term;

  ​      pqrCmd.y = rot_mat21 * b_x_p_term + rot_mat22 * b_y_p_term;

- **Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot - COMPLETED**

### Position/velocity and yaw angle control (scenario 3)

- **implement the code in the function `LateralPositionControl()` - COMPLETED**

    V3F xyz_err = posCmd - pos;

    V3F xyz_dot = velCmd + kpPosXY * xyz_err;

    xyz_dot.constrain(-maxSpeedXY,maxSpeedXY);

    accelCmd = accelCmd + kpVelXY * (xyz_dot - vel);

    accelCmd.constrain(-maxAccelXY,maxAccelXY);

- **implement the code in the function `AltitudeControl()` - COMPLETED**

​    We calculate the position error first.*

​    		**float** pos_err = posZCmd - posZ;

​    		integratedAltitudeError += pos_err * dt;

​    Next, we calculate the position error induced proportional commanded velocity.

​    		**float** z_dot = velZCmd + kpPosZ * pos_err + KiPosZ * integratedAltitudeError;

​    Next, we calculate the error-in-velocity proportional induced acceleration

​    		**float** z_dot_dot = accelZCmd + kpVelZ * (z_dot - velZ);

​    We must limit the resulting acceleration to be in the allowed ascend/descend rates interval.   Remember though that velocities are given in NED coordinates, thus ascend rate has negative sign.

​    		z_dot_dot = CONSTRAIN(z_dot_dot, -maxAscentRate/dt, maxDescentRate/dt);

​    Finally, remember that total vertical NED acceleration results from downward (positive) gravity and

​    upward thrust previously adjusted by quad orientation

​		 thrust = mass * (9.81f - z_dot_dot) / R(2,2);

- **tune parameters `kpPosZ` and `kpPosZ ` - COMPLETED**
- **tune parameters `kpVelXY` and `kpVelZ` - COMPLETED**

- **implement the code in the function `YawControl()` - COMPLETED**

  This is a proportional control.  The complexity is the angle transformation so that the resulting error in angles is between -pi and +pi

- **tune parameters `kpYaw` and the 3rd (z) component of `kpPQR` - COMPLETED**

### Non-idealities and robustness (scenario 4)

1. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle. - **COMPLETED**
2. Tune the integral control - **COMPLETED**

### Extra Challenge 1 (Optional)

**Inspect the python script `traj/MakePeriodicTrajectory.py`. Can you figure out a way to generate a trajectory that has velocity (not just position) information? - COMPLETED**

To implement a trajectory with non constant velocity, we make the following changes in the code Make PeriodicTrajectory.py.  First, we include a perturbation in the x,y,z functions.  In this case, a simple periodic perturbation:

        t_delta = 1 + 0.1 * math.sin(t * 1 * math.pi / period[0])
        x = math.sin(t_delta * t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0]
        y = math.sin(t_delta * t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1]
        z = math.sin(t_delta * t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2]
Second, we calculate the components of the velocity vector

        vx = (x-x_1)/timestep
        vy = (y-y_1)/timestep
        vz = (z-z_1)/timestep
where x_1 is the last step x coordinate, etc

**Generate a new `FigureEightFF.txt` that has velocity terms Did the velocity-specified trajectory make a difference? Why? - COMPLETED**

The quad is able to follow the velocity-adjusted trajectory only for small perturbations of the velocity component.  For perturbations of the velocity above 20%, the quad starts to deviate materially.