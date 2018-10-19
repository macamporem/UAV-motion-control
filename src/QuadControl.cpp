#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

//  cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
//  cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
//  cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
//  cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

  // MACR
  // We know that
  // collective thrust = 𝐹1+𝐹2+𝐹3+𝐹4, with 𝐹1=𝑘𝑓*𝜔1^2 etc
  // With regard to the moments, we know that, e.g.
  // moments x           𝜏𝑥 =(𝐹1+𝐹4−𝐹2−𝐹3) * length, and similar for moment y
  // moment z            𝜏z = 𝜏1+𝜏2+𝜏3+𝜏4, with 𝜏1=𝑘𝑚*𝜔1^2
  // Finally, we know that
  // torque = kappa * thrust


  // To produce individual motor thrust commands
  // we first need to transform input thrust & 3-axis moments into a dimensionless magnitude times kf
  // where kf is proportionality constant between individual motor thrust and squared rotor velocity
  // These are, in sum, the computed force and the 3 moments in the frame body axis
  float c_bar_x_kf = collThrustCmd;
  float p_bar_x_kf = momentCmd.x / (L/sqrtf(2.f));
  float q_bar_x_kf = momentCmd.y / (L/sqrtf(2.f));
  // About the signs:
  //Note that the momentum of rotor 1 points downwards
  //Thus, the opposite momentum of the quaduptor points upwards (ccw)
  //If we want a command a very strong upwards momentumin the quadruptor, means momentCmd<<0
  //It means we will need higher angular rotor velocity
  float r_bar_x_kf = -momentCmd.z / kappa;

  // Now, to produce individual motor thrust commands
  // we add forces and momentums when the rotor cotributes positively to that force or momentum, and
  // substract them when it contributes negatively.
  // for example, the front left motor contributes positively to the force and the three momentums, and therefore:
  cmd.desiredThrustsN[0] = (c_bar_x_kf + p_bar_x_kf + q_bar_x_kf + r_bar_x_kf)/4.f;
  // meanwhile, the front right motor contributes negatively to the momentum in the x axis and the z axis:
  cmd.desiredThrustsN[1] = (c_bar_x_kf - p_bar_x_kf + q_bar_x_kf - r_bar_x_kf )/4.f;
  // and so on for the motors in the rear right and rear left
  cmd.desiredThrustsN[2] = (c_bar_x_kf + p_bar_x_kf - q_bar_x_kf - r_bar_x_kf )/4.f ;
  cmd.desiredThrustsN[3] = (c_bar_x_kf - p_bar_x_kf - q_bar_x_kf + r_bar_x_kf )/4.f;
    
    /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)
    
  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    //The original angular rate equations of for the P controller have the structure
    //
    //    angular_acceleration = k * (angular_rate_error)
    //
    //Now, we know that
    //
    //    angular_momentum = angular_acceleration * moment_of_inertia
    //
    //And we also know that the angular rate error is the difference between the rate commanded and the actual angular rate
    //Therefore, for each of the body frame axis, the commanded angular momentum can be written as:
    
  momentCmd.x = kpPQR.x*Ixx*(pqrCmd.x-pqr.x);
  momentCmd.y = kpPQR.y*Iyy*(pqrCmd.y-pqr.y);
  momentCmd.z = kpPQR.z*Izz*(pqrCmd.z-pqr.z);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  // collThrustCmd is a force in Newtons.  We convert it to acceleration first.
  // Note that c>0 means it points upwards
  // In the body frame calculations for b_x_c, b_y_c, we need to reverse the sign
  // because an upward pointing vector has a negative 'z' sign
  float c = collThrustCmd/mass;
  if (collThrustCmd <= 0.0)
  {
      pqrCmd.x = 0.0;
      pqrCmd.y = 0.0;
      pqrCmd.z = 0.0;
  }
  if (collThrustCmd > 0.0)
  {
      float b_x = R(0,2);
      float b_x_c = CONSTRAIN(accelCmd.x/(-1*c), -maxTiltAngle, maxTiltAngle);
      float b_x_err = b_x_c - b_x;
      float b_x_p_term = kpBank * b_x_err;
      
      float b_y = R(1,2);
      float b_y_c = CONSTRAIN(accelCmd.y/(-1*c), -maxTiltAngle, maxTiltAngle);
      float b_y_err = b_y_c - b_y;
      float b_y_p_term = kpBank * b_y_err;
      
      float b_x_commanded_dot = b_x_p_term;
      float b_y_commanded_dot = b_y_p_term;

      float rot_mat11 = R(1,0)/R(2,2);
      float rot_mat12 = -R(0,0)/R(2,2);
      float rot_mat21 = R(1,1)/R(2,2);
      float rot_mat22 = -R(0,1)/R(2,2);

      pqrCmd.x = rot_mat11 * b_x_commanded_dot + rot_mat12 * b_y_commanded_dot;
      pqrCmd.y = rot_mat21 * b_x_commanded_dot + rot_mat22 * b_y_commanded_dot;
      pqrCmd.z = 0.f;
  }

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    // We calculate the position error first.
    float pos_err = posZCmd - posZ;
    integratedAltitudeError += pos_err * dt;
    // Next, we calculate the position error induced proportional commanded velocity.
    float z_dot = velZCmd + kpPosZ * pos_err + KiPosZ * integratedAltitudeError;
    // Next, we calculate the error-in-velocity proportional induced acceleration
    float z_dot_dot = accelZCmd + kpVelZ * (z_dot - velZ);
    // We must limit the resulting acceleration to be in the allowed ascend/descend rates interval.
    // Remember though that velocities are given in NED coordinates, thus ascend rate has negative sign.
    z_dot_dot = CONSTRAIN(z_dot_dot, -maxAscentRate/dt, maxDescentRate/dt);
    // Finally, remember that total vertical NED acceleration results from downward (positive) gravity and
    // upward thrust previously adjusted by quad orientation
    // z_dot_dot =  (g - R(2,2) * F/m)
    // / R(2,2) * F/m = g - z_dot_dot
    thrust = mass * (9.81f - z_dot_dot) / R(2,2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
  V3F xyz_err = posCmd - pos;
  V3F xyz_dot = velCmd + kpPosXY * xyz_err;
  xyz_dot.constrain(-maxSpeedXY,maxSpeedXY);
  accelCmd = accelCmd + kpVelXY * (xyz_dot - vel);
  accelCmd.constrain(-maxAccelXY,maxAccelXY);
  accelCmd.z = 0;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    float yawCmd_two_pi = fmod(yawCmd, 2.0f*F_PI);
    float yaw_two_pi    = fmod(yaw   , 2.0f*F_PI);
    float yaw_error = yawCmd_two_pi - yaw_two_pi;

    if (yaw_error <= -F_PI)
    {
      yaw_error += (2.0f*F_PI);
    }
    else if (yaw_error > F_PI)
    {
      yaw_error -= (2.0f*F_PI);
    }
    
    yawRateCmd = kpYaw * yaw_error;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
