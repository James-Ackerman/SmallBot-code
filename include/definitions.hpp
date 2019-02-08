#pragma once

#include "main.h"
//#include "okapi/api.hpp"
//using namespace okapi;



 //Sensor definitions
 const int DRIVE_PNEUMATIC = 3;     //port A //change to A if this doesn't work
 const int LINE_TRACKER_LEFT = 2;   // use "#define SENSOR_NAME_HERE portnumber" if const int doesnt work
 const int LINE_TRACKER_RIGHT = 1;
 const int GYRO_PORT = 4;
 const int ULTRASONIC_OUT = 5;
 const int ULTRASONIC_IN = 6;

 // Chassis definition
 // Assume this is correct for now
 // TODO: Update with the real robot values
 const int DRIVE_MOTOR_RIGHT_1 = -20;   //1,2,3 = Front, Middle, Back
 const int DRIVE_MOTOR_RIGHT_2 = 19;
 const int DRIVE_MOTOR_RIGHT_3 = -18;
 const int DRIVE_MOTOR_LEFT_1 = 17;   //1,2,3 = Front, Middle, Back
 const int DRIVE_MOTOR_LEFT_2 = -16;
 const int DRIVE_MOTOR_LEFT_3 = 15;
 const auto WHEEL_DIAMETER = 4_in; //4_in
 const auto CHASSIS_WIDTH = 12.75_in; //13_in

 const int INTAKE_MOTOR= 12;    // 1 for side roller

 const int INDEXER_MOTOR = 14;   // 2 for rotator

 const int FLYWHEEL_MOTOR = 11;

 const int DESCROER_MOTOR = -13;



 // Controller object creation
 //TODO: Reverse motors that need to be reversed
inline auto driveController = ChassisControllerFactory::create(
   {DRIVE_MOTOR_RIGHT_1, DRIVE_MOTOR_RIGHT_2, DRIVE_MOTOR_RIGHT_3},
   {DRIVE_MOTOR_LEFT_1, DRIVE_MOTOR_LEFT_2, DRIVE_MOTOR_LEFT_3},
   AbstractMotor::gearset::green,
   {WHEEL_DIAMETER, CHASSIS_WIDTH}
 );

inline Motor intake(INTAKE_MOTOR);             //motor on INTAKE_MOTOR port
//Motor intake = 7_rmtr;
inline Motor flywheel(FLYWHEEL_MOTOR, true, AbstractMotor::gearset::blue);	  		//motor on FLYWHEEL_MOTOR port
//Motor flywheel = 11_rmtr;
inline Motor indexer(INDEXER_MOTOR);           //motor on INDEXER_MOTOR port
inline Motor descorer(DESCROER_MOTOR);

 // Update inteval (in mS) for the flywheel control loop
const int FW_LOOP_SPEED = 20;

 // Maximum power we want to send to the flywheel motors
const  int FW_MAX_POWER  = 12000;

// velocity measurement
extern float           motor_velocity;         ///< current velocity in rpm

// TBH control algorithm variables
extern long            target_velocity;        ///< target_velocity velocity
extern float           current_error;          ///< error between actual and target_velocity velocities
extern float           last_error;             ///< error last time update called
extern float           gain;                   ///< gain
extern float           drive;                  ///< final drive out of TBH (0.0 to 1.0)
extern float           drive_at_zero;          ///< drive at last zero crossing
extern long            first_cross;            ///< flag indicating first zero crossing
extern float           drive_approx;           ///< estimated open loop drive
// final motor drive
extern long            motor_drive;            ///< final motor control value

 /*-----------------------------------------------------------------------------*/
 /** @brief      Set the flywheen motors                                        */
 /** @param[in]  value motor control value                                      */
 /*-----------------------------------------------------------------------------*/
 void FwMotorSet( int value );
 /*-----------------------------------------------------------------------------*/
 /** @brief      Set the controller position                                    */
 /** @param[in]  desired velocity                                               */
 /** @param[in]  predicted_drive estimated open loop motor drive                */
 /*-----------------------------------------------------------------------------*/
  void FwVelocitySet( int vel, float predicted_drive );

 /*-----------------------------------------------------------------------------*/
 /** @brief      Update the velocity tbh controller variables                   */
 /*-----------------------------------------------------------------------------*/
  void sgn(float x);

  void FwControlUpdateVelocityTbh();

 /*-----------------------------------------------------------------------------*/
 /** @brief     Task to control the velocity of the flywheel                    */
 /*-----------------------------------------------------------------------------*/
   void FwControlTask(void* param);

 inline pros::ADIDigitalOut piston(DRIVE_PNEUMATIC);         //Piston on DRIVE_PNEUMATIC port
 inline pros::ADILineSensor linetrackerL(LINE_TRACKER_LEFT); //Line tracker on LINE_TRACKER_LEFT port
 inline pros::ADILineSensor linetrackerR(LINE_TRACKER_RIGHT); //Line tracker on LINE_TRACKER_LEFT port
 inline pros::ADIAnalogIn gyro (GYRO_PORT);
 inline pros::ADIUltrasonic ultrasonic1(ULTRASONIC_IN, ULTRASONIC_OUT);


//#endif /* end of include guard:  */
