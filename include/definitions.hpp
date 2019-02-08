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
 int FW_LOOP_SPEED = 20;

 // Maximum power we want to send to the flywheel motors
 int FW_MAX_POWER  = 12000;

 // velocity measurement
 float           motor_velocity;         ///< current velocity in rpm

 // TBH control algorithm variables
 long            target_velocity;        ///< target_velocity velocity
 float           current_error;          ///< error between actual and target_velocity velocities
 float           last_error;             ///< error last time update called
 float           gain;                   ///< gain
 float           drive;                  ///< final drive out of TBH (0.0 to 1.0)
 float           drive_at_zero;          ///< drive at last zero crossing
 long            first_cross;            ///< flag indicating first zero crossing
 float           drive_approx;           ///< estimated open loop drive

 // final motor drive
 long            motor_drive;            ///< final motor control value

 /*-----------------------------------------------------------------------------*/
 /** @brief      Set the flywheen motors                                        */
 /** @param[in]  value motor control value                                      */
 /*-----------------------------------------------------------------------------*/
 inline void FwMotorSet( int value )
 {
     flywheel.move_voltage(value);
 }
 /*-----------------------------------------------------------------------------*/
 /** @brief      Set the controller position                                    */
 /** @param[in]  desired velocity                                               */
 /** @param[in]  predicted_drive estimated open loop motor drive                */
 /*-----------------------------------------------------------------------------*/
 inline void FwVelocitySet( int vel, float predicted_drive )
 {
   // set target_velocity velocity (motor rpm)
   target_velocity = vel;

   // Set error so zero crossing is correctly detected
   current_error = target_velocity - motor_velocity;
   last_error    = current_error;

   // Set predicted open loop drive value
   drive_approx  = predicted_drive;
   // Set flag to detect first zero crossing
   first_cross   = 1;
   // clear tbh variable
   drive_at_zero = 0;
 }

 /*-----------------------------------------------------------------------------*/
 /** @brief      Update the velocity tbh controller variables                   */
 /*-----------------------------------------------------------------------------*/
 inline void sgn(float x)
 {
   if (x == 0)
   {
       x = 0;
   }
   else if (x < 0)
   {
       x = -1;
   }
   else if (x > 0)
       x = 1;
 }
 inline void FwControlUpdateVelocityTbh()
 {
   // calculate error in velocity
   // target_velocity is desired velocity
   // current is measured velocity
   current_error = target_velocity - motor_velocity;

   // Calculate new control value
   drive =  drive + (current_error * gain);

   // Clip to the range 0 - 1.
   // We are only going forwards
   if( drive > 1 )
         drive = 1;
   if( drive < 0 )
         drive = 0;

   // Check for zero crossing
   //if( sgn(current_error) != sgn(last_error) )
   if((current_error && last_error > 0)||(current_error && last_error < 0)||(current_error && last_error == 0))
   {
       // First zero crossing after a new set velocity command
       if( first_cross ) {
           // Set drive to the open loop approximation
           drive = drive_approx;
           first_cross = 0;
       }
       else
           drive = 0.5 * ( drive + drive_at_zero );

       // Save this drive value in the "tbh" variable
       drive_at_zero = drive;
   }

   // Save last error
   last_error = current_error;
 }

 /*-----------------------------------------------------------------------------*/
 /** @brief     Task to control the velocity of the flywheel                    */
 /*-----------------------------------------------------------------------------*/
  inline void FwControlTask(void* param)
 {
   // Set the gain
   gain = 0.0011;   // Test with 0.0005  //0.002

   while(1)
       {
       // Calculate velocity
       motor_velocity = flywheel.getActualVelocity();

       // Do the velocity TBH calculations
       FwControlUpdateVelocityTbh() ;

       // Scale drive into the range the motors need
       motor_drive  = (drive * FW_MAX_POWER) + 0.5;

       // Final Limit of motor values - don't really need this
       if( motor_drive >  12000 ) motor_drive =  12000;
       if( motor_drive < -12000 ) motor_drive = -12000;

       // and finally set the motor control value
       FwMotorSet( motor_drive );

       // Run at somewhere between 20 and 50mS
       pros::Task::delay(FW_LOOP_SPEED);
       }
   }


inline pros::ADIDigitalOut piston(DRIVE_PNEUMATIC);         //Piston on DRIVE_PNEUMATIC port
inline pros::ADILineSensor linetrackerL(LINE_TRACKER_LEFT); //Line tracker on LINE_TRACKER_LEFT port
inline pros::ADILineSensor linetrackerR(LINE_TRACKER_RIGHT); //Line tracker on LINE_TRACKER_LEFT port
inline pros::ADIAnalogIn gyro (GYRO_PORT);
inline pros::ADIUltrasonic ultrasonic1(ULTRASONIC_IN, ULTRASONIC_OUT);


//#endif /* end of include guard:  */
