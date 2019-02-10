#include "definitions.hpp"

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
long            motor_drive;            ///< final motor control value//


void FwMotorSet( int value )
{
    flywheel.move_voltage(value);
}
/*-----------------------------------------------------------------------------*/
/** @brief      Set the controller position                                    */
/** @param[in]  desired velocity                                               */
/** @param[in]  predicted_drive estimated open loop motor drive                */
/*-----------------------------------------------------------------------------*/
void FwVelocitySet( int vel, float predicted_drive )
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
void sgn(float x)
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
void FwControlUpdateVelocityTbh()
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
 void FwControlTask(void* param)
{
  // Set the gain
  gain = 0.0012;   // Test with 0.0005  //0.002

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


//#endif /* end of include guard:  */
