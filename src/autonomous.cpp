#include "main.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 *///
void autonomous() {

  // intake.move_voltage(12000);
  // pros::delay(1000);
  // intake.move_voltage(0);
  //pros::delay(500);
  // state != state;
  // sensor.set_value(state);
  //
  // rotatorController.setTarget(180_deg);     //flip intake
  // conveyorController.setTarget(200_rpm);    //move conveyor full speed
  // liftController.setTarget(50_deg);         //raise lift //TODO: measure distance per degree
  // driveController.moveDistanceAsync(1_m);   // Move 1 meter to the first goal
  //driveController.setMaxVoltage(800);
  driveController.setMaxVelocity(60);
  driveController.turnAngle(90_deg);        // Turn 90 degrees

  //pros::delay(1000) or pros::Task::delay(1000) idk which one
}
