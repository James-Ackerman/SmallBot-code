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
  pros::Task FwControl(FwControlTask);
  flywheel.setGearing(AbstractMotor::gearset::blue);
  FwVelocitySet(505, 1);
  FwVelocitySet(505, 1);
  pros::delay(500);
  intake.moveVoltage(-12000);
  indexer.moveVoltage(2000);
  pros::delay(3500);
  //Tira dos bolitas
  while ((abs(flywheel.getActualVelocity())) < 500 || (abs(flywheel.getActualVelocity()) > 510))
  {
  };

  indexer.moveVoltage(-12000);
  pros::delay(145);
  indexer.moveVoltage(0);
  intake.moveVoltage(-12000);
  FwVelocitySet(455, 1);
  pros::delay(1000);
  intake.moveVoltage(0);
  while ((abs(flywheel.getActualVelocity())) < 450 || (abs(flywheel.getActualVelocity()) > 460));

  indexer.moveVoltage(-12000);
  intake.moveVoltage(-12000);
  pros::delay(400);
  indexer.moveVoltage(0);
  intake.moveVoltage(0);
  //Tiro las dos bolitas
  pros::delay(150);
  //driveController.setMaxVelocity(60);       //Slow down bot
  driveController.moveDistance(-0.4_ft);      //Echa para atras para acomodarse con la horizontal del cap
  intake.moveVoltage(-12000);
  pros::delay(150);
  driveController.turnAngle(90_deg);        // Turn 90 degrees to face back front cap
  pros::delay(50);
  driveController.moveDistance(3_ft);
  pros::delay(100);
  intake.moveVoltage(0);                    //intakes ball under back front cap

  pros::delay(4000);                          //Wait for bigbot crossing

  driveController.moveDistance(-3_ft);      //returns to tile
  pros::delay(50);
  driveController.turnAngle(90_deg);        // Turn 90 degrees so flywheel faces backwards
  pros::delay(50);
  driveController.moveDistance(-4_ft);       //move backwards until in front tile
  pros::delay(50);
  driveController.turnAngle(-91_deg);        // Turn 90 degrees so iintake faces front back cap
  pros::delay(50);

  intake.moveVoltage(-6000);
  indexer.moveVoltage(2000);
  FwVelocitySet(460, 1);
  driveController.moveDistance(3.5_ft);
  intake.moveVoltage(-12000);
  indexer.moveVoltage(2000);
  driveController.moveDistance(-1.50_ft);
  driveController.turnAngle(-60_deg);        // Turn 90 degrees

  //Tira 2 Bolitas
  while ((abs(flywheel.getActualVelocity())) < 455 || (abs(flywheel.getActualVelocity()) > 465))
  {};

  indexer.moveVoltage(-12000);
  pros::delay(145);
  indexer.moveVoltage(0);
  intake.moveVoltage(-12000);
  FwVelocitySet(400, 1);
  pros::delay(1000);
  intake.moveVoltage(0);
  while ((abs(flywheel.getActualVelocity())) < 395 || (abs(flywheel.getActualVelocity()) > 405));

  indexer.moveVoltage(-12000);
  intake.moveVoltage(-12000);
  pros::delay(400);
  indexer.moveVoltage(0);
  intake.moveVoltage(0);
  driveController.turnAngle(-10_deg);
  driveController.moveDistance(4.5_ft);
  pros::delay(100);
  driveController.moveDistance(-2.5_ft);
  pros::delay(100);
  driveController.turnAngle(-30_deg);
  driveController.moveDistance(-5_ft);








  // while (abs(flywheel.getActualVelocity()) <= 400);
  // indexer.moveVoltage(-12000);
  // pros::delay(100);
  // indexer.moveVoltage(0);

  //FwVelocitySet(0,1);



  //pros::delay(500);
  // state != state;
  // sensor.set_value(state);
  //
  // rotatorController.setTarget(180_deg);     //flip intake
  // conveyorController.setTarget(200_rpm);    //move conveyor full speed
  // liftController.setTarget(50_deg);         //raise lift //TODO: measure distance per degree
  // driveController.moveDistanceAsync(1_m);   // Move 1 meter to the first goal
  //driveController.setMaxVoltage(800);
  //driveController.setMaxVelocity(60);
  //driveController.turnAngle(90_deg);        // Turn 90 degrees

  //pros::delay(1000) or pros::Task::delay(1000) idk which one
}
