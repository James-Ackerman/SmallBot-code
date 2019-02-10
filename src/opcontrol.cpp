#include "main.h"
//#include "okapi/api.hpp"
//#include "definitions.hpp"
//using namespace okapi; //

//TODO: check syntax for ControllerButton for R1/R2/L1/L2/Down/left/right buttons
ControllerButton RightBumperUP(ControllerDigital::R1);
ControllerButton RightBumperDOWN(ControllerDigital::R2);
ControllerButton LeftBumperUP(ControllerDigital::L1);
ControllerButton LeftBumperDOWN(ControllerDigital::L2);
ControllerButton ButtonA(ControllerDigital::A);
ControllerButton ButtonB(ControllerDigital::B);
ControllerButton ButtonX(ControllerDigital::X);
ControllerButton ButtonY(ControllerDigital::Y);
ControllerButton ButtonUP(ControllerDigital::up);
ControllerButton ButtonDOWN(ControllerDigital::down);
ControllerButton ButtonLEFT(ControllerDigital::left);
ControllerButton ButtonRIGHT(ControllerDigital::right);
////////////////////////////////////OPCONTROL///////////////////////////////////////////////////

void opcontrol() {

  Controller controller;
  bool STATE = LOW;                        //For Pneumatics //Change to false if this doesn't not work
  int FLYWHEEL_STATE = 1;
  int FLYWHEEL_TARGET = 0;
  int FLYWHEEL_LOW = 400;
  int FLYWHEEL_MID = 500;
  int FLYWHEEL_HIGH = 600;
  linetrackerL.calibrate();
  linetrackerR.calibrate();
  gyro.calibrate();

  pros::Task FwControl(FwControlTask);

  //auto flyController = AsyncControllerFactory::velPID(FLYWHEEL_MOTOR, 1.0, 0.01, 0.1);
  flywheel.setGearing(AbstractMotor::gearset::blue);
  while (true)
	{
     // pros::lcd::initialize();
     // pros::lcd::print(0, "Joystick valY: %d", controller.getAnalog(ControllerAnalog::leftY));
     // pros::lcd::print(0, "Joystick valX: %d", controller.getAnalog(ControllerAnalog::leftX));
		 //////////////////////////////CHASSIS(DRIVE)/////////////////////////////////

     driveController.arcade(controller.getAnalog(ControllerAnalog::leftY), -controller.getAnalog(ControllerAnalog::rightX));

     //////////////////////////////TRANSMISSION/////////////////////////////////
     if (ButtonB.changedToPressed() && (abs(controller.getAnalog(ControllerAnalog::leftY) > 0.4)))//
     {
         STATE = !STATE;
         piston.set_value(STATE);
     }
     /////////////////////////////////INTAKE////////////////////////////////////
     if (RightBumperDOWN.isPressed())          //Hold button to Chupa Intake
     {
         intake.move_voltage(12000);
         intake.move_voltage(12000);
     }
     else if (RightBumperUP.isPressed())  //Hold button to Bota Intake
     {
         intake.move_voltage(-12000);
         intake.move_voltage(-12000);
     }
     else                                   //Intake stops if nothing is pressed
     {
         intake.move_voltage(0);
         intake.move_voltage(0);
     }
     ///////////////////////////////INDEXER//////////////////////////////////////
     if (LeftBumperDOWN.isPressed())
     {
         indexer.move_voltage(12000);
     }
     else if (LeftBumperUP.isPressed())
     {
         indexer.move_voltage(-12000);
     }
     else
     {
         indexer.move_voltage(0);
     }
     /////////////////////////////FLYWHEEL////////////////////////////////
      if (ButtonDOWN.changedToPressed())             //Press button to flip intake
      {
              FwVelocitySet(400, 1);
      }

      else if (ButtonRIGHT.changedToPressed())         //Flywheel MID conditions
      {
              FwVelocitySet(500, 1);
      }

      else if (ButtonUP.changedToPressed())             //Flywheel HIGH conditions
      {
              FwVelocitySet(600, 1);
      }

      else if (ButtonLEFT.changedToPressed())
      {
              FwVelocitySet(0, 1);
      }

		 /////////////////////////////////DESCORER//////////////////////////////////////


      // if (descorer.getPosition() > 50)
      //   {
      //      descorer.move_voltage(-500);
      //    }
      // else if
      // else
      // {
        descorer.move_voltage(12000*controller.getAnalog(ControllerAnalog::rightY));
      //}

		 pros::Task::delay(20);
	 }
}
