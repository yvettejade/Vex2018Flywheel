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
 */

 pros::Controller master_auton(pros::E_CONTROLLER_MASTER);
 pros::Motor leftFront_auton(20, true);
 pros::Motor rightFront_auton(11);
 pros::Motor leftBack_auton(9, true);
 pros::Motor rightBack_auton(19);
 pros::Motor fly_auton(6, true);
 pros::Motor flyIntake_auton(16, true);
 //check the ports
 pros::Motor lift_auton(15, true);

 bool firstAuton=true;

 void flywheelTaskAuton(void* param)
 {
   while(true)
   {
     if(firstAuton==true)
     {
         double vel=2;
   			fly_auton.move_velocity(2);
   			pros::delay(100);
         while(vel<200)
         {
             fly_auton.move_velocity(vel*vel);
   					vel=fly_auton.get_actual_velocity();
         }
         firstAuton=false;
     }
     fly_auton.move_velocity(200);
     pros::delay(2);
   }
 }
 void driveOneSquare(double squares, bool forward)
{
    if(forward==false)
    {
      leftFront_auton.move_velocity(-200);
      leftBack_auton.move_velocity(-200);
      rightFront_auton.move_velocity(-200);
      rightBack_auton.move_velocity(-200);
      pros::delay(squares*600);
    }
    else
    {
      leftFront_auton.move_velocity(200);
      leftBack_auton.move_velocity(200);
      rightFront_auton.move_velocity(200);
      rightBack_auton.move_velocity(200);
      pros::delay(squares*600);
    }
    leftFront_auton.move_voltage(0);
    leftBack_auton.move_voltage(0);
    rightFront_auton.move_voltage(0);
    rightBack_auton.move_voltage(0);
}
void encoderRightTurn()
{
  leftFront_auton.tare_position();
  rightFront_auton.tare_position();
  leftBack_auton.tare_position();
  rightBack_auton.tare_position();
  leftFront_auton.move_absolute(1150,200);
  rightFront_auton.move_absolute(-1150,200);
  leftBack_auton.move_absolute(1150,200);
  rightBack_auton.move_absolute(-1150,200);
  pros::delay(400);
}
void encoderLeftTurn()
{
  leftFront_auton.tare_position();
  rightFront_auton.tare_position();
  leftBack_auton.tare_position();
  rightBack_auton.tare_position();
  leftFront_auton.move_absolute(-1200,200);
  rightFront_auton.move_absolute(1200,200);
  leftBack_auton.move_absolute(-1200,200);
  rightBack_auton.move_absolute(1200,200);
  pros::delay(400);
}
void autonomous()
{
  //kick
  fly_auton.move_velocity(5);
  pros::delay(100);

  //start flywheel
  pros::Task f (flywheelTaskAuton, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel");

  //drive towards cap
  driveOneSquare(2, true);

  //intake preload and start driving towards cap and intake ball
  flyIntake_auton.move_velocity(200);
  driveOneSquare(0.25,true);
  pros::delay(700);
  flyIntake_auton.move_voltage(0);

  //back up
  driveOneSquare(2.5, false);
  pros::delay(400);

  //turn to face flags
  encoderRightTurn();
  pros::delay(500);

  //shoot top flag
  flyIntake_auton.move_velocity(200);
  pros::delay(600);
  flyIntake_auton.move_voltage(0);

  //move to shoot middle flag
  driveOneSquare(1,true);

  //shoot middle flag
  flyIntake_auton.move_velocity(200);
  pros::delay(500);
  flyIntake_auton.move_voltage(0);

  //hit low flag
  driveOneSquare(1.5,true);
  pros::delay(500);

  //back up
  driveOneSquare(1.25,false);
  pros::delay(400);

  //turn to face cap
  encoderLeftTurn();
  pros::delay(500);

  //flip cap
  flyIntake_auton.move_velocity(-200);
  driveOneSquare(1.75, true);
  pros::delay(1000);
  flyIntake_auton.move_voltage(0);

  //turn to face platform
  encoderLeftTurn();
  pros::delay(500);

  //climb platform
  driveOneSquare(3.25,true);
}
