#include "main.h"
using namespace std;

pros::Controller master(pros::E_CONTROLLER_MASTER);
//redo motor ports
pros::Motor leftFront(20, true);
pros::Motor rightFront(11);
pros::Motor leftBack(9, true);
pros::Motor rightBack(19);
pros::Motor fly(6, true);
pros::Motor flyIntake(16, true);
//check the ports
pros::Motor lift(15, true);

pros::Vision vision(4);
pros::vision_signature_s_t FLAG =
    pros::Vision::signature_from_utility(1, -3209, -2567, -2888, -5183, -3991, -4586, 3.5, 0);

bool origin=true;
bool first=true;
double currentDist=0, ang=0;

void driveTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_X))
    {
      leftFront.move(-1*master.get_analog(ANALOG_RIGHT_Y));
			leftBack.move(-1*master.get_analog(ANALOG_RIGHT_Y));
			rightFront.move(-1*master.get_analog(ANALOG_LEFT_Y));
			rightBack.move(-1*master.get_analog(ANALOG_LEFT_Y));
    }
    else
    {

      leftFront.move(master.get_analog(ANALOG_LEFT_Y));
			leftBack.move(master.get_analog(ANALOG_LEFT_Y));
			rightFront.move(master.get_analog(ANALOG_RIGHT_Y));
			rightBack.move(master.get_analog(ANALOG_RIGHT_Y));

    }
    pros::delay(2);
  }
}

void brakeTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_UP))
    {
      master.print(0,0,"%s","coast");
      leftFront.set_brake_mode(MOTOR_BRAKE_COAST);
      leftBack.set_brake_mode(MOTOR_BRAKE_COAST);
      rightFront.set_brake_mode(MOTOR_BRAKE_COAST);
      rightBack.set_brake_mode(MOTOR_BRAKE_COAST);
    }
    else if(master.get_digital(DIGITAL_DOWN))
    {
      master.print(0,0,"%s","holding");
      leftFront.set_brake_mode(MOTOR_BRAKE_HOLD);
      leftBack.set_brake_mode(MOTOR_BRAKE_HOLD);
      rightFront.set_brake_mode(MOTOR_BRAKE_HOLD);
      rightBack.set_brake_mode(MOTOR_BRAKE_HOLD);
    }
    pros::delay(2);
  }
}

void intakeTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_R1))
    {
      flyIntake.move_velocity(200);
			//flyIntake.move(127);
    }
    else if(master.get_digital(DIGITAL_R2))
    {
      flyIntake.move_velocity(-200);
      //flyIntake.move(-127);
    }
    else
    {
      flyIntake.move_voltage(0);
    }
    pros::delay(2);
  }
}

void flywheelTask(void* param)
{
  while(true)
  {
    if(first==true)
    {
        double vel=2;
  			fly.move_velocity(2);
  			pros::delay(100);
        while(vel<200)
        {
            fly.move_velocity(vel*vel);
  					vel=fly.get_actual_velocity();
        }
        first=false;
    }
    fly.move_velocity(200);
    pros::delay(2);
  }
}

//LIFT
void liftTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_L1))
    {
      lift.move_velocity(200);
    }
    else if(master.get_digital(DIGITAL_L2))
    {
      lift.move_velocity(-200);
    }
    else
    {
      lift.move_voltage(0);
    }
    pros::delay(2);
  }
}

//FLIPPER
/*void flipperTask(void* param)
{
  while(true)
  {
    if(master.get_digital(DIGITAL_B))
    {
      flipper.tare_position();
      //check how much the flipper has to move
      flipper.move_absolute(50, 100);
    }
    else
    {
      flipper.move_voltage(0);
    }
    pros::delay(2);
  }
}*/

//VISION SENSING
//FIND THE DISTANCE AND THE ANGLE OF THE FLAG
void findDist()
{
  //master.clear();
  //check the angle

  if(vision.get_object_count()>0)
  {
    pros::vision_object_s_t FOUND=vision.get_by_sig(0,1);
    //master.clear();
    //check the angle
    //master.print(0, 0, "%s","here");
    currentDist=FOUND.width;
    ang=FOUND.left_coord;
    pros::delay(1000);
  }
  else
  {
    master.print(0, 0, "%s"," not here");
    master.clear();
  }
}
//MOVE THE ROBOT TO THE IDEAL POSITION
void moveFwd(double ideal)
{
  master.print(0,0,"%s", "move1");
  master.print(0, 6, "%d", currentDist);
  while(currentDist-ideal>=2)
  {
    leftFront.move_velocity(-50);
    rightFront.move_velocity(-50);
    leftBack.move_velocity(-50);
    rightBack.move_velocity(-50);
    pros::delay(2);
    findDist();
  }
  return;
}
void moveBkd(double ideal)
{
  /*master.print(0,0,"%s", "move");
  master.print(0, 6, "%d", currentDist);*/
  while(ideal-currentDist>=2)
  {
    leftFront.move_velocity(50);
    rightFront.move_velocity(50);
    leftBack.move_velocity(50);
    rightBack.move_velocity(50);
    pros::delay(2);
    findDist();
  }
  return;
}
//STRAIGHTEN IT TO FACE THE FLAG
void straighten()
{
  //check the angle
  if(ang>130&&ang<160)
    return;
  while(ang<=130||ang>=700)
  {
    rightFront.move_velocity(100);
    rightBack.move_velocity(100);
    pros::delay(50);
    findDist();
  }
  while(ang>=160&&ang<700)
  {
    leftFront.move_velocity(100);
    leftFront.move_velocity(100);
    pros::delay(50);
    findDist();
  }
}
void opcontrol()
{
  fly.move_velocity(5);
  pros::delay(100);

  //TASKS
  pros::Task f (flywheelTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "flywheel");

  pros::Task i (intakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "intake");

  pros::Task d (driveTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "drive");

  pros::Task b (brakeTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "braking");

  pros::Task l (liftTask, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "lift");

	while(true)
	{

    lift.set_brake_mode(MOTOR_BRAKE_BRAKE);
		pros::delay(2);
	}
}
