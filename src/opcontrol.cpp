#include "main.h"

using namespace okapi;

void foo(void *) {
  auto drive = ChassisControllerFactory::create(
    -18, 19, {}, {}, AbstractMotor::gearset::green, {4.125_in, 10.5_in});
  drive.getGearsetRatioPair();
  pros::delay(50);
}

void opcontrol() {
  //  auto t1 = pros::c::task_create(foo, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
  //  ""); pros::delay(20); pros::c::task_delete(t1);
  auto drive = ChassisControllerFactory::create(
    -18, 19, {}, {}, AbstractMotor::gearset::green, {4.125_in, 10.5_in});
  drive.getGearsetRatioPair();
  pros::delay(50);
}
