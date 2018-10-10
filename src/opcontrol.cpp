#include "main.h"

using namespace okapi;

auto drive = ChassisControllerFactory::create(-18,
                                              19,
                                              {},
                                              {},
                                              AbstractMotor::gearset::green,
                                              {4.125_in, 10.5_in});

void opcontrol() {
  Logger::initialize(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug);
  printf("start\n");
  drive.getGearsetRatioPair();
  pros::delay(50);
  printf("done\n");
}
