#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <vector>

#include "api.h"
#include "hskylib.h"

extern HskyController controller;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Controller master;

void opcontrolInit();
void robotInit();

#endif