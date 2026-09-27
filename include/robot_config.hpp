#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

#include <vector>

#include "api.h"
#include "hskylib.h"

extern HskyController controller;


// delcaring the mtor groups and the controller
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Controller master;

// motor direction
pros::MotorGroup left_motors({1, 2});
pros::MotorGroup right_motors({-3, -4});

pros::Controller master(pros::E_CONTROLLER_MASTER);


void opcontrolInit();
void robotInit();

#endif