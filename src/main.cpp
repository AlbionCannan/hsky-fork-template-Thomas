#include "main.h"

#include "robot_config.hpp"

// Field frame: origin at field center, +x right, +y away from your driver
// station, inches, heading in radians CCW from +x (so facing +y = pi/2).
constexpr double kDeg = loc::kPi / 180.0;

// Prints both poses to the PROS terminal (`pros terminal`) every ~100 ms.
// Compare them: if they drift apart while you drive, sensor correction is working.
static void printPose() {
  const loc::Pose p = localizer.getPose();
  const loc::Pose o = localizer.getOdomPose();
  printf("MCL x=%6.1f y=%6.1f th=%6.1f | odom x=%6.1f y=%6.1f\n",
         p.x, p.y, p.theta / kDeg, o.x, o.y);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 * Keep the robot still: the IMU calibrates here (~2 s).
 */
void initialize() { robotInit(); }

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // Your measured starting pose. Example: facing +y (pi/2).
  localizer.setPose({-60.0, -36.0, 90.0 * kDeg});

  // Motion code goes here; read localizer.getPose() for the robot's position.
}

void opcontrol() {
  opcontrolInit();

  controller.initialize();

  int tick = 0;
  while (true) {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      left_motors.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      left_motors.move(-127);
    } else {
      left_motors.move(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      right_motors.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      right_motors.move(-127);
    } else {
      right_motors.move(0);
    }

    // TESTING: place the robot at a known spot, press A to tell the localizer.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      localizer.setPose({0.0, 0.0, 90.0 * kDeg});
    }
    // TESTING: hold B to disable sensor correction (pure odometry) for A/B comparison.
    localizer.setSensorCorrection(!master.get_digital(pros::E_CONTROLLER_DIGITAL_B));

    if (++tick % 5 == 0) printPose();
    pros::delay(20);
  }
}