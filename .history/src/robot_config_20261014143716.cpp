#include "robot_config.hpp"

#include <hskylib/robot_specs.h>
#include <hskylib/subsystems/pneumatics.h>
#include <hskylib/utils/utils.h>

#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"

#define ROBOT_1

HskyController controller(pros::E_CONTROLLER_MASTER);
pros::Controller master(pros::E_CONTROLLER_MASTER);  // main.cpp reads buttons from this

namespace {
// Cumulative travel of a tracking wheel on a rotation sensor, in inches.
// Holds the last good value if the sensor returns an error.
std::function<double()> trackingWheel(pros::Rotation& sensor, double diameterIn) {
  return [&sensor, diameterIn, last = 0.0]() mutable {
    const std::int32_t centideg = sensor.get_position();
    if (centideg != PROS_ERR) last = centideg / 36000.0 * loc::kPi * diameterIn;
    return last;
  };
}
}  // namespace

//---------------------------------------------------
// ##################### Robot 1 #####################
//---------------------------------------------------

#ifdef ROBOT_1

//===================== CONFIG =====================
// ALL PORTS AND MEASUREMENTS BELOW ARE PLACEHOLDERS. Replace with your robot's.

constexpr double kTrackingWheelDiameter = 2.0;  // in (measure; 2" omni is ~2.0-2.05")

loc::LocalizerConfig makeLocalizerConfig() {
  loc::LocalizerConfig c;
  // Tracking-wheel positions relative to the TRACKING CENTER (the point whose
  // pose you want; usually the robot's center of rotation).
  c.odom.vertLeftOffset = 0.0;  // in, + = vertical wheel is LEFT of center
  c.odom.horzFwdOffset = -2.0;  // in, + = horizontal wheel is FORWARD of center
  c.mcl.fieldHalf = 70.2;       // in, half the inside wall-to-wall width. MEASURE.
  return c;
}

//===================== DEVICES =====================

// Negative port = reversed motor.
pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 5, 6}, pros::MotorGearset::blue);

pros::Imu imu(10);
pros::Rotation vertTracker(11);  // + must mean "robot moved forward"
pros::Rotation horzTracker(12);  // + must mean "robot moved LEFT"
pros::Distance distLeft(13);
pros::Distance distRight(14);
pros::Distance distBack(15);

//==================== SUBSYSTEMS ====================

loc::Localizer localizer(
    makeLocalizerConfig(), imu,
    trackingWheel(vertTracker, kTrackingWheelDiameter),
    trackingWheel(horzTracker, kTrackingWheelDiameter),  // nullptr if no horizontal wheel
    {
        // sensor, {fwd, left, angle}: measured from tracking center to sensor FACE.
        // angle: 0 = forward, +pi/2 = left, -pi/2 = right, pi = back.
        {&distLeft, {0.0, 6.5, loc::kPi / 2}},
        {&distRight, {0.0, -6.5, -loc::kPi / 2}},
        {&distBack, {-7.0, 0.0, loc::kPi}},
    });

// Set true for each tracker that counts the wrong way (see sign check in main.cpp).
constexpr bool kVertReversed = false;
constexpr bool kHorzReversed = false;

//---------------------------------------------------
// ##################### Robot 2 #####################
//---------------------------------------------------

#elifdef ROBOT_2

//===================== CONFIG =====================
// Copy Robot 1's CONFIG / DEVICES / SUBSYSTEMS blocks here and change values.
//===================== DEVICES =====================

//==================== SUBSYSTEMS ====================

#endif

//====================== UTILS ======================

void deviceInit() {
  vertTracker.set_reversed(kVertReversed);
  horzTracker.set_reversed(kHorzReversed);
  vertTracker.reset_position();
  horzTracker.reset_position();

  // Robot must be STILL for ~2 s while the IMU calibrates.
  imu.reset(true);
}

void opcontrolInit() {}

void robotInit() {
  deviceInit();
  localizer.begin();  // starts at (0, 0, 0); autonomous() sets the real start pose
}