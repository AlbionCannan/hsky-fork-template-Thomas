// localizer.hpp — runs odometry + MCL in a background task and publishes the pose.
#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "localization/mcl.hpp"
#include "localization/odometry.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

namespace loc {

struct DistanceSensorBinding {
  pros::Distance* sensor;
  DistanceSensorConfig mount;
};

struct LocalizerConfig {
  OdomConfig odom;
  MCLConfig mcl;
  int loopMs = 10;                   // odometry + predict rate
  int sensorEveryN = 3;              // sensor update every N loops (~30 ms)
  double maxTurnRateForUpdate = 3.0; // rad/s; skip sensor updates while spinning fast
  double initSpread = 1.5;           // in, particle spread when the pose is set
};

class Localizer {
 public:
  // vertInches / horzInches return CUMULATIVE wheel travel in inches.
  // Pass horzInches = nullptr if you have no horizontal tracking wheel.
  Localizer(const LocalizerConfig& cfg, pros::Imu& imu,
            std::function<double()> vertInches, std::function<double()> horzInches,
            std::vector<DistanceSensorBinding> sensors);

  void begin();                 // starts the task; call once, AFTER the IMU is calibrated
  void setPose(const Pose& p);  // field frame, radians CCW from +x
  Pose getPose();               // filtered pose (what controllers should use)
  Pose getOdomPose();           // raw odometry, for comparing/debugging
  void setSensorCorrection(bool on);  // false = pure odometry (useful for A/B testing)

 private:
  void loop();
  void step(bool doSensors);
  double imuRadians();          // raw IMU rotation converted to CCW radians

  LocalizerConfig cfg_;
  pros::Imu& imu_;
  std::function<double()> vert_, horz_;
  std::vector<DistanceSensorBinding> sensors_;
  MCL mcl_;

  pros::Mutex mutex_;
  std::unique_ptr<pros::Task> task_;

  std::vector<Reading> readings_;
  Pose odomPose_;
  double headingOffset_ = 0.0;
  double lastImuRad_ = 0.0;
  double lastTheta_ = 0.0, lastVert_ = 0.0, lastHorz_ = 0.0;
  bool sensorCorrection_ = true;
};

}  // namespace loc