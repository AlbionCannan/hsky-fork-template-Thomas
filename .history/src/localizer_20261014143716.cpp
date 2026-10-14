#include "localization/localizer.hpp"

#include <cmath>
#include <mutex>

namespace loc {

namespace {
std::vector<DistanceSensorConfig> mounts(const std::vector<DistanceSensorBinding>& s) {
  std::vector<DistanceSensorConfig> out;
  for (const auto& b : s) out.push_back(b.mount);
  return out;
}
}  // namespace

Localizer::Localizer(const LocalizerConfig& cfg, pros::Imu& imu,
                     std::function<double()> vertInches, std::function<double()> horzInches,
                     std::vector<DistanceSensorBinding> sensors)
    : cfg_(cfg), imu_(imu), vert_(std::move(vertInches)), horz_(std::move(horzInches)),
      sensors_(std::move(sensors)), mcl_(cfg.mcl, mounts(sensors_)) {}
// Note: mcl_ is declared after sensors_ in the class, so sensors_ is already
// initialized when mounts(sensors_) runs.

double Localizer::imuRadians() {
  // V5 IMU rotation is in degrees, CLOCKWISE positive. Convert to CCW radians.
  const double deg = imu_.get_rotation();
  if (std::isfinite(deg)) lastImuRad_ = -deg * kPi / 180.0;
  return lastImuRad_;  // on error, hold the last good value
}

void Localizer::begin() {
  if (task_) return;
  setPose({0, 0, 0});
  task_ = std::make_unique<pros::Task>([this] { loop(); }, "localizer");
}

void Localizer::setPose(const Pose& p) {
  std::lock_guard<pros::Mutex> lock(mutex_);
  headingOffset_ = p.theta - imuRadians();
  lastTheta_ = p.theta;
  lastVert_ = vert_();
  lastHorz_ = horz_ ? horz_() : 0.0;
  odomPose_ = p;
  mcl_.init(p.x, p.y, cfg_.initSpread);
}

Pose Localizer::getPose() {
  std::lock_guard<pros::Mutex> lock(mutex_);
  return mcl_.estimate(lastTheta_);
}

Pose Localizer::getOdomPose() {
  std::lock_guard<pros::Mutex> lock(mutex_);
  return odomPose_;
}

void Localizer::setSensorCorrection(bool on) {
  std::lock_guard<pros::Mutex> lock(mutex_);
  sensorCorrection_ = on;
}

void Localizer::loop() {
  std::uint32_t t = pros::millis();
  int tick = 0;
  while (true) {
    const bool doSensors = (++tick % cfg_.sensorEveryN) == 0;
    {
      std::lock_guard<pros::Mutex> lock(mutex_);
      step(doSensors);
    }
    pros::Task::delay_until(&t, cfg_.loopMs);
  }
}

void Localizer::step(bool doSensors) {
  const double v = vert_();
  const double h = horz_ ? horz_() : 0.0;
  const double theta = headingOffset_ + imuRadians();

  const Delta d = odomStep(cfg_.odom, v - lastVert_, h - lastHorz_, lastTheta_, theta);
  const double turnRate = std::fabs(theta - lastTheta_) / (cfg_.loopMs / 1000.0);
  lastVert_ = v;
  lastHorz_ = h;
  lastTheta_ = theta;

  odomPose_.x += d.dx;
  odomPose_.y += d.dy;
  odomPose_.theta = theta;
  mcl_.predict(d.dx, d.dy);

  if (!doSensors || !sensorCorrection_ || sensors_.empty()) return;
  if (turnRate > cfg_.maxTurnRateForUpdate) return;  // readings smear while spinning

  readings_.clear();
  for (const auto& b : sensors_) {
    const std::int32_t mm = b.sensor->get();  // 9999 = nothing seen; PROS_ERR on error
    readings_.push_back({mm > 20 && mm < 2000, mm / 25.4});
  }
  mcl_.update(readings_, theta);
}

}  // namespace loc