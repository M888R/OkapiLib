/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <numeric>

namespace okapi {
AsyncMotionProfileController::AsyncMotionProfileController(const TimeUtil &itimeUtil,
                                                           const double imaxVel,
                                                           const double imaxAccel,
                                                           const double imaxJerk,
                                                           std::shared_ptr<ChassisModel> imodel,
                                                           const ChassisScales &iscales,
                                                           AbstractMotor::GearsetRatioPair ipair)
  : members(std::make_shared<members_s>(Logger::instance(),
                                        imaxVel,
                                        imaxAccel,
                                        imaxJerk,
                                        imodel,
                                        iscales,
                                        ipair,
                                        itimeUtil,
                                        this)) {
  if (ipair.ratio == 0) {
    members->logger->error(
      "AsyncMotionProfileController: The gear ratio cannot be zero! Check if you are "
      "using integer division.");
    throw std::invalid_argument(
      "AsyncMotionProfileController: The gear ratio cannot be zero! Check "
      "if you are using integer division.");
  }
}

AsyncMotionProfileController::AsyncMotionProfileController(
  AsyncMotionProfileController &&other) noexcept
  : members(std::move(other.members)) {
}

AsyncMotionProfileController::~AsyncMotionProfileController() {
  members->dtorCalled.store(true, std::memory_order_release);

  for (auto &path : members->paths) {
    free(path.second.left);
    free(path.second.right);
  }

  delete members->task;
}

void AsyncMotionProfileController::generatePath(std::initializer_list<Point> iwaypoints,
                                                const std::string &ipathId) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    members->logger->warn(
      "AsyncMotionProfileController: Not generating a path because no waypoints were given.");
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(
      Waypoint{point.x.convert(meter), point.y.convert(meter), point.theta.convert(radian)});
  }

  TrajectoryCandidate candidate;
  members->logger->info("AsyncMotionProfileController: Preparing trajectory");
  pathfinder_prepare(points.data(),
                     static_cast<int>(points.size()),
                     FIT_HERMITE_CUBIC,
                     PATHFINDER_SAMPLES_FAST,
                     0.001,
                     members->maxVel,
                     members->maxAccel,
                     members->maxJerk,
                     &candidate);

  const int length = candidate.length;

  if (length < 0) {
    auto pointToString = [](Waypoint point) {
      return "Point{x = " + std::to_string(point.x) + ", y = " + std::to_string(point.y) +
             ", theta = " + std::to_string(point.angle) + "}";
    };

    std::string message =
      "AsyncMotionProfileController: Path is impossible with waypoints: " +
      std::accumulate(std::next(points.begin()),
                      points.end(),
                      pointToString(points.at(0)),
                      [&](std::string a, Waypoint b) { return a + ", " + pointToString(b); });

    members->logger->error(message);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    throw std::runtime_error(message);
  }

  auto *trajectory = static_cast<Segment *>(malloc(length * sizeof(Segment)));

  if (trajectory == nullptr) {
    std::string message = "AsyncMotionProfileController: Could not allocate trajectory. The path "
                          "is probably impossible.";
    members->logger->error(message);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    throw std::runtime_error(message);
  }

  members->logger->info("AsyncMotionProfileController: Generating path");
  pathfinder_generate(&candidate, trajectory);

  auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * length);
  auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * length);

  if (leftTrajectory == nullptr || rightTrajectory == nullptr) {
    std::string message = "AsyncMotionProfileController: Could not allocate left and/or right "
                          "trajectories. The path is probably impossible.";
    members->logger->error(message);

    if (leftTrajectory) {
      free(leftTrajectory);
    }

    if (rightTrajectory) {
      free(rightTrajectory);
    }

    if (trajectory) {
      free(trajectory);
    }

    throw std::runtime_error(message);
  }

  members->logger->info("AsyncMotionProfileController: Modifying for tank drive");
  pathfinder_modify_tank(trajectory,
                         length,
                         leftTrajectory,
                         rightTrajectory,
                         members->scales.wheelbaseWidth.convert(meter));

  free(trajectory);

  // Free the old path before overwriting it
  removePath(ipathId);

  members->paths.emplace(ipathId, TrajectoryPair{leftTrajectory, rightTrajectory, length});
  members->logger->info("AsyncMotionProfileController: Completely done generating path");
  members->logger->info("AsyncMotionProfileController: " + std::to_string(length));
}

void AsyncMotionProfileController::removePath(const std::string &ipathId) {
  auto oldPath = members->paths.find(ipathId);
  if (oldPath != members->paths.end()) {
    free(oldPath->second.left);
    free(oldPath->second.right);
    members->paths.erase(ipathId);
  }
}

std::vector<std::string> AsyncMotionProfileController::getPaths() {
  std::vector<std::string> keys;

  for (const auto &path : members->paths) {
    keys.push_back(path.first);
  }

  return keys;
}

void AsyncMotionProfileController::setTarget(std::string ipathId) {
  members->currentPath = ipathId;
  members->isRunning = true;
}

void AsyncMotionProfileController::controllerSet(std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncMotionProfileController::getTarget() {
  return members->currentPath;
}

void AsyncMotionProfileController::loop() {
  auto rate = members->timeUtil.getRate();

  while (!members->dtorCalled.load(std::memory_order_acquire)) {
    if (members->self == nullptr) {
      /**
       * self will be null if task which created the parent object was deleted and the idle task
       * freed its stack. For example, when the robot is running during opcontrol and is suddenly
       * disabled.
       */
      return;
    }

    if (members->isRunning.load(std::memory_order_acquire) && !isDisabled()) {
      members->logger->info("AsyncMotionProfileController: Running with path: " +
                            members->currentPath);
      auto path = members->paths.find(members->currentPath);

      if (path == members->paths.end()) {
        members->logger->warn(
          "AsyncMotionProfileController: Target was set to non-existent path with name: " +
          members->currentPath);
      } else {
        members->logger->debug("AsyncMotionProfileController: Path length is " +
                               std::to_string(path->second.length));

        executeSinglePath(path->second, members->timeUtil.getRate());
        members->model->stop();

        members->logger->info("AsyncMotionProfileController: Done moving");
      }

      members->isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                     std::unique_ptr<AbstractRate> rate) {
  for (int i = 0; i < path.length && !isDisabled(); ++i) {
    const auto leftRPM = convertLinearToRotational(path.left[i].velocity * mps).convert(rpm);
    const auto rightRPM = convertLinearToRotational(path.right[i].velocity * mps).convert(rpm);

    members->model->left(leftRPM / toUnderlyingType(members->pair.internalGearset));
    members->model->right(rightRPM / toUnderlyingType(members->pair.internalGearset));

    rate->delayUntil(1_ms);
  }
}

QAngularSpeed AsyncMotionProfileController::convertLinearToRotational(QSpeed linear) const {
  return (linear * (360_deg / (members->scales.wheelDiameter * 1_pi))) * members->pair.ratio;
}

void AsyncMotionProfileController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncMotionProfileController *>(context)->loop();
  }
}

void AsyncMotionProfileController::waitUntilSettled() {
  members->logger->info("AsyncMotionProfileController: Waiting to settle");

  auto rate = members->timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  members->logger->info("AsyncMotionProfileController: Done waiting to settle");
}

Point AsyncMotionProfileController::getError() const {
  return Point{0_m, 0_m, 0_deg};
}

bool AsyncMotionProfileController::isSettled() {
  return isDisabled() || !members->isRunning.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  auto rate = members->timeUtil.getRate();
  while (members->isRunning.load(std::memory_order_acquire)) {
    rate->delayUntil(1_ms);
  }

  flipDisable(false);
}

void AsyncMotionProfileController::flipDisable() {
  flipDisable(!members->disabled.load(std::memory_order_acquire));
}

void AsyncMotionProfileController::flipDisable(const bool iisDisabled) {
  members->logger->info("AsyncMotionProfileController: flipDisable " + std::to_string(iisDisabled));
  members->disabled.store(iisDisabled, std::memory_order_release);
  // loop() will stop the chassis when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncMotionProfileController::isDisabled() const {
  return members->disabled.load(std::memory_order_acquire);
}

void AsyncMotionProfileController::startThread() {
  if (!members->task) {
    members->task = new CrossplatformThread(trampoline, this);
  }
}
} // namespace okapi
