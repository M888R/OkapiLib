/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include <numeric>

namespace okapi {
AsyncLinearMotionProfileController::AsyncLinearMotionProfileController(
  const TimeUtil &itimeUtil,
  const double imaxVel,
  const double imaxAccel,
  const double imaxJerk,
  std::shared_ptr<ControllerOutput<double>> ioutput)
  : members(std::make_shared<members_s>(Logger::instance(),
                                        imaxVel,
                                        imaxAccel,
                                        imaxJerk,
                                        ioutput,
                                        itimeUtil,
                                        this)) {
}

AsyncLinearMotionProfileController::AsyncLinearMotionProfileController(
  AsyncLinearMotionProfileController &&other) noexcept
  : members(std::move(other.members)) {
}

AsyncLinearMotionProfileController::~AsyncLinearMotionProfileController() {
  members->dtorCalled.store(true, std::memory_order_release);

  for (auto path : members->paths) {
    free(path.second.segment);
  }

  delete members->task;
}

void AsyncLinearMotionProfileController::generatePath(std::initializer_list<double> iwaypoints,
                                                      const std::string &ipathId) {
  if (iwaypoints.size() == 0) {
    // No point in generating a path
    members->logger->warn(
      "AsyncLinearMotionProfileController: Not generating a path because no waypoints were given.");
    return;
  }

  std::vector<Waypoint> points;
  points.reserve(iwaypoints.size());
  for (auto &point : iwaypoints) {
    points.push_back(Waypoint{point, 0, 0});
  }

  TrajectoryCandidate candidate;
  members->logger->info("AsyncLinearMotionProfileController: Preparing trajectory");
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
      "AsyncLinearMotionProfileController: Path is impossible with waypoints: " +
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
    std::string message = "AsyncLinearMotionProfileController: Could not allocate trajectory. The "
                          "path is probably impossible.";
    members->logger->error(message);

    if (candidate.laptr) {
      free(candidate.laptr);
    }

    if (candidate.saptr) {
      free(candidate.saptr);
    }

    throw std::runtime_error(message);
  }

  members->logger->info("AsyncLinearMotionProfileController: Generating path");
  pathfinder_generate(&candidate, trajectory);

  // Free the old path before overwriting it
  removePath(ipathId);

  members->paths.emplace(ipathId, TrajectoryPair{trajectory, length});
  members->logger->info("AsyncLinearMotionProfileController: Completely done generating path");
  members->logger->info("AsyncLinearMotionProfileController: " + std::to_string(length));
}

void AsyncLinearMotionProfileController::removePath(const std::string &ipathId) {
  auto oldPath = members->paths.find(ipathId);
  if (oldPath != members->paths.end()) {
    free(oldPath->second.segment);
    members->paths.erase(ipathId);
  }
}

std::vector<std::string> AsyncLinearMotionProfileController::getPaths() {
  std::vector<std::string> keys;

  for (const auto &path : members->paths) {
    keys.push_back(path.first);
  }

  return keys;
}

void AsyncLinearMotionProfileController::setTarget(const std::string ipathId) {
  members->currentPath = ipathId;
  members->isRunning = true;
}

void AsyncLinearMotionProfileController::controllerSet(const std::string ivalue) {
  setTarget(ivalue);
}

std::string AsyncLinearMotionProfileController::getTarget() {
  return members->currentPath;
}

std::string AsyncLinearMotionProfileController::getTarget() const {
  return members->currentPath;
}

void AsyncLinearMotionProfileController::loop() {
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
      members->logger->info("AsyncLinearMotionProfileController: Running with path: " +
                            members->currentPath);
      auto path = members->paths.find(members->currentPath);

      if (path == members->paths.end()) {
        members->logger->warn(
          "AsyncLinearMotionProfileController: Target was set to non-existent path with name: " +
          members->currentPath);
      } else {
        members->logger->debug("AsyncLinearMotionProfileController: Path length is " +
                               std::to_string(path->second.length));

        executeSinglePath(path->second, members->timeUtil.getRate());
        members->output->controllerSet(0);

        members->logger->info("AsyncLinearMotionProfileController: Done moving");
      }

      members->isRunning.store(false, std::memory_order_release);
    }

    rate->delayUntil(10_ms);
  }
}

void AsyncLinearMotionProfileController::executeSinglePath(const TrajectoryPair &path,
                                                           std::unique_ptr<AbstractRate> rate) {
  for (int i = 0; i < path.length && !isDisabled(); ++i) {
    members->currentProfilePosition = path.segment[i].position;
    members->output->controllerSet(path.segment[i].velocity / members->maxVel);
    rate->delayUntil(1_ms);
  }
}

void AsyncLinearMotionProfileController::trampoline(void *context) {
  if (context) {
    static_cast<AsyncLinearMotionProfileController *>(context)->loop();
  }
}

void AsyncLinearMotionProfileController::waitUntilSettled() {
  members->logger->info("AsyncLinearMotionProfileController: Waiting to settle");

  auto rate = members->timeUtil.getRate();
  while (!isSettled()) {
    rate->delayUntil(10_ms);
  }

  members->logger->info("AsyncLinearMotionProfileController: Done waiting to settle");
}

void AsyncLinearMotionProfileController::moveTo(double iposition, double itarget) {
  std::string name = reinterpret_cast<const char *>(this); // hmmmm...
  generatePath({iposition, itarget}, name);
  setTarget(name);
  waitUntilSettled();
  removePath(name);
}

double AsyncLinearMotionProfileController::getError() const {
  if (const auto path = members->paths.find(getTarget()); path == members->paths.end()) {
    return 0;
  } else {
    // The last position in the path is the target position
    return path->second.segment[path->second.length - 1].position - members->currentProfilePosition;
  }
}

bool AsyncLinearMotionProfileController::isSettled() {
  return isDisabled() || !members->isRunning.load(std::memory_order_acquire);
}

void AsyncLinearMotionProfileController::reset() {
  // Interrupt executeSinglePath() by disabling the controller
  flipDisable(true);

  auto rate = members->timeUtil.getRate();
  while (members->isRunning.load(std::memory_order_acquire)) {
    rate->delayUntil(1_ms);
  }

  flipDisable(false);
}

void AsyncLinearMotionProfileController::flipDisable() {
  flipDisable(!members->disabled.load(std::memory_order_acquire));
}

void AsyncLinearMotionProfileController::flipDisable(const bool iisDisabled) {
  members->logger->info("AsyncLinearMotionProfileController: flipDisable " +
                        std::to_string(iisDisabled));
  members->disabled.store(iisDisabled, std::memory_order_release);
  // loop() will set the output to 0 when executeSinglePath() is done
  // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncLinearMotionProfileController::isDisabled() const {
  return members->disabled.load(std::memory_order_acquire);
}

void AsyncLinearMotionProfileController::startThread() {
  if (!members->task) {
    members->task = new CrossplatformThread(trampoline, this);
  }
}
} // namespace okapi
