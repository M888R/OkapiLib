/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(
  const TimeUtil &itimeUtil,
  std::shared_ptr<ChassisModel> imodel,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  std::unique_ptr<IterativePosPIDController> iturnController,
  const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales)
  : ChassisController(imodel, toUnderlyingType(igearset.internalGearset)),
    members(std::make_shared<membersd>(Logger::instance(),
                                       itimeUtil.getRate(),
                                       std::move(idistanceController),
                                       std::move(iangleController),
                                       std::move(iturnController),
                                       iscales,
                                       igearset,
                                       imodel)) {
  if (igearset.ratio == 0) {
    members->logger->error(
      "ChassisControllerPID: The gear ratio cannot be zero! Check if you are using "
      "integer division.");
    throw std::invalid_argument("ChassisControllerPID: The gear ratio cannot be zero! Check if you "
                                "are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

ChassisControllerPID::ChassisControllerPID(ChassisControllerPID &&other) noexcept
  : ChassisController(std::move(other.model), other.maxVelocity, other.maxVoltage),
    members(std::move(other.members)) {
  other.members->task = nullptr;
}

ChassisControllerPID::~ChassisControllerPID() {
  printf("dtor\n");
  members->dtorCalled.store(true, std::memory_order_release);
  delete members->task;
}

void ChassisControllerPID::loop(void *params) {
  std::shared_ptr<membersd> members = *static_cast<std::shared_ptr<membersd> *>(params);

  auto encStartVals = members->myModel->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;
  modeType pastMode = none;

  while (!members->dtorCalled.load(std::memory_order_acquire)) {
    printf("loop\n");
    /**
     * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
     * waitUntilSettled
     */
    if (!members->doneLooping.load(std::memory_order_acquire)) {
      if (members->mode != pastMode || members->newMovement.load(std::memory_order_acquire)) {
        encStartVals = members->myModel->getSensorVals();
        members->newMovement.store(false, std::memory_order_release);
      }

      switch (members->mode) {
      case distance:
        encVals = members->myModel->getSensorVals() - encStartVals;
        distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
        angleChange = static_cast<double>(encVals[0] - encVals[1]);
        members->myModel->driveVector(members->distancePid->step(distanceElapsed),
                                      members->anglePid->step(angleChange));
        break;

      case angle:
        encVals = members->myModel->getSensorVals() - encStartVals;
        angleChange = static_cast<double>(encVals[0] - encVals[1]);
        members->myModel->rotate(members->turnPid->step(angleChange));
        break;

      default:
        break;
      }

      pastMode = members->mode;
    }

    members->rate->delayUntil(10_ms);
  }

  printf("done\n");
}

void ChassisControllerPID::moveDistanceAsync(const QLength itarget) {
  members->logger->info("ChassisControllerPID: moving " + std::to_string(itarget.convert(meter)) +
                        " meters");

  members->distancePid->reset();
  members->anglePid->reset();
  members->distancePid->flipDisable(false);
  members->anglePid->flipDisable(false);
  members->turnPid->flipDisable(true);
  members->mode = distance;

  const double newTarget =
    itarget.convert(meter) * members->scales.straight * members->gearsetRatioPair.ratio;

  members->logger->info("ChassisControllerPID: moving " + std::to_string(newTarget) +
                        " motor degrees");

  members->distancePid->setTarget(newTarget);
  members->anglePid->setTarget(0);

  members->doneLooping.store(false, std::memory_order_release);
  members->newMovement.store(true, std::memory_order_release);
}

void ChassisControllerPID::moveDistanceAsync(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistanceAsync((itarget / members->scales.straight) * meter);
}

void ChassisControllerPID::moveDistance(const QLength itarget) {
  moveDistanceAsync(itarget);
  waitUntilSettled();
}

void ChassisControllerPID::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / members->scales.straight) * meter);
}

void ChassisControllerPID::turnAngleAsync(const QAngle idegTarget) {
  members->logger->info("ChassisControllerPID: turning " +
                        std::to_string(idegTarget.convert(degree)) + " degrees");

  members->turnPid->reset();
  members->turnPid->flipDisable(false);
  members->distancePid->flipDisable(true);
  members->anglePid->flipDisable(true);
  members->mode = angle;

  const double newTarget =
    idegTarget.convert(degree) * members->scales.turn * members->gearsetRatioPair.ratio;

  members->logger->info("ChassisControllerPID: turning " + std::to_string(newTarget) +
                        " motor degrees");

  members->turnPid->setTarget(newTarget);

  members->doneLooping.store(false, std::memory_order_release);
  members->newMovement.store(true, std::memory_order_release);
}

void ChassisControllerPID::turnAngleAsync(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngleAsync((idegTarget / members->scales.turn) * degree);
}

void ChassisControllerPID::turnAngle(const QAngle idegTarget) {
  turnAngleAsync(idegTarget);
  waitUntilSettled();
}

void ChassisControllerPID::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / members->scales.turn) * degree);
}

void ChassisControllerPID::waitUntilSettled() {
  members->logger->info("ChassisControllerPID: Waiting to settle");
  bool completelySettled = false;

  while (!completelySettled) {
    switch (members->mode) {
    case distance:
      completelySettled = waitForDistanceSettled();
      break;

    case angle:
      completelySettled = waitForAngleSettled();
      break;

    default:
      completelySettled = true;
      break;
    }
  }

  stopAfterSettled();
  members->mode = none;
  members->doneLooping.store(true, std::memory_order_release);
  members->logger->info("ChassisControllerPID: Done waiting to settle");
}

/**
 * Wait for the distance setup (distancePid and anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForDistanceSettled() {
  members->logger->info("ChassisControllerPID: Waiting to settle in distance mode");

  while (!(members->distancePid->isSettled() && members->anglePid->isSettled())) {
    if (members->mode == angle) {
      // False will cause the loop to re-enter the switch
      members->logger->warn(
        "ChassisControllerPID: Mode changed to angle while waiting in distance!");
      return false;
    }

    members->rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

/**
 * Wait for the angle setup (anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForAngleSettled() {
  members->logger->info("ChassisControllerPID: Waiting to settle in angle mode");

  while (!members->turnPid->isSettled()) {
    if (members->mode == distance) {
      // False will cause the loop to re-enter the switch
      members->logger->warn(
        "ChassisControllerPID: Mode changed to distance while waiting in angle!");
      return false;
    }

    members->rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

void ChassisControllerPID::stopAfterSettled() {
  members->distancePid->flipDisable(true);
  members->anglePid->flipDisable(true);
  members->turnPid->flipDisable(true);
  model->stop();
}

void ChassisControllerPID::stop() {
  stopAfterSettled();
  ChassisController::stop();
}

void ChassisControllerPID::startThread() {
  if (!members->task) {
    members->task = new CrossplatformThread(loop, &members);
  }
}

ChassisScales ChassisControllerPID::getChassisScales() const {
  return members->scales;
}

AbstractMotor::GearsetRatioPair ChassisControllerPID::getGearsetRatioPair() const {
  return members->gearsetRatioPair;
}
} // namespace okapi
