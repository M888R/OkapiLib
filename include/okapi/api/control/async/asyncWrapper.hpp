/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCWRAPPER_HPP_
#define _OKAPI_ASYNCWRAPPER_HPP_

#include "okapi/api/control/async/asyncController.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/coreProsAPI.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/api/util/supplier.hpp"
#include <atomic>
#include <memory>

namespace okapi {
template <typename Input, typename Output>
class AsyncWrapper : virtual public AsyncController<Input, Output> {
  public:
  /**
   * A wrapper class that transforms an IterativeController into an AsyncController by running it
   * in another task. The input controller will act like an AsyncController.
   *
   * @param iinput controller input, passed to the IterativeController
   * @param ioutput controller output, written to from the IterativeController
   * @param icontroller the controller to use
   * @param irateSupplier used for rates used in the main loop and in waitUntilSettled
   * @param isettledUtil used in waitUntilSettled
   * @param iscale the scale applied to the controller output
   */
  AsyncWrapper(std::shared_ptr<ControllerInput<Input>> iinput,
               std::shared_ptr<ControllerOutput<Output>> ioutput,
               std::unique_ptr<IterativeController<Input, Output>> icontroller,
               const Supplier<std::unique_ptr<AbstractRate>> &irateSupplier)
    : members(std::make_shared<members_s>(Logger::instance(),
                                          iinput,
                                          ioutput,
                                          std::move(icontroller),
                                          irateSupplier.get(),
                                          irateSupplier.get(),
                                          this)) {
  }

  AsyncWrapper(AsyncWrapper<Input, Output> &&other) noexcept : members(std::move(other.members)) {
  }

  ~AsyncWrapper() override {
    members->dtorCalled.store(true, std::memory_order_release);
    delete members->task;
  }

  /**
   * Sets the target for the controller.
   */
  void setTarget(Input itarget) override {
    members->logger->info("AsyncWrapper: Set target to " + std::to_string(itarget));
    members->hasFirstTarget = true;
    members->controller->setTarget(itarget);
    members->lastTarget = itarget;
  }

  /**
   * Writes the value of the controller output. This method might be automatically called in another
   * thread by the controller.
   *
   * @param ivalue the controller's output
   */
  void controllerSet(Input ivalue) override {
    members->controller->controllerSet(ivalue);
  }

  /**
   * Gets the last set target, or the default target if none was set.
   *
   * @return the last target
   */
  Input getTarget() override {
    return members->controller->getTarget();
  }

  /**
   * Returns the last calculated output of the controller.
   */
  Output getOutput() const {
    return members->controller->getOutput();
  }

  /**
   * Returns the last error of the controller.
   */
  Output getError() const override {
    return members->controller->getError();
  }

  /**
   * Returns whether the controller has settled at the target. Determining what settling means is
   * implementation-dependent.
   *
   * If the controller is disabled, this method must return true.
   *
   * @return whether the controller is settled
   */
  bool isSettled() override {
    return isDisabled() || members->controller->isSettled();
  }

  /**
   * Set time between loops.
   *
   * @param isampleTime time between loops
   */
  void setSampleTime(QTime isampleTime) {
    members->controller->setSampleTime(isampleTime);
  }

  /**
   * Set controller output bounds.
   *
   * @param imax max output
   * @param imin min output
   */
  void setOutputLimits(Output imax, Output imin) {
    members->controller->setOutputLimits(imax, imin);
  }

  /**
   * Get the upper output bound.
   *
   * @return  the upper output bound
   */
  Output getMaxOutput() {
    return members->controller->getMaxOutput();
  }

  /**
   * Get the lower output bound.
   *
   * @return the lower output bound
   */
  Output getMinOutput() {
    return members->controller->getMinOutput();
  }

  /**
   * Resets the controller so it can start from 0 again properly. Keeps configuration from
   * before.
   */
  void reset() override {
    members->logger->info("AsyncWrapper: Reset");
    members->controller->reset();
    members->hasFirstTarget = false;
  }

  /**
   * Changes whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   */
  void flipDisable() override {
    members->logger->info("AsyncWrapper: flipDisable " +
                          std::to_string(!members->controller->isDisabled()));
    members->controller->flipDisable();
    resumeMovement();
  }

  /**
   * Sets whether the controller is off or on. Turning the controller on after it was off will
   * cause the controller to move to its last set target, unless it was reset in that time.
   *
   * @param iisDisabled whether the controller is disabled
   */
  void flipDisable(bool iisDisabled) override {
    members->logger->info("AsyncWrapper: flipDisable " + std::to_string(iisDisabled));
    members->controller->flipDisable(iisDisabled);
    resumeMovement();
  }

  /**
   * Returns whether the controller is currently disabled.
   *
   * @return whether the controller is currently disabled
   */
  bool isDisabled() const override {
    return members->controller->isDisabled();
  }

  /**
   * Blocks the current task until the controller has settled. Determining what settling means is
   * implementation-dependent.
   */
  void waitUntilSettled() override {
    members->logger->info("AsyncWrapper: Waiting to settle");

    while (!isSettled()) {
      members->loopRate->delayUntil(motorUpdateRate);
    }

    members->logger->info("AsyncWrapper: Done waiting to settle");
  }

  /**
   * Starts the internal thread. This should not be called by normal users. This method is called
   * by the AsyncControllerFactory when making a new instance of this class.
   */
  void startThread() {
    if (!members->task) {
      members->task = new CrossplatformThread(trampoline, this);
    }
  }

  protected:
  struct members_s {
    members_s(Logger *ilogger,
              const std::shared_ptr<ControllerInput<Input>> &iinput,
              const std::shared_ptr<ControllerOutput<Output>> &ioutput,
              std::unique_ptr<IterativeController<Input, Output>> icontroller,
              std::unique_ptr<AbstractRate> iloopRate,
              std::unique_ptr<AbstractRate> isettledRate,
              const AsyncWrapper *iself)
      : logger(ilogger),
        input(iinput),
        output(ioutput),
        controller(std::move(icontroller)),
        loopRate(std::move(iloopRate)),
        settledRate(std::move(isettledRate)),
        self(iself) {
    }

    Logger *logger;
    std::shared_ptr<ControllerInput<Input>> input;
    std::shared_ptr<ControllerOutput<Output>> output;
    std::unique_ptr<IterativeController<Input, Output>> controller;
    std::unique_ptr<AbstractRate> loopRate;
    std::unique_ptr<AbstractRate> settledRate;

    bool hasFirstTarget{false};
    Input lastTarget;
    std::atomic_bool dtorCalled{false};

    CrossplatformThread *task{nullptr};
    volatile const AsyncWrapper *self;
  };

  std::shared_ptr<members_s> members;

  static void trampoline(void *context) {
    if (context) {
      static_cast<AsyncWrapper *>(context)->loop();
    }
  }

  void loop() {
    while (!members->dtorCalled.load(std::memory_order_acquire)) {
      if (members->self == nullptr) {
        /**
         * self will be null if task which created the parent object was deleted and the idle task
         * freed its stack. For example, when the robot is running during opcontrol and is suddenly
         * disabled.
         */
        return;
      }

      if (!isDisabled()) {
        members->output->controllerSet(members->controller->step(members->input->controllerGet()));
      }

      members->loopRate->delayUntil(members->controller->getSampleTime());
    }
  }

  /**
   * Resumes moving after the controller is reset. Should not cause movement if the controller is
   * turned off, reset, and turned back on.
   */
  virtual void resumeMovement() {
    if (isDisabled()) {
      // This will grab the output *when disabled*
      members->output->controllerSet(members->controller->getOutput());
    } else {
      if (members->hasFirstTarget) {
        setTarget(members->lastTarget);
      }
    }
  }
};
} // namespace okapi

#endif
