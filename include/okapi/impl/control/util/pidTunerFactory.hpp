/**
 * @author Jonathan Bayless, Team BLRS
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_PIDTUNERFACTORY_HPP_
#define _OKAPI_PIDTUNERFACTORY_HPP_

#include "okapi/api/control/util/pidTuner.hpp"
#include <memory>

namespace okapi {
class PIDTunerFactory {
  public:
  static PIDTuner create(std::shared_ptr<ControllerInput<double>> iinput,
                         std::shared_ptr<ControllerOutput<double>> ioutput,
                         QTime itimeout,
                         std::int32_t igoal,
                         double ikPMin,
                         double ikPMax,
                         double ikIMin,
                         double ikIMax,
                         double ikDMin,
                         double ikDMax,
                         std::int32_t inumIterations = 5,
                         std::int32_t inumParticles = 16,
                         double ikSettle = 1,
                         double ikITAE = 2);

  static std::unique_ptr<PIDTuner> createPtr(std::shared_ptr<ControllerInput<double>> iinput,
                                             std::shared_ptr<ControllerOutput<double>> ioutput,
                                             QTime itimeout,
                                             std::int32_t igoal,
                                             double ikPMin,
                                             double ikPMax,
                                             double ikIMin,
                                             double ikIMax,
                                             double ikDMin,
                                             double ikDMax,
                                             std::int32_t inumIterations = 5,
                                             std::int32_t inumParticles = 16,
                                             double ikSettle = 1,
                                             double ikITAE = 2);
};
} // namespace okapi

#endif
