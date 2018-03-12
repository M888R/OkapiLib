/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
 #include "okapi/device/adiButton.hpp"

 namespace okapi {
   ADIButton::ADIButton(const uint8_t iport, const bool iinverted):
    btn(iport),
    port(iport),
    inverted(iinverted),
    wasPressedLast(false) {}

  bool ADIButton::isPressed() {
    wasPressedLast = currentlyPressed();
    return wasPressedLast;
  }

  bool ADIButton::edge() {
    const bool pressed = currentlyPressed();
    const bool out = pressed ^ wasPressedLast;
    wasPressedLast = pressed;
    return out;
  }

  bool ADIButton::risingEdge() {
    return edge() && wasPressedLast;
  }

  bool ADIButton::fallingEdge() {
    return edge() && !wasPressedLast;
  }

  bool ADIButton::currentlyPressed() {
    const bool pressed = btn.value_get() != 0;
    return inverted ? !pressed : pressed;
  }
 }