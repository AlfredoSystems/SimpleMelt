#ifndef SIMPLEMELT_H
#define SIMPLEMELT_H

// SimpleMelt - meltybrain robot control, independent of the board.
//
//   MeltyDrive         sticks + heading -> motor powers and LEDs
//   HeadingEstimator   accelerometer + magnetometer -> heading
//
// The board (sensors, ESCs, receiver, LEDs) belongs to the sketch: see
// examples/Rotini-V4, whose RotiniV4 class wraps the Rotini V4 hardware.
// Switches, buttons and stick axes come from AlfredoCRSF (CrsfSwitch, getAxis).

#include "MeltyDrive.h"
#include "HeadingEstimator.h"

#endif
