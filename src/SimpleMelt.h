#ifndef SIMPLEMELT_H
#define SIMPLEMELT_H

// SimpleMelt - meltybrain robot control.
//
//   MeltyDrive         sticks + heading -> motor powers and LEDs (any board)
//   HeadingEstimator   accelerometer + magnetometer -> heading (any board)
//   Button             a button or switch position on a CRSF channel
//   RotiniV4           the Rotini V4 board: sensors, ESCs, receiver, LEDs
//
// Include this for everything, or the individual headers for only what you use.

#include "MeltyDrive.h"
#include "HeadingEstimator.h"
#include "Button.h"
#include "RotiniV4.h"

#endif
