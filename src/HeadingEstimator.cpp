#include <Arduino.h>
#include <math.h>
#include "HeadingEstimator.h"

float HeadingEstimator::wrapPi(float a) {
  a = fmodf(a + (float)M_PI, (float)M_TWOPI);
  if (a < 0) a += (float)M_TWOPI;
  return a - (float)M_PI;
}

float HeadingEstimator::wrap2Pi(float a) {
  a = fmodf(a, (float)M_TWOPI);
  return a < 0 ? a + (float)M_TWOPI : a;
}

void HeadingEstimator::begin() {
  theta = 0;
  wPrev = 0;
  r = R0;
  cu = CU0;
  cv = CV0;
  aOff = A_OFF;
  okRun = 0;
}

void HeadingEstimator::update(float dt, float accel_z, bool magFresh, float mx, float my, float mz, int dir, bool motorsOff) {
  // STEP 1 — spin rate from the accelerometer.  a = w^2 r  ->  w = sqrt(a / r)
  float a = accel_z - aOff;
  if (a < 0) a = 0;
  float wAbs = sqrtf(a / r);
  w = dir * wAbs;
  tau = TAU_PER_RAD * wAbs;

  // STEP 2 — predict
  theta = wrap2Pi(theta + 0.5f * (w + wPrev) * dt);
  wPrev = w;
  accepted = false;
  if (!magFresh) return;

  // STEP 3 — compass fix. Spin axis is (-x+y)/sqrt2, so the in-plane pair is (x+y)/sqrt2 and z.
  float u = 0.70710678f * (mx + my) - cu;
  float v = mz - cv;
  mag = sqrtf(u * u + v * v);
  thetaMag = wrap2Pi(atan2f(u, v));
  e = wrapPi(thetaMag - theta);

  if (fabsf(mag - B_NOM) <= GATE_MAG * B_NOM) {                 // gate 1: Earth-sized field
    float wgt = 1.0f / (1.0f + (e / E0) * (e / E0));             // gate 2: soft weight on surprises
    if (wgt < W_MIN) wgt = W_MIN;
    // STEP 4 — blend and learn. Same fraction on every accepted fix; parked tau ~ 0 so k -> wgt.
    float k = DT_FIX / (tau + DT_FIX) * wgt;
    theta = wrap2Pi(theta + k * e);
    accepted = true;
    okRun++;
    // The accel offset is observable only parked flat: motors off, not coasting (motors off while
    // still spinning is not parked), and the fix in band (a robot tilted in the hand leaks the
    // vertical field into the plane and fails gate 1).
    if (motorsOff && wAbs < MIN_SPIN) aOff += (accel_z - aOff) * dt / TAU_OFF;
    // learn only while spinning and only once the mag has been clean for LEARN_AFTER fixes
    if (wAbs > MIN_SPIN && okRun >= LEARN_AFTER) {
      // Heading lagging the compass (e > 0 with dir = +1) means the rate is too low -> r too big.
      // w ~ 1/sqrt(r), hence the 2 and the "* r".
      r -= 2.0f * K_R * dir * e * wgt * DT_FIX * r;
      if (r < 0.6f * R0) r = 0.6f * R0;
      if (r > 2.0f * R0) r = 2.0f * R0;
      // The average of raw (u, v) over whole revolutions is the circle center.
      float kc = wAbs * DT_FIX / ((float)M_TWOPI * C_REVS) * wgt;
      if (kc > 1) kc = 1;
      cu += kc * u;
      cv += kc * v;
    }
  } else {
    okRun = 0;   // a rejection resets the streak
  }
}
