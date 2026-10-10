#ifndef HEADINGESTIMATOR_H
#define HEADINGESTIMATOR_H

#include <math.h>

/* HeadingEstimator — accelerometer rate + magnetometer compass, blended with a
 * complementary filter that averages more compass readings the faster the robot spins.
 *
 * Same math, names and defaults as the browser sim (heading-sim/filter.js), so a
 * tuning found on a log can be pasted here. Call update() once per control loop,
 * in every drive mode, so the compass keeps the heading while parked and the
 * learned values persist across spins.
 *
 * Steps, every loop:
 *   1  w = dir * sqrt((accel_z - A_OFF) / r)           spin rate from the accelerometer
 *   2  theta += (w + wPrev)/2 * dt                     predict (trapezoid)
 *   3  on a NEW mag sample: thetaMag = atan2(u, v), e = wrap(thetaMag - theta)
 *      gate 1: field size must be Earth-sized   gate 2: big surprises get less weight
 *   4  theta += k * e  with  k = 1 / N,  N = 1 + N_PER_RAD * |w|  (readings averaged)
 *      wheels turning: learn r (a steady lag means r is wrong), the circle center and bFlat
 *      parked flat (motors off, both wheels 0 rpm, level): learn the accel offset aOff
 *   5  theta is the heading, [0, 2pi)
 *
 * The ESCs' rpm replies decide parked and turning. The accelerometer cannot, because its zero
 * offset is the unknown: a board with a +38 m/s2 rest reading looks like 19 rad/s of spin. With no
 * ESC replies (USB power) the accelerometer is used, the old rule, and nothing can spin anyway.
 *
 * Saved between loops (the whole state): theta, wPrev, r, cu, cv, aOff, bFlat, okRun, nOff.
 * r, cu, cv, aOff and bFlat are the calibration: they belong in flash between runs.
 */
class HeadingEstimator {
  public:
    // ---- constants: calibration (measured) ----
    float R0    = 0.122;   // m     start radius (old accelerometer_radius + radius_trim); r learns from here
    float A_OFF = 0;       // m/s2  accel Z sitting flat on the floor: START value only. It differs by board
                           //       (-4, +13, +38 seen) and drifts, so the first parked reading replaces it
                           //       outright and later ones average in (N_OFF). Leave 0. TODO: keep in flash
    float CU0   = 0;       // uT    mag circle center, u = (mx+my)/sqrt2: START value only, learned while
    float CV0   = 0;       // uT    mag circle center, v = mz.  spinning (N_C). Leave 0. TODO: keep in flash
    float B_NOM = 19.7;    // uT    in-plane field size while spinning

    // ---- constants: tuning (chosen) ----
    float N_PER_RAD = 1.5;       // readings per rad/s   each accepted compass reading closes 1/N of the gap,
                                 //               N = 1 + N_PER_RAD * |w|: 61 at 40 rad/s, 1 parked (the heading
                                 //               simply is the compass). No timer: a missing or rejected reading
                                 //               does not pull, so there is no catch-up after a gap
    float N_R      = 1500;       // readings      of 1 rad lag that would change r by 100% (0 = never learn r)
    float N_C      = 1000;       // readings      averaged by the circle-center tracker, ~10 revs at cruise
    float N_OFF    = 6000;       // readings      the offset learner averages once settled (~10 s parked).
                                 //               Parked reading k pulls 1/min(k, N_OFF) of the way
    float FLAT_TOL = 5;          // uT            "flat" = spin-axis field within this of bFlat (its value while
                                 //               spinning on the floor). A 15 deg tilt moves it by more, and only
                                 //               tilts past that change the rest reading visibly (0.3 m/s2), so a
                                 //               robot held in the hand teaches nothing. A spinning robot rides
                                 //               2-3 uT from parked (it sits on its skid when parked): keep room
    float PARK_E   = 0.01745;    // rad (1 deg)   parked readings must also show a still compass. With the wheels
                                 //               reported stopped the heading is not predicted, so e IS the compass
                                 //               motion per reading; 1 deg = 10 rad/s. Guards against an ESC that
                                 //               answers 0 rpm while the robot coasts
    float GATE_MAG = 0.35;       // fraction      reject a fix whose field size is off by more than this
    float E0       = 0.7854;     // rad (45 deg)  gate 2: surprises above this get weight 1/(1+(e/E0)^2)
    float W_MIN    = 0.15;       //               floor on that weight, so a wrong heading still converges
    float MIN_SPIN = 15;         // rad/s         below this: no learning (the mag still corrects the heading)
    int   LEARN_AFTER = 10;      // fixes         r and the center only move after this many consecutive accepted
                                 //               fixes: during a rejection streak (motor field at high throttle,
                                 //               a nearby robot) the fixes that pass can be the wrong angle

    // ---- state: everything that carries over to the next loop ----
    float theta = 0;   // rad   heading estimate, [0, 2pi)
    float wPrev = 0;   // rad/s last loop's rate, for the trapezoid
    float r     = 0;   // m     learned accelerometer radius
    float cu    = 0;   // uT    mag circle center, u
    float cv    = 0;   // uT    mag circle center, v
    float aOff  = 0;   // m/s2  accel Z offset, learned while parked flat
    float bFlat = NAN; // uT    spin-axis field while spinning on the floor (NAN until the first spin)
    int   okRun = 0;   //       consecutive accepted fixes (0 after any rejection)
    int   nOff  = 0;   //       parked readings averaged into aOff so far (capped at N_OFF)

    // ---- outputs of the last update(), for telemetry ----
    float w = 0;            // rad/s signed spin rate used this loop
    float n = 1;            //       compass readings averaged at this speed
    float thetaMag = 0;     // rad   last compass heading
    float e = 0;            // rad   last innovation (compass - estimate)
    float mag = 0;          // uT    last in-plane field size
    float axial = 0;        // uT    last spin-axis field component (compare with bFlat)
    bool  accepted = false; // last fix passed gate 1
    bool  parked = false;   // this loop counted as parked (offset learning allowed)
    bool  turning = false;  // this loop counted as wheels turning (r / center learning allowed)

    // What the ESCs say the wheels are doing, from their bidirectional DShot rpm replies
    enum Wheels { WHEELS_UNKNOWN = -1, WHEELS_STOPPED = 0, WHEELS_TURNING = 1 };

    // Loads the state from the calibration constants
    void begin() {
      theta = 0;
      wPrev = 0;
      r = R0;
      cu = CU0;
      cv = CV0;
      aOff = A_OFF;
      bFlat = NAN;
      okRun = 0;
      nOff = 0;
    }

    // dt in s, accel_z in m/s2, mag in uT, magFresh = true only when mx/my/mz is a NEW sample,
    // dir = +1 or -1: the sign of the robot's rotation as the compass heading sees it,
    // motorsOff = true when both motors are commanded to zero,
    // wheels = WHEELS_STOPPED when both ESCs answer 0 rpm, WHEELS_TURNING when either wheel turns,
    // WHEELS_UNKNOWN when the ESCs are not answering (then the accelerometer decides, the old rule).
    void update(float dt, float accel_z, bool magFresh, float mx, float my, float mz, int dir, bool motorsOff,
                Wheels wheels = WHEELS_UNKNOWN) {
      // STEP 1 — spin rate from the accelerometer.  a = w^2 r  ->  w = sqrt(a / r)
      float a = accel_z - aOff;
      if (a < 0) a = 0;
      float wAbs = sqrtf(a / r);
      if (wheels == WHEELS_STOPPED) wAbs = 0;   // wheels on the floor and still = no spin, whatever the
                                                // accelerometer says (its offset may not be known yet)
      w = dir * wAbs;
      n = 1 + N_PER_RAD * wAbs;
      // parked: safe to learn the offset. turning: safe to learn r and the center.
      parked = motorsOff && (wheels == WHEELS_STOPPED || (wheels == WHEELS_UNKNOWN && wAbs < MIN_SPIN));
      turning = wheels == WHEELS_TURNING || (wheels == WHEELS_UNKNOWN && !motorsOff && wAbs > MIN_SPIN);

      // STEP 2 — predict
      theta = wrap2Pi(theta + 0.5f * (w + wPrev) * dt);
      wPrev = w;
      accepted = false;
      if (!magFresh) return;

      // STEP 3 — compass fix. Spin axis is (-x+y)/sqrt2, so the in-plane pair is (x+y)/sqrt2 and z.
      float u = 0.70710678f * (mx + my) - cu;
      float v = mz - cv;
      axial = 0.70710678f * (-mx + my);   // along the spin axis: constant when flat, moves with tilt
      mag = sqrtf(u * u + v * v);
      thetaMag = wrap2Pi(atan2f(u, v));
      e = wrapPi(thetaMag - theta);

      if (fabsf(mag - B_NOM) <= GATE_MAG * B_NOM) {                 // gate 1: Earth-sized field
        float wgt = 1.0f / (1.0f + (e / E0) * (e / E0));             // gate 2: soft weight on surprises
        if (wgt < W_MIN) wgt = W_MIN;
        // STEP 4 — blend and learn. Each accepted reading closes 1/N of the gap; parked N = 1 so k = wgt.
        float k = wgt / n;
        theta = wrap2Pi(theta + k * e);
        accepted = true;
        okRun++;
        // The accel offset is observable only parked flat. Parked: motors off, the wheels not turning
        // (coasting is not parked) and the compass still. Flat: the fix in band (a tilt leaks the
        // vertical field into the plane and fails gate 1) AND, once a spin has shown what the spin-axis
        // field looks like on the floor, that component near its floor value, which catches the tilts
        // gate 1 lets through. Count-based: the first parked reading sets the offset outright, the k-th
        // pulls 1/k of the way, settling at 1/N_OFF. No board needs a configured offset.
        if (parked && fabsf(e) <= PARK_E && (isnan(bFlat) || fabsf(axial - bFlat) <= FLAT_TOL)) {
          if (nOff < N_OFF) nOff++;
          aOff += (accel_z - aOff) / nOff;
        }
        // learn only while the wheels turn, the spin is fast enough, and the mag has been clean for
        // LEARN_AFTER fixes
        if (turning && wAbs > MIN_SPIN && okRun >= LEARN_AFTER) {
          // Heading lagging the compass (e > 0 with dir = +1) means the rate is too low -> r too big.
          // The "* r" makes the step proportional to r (the 2 from w ~ 1/sqrt(r) is folded into N_R).
          if (N_R > 0) r -= dir * e * wgt * r / N_R;
          if (r < 0.6f * R0) r = 0.6f * R0;
          if (r > 2.0f * R0) r = 2.0f * R0;
          // The average of raw (u, v) over many revolutions is the circle center.
          float kc = N_C > 0 ? wgt / N_C : 0;
          cu += kc * u;
          cv += kc * v;
          // The spin-axis field while on the floor: what the parked "flat" test above compares
          // against. Until the first spin that test is skipped, so the first spin also re-arms the
          // offset learner: whatever was learned without the test is replaced by the next parked reading.
          if (isnan(bFlat)) { bFlat = axial; nOff = 0; }
          else bFlat += kc * (axial - bFlat);
        }
      } else {
        okRun = 0;   // a rejection resets the streak
      }
    }

    static float wrapPi(float a) {   // (-pi, pi]
      a = fmodf(a + (float)M_PI, (float)M_TWOPI);
      if (a < 0) a += (float)M_TWOPI;
      return a - (float)M_PI;
    }

    static float wrap2Pi(float a) {  // [0, 2pi)
      a = fmodf(a, (float)M_TWOPI);
      return a < 0 ? a + (float)M_TWOPI : a;
    }
};

#endif
