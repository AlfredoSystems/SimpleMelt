#ifndef HEADINGESTIMATOR_H
#define HEADINGESTIMATOR_H

/* HeadingEstimator — accelerometer rate + magnetometer compass, blended with a
 * complementary filter whose time constant grows with spin speed.
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
 *   4  theta += k * e  with  k = DT_FIX / (tau + DT_FIX),  tau = TAU_PER_RAD * |w|
 *      while spinning: learn r (a steady lag means r is wrong) and the circle center
 *   5  theta is the heading, [0, 2pi)
 *
 * Saved between loops (the whole state): theta, wPrev, r, cu, cv.
 */
class HeadingEstimator {
  public:
    // ---- constants: calibration (measured) ----
    float R0    = 0.122;   // m     start radius (old accelerometer_radius + radius_trim); r learns from here
    float A_OFF = 13.0;    // m/s2  accel Z when the robot sits on the floor. TODO: measure automatically, keep in flash
    float CU0   = 6.4;     // uT    mag circle center at cruise power, u = (mx+my)/sqrt2.  TODO: keep in flash
    float CV0   = 2.4;     // uT    mag circle center, v = mz
    float B_NOM = 19.7;    // uT    in-plane field size while spinning

    // ---- constants: tuning (chosen) ----
    float TAU_PER_RAD = 0.0025;  // s per rad/s   blend time = TAU_PER_RAD * |w|: 0.1 s at 40 rad/s, ~0 parked
    float K_R      = 0.2;        // 1/(rad s)     radius learning rate
    float C_REVS   = 10;         // revolutions   averaged by the circle-center tracker
    float GATE_MAG = 0.35;       // fraction      reject a fix whose field size is off by more than this
    float E0       = 0.7854;     // rad (45 deg)  gate 2: surprises above this get weight 1/(1+(e/E0)^2)
    float W_MIN    = 0.15;       //               floor on that weight, so a wrong heading still converges
    float MIN_SPIN = 15;         // rad/s         below this: no learning (the mag still corrects the heading)
    float DT_FIX   = 1.0 / 600;  // s             nominal interval between compass fixes. Every accepted fix
                                 //               pulls the same fraction DT_FIX/(tau+DT_FIX): no timer, no
                                 //               catch-up, missing and rejected samples behave the same

    // ---- state: everything that carries over to the next loop ----
    float theta = 0;   // rad   heading estimate, [0, 2pi)
    float wPrev = 0;   // rad/s last loop's rate, for the trapezoid
    float r     = 0;   // m     learned accelerometer radius
    float cu    = 0;   // uT    mag circle center, u
    float cv    = 0;   // uT    mag circle center, v

    // ---- outputs of the last update(), for telemetry ----
    float w = 0;            // rad/s signed spin rate used this loop
    float tau = 0;          // s     blend time this loop
    float thetaMag = 0;     // rad   last compass heading
    float e = 0;            // rad   last innovation (compass - estimate)
    float mag = 0;          // uT    last in-plane field size
    bool  accepted = false; // last fix passed gate 1

    void begin();  // loads the state from the calibration constants

    // dt in s, accel_z in m/s2, mag in uT, magFresh = true only when mx/my/mz is a NEW sample,
    // dir = +1 or -1: the sign of the robot's rotation as the compass heading sees it.
    void update(float dt, float accel_z, bool magFresh, float mx, float my, float mz, int dir);

    static float wrapPi(float a);   // (-pi, pi]
    static float wrap2Pi(float a);  // [0, 2pi)
};

#endif
