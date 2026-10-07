#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>

#include "SimpleMelt.h"
#include "SimpleMeltUtility.h"

#include "AlfredoCRSF.h"
#include "SparkFun_LIS331.h"
#include <AlfredoTelemetry.h>

#include "mmc.h"

const int PIN_SPI_SCK = 6;
const int PIN_SPI_MISO = 7;
const int PIN_SPI_MOSI = 5;
const int PIN_ACCELEROMETER_CS = 47;
const int PIN_ACCELEROMETER_INT = 37;
const int PIN_MAGNETOMETER_CS = 48;
const int PIN_MAGNETOMETER_INT = 36;
const int PIN_CRSF_RX = 18;
const int PIN_CRSF_TX = 17;
const int PIN_MOTOR_FOO = 40;
const int PIN_MOTOR_BAR = 41;
const int PIN_MELTY_LED = 15;
const int PIN_STATUS_LED = 4;
const int PIN_SNS_VIN = 10;

SimpleMelt Rotini;

HardwareSerial crsfSerial(1);
AlfredoCRSF crsf;

LIS331 accelerometer;

SFE_MMC5983MA magnetometer;
const uint16_t MAG_RATE_HZ = 1000;  // continuous mode rate: 1000, 200, 100, 50, 20, 10 or 1
bool mag_ok = false;

OneShot125 foo;
OneShot125 bar;

void setup() {
  Rotini.melty_led_offset_CW = 2.22;     // radians (CCW is positive)
  Rotini.melty_led_offset_CCW = 4.02;    // radians (CCW is positive)
  Rotini.turn_speed = 1.1;              // rotations per second
  Rotini.accelerometer_radius = 0.090;  // meters
  Rotini.radius_trim = 0.032;           // meters

  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_MELTY_LED, OUTPUT);
  pinMode(PIN_ACCELEROMETER_CS, OUTPUT);
  pinMode(PIN_MAGNETOMETER_CS, OUTPUT);

  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_MELTY_LED, LOW);
  digitalWrite(PIN_ACCELEROMETER_CS, HIGH);
  digitalWrite(PIN_MAGNETOMETER_CS, HIGH);

  Serial.begin(115200);

  // Started before the sensors so a sensor problem can't keep the robot off the viewer
  Telemetry.begin("Rotini");

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf.begin(crsfSerial);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  accelerometer.setSPICSPin(PIN_ACCELEROMETER_CS);
  accelerometer.begin(LIS331::USE_SPI);  // Selects the bus to be used
  accelerometer.setODR(accelerometer.DR_1000HZ);
  accelerometer.setFullScale(accelerometer.HIGH_RANGE);  //400g range

  mag_ok = beginMag();

  foo.begin(PIN_MOTOR_FOO, 0);
  bar.begin(PIN_MOTOR_BAR, 1);
  foo.setReversed(true);
  bar.setReversed(false);
}

void loop() {

  crsf.update();
  Rotini.rotation = channel_to_axis(1);  //axis_right_x
  Rotini.throttle = channel_to_axis(3);  //axis_left_y
  //axis_right_y = channel_to_axis(2);
  //axis_left_x = channel_to_axis(4);

  //bumpers
  static Button left_bumper;
  left_bumper.update(crsf.getChannel(11), 1000);
  static Button right_bumper;
  right_bumper.update(crsf.getChannel(11), 2000);
  //trim buttons
  static Button left_left_arrow;
  left_left_arrow.update(crsf.getChannel(10), 1000);
  static Button left_right_arrow;
  left_right_arrow.update(crsf.getChannel(10), 2000);
  static Button left_up_arrow;
  left_up_arrow.update(crsf.getChannel(9), 2000);
  static Button left_down_arrow;
  left_down_arrow.update(crsf.getChannel(9), 1000);
  static Button right_left_arrow;
  right_left_arrow.update(crsf.getChannel(7), 1000);
  static Button right_right_arrow;
  right_right_arrow.update(crsf.getChannel(7), 2000);
  static Button right_up_arrow;
  right_up_arrow.update(crsf.getChannel(8), 2000);
  static Button right_down_arrow;
  right_down_arrow.update(crsf.getChannel(8), 1000);
  // SWA - Zorro Switch B
  static Button SWA_backward;
  SWA_backward.update(crsf.getChannel(6), 1000);
  static Button SWA_neutral;
  SWA_neutral.update(crsf.getChannel(6), 1500);
  static Button SWA_forward;
  SWA_forward.update(crsf.getChannel(6), 2000);
  // SWD - Zorro Switch F
  static Button SWD_backward;
  SWD_backward.update(crsf.getChannel(5), 1000);
  static Button SWD_forward;
  SWD_forward.update(crsf.getChannel(5), 2000);
  // SWB - Zorro Switch C
  static Button SWB_backward;
  SWB_backward.update(crsf.getChannel(12), 1000);
  static Button SWB_neutral;
  SWB_neutral.update(crsf.getChannel(12), 1500);
  static Button SWB_forward;
  SWB_forward.update(crsf.getChannel(12), 2000);

  if (crsf.isLinkUp()) {
    if (SWA_backward.is_held()) Rotini.drive_mode = STOP;
    else if (SWA_neutral.is_held()) Rotini.drive_mode = ARCADE;
    else if (SWA_forward.is_held()) Rotini.drive_mode = MELTY;
  } else {
    Rotini.drive_mode = NO_CONNECTION;
  }

  // Reads the accelerometer every pass so telemetry has it in every drive mode
  int16_t accel_x, accel_y, accel_z;
  accelerometer.readAxes(accel_x, accel_y, accel_z);
  Telemetry.add("accel_x", LIS331_to_mps2(accel_x));  // m/s^2
  Telemetry.add("accel_y", LIS331_to_mps2(accel_y));
  Telemetry.add("accel_z", LIS331_to_mps2(accel_z));

  float mag_x, mag_y, mag_z;
  if (mag_ok && updateMag(mag_x, mag_y, mag_z)) {
    Telemetry.add("mag_x", mag_x);  // uT
    Telemetry.add("mag_y", mag_y);
    Telemetry.add("mag_z", mag_z);
  }

  if (Rotini.drive_mode == MELTY) {
	// Passes the accelerometer info to the Rotini object
    Rotini.accelerometer_x = 0; // LIS331_to_mps2(accel_x);
    Rotini.accelerometer_y = 0; // LIS331_to_mps2(accel_y);
    Rotini.accelerometer_z = LIS331_to_mps2(accel_z);

	// Spin_power is the average power that motors are set to.
    if (right_bumper.just_pressed())
      Rotini.spin_power += 0.02; // increase power by 2%
    if (left_bumper.just_pressed())
      Rotini.spin_power -= 0.02; // decrease power by 2%
    if (right_up_arrow.is_held() || right_down_arrow.is_held())
      Rotini.spin_power = 0;     // 0% power, the robot does not spin
    else if (right_left_arrow.is_held() || right_right_arrow.just_released())
      Rotini.spin_power = 0.18;  // 18% power, "cruising power" is what I am using most of the time during a match
    else if (right_right_arrow.is_held())
      Rotini.spin_power = 1;     // 100% power, I press this to accelerate from stand still, to get out of pins, or deal extra damage

	// Controls what direction the motors are spinning in
    if (SWD_backward.is_held())
      Rotini.reversed = false;
    else if (SWD_forward.is_held())
      Rotini.reversed = true;

	// SWB has three states, the state of SWB controls what certain trim buttons do
    if (SWB_backward.is_held()) {  // trims accel radius
      if (left_left_arrow.just_pressed())
        Rotini.radius_trim += 0.002;  // 2 mm
      else if (left_right_arrow.just_pressed())
        Rotini.radius_trim -= 0.002;  // 2 mm
      else if (left_up_arrow.is_held())
        Rotini.radius_trim = 0;
    } else if (SWB_neutral.is_held()) {  // middle pos trims led offset
      if (left_left_arrow.just_pressed())
        Rotini.melty_led_offset_CW += 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_right_arrow.just_pressed())
        Rotini.melty_led_offset_CW -= 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_up_arrow.is_held())
        Rotini.melty_led_offset_CW = 0.698132 + 0.09 * 9.0;
    } else if (SWB_forward.is_held()) {  // up pos trims lag angle
      if (left_left_arrow.just_pressed())
        Rotini.melty_led_offset_CCW += 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_right_arrow.just_pressed())
        Rotini.melty_led_offset_CCW -= 0.1;  // 0.1 radians = 5.729 degrees
      else if (left_up_arrow.is_held())
        Rotini.melty_led_offset_CCW = 0.698132 - 0.09 * 6.0;
    }

    Rotini.meltyStateUpdate();

  } else if (Rotini.drive_mode == ARCADE) {
    Rotini.arcadeStateUpdate();

  } else if (Rotini.drive_mode == STOP) {
    Rotini.stopStateUpdate();

  } else {  //Rotini.drive_mode == DISCONNECTED
    Rotini.disconnectedStateUpdate();
  }

  // Actually commanding LEDs and motors
  foo.set_percent(Rotini.motor_power_foo);
  bar.set_percent(Rotini.motor_power_bar);
  
  digitalWrite(PIN_MELTY_LED, Rotini.melty_led);
  digitalWrite(PIN_STATUS_LED, Rotini.status_led);

  //send telemetry 10 times a second
  static uint32_t last_telem_ms = 0;
  if (millis() - last_telem_ms > 100) {
    float vin = read_voltage(PIN_SNS_VIN);
    //Serial.println(vin);
    Telemetry.add("vin", vin);  // volts
    send_telemetry(vin);
    last_telem_ms = millis();
  }

  Telemetry.add("drive_mode", Rotini.drive_mode);
  Telemetry.send();

  // Link diagnostics over USB, once a second
  static uint32_t last_status_ms = 0;
  if (millis() - last_status_ms >= 1000) {
    Telemetry.printStatus(Serial);
    if (!mag_ok) Serial.println("magnetometer: not found");
    last_status_ms = millis();
  }
}

/////////////////////Magnetometer Code/////////////////////////////////////////////////

// Gives up after a few tries so a missing magnetometer doesn't stop the robot from starting
bool beginMag() {
  int tries = 0;
  while (magnetometer.begin(PIN_MAGNETOMETER_CS) == false) {
    if (++tries >= 5) {
      Serial.println("MMC5983MA did not respond, running without it");
      return false;
    }
    Serial.println("MMC5983MA did not respond. Retrying...");
    delay(500);
    magnetometer.softReset();
    delay(500);
  }
  magnetometer.softReset();
  Serial.println("MMC5983MA connected");

  magnetometer.setFilterBandwidth(800);
  magnetometer.setContinuousModeFrequency(MAG_RATE_HZ);
  magnetometer.enableAutomaticSetReset();
  magnetometer.enableContinuousMode();
  return true;
}

// PIN_MAGNETOMETER_INT is the same pin as the CS, so the interrupt can't be
// used; this reads the latest sample once per continuous mode period instead.
// Returns true with the field in uT (full scale is +/-8 G = 800 uT).
bool updateMag(float &x, float &y, float &z) {
  static uint32_t last_read_us = 0;
  if (micros() - last_read_us < 1000000UL / MAG_RATE_HZ) return false;
  last_read_us = micros();

  uint32_t raw_x, raw_y, raw_z;
  if (!magnetometer.readFieldsXYZ(&raw_x, &raw_y, &raw_z)) return false;
  x = ((float)raw_x - 131072.0f) / 131072.0f * 800.0f;
  y = ((float)raw_y - 131072.0f) / 131072.0f * 800.0f;
  z = ((float)raw_z - 131072.0f) / 131072.0f * 800.0f;
  return true;
}

//////////////////// Helper functions ////////////////////////////////////////////////////////////////////

float LIS331_to_mps2(int16_t native_units) {
  // TODO: This is only correct in 400g mode. Should update when scale changes.
  return ((400.0f * native_units) / 2047.0f) * 9.80665f;
}

float channel_to_axis(unsigned int channel) {
  float axis = min(1.f, max(-1.f, (crsf.getChannel(channel) / 500.f) - 3));  // Map 1000-2000 channel value to -1..1
  if (fabs(axis) < 0.03) axis = 0;                                           // Apply deadzone
  return axis;
}

float read_voltage(int pin_sense_voltage) {
  float adc_vin = analogReadMilliVolts(pin_sense_voltage) * 0.001 * 8.21;
  return adc_vin;
}

void send_telemetry(float telem_voltage) {
  crsf_sensor_battery_t battery_data = {
    .voltage = htobe16((uint16_t)(telem_voltage * 10)),
  };
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery_data, sizeof(crsf_sensor_battery_t));

  // crsf_sensor_gps_t telemetry_data = {
  //   // Cram telemetry data into GPS struct
  //   .latitude = htobe32((int32_t)(RotiniPtr->centripetal_acceleration * 1000)),  // 10,000,000 big endian
  //   .longitude = htobe32((int32_t)(dRotiniPtr->angular_velocity * 1000)),        // 10,000,000 big endian
  //   .groundspeed = htobe16((uint16_t)(RotiniPtr->percent_power * 100)),          // 10 big endian
  //   .heading = htobe16((uint16_t)(RotiniPtr->accelerometer_radius_trim * 1000))  // 10 big endian
  // };
}