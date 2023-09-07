#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>

#include "SimpleMelt.h"
#include "SimpleMeltUtility.h"

#include "ArduinoCRSF.h"
#include "SparkFun_LIS331.h"

const int PIN_SPI_SCK = 17;
const int PIN_SPI_MISO = 1;
const int PIN_SPI_MOSI = 14;
const int PIN_ACCELEROMETER_CS = 11;
const int PIN_ACCELEROMETER_INT = 12;
const int PIN_MAGNETOMETER_CS = 2;
const int PIN_MAGNETOMETER_INT = 13;
const int PIN_CRSF_RX = 7;
const int PIN_CRSF_TX = 8;
const int PIN_MOTOR_FOO = 34;
const int PIN_MOTOR_BAR = 35;
const int PIN_MELTY_LED = 42;
const int PIN_STATUS_LED = 41;
const int PIN_SNS_VIN = 3;

SimpleMelt Rotini;

HardwareSerial crsfSerial(1);
ArduinoCRSF crsf;

LIS331 accelerometer;

ESC foo;
ESC bar;

void setup() {
  //get rid of motor lag angle??????????
  //radius trim still needs: CW-16L, CCW-16L
  //motor_lag_angle still needs: CW-9L, CCW-6R
  //get mag data
  Rotini.melty_led_offset = 0.698132; // radians (CCW is positive)
  Rotini.turn_speed = 1.1; // radians per second
  Rotini.motor_lag_angle = -2.35619; // radians
  Rotini.accelerometer_radius = 0.090; // meters
  Rotini.radius_trim = 0.032; // meters

  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_MELTY_LED, OUTPUT);
  pinMode(PIN_ACCELEROMETER_CS, OUTPUT);
  pinMode(PIN_MAGNETOMETER_CS, OUTPUT);

  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_MELTY_LED, LOW);
  digitalWrite(PIN_ACCELEROMETER_CS, HIGH);
  digitalWrite(PIN_MAGNETOMETER_CS, HIGH);

  Serial.begin(115200);

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_CRSF_RX, PIN_CRSF_TX);
  crsf.begin(crsfSerial);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  accelerometer.setSPICSPin(PIN_ACCELEROMETER_CS);
  accelerometer.begin(LIS331::USE_SPI);  // Selects the bus to be used and sets
  accelerometer.setODR(accelerometer.DR_1000HZ);
  accelerometer.setFullScale(accelerometer.HIGH_RANGE); //400g range

  foo.begin(PIN_MOTOR_FOO, 0);
  bar.begin(PIN_MOTOR_BAR, 1);
  foo.setReversed(true);
  bar.setReversed(false);
}

void loop() {
  
  crsf.update();

  Rotini.rotation = channel_to_axis(1); //axis_right_x
  Rotini.throttle = channel_to_axis(3); //axis_left_y
  //axis_right_y = channel_to_axis(2);
  //axis_left_x = channel_to_axis(4);

  //bumpers
  static Button left_bumper; left_bumper.update(crsf.getChannel(11), 1000);
  static Button right_bumper; right_bumper.update(crsf.getChannel(11), 2000);
  //trim buttons
  static Button left_left_arrow; left_left_arrow.update(crsf.getChannel(10), 1000);
  static Button left_right_arrow; left_right_arrow.update(crsf.getChannel(10), 2000);
  static Button left_up_arrow; left_up_arrow.update(crsf.getChannel(9), 2000);
  static Button left_down_arrow; left_down_arrow.update(crsf.getChannel(9), 1000);
  static Button right_left_arrow; right_left_arrow.update(crsf.getChannel(7), 1000);
  static Button right_right_arrow; right_right_arrow.update(crsf.getChannel(7), 2000);
  static Button right_up_arrow; right_up_arrow.update(crsf.getChannel(8), 2000);
  static Button right_down_arrow; right_down_arrow.update(crsf.getChannel(8), 1000);
  // SWA - Zorro Switch B
  static Button SWA_backward; SWA_backward.update(crsf.getChannel(6), 1000);
  static Button SWA_neutral; SWA_neutral.update(crsf.getChannel(6), 1500);
  static Button SWA_forward; SWA_forward.update(crsf.getChannel(6), 2000);
  // SWD - Zorro Switch F
  static Button SWD_backward; SWD_backward.update(crsf.getChannel(5), 1000);
  static Button SWD_forward; SWD_forward.update(crsf.getChannel(5), 2000);
  // SWB - Zorro Switch C
  static Button SWB_backward; SWB_backward.update(crsf.getChannel(12), 1000);
  static Button SWB_neutral; SWB_neutral.update(crsf.getChannel(12), 1500);
  static Button SWB_forward; SWB_forward.update(crsf.getChannel(12), 2000);
  
  if (crsf.isLinkUp()) {
    if (SWA_backward.is_held()) Rotini.drive_mode = STOP;
    else if (SWA_neutral.is_held()) Rotini.drive_mode = ARCADE;
    else if (SWA_forward.is_held()) Rotini.drive_mode = MELTY;
  } else {
    Rotini.drive_mode = NO_CONNECTION;
  }

  if (Rotini.drive_mode == MELTY) {
    int16_t x, y, z;
    accelerometer.readAxes(x, y, z);
    Rotini.accelerometer_x = 0; //LIS331_to_mps2(x);
    Rotini.accelerometer_y = 0; //LIS331_to_mps2(y);
    Rotini.accelerometer_z = LIS331_to_mps2(z);

    if (right_bumper.just_pressed())
      Rotini.spin_power += 0.02;
    if (left_bumper.just_pressed())
      Rotini.spin_power -= 0.02;

    if (right_up_arrow.is_held() || right_down_arrow.is_held())
      Rotini.spin_power = 0;
    else if (right_right_arrow.is_held())
      Rotini.spin_power = 1;
    else if (right_left_arrow.is_held() || right_right_arrow.just_released()) 
      Rotini.spin_power = 0.18; //this is "cruising power"

    if (SWD_backward.is_held())
      Rotini.reversed = false;
    else if (SWD_forward.is_held())
      Rotini.reversed = true;

    if(SWB_backward.is_held()){  //default down trims accel radius
      if (left_left_arrow.just_pressed())
        Rotini.radius_trim += 0.002;
      else if (left_right_arrow.just_pressed())
        Rotini.radius_trim -= 0.002;
      else if (left_up_arrow.is_held())
        Rotini.radius_trim = 0;
    } else if(SWB_neutral.is_held()) { //middle pos trims led offset
      if (left_left_arrow.just_pressed())
        Rotini.melty_led_offset += 0.09; //about 5 degrees
      else if (left_right_arrow.just_pressed())
        Rotini.melty_led_offset -= 0.09; //about 5 degrees
      else if (left_up_arrow.is_held())
        Rotini.melty_led_offset = 0.698132;
    } else if(SWB_forward.is_held()) { //up pos trims lag angle
      if (left_left_arrow.just_pressed())
        Rotini.motor_lag_angle += 0.09; //about 5 degrees
      else if (left_right_arrow.just_pressed())
        Rotini.motor_lag_angle -= 0.09; //about 5 degrees
      else if (left_up_arrow.is_held())
        Rotini.motor_lag_angle = -2.35619;
    }
    // Serial.print(crsf.getChannel(12));  Serial.print(" ");
    // Serial.print(Rotini.radius_trim);  Serial.print(" ");
    // Serial.print(Rotini.melty_led_offset);  Serial.print(" ");
    // Serial.print(Rotini.motor_lag_angle);  Serial.println(" ");

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

  uint64_t last_telem_ms;
  if(last_telem_ms - millis() > 500){ //send telemetry twice a second
    float vin = read_voltage(PIN_SNS_VIN);
    //Serial.println(vin);
    send_telemetry(vin);
    last_telem_ms = millis();    
  }

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
  uint16_t raw_adc_vin = analogRead(pin_sense_voltage);
  //Serial.print(raw_adc_vin);  Serial.print(" ");

  const static uint16_t adc_vals[]     = { 0, 1962,2380,2785,3200,3610,4000,4410,4795,5164,5500,5810,6080,6350,6580,6785 ,8573 };
  const static uint16_t voltage_vals[] = { 0, 5,6,7,8,9,10,11,12,13,14,15,16,17,18,19 ,27 };
  const static uint8_t len = sizeof(voltage_vals) / sizeof(voltage_vals[0]);

  if (raw_adc_vin < adc_vals[0]) return voltage_vals[0];
  for (uint8_t i = 0; i < len - 1; i++) {
      if (raw_adc_vin < adc_vals[i+1]) {
          return lerp(raw_adc_vin, adc_vals[i], adc_vals[i+1], voltage_vals[i], voltage_vals[i+1]);
      }
  }
  return voltage_vals[len-1];
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