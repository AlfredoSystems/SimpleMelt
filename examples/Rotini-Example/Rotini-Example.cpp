#include <Arduino.h>
#include <SPI.h>

#include "SimpleMelt.h"
#include <HardwareSerial.h>
#include <ArduinoCRSF.h>
#include "SparkFun_LIS331.h"
#include "esc.h"

const int PIN_SPI_SCK = 17;
const int PIN_SPI_MISO = 1;
const int PIN_SPI_MOSI = 14;
const int PIN_ACCELEROMETER_CS = 11;
const int PIN_ACCELEROMETER_INT = 12;
const int PIN_MAGNETOMETER_CS = 2;
const int PIN_MAGNETOMETER_INT = 13;
const int PIN_ELRS_UART_RX = 7;
const int PIN_ELRS_UART_TX = 8;
const int PIN_MOTOR_FOO = 34;
const int PIN_MOTOR_BAR = 35;
const int PIN_MELTY_LED = 42;
const int PIN_STATUS_LED = 41;
const int PIN_SNS_VIN = 3;

SimpleMelt Rotini;

HardwareSerial crsfSerial(1);
ArduinoCRSF elrs;

LIS331 accelerometer;

ESC foo;
ESC bar;

void setup() {
    
    Serial.begin(115200);
    
    pinMode(PIN_STATUS_LED, OUTPUT);
    pinMode(PIN_MELTY_LED, OUTPUT);
    pinMode(PIN_ACCELEROMETER_CS, OUTPUT);
    pinMode(PIN_MAGNETOMETER_CS, OUTPUT);   

    digitalWrite(PIN_STATUS_LED, LOW);  
    digitalWrite(PIN_MELTY_LED, LOW);
    digitalWrite(PIN_ACCELEROMETER_CS, HIGH); // Make CS high
    digitalWrite(PIN_MAGNETOMETER_CS, HIGH); // Make CS high
    
    crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_ELRS_UART_RX, PIN_ELRS_UART_TX);
    if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");
    crsf.begin(crsfSerial);
    
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

    accelerometer.setSPICSPin(PIN_ACCELEROMETER_CS);
    accelerometer.begin(LIS331::USE_SPI); // Selects the bus to be used and sets
    accelerometer.setODR(accelerometer.DR_1000HZ);
    accelerometer.setFullScale(accelerometer.HIGH_RANGE);

    foo.begin(PIN_MOTOR_FOO, 0);
    bar.begin(PIN_MOTOR_BAR, 1);
}

void loop() {
    crsf.update();

    if (crsf.isLinkUp()) {
        if (Rotini.front_left_switch_backward.is_held()) Rotini.drive_mode = STOP;
        else if (Rotini.front_left_switch_neutral.is_held()) Rotini.drive_mode = ARCADE;
        else if (Rotini.front_left_switch_forward.is_held()) Rotini.drive_mode = MELTY;
    } else {
        Rotini.drive_mode = NO_CONNECTION;
    }

   Rotini.axis_left_x = channel_to_axis(4));
   Rotini.axis_left_y = channel_to_axis(3));
   Rotini.axis_right_x = channel_to_axis(1));
   Rotini.axis_right_y = channel_to_axis(2));
   
   Rotini.left_bumper.update(elrs.get_channel(11));
   Rotini.right_bumper.update(elrs.get_channel(11));
   
   Rotini.left_left_arrow.update(elrs.get_channel(10));
   Rotini.left_right_arrow.update(elrs.get_channel(10));
   Rotini.left_up_arrow.update(elrs.get_channel(9));
   Rotini.left_down_arrow.update(elrs.get_channel(9));
   Rotini.right_left_arrow.update(elrs.get_channel(7));
   Rotini.right_right_arrow.update(elrs.get_channel(7));
   Rotini.right_up_arrow.update(elrs.get_channel(8));
   Rotini.right_down_arrow.update(elrs.get_channel(8));
   // SWA
   Rotini.front_left_switch_backward.update(elrs.get_channel(6));
   Rotini.front_left_switch_neutral.update(elrs.get_channel(6));
   Rotini.front_left_switch_forward.update(elrs.get_channel(6));
   // SWD
   Rotini.back_right_switch_backward.update(elrs.get_channel(11));
   Rotini.back_right_switch_forward.update(elrs.get_channel(11));
   
   if (Rotini.drive_mode == MELTY) {
      int16_t x, y, z;
      accelerometer.readAxes(x, y, z);
      Rotini.accelerometer_x = accelerometer.convertToG(6,x); // The convertToG() function
      Rotini.accelerometer_y = accelerometer.convertToG(6,y); // accepts as parameters the
      Rotini.accelerometer_z = accelerometer.convertToG(6,z); // raw value and the current
    
      if (Rotini.right_bumper.just_pressed()) this->spin_power += 0.02;
      if (Rotini.left_bumper.just_pressed()) this->spin_power -= 0.02;
      this->spin_power = constrain(this->spin_power, 0, 1);

      if (Rotini.left_up_arrow.is_held()) this->spin_power = 0;
      else if (Rotini.right_left_arrow.is_held()) this->spin_power = GOOD_POWER;
      else if (Rotini.right_right_arrow.is_held()) this->spin_power = 1;

      if (Rotini.back_right_switch_backward.is_held()) this->reversed = false;
      else if (Rotini.back_right_switch_forward.is_held()) this->reversed = true;

      // TODO: Reset to pre-attack spin_power instead of good spin_power?
      if (Rotini.right_right_arrow.just_released()) this->spin_power = GOOD_POWER;

      if (Rotini.left_left_arrow.just_pressed()) this->radius_trim += 0.001;
      else if (Rotini.left_right_arrow.just_pressed()) this->radius_trim -= 0.001;
      else if (Rotini.left_up_arrow.is_held()) this->radius_trim = 0;
      
      
      Rotini.meltyStateUpdate();


    } else if (Rotini.drive_mode == ARCADE) {
       //get stick input?
        Rotini.arcadeStateUpdate();
        
    } else if (Rotini.drive_mode == STOP){
        Rotini.stopStateUpdate();

    } else {  //state is DISCONNECTED
        Rotini.disconnectedStateUpdate();
    }
    
    ////////////////////// Actually updating LEDs and motors //////////////////////////////////////////

    foo.set_percent(Rotini.motor_power_foo);
    bar.set_percent(Rotini.motor_power_bar);

    digitalWrite(PIN_MELTY_LED, Rotini.melty_led);
    digitalWrite(PIN_STATUS_LED, Rotini.status_led);
    
    
}


 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


float lerp(float val, float in_min, float in_max, float out_min, float out_max) { return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; };
float constrain(float x, float min, float max) { return x > max ? max : (x < min ? min : x); };
float magnitude(float x, float y) { return sqrt(x * x + y * y); };
float modulo(float x, float n) { return x - floor(x / n) * n; }; // Mathematical modulo




float channel_to_axis(unsigned int channel) {
    float axis = min(1.f, max(-1.f, (crsf.getChannel(channel) / 500.f) - 3)); // Map 1000-2000 channel value to -1..1
    if (fabs(axis) < 0.03) axis = 0; // Apply deadzone
    return axis;
}


  // void read_voltage(Rotini* RotiniPtr) {
      // // (VIN * ADC voltage / ADC resolution) * ((R1 + R2) / R2)
      // RotiniPtr->voltage = ((float)analogRead(sns_vin_pin) * 3.3 / 8192) * ((15000 + 2200) / 2200);
  // }



// void ELRS::send_diagnostics(Rotini* RotiniPtr) {
    // crsf_sensor_battery_t battery_data = {
        // .voltage = htobe16((uint16_t)(RotiniPtr->voltage * 10)),
    // };
    // crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &battery_data, sizeof(crsf_sensor_battery_t));
    
    // crsf_sensor_gps_t telemetry_data = { // Cram telemetry data into GPS struct
        // .latitude = htobe32((int32_t)(RotiniPtr->centripetal_acceleration * 1000)), // 10,000,000 big endian
        // .longitude = htobe32((int32_t)(dRotiniPtr->angular_velocity * 1000)), // 10,000,000 big endian
        // .groundspeed = htobe16((uint16_t)(RotiniPtr->percent_power * 100)),  // 10 big endian
        // .heading = htobe16((uint16_t)(RotiniPtr->accelerometer_radius_trim * 1000))  // 10 big endian
    // };
    
// }

