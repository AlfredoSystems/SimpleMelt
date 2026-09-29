#ifndef MAGLOG_H
#define MAGLOG_H

#include <Arduino.h>

// One logged sample. Timestamps are micros() values, which are uint32_t and
// wrap every ~71 minutes -- 64-bit fields here just cost RAM for no range.
typedef struct {
  uint32_t log_timestamp;
  uint32_t angle_timestamp;
  uint32_t mag_raw_x;
  uint32_t mag_raw_y;
  uint32_t mag_raw_z;
  float angle;
  int16_t accel_raw_z;
} magLogPacket_t;  // 26 bytes -> 28 padded

// Ring-free fixed-capacity log with a non-blocking serial dump.
//
// The dump is the important part: at 115200 baud a full 2000-row dump is
// several seconds of dead time, and the bot is still spinning during it.
// Call service() every loop() and it emits a few rows per pass instead of
// parking inside the print.
template <uint16_t CAPACITY>
class MagLog {
 public:
  // Append a sample. Silently ignored once full, and while a dump is running
  // (so the buffer can't shift under the dump).
  void add(uint32_t log_timestamp_us, int16_t accel_raw_z,
           uint32_t mag_raw_x, uint32_t mag_raw_y, uint32_t mag_raw_z,
           uint32_t angle_timestamp_us, float angle) {
    if (_count >= CAPACITY || _dumping) return;

    magLogPacket_t &p = _log[_count];
    p.log_timestamp = log_timestamp_us;
    p.angle_timestamp = angle_timestamp_us;
    p.mag_raw_x = mag_raw_x;
    p.mag_raw_y = mag_raw_y;
    p.mag_raw_z = mag_raw_z;
    p.angle = angle;
    p.accel_raw_z = accel_raw_z;
    _count++;
  }

  void clear() {
    _count = 0;
    _dump_index = 0;
    _dumping = false;
  }

  uint16_t count() const { return _count; }
  uint16_t capacity() const { return CAPACITY; }
  bool is_full() const { return _count >= CAPACITY; }
  bool is_dumping() const { return _dumping; }

  // Ask for a dump. Returns to idle on its own once every row is out.
  void startDump() {
    if (_count == 0 || _dumping) return;
    _dump_index = 0;
    _dumping = true;
    printHeader();
  }

  void cancelDump() { _dumping = false; }

  // Call every loop(). Emits up to rows_per_call rows, then returns.
  // Returns true while a dump is still in progress.
  bool service(uint8_t rows_per_call = 8) {
    if (!_dumping) return false;

    for (uint8_t i = 0; i < rows_per_call && _dump_index < _count; i++) {
      printRow(_log[_dump_index]);
      _dump_index++;
    }

    if (_dump_index >= _count) {
      Serial.println(F("# end of log"));
      _dumping = false;
    }
    return _dumping;
  }

  // Blocking variant, for use from setup() or a known-safe state only.
  void dumpBlocking() {
    printHeader();
    for (uint16_t i = 0; i < _count; i++) printRow(_log[i]);
    Serial.println(F("# end of log"));
  }

 private:
  void printHeader() {
    Serial.print(F("# maglog rows="));
    Serial.println(_count);
    Serial.println(F("log_timestamp,accel_raw_z,mag_raw_x,mag_raw_y,mag_raw_z,angle_timestamp,angle"));
  }

  void printRow(const magLogPacket_t &p) {
    Serial.print(p.log_timestamp); Serial.print(',');
    Serial.print(p.accel_raw_z);   Serial.print(',');
    Serial.print(p.mag_raw_x);     Serial.print(',');
    Serial.print(p.mag_raw_y);     Serial.print(',');
    Serial.print(p.mag_raw_z);     Serial.print(',');
    Serial.print(p.angle_timestamp); Serial.print(',');
    Serial.println(p.angle, 4);
  }

  magLogPacket_t _log[CAPACITY];
  uint16_t _count = 0;
  uint16_t _dump_index = 0;
  bool _dumping = false;
};

#endif
