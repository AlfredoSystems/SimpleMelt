# Rotini-Firmware
 
PlatformIO setup: `pio project init -b featheresp32-s2`. Clone [AlfredoSystem/ArduinoCRSF](https://github.com/AlfredoSystems/ArduinoCRSF) to `lib/`.

Put the following in `platformio.ini`.

```
[env:adafruit_metro_esp32s2]
board = adafruit_metro_esp32s2
platform = espressif32
framework = arduino
```