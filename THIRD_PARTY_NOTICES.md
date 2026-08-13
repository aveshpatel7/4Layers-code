# Third-party components

This firmware uses third-party components/libraries:

- Arduino-ESP32 by Espressif — LGPL-2.1-or-later
- ArduinoJson 6.21.6 by Benoit Blanchon — MIT
- PubSubClient 2.8 by Nick O'Leary — MIT

The project downloads the Arduino-ESP32 and ArduinoJson components through the ESP-IDF
Component Manager and downloads the pinned PubSubClient v2.8 source during CMake configure.
