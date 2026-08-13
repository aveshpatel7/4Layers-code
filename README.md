# 4Layers ESP-IDF Firmware

## What this project is

This is the ESP-IDF delivery version of the original Arduino firmware
`4layers_Letest_Provisioning_Firmware.ino`.

The application logic is intentionally kept in Arduino-ESP32 APIs, but Arduino-ESP32
is integrated as an ESP-IDF component. This means the project is built with the ESP-IDF
build system and can be opened, built and flashed from VS Code + Espressif ESP-IDF.

This is **not** a full rewrite to native ESP-IDF APIs. That was intentionally avoided to
minimize behavior changes from the original working firmware.

## Target

- ESP32 (classic ESP32 / `esp32` target)
- ESP-IDF 5.3 through 6.0
- Arduino-ESP32 3.3.10

## Requirements

1. VS Code
2. Espressif ESP-IDF extension
3. ESP-IDF installed and its environment activated
4. Internet access for the first build (Arduino-ESP32, ArduinoJson and PubSubClient are downloaded)

## Build in VS Code

1. Open **this folder** in VS Code.
2. Open an **ESP-IDF terminal**.
3. In the project folder run:

```powershell
idf.py set-target esp32
idf.py build
```

The first configure/build downloads the pinned dependencies automatically.

## Flash

Find the ESP32 COM port, for example `COM3`, then:

```powershell
idf.py -p COM3 flash monitor
```

Replace `COM3` with the actual port.

To stop the serial monitor, press `Ctrl+]`.

## Project structure

```text
4layers_ESP_IDF/
├── main/
│   ├── main.cpp
│   ├── CMakeLists.txt
│   └── idf_component.yml
├── partitions.csv
├── sdkconfig.defaults
├── CMakeLists.txt
└── README.md
```

## Important dependency note

PubSubClient v2.8 is not pulled from the ESP Component Registry in this project. The
`main/CMakeLists.txt` downloads the pinned upstream `v2.8` source automatically during
CMake configuration.

After the first successful configure, the downloaded files are located at:

```text
main/.deps/PubSubClient/
```

## Firmware behavior

The original firmware behavior is preserved, including Wi-Fi provisioning, BLE setup,
MQTT control, NVS state storage, web setup portal, factory reset and HTTP OTA update.

## Security note

The original source contains MQTT connection credentials. Treat this project as
confidential unless those credentials have been intentionally provisioned for the client.
Rotate the credentials before publishing the source to a public repository.
