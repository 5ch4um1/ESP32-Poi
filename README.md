# ESP32-Poi

ESP32-C3 firmware compatible with [Open Pixel Poi](https://openpixelpoi.com) boards, designed for Poi and other flow props.

This firmware provides high-performance POV (Persistence of Vision) rendering, support for multiple pattern banks/slots, procedural animations, and a low-latency BLE streaming mode for real-time control.

## Getting Started

### Prerequisites
* **ESP-IDF v5.5.1**: This project is tested and optimized for the latest ESP-IDF.
* **Hardware**: ESP32-C3 based Open Pixel Poi hardware.

### Build & Flash
Follow the ESP-IDF setup instructions, then run:
```bash
idf.py build
idf.py -p [PORT] flash monitor
```

## How it Works

The firmware utilizes several key ESP32-C3 features:
* **LittleFS**: Used for high-speed access to stored pattern files (`.bin`). Patterns are organized into 3 banks with 5 slots each.
* **NimBLE Stack**: A lightweight Bluetooth Low Energy stack used for configuration and the high-speed streaming protocol.
* **RMT Peripheral**: Used to drive the WS2812B LED strip with precise timing, ensuring flicker-free rendering.
* **DMA Ring Buffer**: Specifically for streaming mode, allowing the device to buffer incoming frames to mitigate BLE jitter.

## Usage & Controls

The device is controlled via a single button (GPIO 3):

* **Single Click**: Cycle to the next pattern slot (0-4).
* **Hold (0.5s)**: Display current battery voltage on the LED strip.
* **Double Click + Hold**: Select Bank (Red=Bank 0, Green=Bank 1, Blue=Bank 2, White=Shuffle All).
* **Triple Click + Hold**: Select Brightness level.
* **Quadruple Click + Hold**: Select POV Speed preset.
* **Long Hold (2s)**: Power Off (Deep Sleep). The device can be woken up by clicking the button.

## Streaming Mode

The streaming mode allows real-time control of every LED on the strip over BLE.

* **Protocol**: Uses a custom 480-byte MTU protocol to send raw RGB data.
* **Performance**: Supports up to 500Hz refresh rates with a high-priority task synchronized to an internal timer.
* **Web App**: Test the streaming mode with one or two poi at [5ch4um1.es/oppstream2.html](https://5ch4um1.es/oppstream2.html).

### Compatible Controllers
* **Smartwatch Controller**: A dedicated smartwatch firmware (ESP32-C6) for controlling these poi can be found here: [waveshare-amoled-2.06-esp32c6.ble-esp32-poi-streaming](https://github.com/5ch4um1/waveshare-amoled-2.06-esp32c6.ble-esp32-poi-streaming).

## Gemini Skill: FWAgent

This repository includes a specialized Gemini CLI skill, **FWAgent**, designed to assist in the development, debugging, and optimization of this firmware.

*   **Firmware Expert**: Provides deep knowledge of ESP-IDF v5.x and the RISC-V ESP32-C3 architecture.
*   **Automated Workflows**: Streamlines building, flashing, and monitoring the firmware with the correct environment settings.
*   **Hardware Mapping**: Includes built-in references for ESP32-C3 GPIOs, peripherals (RMT, ADC, NimBLE), and strapping pins.
*   **Optimization Support**: Offers guidance on high-performance LED rendering and low-latency task scheduling for POI applications.

To use it with Gemini CLI, ensure the skill is activated within the `poi_firmware` directory.

## Battery Management
The firmware monitors battery voltage via ADC.
* **Warning Level (< 3.45V)**: Enters Emergency SOS mode (pulsing red SOS signal).
* **Critical Level (< 3.40V)**: Automatic shutdown to protect the LiPo battery.
