# Zigbee Plant Monitor

ESP-IDF firmware for a Seeed XIAO ESP32-C6 that acts as a Zigbee coordinator for
plant sensors and displays their readings on a Waveshare 4.2-inch V2.1 e-paper
display (400 × 300, IL0398 controller).

## Features

- Opens and closes the Zigbee network for pairing from the serial console.
- Receives supported Tuya sensor datapoints:
  - Soil moisture (DP3)
  - Temperature (DP5)
  - Battery (DP15)
  - Air humidity (DP109)
- Displays joined sensors and their latest readings. A warning icon appears
  beside soil moisture readings below 20%.
- Marks readings stale after a configurable timeout.
- Lists devices with their last-heard time and last recorded RSSI; supports
  renaming and removing devices from the local list, with a Zigbee leave request.
- Uses the XIAO ESP32-C6 external antenna.
- Shows a four-frame plant growth indicator when the display refreshes.

## Hardware

The display is connected directly over SPI:

| Display signal | ESP32-C6 GPIO |
|---|---:|
| BUSY | 21 |
| RESET | 22 |
| DC | 23 |
| CS | 2 |
| SCK | 19 |
| MOSI | 18 |

The external antenna switch is configured on GPIO3 and GPIO14 in firmware.

## Serial commands

Enter a command followed by Enter:

| Command | Description |
|---|---|
| `Scan` | Open the Zigbee network for pairing until stopped. |
| `Stop` | Close the network to new pairings. |
| `Devices` | List joined devices, including last heard and last recorded RSSI. |
| `Rename <number>` | Rename a device from the current `Devices` list. |
| `Remove <number>` | Remove a device locally and ask it to leave the network. Confirm with `YES`. |
| `Timeout` | Show the sensor stale timeout. |
| `Timeout <minutes>` | Set and save the stale timeout (1–10,080 minutes; default 60). |
| `Refresh` | Show the display refresh interval. |
| `Refresh <minutes>` | Set and save the display refresh interval (1–10,080 minutes; default 1). |

The display updates when sensor or device data changes, subject to the configured
minimum refresh interval. Since this is an e-paper display, frequent full
refreshes can increase display wear and power use.

Device names and timeout/refresh settings are stored in NVS. Zigbee network
configuration is also stored there by the Zigbee stack. If NVS initialization
fails at startup, the firmware erases NVS and retries; this resets the stored
network and settings.

## Build and flash

Install ESP-IDF with the ESP32-C6 target and its required tools. From the project
directory, with the ESP-IDF environment enabled:

```sh
idf.py set-target esp32c6
idf.py build
idf.py -p <serial-port> flash monitor
```

The project uses ESP-IDF's component manager for the ESP Zigbee library and
LVGL. The checked-in `sdkconfig.defaults` enables the USB Serial/JTAG console
and Zigbee coordinator configuration.
