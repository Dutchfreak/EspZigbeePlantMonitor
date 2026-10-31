# Zigbee Plant Monitor

![Zigbee Plant Monitor showcase 1](<Pictures/ShowCase 1.jpg>)

ESP-IDF firmware for a Seeed XIAO ESP32-C6 that acts as a Zigbee coordinator for
plant sensors and displays their readings on a Waveshare 4.2-inch V2.1 e-paper
display (400 × 300, IL0398 controller).

![Zigbee Plant Monitor showcase 2](<Pictures/ShowCase 2.jpg>)

## Table of contents

- [Features](#features)
- [Parts](#parts)
- [Printing](#printing)
- [Wiring](#wiring)
- [Build and flash](#build-and-flash)
- [Assembly](#assembly)
  - [Step 1](#step-1)
  - [Step 2](#step-2)
  - [Step 3](#step-3)
- [Serial commands](#serial-commands)

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

## Parts

- Seeed Studio XIAO ESP32-C6
- Waveshare 4.2-inch, 400 × 300 E-Ink display module
- 4 cap-head screws:
  - 2 × M3 × 10 mm
  - 2 × M3 × 16 mm
- 4 printed parts:
  - Front plate
  - Back plate
  - 2 feet
- 1 USB cable (make sure the USB-C plug is not too thick)

## Printing

The 3D-printed parts are in the `3d Parts` folder. I used a generic white
filament for the body. For the logo, I used ColorFabb StoneFil in Moss Green and
Red Brick, along with a generic light-blue PLA.

The logo consists of four separate parts, making it easy to assign different
colors in your slicer.

## Wiring

The only soldering required is connecting the display to the ESP32-C6. The
display connects directly over SPI:

| Display signal | ESP32-C6 GPIO |
|---|---:|
| BUSY | 21 |
| RESET | 22 |
| DC | 23 |
| CS | 2 |
| SCK | 19 |
| MOSI | 18 |

The external antenna switch is configured on GPIO3 and GPIO14 in firmware.

## Build and flash

Install ESP-IDF with the ESP32-C6 target and its required tools. From the project
directory, with the ESP-IDF environment enabled:

```sh
idf.py set-target esp32c6
idf.py build
idf.py -p <serial-port> flash monitor
```

Alternatively, use the ESP-IDF Explorer buttons in VS Code.

The project uses ESP-IDF's component manager for the ESP Zigbee library and
LVGL. The checked-in `sdkconfig.defaults` enables the USB Serial/JTAG console
and Zigbee coordinator configuration.

## Assembly

First, solder the display wires to the ESP32-C6 according to the wiring table:

| Display signal | ESP32-C6 GPIO |
|---|---:|
| BUSY | 21 |
| RESET | 22 |
| DC | 23 |
| CS | 2 |
| SCK | 19 |
| MOSI | 18 |


### Step 1

![Assembly step A](<Pictures/Assembly A.jpg>)

Connect a USB cable to the ESP32-C6 and thread it through the strain relief in
the housing.

Mount the ESP32-C6 upside down, with the LEDs facing the back, into the clip. It
should snap into place. If it is difficult to insert, angle the top in first,
then push the board into the clip.

### Step 2

![Assembly step B](<Pictures/Assembly B.jpg>)

Route the wires so they do not interfere with the screen's screw holes, then
place the screen in the housing. Secure it with two M3 × 10 mm screws.

![Assembly step C](<Pictures/Assembly C.jpg>)

### Step 3

![Assembly step D](<Pictures/Assembly D.jpg>)

Take the feet and insert an M3 × 16 mm screw through the top hole. Screw it into
the top hole of the screen, making sure the lower hole in the foot fits over the
lower screw.

![Assembly step E](<Pictures/Assembly E.jpg>)

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
