# BlynkDHTdallasPIRbeep

## Description
This code is written for use with an ESP8266, a DHTXX (humidity), a DS18B01 (temp), a logic output PIR sensor (motion), and a logic input beeper. Upon bootup it connects to the Blynk server and periodically uploads temp and humidity based on continuously averaged sensor data. It continuously reads the PIR sensor and displays it's status on an app LED widget. If the PIR goes active and an app "ARM" button is enabled, the code will beep the beeper twice and send a notification.

Firmware is Browser OTA upgradeable, and as non-blocking as possible. However no modifications were done to the included libraries, which may contain some lines of blocking code.

## Blynk Virtual Pins
V-pin | Data
--- | ---------------------
V0 | Firmware Upload Button [0 = normal, 1 = upload]
V1 | Alarm Arm Button
V2 | Alarm Status LED
V3 | Alarm Trigger Count
V4 | Temperature 1 - Dallas [F]
V5 | Relative Humidity [%]
V6 | Heat Index [F]
V7 | Dew Point [F]

## ESP8266 Pinout
ESP pin | Connect to
------ | -------------------
io13 | PIR output pin
io12 | DHT sensor data pin
io14 | Dallas temp sensor (non-parasitic power)
io16 | Beeper (w/ NPN... high = beep)
io0 | 4k7 High (Low = UART flash bootloader)
io2 | 4k7 High
io15 | 4k7 Low
EN | 4k7 High
RST | 4k7 High
