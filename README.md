# Aquarium_Air_Pump_Controller

**Short description:** Simple electronics for controlling aquarium air pump speed with some additional functions built around the WCH **CH32V003F4U6** MCU

![Pump control](https://github.com/TilenTinta/Aquarium_Air_Pump_Controller/blob/main/Images/PCB_3D.png)  

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
  - [Hardware](#hardware)
  - [Software ](#software)
- [Mounting](#mounting)
- [Flashing Firmware](#flashing-firmware)
- [Debugging](#debugging)
  - [LED Status Codes](#led-status-codes)


---

## Overview

This controller is a very compact solution for controling aquarium air pump speed. It is meant to be mounted on the back side of cheap chinese pumps that uses DC brushed motors like **PXYP370** and are powered from USB. By adjusting potenciometer on PCB you can tune pump speed and by that the amount of air provided in aquarium/filter. If you want to control this pump from external device like **ESP32** you can connect it over UART pads. This allow you to monitor and control the pump.

---

## Features

### Hardware

- **MCU:** WCH **CH32V003F4U6** with SWIO. 
- **Power:** Directly from 5V - USB.
- **Power regulation:** N-Channel MOSFET AP30N03DF.
- **Adjustment:** Duty cycle with potenciometer
- **Sensors:** Voltage IN, temperature.
- **Indicators/Controls:** Status LED and UART. 

### Software

- **Firmware:** C, developed in **MounRiver Studio IDE**. 
- **Updates:** **Custom bootloader** supports USB firmware updates. 

---

## Mounting

To mount controler on air pump you need to first remove silicon housing from the pump. Desolder both wires and bleed resistor from the motor contacts and cleen them. Then solcer the wires from USB cable to PCB. Correctly oriented pcb can be then soldered directly on motor contacts.

[Image when assembled - Comming soon...]

---

## Flashing Firmware

- Open the project in **MounRiver Studio IDE**, compile, and **upload** to the **CH32V003F4U6** MCU. 

> The board also includes a **custom bootloader** for **USB firmware updates** when supported by the desktop toolchain. 

---

## Debugging

### LED Status Codes  

| LED Pattern     | Meaning                   |
|-----------------|---------------------------|
| Blinking **red**  | Device initialization   |
| Solid **red**     | Device error            |


