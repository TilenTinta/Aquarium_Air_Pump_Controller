# Aquarium_Air_Pump_Controller

Simple electronics for controlling aquarium air pump speed with some additional functions built around the WCH **CH32V003F4U6** MCU

![Pump control pcb](https://github.com/TilenTinta/Aquarium_Air_Pump_Controller/blob/main/Images/PCB_3D.jpg)  

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
  - [Hardware](#hardware)
  - [Software](#software)
- [Mounting](#mounting)
- [Flashing Firmware](#flashing-firmware)
- [Working device](#working-device)
  - [Working Principle](#working-principle)
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
- **Updates:** **Custom bootloader** supports UART firmware updates. 

---

## Mounting

To mount controller on air pump you first need to remove silicon housing from the pump. Desolder both wires and bleed resistor from the motor contacts and clean them. Then solder the wires from USB cable to PCB. Correctly oriented pcb can be then soldered directly on motor contacts.

![Pump control full](https://github.com/TilenTinta/Aquarium_Air_Pump_Controller/blob/main/Images/Mount-step_4.jpg)  

---

## Flashing Firmware

- Open the project in **MounRiver Studio IDE**, compile, and **upload** to the **CH32V003F4U6** MCU. 

> The board also includes a **custom bootloader** for **UART firmware updates** when supported by the desktop toolchain. 

---

## Working device

When USB cable is pluging the controler goes thrue start up routine. After that if everything is okay (input voltage, temperatue) it goes in running mode, otherwise it falls in error state which is indicated with **solid red light**.

### Working Principle

- **Sampling:** The system samples data at 50Hz*. In the first sample, the MCU collects raw measurements. In the next sample, these values are converted into real-world units (V, °C, %). This results in an effective sampling rate of 25Hz for the calculated values.
- **Control Logic:** Based on the collected values, the controller can either trigger an error or set a new duty cycle for the motor based on the potentiometer reading.
- **Error Conditions:**
    - **Over-temperature:** 80°C
    - **Under-voltage:** 3.5V
- **Duty Cycle:** The duty cycle can be set between 10% and 90%. Values below 10% are treated as 0%, and values above 90% are treated as 100%.
    - **Note:** This DC motors require a minimum of 3.0V - 3.5V to start.

![Pump control PWM](https://github.com/TilenTinta/Aquarium_Air_Pump_Controller/blob/main/Images/Aquarium_pump_controler_PWM-1kHz.png) 
---

## Debugging

### LED Status Codes  

| LED Pattern     | Meaning                   |
|-----------------|---------------------------|
| Blinking **red**  | Device initialization   |
| Solid **red**     | Device error            |


