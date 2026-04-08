# Rocket Camera

## Sensors/Peripherals

- Runcam Mini DVR
  - Simple interrupt pin to trigger starting and stopping recording
- MPL3115A2 Pressure Sensor (I2C)
  - Used to measure altitude and vertical speed to trigger what stage of flight we are in
- RushFPV 3.3Ghz 4W VTX (IRC Tramp)
  - On boot set to max (4W) power and 3330Mhz frequency
  - On Landing set to min (PIT or 25mW) power
  - Uses a single wire half duplex UART for control and feedback. Need to use a custom PIO program to control it
- Holybro Micro OSD V2 (Mavlink V1)
  - Used to overlay telemetry data on the video feed
  - Send the latest altitude and vertical speed
  - Uses standard UART (TX/RX) for control and feedback

## RushFPV 3.3Ghz 4W VTX

### **General Characteristics**

| Parameter | Specification |
| :--- | :--- |
| **Frequency band** | 3170-3470MHz |
| **Frequency precision** | +200KHz (Typ.) |
| **Channel customer** | 16 |
| **S/N (For +3MHz)** | >55dBc |
| **Modulation type** | FM |
| **Antenna Port** | SMA 50 Ohms |
| **Frequency control** | PLL |
| **Video input level** | 1V+0.2Vp-p (typ.) |
| **Frequency stability** | =100KHz (Typ.) |
| **Operating Temperature** | -10°C~+70°C |
| **Power Supply** | DC 7-30V |
| **Control Protocol** | IRC Tramp |

---

### **Channel Frequencies**

| Band | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **3G3 BAND A** | 3330 | 3350 | 3370 | 3390 | 3410 | 3430 | 3450 | 3470 |
| **3G3 BAND B** | 3170 | 3190 | 3210 | 3230 | 3250 | 3270 | 3290 | 3310 |

---

### **Power Levels**

| Power Level | Values | Min. | Typ. | Max. | Current @12V |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **0** | PIT | -30dBm | -28dBm | -26dBm | 160mA |
| **1** | 25mW | 12dBm | 14dBm | 16dBm | 270mA |
| **2** | 200mW | 21dBm | 23dBm | 25dBm | 550mA |
| **3** | 1000mW | 29dBm | 30dBm | 31dBm | 880mA |
| **4** | 4000mW | 35.5dBm | 36dBm | 36.5dBm | 1400mA |

## Pinout

| Pin | Function |
| --- | -------- |
