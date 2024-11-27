# Automotive Dashboard with ESP32 and Arduino Nano

This project implements a CAN-based automotive dashboard using an ESP32 and an Arduino Nano, with a Nextion display for visualizing data. The system controls indicators, reads environmental data (temperature and humidity), and sends this data to the dashboard via CAN communication.

---

## Features
- **ESP32 Functionalities**:
  - Reads temperature and humidity using a BME280 sensor.
  - Controls indicators (left/right) with blinking synchronization.
  - Sends data over CAN with a baud rate of 250kbps.

- **Arduino Nano Functionalities**:
  - Receives CAN data.
  - Displays temperature, humidity, and indicator status on the Nextion dashboard.

- **RTOS and Queue Implementation**:
  - Used FreeRTOS for managing CAN communication.
  - Implemented queue-based message handling for reliable data transfer.

---

## Data Transmission IDs
- **Temperature and Humidity**: `0x010`
- **Indicator Control**: `0x001`

---

## Tools and Technologies
- **Microcontrollers**: ESP32, Arduino Nano.
- **Communication Protocol**: CAN Bus (125kbps baud rate).
- **Display**: Nextion touchscreen.
- **Programming Languages**: C/C++ (Arduino framework).
- **Platform**: Visual Studio Code with PlatformIO.

---

## How It Works
1. **ESP32**:
   - Reads data from the BME280 sensor and processes indicator button inputs.
   - Sends the data to Arduino Nano via CAN communication.

2. **Arduino Nano**:
   - Receives the CAN data and updates the Nextion dashboard in real time.

3. **Nextion Dashboard**:
   - Displays live data, including temperature, humidity, and indicator status.
---

## Future Enhancements
- Add more sensors for comprehensive vehicle monitoring.
- Integrate error handling for robust CAN communication.

---

## License
This project is licensed under the MIT License. Feel free to use and modify it for your own purposes!

---

**Developed by [AKSHAY P KUMAR ](https://github.com/akshayembedded).**
