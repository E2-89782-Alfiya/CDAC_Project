# Real-Time CAN Communication System with FreeRTOS, IoT Integration and Blynk Alerts

## 📘 Project Overview
This project implements a smart, real-time sensor data acquisition and monitoring system using **STM32F407VG-DISC1** microcontrollers, **FreeRTOS**, and **IoT integration**.  
Two STM32 boards communicate over the **CAN bus**, where the transmitter collects environmental sensor data and sends it to the receiver.  
The receiver displays the data on an LCD, uploads it to the cloud using an **ESP8266** Wi-Fi module, and sends **real-time alerts via Blynk** if critical thresholds are exceeded.  
A **Flask REST API** stores all sensor data into a **MySQL** database for historical logging and analysis, while **Blynk** provides instant push notifications to the user’s mobile device.

## 🔧 Key Features

  ### Sensor Integration
  - **LDR** – Light Intensity measurement  
  - **MQ5** – Gas detection  
  - **DHT11** – Temperature & Humidity monitoring  
  
  ### CAN Communication
  - Data exchange between two STM32 boards at **1 Mbps**  
  - CAN ID filtering for selective message processing  
  
  ### Receiver Node Functionality
  - Decodes CAN messages  
  - Displays values on a **16x2 I2C LCD**  
  - Sends data to **ESP8266** over **UART2**  
  - **Triggers Blynk alerts** when gas concentration, temperature, or light levels cross safety limits
  
  ### Cloud Integration
  - **ESP8266** formats data into JSON  
  - Sends it to a **Flask server** for storage in **MySQL**  
  - **Blynk IoT platform** sends mobile notifications in real-time  
  - Optional: UART debug logging

## 🧠 Technologies Used
- **Microcontroller:** STM32F407VG-DISC1  
- **RTOS:** FreeRTOS  
- **Protocols:** CAN Bus, UART, I2C  
- **Connectivity:** ESP8266 (AT commands)  
- **Server:** Flask (Python), Blynk IoT  
- **Database:** MySQL  
- **Languages:** C (STM32), Python (Flask), SQL  


## 🎯 Objectives
- Implement a robust and scalable sensor data network using STM32 and CAN.  
- Enable wireless cloud integration for remote monitoring.  
- Provide **instant safety alerts via Blynk** to mobile devices.  
- Store sensor readings in MySQL for historical analysis.

## 📈 Applications
- Industrial and environmental monitoring  
- Smart agriculture systems  
- Laboratory automation  
- IoT-based smart infrastructure with real-time alerts  

## 🗄 Database Schema
```sql
CREATE TABLE SensorData (
    ID INT NOT NULL AUTO_INCREMENT PRIMARY KEY,
    LDR_DATA INT,
    MQ5_DATA INT,
    TEMPERATURE FLOAT,
    HUMIDITY INT,
    LOCATION VARCHAR(30),
    DATE_TIME TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

## 📂 Project Structure
```
CODE/Final_Project_Code/
│
├── ESP8266/                 # ESP8266 Wi-Fi module code (UART JSON send to server)
├── Server_Dashboard/        # Flask REST API + MySQL Database + Web Dashboard
├── STM32_CAN_Receiver/      # Receiver Node (STM32 + CAN RX + LCD + ESP8266 )
└── STM32_CAN_Transmitter/   # Transmitter Node (STM32 + Sensors + CAN TX)
```

## 🚀 How to Run

### 1️⃣ Hardware Setup
- Connect **LDR, MQ5 and DHT11** sensors to the **STM32_CAN_Transmitter** board.  
- Connect **CAN bus** between STM32 boards:  
  - **CANH ↔ CANH**  (use CAN Transreceiver eg. MCP2551)
  - **CANL ↔ CANL**  (use CAN Transreceiver eg. MCP2551)
- Attach **16x2 I²C LCD** and **ESP8266** to the **STM32_CAN_Receiver** board.

### 2️⃣ Firmware
- Flash **STM32_CAN_Transmitter** code to the transmitter board.  
- Flash **STM32_CAN_Receiver** code to the receiver board.
- Flash **ESP8266** code to the NODE MCU board.
- Run **Server_Dashboard** code 

### 3️⃣ Server Setup
Install dependencies:
```bash
pip install flask mysql-connector-python
```
Start the Flask server:
```bash
python3 server.py
```
Ensure MySQL database contains the **SensorData** table as per the schema above.

### 4️⃣ Blynk Setup
- Create a new project in the **Blynk** app.  
- Get the **Auth Token** and insert it into your **ESP8266 firmware**.  
- Configure **virtual pins** for sending alerts when thresholds are exceeded.

### 5️⃣ Testing
- Check **LCD** for live values.   
- Query **MySQL database** for stored sensor readings.  
- Trigger alerts in **Blynk** when thresholds are crossed.

## 🔮 Future Scope
- Automatic **database cleanup** when table size exceeds limit.  
- Add **real-time graphs** on the web dashboard.  
- Enable **remote device control** via the cloud interface.

