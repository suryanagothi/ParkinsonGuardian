# Parkinson Guardian — Smart Health Monitoring Glove 🩺🤖

An IoT-based assistive wearable project built on the **ESP32 DevKit V1 (30-pin)** designed to monitor real-time Parkinson's symptoms (tremors), vital signs (Heart Rate & SpO2), body temperature, and provide an emergency SOS button with Blynk cloud integration.

## 🚀 Features
* **Tremor Detection:** High-frequency motion tracking via the MPU6050 accelerometer/gyroscope with dynamic jerk calculation (50 Hz sampling).
* **Vital Signs Monitoring:** Real-time Heart Rate (BPM) and blood oxygen saturation (SpO2) tracking using the MAX30102 pulse oximeter.
* **Temperature Tracking:** Ambient and body temperature monitoring using the DHT11 sensor.
* **Emergency SOS & Feedback:** Physical pushbutton for instant emergency triggers, supported by local buzzer and vibration motor alerts.
* **Cloud Dashboard (Blynk):** Live data telemetry pushed to the Blynk IoT platform at a 2 Hz update rate.

## 🧰 Hardware Bill of Materials (BOM)
* **Microcontroller:** ESP32 DevKit V1 (30-pin)
* **Motion Sensor:** MPU6050 6-axis Accelerometer & Gyroscope
* **Pulse Oximeter:** MAX30102 High-Sensitivity Pulse and Proximity Sensor
* **Temperature Sensor:** DHT11 Digital Temperature & Humidity Sensor
* **Peripherals:** Active Buzzer, Mini Vibration Motor, Tactile Push Button (SOS)
* **Power Source:** USB / LiPo battery

## 📌 Pin Mapping & Wiring Diagram
| Component / Peripheral | Pin Name | ESP32 GPIO Pin | Description |
| :--- | :--- | :--- | :--- |
| **I2C Bus (MPU6050 & MAX30102)** | SDA | GPIO 21 | I2C Data Line |
| | SCL | GPIO 22 | I2C Clock Line |
| **DHT11 Sensor** | DATA | GPIO 4 | Digital Temperature Data |
| **Buzzer** | SIG | GPIO 18 | Audible Alert Output |
| **Vibration Motor** | SIG | GPIO 19 | Tactile Feedback Output |
| **SOS Button** | BUTTON | GPIO 15 | Active-Low Emergency Pushbutton (with Internal Pullup) |

## 📦 Prerequisites & Libraries
Make sure the following libraries are installed via the **Arduino Library Manager**:
* **Blynk** (by Volodymyr Shymanskyy)
* **Adafruit MPU6050**
* **Adafruit Unified Sensor**
* **DHT sensor library** (by Adafruit)
* **SparkFun MAX3010x Pulse and Proximity Sensor Library** (MAX30105)

## ⚙️ Setup & Installation
1. Clone or download this repository.
2. Open the file `Arduino/ParkinsonGuardian_ESP32.ino` using the **Arduino IDE**.
3. Install all required dependencies listed above.
4. Select your board (**ESP32 Dev Module**) and correct COM port under **Tools**.
5. Compile and upload the sketch to your ESP32.

## 📄 License
This project is open-source and available under the MIT License.
