/*
 * ParkinsonGuardian_ESP32.ino
 * Smart Parkinson's Disease Monitoring Glove (With Dynamic Blynk Buzzer Control)
 * Target: ESP32 DevKit V1 (30-pin)
 *
 * Blynk Datastreams:
 * - V0: Tremor Level (Widget: Gauge / Value Display)
 * - V1: Heart Rate BPM (Widget: Gauge / Value Display)
 * - V2: SpO2 % (Widget: Gauge / Value Display)
 * - V3: Body Temperature °C (Widget: Value Display)
 * - V4: Buzzer Alert Threshold Slider (Widget: Slider, Range 0-1000)
 */

#define BLYNK_TEMPLATE_ID    "parkinsons glove"
#define BLYNK_TEMPLATE_NAME "Parkinson Guardian"
#define BLYNK_DEVICE_NAME   "Parkison Guardian"
#define BLYNK_AUTH_TOKEN    "PJBRTt4aysQwUgogFL06vJCqcaw-SXdV"

const char* WIFI_SSID     = "AI-LAB";
const char* WIFI_PASSWORD = "AI@lbrce76";

// ---------------------------------------------------------------------------
// Pin mapping

// ---------------------------------------------------------------------------
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        22
#define DHT_PIN            4         
#define DHT_TYPE           DHT11     
#define BUZZER_PIN         18
#define VIBRATION_PIN      19
#define SOS_BUTTON_PIN     15

// Blynk virtual pins
#define VPIN_TREMOR        V0
#define VPIN_HEART_RATE    V1
#define VPIN_SPO2          V2
#define VPIN_BODY_TEMP     V3
#define VPIN_BUZZER_THRESH V4  // Dynamic slider for buzzer sensitivity

// ---------------------------------------------------------------------------
// Timing intervals (ms) — non-blocking
// ---------------------------------------------------------------------------
const unsigned long MPU_INTERVAL_MS        = 20;    // 50 Hz tremor sampling
const unsigned long MAX30102_INTERVAL_MS   = 10;    // 100 Hz PPG sampling
const unsigned long DHT_INTERVAL_MS        = 2000;  // 2 Hz temperature/humidity
const unsigned long BLYNK_SEND_INTERVAL_MS = 500;   // 2 Hz dashboard updates
const unsigned long ALERT_CHECK_INTERVAL_MS = 100;
const unsigned long SOS_DEBOUNCE_MS        = 50;
const unsigned long ALERT_COOLDOWN_MS      = 30000; // 30 s between duplicate alerts

// ---------------------------------------------------------------------------
// Alert thresholds
// ---------------------------------------------------------------------------
const float HR_MIN_BPM            = 50.0f;
const float HR_MAX_BPM            = 120.0f;
const float SPO2_MIN_PERCENT      = 92.0f;
const int   TREMOR_HIT_COUNT      = 10;     // Consecutive samples required above threshold
const float BODY_TEMP_MAX_C       = 38.5f;  // Fever alert threshold

// Dynamic threshold updated via Blynk app slider (Default: 800)
float tremorAlertThreshold        = 800.0f;

// ---------------------------------------------------------------------------
// Includes
// ---------------------------------------------------------------------------
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MAX30105.h>
#include <DHT.h>
#include <math.h>
#include <heartRate.h>

// ---------------------------------------------------------------------------
// Sensor objects
// ---------------------------------------------------------------------------
Adafruit_MPU6050 mpu;
MAX30105         particleSensor;
DHT              dht(DHT_PIN, DHT_TYPE);

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
unsigned long lastMpuMs        = 0;
unsigned long lastMax30102Ms   = 0;
unsigned long lastDhtMs        = 0;
unsigned long lastBlynkSendMs  = 0;
unsigned long lastAlertCheckMs = 0;

bool alertActive      = false;
bool mpuReady         = false;
bool max30102Ready    = false;
bool dhtReady         = false;

// MPU6050 / tremor
float lastAccelMag    = 0.0f;
float tremorLevel     = 0.0f;
int   tremorHitStreak = 0;
bool  tremorAlert     = false;

// MAX30102 HR / SpO2
const byte  MAX30102_BUFFER_LEN = 100;
long        irBuffer[MAX30102_BUFFER_LEN];
long        redBuffer[MAX30102_BUFFER_LEN];
int         bufferIndex         = 0;
bool        bufferFull          = false;

float heartRateBpm = 0.0f;
float spo2Percent  = 0.0f;
bool  hrValid        = false;
bool  spo2Valid      = false;

// DHT11
float ambientTempC = 0.0f;
float bodyTempC    = 0.0f;

// SOS button
bool          sosPressed      = false;
bool          sosLastRawState = HIGH;
unsigned long sosLastChangeMs = 0;

// Alert cooldown timestamps
unsigned long lastHrAlertMs     = 0;
unsigned long lastSpo2AlertMs   = 0;
unsigned long lastTremorAlertMs = 0;
unsigned long lastSosAlertMs    = 0;
unsigned long lastTempAlertMs   = 0;

// Heart-rate beat detector state
const int16_t HR_RATE_SIZE = 4;
int16_t hrRates[HR_RATE_SIZE];
byte    hrRateIndex = 0;
long    hrLastBeat  = 0;

// ---------------------------------------------------------------------------
// Blynk Handlers
// ---------------------------------------------------------------------------
// Receives slider movements from Blynk App live
BLYNK_WRITE(VPIN_BUZZER_THRESH) {
  tremorAlertThreshold = param.asFloat();
  Serial.print(F("[BLYNK] Buzzer Threshold updated to: "));
  Serial.println(tremorAlertThreshold);
}

// Syncs slider value from Blynk Cloud when ESP32 connects
BLYNK_CONNECTED() {
  Blynk.syncVirtual(VPIN_BUZZER_THRESH);
}

// ---------------------------------------------------------------------------
// SpO2 ratio calculation
// ---------------------------------------------------------------------------
void calculateSpO2FromBuffer(float* spo2Out, bool* validOut) {
  *validOut = false;
  *spo2Out  = 0.0f;

  if (!bufferFull) return;

  int32_t irMean = 0;
  int32_t redMean = 0;
  for (int i = 0; i < MAX30102_BUFFER_LEN; i++) {
    irMean  += irBuffer[i];
    redMean += redBuffer[i];
  }
  irMean  /= MAX30102_BUFFER_LEN;
  redMean /= MAX30102_BUFFER_LEN;

  if (irMean <= 0 || redMean <= 0) return;

  float irAc = 0.0f;
  float redAc = 0.0f;
  for (int i = 0; i < MAX30102_BUFFER_LEN; i++) {
    float irDev  = (float)(irBuffer[i]  - irMean);
    float redDev = (float)(redBuffer[i] - redMean);
    irAc  += irDev * irDev;
    redAc += redDev * redDev;
  }
  irAc  = sqrtf(irAc / MAX30102_BUFFER_LEN);
  redAc = sqrtf(redAc / MAX30102_BUFFER_LEN);

  if (irAc <= 0.0f) return;

  float ratio = (redAc / (float)redMean) / (irAc / (float)irMean);
  float spo2  = -45.060f * ratio * ratio + 30.354f * ratio + 94.845f;

  if (spo2 > 100.0f) spo2 = 100.0f;
  if (spo2 < 70.0f)  return;

  *spo2Out  = spo2;
  *validOut = true;
}

// ---------------------------------------------------------------------------
// Alert helpers
// ---------------------------------------------------------------------------
void setAlertOutputs(bool on) {
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
  digitalWrite(VIBRATION_PIN, on ? HIGH : LOW);
  alertActive = on;
}

void sendBlynkAlert(const char* eventCode, const char* message, unsigned long* cooldownMs) {
  unsigned long now = millis();
  if (now - *cooldownMs >= ALERT_COOLDOWN_MS) {
    *cooldownMs = now;
    Blynk.logEvent(eventCode, message);
    Serial.print(F("[ALERT] "));
    Serial.println(message);
  }
}

void updateAlertOutputs() {
  bool shouldAlert = false;

  if (sosPressed) {
    shouldAlert = true;
    sendBlynkAlert("severe_tremor", "SOS button pressed — emergency!", &lastSosAlertMs);
  }
  if (hrValid && (heartRateBpm < HR_MIN_BPM || heartRateBpm > HR_MAX_BPM)) {
    shouldAlert = true;
  }
  if (spo2Valid && spo2Percent < SPO2_MIN_PERCENT) {
    shouldAlert = true;
  }
  if (tremorAlert) {
    shouldAlert = true;
    sendBlynkAlert("severe_tremor", "Severe tremor detected!", &lastTremorAlertMs);
  }
  if (dhtReady && bodyTempC > BODY_TEMP_MAX_C) {
    shouldAlert = true;
  }

  setAlertOutputs(shouldAlert);
}

// ---------------------------------------------------------------------------
// Sensor init
// ---------------------------------------------------------------------------
bool initMPU6050() {
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println(F("[ERR] MPU6050 not found on I2C bus"));
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  lastAccelMag = sqrtf(a.acceleration.x * a.acceleration.x +
                       a.acceleration.y * a.acceleration.y +
                       a.acceleration.z * a.acceleration.z);

  Serial.println(F("[OK] MPU6050 initialized"));
  return true;
}

bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("[ERR] MAX30102 not found on I2C bus"));
    return false;
  }

  byte ledBrightness = 80;
  byte sampleAverage = 4;
  byte ledMode       = 2; // Red + IR
  int  sampleRate    = 100;
  int  pulseWidth    = 411;
  int  adcRange      = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);

  Serial.println(F("[OK] MAX30102 initialized"));
  return true;
}

// ---------------------------------------------------------------------------
// Non-blocking sensor readers
// ---------------------------------------------------------------------------
void readMPU6050(unsigned long nowMs) {
  if (!mpuReady || (nowMs - lastMpuMs) < MPU_INTERVAL_MS) return;
  lastMpuMs = nowMs;

  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) return;

  float accelMag = sqrtf(a.acceleration.x * a.acceleration.x +
                       a.acceleration.y * a.acceleration.y +
                       a.acceleration.z * a.acceleration.z);
  float dtSec = MPU_INTERVAL_MS / 1000.0f;
  float jerk  = fabsf(accelMag - lastAccelMag) / dtSec;
  lastAccelMag = accelMag;

  // Scale jerk to 0 - 1000 range
  tremorLevel = constrain(jerk * 20.0f, 0.0f, 1000.0f);

  // Evaluates against tremorAlertThreshold set dynamically by Blynk Slider
  if (tremorLevel >= tremorAlertThreshold) {
    tremorHitStreak++;
  } else {
    tremorHitStreak = max(0, tremorHitStreak - 1);
  }

  tremorAlert = (tremorHitStreak >= TREMOR_HIT_COUNT);
}

void readMAX30102(unsigned long nowMs) {
  if (!max30102Ready || (nowMs - lastMax30102Ms) < MAX30102_INTERVAL_MS) return;
  lastMax30102Ms = nowMs;

  long irValue  = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  if (irValue < 50000) {
    hrValid   = false;
    spo2Valid = false;
    return;
  }

  if (checkForBeat(irValue)) {
    long delta = millis() - hrLastBeat;
    hrLastBeat = millis();

    float beatsPerMinute = 60.0f / (delta / 1000.0f);

    if (beatsPerMinute > 30.0f && beatsPerMinute < 220.0f) {
      hrRates[hrRateIndex++] = (int16_t)beatsPerMinute;
      hrRateIndex %= HR_RATE_SIZE;

      int32_t hrBeatAvg = 0;
      for (byte x = 0; x < HR_RATE_SIZE; x++) {
        hrBeatAvg += hrRates[x];
      }
      hrBeatAvg /= HR_RATE_SIZE;

      heartRateBpm = (float)hrBeatAvg;
      hrValid      = true;
    }
  }

  irBuffer[bufferIndex]  = irValue;
  redBuffer[bufferIndex] = redValue;
  bufferIndex++;
  if (bufferIndex >= MAX30102_BUFFER_LEN) {
    bufferIndex = 0;
    bufferFull  = true;
  }

  if (bufferFull) {
    float spo2 = 0.0f;
    bool  valid = false;
    calculateSpO2FromBuffer(&spo2, &valid);
    if (valid) {
      spo2Percent = spo2;
      spo2Valid   = true;
    }
  }
}

void readDHT11(unsigned long nowMs) {
  if (!dhtReady || (nowMs - lastDhtMs) < DHT_INTERVAL_MS) return;
  lastDhtMs = nowMs;

  float temp = dht.readTemperature();
  if (!isnan(temp)) {
    ambientTempC = temp;
    bodyTempC    = temp;
  }
}

void readSOSButton(unsigned long nowMs) {
  bool raw = digitalRead(SOS_BUTTON_PIN);

  if (raw != sosLastRawState) {
    sosLastChangeMs = nowMs;
    sosLastRawState = raw;
  }

  if ((nowMs - sosLastChangeMs) >= SOS_DEBOUNCE_MS) {
    sosPressed = (raw == LOW);
  }
}

void checkThresholdAlerts(unsigned long nowMs) {
  if ((nowMs - lastAlertCheckMs) < ALERT_CHECK_INTERVAL_MS) return;
  lastAlertCheckMs = nowMs;

  updateAlertOutputs();
}

void sendBlynkData(unsigned long nowMs) {
  if (!Blynk.connected() || (nowMs - lastBlynkSendMs) < BLYNK_SEND_INTERVAL_MS) return;
  lastBlynkSendMs = nowMs;

  Blynk.virtualWrite(VPIN_TREMOR, tremorLevel);

  if (hrValid)   Blynk.virtualWrite(VPIN_HEART_RATE, heartRateBpm);
  if (spo2Valid) Blynk.virtualWrite(VPIN_SPO2, spo2Percent);
  if (dhtReady)  Blynk.virtualWrite(VPIN_BODY_TEMP, bodyTempC);
}

void printSerialDebug(unsigned long nowMs) {
  static unsigned long lastPrintMs = 0;
  if ((nowMs - lastPrintMs) < 2000) return;
  lastPrintMs = nowMs;

  Serial.print(F("Tremor:"));
  Serial.print(tremorLevel, 1);
  Serial.print(F(" (Thresh:"));
  Serial.print(tremorAlertThreshold, 0);
  Serial.print(F(") HR:"));
  Serial.print(hrValid ? String(heartRateBpm, 1) : String(F("--")));
  Serial.print(F(" SpO2:"));
  Serial.print(spo2Valid ? String(spo2Percent, 1) : String(F("--")));
  Serial.print(F(" Temp:"));
  Serial.print(dhtReady ? String(bodyTempC, 2) : String(F("--")));
  Serial.print(F(" Alert:"));
  Serial.println(alertActive ? F("ON") : F("OFF"));
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println(F("=== Parkinson Guardian Glove — Ready ==="));

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
  setAlertOutputs(false);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  mpuReady      = initMPU6050();
  max30102Ready = initMAX30102();
  
  dht.begin();
  dhtReady = true;

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  unsigned long nowMs = millis();

  Blynk.run();

  readSOSButton(nowMs);
  readMPU6050(nowMs);
  readMAX30102(nowMs);
  readDHT11(nowMs);
  checkThresholdAlerts(nowMs);
  sendBlynkData(nowMs);
  printSerialDebug(nowMs);
}
