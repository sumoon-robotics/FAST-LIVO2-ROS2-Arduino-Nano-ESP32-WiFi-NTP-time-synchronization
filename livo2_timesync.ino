/*
 * FAST-LIVO2 Time Sync Firmware
 * Arduino Nano ESP32
 *
 * Replaces STM32F407 — adds WiFi NTP for automatic time sync.
 * No manual reseeding required. Generates:
 *
 *   D1  (TX1)  → GPRMC 9600 baud    → Mid-360 Gray/White
 *   D4  (TX2)  → GPRMC mirror       → USB-TTL RX → Jetson /dev/ttyUSB0
 *   D5         → PPS 1Hz            → Mid-360 Purple/White
 *   A0         → Camera trigger 10Hz → HIKrobot Line 2 (+)
 *   GND        → Ground             → Mid-360, HIKrobot, USB-TTL
 *   USB-C      → Power + program    → Jetson
 *
 * Board:  Arduino Nano ESP32 (Espressif ESP32 package)
 * Tools → Board → Arduino Nano ESP32
 *
 * Wiring diagram:
 *
 *   Arduino Nano ESP32          Mid-360 Sync Cable
 *   ──────────────────          ──────────────────
 *   D1  (TX1)          ──►     Gray/White  (GPRMC input)
 *   D5                 ──►     Purple/White (PPS 1Hz)
 *   GND                ──►     Black (GND)
 *
 *   Arduino Nano ESP32          USB-TTL Adapter
 *   ──────────────────          ────────────────
 *   D4  (TX2)          ──►     RX
 *   GND                ──►     GND
 *   USB-TTL USB        ──►     Jetson → /dev/ttyUSB0
 *
 *   Arduino Nano ESP32          HIKrobot MV-CA016-10UC I/O Cable
 *   ──────────────────          ──────────────────────────────────
 *   A0                 ──►     Line 2 trigger input (+)
 *   GND                ──►     Line 2 trigger input (-)
 */

#include <WiFi.h>
#include <time.h>

// ============================================================
// CONFIGURE THESE BEFORE UPLOADING
// ============================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
// ============================================================

// Pin definitions
#define GPRMC_TX          1    // D1 → Mid-360 GPRMC input
#define GPRMC_MONITOR_TX  4    // D4 → USB-TTL → Jetson /dev/ttyUSB0
#define PPS_PIN           5    // D5 → Mid-360 PPS input
#define CAMERA_TRIGGER    A0   // A0 → HIKrobot Line 2 trigger

// NTP (UTC — FAST-LIVO2 uses UTC internally)
const char* NTP_SERVER      = "pool.ntp.org";
const long  GMT_OFFSET_SEC  = 0;
const int   DAYLIGHT_OFFSET = 0;

// Camera trigger timing
#define CAMERA_HZ        10          // 10Hz — matches FAST-LIVO2 VIO rate
#define CAMERA_PERIOD_MS (1000 / CAMERA_HZ)  // 100ms
#define CAMERA_PULSE_MS  10          // 10ms pulse width

// UART instances
HardwareSerial GPRMCSerial(1);    // UART1 → Mid-360
HardwareSerial MonitorSerial(2);  // UART2 → USB-TTL → Jetson

bool timesynced = false;

// ─────────────────────────────────────────────────────────────
// NMEA checksum (XOR of all bytes between $ and *)
// ─────────────────────────────────────────────────────────────
uint8_t nmeaChecksum(const char* sentence) {
  uint8_t cs = 0;
  for (int i = 1; sentence[i] != '*' && sentence[i] != '\0'; i++) {
    cs ^= (uint8_t)sentence[i];
  }
  return cs;
}

// ─────────────────────────────────────────────────────────────
// Build and send GPRMC sentence on both UART1 and UART2
// ─────────────────────────────────────────────────────────────
void sendGPRMC() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[WARN] getLocalTime failed");
    return;
  }

  // GPRMC format:
  // $GPRMC,HHMMSS.000,A,LAT,N,LON,E,SPD,CRS,DDMMYY,0.0,,A*CS
  // Lat/lon fixed at 0.0 — Mid-360 only uses time fields
  char sentence[120];
  snprintf(sentence, sizeof(sentence),
    "$GPRMC,%02d%02d%02d.000,A,0.0,N,0.0,E,0.0,0.0,%02d%02d%02d,0.0,,A",
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec,
    timeinfo.tm_mday,
    timeinfo.tm_mon + 1,
    timeinfo.tm_year % 100
  );

  char full[130];
  snprintf(full, sizeof(full), "%s*%02X\r\n", sentence, nmeaChecksum(sentence));

  GPRMCSerial.print(full);     // → Mid-360
  MonitorSerial.print(full);   // → Jetson /dev/ttyUSB0
  Serial.print("[GPRMC] ");
  Serial.print(full);
}

// ─────────────────────────────────────────────────────────────
// Camera trigger — non-blocking 10Hz PWM on A0
// ─────────────────────────────────────────────────────────────
void handleCameraTrigger() {
  static unsigned long lastTrigger = 0;
  static bool pulseActive = false;
  static unsigned long pulseStart = 0;

  unsigned long now = millis();

  if (!pulseActive && (now - lastTrigger >= CAMERA_PERIOD_MS)) {
    digitalWrite(CAMERA_TRIGGER, HIGH);
    pulseActive = true;
    pulseStart = now;
    lastTrigger = now;
  }

  if (pulseActive && (now - pulseStart >= CAMERA_PULSE_MS)) {
    digitalWrite(CAMERA_TRIGGER, LOW);
    pulseActive = false;
  }
}

// ─────────────────────────────────────────────────────────────
// WiFi + NTP sync
// ─────────────────────────────────────────────────────────────
void syncNTP() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] FAILED — check credentials");
    timesynced = false;
    return;
  }

  Serial.printf("\n[WiFi] Connected — IP: %s\n",
    WiFi.localIP().toString().c_str());

  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET, NTP_SERVER);
  Serial.print("[NTP]  Syncing");

  struct tm timeinfo;
  int ntpAttempts = 0;
  while (!getLocalTime(&timeinfo) && ntpAttempts < 10) {
    delay(1000);
    Serial.print(".");
    ntpAttempts++;
  }

  if (getLocalTime(&timeinfo)) {
    timesynced = true;
    Serial.printf("\n[NTP]  Synced — %02d:%02d:%02d UTC  %02d/%02d/20%02d\n",
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
      timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year % 100);
  } else {
    Serial.println("\n[NTP]  FAILED — no internet?");
    timesynced = false;
  }
}

// ─────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n================================");
  Serial.println(" FAST-LIVO2 Time Sync v1.0");
  Serial.println(" Arduino Nano ESP32");
  Serial.println("================================");

  // UART1 → Mid-360 GPRMC
  GPRMCSerial.begin(9600, SERIAL_8N1, -1, GPRMC_TX);

  // UART2 → USB-TTL → Jetson /dev/ttyUSB0
  MonitorSerial.begin(9600, SERIAL_8N1, -1, GPRMC_MONITOR_TX);

  // PPS → Mid-360
  pinMode(PPS_PIN, OUTPUT);
  digitalWrite(PPS_PIN, LOW);

  // Camera trigger → HIKrobot
  pinMode(CAMERA_TRIGGER, OUTPUT);
  digitalWrite(CAMERA_TRIGGER, LOW);

  Serial.println("[PINS] D1=GPRMC  D4=Monitor  D5=PPS  A0=CamTrig");

  syncNTP();

  Serial.println("[READY] Running...");
}

// ─────────────────────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastSecond = 0;
  unsigned long now = millis();

  // 1Hz — PPS pulse + GPRMC
  if (now - lastSecond >= 1000) {
    lastSecond = now;

    // PPS: 100ms high, 900ms low
    digitalWrite(PPS_PIN, HIGH);
    delay(100);
    digitalWrite(PPS_PIN, LOW);

    if (timesynced) {
      sendGPRMC();
    } else {
      Serial.println("[WARN] Not synced — retrying NTP...");
      syncNTP();
    }
  }

  // 10Hz — camera trigger (non-blocking)
  handleCameraTrigger();
}
