#include "leds.h"
#include "pov.h"
#include "status_leds.h"
#include "pov_font.h"

#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <string>
#include "luna.h"
#include "jyro.h"

// ================= COMMS MODE SWITCH =================
#define COMM_MODE_WIFI 1
#define COMM_MODE_BLE  2
#define COMM_MODE COMM_MODE_BLE
// =====================================================

#if COMM_MODE == COMM_MODE_WIFI
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

#if COMM_MODE == COMM_MODE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif

/****************************************************
 * ===== WIFI / UDP TELEMETRY (Laptop <-> ESP32) =====
 * ESP32 creates a WiFi Access Point (AP). Connect your laptop to it.
 * Telemetry is sent via UDP broadcast on UDP_PORT.
 ****************************************************/
#if COMM_MODE == COMM_MODE_WIFI
static const char* AP_SSID = "SpinTop";
static const char* AP_PASS = "12345678";     // >= 8 chars for WPA2
static constexpr uint16_t UDP_PORT = 4210;
static constexpr uint16_t UDP_CMD_PORT = 4211;
static constexpr uint8_t  TELEMETRY_MAGIC[2] = {'S', 'P'};
static constexpr uint8_t  TELEMETRY_VERSION = 1;
static constexpr uint8_t  TELEMETRY_DOWNSAMPLE = 4;

static WiFiUDP udp;
static WiFiUDP udp_cmd;
static IPAddress udp_broadcast_ip(255, 255, 255, 255);

static void wifi_debug_init()
{
  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);

  Serial.println();
  Serial.println("=== WiFi Debug (AP mode) ===");
  Serial.print("softAP start: ");
  Serial.println(ok ? "OK" : "FAIL");

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP:   ");
  Serial.println(ip);
  Serial.print("UDP port:");
  Serial.println(UDP_PORT);

  udp.begin(UDP_PORT);
  udp_cmd.begin(UDP_CMD_PORT);
}

static void wifi_debug_send(const char* msg)
{
  if (!msg) return;

  udp.beginPacket(udp_broadcast_ip, UDP_PORT);
  udp.write((const uint8_t*)msg, (int)strlen(msg));
  udp.endPacket();
}

static void write_u16_le(uint8_t* out, uint16_t v)
{
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)(v >> 8);
}

static void write_f32_le(uint8_t* out, float v)
{
  uint8_t* p = reinterpret_cast<uint8_t*>(&v);
  out[0] = p[0];
  out[1] = p[1];
  out[2] = p[2];
  out[3] = p[3];
}

static void wifi_debug_send_telemetry(uint16_t dist_mm, float gz_dps, float rpm,
                                      float wobble, float rps)
{
  static uint32_t counter = 0;
  counter++;
  if ((counter % TELEMETRY_DOWNSAMPLE) != 0) return;

  uint8_t buf[22];
  buf[0] = TELEMETRY_MAGIC[0];
  buf[1] = TELEMETRY_MAGIC[1];
  buf[2] = TELEMETRY_VERSION;
  buf[3] = 0;
  write_u16_le(&buf[4], dist_mm);
  write_f32_le(&buf[6], gz_dps);
  write_f32_le(&buf[10], rpm);
  write_f32_le(&buf[14], wobble);
  write_f32_le(&buf[18], rps);

  udp.beginPacket(udp_broadcast_ip, UDP_PORT);
  udp.write(buf, sizeof(buf));
  udp.endPacket();
}
#endif

/****************************************************
 * ===== BLE TELEMETRY + COMMANDS =====
 * BLE GATT:
 * - Command RX: write text commands
 * - Command TX: notify responses
 * - Telemetry:  notify binary packets
 ****************************************************/
#if COMM_MODE == COMM_MODE_BLE
static const char* BLE_DEVICE_NAME = "SpinTop";
static const char* BLE_SERVICE_UUID   = "5f6d0001-3d42-4cc9-8c92-6b0c9e37b2a1";
static const char* BLE_CMD_RX_UUID    = "5f6d0002-3d42-4cc9-8c92-6b0c9e37b2a1";
static const char* BLE_CMD_TX_UUID    = "5f6d0003-3d42-4cc9-8c92-6b0c9e37b2a1";
static const char* BLE_TELEM_UUID     = "5f6d0004-3d42-4cc9-8c92-6b0c9e37b2a1";
static constexpr uint8_t  TELEMETRY_MAGIC[2] = {'S', 'P'};
static constexpr uint8_t  TELEMETRY_VERSION = 1;
static constexpr uint8_t  TELEMETRY_DOWNSAMPLE = 4;

static BLECharacteristic* ble_cmd_tx = nullptr;
static BLECharacteristic* ble_telem = nullptr;
static bool ble_client_connected = false;

static void write_u16_le(uint8_t* out, uint16_t v)
{
  out[0] = (uint8_t)(v & 0xFF);
  out[1] = (uint8_t)(v >> 8);
}

static void write_f32_le(uint8_t* out, float v)
{
  uint8_t* p = reinterpret_cast<uint8_t*>(&v);
  out[0] = p[0];
  out[1] = p[1];
  out[2] = p[2];
  out[3] = p[3];
}

static void handle_command_line(char* line);

class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* /*server*/) override {
    ble_client_connected = true;
  }

  void onDisconnect(BLEServer* server) override {
    ble_client_connected = false;
    server->getAdvertising()->start();
  }
};

class BleCmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    std::string value = characteristic->getValue();
    if (value.empty()) return;

    char buf[160];
    size_t n = value.size();
    if (n >= sizeof(buf)) n = sizeof(buf) - 1;
    memcpy(buf, value.data(), n);
    buf[n] = '\0';

    char* line = strtok(buf, "\n");
    while (line) {
      size_t len = strlen(line);
      while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[len - 1] = '\0';
        len--;
      }
      if (*line) {
        handle_command_line(line);
      }
      line = strtok(nullptr, "\n");
    }
  }
};

static void ble_debug_init()
{
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(64);

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());

  BLEService* service = server->createService(BLE_SERVICE_UUID);

  BLECharacteristic* cmd_rx = service->createCharacteristic(
    BLE_CMD_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  ble_cmd_tx = service->createCharacteristic(
    BLE_CMD_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  ble_telem = service->createCharacteristic(
    BLE_TELEM_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  cmd_rx->setCallbacks(new BleCmdCallbacks());
  ble_cmd_tx->addDescriptor(new BLE2902());
  ble_telem->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println();
  Serial.println("=== BLE Debug (GATT) ===");
  Serial.print("Device: ");
  Serial.println(BLE_DEVICE_NAME);
}

static void ble_send_telemetry(uint16_t dist_mm, float gz_dps, float rpm,
                               float wobble, float rps)
{
  static uint32_t counter = 0;
  counter++;
  if ((counter % TELEMETRY_DOWNSAMPLE) != 0) return;
  if (!ble_client_connected || !ble_telem) return;

  uint8_t buf[22];
  buf[0] = TELEMETRY_MAGIC[0];
  buf[1] = TELEMETRY_MAGIC[1];
  buf[2] = TELEMETRY_VERSION;
  buf[3] = 0;
  write_u16_le(&buf[4], dist_mm);
  write_f32_le(&buf[6], gz_dps);
  write_f32_le(&buf[10], rpm);
  write_f32_le(&buf[14], wobble);
  write_f32_le(&buf[18], rps);

  ble_telem->setValue(buf, sizeof(buf));
  ble_telem->notify();
}
#endif

/****************************************************
 * ===== debug_print() =====
 * Prints to Serial always. Telemetry is sent via selected comms mode.
 ****************************************************/
static void debug_print(const char* msg)
{
  if (!msg) return;
  Serial.print(msg);
}

/****************************************************
 * ===== debug_print_ble() =====
 * Like debug_print(), but also forwards the message over BLE CMD_TX
 * (when connected) so it shows up on the phone's Logs page while the
 * top is spinning untethered. Only use for low-rate messages (e.g. once
 * per revolution) -- unlike telemetry, CMD_TX notifications aren't
 * downsampled, so high-rate messages here would flood the BLE link.
 ****************************************************/
static void debug_print_ble(const char* msg)
{
  debug_print(msg);
#if COMM_MODE == COMM_MODE_BLE
  if (msg && ble_client_connected && ble_cmd_tx) {
    size_t len = strlen(msg);
    if (len > 60) len = 60;
    ble_cmd_tx->setValue((uint8_t*)msg, len);
    ble_cmd_tx->notify();
  }
#endif
}

// ================= LEDS & POV CONFIG =================

static constexpr uint32_t LED_SPI_HZ = 8000000;
static constexpr uint8_t  LED_GLOBAL_BRIGHTNESS = 8;

static const char* POV_TEXT = "BOAZ";

static constexpr uint8_t POV_FG_R = 160;
static constexpr uint8_t POV_FG_G = 0;
static constexpr uint8_t POV_FG_B = 255;

static constexpr uint8_t POV_BG_R = 0;
static constexpr uint8_t POV_BG_G = 0;
static constexpr uint8_t POV_BG_B = 0;

// ================= SPIN CALIB CONFIG =================
#define CALIB_SPIN_NUM 3
static constexpr uint32_t SPIN_CALIB_TIMEOUT_MS = 3000;
// ====================================================

// ================= STATUS LED BLINK =================
#define STATUS_LED_BLINK_ENABLE true
static constexpr uint32_t STATUS_LED_BLINK_MS = 500;
static constexpr StatusLED STATUS_LED_GREEN = LED1;
static constexpr StatusLED STATUS_LED_YELLOW = LED2;
// ====================================================

// ================= SIMPLE POV TEST =================
static constexpr float TARGET_ANGLE_DEG = 0.0f;   // Light location
static constexpr uint8_t TARGET_R = 160;
static constexpr uint8_t TARGET_G = 0;
static constexpr uint8_t TARGET_B = 255;
static constexpr uint8_t ARM_ZERO_INDEX = 3;       // Arm 4 (0-based) is aligned with Luna
static constexpr uint8_t WORD_W = 36;              // 6 chars * (5 cols + 1 space)
static constexpr uint8_t WORD_H = 7;
static constexpr float WORD_COL_DEG = 4.0f;        // Column width in degrees
static constexpr float WORD_ANGLE_OFFSET_DEG = 0.0f; // Center word on zero
static constexpr bool WORD_FLIP_ROWS = true;          // Top farther from center
static uint8_t WORD_COLS[WORD_W] = {0};
static constexpr uint16_t MIN_SPIN_DPS = 60;       // Below this, LEDs stay off
static constexpr uint16_t ZERO_STALE_MS = 400;     // No zero hits => LEDs off
static constexpr uint8_t  MIN_ZERO_HITS = 5;       // Require a few zero hits to lock
static constexpr float    TILT_MAX_DEG = 15.0f;    // Ignore zero updates when tilt exceeds this
static constexpr uint32_t TILT_STABLE_MS = 200;    // Require stable tilt before accepting zero
static constexpr float    ACCEL_NORM_TOL_G = 0.25f; // Accept tilt only when |norm-1g| <= tol
// Wobble gate: blocks zero-updates when gx/gy is abnormally high.
// IMPORTANT: at high RPM a stable but slightly tilted top projects its fast spin
// onto gx/gy — so the threshold MUST scale with spin speed.
// We use: wobble_threshold = max(WOBBLE_BASE_DPS, spin_dps * WOBBLE_SPIN_RATIO)
// This is loose enough for normal tilt at any speed, but still catches violent precession.
static constexpr float    WOBBLE_BASE_DPS  = 150.0f; // minimum threshold at low/zero spin
static constexpr float    WOBBLE_SPIN_RATIO = 0.15f; // 15% of spin rate
static constexpr uint32_t WOBBLE_STABLE_MS = 100;    // ms of clean wobble before accepting zero
// Period-plausibility gate: once the spin period is known, only accept a zero
// that arrives near an integer-multiple of the established period.
// This catches sporadic noise or accidental near-reads that would corrupt the phase.
// The gate is bypassed when the system goes stale so it can re-sync.
static constexpr float    ZERO_PERIOD_TOL  = 0.22f;  // ±22% of period (~80° acceptance window)
static const uint8_t SYMBOL_PACMAN_OPEN[7]  = {0x3C, 0x42, 0x81, 0x81, 0x81, 0x42, 0x3C};
static const uint8_t SYMBOL_PACMAN_CLOSED[7]= {0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C};
// ====================================================

// ===== Debug print control =====
static constexpr uint32_t PRINT_PERIOD_MS = 100;
static uint32_t last_print_ms = 0;

// ===== Safety: clear LEDs on stale/no-motion =====
static constexpr uint32_t LUNA_STALE_MS = 300;
static uint32_t last_luna_ms = 0;

// ===== Theta integration =====
static float theta_deg = 0.0f;
static uint32_t last_theta_us = 0;

// ===== Phase lock using Luna zero events =====
static uint32_t last_zero_us = 0;
static uint32_t zero_period_us = 0;
static bool have_period = false;
static uint8_t zero_hit_count = 0;
static bool missed_zero_warned = false;

// ===== Heading slew correction (smooth catch-up at ZERO SET) =====
// theta_deg is continuously gyro-integrated and never hard-reset. At each
// zero-crossing the residual phase error vs. 0 is corrected gradually over
// HEADING_SLEW_DURATION_US by adding slew_correction_dps to the integration
// rate, instead of snapping theta_deg -- this turns a per-revolution jump
// into a brief smooth slide. Errors larger than HEADING_SNAP_THRESHOLD_DEG
// (first lock / long stale gap, nothing on-screen to preserve) snap instantly.
static float slew_correction_dps = 0.0f;
static uint32_t slew_start_us = 0;
static constexpr uint32_t HEADING_SLEW_DURATION_US = 80000;   // 80ms catch-up window
static constexpr float    HEADING_SNAP_THRESHOLD_DEG = 30.0f; // beyond this, snap instantly

// ===== Zero-crossing debounce by expected period =====
// A zero-crossing arriving sooner than this fraction of the gyro-estimated
// revolution period is treated as an extra LIDAR hit from the same object
// pass (not a new revolution) and ignored entirely.
static constexpr float ZERO_MIN_PERIOD_FRACTION = 0.6f;

// ===== Tilt stability gate =====
static uint32_t last_tilt_bad_ms = 0;
static uint32_t last_wobble_bad_ms = 0;

// ===== Gyro bias calibration =====
static float gz_bias_dps = 0.0f;

// ===== Zero reference =====
static LunaZeroRef zref;

// ===== Simple POV state =====
static bool target_led_on = false;
static uint8_t target_arm_index = 0;
static int8_t target_col_index = -1;

// ===== POV control =====
static bool pov_enabled = false;
static char pov_text[32] = {0};
static uint8_t pov_fg_r = POV_FG_R;
static uint8_t pov_fg_g = POV_FG_G;
static uint8_t pov_fg_b = POV_FG_B;
static uint8_t pov_bg_r = POV_BG_R;
static uint8_t pov_bg_g = POV_BG_G;
static uint8_t pov_bg_b = POV_BG_B;
static uint8_t led_global_brightness = LED_GLOBAL_BRIGHTNESS;
static bool pov_rgb_mode = false;
static bool pov_symbol_mode = false;
static bool pov_anim_pig = false;
static bool pov_anim_pacman = false;
static bool auto_pov_started = false;
static uint32_t boot_ms = 0;

// ===== Manual LED control =====
static bool manual_led_active = false;
static uint16_t manual_led_index = 0;

// ---- helper: update theta from gz (deg/s) ----
// theta_deg is a continuous heading estimate in [0,360), integrated from the
// live gyro every loop and never hard-reset. See the slew-correction comment
// above for how zero-crossings keep it anchored without visible jumps.
static void update_theta_from_gz(float gz_dps)
{
  uint32_t now = micros();
  if (last_theta_us == 0) {
    last_theta_us = now;
    return;
  }
  float dt = (now - last_theta_us) * 1e-6f;
  last_theta_us = now;

  if (dt <= 0.0f || dt > 0.1f) {
    return;
  }

  float omega_dps = gz_dps;
  if ((now - slew_start_us) < HEADING_SLEW_DURATION_US) {
    omega_dps += slew_correction_dps;
  }

  theta_deg += omega_dps * dt;
  theta_deg = fmodf(theta_deg, 360.0f);
  if (theta_deg < 0.0f) theta_deg += 360.0f;
}

// ---- helper: still calibration ----
static float calibrate_gz_bias(uint32_t duration_ms = 1500)
{
  const uint32_t start = millis();
  uint32_t count = 0;
  double acc = 0.0;

  while (millis() - start < duration_ms) {
    float gx, gy, gz;
    jyro_read_xyz_dps(gx, gy, gz);
    acc += gz;
    count++;
    delay(5);
  }
  return (count > 0) ? (float)(acc / count) : 0.0f;
}

/****************************************************
 * ===== SPIN-BASED BIAS CALIBRATION =====
 * - Uses ZERO SET events (block detection)
 * - Prints spin count
 * - Yellow status LED ON during calibration
 ****************************************************/
static float calibrate_gz_bias_over_spins(uint8_t spins_needed)
{
  uint8_t spins_done = 0;
  double sum_gz = 0.0;
  uint32_t samples = 0;
  const uint32_t start_ms = millis();

  Serial.println("Spin calibration started");
  statusLeds_allOn();

  while (spins_done < spins_needed) {
    if (SPIN_CALIB_TIMEOUT_MS > 0 && (millis() - start_ms) > SPIN_CALIB_TIMEOUT_MS) {
      Serial.println("Spin calibration timed out (no Luna data)");
      break;
    }

    float gx, gy, gz;
    jyro_read_xyz_dps(gx, gy, gz);
    update_theta_from_gz(gz);

    uint16_t d_mm;
    if (!luna_read_mm_latest(d_mm)) continue;

    if (luna_update_zero(zref, d_mm, theta_deg)) {
      spins_done++;
      Serial.print("Spin ");
      Serial.print(spins_done);
      Serial.print(" / ");
      Serial.println(spins_needed);
    }

    sum_gz += gz;
    samples++;
  }

  statusLeds_allOff();
  Serial.println("Spin calibration done");

  return (samples > 0) ? (float)(sum_gz / samples) : 0.0f;
}

static void set_target_leds(bool on, uint8_t arm_index)
{
  for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint8_t i = 0; i < LEDS_PER_ARM; i++) {
      uint16_t off = arm * LEDS_PER_ARM + i;
      if (on && (arm == arm_index)) {
        leds_set_rgb(off, TARGET_R, TARGET_G, TARGET_B);
      } else {
        leds_set_rgb(off, 0, 0, 0);
      }
    }
  }
  leds_show();
}

static void build_word_cols_from_text(const char* text)
{
  const uint8_t max_chars = 6;
  const uint8_t cols_per_char = 6; // 5 + 1 spacing
  const uint8_t max_cols = WORD_W;

  for (uint8_t i = 0; i < max_cols; i++) WORD_COLS[i] = 0x00;

  if (!text) return;

  uint8_t col_idx = 0;
  for (uint8_t ci = 0; ci < max_chars && text[ci]; ci++) {
    char c = (char)toupper((unsigned char)text[ci]);
    for (uint8_t col = 0; col < 5 && col_idx < max_cols; col++) {
      WORD_COLS[col_idx++] = getFontColumn(c, col);
    }
    if (col_idx < max_cols) {
      WORD_COLS[col_idx++] = 0x00; // spacing column
    }
  }
}

static void set_word_cols_from_symbol(const uint8_t* cols, uint8_t width)
{
  for (uint8_t i = 0; i < WORD_W; i++) WORD_COLS[i] = 0x00;
  if (!cols || width == 0) return;

  if (width > WORD_W) width = WORD_W;
  uint8_t start = (WORD_W - width) / 2;
  for (uint8_t i = 0; i < width; i++) {
    WORD_COLS[start + i] = cols[i];
  }
}

static bool equals_ignore_case(const char* a, const char* b)
{
  if (!a || !b) return false;
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return false;
    a++;
    b++;
  }
  return *a == '\0' && *b == '\0';
}

static bool set_symbol_by_name(const char* name)
{
  static const uint8_t SYMBOL_SMILE[7]  = {0x3E, 0x41, 0x55, 0x51, 0x55, 0x41, 0x3E};
  static const uint8_t SYMBOL_HEART[7]  = {0x06, 0x0F, 0x1F, 0x3E, 0x1F, 0x0F, 0x06};
  static const uint8_t SYMBOL_STAR[7]   = {0x10, 0x54, 0x38, 0xFE, 0x38, 0x54, 0x10};
  static const uint8_t SYMBOL_ARROW[7]  = {0x08, 0x0C, 0xFE, 0xFF, 0xFE, 0x0C, 0x08};
  static const uint8_t SYMBOL_SKULL[7]  = {0x3C, 0x7E, 0xDB, 0xFF, 0x7E, 0x24, 0x24};
  static const uint8_t SYMBOL_BATTERY[7]= {0x7F, 0x41, 0x5D, 0x5D, 0x5D, 0x41, 0x7F};
  static const uint8_t SYMBOL_PIG[7]    = {0x7E, 0x49, 0x79, 0x7F, 0x79, 0x49, 0x7E};

  if (equals_ignore_case(name, "SMILE")) {
    set_word_cols_from_symbol(SYMBOL_SMILE, 7);
    pov_anim_pig = false;
    return true;
  }
  if (equals_ignore_case(name, "HEART")) {
    set_word_cols_from_symbol(SYMBOL_HEART, 7);
    pov_anim_pig = false;
    return true;
  }
  if (equals_ignore_case(name, "STAR")) {
    set_word_cols_from_symbol(SYMBOL_STAR, 7);
    pov_anim_pig = false;
    return true;
  }
  if (equals_ignore_case(name, "ARROW")) {
    set_word_cols_from_symbol(SYMBOL_ARROW, 7);
    pov_anim_pig = false;
    return true;
  }
  if (equals_ignore_case(name, "SKULL")) {
    set_word_cols_from_symbol(SYMBOL_SKULL, 7);
    pov_anim_pig = false;
    return true;
  }
  if (equals_ignore_case(name, "BATTERY")) {
    set_word_cols_from_symbol(SYMBOL_BATTERY, 7);
    pov_anim_pig = false;
    return true;
  }
  if (equals_ignore_case(name, "PIG")) {
    set_word_cols_from_symbol(SYMBOL_PIG, 7);
    pov_anim_pig = true;
    pov_anim_pacman = false;
    return true;
  }
  if (equals_ignore_case(name, "PACMAN")) {
    set_word_cols_from_symbol(SYMBOL_PACMAN_OPEN, 7);
    pov_anim_pig = false;
    pov_anim_pacman = true;
    return true;
  }

  return false;
}

static void hsv_to_rgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
  float c = v * s;
  float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
  float m = v - c;

  float r1 = 0, g1 = 0, b1 = 0;
  if (h < 60) { r1 = c; g1 = x; b1 = 0; }
  else if (h < 120) { r1 = x; g1 = c; b1 = 0; }
  else if (h < 180) { r1 = 0; g1 = c; b1 = x; }
  else if (h < 240) { r1 = 0; g1 = x; b1 = c; }
  else if (h < 300) { r1 = x; g1 = 0; b1 = c; }
  else { r1 = c; g1 = 0; b1 = x; }

  r = (uint8_t)((r1 + m) * 255.0f);
  g = (uint8_t)((g1 + m) * 255.0f);
  b = (uint8_t)((b1 + m) * 255.0f);
}

static bool set_word_columns_all(float heading)
{
  if (pov_anim_pacman) {
    bool open = ((millis() / 250) % 2) == 0;
    set_word_cols_from_symbol(open ? SYMBOL_PACMAN_OPEN : SYMBOL_PACMAN_CLOSED, 7);
  }

  uint8_t row_offset = 0;
  if (LEDS_PER_ARM > WORD_H) {
    row_offset = (LEDS_PER_ARM - WORD_H) / 2;
  }

  uint8_t col_bits_per_arm[LED_ARM_COUNT] = {0};
  bool arm_active[LED_ARM_COUNT] = {false};
  bool any_on = false;

  for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    float arm_heading = fmodf(heading + (float)(arm - ARM_ZERO_INDEX) * 90.0f, 360.0f);
    if (arm_heading < 0.0f) arm_heading += 360.0f;

    int col = (int)(arm_heading / WORD_COL_DEG);
    if (col >= 0 && col < (int)WORD_W) {
      col_bits_per_arm[arm] = WORD_COLS[col];
      arm_active[arm] = true;
      any_on = true;
    }
  }

  for (uint8_t arm = 0; arm < LED_ARM_COUNT; arm++) {
    for (uint8_t i = 0; i < LEDS_PER_ARM; i++) {
      uint16_t off = arm * LEDS_PER_ARM + i;
      bool on = false;
      int row_shift = 0;
      if (pov_anim_pig) {
        row_shift = (int)((millis() / 300) % 3) - 1;
      }

      if (arm_active[arm]) {
        if (i >= row_offset && i < (uint8_t)(row_offset + WORD_H)) {
          uint8_t row = i - row_offset;
          if (WORD_FLIP_ROWS) {
            row = (WORD_H - 1) - row;
          }
          int bit_row = (int)row + row_shift;
          if (bit_row >= 0 && bit_row < (int)WORD_H) {
            on = ((col_bits_per_arm[arm] >> bit_row) & 0x01) != 0;
          }
        }
      }

      if (on) {
        if (pov_rgb_mode) {
          float hue = fmodf((millis() * 0.04f) + (off * 12.0f), 360.0f);
          uint8_t r, g, b;
          hsv_to_rgb(hue, 1.0f, 1.0f, r, g, b);
          leds_set_rgb(off, r, g, b);
        } else {
          leds_set_rgb(off, pov_fg_r, pov_fg_g, pov_fg_b);
        }
      } else {
        leds_set_rgb(off, pov_bg_r, pov_bg_g, pov_bg_b);
      }
    }
  }

  leds_show();
  return any_on;
}

static float angle_delta_deg(float a, float b)
{
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return d;
}

static void send_cmd_response(const char* msg)
{
  if (!msg) return;
#if COMM_MODE == COMM_MODE_WIFI
  udp_cmd.beginPacket(udp_cmd.remoteIP(), udp_cmd.remotePort());
  udp_cmd.write((const uint8_t*)msg, (int)strlen(msg));
  udp_cmd.endPacket();
#elif COMM_MODE == COMM_MODE_BLE
  if (!ble_client_connected || !ble_cmd_tx) return;
  ble_cmd_tx->setValue(msg);
  ble_cmd_tx->notify();
#endif
}

static bool parse_u16(const char* s, uint16_t &out)
{
  if (!s || !*s) return false;
  char* end = nullptr;
  unsigned long v = strtoul(s, &end, 10);
  if (end == s) return false;
  out = (uint16_t)v;
  return true;
}

static uint8_t parse_u8(const char* s, uint8_t def = 0)
{
  uint16_t v = 0;
  if (!parse_u16(s, v)) return def;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

static void handle_command_line(char* buf)
{
  if (!buf) return;
  char* tok = strtok(buf, " \r\n");
  if (!tok) return;

  if (strcmp(tok, "POV") == 0) {
    char* sub = strtok(nullptr, " \r\n");
    if (!sub) return;

    if (strcmp(sub, "START") == 0) {
      pov_enabled = true;
      manual_led_active = false;
      pov_init(pov_text, pov_fg_r, pov_fg_g, pov_fg_b, pov_bg_r, pov_bg_g, pov_bg_b, led_global_brightness);
      send_cmd_response("OK POV START\n");
      return;
    }

    if (strcmp(sub, "STOP") == 0) {
      pov_enabled = false;
      manual_led_active = false;
      leds_clear();
      leds_show();
      send_cmd_response("OK POV STOP\n");
      return;
    }

    if (strcmp(sub, "TEXT") == 0) {
      char* rest = strtok(nullptr, "");
      if (rest) {
        while (*rest == ' ') rest++;
        strncpy(pov_text, rest, sizeof(pov_text) - 1);
        pov_text[sizeof(pov_text) - 1] = '\0';
        pov_symbol_mode = false;
        pov_anim_pig = false;
        build_word_cols_from_text(pov_text);
        if (pov_enabled) {
          pov_init(pov_text, pov_fg_r, pov_fg_g, pov_fg_b, pov_bg_r, pov_bg_g, pov_bg_b, led_global_brightness);
        }
        send_cmd_response("OK POV TEXT\n");
      }
      return;
    }

    if (strcmp(sub, "SYMBOL") == 0) {
      char* name = strtok(nullptr, " \r\n");
      if (!name) return;
      if (set_symbol_by_name(name)) {
        pov_symbol_mode = true;
        if (pov_enabled) {
          pov_init(pov_text, pov_fg_r, pov_fg_g, pov_fg_b, pov_bg_r, pov_bg_g, pov_bg_b, led_global_brightness);
        }
        send_cmd_response("OK POV SYMBOL\n");
      } else {
        send_cmd_response("ERR POV SYMBOL\n");
      }
      return;
    }

    if (strcmp(sub, "MODE") == 0) {
      char* mode = strtok(nullptr, " \r\n");
      if (!mode) return;
      if (equals_ignore_case(mode, "RGB")) {
        pov_rgb_mode = true;
        send_cmd_response("OK POV MODE RGB\n");
      } else if (equals_ignore_case(mode, "SOLID")) {
        pov_rgb_mode = false;
        send_cmd_response("OK POV MODE SOLID\n");
      }
      return;
    }

    if (strcmp(sub, "COLORS") == 0) {
      pov_fg_r = parse_u8(strtok(nullptr, " \r\n"), pov_fg_r);
      pov_fg_g = parse_u8(strtok(nullptr, " \r\n"), pov_fg_g);
      pov_fg_b = parse_u8(strtok(nullptr, " \r\n"), pov_fg_b);
      pov_bg_r = parse_u8(strtok(nullptr, " \r\n"), pov_bg_r);
      pov_bg_g = parse_u8(strtok(nullptr, " \r\n"), pov_bg_g);
      pov_bg_b = parse_u8(strtok(nullptr, " \r\n"), pov_bg_b);
      if (pov_enabled) {
        pov_init(pov_text, pov_fg_r, pov_fg_g, pov_fg_b, pov_bg_r, pov_bg_g, pov_bg_b, led_global_brightness);
      }
      send_cmd_response("OK POV COLORS\n");
      return;
    }
  }

  if (strcmp(tok, "LED") == 0) {
    char* sub = strtok(nullptr, " \r\n");
    if (!sub) return;

    if (strcmp(sub, "BRIGHTNESS") == 0) {
      uint16_t b = 0;
      if (parse_u16(strtok(nullptr, " \r\n"), b)) {
        led_global_brightness = (uint8_t)b;
        leds_set_global_brightness(led_global_brightness);
        leds_show();
        send_cmd_response("OK LED BRIGHTNESS\n");
      }
      return;
    }

    if (strcmp(sub, "SINGLE") == 0) {
      if (pov_enabled) {
        send_cmd_response("ERR POV ON\n");
        return;
      }

      uint16_t idx = 0;
      if (!parse_u16(strtok(nullptr, " \r\n"), idx)) return;
      if (idx >= LED_COUNT) {
        send_cmd_response("ERR IDX\n");
        return;
      }
      uint8_t r = parse_u8(strtok(nullptr, " \r\n"));
      uint8_t g = parse_u8(strtok(nullptr, " \r\n"));
      uint8_t b = parse_u8(strtok(nullptr, " \r\n"));

      leds_clear();
      leds_set_rgb(idx, r, g, b);
      leds_show();

      manual_led_active = true;
      manual_led_index = idx;
      send_cmd_response("OK LED SINGLE\n");
      return;
    }

    if (strcmp(sub, "CLEAR") == 0) {
      leds_clear();
      leds_show();
      manual_led_active = false;
      send_cmd_response("OK LED CLEAR\n");
      return;
    }
  }

  if (strcmp(tok, "STATUS") == 0) {
    char resp[80];
    snprintf(resp, sizeof(resp), "POV=%s MANUAL=%s IDX=%u\n",
             pov_enabled ? "ON" : "OFF",
             manual_led_active ? "ON" : "OFF",
             manual_led_index);
    send_cmd_response(resp);
    return;
  }
}

static void handle_udp_command()
{
#if COMM_MODE == COMM_MODE_WIFI
  int packetSize = udp_cmd.parsePacket();
  if (packetSize <= 0) return;

  char buf[128];
  int len = udp_cmd.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  handle_command_line(buf);
#endif
}

static void comm_init()
{
#if COMM_MODE == COMM_MODE_WIFI
  wifi_debug_init();
#elif COMM_MODE == COMM_MODE_BLE
  ble_debug_init();
#endif
}

static void comm_send_telemetry(uint16_t dist_mm, float gz_dps, float rpm,
                                float wobble, float rps)
{
#if COMM_MODE == COMM_MODE_WIFI
  wifi_debug_send_telemetry(dist_mm, gz_dps, rpm, wobble, rps);
#elif COMM_MODE == COMM_MODE_BLE
  ble_send_telemetry(dist_mm, gz_dps, rpm, wobble, rps);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(200);

  boot_ms = millis();
  last_luna_ms = boot_ms;

  comm_init();

  statusLeds_init();
  statusLeds_allOff();

#if STATUS_LED_BLINK_ENABLE
  statusLed_off(STATUS_LED_GREEN);
  statusLed_startBlink(STATUS_LED_YELLOW, STATUS_LED_BLINK_MS);
#endif

  jyro_init();

  Serial.println("Hold still: calibrating gyro Z bias...");
  gz_bias_dps = calibrate_gz_bias(1500);

  Serial.print("gz_bias_dps (still) = ");
  Serial.println(gz_bias_dps, 4);

  luna_init_mm();

  // TEMP: reverted to hits_needed=2 / fars_needed=2 (matches older src.zip logic)
  // to test whether the extra debounce reduces drift/jump at the zero point.
  zref.hits_needed = 2;
  zref.fars_needed = 2;

  leds_init(LED_SPI_HZ, led_global_brightness);
  strncpy(pov_text, POV_TEXT, sizeof(pov_text) - 1);
  pov_text[sizeof(pov_text) - 1] = '\0';
  build_word_cols_from_text(pov_text);

  if (CALIB_SPIN_NUM > 0) {
    Serial.print("Spin the top for ");
    Serial.print(CALIB_SPIN_NUM);
    Serial.println(" revolutions...");
    gz_bias_dps = calibrate_gz_bias_over_spins(CALIB_SPIN_NUM);
    Serial.print("gz_bias_dps (FINAL) = ");
    Serial.println(gz_bias_dps, 4);
  }

#if STATUS_LED_BLINK_ENABLE
  statusLed_stopBlink(STATUS_LED_YELLOW);
  statusLed_off(STATUS_LED_YELLOW);
  statusLed_on(STATUS_LED_GREEN);
#endif

  Serial.println("System running (POV standby)");
}

void loop() {
  handle_udp_command();

  // --- Read gyro ---
  float gx, gy, gz;
  jyro_read_xyz_dps(gx, gy, gz);

  // Correct bias
  float gz_corr = gz - gz_bias_dps;

  // Integrate to get theta
  update_theta_from_gz(gz_corr);

  float spin_dps_abs = fabsf(gz_corr);

  uint32_t now_ms = millis();

  // --- Read Luna (latest) ---
  uint16_t d_mm;
  if (luna_read_mm_latest(d_mm)) {
    last_luna_ms = millis();

    // Detect block pass -> defines 0 deg
    bool zero_detected = luna_update_zero(zref, d_mm, theta_deg);
    if (zero_detected) {
      uint32_t now_us = micros();

      // Reject zero-crossings that arrive too soon to plausibly be a new
      // revolution, based on the gyro's current spin rate (independent of
      // LIDAR history). This filters out an extra LIDAR hit from the same
      // object pass rather than treating it as the next revolution.
      bool too_soon = false;
      if (last_zero_us != 0 && spin_dps_abs > 1.0f) {
        float expected_period_us = (360.0f / spin_dps_abs) * 1e6f;
        too_soon = (float)(now_us - last_zero_us) < (ZERO_MIN_PERIOD_FRACTION * expected_period_us);
      }

      if (too_soon) {
        debug_print_ble("ZERO IGNORED (too soon)\n");
      } else {
        zero_hit_count++;
        if (last_zero_us != 0) {
          zero_period_us = now_us - last_zero_us;
          if (zero_period_us > 0) {
            have_period = true;
          }
        }
        last_zero_us = now_us;

        // theta_deg should be ~0 here (one full revolution since the last
        // ZERO SET). Wrap any residual error to [-180,180] and correct it
        // gradually (slew) rather than snapping -- unless it's large enough
        // that nothing on-screen is worth preserving (first lock / long gap).
        {
          float error_deg = theta_deg;
          if (error_deg > 180.0f) error_deg -= 360.0f;

          // DIAGNOSTIC: theta_deg should be ~0/360 here if gyro integration
          // matches the true 360 deg/revolution. Print it to check for a
          // systematic rate mismatch (e.g. landing near 180 every time).
          char dbg[64];
          snprintf(dbg, sizeof(dbg), "ZERO SET theta=%.1f err=%.1f per_ms=%.1f\n",
                   theta_deg, error_deg, (float)zero_period_us / 1000.0f);
          debug_print_ble(dbg);

          if (!have_period || fabsf(error_deg) > HEADING_SNAP_THRESHOLD_DEG) {
            theta_deg = 0.0f;
            slew_correction_dps = 0.0f;
          } else {
            slew_correction_dps = -error_deg / (HEADING_SLEW_DURATION_US * 1e-6f);
            slew_start_us = now_us;
          }
        }
        zref.zero_offset_deg = 0.0f;
        zref.has_zero = true;
        missed_zero_warned = false;
      }
    }

    // Diagnostic: a zero-crossing should arrive roughly once per
    // zero_period_us. If it's significantly overdue, the LIDAR likely
    // missed the block on this pass (whole revolution skipped).
    if (have_period && last_zero_us != 0 && zero_period_us > 0 && !missed_zero_warned) {
      uint32_t elapsed_us = micros() - last_zero_us;
      if (elapsed_us > (uint32_t)(1.5f * (float)zero_period_us)) {
        missed_zero_warned = true;
        debug_print_ble("MISSED ZERO (revolution overdue)\n");
      }
    }

    if (pov_enabled) {
      // Simple POV: light one LED on the arm closest to the target angle
      bool spin_ok = (spin_dps_abs >= (float)MIN_SPIN_DPS);
      bool zero_fresh = (last_zero_us != 0) && ((millis() - (last_zero_us / 1000)) <= ZERO_STALE_MS);
      bool pov_ready = spin_ok && have_period && zero_fresh && (zero_hit_count >= MIN_ZERO_HITS);

      // theta_deg is the continuously gyro-integrated heading (see
      // update_theta_from_gz / slew-correction comments above) -- already
      // wrapped to [0,360).
      float heading = NAN;
      bool heading_valid = false;
      if (pov_ready) {
        heading = theta_deg;
        heading_valid = true;
      }
      if (heading_valid) {
        float word_heading = fmodf(heading + WORD_ANGLE_OFFSET_DEG, 360.0f);
        target_led_on = set_word_columns_all(word_heading);
        if (target_led_on) {
          debug_print("POV target hit: LED on\n");
        }
      }
      // TEMP: when pov_ready/heading is not valid (e.g. a brief gap before
      // the next ZERO SET), freeze the last rendered frame instead of
      // blanking all LEDs -- avoids the text "disappearing" on short blips.
    } else if (target_led_on) {
      target_led_on = false;
      target_col_index = -1;
      set_target_leds(false, target_arm_index);
    }

    // Debug print at low rate
    if (now_ms - last_print_ms >= PRINT_PERIOD_MS) {
      last_print_ms = now_ms;

      float spin_dps = fabsf(gz_corr);
      float rpm = jyro_spin_dps_to_rpm(spin_dps);
      float rps = spin_dps / 360.0f;
      float wobble = sqrtf(gx*gx + gy*gy);

      char buf[160];
      if (pov_enabled && have_period) {
        snprintf(buf, sizeof(buf),
                 "d_mm=%u  gz=%.1f dps  rpm=%.1f  wobble=%.1f  rps=%.2f\n",
                 d_mm, gz_corr, rpm, wobble, rps);

  comm_send_telemetry(d_mm, gz_corr, rpm, wobble, rps);
      } else {
        snprintf(buf, sizeof(buf),
                 "d_mm=%u  gz=%.1f dps  rpm=%.1f  wobble=%.1f  rps=%.2f\n",
                 d_mm, gz_corr, rpm, wobble, rps);

  comm_send_telemetry(d_mm, gz_corr, rpm, wobble, rps);
      }

      debug_print(buf);
    }
  }

#if STATUS_LED_BLINK_ENABLE
  statusLeds_update();
#endif

  // No delay: keep loop fast.
}
