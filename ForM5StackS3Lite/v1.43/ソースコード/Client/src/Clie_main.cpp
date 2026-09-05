/*=============================================================================*
 * M5Stack ATOM S3 Lite クライアント用ファームウェア  Ver1.42
 * USB-NCMモード: iOS / Android と USB-C 直結 (192.168.7.x ネットワーク)
 *   - ESP32-S3 が USB-NCM デバイスとして動作
 *   - iOS/Android アプリが中継 (NCM UDP 8888 ↔ Server UDP 8889)
 * WiFiモード: Server AP へ直接接続 (従来と同じ)
 * 共通: EEPROM プロファイル, SDカードなし, SYNCタイミング, PING/PONG
 *
 * USBNCM.h は arduino-esp32 >= 3.1.0 (espressif32 >= 6.9.0) が必要。
 * 旧バージョンでは HAS_USBNCM=0 となり USB-CDC シリアルトンネルに自動フォールバック。
 *
 * Copyright (C) 2025- JI1ORE
 *=============================================================================*/
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Adafruit_NeoPixel.h>
#include "esp_wifi.h"
#include <EEPROM.h>
#include "M5Unified.h"

// BLE NUS (Nordic UART Service) — same library as DualKey so Android can connect
// without pairing/bonding (BLESecurity disables it explicitly).
// ATOM Lite (ESP32) と ATOM S3 Lite (ESP32-S3) の両方で有効。
#include <BLEDevice.h>
#include <BLEServer.h>
#define HAS_BT 1
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // Phone → Atom (write)
#define NUS_TX_CHAR_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // Atom → Phone (notify)
static BLEServer         *bleServer    = nullptr;
static BLECharacteristic *bleTxChar    = nullptr;
static bool               bleConnected = false;
static uint8_t            bleRxBuf[64];
static int                bleRxLen     = 0;
class BleSrvCb : public BLEServerCallbacks {
    void onConnect(BLEServer*)    override { bleConnected = true; }
    void onDisconnect(BLEServer*) override {
        bleConnected = false;
        BLEDevice::startAdvertising();
    }
};
class BleRxCb : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *c) override {
        String v = c->getValue();
        int n = (int)v.length();
        if (n <= 0) return;
        if (bleRxLen + n > (int)sizeof(bleRxBuf)) {
            int drop = (bleRxLen + n) - (int)sizeof(bleRxBuf);
            memmove(bleRxBuf, bleRxBuf + drop, bleRxLen - drop);
            bleRxLen -= drop;
        }
        memcpy(bleRxBuf + bleRxLen, v.c_str(), n);
        bleRxLen += n;
    }
};

// ---- USB-NCM ----------------------------------------------------------------
#if __has_include("USB.h")
  #include "USB.h"
#endif
#if defined(FORCE_CDC_MODE)
  #define HAS_USBNCM 0
#elif __has_include("USBNCM.h")
  #include "USBNCM.h"
  static USBNCM ncm;
  #define HAS_USBNCM 1
#else
  #define HAS_USBNCM 0
#endif

// NCM インタフェース固定 IP
static const IPAddress NCM_LOCAL_IP(192, 168, 7, 1);   // ATOM S3 Lite (デバイス)
static const IPAddress NCM_HOST_IP (192, 168, 7, 2);   // iOS/Android (ホスト, DHCP割当)
static const IPAddress NCM_SUBNET  (255, 255, 255, 252);

// ---- EEPROM -----------------------------------------------------------------
#define EEPROM_ADDR_PROFILE_INDEX  0
#define EEPROM_ADDR_USB_MODE       1
#define EEPROM_ADDR_NCM_MODE       2   // 1=NCM(iOS), 0=CDC(Android)
#define EEPROM_ADDR_BT_MODE        3   // 1=Bluetooth SPP, 0=USB
#define EEPROM_ADDR_PROFILE_BASE  16
#define MAX_WIFI_PROFILES          5
#define EEPROM_SIZE (sizeof(WiFiProfileEEPROM) * MAX_WIFI_PROFILES + sizeof(SystemSettingsEEPROM) + 16)
#define EEPROM_ADDR_SETTINGS (EEPROM_ADDR_PROFILE_BASE + sizeof(WiFiProfileEEPROM) * MAX_WIFI_PROFILES)

#define MAX_SSID_LEN     32
#define MAX_PASS_LEN     64
#define MAX_HOSTNAME_LEN 32
#define PROFILE_MAGIC    0xA5

struct __attribute__((packed)) WiFiProfileEEPROM {
  uint8_t magic;
  char    ssid[MAX_SSID_LEN];
  char    password[MAX_PASS_LEN];
  uint8_t localIP[4];
  uint8_t gateway[4];
  uint8_t subnet[4];
  uint8_t serverIP[4];
  bool    isApMode;
  bool    hasIP;
  uint8_t colorId;
  char    hostname[MAX_HOSTNAME_LEN];
  char    serverhostname[MAX_HOSTNAME_LEN];
};

struct __attribute__((packed)) SystemSettingsEEPROM {
  uint8_t       magic;
  int           keyRepeatCount;
  int           sampleSendDupCnt;
  int           sampleIntervalMs;
  int           sampleSendAmnt;
  unsigned long debounceUs;
};

// ---- ピン定義 ---------------------------------------------------------------
#ifdef M5ATOMLite
// ATOM Lite (ESP32)
const int KEY_PIN        = 19;
const int LED_PIN        = 27;
const int BUTTON_PIN     = 39;
const int SD_SPI_CS_PIN  = -1;
#define MBTN_PIN 32
#define DATA_PIN  26
#else
// ATOM S3 Lite (ESP32-S3)
const int KEY_PIN        = 6;
const int LED_PIN        = 35;
const int BUTTON_PIN     = 41;
const int SD_SPI_CS_PIN  = -1;
#define MBTN_PIN 38
#define DATA_PIN  2
#endif
#define NUMPIXELS 1
#define ESP_INTR_FLAG_DEFAULT 0

// ---- 色設定 ------------------------------------------------------------------
struct ColorProfile { const char *name; uint8_t r, g, b; };
static ColorProfile MagentaProfile = {"Magenta", 255,   0, 255};
static ColorProfile OrangeProfile  = {"Orange",  255, 165,   0};
static ColorProfile yellowProfile  = {"Yellow",  255, 255,   0};
static ColorProfile cyanProfile    = {"Cyan",      0, 255, 255};
static ColorProfile RedProfile     = {"Red",     255,   0,   0};
static ColorProfile BlueProfile    = {"Blue",      0,   0, 255};
static ColorProfile GreenProfile   = {"Green",     0, 255,   0};
static ColorProfile WhiteProfile   = {"White",   255, 255, 255};

// ---- WiFi プロファイル -------------------------------------------------------
struct WiFiProfile {
  const char *ssid;
  const char *password;
  IPAddress   localIP, gateway, subnet, serverIP;
  bool        isApMode;
  bool        hasIP;
  ColorProfile *color;
  const char *hostname;
  const char *serverhostname;
};

static WiFiProfile wifiProfiles[MAX_WIFI_PROFILES];
static int profileCount = 0;

static WiFiProfile fallbackProfile = {
  "RemoteKeyer-AP", "password123",
  IPAddress(192,168,4,2), IPAddress(192,168,4,1), IPAddress(255,255,255,0), IPAddress(192,168,4,1),
  true, false, &cyanProfile, "RKCLIENT01", "RKSERVER01"
};
static const bool fallbackUsbMode = false;

static IPAddress serverIP;

// ---- UDP --------------------------------------------------------------------
#define LOCAL_PORT  8888
#define SERVER_PORT 8888
static WiFiUDP Udp;

// ---- LED --------------------------------------------------------------------
static Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
static Adafruit_NeoPixel pixelData(1, DATA_PIN, NEO_GRB + NEO_KHZ800);

// ---- 動作パラメータ ----------------------------------------------------------
static unsigned long MAX_ON_DURATION    = 3000;
static const int     PING_INTERVAL      = 5000;
static const int     ReCal_RTT          = 5000;
static int           keyRepeatCount     = 10;
static int           SAMPLE_SEND_DUP_CNT = 1;
static int           SAMPLE_INTERVAL_MS  = 10;
static int           SAMPLE_SEND_AMNT    = 8;
static unsigned long DEBOUNCE_US         = 5000;
static const int     HOSTNAME_RESTIME    = 30;

static const int RTT_THRESH_1 = 5,  RTT_THRESH_2 = 10,  RTT_THRESH_3 = 25;
static const int RTT_THRESH_4 = 50, RTT_THRESH_5 = 75,  RTT_THRESH_6 = 100, RTT_THRESH_7 = 200;
static const int DELAY_VERY_LOW=100, DELAY_LOW=100, DELAY_MID=100, DELAY_HIGH=100;
static const int DELAY_VERY_HIGH=150, DELAY_VERY2_HIGH=150, DELAY_VERY3_HIGH=250, DELAY_MAX=500;

static const unsigned long BLINK_INTERVAL_WIFI = 1000;
static const unsigned long BLINK_INTERVAL_SYNC =  300;
static unsigned long blinkInterval = BLINK_INTERVAL_WIFI;

// ---- 状態変数 ----------------------------------------------------------------
volatile bool          immediateSendRequested = false;
volatile bool          keyState               = false;
volatile unsigned long lastIsrMicros          = 0;
volatile unsigned long lastKeyChangeMicros    = 0;

static unsigned long lastOnMillis          = 0;
static unsigned long lastBlinkMillis       = 0;
static unsigned long lastPingMillis        = 0;
static unsigned long lastPingSendMillis    = 0;
static unsigned long lastSyncRequestMillis = 0;
static long          delayOffsetMs         = 0;
static bool          timeSynced            = false;
static bool          ledState              = false;
static unsigned long dynamicTimeout        = 8000;
static unsigned long lastDelayRecalcMillis = 0;
static int           fixedDelayMs          = 50;
static bool          fixedDelayLocked      = false;
static bool          currentKeyState       = false;
static bool          usbMode               = false;
static bool          ncmMode               = true;  // true=NCM(iOS), false=CDC(Android)
static bool          btMode                = false;  // true=Bluetooth SPP
static int           lostPingCount         = 0;
static bool          pongReceived          = true;
static const unsigned long DEFAULT_TIMEOUT = 12000;
static int           activeProfileIndex    = -1;
static unsigned long profileSelectStart    = 0;
static const unsigned long PROFILE_SELECT_WINDOW = 10000;
static int           selectedProfileIndex  = 0;
static unsigned long fixedDelayLockUntil   = 0;
static unsigned long lastImmediateSendMillis = 0;

// ---- サンプリング ------------------------------------------------------------
static byte          keyBuf          = 0;
static int           sampleIndex     = 0;
static unsigned long sampleStartTimeMs = 0;
static unsigned long lastSampleTimeMs  = 0;
static unsigned long sampleTimes[16];

// ---- RTT / SYNC --------------------------------------------------------------
static const int RTT_HISTORY_SIZE = 5;
static long      rttHistory[RTT_HISTORY_SIZE] = {0};
static int       rttIndex = 0;
static long      pingRttMs = 0;
static const int SYNC_SAMPLE_COUNT = 10;
static int       syncSampleIndex   = 0;
static long      syncOffsets[SYNC_SAMPLE_COUNT];
static bool      syncCollecting    = false;

// ---- 関数プロトタイプ --------------------------------------------------------
bool connectToWiFi();
void requestMillisSync();
void sendKeyPacket(bool isOn, byte trxSel, uint64_t opTimeMs);
void initLed();
void clearLed();
void setLedFromProfileIndex(int index);
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void setDataColor(uint8_t r, uint8_t g, uint8_t b);
void setLedColorFromProfile(ColorProfile *color);
void flashLedProfile(ColorProfile *color, int durationMs, int repeat);
void flashOnce(ColorProfile *color, int durationMs);
void flashRedBeforeRestart();
void saveProfileIndexToEEPROM(int index);
int  loadProfileIndexFromEEPROM();
void saveWiFiProfileToEEPROM(const WiFiProfile &src, int index);
bool loadWiFiProfileFromEEPROM(WiFiProfile &dst, int index);
void clearEEPROM();
bool restoreProfilesFromEEPROM();
void getserverIP();
void saveSystemSettingsToEEPROM(const SystemSettingsEEPROM &settings);
bool loadSystemSettingsFromEEPROM(SystemSettingsEEPROM &settings);
int  rttToDelayMs(long rttMs);
void freeWiFiProfileStrings(WiFiProfile &p);
long getAverageRtt();
bool initNcm();
bool initBluetooth();

// ---- ISR --------------------------------------------------------------------
void IRAM_ATTR keyISR(void *arg) {
  if (!timeSynced) return;
  unsigned long now = micros();
  if (now - lastIsrMicros < DEBOUNCE_US) return;
  lastIsrMicros = now;
  keyState = (digitalRead(KEY_PIN) == LOW);
  lastKeyChangeMicros = now;
  immediateSendRequested = true;
}

// ---- ユーティリティ ----------------------------------------------------------
int rttToDelayMs(long rttMs) {
  if (rttMs <= RTT_THRESH_1) return DELAY_VERY_LOW;
  if (rttMs <= RTT_THRESH_2) return DELAY_LOW;
  if (rttMs <= RTT_THRESH_3) return DELAY_MID;
  if (rttMs <= RTT_THRESH_4) return DELAY_HIGH;
  if (rttMs <= RTT_THRESH_5) return DELAY_VERY_HIGH;
  if (rttMs <= RTT_THRESH_6) return DELAY_VERY2_HIGH;
  if (rttMs <= RTT_THRESH_7) return DELAY_VERY3_HIGH;
  return DELAY_MAX;
}

long getAverageRtt() {
  long sum = 0;
  for (int i = 0; i < RTT_HISTORY_SIZE; i++) sum += rttHistory[i];
  return sum / RTT_HISTORY_SIZE;
}

void freeWiFiProfileStrings(WiFiProfile &p) {
  if (p.ssid)           { free(const_cast<char*>(p.ssid));           p.ssid           = nullptr; }
  if (p.password)       { free(const_cast<char*>(p.password));       p.password       = nullptr; }
  if (p.hostname)       { free(const_cast<char*>(p.hostname));       p.hostname       = nullptr; }
  if (p.serverhostname) { free(const_cast<char*>(p.serverhostname)); p.serverhostname = nullptr; }
}

// ---- USB-NCM 初期化 ----------------------------------------------------------
bool initNcm() {
#if HAS_USBNCM
  ncm.setLocalIP(NCM_LOCAL_IP);
  ncm.setSubnetMask(NCM_SUBNET);
  ncm.begin();
  USB.begin();
  // ホスト接続を最大 15 秒待つ
  unsigned long t = millis();
  while (!ncm && millis() - t < 15000) {
    setLedColor(255, 0, 0);  // 赤: 接続待ち
    delay(200);
    clearLed();
    delay(200);
  }
  return (bool)ncm;
#else
  return false;  // フォールバック: シリアルトンネル使用
#endif
}

// ---- Bluetooth 初期化 -------------------------------------------------------
#if HAS_BT
static inline bool bleHasClient() { return bleConnected; }
static int bleAvailable() { return bleRxLen; }
static int bleReadByte() {
  if (bleRxLen <= 0) return -1;
  int b = bleRxBuf[0];
  memmove(bleRxBuf, bleRxBuf + 1, bleRxLen - 1);
  bleRxLen--;
  return b;
}
static void bleWriteBytes(const uint8_t* data, size_t len) {
  if (!bleTxChar || !bleConnected) return;
  bleTxChar->setValue((uint8_t*)data, len);
  bleTxChar->notify();
}
static void bleEnd() {
  if (bleServer) {
    BLEDevice::stopAdvertising();
    BLEDevice::deinit(true);
    bleServer = nullptr;
    bleTxChar = nullptr;
  }
  bleConnected = false;
  bleRxLen = 0;
}
#endif

bool initBluetooth() {
#if HAS_BT
  Serial.println("[BLE] initBluetooth: start");
  if (bleServer) { Serial.println("[BLE] already initialized"); return true; }

  BLEDevice::init("RemoteKeyer-BLE");
  BLESecurity::setAuthenticationMode(false, false, false);  // ボンディング不要: Androidが再ペアリングなしで接続可能
  BLESecurity::resetSecurity();                             // 旧ボンド情報をクリア

  bleServer = BLEDevice::createServer();
  if (!bleServer) { Serial.println("[BLE] createServer FAILED"); return false; }
  bleServer->setCallbacks(new BleSrvCb());

  BLEService *svc = bleServer->createService(NUS_SERVICE_UUID);
  if (!svc) { Serial.println("[BLE] createService FAILED"); return false; }

  bleTxChar = svc->createCharacteristic(NUS_TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  if (!bleTxChar) { Serial.println("[BLE] TX char FAILED"); return false; }

  BLECharacteristic *rxChar = svc->createCharacteristic(
      NUS_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  if (!rxChar) { Serial.println("[BLE] RX char FAILED"); return false; }
  rxChar->setCallbacks(new BleRxCb());

  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  if (!adv) { Serial.println("[BLE] getAdvertising FAILED"); return false; }
  adv->addServiceUUID(NUS_SERVICE_UUID);
  // 広告パケット(31B)はUUIDで埋まるので名前はスキャンレスポンスに入れる
  BLEAdvertisementData scanData;
  scanData.setName("RemoteKeyer-BLE");
  adv->setScanResponseData(scanData);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] initBluetooth: done");
  return true;
#else
  return false;
#endif
}

// ---- setup() ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  auto cfg = M5.config();
  cfg.external_spk = false;
  cfg.internal_mic = false;
  cfg.output_power = false;
  cfg.clear_display = false;
  M5.begin(cfg);

  // SD カードなし (SD_SPI_CS_PIN == -1) → スキップ
  // EEPROMからプロファイルを復元
  if (!restoreProfilesFromEEPROM()) {
    wifiProfiles[0]      = fallbackProfile;
    profileCount         = 1;
    selectedProfileIndex = 0;
    if (fallbackUsbMode) {
      EEPROM.write(EEPROM_ADDR_USB_MODE, 1);
      EEPROM.commit();
    }
  }
  if (selectedProfileIndex >= profileCount) selectedProfileIndex = 0;

  SystemSettingsEEPROM sysSettings;
  if (loadSystemSettingsFromEEPROM(sysSettings)) {
    keyRepeatCount   = constrain(sysSettings.keyRepeatCount,   1, 50);
    SAMPLE_SEND_DUP_CNT = constrain(sysSettings.sampleSendDupCnt, 1, 10);
    SAMPLE_INTERVAL_MS  = constrain(sysSettings.sampleIntervalMs,  1, 100);
    SAMPLE_SEND_AMNT    = constrain(sysSettings.sampleSendAmnt,    1, 16);
    DEBOUNCE_US         = constrain((long)sysSettings.debounceUs, 100L, 50000L);
  } else {
    sysSettings = {0xA5, 10, 1, 10, 8, 5000};
    saveSystemSettingsToEEPROM(sysSettings);
  }

  selectedProfileIndex = loadProfileIndexFromEEPROM();
  activeProfileIndex   = selectedProfileIndex;
  usbMode = (EEPROM.read(EEPROM_ADDR_USB_MODE) == 1);
  ncmMode = (EEPROM.read(EEPROM_ADDR_NCM_MODE) == 1);  // 0x01=NCM(iOS), それ以外(0xFF未初期化含む)=CDC(Android)
  btMode  = (EEPROM.read(EEPROM_ADDR_BT_MODE)  == 1);

  if (profileCount == 0 && !usbMode) return;
  if (selectedProfileIndex >= profileCount) selectedProfileIndex = 0;
  activeProfileIndex = selectedProfileIndex;

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(KEY_PIN,    INPUT_PULLUP);
  pinMode(MBTN_PIN,   INPUT_PULLUP);
  profileSelectStart = millis();

  auto showUsbLed = [&]() {
    if (btMode)       setLedColorFromProfile(&RedProfile);     // BT=赤
    else if (ncmMode) setLedColorFromProfile(&OrangeProfile);  // NCM=オレンジ (iOS)
    else              setLedColorFromProfile(&cyanProfile);    // CDC=シアン (Android)
  };

  initLed();
  if (usbMode) showUsbLed();
  else         setLedFromProfileIndex(selectedProfileIndex);

  // 10 秒間: 短押し=WiFiモード時プロファイル切替/USBモード時NCM⇔CDC切替
  //          長押し 1 秒=USB/WiFi 切替
  while (millis() - profileSelectStart < PROFILE_SELECT_WINDOW) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      unsigned long pressStart = millis();
      delay(20);
      while (digitalRead(BUTTON_PIN) == LOW && millis() - pressStart < 3000) delay(10);
      unsigned long dur = millis() - pressStart;
      if (dur >= 1000) {
        usbMode = !usbMode;
        for (int i = 0; i < 3; i++) {
          clearLed(); delay(150);
          if (usbMode) showUsbLed();
          else         setLedFromProfileIndex(selectedProfileIndex);
          delay(150);
        }
      } else if (usbMode) {
#if HAS_BT && HAS_USBNCM
        // USB モード中の短押し: CDC(Android) → NCM(iOS) → BT → CDC... の3択サイクル
        if (btMode)        { btMode = false; ncmMode = false; }  // BT → CDC(Android)
        else if (ncmMode)  { ncmMode = false; btMode = true;  }  // NCM(iOS) → BT
        else               { ncmMode = true;  btMode = false; }  // CDC(Android) → NCM(iOS)
        for (int i = 0; i < 2; i++) {
          clearLed(); delay(100);
          showUsbLed(); delay(100);
        }
#elif HAS_BT
        // USB モード中の短押し: USB(CDC) ↔ BT 切替
        btMode = !btMode;
        if (btMode) ncmMode = false;
        for (int i = 0; i < 2; i++) {
          clearLed(); delay(100);
          showUsbLed(); delay(100);
        }
#else
        // USB モード中の短押し: NCM(iOS) ↔ CDC(Android) 切替
        ncmMode = !ncmMode;
        for (int i = 0; i < 2; i++) {
          clearLed(); delay(100);
          showUsbLed(); delay(100);
        }
#endif
      } else if (profileCount > 0) {
        selectedProfileIndex = (selectedProfileIndex + 1) % profileCount;
        setLedFromProfileIndex(selectedProfileIndex);
      }
    }
    delay(20);
  }

  EEPROM.write(EEPROM_ADDR_USB_MODE, usbMode ? 1 : 0);
  EEPROM.write(EEPROM_ADDR_NCM_MODE, ncmMode ? 1 : 0);
  EEPROM.write(EEPROM_ADDR_BT_MODE,  btMode  ? 1 : 0);
  EEPROM.commit();
  saveProfileIndexToEEPROM(selectedProfileIndex);
  EEPROM.end();
  activeProfileIndex = selectedProfileIndex;

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_set_intr_type((gpio_num_t)KEY_PIN, GPIO_INTR_ANYEDGE);
  gpio_isr_handler_add((gpio_num_t)KEY_PIN, keyISR, NULL);

  // ---- Bluetooth モード ------------------------------------------------------
#if HAS_BT
  if (btMode) {
    initBluetooth();
    syncCollecting  = true;
    syncSampleIndex = 0;
    return;
  }
#endif

  if (usbMode) {
#if HAS_USBNCM
    if (ncmMode) {
      // ---- USB-NCM モード (iOS) -------------------------------------------
      initNcm();
      serverIP = NCM_HOST_IP;
      Udp.begin(LOCAL_PORT);
      setLedColorFromProfile(&OrangeProfile);  // オレンジ: NCM接続中
      syncCollecting  = true;
      syncSampleIndex = 0;
      requestMillisSync();
      lastPingMillis = millis();
      fixedDelayMs   = 100;
    } else {
      // ---- USB-CDC モード (Android) ----------------------------------------
      syncCollecting  = true;
      syncSampleIndex = 0;
      byte pkt[5] = {0xE0, 0, 0, 0, 0};
      unsigned long t = millis();
      pkt[1]=(t>>24)&0xFF; pkt[2]=(t>>16)&0xFF;
      pkt[3]=(t>>8)&0xFF;  pkt[4]=t&0xFF;
      Serial.write(pkt, 5);
      setLedColorFromProfile(&OrangeProfile);  // オレンジ: CDC待機中
    }
#else
    // フォールバック: USB-CDC シリアルトンネル (NCM非対応ビルド)
    syncCollecting  = true;
    syncSampleIndex = 0;
    byte pkt[5] = {0xE0, 0, 0, 0, 0};
    unsigned long t = millis();
    pkt[1]=(t>>24)&0xFF; pkt[2]=(t>>16)&0xFF;
    pkt[3]=(t>>8)&0xFF;  pkt[4]=t&0xFF;
    Serial.write(pkt, 5);
    setLedColorFromProfile(&OrangeProfile);
#endif
    return;
  }

  // ---- WiFi モード ---------------------------------------------------------
  connectToWiFi();
  getserverIP();
  Udp.begin(LOCAL_PORT);
  syncCollecting  = true;
  syncSampleIndex = 0;
  requestMillisSync();
  lastPingMillis = millis();
}

// ---- loop() -----------------------------------------------------------------
void loop() {
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();
  byte trxCode = 0x01;

  // ---- Bluetooth SPP 処理 ---------------------------------------------------
#if HAS_BT
  if (btMode) {
    static byte btRxBuf[16];
    static int  btRxLen = 0;
    static unsigned long lastBtSyncMs = 0;

    // BLE 受信 (USB-CDC と同じ順序: まず受信してから !timeSynced チェック)
    while (bleAvailable() > 0 && btRxLen < 16)
      btRxBuf[btRxLen++] = (byte)bleReadByte();

    if (btRxLen >= 9 && btRxBuf[0] == 0xE1) {
      uint64_t serverRecvMs = 0;
      for (int i=0;i<8;i++) serverRecvMs = (serverRecvMs<<8)|btRxBuf[1+i];
      long offset = (long)serverRecvMs - (long)nowMillis;
      if (syncCollecting) {
        if (syncSampleIndex < SYNC_SAMPLE_COUNT) syncOffsets[syncSampleIndex++] = offset;
        if (syncSampleIndex >= SYNC_SAMPLE_COUNT) {
          long minOff = syncOffsets[0];
          for (int i=1;i<SYNC_SAMPLE_COUNT;i++) if (syncOffsets[i]<minOff) minOff=syncOffsets[i];
          delayOffsetMs  = minOff;
          timeSynced     = true;
          syncCollecting = false;
          fixedDelayMs   = 100;
          setLedColor(0, 0, 20);
        } else {
          byte pkt[5] = {0xE0,0,0,0,0};
          unsigned long t = millis();
          pkt[1]=(t>>24)&0xFF; pkt[2]=(t>>16)&0xFF;
          pkt[3]=(t>>8)&0xFF;  pkt[4]=t&0xFF;
          bleWriteBytes(pkt, 5);
          lastBtSyncMs = nowMillis;
        }
      } else {
        delayOffsetMs = offset;
        lastBtSyncMs  = nowMillis;
      }
      memmove(btRxBuf, btRxBuf+9, btRxLen-9);
      btRxLen -= 9;
    } else if (btRxLen >= 16) { btRxLen = 0; }

    // 定期 SYNC (未同期・同期後どちらも送信)
    if (bleHasClient() && nowMillis - lastBtSyncMs >= 1500) {
      lastBtSyncMs = nowMillis;
      byte pkt[5] = {0xE0,0,0,0,0};
      unsigned long t = millis();
      pkt[1]=(t>>24)&0xFF; pkt[2]=(t>>16)&0xFF;
      pkt[3]=(t>>8)&0xFF;  pkt[4]=t&0xFF;
      bleWriteBytes(pkt, 5);
    }

    // 赤点滅: BT未接続/SYNC待ち
    if (!timeSynced) {
      if (nowMillis - lastBlinkMillis >= 500) {
        lastBlinkMillis = nowMillis;
        ledState = !ledState;
        ledState ? setLedColorFromProfile(&RedProfile) : clearLed();
      }
      // 短押し で USB-CDC モードへ戻る
      {
        static bool btBtnWasHigh = true;
        bool btnLow = (digitalRead(BUTTON_PIN) == LOW);
        if (btnLow && btBtnWasHigh) {
          delay(20);
          unsigned long ps = millis();
          while (digitalRead(BUTTON_PIN) == LOW && millis()-ps < 2000) delay(10);
          if (millis()-ps < 1000) {
            btMode = false;
            bleEnd();
            EEPROM.begin(EEPROM_SIZE);
            EEPROM.write(EEPROM_ADDR_BT_MODE, 0);
            EEPROM.commit();
            EEPROM.end();
            syncCollecting = true; syncSampleIndex = 0;
            setLedColorFromProfile(&OrangeProfile);  // USB(CDC)モードへ戻る
          }
          btBtnWasHigh = true;
        } else {
          btBtnWasHigh = !btnLow;
        }
      }
      yield(); return;
    }

    // キー送信 (USB-CDC と同じ処理、BLE notify 経由)
    noInterrupts();
    bool btDoSend  = immediateSendRequested;
    bool btSendSt  = keyState;
    unsigned long btIsrUs = lastKeyChangeMicros;
    if (btDoSend) immediateSendRequested = false;
    interrupts();

    if (btDoSend) {
      currentKeyState = btSendSt;
      uint64_t opTimeMs = (btIsrUs/1000) + delayOffsetMs + fixedDelayMs;
      uint8_t cwPkt[10];
      cwPkt[0] = btSendSt ? 0x01 : 0x00; cwPkt[1] = 0x01;
      for (int i=0;i<8;i++) cwPkt[2+i] = (opTimeMs>>(56-i*8))&0xFF;
      int rep = btSendSt ? 3 : 5;
      bool abort = false;
      for (int i=0;i<rep && !abort;i++) {
        bleWriteBytes(cwPkt, 10);
        if (i<rep-1) {
          for (int ms=0;ms<5 && !abort;ms++) {
            delay(1);
            noInterrupts();
            if (immediateSendRequested && keyState!=btSendSt) abort=true;
            interrupts();
          }
        }
      }
      if (btSendSt) lastOnMillis = nowMillis;
      setLedColor(0, 0, btSendSt ? 255 : 20);
    }
    if (currentKeyState && nowMillis - lastOnMillis > MAX_ON_DURATION) {
      currentKeyState = false;
      uint64_t opTimeMs = (unsigned long)nowMillis + delayOffsetMs + fixedDelayMs;
      uint8_t offPkt[10] = {0x00, 0x01};
      for (int i=0;i<8;i++) offPkt[2+i] = (opTimeMs>>(56-i*8))&0xFF;
      for (int i=0;i<5;i++) { bleWriteBytes(offPkt, 10); if (i<4) delay(5); }
      setLedColor(0, 0, 20);
    }

    // BLE 切断検出
    if (!bleHasClient()) {
      timeSynced     = false;
      syncCollecting = true;
      syncSampleIndex = 0;
      lastBtSyncMs   = 0;
    }
    yield(); return;
  }
#endif

  // ---- USB-CDC 処理 (HAS_USBNCM==0 or CDCモード選択時) ----------------------
#if HAS_USBNCM
  if (usbMode && !ncmMode) {
#else
  if (usbMode) {
#endif
    static byte usbRxBuf[16];
    static int  usbRxLen = 0;
    static unsigned long lastUsbSyncMs = 0;

    while (Serial.available() > 0 && usbRxLen < 16)
      usbRxBuf[usbRxLen++] = (byte)Serial.read();

    if (usbRxLen >= 9 && usbRxBuf[0] == 0xE1) {
      uint64_t serverRecvMs = 0;
      for (int i = 0; i < 8; i++) serverRecvMs = (serverRecvMs << 8) | usbRxBuf[1+i];
      long offset = (long)serverRecvMs - (long)nowMillis;
      if (syncCollecting) {
        if (syncSampleIndex < SYNC_SAMPLE_COUNT) syncOffsets[syncSampleIndex++] = offset;
        if (syncSampleIndex >= SYNC_SAMPLE_COUNT) {
          long minOff = syncOffsets[0];
          for (int i = 1; i < SYNC_SAMPLE_COUNT; i++)
            if (syncOffsets[i] < minOff) minOff = syncOffsets[i];
          delayOffsetMs  = minOff;
          timeSynced     = true;
          syncCollecting = false;
          fixedDelayMs   = 100;
          setLedColor(0, 0, 20);
        } else {
          byte pkt[5] = {0xE0, 0, 0, 0, 0};
          unsigned long t = millis();
          pkt[1]=(t>>24)&0xFF; pkt[2]=(t>>16)&0xFF;
          pkt[3]=(t>>8)&0xFF;  pkt[4]=t&0xFF;
          Serial.write(pkt, 5);
          lastUsbSyncMs = nowMillis;
        }
      } else {
        delayOffsetMs = offset;
        lastUsbSyncMs = nowMillis;
      }
      memmove(usbRxBuf, usbRxBuf+9, usbRxLen-9);
      usbRxLen -= 9;
    } else if (usbRxLen >= 16) { usbRxLen = 0; }

    if (nowMillis - lastUsbSyncMs >= 1500) {
      lastUsbSyncMs = nowMillis;
      byte pkt[5] = {0xE0, 0, 0, 0, 0};
      unsigned long t = millis();
      pkt[1]=(t>>24)&0xFF; pkt[2]=(t>>16)&0xFF;
      pkt[3]=(t>>8)&0xFF;  pkt[4]=t&0xFF;
      Serial.write(pkt, 5);
    }

    if (!timeSynced) {
      // シアン点滅: USB-CDC 未接続/SYNC待ち (Android)
      if (nowMillis - lastBlinkMillis >= 500) {
        lastBlinkMillis = nowMillis;
        ledState = !ledState;
        ledState ? setLedColorFromProfile(&cyanProfile) : clearLed();
      }
      // 短押しで Bluetooth モードへ切替
#if HAS_BT
      {
        static bool usbBtnWasHigh = true;
        bool btnLow = (digitalRead(BUTTON_PIN) == LOW);
        if (btnLow && usbBtnWasHigh) {
          delay(20);
          unsigned long ps = millis();
          while (digitalRead(BUTTON_PIN) == LOW && millis()-ps < 2000) delay(10);
          unsigned long dur = millis()-ps;
          if (dur < 1000) {
            // 短押し: BT モードへ
            btMode = true;
            syncCollecting  = true;
            syncSampleIndex = 0;
            lastBlinkMillis = 0;
            EEPROM.begin(EEPROM_SIZE);
            EEPROM.write(EEPROM_ADDR_BT_MODE, 1);
            EEPROM.commit();
            EEPROM.end();
            initBluetooth();
          }
          usbBtnWasHigh = true;
        } else {
          usbBtnWasHigh = !btnLow;
        }
      }
#endif
      yield(); return;
    }

    noInterrupts();
    bool doSend  = immediateSendRequested;
    bool sendSt  = keyState;
    unsigned long isrUs = lastKeyChangeMicros;
    if (doSend) immediateSendRequested = false;
    interrupts();

    if (doSend) {
      currentKeyState = sendSt;
      uint64_t opTimeMs = (isrUs/1000) + delayOffsetMs + fixedDelayMs;
      uint8_t cwPkt[10];
      cwPkt[0] = sendSt ? 0x01 : 0x00; cwPkt[1] = 0x01;
      for (int i = 0; i < 8; i++) cwPkt[2+i] = (opTimeMs>>(56-i*8))&0xFF;
      int rep = sendSt ? 3 : 5;
      bool abort = false;
      for (int i = 0; i < rep && !abort; i++) {
        Serial.write(cwPkt, 10);
        if (i < rep-1) {
          for (int ms=0; ms<5 && !abort; ms++) {
            delay(1);
            noInterrupts();
            if (immediateSendRequested && keyState != sendSt) abort = true;
            interrupts();
          }
        }
      }
      if (sendSt) lastOnMillis = nowMillis;
      setLedColor(0, 0, sendSt ? 255 : 20);
    }
    if (currentKeyState && nowMillis - lastOnMillis > MAX_ON_DURATION) {
      currentKeyState = false;
      uint64_t opTimeMs = (unsigned long)nowMillis + delayOffsetMs + fixedDelayMs;
      uint8_t offPkt[10] = {0x00, 0x01};
      for (int i = 0; i < 8; i++) offPkt[2+i] = (opTimeMs>>(56-i*8))&0xFF;
      for (int i = 0; i < 5; i++) { Serial.write(offPkt, 10); if (i<4) delay(5); }
      setLedColor(0, 0, 20);
    }
    yield();
    return;
  }  // end USB-CDC block

  // ---- 以下: WiFi モード & USB-NCM モード (共通 UDP 処理) -------------------
  // USB-NCM モードでは serverIP = NCM_HOST_IP (iOS/Android 中継アプリ)
  // WiFi モードでは serverIP = Server の IP
  // どちらも WiFiUDP Udp で透過的に動作 (lwIP がルーティング)

  // SYNC 収集中かつ接続あり
  static bool syncRequested = false;
  static unsigned long lastSyncRetryMillis = 0;

  bool isConnected;
#if HAS_USBNCM
  isConnected = (usbMode && ncmMode) ? (bool)ncm : (WiFi.status() == WL_CONNECTED);
#else
  isConnected = (WiFi.status() == WL_CONNECTED);
#endif

  if (!timeSynced && isConnected) {
    if (!syncRequested || millis() - lastSyncRetryMillis >= 2000) {
      requestMillisSync();
      syncRequested   = true;
      syncCollecting  = true;
      syncSampleIndex = 0;
      lastSyncRetryMillis = millis();
    }
  }

  // メカニカルボタン補助 (MBTN_PIN)
  {
    static bool lastMbtnState = HIGH;
    bool mbtnState = digitalRead(MBTN_PIN);
    if (mbtnState != lastMbtnState) {
      lastMbtnState = mbtnState;
      keyState = (mbtnState == LOW);
      if (mbtnState == LOW) setDataColor(255, 0, 0);
      else                  setDataColor(0, 255, 0);
      immediateSendRequested = true;
      lastKeyChangeMicros = micros();
    }
  }

  // 即時送信
  bool doSend = false, sendState = false;
  unsigned long isrMicros = 0;
  noInterrupts();
  if (immediateSendRequested) {
    doSend    = true;
    sendState = keyState;
    isrMicros = lastKeyChangeMicros;
    immediateSendRequested = false;
  }
  interrupts();

  if (doSend && timeSynced) {
    fixedDelayLocked = true;
    currentKeyState  = sendState;
    uint64_t opTimeMs = (isrMicros/1000) + delayOffsetMs + fixedDelayMs;
    int repeat = currentKeyState ? 1 : 3;
    for (int i = 0; i < repeat; i++) {
      sendKeyPacket(currentKeyState, trxCode, opTimeMs);
      taskYIELD();
    }
    lastOnMillis = currentKeyState ? nowMillis : lastOnMillis;
    lastImmediateSendMillis = millis();
  }

  // サンプリング送信
  if (timeSynced && millis() - lastSampleTimeMs >= SAMPLE_INTERVAL_MS) {
    if (immediateSendRequested) return;
    lastSampleTimeMs = millis();
    bool down = (digitalRead(KEY_PIN) == LOW);
    if (sampleIndex == 0) { sampleStartTimeMs = millis(); keyBuf = 0; }
    sampleTimes[sampleIndex] = millis();
    if (down) keyBuf |= (1 << sampleIndex);
    sampleIndex++;
    if (sampleIndex >= SAMPLE_SEND_AMNT) {
      byte pkt[21];
      pkt[0] = trxCode; pkt[1] = keyBuf;
      pkt[2] = (sampleStartTimeMs>>16)&0xFF;
      pkt[3] = (sampleStartTimeMs>>8)&0xFF;
      pkt[4] =  sampleStartTimeMs&0xFF;
      for (int i = 0; i < SAMPLE_SEND_AMNT; i++)
        pkt[5+i] = (byte)constrain(sampleTimes[i]-sampleStartTimeMs, 0, 255);
      for (int i = 0; i < SAMPLE_SEND_DUP_CNT; i++) {
        Udp.beginPacket(serverIP, SERVER_PORT);
        Udp.write(pkt, 5+SAMPLE_SEND_AMNT);
        Udp.endPacket();
        taskYIELD();
      }
      sampleIndex = 0;
    }
  }

  // UDP 受信
  if (isConnected) {
    int size = Udp.parsePacket();
    if (size > 0) {
      byte buf[12];
      int len = Udp.read(buf, size);

      if (size == 9 && buf[0] == 0xE1) {
        uint64_t serverRecvMs = 0;
        for (int i = 0; i < 8; i++) serverRecvMs = (serverRecvMs<<8)|buf[1+i];
        long offset = (long)serverRecvMs - (long)nowMillis;

        if (!usbMode && offset < 3000) { flashRedBeforeRestart(); ESP.restart(); return; }

        if (syncCollecting) {
          if (syncSampleIndex < SYNC_SAMPLE_COUNT) syncOffsets[syncSampleIndex++] = offset;
          if (syncSampleIndex >= SYNC_SAMPLE_COUNT) {
            long minOff = syncOffsets[0];
            for (int i = 1; i < SYNC_SAMPLE_COUNT; i++)
              if (syncOffsets[i] < minOff) minOff = syncOffsets[i];
            delayOffsetMs  = minOff;
            timeSynced     = true;
            syncCollecting = false;
            fixedDelayLocked   = true;
            fixedDelayLockUntil = millis() + 30000;
            blinkInterval  = BLINK_INTERVAL_SYNC;
            rttIndex = 0;
            for (int i = 0; i < RTT_HISTORY_SIZE; i++) rttHistory[i] = 0;
            sampleIndex = 0; keyBuf = 0;
            lastSampleTimeMs    = millis();
            lastSyncRequestMillis = millis();
            lastSyncRetryMillis   = millis();
            setLedColorFromProfile(&RedProfile); delay(100);
            currentKeyState ? setLedColorFromProfile(&GreenProfile) : clearLed();
          } else {
            requestMillisSync();
            lastSyncRetryMillis = millis();
          }
          return;
        }

        delayOffsetMs    = offset;
        timeSynced       = true;
        fixedDelayMs     = usbMode ? 100 : rttToDelayMs(pingRttMs);
        fixedDelayLocked = true;
        fixedDelayLockUntil = millis() + 30000;
        lastPingMillis   = nowMillis;
        lastPingSendMillis = nowMillis;
        blinkInterval    = BLINK_INTERVAL_SYNC;
        rttIndex = 0;
        for (int i = 0; i < RTT_HISTORY_SIZE; i++) rttHistory[i] = 0;
        sampleIndex = 0; keyBuf = 0;
        lastSampleTimeMs     = nowMillis;
        lastSyncRequestMillis = nowMillis;
        lastSyncRetryMillis   = nowMillis;
        setLedColorFromProfile(&RedProfile); delay(100);
        currentKeyState ? setLedColorFromProfile(&GreenProfile) : clearLed();
      }
      else if (size == 1 && buf[0] == 0xFE) {
        pongReceived = true;
        long rtt = nowMillis - lastPingSendMillis;
        rttHistory[rttIndex] = rtt;
        rttIndex = (rttIndex+1) % RTT_HISTORY_SIZE;
        pingRttMs = getAverageRtt();
        if (!fixedDelayLocked && !currentKeyState && (millis()-lastOnMillis > 1000)) {
          fixedDelayMs     = rttToDelayMs(pingRttMs);
          fixedDelayLocked = true;
        }
        lastPingMillis = nowMillis;
      }
    }
  }

  // PING 送信
  if (timeSynced && nowMillis - lastPingSendMillis > PING_INTERVAL) {
    if (!pongReceived && !currentKeyState) lostPingCount++;
    else if (currentKeyState)             lostPingCount = 0;
    pongReceived = false;
    byte ping = 0xFF;
    Udp.beginPacket(serverIP, SERVER_PORT);
    Udp.write(&ping, 1);
    Udp.endPacket();
    lastPingSendMillis = nowMillis;
  }

  // RTT 再計算
  if (timeSynced && nowMillis - lastDelayRecalcMillis >= ReCal_RTT) {
    lastDelayRecalcMillis = nowMillis;
    pingRttMs = getAverageRtt();
    if (!fixedDelayLocked) fixedDelayMs = rttToDelayMs(pingRttMs);
  }

  // タイムアウト検出
  if (timeSynced && nowMillis - lastPingMillis >= dynamicTimeout) {
    timeSynced       = false;
    blinkInterval    = BLINK_INTERVAL_WIFI;
    fixedDelayLocked = false;
  }

  if (timeSynced && !currentKeyState && (millis()-lastOnMillis >= 5000))
    fixedDelayLocked = false;

  // LED 点滅
  if (nowMillis - lastBlinkMillis >= blinkInterval) {
    ledState = !ledState;
    ledState ? setLedColorFromProfile(&GreenProfile) : clearLed();
    lastBlinkMillis = nowMillis;
  }

  // WiFi 切断検出 (WiFi モードのみ)
  if (!usbMode && WiFi.status() != WL_CONNECTED) {
    timeSynced       = false;
    blinkInterval    = BLINK_INTERVAL_WIFI;
    fixedDelayLocked = false;
    static unsigned long lastReconnectMs = 0;
    if (nowMillis - lastReconnectMs >= 30000) {
      lastReconnectMs = nowMillis;
      WiFiProfile &rp = wifiProfiles[activeProfileIndex];
      WiFi.begin(rp.ssid, rp.password);
    }
  }

  // SYNC 再送
  if (!timeSynced && isConnected && millis()-lastSyncRetryMillis >= 5000) {
    syncCollecting  = true;
    syncSampleIndex = 0;
    requestMillisSync();
    lastSyncRetryMillis = millis();
  }
  if (!timeSynced) return;

  // ON 安全対策
  if (currentKeyState && nowMillis - lastOnMillis > MAX_ON_DURATION) {
    currentKeyState = false;
    uint64_t opTimeMs = nowMillis + delayOffsetMs + fixedDelayMs;
    sendKeyPacket(false, trxCode, opTimeMs);
  }

  if (fixedDelayLocked && millis() > fixedDelayLockUntil) fixedDelayLocked = false;

  delay(1);
}

// =============================================================================
// WiFi 接続
// =============================================================================
bool connectToWiFi() {
  WiFiProfile &profile = wifiProfiles[activeProfileIndex];
  if (!profile.isApMode && profile.hostname && strlen(profile.hostname) > 0)
    WiFi.setHostname(profile.hostname);
  if (profile.hasIP)
    WiFi.config(profile.localIP, profile.gateway, profile.subnet);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(profile.ssid, profile.password);
  if (profile.isApMode) esp_wifi_set_max_tx_power(84);
  for (int t = 0; t < 60; t++) {
    if (WiFi.status() == WL_CONNECTED) {
      serverIP = profile.serverIP;
      dynamicTimeout = DEFAULT_TIMEOUT;
      return true;
    }
    delay(500);
  }
  return false;
}

// =============================================================================
// SYNC 要求
// =============================================================================
void requestMillisSync() {
  lastSyncRequestMillis = millis();
  lastPingSendMillis    = lastSyncRequestMillis;
  byte pkt[5] = {
    0xE0,
    (byte)((lastSyncRequestMillis>>24)&0xFF),
    (byte)((lastSyncRequestMillis>>16)&0xFF),
    (byte)((lastSyncRequestMillis>>8)&0xFF),
    (byte)( lastSyncRequestMillis&0xFF)
  };
  Udp.beginPacket(serverIP, SERVER_PORT);
  Udp.write(pkt, 5);
  Udp.endPacket();
}

// =============================================================================
// キーパケット送信
// =============================================================================
void sendKeyPacket(bool isOn, byte trxSel, uint64_t opTimeMs) {
  byte pkt[10];
  pkt[0] = isOn ? 0x01 : 0x00;
  pkt[1] = trxSel;
  for (int i = 0; i < 8; i++) pkt[2+i] = (opTimeMs>>(56-i*8))&0xFF;
  for (int i = 0; i < keyRepeatCount; i++) {
    if (!Udp.beginPacket(serverIP, SERVER_PORT)) return;
    Udp.write(pkt, 10);
    Udp.endPacket();
    taskYIELD();
  }
  isOn ? setLedColorFromProfile(&RedProfile) : clearLed();
}

void flashRedBeforeRestart() {
  for (int i = 0; i < 3; i++) {
    setLedColorFromProfile(&RedProfile); delay(100);
    clearLed(); delay(100);
  }
}

// =============================================================================
// mDNS サーバー IP 解決
// =============================================================================
void getserverIP() {
  WiFiProfile &profile = wifiProfiles[activeProfileIndex];
  if (!profile.hasIP && profile.serverhostname && strlen(profile.serverhostname) > 0) {
    if (!MDNS.begin(profile.hostname)) return;
    String hostname = profile.serverhostname;
    delay(1000);
    if (!hostname.endsWith(".local")) hostname += ".local";
    String queryName = hostname; queryName.replace(".local", "");
    IPAddress resolved;
    for (int i = 0; i < HOSTNAME_RESTIME; i++) {
      resolved = MDNS.queryHost(queryName);
      if (resolved != INADDR_NONE) break;
      delay(1000);
    }
    if (resolved != INADDR_NONE) serverIP = resolved;
  } else {
    serverIP = profile.serverIP;
  }
}

// =============================================================================
// LED
// =============================================================================
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r,g,b)); pixels.show();
}
void setDataColor(uint8_t r, uint8_t g, uint8_t b) {
  pixelData.setPixelColor(0, pixelData.Color(r,g,b)); pixelData.show();
}
void setLedColorFromProfile(ColorProfile *color) {
  if (!color) return; setLedColor(color->r, color->g, color->b);
}
void flashLedProfile(ColorProfile *color, int durationMs, int repeat) {
  if (!color) return;
  for (int i = 0; i < repeat; i++) {
    setLedColorFromProfile(color); delay(durationMs);
    setLedColor(0,0,0);            delay(durationMs);
  }
}
void flashOnce(ColorProfile *color, int durationMs) {
  if (!color) return;
  setLedColorFromProfile(color); delay(durationMs); setLedColor(0,0,0);
}
void initLed() {
  pixels.begin(); pixels.setBrightness(50);
  pixelData.begin(); pixelData.setBrightness(50);
}
void setLedFromProfileIndex(int index) {
  if (index >= 0 && index < profileCount)
    setLedColorFromProfile(wifiProfiles[index].color);
}
void clearLed() {
  pixels.clear(); pixels.show();
  pixelData.clear(); pixelData.show();
}

// =============================================================================
// EEPROM
// =============================================================================
void saveProfileIndexToEEPROM(int index) {
  EEPROM.write(EEPROM_ADDR_PROFILE_INDEX, index); EEPROM.commit();
}
int loadProfileIndexFromEEPROM() {
  byte val = EEPROM.read(EEPROM_ADDR_PROFILE_INDEX);
  return (val < MAX_WIFI_PROFILES) ? val : 0;
}

void saveWiFiProfileToEEPROM(const WiFiProfile &src, int index) {
  if (!src.ssid || strlen(src.ssid)==0) return;
  WiFiProfileEEPROM dst; memset(&dst, 0, sizeof(dst));
  dst.magic = PROFILE_MAGIC;
  strncpy(dst.ssid,     src.ssid,     MAX_SSID_LEN);
  strncpy(dst.password, src.password, MAX_PASS_LEN);
  if (src.hostname)       strncpy(dst.hostname,       src.hostname,       MAX_HOSTNAME_LEN);
  if (src.serverhostname) strncpy(dst.serverhostname, src.serverhostname, MAX_HOSTNAME_LEN);
  for (int i=0;i<4;i++) {
    dst.localIP[i]=src.localIP[i]; dst.gateway[i]=src.gateway[i];
    dst.subnet[i]=src.subnet[i];   dst.serverIP[i]=src.serverIP[i];
  }
  dst.isApMode = src.isApMode; dst.hasIP = src.hasIP;
  if      (src.color==&cyanProfile)    dst.colorId=0;
  else if (src.color==&MagentaProfile) dst.colorId=1;
  else if (src.color==&yellowProfile)  dst.colorId=2;
  else if (src.color==&RedProfile)     dst.colorId=3;
  else if (src.color==&BlueProfile)    dst.colorId=4;
  else if (src.color==&GreenProfile)   dst.colorId=5;
  else if (src.color==&WhiteProfile)   dst.colorId=6;
  else                                 dst.colorId=0;
  EEPROM.put(EEPROM_ADDR_PROFILE_BASE + index*sizeof(WiFiProfileEEPROM), dst);
  EEPROM.commit();
}

bool loadWiFiProfileFromEEPROM(WiFiProfile &dst, int index) {
  WiFiProfileEEPROM raw;
  EEPROM.get(EEPROM_ADDR_PROFILE_BASE + index*sizeof(WiFiProfileEEPROM), raw);
  if (raw.magic != PROFILE_MAGIC) return false;
  freeWiFiProfileStrings(dst);
  dst.ssid     = strdup(raw.ssid);
  dst.password = strdup(raw.password);
  dst.localIP  = IPAddress(raw.localIP);
  dst.gateway  = IPAddress(raw.gateway);
  dst.subnet   = IPAddress(raw.subnet);
  dst.serverIP = IPAddress(raw.serverIP);
  dst.isApMode = raw.isApMode; dst.hasIP = raw.hasIP;
  dst.hostname       = strdup(raw.hostname);
  dst.serverhostname = strdup(raw.serverhostname);
  const ColorProfile *tbl[] = {&cyanProfile,&MagentaProfile,&yellowProfile,
                                &RedProfile, &BlueProfile,   &GreenProfile, &WhiteProfile};
  dst.color = (raw.colorId < 7) ? const_cast<ColorProfile*>(tbl[raw.colorId]) : &RedProfile;
  return true;
}

void clearEEPROM() {
  for (int i=0;i<EEPROM_SIZE;i++) EEPROM.write(i,0xFF);
  EEPROM.commit(); delay(100);
}

bool restoreProfilesFromEEPROM() {
  profileCount = 0;
  for (int i=0;i<MAX_WIFI_PROFILES;i++)
    if (loadWiFiProfileFromEEPROM(wifiProfiles[i],i)) profileCount++;
  return profileCount > 0;
}

void saveSystemSettingsToEEPROM(const SystemSettingsEEPROM &s) {
  EEPROM.put(EEPROM_ADDR_SETTINGS, s); EEPROM.commit();
}
bool loadSystemSettingsFromEEPROM(SystemSettingsEEPROM &s) {
  EEPROM.get(EEPROM_ADDR_SETTINGS, s);
  return s.magic == 0xA5;
}
