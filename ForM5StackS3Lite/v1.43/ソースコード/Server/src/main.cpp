/*=============================================================================*
 * M5Stack ATOM S3 Lite サーバー用ファームウェア  Ver1.42
 * USB-NCMモード: Raspberry Pi と USB-C 直結 (192.168.7.x ネットワーク)
 *   - ESP32-S3 が USB-NCM デバイスとして Pi に見える
 *   - Pi の cw_bridge_ncm.py が UDP 経由でパケットを中継
 *   - WiFi 不要, Pi の USB ポートから電源供給も可能
 * WiFi APモード: クライアントが直接接続 (従来と同じ)
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

// ---- USB-NCM ----------------------------------------------------------------
#include "USB.h"
#if __has_include("USBNCM.h")
  #include "USBNCM.h"
  static USBNCM ncm;
  #define HAS_USBNCM 1
#else
  #define HAS_USBNCM 0
#endif

static const IPAddress NCM_LOCAL_IP(192, 168, 7, 1);   // ATOM S3 Lite (サーバー)
static const IPAddress NCM_HOST_IP (192, 168, 7, 2);   // Pi (ホスト, DHCP割当)
static const IPAddress NCM_SUBNET  (255, 255, 255, 252);

// ---- EEPROM -----------------------------------------------------------------
#define EEPROM_ADDR_PROFILE_INDEX  0
#define EEPROM_ADDR_USB_MODE       1
#define EEPROM_ADDR_PROFILE_BASE  16
#define MAX_WIFI_PROFILES          5
#define EEPROM_SIZE (sizeof(WiFiProfileEEPROM) * MAX_WIFI_PROFILES + 16)

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
  bool    hasIP;
  bool    isApMode;
  uint8_t colorId;
  char    hostname[MAX_HOSTNAME_LEN];
};

// ---- ピン定義 ---------------------------------------------------------------
#ifdef M5ATOMLite
// ATOM Lite (ESP32)
const int keyPinTx1      = 22;
const int keyPinTx2      = 22;
const int LED_PIN        = 27;
const int BUTTON_PIN     = 39;
#elif M5ATOMS3Lite
// ATOM S3 Lite (ESP32-S3)
const int keyPinTx1      = 5;
const int keyPinTx2      = 5;
const int LED_PIN        = 35;
const int BUTTON_PIN     = 41;
#endif
const int SD_SPI_CS_PIN  = -1;   // SD カードなし

// ---- GPIO マクロ ------------------------------------------------------------
// keyPin は ATOM Lite(22)/ATOM S3 Lite(5) いずれも <32 なので w1ts/w1tc が使用可能
#define GPIO_SET(pin) (GPIO.out_w1ts = (1UL << (pin)))
#define GPIO_CLR(pin) (GPIO.out_w1tc = (1UL << (pin)))

// ---- 色設定 ------------------------------------------------------------------
struct ColorProfile { const char *name; uint8_t r, g, b; };
static ColorProfile MagentaProfile = {"Magenta", 255,   0, 255};
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
  IPAddress   localIP, gateway, subnet;
  bool        hasIP;
  bool        isApMode;
  ColorProfile *color;
  const char *hostname;
};

static WiFiProfile wifiProfiles[MAX_WIFI_PROFILES];
static int profileCount = 0;

static WiFiProfile fallbackProfile = {
  "RemoteKeyer-AP", "password123",
  IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0),
  false, true, &cyanProfile, "RKSERVER01"
};
static const bool fallbackUsbMode = false;
static int activeProfileIndex = -1;

// ---- UDP / イベントキュー ---------------------------------------------------
#define LOCAL_PORT        8888
#define SEL_TRX1          0x01
#define SEL_TRX2          0x02
#define MAX_EVENTS         256
#define SAME_MAX             4
#define MAX_ON_DURATION   3000
#define MAX_FUTURE_WAIT   2000
static const unsigned long MIN_EVENT_INTERVAL = 40;

struct CWEvent { bool isOn; byte trxSel; uint64_t opTimeMs; };
static CWEvent   eventQueue[MAX_EVENTS];
static uint16_t  queueHead = 0, queueTail = 0;

static WiFiUDP Udp;
static Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);

static unsigned long lastPacketTime  = 0;
static unsigned long lastPingTime    = 0;
static unsigned long lastOnTimeTx1   = 0;
static unsigned long lastOnTimeTx2   = 0;
static unsigned long lastBlinkTime   = 0;
static bool          ledState        = false;
static bool          lastStateTx1    = false;
static bool          lastStateTx2    = false;
static byte          lastTrxSel      = 0x00;
static byte          lastSeqNum      = 0xFF;
static bool          isReady         = false;
static bool          serverAlive     = true;
static unsigned long dynamicTimeout  = 12000;
static bool          usbMode         = false;
static int           selectedProfileIndex = 0;
static unsigned long profileSelectStart   = 0;
static const unsigned long PROFILE_SELECT_WINDOW = 10000;

// ---- プロトタイプ ------------------------------------------------------------
bool connectServerProfile();
void initLed();
void setLedFromProfileIndex(int index);
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
void setLedColorFromProfile(ColorProfile *color);
void flashLedProfile(ColorProfile *color, int durationMs, int repeat);
void saveProfileIndexToEEPROM(int index);
int  loadProfileIndexFromEEPROM();
void saveWiFiProfileToEEPROM(const WiFiProfile &src, int index);
bool loadWiFiProfileFromEEPROM(WiFiProfile &dst, int index);
void clearEEPROM();
bool restoreProfilesFromEEPROM();
void enqueueEvent(const CWEvent &ev);

// ---- イベントキュー挿入 (時刻順ソート + 重複排除) ---------------------------
void enqueueEvent(const CWEvent &ev) {
  uint16_t nextTail = (queueTail+1) % MAX_EVENTS;
  if (nextTail == queueHead) return;  // キューフル

  // 重複排除
  int sameCount = 0;
  for (int i = 1; i <= SAME_MAX; ++i) {
    if ((queueTail + MAX_EVENTS - i) % MAX_EVENTS == queueHead) break;
    CWEvent &prev = eventQueue[(queueTail + MAX_EVENTS - i) % MAX_EVENTS];
    if (ev.isOn==prev.isOn && ev.trxSel==prev.trxSel &&
        abs((long)(ev.opTimeMs - prev.opTimeMs)) < (long)(MIN_EVENT_INTERVAL*(i+1)))
      sameCount++;
    else break;
  }
  if (sameCount >= SAME_MAX - 1) return;

  // 時刻順挿入
  uint16_t insertPos = queueTail;
  for (uint16_t i=queueHead; i!=queueTail; i=(i+1)%MAX_EVENTS) {
    if (ev.opTimeMs < eventQueue[i].opTimeMs) { insertPos=i; break; }
  }
  if (insertPos != queueTail) {
    for (uint16_t i=queueTail; i!=insertPos; i=(i==0?MAX_EVENTS-1:i-1)) {
      uint16_t prev = (i==0?MAX_EVENTS-1:i-1);
      eventQueue[i] = eventQueue[prev];
    }
  }
  eventQueue[insertPos] = ev;
  queueTail = nextTail;
}

// ---- USB-NCM 初期化 ---------------------------------------------------------
bool initNcm() {
#if HAS_USBNCM
  ncm.setLocalIP(NCM_LOCAL_IP);
  ncm.setSubnetMask(NCM_SUBNET);
  ncm.begin();
  USB.begin();
  unsigned long t = millis();
  while (!ncm && millis()-t < 15000) {
    setLedColor(255, 0, 0); delay(200); setLedColor(0,0,0); delay(200);
  }
  return (bool)ncm;
#else
  return false;
#endif
}

// ---- setup() ----------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  auto cfg = M5.config();
  cfg.external_spk = false; cfg.internal_mic = false;
  cfg.output_power = false; cfg.clear_display = true;
  M5.begin(cfg);

  // SD カードなし → EEPROMから復元
  if (!restoreProfilesFromEEPROM()) {
    wifiProfiles[0]      = fallbackProfile;
    profileCount         = 1;
    selectedProfileIndex = 0;
    if (fallbackUsbMode) { EEPROM.write(EEPROM_ADDR_USB_MODE,1); EEPROM.commit(); }
  }

  selectedProfileIndex = loadProfileIndexFromEEPROM();
  if (selectedProfileIndex >= profileCount) selectedProfileIndex = 0;
  activeProfileIndex = selectedProfileIndex;
  usbMode = (EEPROM.read(EEPROM_ADDR_USB_MODE) == 1);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  initLed();
  profileSelectStart = millis();

  if (usbMode) setLedColor(255, 0, 0);
  else         setLedFromProfileIndex(selectedProfileIndex);

  // 10 秒間: 短押し=プロファイル切替, 長押し 1 秒=USB-NCM/WiFi 切替
  while (millis() - profileSelectStart < PROFILE_SELECT_WINDOW) {
    M5.update();
    if (M5.BtnA.isPressed()) {
      unsigned long pressStart = millis();
      while (M5.BtnA.isPressed()) { M5.update(); delay(10); }
      unsigned long dur = millis() - pressStart;
      if (dur >= 1000) {
        usbMode = !usbMode;
        for (int i=0;i<3;i++) {
          setLedColor(0,0,0); delay(150);
          if (usbMode) setLedColor(255,0,0);
          else         setLedFromProfileIndex(selectedProfileIndex);
          delay(150);
        }
      } else {
        selectedProfileIndex = (selectedProfileIndex+1) % profileCount;
        if (usbMode) setLedColor(255,0,0);
        else         setLedFromProfileIndex(selectedProfileIndex);
      }
    }
    delay(20);
  }

  activeProfileIndex = selectedProfileIndex;
  setLedFromProfileIndex(activeProfileIndex);
  esp_rom_gpio_pad_select_gpio((gpio_num_t)BUTTON_PIN);
  gpio_set_direction((gpio_num_t)BUTTON_PIN, GPIO_MODE_DISABLE);

  EEPROM.write(EEPROM_ADDR_USB_MODE, usbMode ? 1 : 0);
  EEPROM.commit();
  saveProfileIndexToEEPROM(selectedProfileIndex);
  EEPROM.end();
  delay(100);

  pinMode(keyPinTx1, OUTPUT);
  GPIO_CLR(keyPinTx1);

  if (usbMode) {
    // ---- USB-NCM モード ---------------------------------------------------
#if HAS_USBNCM
    initNcm();
    Udp.begin(LOCAL_PORT);
    setLedColor(0, 0, 255);  // 青: NCM 接続中
#else
    // フォールバック: USB-CDC シリアルトンネル
    setLedColor(0, 0, 255);
#endif
    lastPacketTime = millis();
    lastPingTime   = millis();
    return;
  }

  // ---- WiFi モード ---------------------------------------------------------
  bool wifiConnected = connectServerProfile();
  bool apMode = wifiProfiles[activeProfileIndex].isApMode;
  if (wifiConnected || apMode) {
    MDNS.begin(wifiProfiles[activeProfileIndex].hostname);
    Udp.begin(LOCAL_PORT);
    lastPacketTime = millis();
    lastPingTime   = millis();
    if (wifiConnected && !apMode) {
      int rssi = WiFi.RSSI();
      dynamicTimeout = constrain(map(rssi,-90,-30,12000,8000), 8000, 12000);
    }
  }
}

// ---- loop() -----------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // ---- USB-CDC フォールバック (HAS_USBNCM==0 かつ usbMode) -----------------
#if !HAS_USBNCM
  if (usbMode) {
    static byte serialBuf[16];
    static int  serialBufLen    = 0;
    static unsigned long serialBufStartMs = 0;

    while (Serial.available()>0 && serialBufLen<16) {
      if (serialBufLen==0) serialBufStartMs = now;
      serialBuf[serialBufLen++] = (byte)Serial.read();
    }
    if (serialBufLen>0 && now-serialBufStartMs>200) serialBufLen=0;

    if (serialBufLen>=5 && serialBuf[0]==0xE0) {
      byte reply[9] = {0xE1};
      for (int i=0;i<4;i++) reply[1+i]=serialBuf[1+i];
      unsigned long sms = now;
      for (int i=0;i<4;i++) reply[5+i]=(byte)((sms>>(24-i*8))&0xFF);
      Serial.write(reply,9);
      isReady=true; serverAlive=true;
      memmove(serialBuf,serialBuf+5,serialBufLen-5); serialBufLen-=5;
      lastPacketTime=now; lastPingTime=now;
    }
    else if (serialBufLen>=1 && serialBuf[0]==0xFF) {
      Serial.write((byte)0xFE);
      memmove(serialBuf,serialBuf+1,serialBufLen-1); serialBufLen-=1;
      lastPacketTime=now; lastPingTime=now; serverAlive=true;
    }
    else if (serialBufLen>=10 && (serialBuf[0]==0x00||serialBuf[0]==0x01)) {
      CWEvent ev;
      ev.isOn   = serialBuf[0]&0x01;
      ev.trxSel = serialBuf[1];
      ev.opTimeMs = 0;
      for (int i=0;i<8;i++) ev.opTimeMs=(ev.opTimeMs<<8)|serialBuf[2+i];
      if (ev.opTimeMs==0) ev.opTimeMs=(uint64_t)now+100;
      enqueueEvent(ev);
      memmove(serialBuf,serialBuf+10,serialBufLen-10); serialBufLen-=10;
      lastPacketTime=now; lastPingTime=now;
      isReady=true; serverAlive=true;
    }

    while (queueHead!=queueTail && now>=(unsigned long)eventQueue[queueHead].opTimeMs) {
      CWEvent &ev = eventQueue[queueHead];
      if (ev.trxSel!=lastTrxSel) {
        if (lastStateTx1) GPIO_CLR(keyPinTx1);
        if (lastStateTx2) GPIO_CLR(keyPinTx2);
        lastStateTx1=false; lastStateTx2=false;
      }
      if (ev.trxSel==SEL_TRX1) {
        if (ev.isOn!=lastStateTx1) {
          ev.isOn ? GPIO_SET(keyPinTx1) : GPIO_CLR(keyPinTx1);
          lastStateTx1=ev.isOn;
        }
        // PTTのようにONを維持したまま繰り返し送られてくる場合も安全タイマーを延長する
        if (ev.isOn) lastOnTimeTx1=now;
      }
      if (ev.trxSel==SEL_TRX2) {
        if (ev.isOn!=lastStateTx2) {
          ev.isOn ? GPIO_SET(keyPinTx2) : GPIO_CLR(keyPinTx2);
          lastStateTx2=ev.isOn;
        }
        if (ev.isOn) lastOnTimeTx2=now;
      }
      lastTrxSel=ev.trxSel;
      queueHead=(queueHead+1)%MAX_EVENTS;
    }
    if (lastStateTx1 && now-lastOnTimeTx1>MAX_ON_DURATION) { GPIO_CLR(keyPinTx1); lastStateTx1=false; }
    if (lastStateTx2 && now-lastOnTimeTx2>MAX_ON_DURATION) { GPIO_CLR(keyPinTx2); lastStateTx2=false; }
    if (now-lastPacketTime>10000) {
      if (lastStateTx1){GPIO_CLR(keyPinTx1); lastStateTx1=false;}
      if (lastStateTx2){GPIO_CLR(keyPinTx2); lastStateTx2=false;}
      isReady=false; serverAlive=false;
    }
    setLedColor(0,0,255);
    if (queueHead==queueTail) delay(1);
    return;
  }
#endif  // !HAS_USBNCM

  // ---- 以下: WiFi モード & USB-NCM モード (共通 UDP 処理) -------------------
  // USB-NCM モードでは Udp が NCM インタフェースにバインド (lwIP 自動ルーティング)
  // WiFi モードと完全に同一のパケット処理ロジック

  int packetSize = Udp.parsePacket();

  // SYNC リクエスト (5 バイト)
  if (packetSize == 5) {
    byte buf[5]; Udp.read(buf,5);
    if (buf[0]==0xE0) {
      byte reply[9] = {0xE1};
      for (int i=0;i<4;i++) reply[1+i]=buf[1+i];
      for (int i=0;i<4;i++) reply[5+i]=(now>>(24-i*8))&0xFF;
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(reply,9); Udp.endPacket();
      isReady=true; serverAlive=true; lastPacketTime=now;
    }
  }

  // イベントキュー処理
  while (queueHead!=queueTail && now>=(unsigned long)eventQueue[queueHead].opTimeMs) {
    CWEvent &ev = eventQueue[queueHead];
    if (ev.trxSel!=lastTrxSel) {
      if (lastStateTx1) GPIO_CLR(keyPinTx1);
      if (lastStateTx2) GPIO_CLR(keyPinTx2);
      lastStateTx1=false; lastStateTx2=false;
    }
    if (ev.trxSel==SEL_TRX1) {
      if (ev.isOn!=lastStateTx1) {
        ev.isOn ? GPIO_SET(keyPinTx1) : GPIO_CLR(keyPinTx1);
        lastStateTx1=ev.isOn;
      }
      // PTTのようにONを維持したまま繰り返し送られてくる場合も安全タイマーを延長する
      if (ev.isOn) lastOnTimeTx1=now;
    }
    if (ev.trxSel==SEL_TRX2) {
      if (ev.isOn!=lastStateTx2) {
        ev.isOn ? GPIO_SET(keyPinTx2) : GPIO_CLR(keyPinTx2);
        lastStateTx2=ev.isOn;
      }
      if (ev.isOn) lastOnTimeTx2=now;
    }
    lastTrxSel=ev.trxSel;
    queueHead=(queueHead+1)%MAX_EVENTS;
  }

  // キーパケット (10 バイト)
  if (packetSize == 10) {
    byte buf[10]; Udp.read(buf,10);
    CWEvent ev;
    ev.isOn   = buf[0]&0x01;
    ev.trxSel = buf[1];
    ev.opTimeMs = 0;
    for (int i=0;i<8;i++) ev.opTimeMs=(ev.opTimeMs<<8)|buf[2+i];
    enqueueEvent(ev);
    lastPacketTime=now; lastPingTime=now; serverAlive=true;
  }
  // サンプルパケット (13 バイト)
  else if (packetSize == 13) {
    byte buf[13]; Udp.read(buf,13);
    byte trxSel = buf[0]; byte keyBits = buf[1];
    unsigned long baseTime = ((unsigned long)buf[2]<<16)|((unsigned long)buf[3]<<8)|buf[4];
    for (int i=0;i<8;i++) {
      bool isOn = (keyBits>>(7-i))&0x01;
      unsigned long opTimeMs = baseTime + buf[5+i];
      if (millis() > opTimeMs+20) continue;
      CWEvent ev = {isOn, trxSel, opTimeMs};
      enqueueEvent(ev);
    }
    lastPacketTime=millis(); lastPingTime=millis(); serverAlive=true;
  }
  // PING (1 バイト)
  else if (packetSize == 1) {
    byte ping; Udp.read(&ping,1);
    if (ping==0xFF) {
      byte reply=0xFE;
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(&reply,1); Udp.endPacket();
      lastPingTime=now; lastPacketTime=now; serverAlive=true;
    }
  }

  // ON 安全対策
  if (lastStateTx1 && now-lastOnTimeTx1>MAX_ON_DURATION) { GPIO_CLR(keyPinTx1); lastStateTx1=false; }
  if (lastStateTx2 && now-lastOnTimeTx2>MAX_ON_DURATION) { GPIO_CLR(keyPinTx2); lastStateTx2=false; }

  // 通信タイムアウト
  if (now-lastPacketTime >= dynamicTimeout) {
    if (lastStateTx1) GPIO_CLR(keyPinTx1);
    if (lastStateTx2) GPIO_CLR(keyPinTx2);
    lastStateTx1=false; lastStateTx2=false;
    isReady=false; serverAlive=false;
  }

  // LED 状態表示
  if (isReady && serverAlive) {
    if (now-lastBlinkTime >= 500) {
      ledState = !ledState;
      ledState ? setLedColorFromProfile(&BlueProfile) : setLedColorFromProfile(&RedProfile);
      lastBlinkTime=now;
    }
  } else if (!serverAlive) {
    setLedColorFromProfile(&RedProfile);
  } else {
    setLedColorFromProfile(&GreenProfile);
  }

  if (queueHead==queueTail) delay(1);
}

// =============================================================================
// WiFi AP / STA 接続
// =============================================================================
bool connectServerProfile() {
  WiFiProfile &profile = wifiProfiles[activeProfileIndex];
  if (!profile.isApMode && profile.hostname && strlen(profile.hostname)>0)
    WiFi.setHostname(profile.hostname);
  if (profile.isApMode) {
    WiFi.disconnect(true); WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(profile.localIP, profile.gateway, profile.subnet);
    WiFi.softAP(profile.ssid, profile.password);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    esp_wifi_set_ps(WIFI_PS_NONE);
    unsigned long startWait = millis();
    while (millis()-startWait < 30000) {
      if (WiFi.softAPgetStationNum()>0) {
        setLedColor(profile.color->r, profile.color->g, profile.color->b);
        return true;
      }
      delay(500);
    }
    return true;  // AP 起動成功 (クライアント未接続でも続行)
  } else {
    WiFi.mode(WIFI_STA);
    if (profile.hasIP) WiFi.config(profile.localIP, profile.gateway, profile.subnet);
    WiFi.begin(profile.ssid, profile.password);
    for (int t=0;t<60;t++) {
      if (WiFi.status()==WL_CONNECTED) {
        esp_wifi_set_ps(WIFI_PS_NONE);
        setLedColor(profile.color->r, profile.color->g, profile.color->b);
        return true;
      }
      delay(500);
    }
    return false;
  }
}

// =============================================================================
// LED
// =============================================================================
void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(0, pixels.Color(r,g,b)); pixels.show();
}
void setLedColorFromProfile(ColorProfile *color) {
  if (!color) return; setLedColor(color->r, color->g, color->b);
}
void flashLedProfile(ColorProfile *color, int durationMs, int repeat) {
  if (!color) return;
  for (int i=0;i<repeat;i++) {
    setLedColorFromProfile(color); delay(durationMs);
    setLedColor(0,0,0);            delay(durationMs);
  }
}
void initLed() { pixels.begin(); pixels.setBrightness(50); }
void setLedFromProfileIndex(int index) {
  if (index>=0 && index<profileCount)
    setLedColorFromProfile(wifiProfiles[index].color);
}

// =============================================================================
// EEPROM
// =============================================================================
void saveProfileIndexToEEPROM(int index) {
  EEPROM.write(EEPROM_ADDR_PROFILE_INDEX, index); EEPROM.commit();
}
int loadProfileIndexFromEEPROM() {
  byte val = EEPROM.read(EEPROM_ADDR_PROFILE_INDEX);
  return (val < profileCount) ? val : 0;
}

void saveWiFiProfileToEEPROM(const WiFiProfile &src, int index) {
  if (!src.ssid || strlen(src.ssid)==0) return;
  WiFiProfileEEPROM dst; memset(&dst,0,sizeof(dst));
  dst.magic=PROFILE_MAGIC;
  strncpy(dst.ssid,     src.ssid,     MAX_SSID_LEN);
  strncpy(dst.password, src.password, MAX_PASS_LEN);
  strncpy(dst.hostname, src.hostname, MAX_HOSTNAME_LEN);
  for (int i=0;i<4;i++) {
    dst.localIP[i]=src.localIP[i]; dst.gateway[i]=src.gateway[i]; dst.subnet[i]=src.subnet[i];
  }
  dst.hasIP=src.hasIP; dst.isApMode=src.isApMode;
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
  if (raw.magic!=PROFILE_MAGIC) return false;
  dst.ssid     = strdup(raw.ssid);
  dst.password = strdup(raw.password);
  dst.localIP  = IPAddress(raw.localIP);
  dst.gateway  = IPAddress(raw.gateway);
  dst.subnet   = IPAddress(raw.subnet);
  dst.hasIP    = raw.hasIP;
  dst.isApMode = raw.isApMode;
  dst.hostname = strdup(raw.hostname);
  const ColorProfile *tbl[] = {&cyanProfile,&MagentaProfile,&yellowProfile,
                                &RedProfile, &BlueProfile,   &GreenProfile, &WhiteProfile};
  dst.color = (raw.colorId<7) ? const_cast<ColorProfile*>(tbl[raw.colorId]) : &RedProfile;
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
  return profileCount>0;
}
