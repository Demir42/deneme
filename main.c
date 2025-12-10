/*
 * ESP32 WROVER + PSRAM Yakıt İstasyonu Terminali
 * - TFT + Dokunmatik (TFT_eSPI + XPT2046)
 * - PSRAM ile tam ekran sprite (flicker azaltma)
 * - NVS konfig
 * - Klavye (DEL, ABC/abc/123)
 * - WiFi ayarlari (tarama + sifre, baglanti testi)
 * - Telefon / API (telefon basinda otomatik '+')
 * - RFID (MFRC522, ayri SPI pinleri):
 *    - Yonetici kart kaydetme
 *    - Sofor kart + plaka eslestirme
 * - Factory Reset (NVS sil + reset)
 * - RS485 / Modbus + Normal Calisma Modu (Idle / Dolum / OzEt)
 */

#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Preferences.h>
#include <WiFi.h>
#include <MFRC522.h>
#include <HardwareSerial.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <esp32-hal-psram.h>
#include "nvs_flash.h"

// -----------------------------------------------------------------------------
// Sabitler / Donanım Pinleri
// -----------------------------------------------------------------------------
#define TOUCH_CS    33
#define TOUCH_IRQ   14

#define LCD_BL_PIN  32   // TFT backlight

#define TS_SWAP_XY  false
#define TS_INVERT_X true
#define TS_INVERT_Y true

// Üst bar yüksekliği
#define TOP_BAR_H   28

// TFT SPI pinleri (TFT_eSPI User_Setup ile uyumlu)
#define TFT_SCLK 18
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_CS   12

// RFID pinleri (ayri SPI pinleri)
#define RFID_SCK   25
#define RFID_MISO  27
#define RFID_MOSI  26
#define RFID_SS    15
#define RFID_RST   4

// RS485 pinleri
#define RS485_TX_PIN   22
#define RS485_RX_PIN   21
#define RS485_REDE_PIN 13

// Klavye sabitleri
#define KB_BOX_H   26
#define KB_BOX_Y   (TOP_BAR_H + 40)
#define KB_BOX_Y_BACKBTN (TOP_BAR_H + 4)
#define KB_TOP_Y   (KB_BOX_Y + KB_BOX_H + 8)

// Alt bar yüksekliği
#define BOTTOM_BAR_H 48

// WiFi liste alanı üstü
#define WIFI_LIST_TOP  (TOP_BAR_H + 16)

// Tüm yazılar için ana font
#define FONT_MAIN 1

// -----------------------------------------------------------------------------
// Ekran / Dokunmatik / RFID Nesneleri
// -----------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);

MFRC522 mfrc522(RFID_SS, RFID_RST);

// RS485 UART
HardwareSerial RS485Serial(2);

// -----------------------------------------------------------------------------
// SPI sahipligi: TFT/Dokunmatik vs RFID
// -----------------------------------------------------------------------------
enum SpiOwner {
  SPI_OWNER_TFT,
  SPI_OWNER_RFID
};

SpiOwner g_spiOwner = SPI_OWNER_TFT;
bool g_rfidInitialized = false;

// SPI'yi TFT + dokunmatik tarafına ayarla
void spiUseTFT() {
  if (g_spiOwner == SPI_OWNER_TFT) return;
  SPI.end();
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  g_spiOwner = SPI_OWNER_TFT;
}

// SPI'yi RFID pinlerine ayarla
void spiUseRFID() {
  if (g_spiOwner == SPI_OWNER_RFID) return;
  SPI.end();
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  g_spiOwner = SPI_OWNER_RFID;

  if (!g_rfidInitialized) {
    mfrc522.PCD_Init();
    Serial.println(F("MFRC522 baslatildi (ayri SPI pinleri ile)."));
    mfrc522.PCD_DumpVersionToSerial();
    g_rfidInitialized = true;
  }
}

// -----------------------------------------------------------------------------
// Ekran State Machine
// -----------------------------------------------------------------------------
enum ScreenState
{
  SCR_SETUP_MENU = 0,
  SCR_WIFI_SETTINGS,
  SCR_PHONE_API,
  SCR_ADMIN_CARD,
  SCR_DRIVER_MENU,           // Sofor alt menü
  SCR_DRIVER_CARD,           // Yeni kart + plaka
  SCR_DRIVER_LIST,           // Kayitli kartlar
  SCR_TEXT_INPUT,
  SCR_FACTORY_RESET_CONFIRM,
  SCR_MESSAGE,               // Bilgi / bildirim ekrani
  SCR_IDLE,                  // Normal bekleme
  SCR_FUELING,               // Dolum devam ediyor
  SCR_FUEL_SUMMARY           // Dolum ozeti / bitti ekrani
};

ScreenState currentScreen = SCR_SETUP_MENU;

// -----------------------------------------------------------------------------
// RS485 / Modbus / Normal Mod Durumlari
// -----------------------------------------------------------------------------
const uint8_t  MB_SLAVE_ADDR          = 1;
const uint8_t  MB_FC_READ_HOLDING     = 0x03;
const uint8_t  MB_FC_WRITE_SINGLE_REG = 0x06;

const uint16_t REG_CONTROL_CMD   = 0;
const uint16_t REG_STATUS_FLAGS  = 1;
const uint16_t REG_SESSION_VOL_H = 2;
const uint16_t REG_SESSION_VOL_L = 3;
const uint16_t REG_TOTAL_VOL_H   = 4;
const uint16_t REG_TOTAL_VOL_L   = 5;
const uint16_t REG_FLOW_RATE     = 6;

const uint16_t STATUS_READY_BIT          = (1 << 0);
const uint16_t STATUS_SESSION_ACTIVE_BIT = (1 << 1);
const uint16_t STATUS_ERROR_BIT          = (1 << 2);
const uint16_t STATUS_FLOW_ACTIVE_BIT    = (1 << 3);

struct MeterData
{
  uint16_t statusFlags;
  uint32_t sessionVolCl;
  uint32_t totalVolCl;
  uint16_t flowRateClm;
};

MeterData    g_lastMeter;
bool         g_sessionActive           = false;
String       g_activeDriverUid;
String       g_activeDriverPlate;
float        g_lastSessionLiters       = 0.0f;
unsigned long g_lastMeterPollMs        = 0;
const unsigned long METER_POLL_INTERVAL_MS = 300;

// Dolum bitti ekrani zamanlayici
unsigned long g_fuelSummaryStartMs     = 0;
const unsigned long FUEL_SUMMARY_DISPLAY_MS = 3000;

// -----------------------------------------------------------------------------
// Ana Menü Butonları
// -----------------------------------------------------------------------------
enum ButtonId
{
  BTN_WIFI = 0,
  BTN_RFID_MENU,
  BTN_PHONE_API,
  BTN_LOG,
  BTN_SAVE_EXIT,
  BTN_FACTORY_RESET,
  BTN_COUNT
};

struct Button
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  const char *label;
};

Button buttons[BTN_COUNT];

// Sofor/RFID alt menü butonlari
struct RectBtn {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

RectBtn driverMenuNewBtn;
RectBtn driverMenuAdminBtn;
RectBtn driverMenuListBtn;

// Kayitli kart listesi icin scroll
int driverListFirstIndex = 0;
const int DRIVER_LIST_ROW_H = 18;


// -----------------------------------------------------------------------------
// Konfig Yapısı (NVS)
// -----------------------------------------------------------------------------
#define MAX_DRIVERS  20

struct WifiConfig
{
  String ssid;
  String password;
  bool   isSet;
};

struct PhoneApiConfig
{
  String phoneNumber;
  String apiKey;
  bool   isSet;
};

struct AdminCardConfig
{
  String uidHex;
  bool   isSet;
};

struct DriverCard
{
  String uidHex;
  String plate;
};

struct DriverCardList
{
  DriverCard items[MAX_DRIVERS];
  uint8_t    count;
};

struct AppConfig
{
  WifiConfig      wifi;
  PhoneApiConfig  phoneApi;
  AdminCardConfig adminCard;
  DriverCardList  drivers;
};

Preferences prefs;
AppConfig   config;

// -----------------------------------------------------------------------------
// Klavye Yapısı
// -----------------------------------------------------------------------------
enum KeyboardLayout
{
  KB_LAYOUT_UPPER = 0,
  KB_LAYOUT_LOWER,
  KB_LAYOUT_NUMSYM
};

enum KeyType
{
  KT_CHAR = 0,
  KT_BACKSPACE,
  KT_SPACE,
  KT_ENTER,
  KT_LAYOUT_CYCLE
};

struct KeyboardKey
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  char    label[6];
  KeyType type;
  char    value;
};

#define MAX_KEYS 60
KeyboardKey    kbKeys[MAX_KEYS];
uint8_t        kbKeyCount = 0;
KeyboardLayout kbCurrentLayout = KB_LAYOUT_UPPER;

enum TextInputPurpose
{
  TIP_NONE = 0,
  TIP_GENERIC,
  TIP_WIFI_PASSWORD,
  TIP_DRIVER_PLATE,
  TIP_PHONE_NUMBER
};

struct TextInputContext
{
  String      title;
  String      hint;
  String     *target;
  uint16_t    maxLen;
  ScreenState returnScreen;
  bool        active;
};

TextInputContext  textInput;
TextInputPurpose  textInputPurpose = TIP_NONE;
String            kbBuffer;
uint16_t          kbMaxLen = 0;

// -----------------------------------------------------------------------------
// WiFi Tarama Listesi
// -----------------------------------------------------------------------------
#define WIFI_MAX_NETWORKS     20
#define WIFI_LIST_ROWS        5

struct WifiScanItem
{
  String  ssid;
  int32_t rssi;
  bool    secure;
};

WifiScanItem wifiScanList[WIFI_MAX_NETWORKS];
int          wifiScanCount      = 0;
int          wifiListFirstIndex = 0;
int          wifiSelectedIndex  = -1;
String       wifiPasswordBuffer;

// -----------------------------------------------------------------------------
// Telefon / API
// -----------------------------------------------------------------------------
String phoneEditBuffer;   // '+' SONRASI
String apiKeyEditBuffer;

// -----------------------------------------------------------------------------
// RFID ile ilgili degiskenler (Admin + Sofor)
// -----------------------------------------------------------------------------
String adminLastUid;
String driverCurrentUid;
String driverPlateBuffer;
String driverScreenInfo;

// -----------------------------------------------------------------------------
// WiFi + NTP / Zaman
// -----------------------------------------------------------------------------
bool wifiClientStarted      = false;
bool timeConfigured         = false;
unsigned long lastTopBarUpdateMs = 0;

const long GMT_OFFSET_SEC       = 3 * 3600;  // Türkiye UTC+3
const int  DAYLIGHT_OFFSET_SEC  = 0;
const char *NTP_SERVER          = "pool.ntp.org";

// -----------------------------------------------------------------------------
// Bilgi Mesaji (SCR_MESSAGE)
// -----------------------------------------------------------------------------
struct InfoMessage {
  String      title;
  String      line1;
  String      line2;
  ScreenState returnScreen;
  uint32_t    startMs;
  uint32_t    timeoutMs;
};

InfoMessage infoMsg;

// -----------------------------------------------------------------------------
// Prototipler
// -----------------------------------------------------------------------------
void initConfigDefaults();
void loadConfigFromNVS();
void saveConfigToNVS();

void configSetWifi(const String &ssid, const String &password, bool save = true);
void configSetPhoneApi(const String &phone, const String &apiKey, bool save = true);
void configSetAdminCard(const String &uidHex, bool save = true);
bool configAddOrUpdateDriver(const String &uidHex, const String &plate, bool save = true);

bool isConfigOkForButton(ButtonId id);
void drawStatusForButton(ButtonId id, bool ok, uint16_t fillColor);

// Top bar / WiFi / Zaman
const char* getScreenTitle(ScreenState s);
void drawWifiIcon(int16_t x, int16_t y, bool connected);
String getCurrentTimeString();
String getCurrentDateString();
void drawTopBar(const char* title);
void updateTopBarForCurrentScreen();
void handleWifiAndTime();
bool wifiAttemptConnectBlocking(const String &ssid, const String &password);

// Bilgi mesaji
void showInfoMessage(const String &title, const String &line1, const String &line2,
                     uint8_t retScreenState, uint32_t durationMs = 1500);

// Ana menü
void drawSetupMenu();
void drawButton(ButtonId id, bool pressed);
int  hitTestButtons(int16_t x, int16_t y);
bool readTouchPress(int16_t &x, int16_t &y);
void handleTouchOnSetupMenu();
void handleButtonPress(ButtonId id);

// Klavye / metin girişi
void setKeyLabel(KeyboardKey &key, const char *text);
void kbStart(const String &title, const String &hint, String *target, uint16_t maxLen,
             uint8_t returnScreenState, TextInputPurpose purpose);
void drawTextInputScreen();
void kbDrawTextLine();
void kbBuildLayout();
void kbBuildLayoutGeneric();
void kbBuildLayoutDriverPlate();
void kbBuildLayoutPhone();
void kbDrawKeyboard();
void kbDrawKey(uint8_t index, bool pressed);
int  kbHitTestKey(int16_t x, int16_t y);
void handleKeyboardTouch();
void kbProcessKey(uint8_t index);

// WiFi ayarları
void startWifiSettingsScreen();
void wifiScanNetworks();
void drawWifiSettingsScreen();
void drawWifiNetworksList();
void handleTouchOnWifiSettings();
void wifiOpenPasswordInput(int index);

// Telefon / API ekranı
void startPhoneApiScreen();
void drawPhoneApiScreen();
void handleTouchOnPhoneApi();

// RFID yardımcı
String uidToHexString(const MFRC522::Uid &uid);

// Admin Kart ekranı
void startAdminCardScreen();
void drawAdminCardScreen(const String &uidHex);
void handleTouchOnAdminCard();

// Sofor alt menü + yeni kart + liste
void startDriverMenuScreen();
void drawDriverMenuScreen();
void handleTouchOnDriverMenu();

void startDriverCardScreen();
void drawDriverCardScreen(const String &infoLine);
void handleTouchOnDriverCard();

void startDriverListScreen();
void drawDriverListScreen();
void handleTouchOnDriverList();

// Factory reset
void drawFactoryResetConfirmScreen();
void handleTouchOnFactoryResetConfirm();
void doFactoryReset();

// RS485 / Modbus / Normal Mod
void initRs485();
uint16_t modbusCRC16(const uint8_t *data, uint16_t length);
void rs485SetTx();
void rs485SetRx();
bool rs485ReadBytes(uint8_t *buf, uint16_t len, uint32_t timeoutMs);
bool modbusWriteSingleRegister(uint8_t slaveAddr, uint16_t regAddr, uint16_t value);
bool modbusReadHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t *outRegs);
bool meterStartSession();
bool meterRead(MeterData &out);

void drawIdleScreen();
void handleTouchOnIdle();
void drawFuelingScreen(const MeterData &md);
void handleTouchOnFueling();
void drawFuelSummaryScreen();
void handleTouchOnFuelSummary();
void handleRfidInNormalMode();
void handleMeterPolling();
int  findDriverIndexByUid(const String &uidHex);
bool isNormalModeConfigComplete();

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println(F("=== ESP32 WROVER + PSRAM Yakıt Terminali ==="));

  if (psramFound()) {
    Serial.printf("PSRAM bulundu. Bos PSRAM: %d bayt\n", ESP.getFreePsram());
  } else {
    Serial.println(F("UYARI: PSRAM bulunamadi! Sprite normal RAM'den alinacak."));
  }

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);

  tft.init();
  tft.setRotation(3);

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  g_spiOwner = SPI_OWNER_TFT;

  ts.begin();
  ts.setRotation(3);

  spr.setColorDepth(16);
  void *buf = spr.createSprite(tft.width(), tft.height());
  if (!buf) {
    Serial.println(F("HATA: Sprite icin bellek ayrilamadi!"));
  } else {
    Serial.printf("Sprite olusturuldu (%dx%d)\n", spr.width(), spr.height());
  }
  spr.fillSprite(TFT_BLACK);
  spr.pushSprite(0, 0);

  initConfigDefaults();
  loadConfigFromNVS();

  // RS485 başlat
  initRs485();

  // Açılışta kullanıcı ayarları kontrolü
  spr.fillSprite(TFT_BLACK);
  drawTopBar("Baslangic");
  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t sw = spr.width();
  int16_t sh = spr.height();
  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  spr.drawString("Kullanici ayarlari", sw / 2, centerY - 10);
  spr.drawString("kontrol ediliyor...", sw / 2, centerY + 10);
  spr.pushSprite(0, 0);

  Serial.println(F("Kullanici ayarlari kontrol ediliyor..."));
  delay(1000);

  bool configOk = isNormalModeConfigComplete();

  if (configOk)
  {
    Serial.println(F("Tum ayarlar tam. WiFi baglantisi denenecek."));

    spr.fillSprite(TFT_BLACK);
    drawTopBar("Baslangic");
    spr.setTextDatum(MC_DATUM);
    spr.setTextFont(FONT_MAIN);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.drawString("Ayarlar tamam.", sw / 2, centerY - 10);
    spr.drawString("WiFi'ya baglaniliyor...", sw / 2, centerY + 10);
    spr.pushSprite(0, 0);

    bool wifiOk = wifiAttemptConnectBlocking(config.wifi.ssid, config.wifi.password);
    if (wifiOk)
    {
      Serial.println(F("WiFi baglandi. Sofor kart ekranindan baslaniyor."));
      currentScreen = SCR_IDLE;
      drawIdleScreen();
    }
    else
    {
      Serial.println(F("WiFi baglanamadi. Ayarlar menusune geciliyor."));

      spr.fillSprite(TFT_BLACK);
      drawTopBar("Baslangic");
      spr.setTextDatum(MC_DATUM);
      spr.setTextFont(FONT_MAIN);
      spr.setTextColor(TFT_YELLOW, TFT_BLACK);
      spr.drawString("WiFi baglanamadi.", sw / 2, centerY - 10);
      spr.drawString("Ayarlar ekranina gidiliyor.", sw / 2, centerY + 10);
      spr.pushSprite(0, 0);
      delay(1500);

      currentScreen = SCR_SETUP_MENU;
      drawSetupMenu();
    }
  }
  else
  {
    Serial.println(F("Eksik ayar var. Kurulum menusu ile baslaniyor."));

    spr.fillSprite(TFT_BLACK);
    drawTopBar("Baslangic");
    spr.setTextDatum(MC_DATUM);
    spr.setTextFont(FONT_MAIN);
    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.drawString("Eksik ayar var.", sw / 2, centerY - 10);
    spr.drawString("Ayarlar ekranina gidiliyor.", sw / 2, centerY + 10);
    spr.pushSprite(0, 0);
    delay(1500);

    currentScreen = SCR_SETUP_MENU;
    drawSetupMenu();
  }

  Serial.println(F("Hazir."));
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop()
{
  handleWifiAndTime();

  unsigned long nowMs = millis();
  if (nowMs - lastTopBarUpdateMs >= 1000) {
    lastTopBarUpdateMs = nowMs;
    if (currentScreen != SCR_MESSAGE)
      updateTopBarForCurrentScreen();
  }

  switch (currentScreen)
  {
    case SCR_SETUP_MENU:
      handleTouchOnSetupMenu();
      break;

    case SCR_WIFI_SETTINGS:
      handleTouchOnWifiSettings();
      break;

    case SCR_PHONE_API:
      handleTouchOnPhoneApi();
      break;

    case SCR_ADMIN_CARD:
      handleTouchOnAdminCard();
      break;

    case SCR_DRIVER_MENU:
      handleTouchOnDriverMenu();
      break;

    case SCR_DRIVER_CARD:
      handleTouchOnDriverCard();
      break;

    case SCR_DRIVER_LIST:
      handleTouchOnDriverList();
      break;

    case SCR_TEXT_INPUT:
      handleKeyboardTouch();
      break;

    case SCR_FACTORY_RESET_CONFIRM:
      handleTouchOnFactoryResetConfirm();
      break;

    case SCR_IDLE:
      handleTouchOnIdle();
      break;

    case SCR_FUELING:
      handleTouchOnFueling();
      handleMeterPolling();
      break;

    case SCR_FUEL_SUMMARY:
      handleTouchOnFuelSummary();
      break;

    case SCR_MESSAGE:
      if (millis() - infoMsg.startMs >= infoMsg.timeoutMs)
      {
        ScreenState ret = infoMsg.returnScreen;
        currentScreen = ret;

        if (ret == SCR_SETUP_MENU)
          drawSetupMenu();
        else if (ret == SCR_WIFI_SETTINGS)
          drawWifiSettingsScreen();
        else if (ret == SCR_PHONE_API)
          drawPhoneApiScreen();
        else if (ret == SCR_ADMIN_CARD)
          drawAdminCardScreen(adminLastUid);
        else if (ret == SCR_DRIVER_MENU)
          drawDriverMenuScreen();
        else if (ret == SCR_DRIVER_CARD)
          drawDriverCardScreen(driverScreenInfo);
        else if (ret == SCR_DRIVER_LIST)
          drawDriverListScreen();
        else if (ret == SCR_IDLE)
          drawIdleScreen();
        else if (ret == SCR_FUEL_SUMMARY)
          drawFuelSummaryScreen();
      }
      break;

    default:
      break;
  }

  // Normal mod RFID okuma (Idle / Fueling / Summary)
  if (currentScreen == SCR_IDLE ||
      currentScreen == SCR_FUELING ||
      currentScreen == SCR_FUEL_SUMMARY) {
    handleRfidInNormalMode();
  }
}

// -----------------------------------------------------------------------------
// Konfig: Varsayılan değerler
// -----------------------------------------------------------------------------
void initConfigDefaults()
{
  config.wifi.ssid     = "";
  config.wifi.password = "";
  config.wifi.isSet    = false;

  config.phoneApi.phoneNumber = "";
  config.phoneApi.apiKey      = "";
  config.phoneApi.isSet       = false;

  config.adminCard.uidHex = "";
  config.adminCard.isSet  = false;

  config.drivers.count = 0;
  for (uint8_t i = 0; i < MAX_DRIVERS; i++)
  {
    config.drivers.items[i].uidHex = "";
    config.drivers.items[i].plate  = "";
  }
}

// -----------------------------------------------------------------------------
// Konfig: NVS'den yükle
// -----------------------------------------------------------------------------
void loadConfigFromNVS()
{
  if (!prefs.begin("fuelterm", true))
  {
    Serial.println(F("NVS acilamadi (read). Varsayilan konfig kullaniliyor."));
    return;
  }

  config.wifi.ssid     = prefs.getString("wifi_ssid", "");
  config.wifi.password = prefs.getString("wifi_pwd", "");
  config.wifi.isSet    = (config.wifi.ssid.length() > 0);

  config.phoneApi.phoneNumber = prefs.getString("phone", "");
  config.phoneApi.apiKey      = prefs.getString("api_key", "");
  config.phoneApi.isSet       =
      (config.phoneApi.phoneNumber.length() > 0 && config.phoneApi.apiKey.length() > 0);

  config.adminCard.uidHex = prefs.getString("admin_uid", "");
  config.adminCard.isSet  = (config.adminCard.uidHex.length() > 0);

  uint32_t drvCount = prefs.getUInt("drv_count", 0);
  if (drvCount > MAX_DRIVERS) drvCount = MAX_DRIVERS;
  config.drivers.count = (uint8_t)drvCount;

  for (uint8_t i = 0; i < config.drivers.count; i++)
  {
    String keyUid   = "drv_uid_" + String(i);
    String keyPlate = "drv_lic_" + String(i);

    config.drivers.items[i].uidHex = prefs.getString(keyUid.c_str(), "");
    config.drivers.items[i].plate  = prefs.getString(keyPlate.c_str(), "");
  }

  prefs.end();

  Serial.println(F("NVS'den konfig yüklendi:"));
  Serial.printf("  WiFi: %s\n",  config.wifi.isSet      ? config.wifi.ssid.c_str()      : "YOK");
  Serial.printf("  Tel: %s\n",   config.phoneApi.isSet  ? config.phoneApi.phoneNumber.c_str() : "YOK");
  Serial.printf("  API: %s\n",   config.phoneApi.isSet  ? "VAR" : "YOK");
  Serial.printf("  Admin UID: %s\n", config.adminCard.isSet ? config.adminCard.uidHex.c_str() : "YOK");
  Serial.printf("  Sofor kart sayisi: %u\n", config.drivers.count);
}

// -----------------------------------------------------------------------------
// Konfig: NVS'ye kaydet
// -----------------------------------------------------------------------------
void saveConfigToNVS()
{
  if (!prefs.begin("fuelterm", false))
  {
    Serial.println(F("NVS acilamadi (write). Kayit yapilamadi!"));
    return;
  }

  prefs.putString("wifi_ssid", config.wifi.ssid);
  prefs.putString("wifi_pwd",  config.wifi.password);

  prefs.putString("phone",   config.phoneApi.phoneNumber);
  prefs.putString("api_key", config.phoneApi.apiKey);

  prefs.putString("admin_uid", config.adminCard.uidHex);

  prefs.putUInt("drv_count", config.drivers.count);
  for (uint8_t i = 0; i < config.drivers.count; i++)
  {
    String keyUid   = "drv_uid_" + String(i);
    String keyPlate = "drv_lic_" + String(i);

    prefs.putString(keyUid.c_str(),   config.drivers.items[i].uidHex);
    prefs.putString(keyPlate.c_str(), config.drivers.items[i].plate);
  }

  prefs.end();
  Serial.println(F("Konfig NVS'ye kaydedildi."));
}

// -----------------------------------------------------------------------------
// Konfig yardımcıları
// -----------------------------------------------------------------------------
void configSetWifi(const String &ssid, const String &password, bool save)
{
  config.wifi.ssid     = ssid;
  config.wifi.password = password;
  config.wifi.isSet    = (ssid.length() > 0);

  if (save) saveConfigToNVS();
}

void configSetPhoneApi(const String &phone, const String &apiKey, bool save)
{
  config.phoneApi.phoneNumber = phone;
  config.phoneApi.apiKey      = apiKey;
  config.phoneApi.isSet       =
      (phone.length() > 0 && apiKey.length() > 0);

  if (save) saveConfigToNVS();
}

void configSetAdminCard(const String &uidHex, bool save)
{
  config.adminCard.uidHex = uidHex;
  config.adminCard.isSet  = (uidHex.length() > 0);

  if (save) saveConfigToNVS();
}

bool configAddOrUpdateDriver(const String &uidHex, const String &plate, bool save)
{
  for (uint8_t i = 0; i < config.drivers.count; i++)
  {
    if (config.drivers.items[i].uidHex == uidHex)
    {
      config.drivers.items[i].plate = plate;
      if (save) saveConfigToNVS();
      return true;
    }
  }

  if (config.drivers.count >= MAX_DRIVERS)
  {
    Serial.println(F("Sofor kart listesi dolu! Yeni kart eklenemedi."));
    return false;
  }

  uint8_t idx = config.drivers.count;
  config.drivers.items[idx].uidHex = uidHex;
  config.drivers.items[idx].plate  = plate;
  config.drivers.count++;

  if (save) saveConfigToNVS();
  return true;
}

int findDriverIndexByUid(const String &uidHex)
{
  for (uint8_t i = 0; i < config.drivers.count; i++)
  {
    if (config.drivers.items[i].uidHex == uidHex)
      return (int)i;
  }
  return -1;
}

// Normal moda gecmek icin gerekli asgari alanlar:
// - WiFi ayarli
// - Yonetici kart tanimli
// - En az 1 sofor karti kayitli
// Telefon / API artik opsiyonel (dahili log kullandigimiz icin)
bool isNormalModeConfigComplete()
{
  return config.wifi.isSet &&
         config.adminCard.isSet &&
         (config.drivers.count > 0);
}

// -----------------------------------------------------------------------------
// Bir buton için konfig durumu [OK] / [X]
// -----------------------------------------------------------------------------
bool isConfigOkForButton(ButtonId id)
{
  switch (id)
  {
    case BTN_WIFI:
      // WiFi ayarlari yapilmis mi?
      return config.wifi.isSet;

    case BTN_RFID_MENU:
      // RFID ayarlari: hem yonetici kart hem de en az 1 sofor karti olsun
      return config.adminCard.isSet && (config.drivers.count > 0);

    case BTN_PHONE_API:
      // Telefon / API istege bagli, ama durumunu gostermek guzel
      return config.phoneApi.isSet;

    default:
      return false;                           // Diger satirlarda ikon yok
  }
}

// -----------------------------------------------------------------------------
// Belirli bir butonun köşesine [OK] / [X] çiz
// -----------------------------------------------------------------------------
void drawStatusForButton(ButtonId id, bool ok, uint16_t fillColor)
{
  Button &b = buttons[id];

  const char *txt = ok ? "[OK]" : "[X]";
  uint16_t txtColor = ok ? TFT_GREEN : TFT_RED;

  spr.setTextDatum(TR_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(txtColor, fillColor);

  int16_t x = b.x + b.w - 4;
  int16_t y = b.y + 4;

  spr.drawString(txt, x, y);
}

// -----------------------------------------------------------------------------
// Top bar: baslik, wifi ikon, tarih/saat
// -----------------------------------------------------------------------------
const char* getScreenTitle(ScreenState s)
{
  switch (s)
  {
    case SCR_SETUP_MENU:            return "Ayarlar";
    case SCR_WIFI_SETTINGS:         return "WiFi Ayarlari";
    case SCR_PHONE_API:             return "Telefon / API";
    case SCR_ADMIN_CARD:            return "Yonetici RFID";
    case SCR_DRIVER_MENU:           return "RFID Ayarlari";
    case SCR_DRIVER_CARD:           return "Yeni Sofor RFID/Plaka";
    case SCR_DRIVER_LIST:           return "Kayitli RFID ve Plakalar";
    case SCR_TEXT_INPUT:            return textInput.title.c_str();
    case SCR_FACTORY_RESET_CONFIRM: return "Factory Reset";
    case SCR_IDLE:                  return "Bekleme";
    case SCR_FUELING:               return "Dolum";
    case SCR_FUEL_SUMMARY:          return "Dolum Bitti";
    case SCR_MESSAGE:               return infoMsg.title.c_str();
    default:                        return "";
  }
}

// Akilli telefon tarzı "sebekes" ikon (dikey barlar)
void drawWifiIcon(int16_t x, int16_t y, bool connected)
{
  uint16_t colOn  = connected ? TFT_GREEN    : TFT_DARKGREY;
  uint16_t colOff = TFT_DARKGREY;

  int baseY      = y + TOP_BAR_H - 3;  // alt hizası
  int barW       = 3;
  int gap        = 2;
  int numBars    = 4;

  for (int i = 0; i < numBars; i++)
  {
    int h   = 4 + i * 3;                // 4, 7, 10, 13 piksel
    int bx  = x + i * (barW + gap);
    uint16_t col = connected ? colOn : colOff;

    spr.fillRect(bx, baseY - h, barW, h, col);
  }
}


String getCurrentTimeString()
{
  time_t now;
  struct tm ti;
  time(&now);
  localtime_r(&now, &ti);

  int yearFull = ti.tm_year + 1900;
  if (yearFull < 2020) {
    return String("--:--:--");
  }

  int hh = ti.tm_hour;
  int mm = ti.tm_min;
  int ss = ti.tm_sec;

  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hh, mm, ss);
  return String(buf);
}

String getCurrentDateString()
{
  time_t now;
  struct tm ti;
  time(&now);
  localtime_r(&now, &ti);

  int yearFull = ti.tm_year + 1900;
  if (yearFull < 2020) {
    return String("--.--.--");
  }

  int dd = ti.tm_mday;
  int mo = ti.tm_mon + 1;
  int yy = yearFull % 100;

  char buf[16];
  snprintf(buf, sizeof(buf), "%02d.%02d.%02d", dd, mo, yy);
  return String(buf);
}

void drawTopBar(const char* title)
{
  int16_t sw = spr.width();
  spr.fillRect(0, 0, sw, TOP_BAR_H, TFT_BLUE);

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  drawWifiIcon(2, 0, wifiConnected);

  String dateStr = getCurrentDateString();
  String timeStr = getCurrentTimeString();

  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);

  spr.setTextDatum(TR_DATUM);
  spr.drawString(dateStr, sw - 4, 4);

  spr.setTextDatum(BR_DATUM);
  spr.drawString(timeStr, sw - 4, TOP_BAR_H - 2);

  spr.setTextDatum(MC_DATUM);
  spr.drawString(title, sw / 2, TOP_BAR_H / 2 + 2);
}

void updateTopBarForCurrentScreen()
{
  const char* title = getScreenTitle(currentScreen);
  drawTopBar(title);
  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// WiFi + NTP yönetimi
// -----------------------------------------------------------------------------
void handleWifiAndTime()
{
  // Artık burada otomatik WiFi baglanti denemiyoruz.
  // Sadece baglanti kurulmus ise NTP zamanini ayarliyoruz.
  if (WiFi.status() == WL_CONNECTED && !timeConfigured)
  {
    Serial.print(F("WiFi baglandi. IP: "));
    Serial.println(WiFi.localIP());
    Serial.println(F("NTP ayarlaniyor..."));
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    timeConfigured = true;
  }
}

// WiFi: bloklu baglanma denemesi
bool wifiAttemptConnectBlocking(const String &ssid, const String &password)
{
  Serial.printf("WiFi baglantisi (bloklu) deneniyor: %s\n", ssid.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(100);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long start = millis();
  const unsigned long timeout = 15000; // 15 saniye

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout)
  {
    delay(200);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("WiFi baglandi. IP: "));
    Serial.println(WiFi.localIP());
    wifiClientStarted = true;
    timeConfigured    = false;
    return true;
  }
  else
  {
    Serial.println(F("WiFi baglanamadi (timeout veya hata)."));
    wifiClientStarted = false;
    return false;
  }
}

// -----------------------------------------------------------------------------
// RS485 / Modbus Fonksiyonlari
// -----------------------------------------------------------------------------
void initRs485()
{
  pinMode(RS485_REDE_PIN, OUTPUT);
  rs485SetRx();
  RS485Serial.begin(19200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.println(F("RS485 baslatildi (UART2, 19200 8N1)."));
}

void rs485SetTx()
{
  digitalWrite(RS485_REDE_PIN, HIGH);
}

void rs485SetRx()
{
  digitalWrite(RS485_REDE_PIN, LOW);
}

uint16_t modbusCRC16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool rs485ReadBytes(uint8_t *buf, uint16_t len, uint32_t timeoutMs)
{
  uint32_t start = millis();
  uint16_t index = 0;
  while (index < len) {
    if (RS485Serial.available() > 0) {
      buf[index++] = (uint8_t)RS485Serial.read();
    } else {
      if (millis() - start > timeoutMs) return false;
      delay(1);
    }
  }
  return true;
}

bool modbusWriteSingleRegister(uint8_t slaveAddr, uint16_t regAddr, uint16_t value)
{
  uint8_t frame[8];
  frame[0] = slaveAddr;
  frame[1] = MB_FC_WRITE_SINGLE_REG;
  frame[2] = (uint8_t)((regAddr >> 8) & 0xFF);
  frame[3] = (uint8_t)(regAddr & 0xFF);
  frame[4] = (uint8_t)((value >> 8) & 0xFF);
  frame[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = modbusCRC16(frame, 6);
  frame[6] = (uint8_t)(crc & 0xFF);
  frame[7] = (uint8_t)((crc >> 8) & 0xFF);

  while (RS485Serial.available() > 0) RS485Serial.read();

  rs485SetTx();
  RS485Serial.write(frame, 8);
  RS485Serial.flush();
  rs485SetRx();

  uint8_t resp[8];
  if (!rs485ReadBytes(resp, 8, 100)) {
    Serial.println(F("modbusWriteSingleRegister: timeout"));
    return false;
  }

  uint16_t recvCrc = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t calcCrc = modbusCRC16(resp, 6);
  if (recvCrc != calcCrc) {
    Serial.println(F("modbusWriteSingleRegister: CRC hatasi"));
    return false;
  }

  if (resp[0] != slaveAddr || resp[1] != MB_FC_WRITE_SINGLE_REG) {
    Serial.println(F("modbusWriteSingleRegister: addr/fn hatali"));
    return false;
  }

  uint16_t regEcho = ((uint16_t)resp[2] << 8) | resp[3];
  uint16_t valEcho = ((uint16_t)resp[4] << 8) | resp[5];
  if (regEcho != regAddr || valEcho != value) {
    Serial.println(F("modbusWriteSingleRegister: echo farkli"));
    return false;
  }

  return true;
}

bool modbusReadHoldingRegisters(uint8_t slaveAddr, uint16_t startAddr, uint16_t quantity, uint16_t *outRegs)
{
  if (quantity == 0) return false;

  uint8_t frame[8];
  frame[0] = slaveAddr;
  frame[1] = MB_FC_READ_HOLDING;
  frame[2] = (uint8_t)((startAddr >> 8) & 0xFF);
  frame[3] = (uint8_t)(startAddr & 0xFF);
  frame[4] = (uint8_t)((quantity >> 8) & 0xFF);
  frame[5] = (uint8_t)(quantity & 0xFF);
  uint16_t crc = modbusCRC16(frame, 6);
  frame[6] = (uint8_t)(crc & 0xFF);
  frame[7] = (uint8_t)((crc >> 8) & 0xFF);

  while (RS485Serial.available() > 0) RS485Serial.read();

  rs485SetTx();
  RS485Serial.write(frame, 8);
  RS485Serial.flush();
  rs485SetRx();

  uint8_t hdr[3];
  if (!rs485ReadBytes(hdr, 3, 100)) {
    Serial.println(F("modbusReadHolding: timeout header"));
    return false;
  }

  if (hdr[0] != slaveAddr) {
    Serial.println(F("modbusReadHolding: slave addr farkli"));
    return false;
  }

  if (hdr[1] != MB_FC_READ_HOLDING) {
    Serial.print(F("modbusReadHolding: fn kodu: 0x"));
    Serial.println(hdr[1], HEX);
    return false;
  }

  uint8_t byteCount = hdr[2];
  uint16_t expectedBytes = quantity * 2;
  if (byteCount != expectedBytes) {
    Serial.println(F("modbusReadHolding: byteCount farkli"));
    return false;
  }

  uint16_t remain = byteCount + 2;
  uint8_t buf2[64];
  if (!rs485ReadBytes(buf2, remain, 100)) {
    Serial.println(F("modbusReadHolding: timeout data+crc"));
    return false;
  }

  uint8_t full[3 + 64];
  full[0] = hdr[0];
  full[1] = hdr[1];
  full[2] = hdr[2];
  memcpy(full + 3, buf2, remain);
  uint16_t fullLen = 3 + remain;

  uint16_t recvCrc = (uint16_t)buf2[remain - 2] | ((uint16_t)buf2[remain - 1] << 8);
  uint16_t calcCrc = modbusCRC16(full, fullLen - 2);
  if (recvCrc != calcCrc) {
    Serial.println(F("modbusReadHolding: CRC hatasi"));
    return false;
  }

  for (uint16_t i = 0; i < quantity; i++) {
    uint16_t hi = buf2[2 * i];
    uint16_t lo = buf2[2 * i + 1];
    outRegs[i] = (hi << 8) | lo;
  }

  return true;
}

bool meterStartSession()
{
  Serial.println(F("meterStartSession(): CONTROL_CMD=1"));
  return modbusWriteSingleRegister(MB_SLAVE_ADDR, REG_CONTROL_CMD, 1);
}

bool meterRead(MeterData &out)
{
  uint16_t regs[6];
  if (!modbusReadHoldingRegisters(MB_SLAVE_ADDR, REG_STATUS_FLAGS, 6, regs)) {
    return false;
  }
  out.statusFlags = regs[0];
  uint16_t svH = regs[1];
  uint16_t svL = regs[2];
  uint16_t tvH = regs[3];
  uint16_t tvL = regs[4];
  out.sessionVolCl = ((uint32_t)svH << 16) | svL;
  out.totalVolCl   = ((uint32_t)tvH << 16) | tvL;
  out.flowRateClm  = regs[5];
  return true;
}

// -----------------------------------------------------------------------------
// Bilgi Mesaji Ekrani
// -----------------------------------------------------------------------------
void showInfoMessage(const String &title, const String &line1, const String &line2,
                     uint8_t retScreenState, uint32_t durationMs)
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar(title.c_str());

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  if (line1.length() > 0)
    spr.drawString(line1, sw / 2, centerY - 10);
  if (line2.length() > 0)
    spr.drawString(line2, sw / 2, centerY + 10);

  spr.pushSprite(0, 0);

  infoMsg.title        = title;
  infoMsg.line1        = line1;
  infoMsg.line2        = line2;
  infoMsg.returnScreen = (ScreenState)retScreenState;
  infoMsg.startMs      = millis();
  infoMsg.timeoutMs    = durationMs;

  currentScreen = SCR_MESSAGE;
}

// -----------------------------------------------------------------------------
// Kurulum Ana Menüsü Çizimi (dikey liste)
// -----------------------------------------------------------------------------
void drawSetupMenu()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar(getScreenTitle(SCR_SETUP_MENU));

  int16_t sw = spr.width();

  int16_t marginX = 8;
  int16_t marginY = 4;

  int16_t btnW   = sw - 2 * marginX;
  int16_t btnH   = 24;                // 6 satir icin yeterli
  int16_t startY = TOP_BAR_H + 6;

  auto setBtn = [&](ButtonId id, const char *label, int index)
  {
    buttons[id].x     = marginX;
    buttons[id].y     = startY + index * (btnH + marginY);
    buttons[id].w     = btnW;
    buttons[id].h     = btnH;
    buttons[id].label = label;
  };

  // Yukaridan asagi yeni sira:
  setBtn(BTN_WIFI,         "WiFi Ayarlari",        0);
  setBtn(BTN_RFID_MENU,    "RFID Ayarlari",        1);
  setBtn(BTN_PHONE_API,    "Telefon / API",        2);
  setBtn(BTN_LOG,          "Dahili log",           3);
  setBtn(BTN_SAVE_EXIT,    "Kaydet ve Cik",        4);
  setBtn(BTN_FACTORY_RESET,"Factory Reset",        5);

  for (int i = 0; i < BTN_COUNT; i++)
  {
    drawButton((ButtonId)i, false);
  }

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Tek bir butonu çiz
// -----------------------------------------------------------------------------
void drawButton(ButtonId id, bool pressed)
{
  Button &b = buttons[id];

  uint16_t fillColor;
  if (id == BTN_FACTORY_RESET)
    fillColor = pressed ? TFT_MAROON : TFT_RED;    // Reset kırmızı
  else
    fillColor = pressed ? TFT_DARKCYAN : TFT_BLUE; // Diğerleri mavi ton

  uint16_t borderColor = TFT_WHITE;
  uint16_t textColor   = TFT_WHITE;

  spr.fillRoundRect(b.x, b.y, b.w, b.h, 6, fillColor);
  spr.drawRoundRect(b.x, b.y, b.w, b.h, 6, borderColor);

  spr.setTextDatum(ML_DATUM);   // Soldan hizalı, ortalanmış yükseklik
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(textColor, fillColor);

  int16_t textX = b.x + 6;
  int16_t textY = b.y + b.h / 2;
  spr.drawString(b.label, textX, textY);

  // Durum ikonu gosterilecek satirlar:
  // - WiFi Ayarlari
  // - RFID Ayarlari (yonetici + sofor kartlari)
  // - Telefon / API (opsiyonel)
  if (id == BTN_WIFI || id == BTN_RFID_MENU || id == BTN_PHONE_API)
  {
    bool ok = isConfigOkForButton(id);
    drawStatusForButton(id, ok, fillColor);
  }
}

// -----------------------------------------------------------------------------
// Hangi butona basildi?
// -----------------------------------------------------------------------------
int hitTestButtons(int16_t x, int16_t y)
{
  for (int i = 0; i < BTN_COUNT; i++)
  {
    Button &b = buttons[i];
    if (x >= b.x && x <= b.x + b.w &&
        y >= b.y && y <= b.y + b.h)
    {
      return i;
    }
  }
  return -1;
}

// -----------------------------------------------------------------------------
// Dokunmatik'ten tek basma olayi
// -----------------------------------------------------------------------------
bool readTouchPress(int16_t &x, int16_t &y)
{
  static bool wasTouched = false;

  bool nowTouched = ts.touched();

  if (nowTouched)
  {
    TS_Point p = ts.getPoint();

    int16_t sw = tft.width();
    int16_t sh = tft.height();

    int16_t rawX = p.x;
    int16_t rawY = p.y;

    int16_t mappedX = map(rawX, 200, 3800, 0, sw - 1);
    int16_t mappedY = map(rawY, 200, 3800, 0, sh - 1);

    if (TS_SWAP_XY)
    {
      int16_t tmp = mappedX;
      mappedX = mappedY;
      mappedY = tmp;
    }

    if (TS_INVERT_X)
      mappedX = (sw - 1) - mappedX;

    if (TS_INVERT_Y)
      mappedY = (sh - 1) - mappedY;

    if (mappedX < 0) mappedX = 0;
    if (mappedX >= sw) mappedX = sw - 1;
    if (mappedY < 0) mappedY = 0;
    if (mappedY >= sh) mappedY = sh - 1;

    if (!wasTouched)
    {
      wasTouched = true;
      x = mappedX;
      y = mappedY;
      return true;
    }
  }
  else
  {
    if (wasTouched)
    {
      wasTouched = false;
    }
  }

  return false;
}

// -----------------------------------------------------------------------------
// SETUP MENÜSÜ için dokunmatik
// -----------------------------------------------------------------------------
void handleTouchOnSetupMenu()
{
  int16_t x, y;
  if (readTouchPress(x, y))
  {
    int btn = hitTestButtons(x, y);
    if (btn >= 0)
    {
      handleButtonPress((ButtonId)btn);
    }
  }
}

// -----------------------------------------------------------------------------
// Bir menü butonuna basılınca
// -----------------------------------------------------------------------------
void handleButtonPress(ButtonId id)
{
  drawButton(id, true);
  spr.pushSprite(0, 0);
  delay(120);
  drawButton(id, false);
  spr.pushSprite(0, 0);

  switch (id)
  {
    case BTN_WIFI:
      Serial.println(F("WiFi Ayarlari butonu tiklandi."));
      startWifiSettingsScreen();
      break;

    case BTN_RFID_MENU:
      Serial.println(F("RFID Ayarlari butonu tiklandi."));
      startDriverMenuScreen();   // bu artik RFID alt menusu
      break;

    case BTN_PHONE_API:
      Serial.println(F("Telefon / API butonu tiklandi."));
      startPhoneApiScreen();
      break;

    case BTN_LOG:
      Serial.println(F("Dahili log butonu tiklandi."));
      // Simdilik sadece yer tutucu, ileride gercek menuyu buradan acacagiz
      showInfoMessage("Dahili Log",
                      "Dahili log menusu",
                      "Henuz uygulanmadi",
                      SCR_SETUP_MENU,
                      1500);
      break;

    case BTN_SAVE_EXIT:
    {
      Serial.println(F("Kaydet ve Cik butonu tiklandi."));
      if (isNormalModeConfigComplete())
      {
        showInfoMessage("Normal Mod",
                        "Normal calismaya geciliyor",
                        "",
                        SCR_IDLE,
                        1200);
      }
      else
      {
        showInfoMessage("Normal Mod",
                        "Eksik ayarlar var",
                        "WiFi/RFID ayarlarini kontrol edin",
                        SCR_SETUP_MENU,
                        2000);
      }
      break;
    }

    case BTN_FACTORY_RESET:
      Serial.println(F("Factory Reset butonu tiklandi."));
      currentScreen = SCR_FACTORY_RESET_CONFIRM;
      drawFactoryResetConfirmScreen();
      break;

    default:
      break;
  }
}

// -----------------------------------------------------------------------------
// Yardımcı: Klavye tuş etiketi
// -----------------------------------------------------------------------------
void setKeyLabel(KeyboardKey &key, const char *text)
{
  uint8_t i = 0;
  for (; i < sizeof(key.label) - 1 && text[i] != '\0'; i++)
  {
    key.label[i] = text[i];
  }
  key.label[i] = '\0';
}

// -----------------------------------------------------------------------------
// Klavye: metin girişi başlat
// -----------------------------------------------------------------------------
void kbStart(const String &title, const String &hint, String *target, uint16_t maxLen,
             uint8_t returnScreenState, TextInputPurpose purpose)
{
  if (!target) return;

  textInput.title        = title;
  textInput.hint         = hint;
  textInput.target       = target;
  textInput.maxLen       = maxLen;
  textInput.returnScreen = (ScreenState)returnScreenState;
  textInput.active       = true;

  textInputPurpose = purpose;

  kbBuffer   = *target;
  kbMaxLen   = maxLen;
  kbCurrentLayout = KB_LAYOUT_UPPER;

  drawTextInputScreen();
  currentScreen = SCR_TEXT_INPUT;
}

// -----------------------------------------------------------------------------
// Metin girişi ekranını çiz
// -----------------------------------------------------------------------------
void drawTextInputScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar(textInput.title.c_str());

  int16_t sw = spr.width();

  int16_t backW = 44;
  int16_t backH = 20;
  int16_t backX = sw - backW - 4;
  int16_t backY = TOP_BAR_H + 4;
  spr.fillRoundRect(backX, backY, backW, backH, 4, TFT_NAVY);
  spr.drawRoundRect(backX, backY, backW, backH, 4, TFT_WHITE);
  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_NAVY);
  spr.drawString("Geri", backX + backW / 2, backY + backH / 2);

  int16_t hintY = backY + backH + 4;
  spr.setTextDatum(TL_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  spr.drawString(textInput.hint, 8, hintY);

  spr.drawRoundRect(8, KB_BOX_Y, sw - 16, KB_BOX_H, 4, TFT_WHITE);

  kbDrawTextLine();
  kbBuildLayout();
  kbDrawKeyboard();

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Metin kutusunu güncelle
// -----------------------------------------------------------------------------
void kbDrawTextLine()
{
  int16_t sw = spr.width();

  spr.fillRect(10, KB_BOX_Y + 2, sw - 20, KB_BOX_H - 4, TFT_BLACK);

  spr.setTextDatum(TL_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  spr.drawString(kbBuffer, 12, KB_BOX_Y + 6);
}

// -----------------------------------------------------------------------------
// Klavye layout'unu oluştur
// -----------------------------------------------------------------------------
void kbBuildLayoutGeneric()
{
  kbKeyCount = 0;

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t kbTop    = KB_TOP_Y;
  int16_t marginX  = 3;
  int16_t marginY  = 4;
  int     rows     = 4;
  int16_t usableH  = sh - kbTop - marginY;
  int16_t rowH     = (usableH - (rows + 1) * marginY) / rows;

  auto addRowChars = [&](const char *chars, uint8_t rowIndex)
  {
    uint8_t len = strlen(chars);
    if (len == 0) return;

    int16_t totalWidth = sw - 2 * marginX;
    int16_t keyW       = totalWidth / len;
    int16_t extra      = totalWidth - keyW * len;
    int16_t startX     = marginX + extra / 2;
    int16_t y          = kbTop + marginY + rowIndex * (rowH + marginY);

    for (uint8_t i = 0; i < len && kbKeyCount < MAX_KEYS; i++)
    {
      KeyboardKey &k = kbKeys[kbKeyCount++];
      k.x    = startX + i * keyW;
      k.y    = y;
      k.w    = keyW - 2;
      k.h    = rowH;
      k.type = KT_CHAR;
      k.value = chars[i];

      k.label[0] = chars[i];
      k.label[1] = '\0';
    }
  };

  const char *uRow1 = "QWERTYUIOP";
  const char *uRow2 = "ASDFGHJKL";
  const char *uRow3 = "ZXCVBNM";

  char row1buf[11];
  char row2buf[10];
  char row3buf[8];

  if (kbCurrentLayout == KB_LAYOUT_UPPER || kbCurrentLayout == KB_LAYOUT_LOWER)
  {
    for (uint8_t i = 0; i < 10; i++)
    {
      char c = uRow1[i];
      row1buf[i] = (kbCurrentLayout == KB_LAYOUT_UPPER) ? toupper(c) : tolower(c);
    }
    row1buf[10] = '\0';

    for (uint8_t i = 0; i < 9; i++)
    {
      char c = uRow2[i];
      row2buf[i] = (kbCurrentLayout == KB_LAYOUT_UPPER) ? toupper(c) : tolower(c);
    }
    row2buf[9] = '\0';

    for (uint8_t i = 0; i < 7; i++)
    {
      char c = uRow3[i];
      row3buf[i] = (kbCurrentLayout == KB_LAYOUT_UPPER) ? toupper(c) : tolower(c);
    }
    row3buf[7] = '\0';

    addRowChars(row1buf, 0);
    addRowChars(row2buf, 1);
    addRowChars(row3buf, 2);
  }
  else
  {
    const char *nRow1 = "1234567890";
    const char *nRow2 = "()-_./+";
    const char *nRow3 = "!@#$%&*?";

    addRowChars(nRow1, 0);
    addRowChars(nRow2, 1);
    addRowChars(nRow3, 2);
  }

  int16_t totalWidth = sw - 2 * marginX;
  int16_t gapX       = marginX;
  int16_t keyW       = (totalWidth - 3 * gapX) / 4;
  int16_t y          = kbTop + marginY + 3 * (rowH + marginY);
  int16_t x          = marginX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_LAYOUT_CYCLE;
    k.value = 0;

    const char *lbl = (kbCurrentLayout == KB_LAYOUT_UPPER) ? "ABC" :
                      (kbCurrentLayout == KB_LAYOUT_LOWER) ? "abc" : "123";
    setKeyLabel(k, lbl);
  }
  x += keyW + gapX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_SPACE;
    k.value = ' ';
    setKeyLabel(k, "SPACE");
  }
  x += keyW + gapX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_BACKSPACE;
    k.value = 0;
    setKeyLabel(k, "DEL");
  }
  x += keyW + gapX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_ENTER;
    k.value = 0;
    setKeyLabel(k, "OK");
  }
}

void kbBuildLayoutDriverPlate()
{
  kbKeyCount = 0;

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t kbTop    = KB_TOP_Y;
  int16_t marginX  = 3;
  int16_t marginY  = 4;
  int     rows     = 5;  // 4 karakter satiri + 1 alt satir
  int16_t usableH  = sh - kbTop - marginY;
  int16_t rowH     = (usableH - (rows + 1) * marginY) / rows;

  auto addRowChars = [&](const char *chars, uint8_t rowIndex)
  {
    uint8_t len = strlen(chars);
    if (len == 0) return;

    int16_t totalWidth = sw - 2 * marginX;
    int16_t keyW       = totalWidth / len;
    int16_t extra      = totalWidth - keyW * len;
    int16_t startX     = marginX + extra / 2;
    int16_t y          = kbTop + marginY + rowIndex * (rowH + marginY);

    for (uint8_t i = 0; i < len && kbKeyCount < MAX_KEYS; i++)
    {
      KeyboardKey &k = kbKeys[kbKeyCount++];
      k.x    = startX + i * keyW;
      k.y    = y;
      k.w    = keyW - 2;
      k.h    = rowH;
      k.type = KT_CHAR;
      k.value = chars[i];

      k.label[0] = chars[i];
      k.label[1] = '\0';
    }
  };

  // Ust satir sayilar
  addRowChars("1234567890", 0);
  // Buyuk QWERTY
  addRowChars("QWERTYUIOP", 1);
  addRowChars("ASDFGHJKL",  2);
  addRowChars("ZXCVBNM",    3);

  // Alt satir: SPACE - DEL - OK (3 genis buton)
  int16_t totalWidth = sw - 2 * marginX;
  int16_t gapX       = marginX;
  int16_t keyW       = (totalWidth - 2 * gapX) / 3;
  int16_t y          = kbTop + marginY + 4 * (rowH + marginY);
  int16_t x          = marginX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_SPACE;
    k.value = ' ';
    setKeyLabel(k, "SPACE");
  }
  x += keyW + gapX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_BACKSPACE;
    k.value = 0;
    setKeyLabel(k, "DEL");
  }
  x += keyW + gapX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type  = KT_ENTER;
    k.value = 0;
    setKeyLabel(k, "OK");
  }
}

void kbBuildLayoutPhone()
{
  kbKeyCount = 0;

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t kbTop    = KB_TOP_Y;
  int16_t marginX  = 3;
  int16_t marginY  = 4;
  int     rows     = 5;  // 4 rakam satiri + 1 alt satir
  int16_t usableH  = sh - kbTop - marginY;
  int16_t rowH     = (usableH - (rows + 1) * marginY) / rows;

  auto addRowChars = [&](const char *chars, uint8_t rowIndex)
  {
    uint8_t len = strlen(chars);
    if (len == 0) return;

    int16_t totalWidth = sw - 2 * marginX;
    int16_t keyW       = totalWidth / len;
    int16_t extra      = totalWidth - keyW * len;
    int16_t startX     = marginX + extra / 2;
    int16_t y          = kbTop + marginY + rowIndex * (rowH + marginY);

    for (uint8_t i = 0; i < len && kbKeyCount < MAX_KEYS; i++)
    {
      KeyboardKey &k = kbKeys[kbKeyCount++];
      k.x    = startX + i * keyW;
      k.y    = y;
      k.w    = keyW - 2;
      k.h    = rowH;
      k.type = KT_CHAR;
      k.value = chars[i];

      k.label[0] = chars[i];
      k.label[1] = '\0';
    }
  };

  // 3 satir rakam + 1 satir 0
  addRowChars("123", 0);
  addRowChars("456", 1);
  addRowChars("789", 2);
  addRowChars("0",   3);

  // Alt satir: sadece DEL ve OK
  int16_t totalWidth = sw - 2 * marginX;
  int16_t gapX       = marginX;
  int16_t keyW       = (totalWidth - gapX) / 2;   // 2 tus, 1 bosluk
  int16_t y          = kbTop + marginY + 4 * (rowH + marginY);
  int16_t x          = marginX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type = KT_BACKSPACE;
    k.value = 0;
    setKeyLabel(k, "DEL");
  }
  x += keyW + gapX;

  if (kbKeyCount < MAX_KEYS)
  {
    KeyboardKey &k = kbKeys[kbKeyCount++];
    k.x = x;
    k.y = y;
    k.w = keyW;
    k.h = rowH;
    k.type = KT_ENTER;
    k.value = 0;
    setKeyLabel(k, "OK");
  }
}

void kbBuildLayout()
{
  // Metin girişinin amacına göre farklı layoutlar
  if (textInputPurpose == TIP_DRIVER_PLATE)
  {
    kbBuildLayoutDriverPlate();   // Plaka: büyük harf + sayi, tek layout
  }
  else if (textInputPurpose == TIP_PHONE_NUMBER)
  {
    kbBuildLayoutPhone();         // Telefon: sadece sayi
  }
  else
  {
    kbBuildLayoutGeneric();       // WiFi sifresi, API key, digerleri: eski klavye
  }
}

// -----------------------------------------------------------------------------
// Klavyedeki tüm tuşları çiz
// -----------------------------------------------------------------------------
void kbDrawKeyboard()
{
  for (uint8_t i = 0; i < kbKeyCount; i++)
  {
    kbDrawKey(i, false);
  }
}

// -----------------------------------------------------------------------------
// Tek bir klavye tuşunu çiz
// -----------------------------------------------------------------------------
void kbDrawKey(uint8_t index, bool pressed)
{
  if (index >= kbKeyCount) return;

  KeyboardKey &k = kbKeys[index];

  uint16_t fillColor   = pressed ? TFT_DARKGREY : TFT_NAVY;
  uint16_t borderColor = TFT_WHITE;
  uint16_t textColor   = TFT_WHITE;

  spr.fillRoundRect(k.x, k.y, k.w, k.h, 4, fillColor);
  spr.drawRoundRect(k.x, k.y, k.w, k.h, 4, borderColor);

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(textColor, fillColor);
  spr.drawString(k.label, k.x + k.w / 2, k.y + k.h / 2);
}

// -----------------------------------------------------------------------------
// Klavye: Hangi tuşa tıklandı?
// -----------------------------------------------------------------------------
int kbHitTestKey(int16_t x, int16_t y)
{
  for (uint8_t i = 0; i < kbKeyCount; i++)
  {
    KeyboardKey &k = kbKeys[i];
    if (x >= k.x && x <= k.x + k.w &&
        y >= k.y && y <= k.y + k.h)
    {
      return i;
    }
  }
  return -1;
}

// -----------------------------------------------------------------------------
// Klavye ekranı için dokunmatik
// -----------------------------------------------------------------------------
void handleKeyboardTouch()
{
  int16_t x, y;
  if (!readTouchPress(x, y)) return;

  int16_t sw = spr.width();

  int16_t backW = 44;
  int16_t backH = 20;
  int16_t backX = sw - backW - 4;
  int16_t backY = TOP_BAR_H + 4;

  if (x >= backX && x <= backX + backW &&
      y >= backY && y <= backY + backH)
  {
    Serial.println(F("Klavye: Geri butonu"));

    if (textInput.active)
    {
      ScreenState ret = textInput.returnScreen;
      textInput.active = false;
      textInputPurpose = TIP_NONE;

      currentScreen = ret;
      if (ret == SCR_SETUP_MENU)
        drawSetupMenu();
      else if (ret == SCR_WIFI_SETTINGS)
        drawWifiSettingsScreen();
      else if (ret == SCR_PHONE_API)
        drawPhoneApiScreen();
      else if (ret == SCR_ADMIN_CARD)
        drawAdminCardScreen(adminLastUid);
      else if (ret == SCR_DRIVER_CARD)
        drawDriverCardScreen(driverScreenInfo);
    }
    return;
  }

  int idx = kbHitTestKey(x, y);
  if (idx >= 0)
  {
    kbProcessKey((uint8_t)idx);
    spr.pushSprite(0, 0);
  }
}

// -----------------------------------------------------------------------------
// Klavye: tuşa basılınca
// -----------------------------------------------------------------------------
void kbProcessKey(uint8_t index)
{
  if (index >= kbKeyCount) return;

  KeyboardKey &k = kbKeys[index];

  kbDrawKey(index, true);
  spr.pushSprite(0, 0);
  delay(80);
  kbDrawKey(index, false);
  spr.pushSprite(0, 0);

  switch (k.type)
  {
    case KT_CHAR:
      if (kbBuffer.length() < kbMaxLen)
      {
        kbBuffer += k.value;
        kbDrawTextLine();
      }
      break;

    case KT_SPACE:
      if (kbBuffer.length() < kbMaxLen)
      {
        kbBuffer += ' ';
        kbDrawTextLine();
      }
      break;

    case KT_BACKSPACE:
      if (kbBuffer.length() > 0)
      {
        kbBuffer.remove(kbBuffer.length() - 1);
        kbDrawTextLine();
      }
      break;

    case KT_ENTER:
    {
      if (!textInput.active || textInput.target == nullptr)
        return;

      *(textInput.target) = kbBuffer;

      Serial.print(F("Klavye girisi tamamlandi: "));
      Serial.println(*(textInput.target));

      if (textInputPurpose == TIP_WIFI_PASSWORD)
      {
        if (wifiSelectedIndex >= 0 && wifiSelectedIndex < wifiScanCount)
        {
          String ssid = wifiScanList[wifiSelectedIndex].ssid;

          int16_t sw = spr.width();
          int16_t sh = spr.height();
          spr.fillSprite(TFT_BLACK);
          drawTopBar("WiFi");
          spr.setTextDatum(MC_DATUM);
          spr.setTextFont(FONT_MAIN);
          spr.setTextColor(TFT_WHITE, TFT_BLACK);
          spr.drawString(ssid, sw / 2, sh / 2 - 10);
          spr.drawString("agina baglaniliyor...", sw / 2, sh / 2 + 10);
          spr.pushSprite(0, 0);

          bool okConn = wifiAttemptConnectBlocking(ssid, kbBuffer);

          textInput.active = false;
          textInputPurpose = TIP_NONE;

          if (okConn)
          {
            config.wifi.ssid     = ssid;
            config.wifi.password = kbBuffer;
            config.wifi.isSet    = true;
            saveConfigToNVS();

            String line1 = ssid;
            String line2 = "Agina baglanildi";
            showInfoMessage("WiFi", line1, line2, SCR_SETUP_MENU, 1500);
          }
          else
          {
            String line1 = ssid;
            String line2 = "Baglanilamadi (sifre/hata)";
            showInfoMessage("WiFi", line1, line2, SCR_WIFI_SETTINGS, 2000);
          }
          return;
        }
        else
        {
          Serial.println(F("Uyari: wifiSelectedIndex gecersiz, WiFi kaydedilemedi."));
          textInput.active = false;
          textInputPurpose = TIP_NONE;
          showInfoMessage("WiFi", "Kayit hatasi", "Gecersiz secim", SCR_WIFI_SETTINGS, 1500);
          return;
        }
      }
      else if (textInputPurpose == TIP_DRIVER_PLATE)
      {
        if (driverCurrentUid.length() > 0)
        {
          bool ok = configAddOrUpdateDriver(driverCurrentUid, kbBuffer, true);
          if (ok)
          {
            driverScreenInfo = "Kaydedildi: " + driverCurrentUid + " -> " + kbBuffer;
            Serial.print(F("Sofor kart kaydedildi: UID="));
            Serial.print(driverCurrentUid);
            Serial.print(F(" Plaka="));
            Serial.println(kbBuffer);

            textInput.active = false;
            textInputPurpose = TIP_NONE;

            String line1 = "Sofor kart kaydedildi";
            String line2 = driverCurrentUid + " / " + kbBuffer;
            driverCurrentUid = "";
            showInfoMessage("Sofor Kart", line1, line2, SCR_SETUP_MENU, 1500);
            return;
          }
          else
          {
            driverScreenInfo = "HATA: Liste dolu!";
            Serial.println(F("Sofor kart kayit hatasi: liste dolu."));

            textInput.active = false;
            textInputPurpose = TIP_NONE;
            showInfoMessage("Sofor Kart", "Kayit hatasi", "Liste dolu", SCR_DRIVER_MENU, 1500);
            return;
          }
        }
        else
        {
          driverScreenInfo = "HATA: UID yok!";
          Serial.println(F("Uyari: driverCurrentUid bos, plaka kaydedilemedi."));

          textInput.active = false;
          textInputPurpose = TIP_NONE;
          showInfoMessage("Sofor Kart", "Kayit hatasi", "UID yok", SCR_DRIVER_MENU, 1500);
          return;
        }
      }

      {
        ScreenState ret = textInput.returnScreen;
        textInput.active = false;
        textInputPurpose = TIP_NONE;

        currentScreen = ret;
        if (ret == SCR_SETUP_MENU)
          drawSetupMenu();
        else if (ret == SCR_WIFI_SETTINGS)
          drawWifiSettingsScreen();
        else if (ret == SCR_PHONE_API)
          drawPhoneApiScreen();
        else if (ret == SCR_ADMIN_CARD)
          drawAdminCardScreen(adminLastUid);
        else if (ret == SCR_DRIVER_CARD)
          drawDriverCardScreen(driverScreenInfo);
      }
      return;
    }

    case KT_LAYOUT_CYCLE:
      kbCurrentLayout = (KeyboardLayout)((kbCurrentLayout + 1) % 3);
      kbBuildLayout();
      kbDrawKeyboard();
      break;
  }

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// WiFi Ayarları: ekranı başlat
// -----------------------------------------------------------------------------
void startWifiSettingsScreen()
{
  currentScreen = SCR_WIFI_SETTINGS;

  spr.fillSprite(TFT_BLACK);
  drawTopBar("WiFi Ayarlari");
  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.drawString("WiFi aglari taraniyor...", spr.width() / 2, spr.height() / 2);
  spr.pushSprite(0, 0);

  wifiScanNetworks();
  drawWifiSettingsScreen();
}

// -----------------------------------------------------------------------------
// WiFi Ayarları: ağları tara
// -----------------------------------------------------------------------------
void wifiScanNetworks()
{
  Serial.println(F("WiFi taramasi basliyor..."));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  int n = WiFi.scanNetworks();
  if (n <= 0)
  {
    wifiScanCount = 0;
    Serial.println(F("WiFi ag bulunamadi."));
  }
  else
  {
    wifiScanCount = (n > WIFI_MAX_NETWORKS) ? WIFI_MAX_NETWORKS : n;
    for (int i = 0; i < wifiScanCount; i++)
    {
      wifiScanList[i].ssid   = WiFi.SSID(i);
      wifiScanList[i].rssi   = WiFi.RSSI(i);
      wifiScanList[i].secure = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    Serial.printf("Toplam %d ag bulundu (gosterilen: %d)\n", n, wifiScanCount);
  }

  wifiListFirstIndex = 0;
  wifiSelectedIndex  = -1;

  WiFi.scanDelete();
}

// -----------------------------------------------------------------------------
// WiFi Ayarları: ekranı çiz
// -----------------------------------------------------------------------------
void drawWifiSettingsScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar("WiFi Ayarlari");

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t hintY = TOP_BAR_H + 4;
  spr.setTextDatum(TL_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  if (wifiScanCount == 0)
    spr.drawString("Ag bulunamadi. 'Tara' ile yenile.", 8, hintY);
  else
    spr.drawString("Bir ag secin, sifreyi girin.", 8, hintY);

  drawWifiNetworksList();

  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t margin = 6;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW   = (sw - margin * 5) / 4;

  int16_t backX = margin;
  int16_t scanX = backX + btnW + margin;
  int16_t upX   = scanX + btnW + margin;
  int16_t downX = upX + btnW + margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  spr.fillRoundRect(backX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(backX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Geri", backX + btnW / 2, btnY + btnH / 2);

  spr.fillRoundRect(scanX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(scanX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.drawString("Tara", scanX + btnW / 2, btnY + btnH / 2);

  spr.fillRoundRect(upX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(upX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.drawString("Yukari", upX + btnW / 2, btnY + btnH / 2);

  spr.fillRoundRect(downX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(downX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.drawString("Asagi", downX + btnW / 2, btnY + btnH / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// WiFi Ayarları: liste kısmını çiz
// -----------------------------------------------------------------------------
void drawWifiNetworksList()
{
  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t listTop    = WIFI_LIST_TOP;
  int16_t listBottom = sh - BOTTOM_BAR_H - 4;
  int16_t listH      = listBottom - listTop;
  if (listH <= 0) return;

  int16_t rowH = listH / WIFI_LIST_ROWS;

  spr.fillRect(0, listTop, sw, listH, TFT_BLACK);

  spr.setTextFont(FONT_MAIN);
  spr.setTextDatum(TL_DATUM);

  for (int row = 0; row < WIFI_LIST_ROWS; row++)
  {
    int idx = wifiListFirstIndex + row;
    int16_t y = listTop + row * rowH;

    if (idx >= wifiScanCount) continue;

    bool selected = (idx == wifiSelectedIndex);
    uint16_t bg = selected ? TFT_DARKCYAN : TFT_NAVY;

    spr.fillRoundRect(4, y + 2, sw - 8, rowH - 4, 4, bg);
    spr.setTextColor(TFT_WHITE, bg);

    String line = wifiScanList[idx].ssid;
    if (line.length() == 0) line = "<ssid yok>";

    line += "  ";
    line += String(wifiScanList[idx].rssi);
    line += "dBm";
    if (wifiScanList[idx].secure) line += " *";

    spr.drawString(line, 8, y + 4);
  }
}

// -----------------------------------------------------------------------------
// WiFi Ayarları: dokunmatik event
// -----------------------------------------------------------------------------
void handleTouchOnWifiSettings()
{
  int16_t x, y;
  if (!readTouchPress(x, y)) return;

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t bottomY = sh - BOTTOM_BAR_H;
  int16_t margin  = 6;
  int16_t btnH    = BOTTOM_BAR_H - 16;
  int16_t btnY    = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW    = (sw - margin * 5) / 4;

  int16_t backX = margin;
  int16_t scanX = backX + btnW + margin;
  int16_t upX   = scanX + btnW + margin;
  int16_t downX = upX + btnW + margin;

  if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H)
  {
    if (x >= backX && x <= backX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      Serial.println(F("WiFi: Geri butonu"));
      currentScreen = SCR_SETUP_MENU;
      drawSetupMenu();
      return;
    }

    if (x >= scanX && x <= scanX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      Serial.println(F("WiFi: Yeniden tarama"));
      startWifiSettingsScreen();
      return;
    }

    if (x >= upX && x <= upX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      if (wifiListFirstIndex > 0)
      {
        wifiListFirstIndex--;
        drawWifiNetworksList();
        spr.pushSprite(0, 0);
      }
      return;
    }

    if (x >= downX && x <= downX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      if (wifiListFirstIndex + WIFI_LIST_ROWS < wifiScanCount)
      {
        wifiListFirstIndex++;
        drawWifiNetworksList();
        spr.pushSprite(0, 0);
      }
      return;
    }
  }

  int16_t listTop    = WIFI_LIST_TOP;
  int16_t listBottom = sh - BOTTOM_BAR_H - 4;
  int16_t listH      = listBottom - listTop;
  if (listH <= 0) return;
  int16_t rowH = listH / WIFI_LIST_ROWS;

  if (y >= listTop && y < listBottom)
  {
    int row = (y - listTop) / rowH;
    int idx = wifiListFirstIndex + row;
    if (idx < wifiScanCount)
    {
      wifiSelectedIndex = idx;
      drawWifiNetworksList();
      spr.pushSprite(0, 0);

      Serial.print(F("WiFi ag secildi: "));
      Serial.println(wifiScanList[idx].ssid);

      delay(120);
      wifiOpenPasswordInput(idx);
    }
  }
}

// -----------------------------------------------------------------------------
// WiFi Ayarları: şifre girişi başlat
// -----------------------------------------------------------------------------
void wifiOpenPasswordInput(int index)
{
  if (index < 0 || index >= wifiScanCount) return;

  wifiPasswordBuffer = "";

  String title = "WiFi Sifresi";
  String hint  = "Ag: " + wifiScanList[index].ssid;

  kbStart(title, hint, &wifiPasswordBuffer, 64,
          SCR_WIFI_SETTINGS, TIP_WIFI_PASSWORD);
}

// -----------------------------------------------------------------------------
// Telefon / API ekranini baslat
// -----------------------------------------------------------------------------
void startPhoneApiScreen()
{
  String rawPhone = config.phoneApi.phoneNumber;
  if (rawPhone.startsWith("+"))
  {
    rawPhone.remove(0, 1);
  }
  phoneEditBuffer   = rawPhone;
  apiKeyEditBuffer  = config.phoneApi.apiKey;

  currentScreen = SCR_PHONE_API;
  drawPhoneApiScreen();
}

// -----------------------------------------------------------------------------
// Telefon / API ekranı çizimi
// -----------------------------------------------------------------------------
void drawPhoneApiScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar("Telefon / API");

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t margin = 10;
  int16_t fieldH = 50;
  int16_t phoneY = TOP_BAR_H + 6;
  int16_t apiY   = phoneY + fieldH + 10;

  spr.setTextDatum(TL_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  spr.drawString("Telefon Numarasi", margin, phoneY);

  spr.drawRoundRect(margin, phoneY + 10, sw - 2 * margin, fieldH - 14, 6, TFT_WHITE);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  String phoneText;
  if (phoneEditBuffer.length())
    phoneText = "+" + phoneEditBuffer;
  else
    phoneText = "<ayarlanmadi>";

  spr.drawString(phoneText, margin + 6, phoneY + 20);

  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  spr.drawString("CallMeBot API Key", margin, apiY);

  spr.drawRoundRect(margin, apiY + 10, sw - 2 * margin, fieldH - 14, 6, TFT_WHITE);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  String apiText = apiKeyEditBuffer.length() ? apiKeyEditBuffer : String("<ayarlanmadi>");
  spr.drawString(apiText, margin + 6, apiY + 20);

  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW   = (sw - 3 * margin) / 2;

  int16_t backX  = margin;
  int16_t saveX  = backX + btnW + margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  spr.fillRoundRect(backX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(backX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Geri", backX + btnW / 2, btnY + btnH / 2);

  spr.fillRoundRect(saveX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(saveX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.drawString("Kaydet", saveX + btnW / 2, btnY + btnH / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Telefon / API ekranı dokunmatik
// -----------------------------------------------------------------------------
void handleTouchOnPhoneApi()
{
  int16_t x, y;
  if (!readTouchPress(x, y)) return;

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t margin = 10;
  int16_t fieldH = 50;
  int16_t phoneY = TOP_BAR_H + 6;
  int16_t apiY   = phoneY + fieldH + 10;

  int16_t phoneBoxX = margin;
  int16_t phoneBoxY = phoneY + 10;
  int16_t phoneBoxW = sw - 2 * margin;
  int16_t phoneBoxH = fieldH - 14;

  int16_t apiBoxX = margin;
  int16_t apiBoxY = apiY + 10;
  int16_t apiBoxW = sw - 2 * margin;
  int16_t apiBoxH = fieldH - 14;

  int16_t bottomY = sh - BOTTOM_BAR_H;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW   = (sw - 3 * margin) / 2;

  int16_t backX  = margin;
  int16_t saveX  = backX + btnW + margin;

  if (x >= phoneBoxX && x <= phoneBoxX + phoneBoxW &&
      y >= phoneBoxY && y <= phoneBoxY + phoneBoxH)
  {
    Serial.println(F("Telefon alani tiklandi - klavye aciliyor"));
    kbStart("Telefon",
        "+ olmadan ulke kodu ile beraber girin 90555...",
        &phoneEditBuffer,
        20,
        SCR_PHONE_API,
        TIP_PHONE_NUMBER);

    return;
  }

  if (x >= apiBoxX && x <= apiBoxX + apiBoxW &&
      y >= apiBoxY && y <= apiBoxY + apiBoxH)
  {
    Serial.println(F("API Key alani tiklandi - klavye aciliyor"));
    kbStart("API Key",
            "CallMeBot API key",
            &apiKeyEditBuffer,
            64,
            SCR_PHONE_API,
            TIP_GENERIC);
    return;
  }

  if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H)
  {
    if (x >= backX && x <= backX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      Serial.println(F("Telefon/API: Geri"));
      currentScreen = SCR_SETUP_MENU;
      drawSetupMenu();
      return;
    }

    if (x >= saveX && x <= saveX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      Serial.println(F("Telefon/API: Kaydet"));

      String finalPhone = phoneEditBuffer;
      if (finalPhone.length() > 0)
      {
        if (!finalPhone.startsWith("+"))
          finalPhone = "+" + finalPhone;
      }
      else
      {
        finalPhone = "";
      }

      configSetPhoneApi(finalPhone, apiKeyEditBuffer, true);

      Serial.print(F("Telefon (kayitli): "));
      Serial.println(config.phoneApi.phoneNumber);
      Serial.print(F("API Key: "));
      Serial.println(config.phoneApi.apiKey);

      String line1 = "Telefon/API kaydedildi";
      String line2 = finalPhone.length() ? finalPhone : String("Numara yok");
      showInfoMessage("Telefon/API", line1, line2, SCR_SETUP_MENU, 1500);
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// RFID Yardimci: UID'yi hex string'e çevir (AA:BB:CC:DD)
// -----------------------------------------------------------------------------
String uidToHexString(const MFRC522::Uid &uid)
{
  String s;
  for (byte i = 0; i < uid.size; i++)
  {
    if (i > 0) s += ":";
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

// -----------------------------------------------------------------------------
// Admin Kart ekranini baslat
// -----------------------------------------------------------------------------
void startAdminCardScreen()
{
  adminLastUid = "";
  currentScreen = SCR_ADMIN_CARD;
  drawAdminCardScreen(adminLastUid);
}

// -----------------------------------------------------------------------------
// Admin Kart ekranı çizimi
// -----------------------------------------------------------------------------
void drawAdminCardScreen(const String &uidHex)
{
  (void)uidHex; // artik parametreyi kullanmiyoruz

  spr.fillSprite(TFT_BLACK);
  drawTopBar("Yonetici RFID");

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  // Mevcut yonetici kart bilgisini goster
  String currentUid;
  if (config.adminCard.isSet && config.adminCard.uidHex.length() > 0)
    currentUid = config.adminCard.uidHex;
  else
    currentUid = "Tanimlanmadi";

  spr.drawString("Yonetici kart okutun", sw / 2, centerY - 10);

  String line2 = "Mevcut yonetici karti: " + currentUid;
  spr.drawString(line2, sw / 2, centerY + 10);

  // Alt tarafta Geri butonu
  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t margin = 10;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW   = sw - 2 * margin;
  int16_t backX  = margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  spr.fillRoundRect(backX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(backX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Geri", backX + btnW / 2, btnY + btnH / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Admin Kart ekranı: dokunmatik + RFID okuma
// -----------------------------------------------------------------------------
void handleTouchOnAdminCard()
{
  int16_t x, y;
  if (readTouchPress(x, y))
  {
    int16_t sw = spr.width();
    int16_t sh = spr.height();
    int16_t bottomY = sh - BOTTOM_BAR_H;
    int16_t margin  = 10;
    int16_t btnH    = BOTTOM_BAR_H - 16;
    int16_t btnY    = bottomY + (BOTTOM_BAR_H - btnH) / 2;
    int16_t btnW    = sw - 2 * margin;
    int16_t backX   = margin;

    if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H &&
        x >= backX && x <= backX + btnW &&
        y >= btnY  && y <= btnY + btnH)
    {
      Serial.println(F("Admin Kart: Geri"));
      currentScreen = SCR_DRIVER_MENU;
      drawDriverMenuScreen();
      return;
    }
  }

  spiUseRFID();
  if (!mfrc522.PICC_IsNewCardPresent())
  {
    spiUseTFT();
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial())
  {
    spiUseTFT();
    return;
  }

  String uidHex = uidToHexString(mfrc522.uid);

  Serial.print(F("Admin kart okundu, UID = "));
  Serial.println(uidHex);

  configSetAdminCard(uidHex, true);
  adminLastUid = uidHex;

  Serial.println(F("Admin kart NVS'ye kaydedildi."));

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  spiUseTFT();

  String line1 = "Yeni yonetici karti:";
  String line2 = uidHex;
  // Islemi bitirince tekrar RFID menusu'ne don
  showInfoMessage("Yonetici RFID", line1, line2, SCR_DRIVER_MENU, 1500);
}

// -----------------------------------------------------------------------------
// Sofor alt menü ekranini baslat
// -----------------------------------------------------------------------------
void startDriverMenuScreen()
{
  currentScreen = SCR_DRIVER_MENU;
  drawDriverMenuScreen();
}

// -----------------------------------------------------------------------------
// Sofor alt menü çiz
// -----------------------------------------------------------------------------
void drawDriverMenuScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar("RFID Ayarlari");

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t margin = 10;
  int16_t btnH   = 44;
  int16_t space  = 10;

  int16_t areaTop    = TOP_BAR_H;
  int16_t areaBottom = sh - BOTTOM_BAR_H;
  int16_t areaH      = areaBottom - areaTop;

  int16_t totalButtonsH = btnH * 3 + space * 2;
  int16_t startY = areaTop + (areaH - totalButtonsH) / 2;

  driverMenuNewBtn.x = margin;
  driverMenuNewBtn.y = startY;
  driverMenuNewBtn.w = sw - 2 * margin;
  driverMenuNewBtn.h = btnH;

  driverMenuAdminBtn.x = margin;
  driverMenuAdminBtn.y = startY + btnH + space;
  driverMenuAdminBtn.w = sw - 2 * margin;
  driverMenuAdminBtn.h = btnH;

  driverMenuListBtn.x = margin;
  driverMenuListBtn.y = startY + (btnH + space) * 2;
  driverMenuListBtn.w = sw - 2 * margin;
  driverMenuListBtn.h = btnH;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  // 1) Yeni sofor kart / plaka
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.fillRoundRect(driverMenuNewBtn.x, driverMenuNewBtn.y,
                    driverMenuNewBtn.w, driverMenuNewBtn.h,
                    6, TFT_BLUE);
  spr.drawRoundRect(driverMenuNewBtn.x, driverMenuNewBtn.y,
                    driverMenuNewBtn.w, driverMenuNewBtn.h,
                    6, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Yeni sofor RFID / plaka",
                 driverMenuNewBtn.x + driverMenuNewBtn.w / 2,
                 driverMenuNewBtn.y + driverMenuNewBtn.h / 2);

  // 2) Yonetici kart
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.fillRoundRect(driverMenuAdminBtn.x, driverMenuAdminBtn.y,
                    driverMenuAdminBtn.w, driverMenuAdminBtn.h,
                    6, TFT_BLUE);
  spr.drawRoundRect(driverMenuAdminBtn.x, driverMenuAdminBtn.y,
                    driverMenuAdminBtn.w, driverMenuAdminBtn.h,
                    6, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Yonetici RFID tanimla/degistir",
                 driverMenuAdminBtn.x + driverMenuAdminBtn.w / 2,
                 driverMenuAdminBtn.y + driverMenuAdminBtn.h / 2);

  // 3) Kayitli kartlar
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.fillRoundRect(driverMenuListBtn.x, driverMenuListBtn.y,
                    driverMenuListBtn.w, driverMenuListBtn.h,
                    6, TFT_BLUE);
  spr.drawRoundRect(driverMenuListBtn.x, driverMenuListBtn.y,
                    driverMenuListBtn.w, driverMenuListBtn.h,
                    6, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Kayitli RFID ve plakalar",
                 driverMenuListBtn.x + driverMenuListBtn.w / 2,
                 driverMenuListBtn.y + driverMenuListBtn.h / 2);

  // Alt bar: Geri
  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t btnH2 = BOTTOM_BAR_H - 16;
  int16_t btnY2 = bottomY + (BOTTOM_BAR_H - btnH2) / 2;
  int16_t btnW2 = sw - 2 * margin;
  int16_t backX = margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.fillRoundRect(backX, btnY2, btnW2, btnH2, 5, TFT_BLUE);
  spr.drawRoundRect(backX, btnY2, btnW2, btnH2, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Geri", backX + btnW2 / 2, btnY2 + btnH2 / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Sofor alt menü dokunmatik
// -----------------------------------------------------------------------------
void handleTouchOnDriverMenu()
{
  int16_t x, y;
  if (!readTouchPress(x, y)) return;

  int16_t sw = spr.width();
  int16_t sh = spr.height();
  int16_t bottomY = sh - BOTTOM_BAR_H;
  int16_t margin  = 10;
  int16_t btnH2   = BOTTOM_BAR_H - 16;
  int16_t btnY2   = bottomY + (BOTTOM_BAR_H - btnH2) / 2;
  int16_t btnW2   = sw - 2 * margin;
  int16_t backX   = margin;

  // Alt bardaki Geri
  if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H &&
      x >= backX && x <= backX + btnW2 &&
      y >= btnY2 && y <= btnY2 + btnH2)
  {
    Serial.println(F("RFID Menu: Geri"));
    currentScreen = SCR_SETUP_MENU;
    drawSetupMenu();
    return;
  }

  // Yeni sofor kart/plaka
  if (x >= driverMenuNewBtn.x && x <= driverMenuNewBtn.x + driverMenuNewBtn.w &&
      y >= driverMenuNewBtn.y && y <= driverMenuNewBtn.y + driverMenuNewBtn.h)
  {
    Serial.println(F("RFID Menu: Yeni sofor kart/plaka"));
    startDriverCardScreen();
    return;
  }

  // Yonetici RFID
  if (x >= driverMenuAdminBtn.x && x <= driverMenuAdminBtn.x + driverMenuAdminBtn.w &&
      y >= driverMenuAdminBtn.y && y <= driverMenuAdminBtn.y + driverMenuAdminBtn.h)
  {
    Serial.println(F("RFID Menu: Yonetici RFID tanimla/degistir"));
    startAdminCardScreen();
    return;
  }

  // Kayitli kartlar
  if (x >= driverMenuListBtn.x && x <= driverMenuListBtn.x + driverMenuListBtn.w &&
      y >= driverMenuListBtn.y && y <= driverMenuListBtn.y + driverMenuListBtn.h)
  {
    Serial.println(F("RFID Menu: Kayitli kartlar"));
    startDriverListScreen();
    return;
  }
}

// -----------------------------------------------------------------------------
// Sofor Kart / Plaka (yeni kart) ekranini baslat
// -----------------------------------------------------------------------------
void startDriverCardScreen()
{
  driverCurrentUid.clear();
  driverPlateBuffer.clear();
  driverScreenInfo = "Sofor kartinizi okutun";
  currentScreen = SCR_DRIVER_CARD;
  drawDriverCardScreen(driverScreenInfo);
}

// -----------------------------------------------------------------------------
// Sofor Kart / Plaka (yeni kart) ekranı çizimi
// -----------------------------------------------------------------------------
void drawDriverCardScreen(const String &infoLine)
{
  spr.fillSprite(TFT_BLACK);
    drawTopBar(getScreenTitle(SCR_DRIVER_CARD));

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  if (infoLine.length() == 0)
  {
    spr.drawString("Sofor kartinizi", sw / 2, centerY - 10);
    spr.drawString("okutun",          sw / 2, centerY + 10);
  }
  else
  {
    spr.drawString(infoLine, sw / 2, centerY);
  }

  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t margin = 10;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW   = sw - 2 * margin;
  int16_t backX  = margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  spr.fillRoundRect(backX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(backX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Geri", backX + btnW / 2, btnY + btnH / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Sofor Kart / Plaka (yeni kart) ekranı: dokunmatik + RFID
// -----------------------------------------------------------------------------
void handleTouchOnDriverCard()
{
  int16_t x, y;
  if (readTouchPress(x, y))
  {
    int16_t sw = spr.width();
    int16_t sh = spr.height();
    int16_t bottomY = sh - BOTTOM_BAR_H;
    int16_t margin  = 10;
    int16_t btnH    = BOTTOM_BAR_H - 16;
    int16_t btnY    = bottomY + (BOTTOM_BAR_H - btnH) / 2;
    int16_t btnW    = sw - 2 * margin;
    int16_t backX   = margin;

    if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H &&
        x >= backX && x <= backX + btnW &&
        y >= btnY  && y <= btnY + btnH)
    {
      Serial.println(F("Sofor Kart: Geri"));
      currentScreen = SCR_DRIVER_MENU;
      drawDriverMenuScreen();
      return;
    }
  }

  if (driverCurrentUid.length() == 0)
  {
    spiUseRFID();
    if (!mfrc522.PICC_IsNewCardPresent())
    {
      spiUseTFT();
      return;
    }
    if (!mfrc522.PICC_ReadCardSerial())
    {
      spiUseTFT();
      return;
    }

    driverCurrentUid = uidToHexString(mfrc522.uid);

    Serial.print(F("Sofor kart okundu, UID = "));
    Serial.println(driverCurrentUid);

    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();

    spiUseTFT();

    driverPlateBuffer = "";
    String hint = "Kart: " + driverCurrentUid;
    kbStart("Plaka", hint, &driverPlateBuffer, 16,
            SCR_DRIVER_CARD, TIP_DRIVER_PLATE);
  }
}

// -----------------------------------------------------------------------------
// Kayitli kartlar ekranini baslat
// -----------------------------------------------------------------------------
void startDriverListScreen()
{
  driverListFirstIndex = 0;
  currentScreen = SCR_DRIVER_LIST;
  drawDriverListScreen();
}

// -----------------------------------------------------------------------------
// Kayitli kartlar ekranı çizimi
// -----------------------------------------------------------------------------
void drawDriverListScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar("Kayitli RFID ve Plakalar");

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(TL_DATUM);
  spr.setTextFont(FONT_MAIN);

  int16_t headerY = TOP_BAR_H + 4;

  if (config.drivers.count == 0)
  {
    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.drawString("Kayitli kart yok.", 8, headerY);
  }
  else
  {
    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.drawString("UID  ->  Plaka", 8, headerY);

    int16_t listTop    = headerY + 16;
    int16_t listBottom = sh - BOTTOM_BAR_H - 4;
    int16_t y          = listTop;

    spr.setTextColor(TFT_WHITE, TFT_BLACK);

    for (int i = driverListFirstIndex; i < config.drivers.count; i++)
    {
      if (y > listBottom - DRIVER_LIST_ROW_H) break;

      String line = config.drivers.items[i].uidHex + "  " +
                    config.drivers.items[i].plate;
      spr.drawString(line, 8, y);
      y += DRIVER_LIST_ROW_H;
    }
  }

  // Alt bar: Geri + Yukari / Asagi
  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t margin = 6;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;

  int16_t btnW   = (sw - margin * 4) / 3;
  int16_t backX  = margin;
  int16_t upX    = backX + btnW + margin;
  int16_t downX  = upX + btnW + margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  // Geri
  spr.fillRoundRect(backX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(backX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Geri", backX + btnW / 2, btnY + btnH / 2);

  // Yukari
  spr.fillRoundRect(upX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(upX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.drawString("Yukari", upX + btnW / 2, btnY + btnH / 2);

  // Asagi
  spr.fillRoundRect(downX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(downX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.drawString("Asagi", downX + btnW / 2, btnY + btnH / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Kayitli kartlar ekranı dokunmatik
// -----------------------------------------------------------------------------
void handleTouchOnDriverList()
{
  int16_t x, y;
  if (!readTouchPress(x, y)) return;

  int16_t sw = spr.width();
  int16_t sh = spr.height();
  int16_t bottomY = sh - BOTTOM_BAR_H;

  int16_t margin = 6;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;

  int16_t btnW   = (sw - margin * 4) / 3;
  int16_t backX  = margin;
  int16_t upX    = backX + btnW + margin;
  int16_t downX  = upX + btnW + margin;

  if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H)
  {
    // Geri
    if (x >= backX && x <= backX + btnW &&
        y >= btnY  && y <= btnY + btnH)
    {
      Serial.println(F("Kayitli kartlar: Geri"));
      currentScreen = SCR_DRIVER_MENU;
      drawDriverMenuScreen();
      return;
    }

    // Yukari
    if (x >= upX && x <= upX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      if (driverListFirstIndex > 0)
      {
        driverListFirstIndex--;
        drawDriverListScreen();
      }
      return;
    }

    // Asagi
    if (x >= downX && x <= downX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      if (config.drivers.count > 0)
      {
        int16_t headerY    = TOP_BAR_H + 4;
        int16_t listTop    = headerY + 16;
        int16_t listBottom = sh - BOTTOM_BAR_H - 4;
        int    visibleRows = (listBottom - listTop) / DRIVER_LIST_ROW_H;
        if (visibleRows < 1) visibleRows = 1;

        if (driverListFirstIndex + visibleRows < config.drivers.count)
        {
          driverListFirstIndex++;
          drawDriverListScreen();
        }
      }
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// Factory Reset Ekrani
// -----------------------------------------------------------------------------
void drawFactoryResetConfirmScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar("Factory Reset");

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_RED, TFT_BLACK);
  spr.drawString("Tum ayarlar silinecek!", sw / 2, TOP_BAR_H + 40);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);
  spr.drawString("Devam etmek istiyor musunuz?", sw / 2, TOP_BAR_H + 65);

  int16_t bottomY = sh - BOTTOM_BAR_H;
  spr.fillRect(0, bottomY, sw, BOTTOM_BAR_H, TFT_BLACK);
  spr.drawLine(0, bottomY, sw, bottomY, TFT_DARKGREY);

  int16_t margin = 10;
  int16_t btnH   = BOTTOM_BAR_H - 16;
  int16_t btnY   = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW   = (sw - 3 * margin) / 2;
  int16_t cancelX = margin;
  int16_t resetX  = cancelX + btnW + margin;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);

  spr.fillRoundRect(cancelX, btnY, btnW, btnH, 5, TFT_BLUE);
  spr.drawRoundRect(cancelX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_BLUE);
  spr.drawString("Iptal", cancelX + btnW / 2, btnY + btnH / 2);

  spr.fillRoundRect(resetX, btnY, btnW, btnH, 5, TFT_RED);
  spr.drawRoundRect(resetX, btnY, btnW, btnH, 5, TFT_WHITE);
  spr.setTextColor(TFT_WHITE, TFT_RED);
  spr.drawString("Sifirla", resetX + btnW / 2, btnY + btnH / 2);

  spr.pushSprite(0, 0);
}

// -----------------------------------------------------------------------------
// Factory Reset ekranı: dokunmatik
// -----------------------------------------------------------------------------
void handleTouchOnFactoryResetConfirm()
{
  int16_t x, y;
  if (!readTouchPress(x, y)) return;

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  int16_t bottomY = sh - BOTTOM_BAR_H;
  int16_t margin  = 10;
  int16_t btnH    = BOTTOM_BAR_H - 16;
  int16_t btnY    = bottomY + (BOTTOM_BAR_H - btnH) / 2;
  int16_t btnW    = (sw - 3 * margin) / 2;
  int16_t cancelX = margin;
  int16_t resetX  = cancelX + btnW + margin;

  if (y >= bottomY && y <= bottomY + BOTTOM_BAR_H)
  {
    if (x >= cancelX && x <= cancelX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      Serial.println(F("Factory Reset: Iptal"));
      currentScreen = SCR_SETUP_MENU;
      drawSetupMenu();
      return;
    }

    if (x >= resetX && x <= resetX + btnW &&
        y >= btnY && y <= btnY + btnH)
    {
      Serial.println(F("Factory Reset: Onaylandi"));
      doFactoryReset();
      return;
    }
  }
}

// -----------------------------------------------------------------------------
// Factory Reset uygulama
// -----------------------------------------------------------------------------
void doFactoryReset()
{
  Serial.println(F("FACTORY RESET: NVS siliniyor ve yeniden baslatiliyor..."));

  WiFi.disconnect(true, true);
  esp_err_t err = nvs_flash_erase();
  if (err != ESP_OK)
  {
    Serial.printf("nvs_flash_erase hata: %d\n", (int)err);
  }
  delay(500);
  ESP.restart();
}

// -----------------------------------------------------------------------------
// Normal Calisma: IDLE / FUELING / SUMMARY
// -----------------------------------------------------------------------------
void drawIdleScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar(getScreenTitle(SCR_IDLE));

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  // Yaziyi buyuttuk ve ortaladık
  spr.setTextSize(2);
  spr.drawString("Sofor kartini okutarak",   sw / 2, centerY - 16);
  spr.drawString("dolumu baslatabilirsiniz.", sw / 2, centerY + 16);
  spr.setTextSize(1);

  spr.pushSprite(0, 0);
}

void handleTouchOnIdle()
{
  // Simdilik dokunus ile bir sey yapmiyoruz.
}

void drawFuelingScreen(const MeterData &md)
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar(getScreenTitle(SCR_FUELING));

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  float sessionLiters = md.sessionVolCl / 100.0f;
  float flowLpm       = md.flowRateClm  / 100.0f;

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  char buf[32];

  // Plaka / Toplam / Debi ortada ve buyuk
  spr.setTextSize(2);

  String line1 = "Plaka: " + g_activeDriverPlate;
  spr.drawString(line1, sw / 2, centerY - 24);

  snprintf(buf, sizeof(buf), "Toplam: %.2f L", sessionLiters);
  spr.drawString(buf, sw / 2, centerY);

  snprintf(buf, sizeof(buf), "Debi: %.2f L/dk", flowLpm);
  spr.drawString(buf, sw / 2, centerY + 24);

  spr.setTextSize(1);

  spr.pushSprite(0, 0);
}

void handleTouchOnFueling()
{
  // İstersek buraya "iptal" vb. koyabiliriz, simdilik bos.
}

void drawFuelSummaryScreen()
{
  spr.fillSprite(TFT_BLACK);
  drawTopBar(getScreenTitle(SCR_FUEL_SUMMARY));

  int16_t sw = spr.width();
  int16_t sh = spr.height();

  spr.setTextDatum(MC_DATUM);
  spr.setTextFont(FONT_MAIN);
  spr.setTextColor(TFT_WHITE, TFT_BLACK);

  int16_t centerY = TOP_BAR_H + (sh - TOP_BAR_H - BOTTOM_BAR_H) / 2;

  char buf[32];

  // Dolum bitti + plaka + toplam, tek ekranda, buyuk
  spr.setTextSize(2);
  spr.drawString("DOLUM BITTI", sw / 2, centerY - 32);

  String line1 = "Plaka: " + g_activeDriverPlate;
  spr.drawString(line1, sw / 2, centerY);

  snprintf(buf, sizeof(buf), "Toplam: %.2f L", g_lastSessionLiters);
  spr.drawString(buf, sw / 2, centerY + 32);

  spr.setTextSize(1);

  spr.pushSprite(0, 0);
}

void handleTouchOnFuelSummary()
{
  // Artık dokunmatik beklemiyoruz. Zaman dolunca otomatik IDLE'a don.
  unsigned long now = millis();
  if (now - g_fuelSummaryStartMs >= FUEL_SUMMARY_DISPLAY_MS)
  {
    currentScreen = SCR_IDLE;
    drawIdleScreen();
  }
}

// Modbus'i periyodik poll eden fonksiyon
void handleMeterPolling()
{
  if (currentScreen != SCR_FUELING) return;

  unsigned long now = millis();
  if (now - g_lastMeterPollMs < METER_POLL_INTERVAL_MS) return;
  g_lastMeterPollMs = now;

  MeterData md;
  if (!meterRead(md)) {
    Serial.println(F("meterRead hata"));
    return;
  }

  g_lastMeter = md;
  float sessionLiters = md.sessionVolCl / 100.0f;
  g_lastSessionLiters = sessionLiters;

  drawFuelingScreen(md);

  bool active = (md.statusFlags & STATUS_SESSION_ACTIVE_BIT) != 0;

  // Oturum yeni bitti mi?
  if (g_sessionActive && !active)
  {
    g_sessionActive = false;

    // Dolum bitti: tek bir ozet ekrani goster, sonra otomatik IDLE'a don
    currentScreen = SCR_FUEL_SUMMARY;
    g_fuelSummaryStartMs = millis();
    drawFuelSummaryScreen();
  }
  else
  {
    g_sessionActive = active;
  }
}

// Normal mod RFID: Idle / Fueling / Summary
void handleRfidInNormalMode()
{
  spiUseRFID();
  if (!mfrc522.PICC_IsNewCardPresent())
  {
    spiUseTFT();
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial())
  {
    spiUseTFT();
    return;
  }

  String uidHex = uidToHexString(mfrc522.uid);

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  spiUseTFT();

  Serial.print(F("Normal mod RFID: "));
  Serial.println(uidHex);

  bool isAdmin = (config.adminCard.isSet && uidHex == config.adminCard.uidHex);

  if (isAdmin)
  {
    showInfoMessage("Yonetici", "Admin kart okundu", "Ayarlar aciliyor",
                    SCR_SETUP_MENU, 1500);
    return;
  }

  if (currentScreen == SCR_IDLE || currentScreen == SCR_FUEL_SUMMARY)
  {
    int idx = findDriverIndexByUid(uidHex);
    if (idx < 0)
    {
      showInfoMessage("Sofor Kart", "Kart tanimli degil", "", SCR_IDLE, 1500);
      return;
    }

    g_activeDriverUid   = uidHex;
    g_activeDriverPlate = config.drivers.items[idx].plate;

    if (!meterStartSession())
    {
      showInfoMessage("RS485", "Dolum baslatilamadi", "Baglanti hatasi", SCR_IDLE, 1500);
      return;
    }

    g_sessionActive     = true;
    g_lastMeterPollMs   = 0;
    g_lastSessionLiters = 0.0f;

    MeterData md;
    if (meterRead(md))
    {
      g_lastMeter = md;
      g_lastSessionLiters = md.sessionVolCl / 100.0f;
    }
    else
    {
      memset(&g_lastMeter, 0, sizeof(g_lastMeter));
    }

    currentScreen = SCR_FUELING;
    drawFuelingScreen(g_lastMeter);
  }
}

