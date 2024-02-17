//#undef ARDUINO_USB_MODE
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include "SerialCommands.h"
#include "EEPROM.h"
#include "sntp.h"
#include "SPI.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"

#define LED_BUILTIN 8
#define LED LED_BUILTIN

#ifndef VERBOSE
#define VERBOSE
#endif
#ifndef DEBUG
#define DEBUG
#endif

/* NTP server to use, can be configured later on via AT commands */
#ifndef DEFAULT_NTP_SERVER
#define DEFAULT_NTP_SERVER "at.pool.ntp.org"
#endif

/* ESP yield */
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
 #define doYIELD yield();
#else
 #define doYIELD
#endif

/* our AT commands over UART to config OTP's and WiFi */
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");

#define CFGVERSION 0x02 // switch between 0x01/0x02 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag
#define CFG_EEPROM 0x00 

#ifdef VERBOSE
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define DOLOG(L)    if(cfg.do_verbose) Serial.print(L);
  #define DOLOGLN(L)  if(cfg.do_verbose) Serial.println(L);
 #else
  #define DOLOG(L)    if(cfg.do_verbose) Serial.print(L);
  #define DOLOGLN(L)  if(cfg.do_verbose) Serial.println(L);
 #endif
#else
 #define DOLOG(L) 
 #define DOLOGLN(L) 
#endif

typedef struct cc1101_cfg_t {
  uint8_t initialized;
  uint8_t version;
  uint8_t is_sender;
  uint8_t CCMode;
  uint8_t Modulation;
  double MHz;
  double Deviation;
  uint8_t Channel;
  double Chsp;
  double RxBW;
  double DRate;
  uint8_t PA;
  uint8_t SyncMode;
  uint8_t SyncWord1;
  uint8_t SyncWord2;
  uint8_t AdrChk;
  uint8_t Addr;
  uint8_t WhiteData;
  uint8_t PktFormat;
  uint8_t LengthConfig;
  uint8_t PacketLength;
  uint8_t Crc;
  uint8_t CRC_AF;
  uint8_t DcFilterOff;
  uint8_t Manchester;
  uint8_t FEC;
  uint8_t PRE;
  uint8_t PQT;
  uint8_t AppendStatus;
};

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  uint8_t do_verbose   = 0;
  uint8_t do_debug     = 0;
  char wifi_ssid[32]   = {0};   // max 31 + 1
  char wifi_pass[64]   = {0};   // nax 63 + 1
  char ntp_host[64]    = {0};   // max hostname + 1
  cc1101_cfg_t cc1101[1];
};
cfg_t cfg;

uint8_t ntp_is_synced              = 1;
uint8_t logged_wifi_status         = 0;
uint8_t cc1101_enabled[1]          = {0};
uint8_t cc1101_changed[1]          = {0};
uint8_t cc1101_initialized[1]      = {0};
unsigned long last_wifi_check      = 0;
unsigned long led_last_check       = 0;
unsigned long last_logline         = 0;
uint8_t led_status                 = 0;
unsigned long last_cc1101_check[1] = {0};
char uart_buffer[128] = {0};
byte in_buffer[128]   = {0};
void(* resetFunc)(void) = 0;

char* at_cmd_check(const char *cmd, const char *at_cmd, unsigned short at_len){
  unsigned short l = strlen(cmd); /* AT+<cmd>=, or AT, or AT+<cmd>? */
  if(at_len >= l && strncmp(cmd, at_cmd, l) == 0){
    if(*(cmd+l-1) == '='){
      return (char *)at_cmd+l;
    } else {
      return (char *)at_cmd;
    }
  }
  return NULL;
}

void at_cmd_handler(SerialCommands* s, const char* atcmdline){
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  #ifdef AT_DEBUG
  if(cfg.do_debug){
  DOLOG(F("AT: ["));
  DOLOG(atcmdline);
  DOLOG(F("], size: "));
  DOLOGLN(cmd_len);
  }
  #endif
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 31){
      s->GetSerial()->println(F("WiFI SSID max 31 chars"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.wifi_ssid);
    return;
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      s->GetSerial()->println(F("WiFI password max 63 chars"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    strncpy((char *)&cfg.wifi_pass, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
  #ifdef DEBUG
  } else if(p = at_cmd_check("AT+WIFI_PASS?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.wifi_pass);
  #endif
  #ifdef DEBUG
  } else if(p = at_cmd_check("AT+DEBUG=1", atcmdline, cmd_len)){
    cfg.do_debug = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+DEBUG=0", atcmdline, cmd_len)){
    cfg.do_debug = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+DEBUG?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.do_debug);
  #endif
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.do_verbose);
  #endif
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)){
    uint8_t wifi_stat = WiFi.status();
    switch(wifi_stat) {
        case WL_CONNECTED:
          s->GetSerial()->println(F("connected"));
          break;
        case WL_CONNECT_FAILED:
          s->GetSerial()->println(F("failed"));
          break;
        case WL_CONNECTION_LOST:
          s->GetSerial()->println(F("connection lost"));
          break;
        case WL_DISCONNECTED:
          s->GetSerial()->println(F("disconnected"));
          break;
        case WL_IDLE_STATUS:
          s->GetSerial()->println(F("idle"));
          break;
        case WL_NO_SSID_AVAIL:
          s->GetSerial()->println(F("no SSID configured"));
          break;
        default:
          s->GetSerial()->println(wifi_stat);
    }
    return;
  } else if(p = at_cmd_check("AT+CC1101=", atcmdline, cmd_len)){
    DOLOGLN(F("GOT CFG 1"));
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 128){
      s->GetSerial()->println(F("cc1101 cfg max 128 chars"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    DOLOGLN(F("GOT CFG 2:"));
    DOLOGLN(p);
    /* parse/check the cc1101 cfg */
    int r = sscanf(p, "%d,%d,%d,%lf,%lf,%d,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        &cfg.cc1101[0].is_sender,
        &cfg.cc1101[0].CCMode,
        &cfg.cc1101[0].Modulation,
        &cfg.cc1101[0].MHz,
        &cfg.cc1101[0].Deviation,
        &cfg.cc1101[0].Channel,
        &cfg.cc1101[0].Chsp,
        &cfg.cc1101[0].RxBW,
        &cfg.cc1101[0].DRate,
        &cfg.cc1101[0].PA,
        &cfg.cc1101[0].SyncMode,
        &cfg.cc1101[0].SyncWord1,
        &cfg.cc1101[0].SyncWord2,
        &cfg.cc1101[0].AdrChk,
        &cfg.cc1101[0].Addr,
        &cfg.cc1101[0].WhiteData,
        &cfg.cc1101[0].PktFormat,
        &cfg.cc1101[0].LengthConfig,
        &cfg.cc1101[0].PacketLength,
        &cfg.cc1101[0].Crc,
        &cfg.cc1101[0].CRC_AF,
        &cfg.cc1101[0].DcFilterOff,
        &cfg.cc1101[0].Manchester,
        &cfg.cc1101[0].FEC,
        &cfg.cc1101[0].PRE,
        &cfg.cc1101[0].PQT,
        &cfg.cc1101[0].AppendStatus);
    DOLOG(F("GOT CFG 3: "));
    DOLOGLN(r);
    if(r == 0){
      s->GetSerial()->println(F("cc1101 cfg nr fields wrong"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    DOLOGLN(F("GOT CFG 4"));
    if(   (cfg.cc1101[0].is_sender == 1 || cfg.cc1101[0].is_sender == 0)
       && (cfg.cc1101[0].CCMode == 1 || cfg.cc1101[0].CCMode == 0)
       && (cfg.cc1101[0].Modulation >= 0 || cfg.cc1101[0].Modulation <= 4)
       && (cfg.cc1101[0].Channel >= 0 && cfg.cc1101[0].Channel <= 255)
       && (    cfg.cc1101[0].PA == -30 
            || cfg.cc1101[0].PA == -20
            || cfg.cc1101[0].PA == -15
            || cfg.cc1101[0].PA == -10
            || cfg.cc1101[0].PA ==  -6
            || cfg.cc1101[0].PA ==   0
            || cfg.cc1101[0].PA ==   5
            || cfg.cc1101[0].PA ==   7
            || cfg.cc1101[0].PA ==  10
            || cfg.cc1101[0].PA ==  11
            || cfg.cc1101[0].PA ==  12
       )
       && (cfg.cc1101[0].SyncMode >= 0 && cfg.cc1101[0].SyncMode <= 7)
       && (cfg.cc1101[0].SyncWord1 >= 0 && cfg.cc1101[0].SyncWord1 <= 255)
       && (cfg.cc1101[0].SyncWord2 >= 0 && cfg.cc1101[0].SyncWord2 <= 255)
       && (cfg.cc1101[0].AdrChk >= 0 || cfg.cc1101[0].AdrChk <= 3)
       && (cfg.cc1101[0].Addr >= 0 || cfg.cc1101[0].Addr <= 255)
       && (cfg.cc1101[0].WhiteData == 0 || cfg.cc1101[0].WhiteData <= 0)
       && (cfg.cc1101[0].PktFormat >= 0 || cfg.cc1101[0].PktFormat <= 3)
       && (cfg.cc1101[0].LengthConfig >= 0 || cfg.cc1101[0].LengthConfig <= 3)
       && (cfg.cc1101[0].PacketLength >= 0 || cfg.cc1101[0].PacketLength <= 255)
       && (cfg.cc1101[0].Crc == 0 || cfg.cc1101[0].Crc == 1)
       && (cfg.cc1101[0].CRC_AF == 0 || cfg.cc1101[0].CRC_AF == 1)
       && (cfg.cc1101[0].DcFilterOff == 0 || cfg.cc1101[0].DcFilterOff == 1)
       && (cfg.cc1101[0].Manchester == 0 || cfg.cc1101[0].Manchester == 1)
       && (cfg.cc1101[0].FEC == 0 || cfg.cc1101[0].FEC == 1)
       && (cfg.cc1101[0].PRE >= 0 || cfg.cc1101[0].PRE <= 7)
       && (cfg.cc1101[0].PQT >= 0 || cfg.cc1101[0].PQT <= 4)
       && (cfg.cc1101[0].AppendStatus == 1 || cfg.cc1101[0].AppendStatus == 0)
    ){
      if(cfg.cc1101[0].is_sender){
        DOLOGLN(F("SENDER"));
      } else {
        DOLOGLN(F("RECEIVE"));
      }
      /* keep/store */
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
      cc1101_changed[0] = 1;
    } else {
      s->GetSerial()->println(F("cc1101 cfg invalid"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      s->GetSerial()->println(F("NTP hostname max 63 chars"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    strncpy((char *)&cfg.ntp_host, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_wifi();
    configTime(0, 0, (char *)&cfg.ntp_host);
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)){
    s->GetSerial()->println(cfg.ntp_host);
    return;
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)){
    if(ntp_is_synced)
      s->GetSerial()->println(F("ntp synced"));
    else
      s->GetSerial()->println(F("not ntp synced"));
    return;
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    s->GetSerial()->println(F("OK"));
    resetFunc();
    return;
  } else if(cmd_len >= 3 && (p = at_cmd_check("AT+", atcmdline, cmd_len))){
    s->GetSerial()->println(F("ERROR unkown AT command"));
    return;
  } else {
	/* just relay the data */
    memcpy((char *)&uart_buffer, atcmdline, cmd_len);
    return;
  }
  s->GetSerial()->println(F("OK"));
  return;
}

void setup(){

  // Serial setup, at 115200
  Serial.begin(115200);
  Serial.setDebugOutput(true);
 
  // setup cfg
  setup_cfg();

  DOLOG("SS: ");
  DOLOG(SS);
  DOLOG(", MOSI: ");
  DOLOG(MOSI);
  DOLOG(", MISO: ");
  DOLOG(MISO);
  DOLOG(", SCK: ");
  DOLOGLN(SCK);

  // Setup AT command handler
  ATSc.SetDefaultHandler(&at_cmd_handler);

  // see http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  // for OTP tokens, this ALWAYS have to be "UTC"
  setenv("TZ", "UTC", 1);
  tzset();

  // setup WiFi with ssid/pass from EEPROM if set
  setup_wifi();

  if(strlen(cfg.ntp_host) && strlen(cfg.wifi_ssid) && strlen(cfg.wifi_pass)){
    DOLOG(F("will sync with ntp, wifi ssid/pass ok: "));
    DOLOGLN(cfg.ntp_host);
  }
  // setup NTP sync to RTC
  configTime(0, 0, (char *)&cfg.ntp_host);

  /* ESP32 header not set? ELECHOUSE_cc1101 library uses that for default SPI
     pins and doesn't find it */
  ELECHOUSE_cc1101.setSpiPin(SCK, MISO, MOSI, SS);
  if(ELECHOUSE_cc1101.getCC1101()){
    DOLOGLN("SPI cc1101 found, Connection OK");
    cc1101_enabled[0] = 1;
  } else {
    DOLOGLN("SPI cc1101 not found, Connection Error");
    cc1101_enabled[0] = 0;
  }

  pinMode(LED, OUTPUT);

  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setGDO(0, 1);
}

void loop(){
  // any new AT command? on USB uart
  ATSc.ReadSerial();

  // check cc1101 status
  if(millis() - last_cc1101_check[0] > 5000){
    if(ELECHOUSE_cc1101.getCC1101()){
      DOLOGLN("SPI cc1101 found, Connection OK");
      cc1101_enabled[0] = 1;
    } else {
      DOLOGLN("SPI cc1101 not found, Connection Error");
      cc1101_enabled[0] = 0;
    }
    last_cc1101_check[0] = millis();
  }

  if(cc1101_enabled[0] && (cc1101_changed[0] || !cc1101_initialized[0])){
    DOLOGLN(F("CC1101 settings changed"));
    ELECHOUSE_cc1101.setCCMode(cfg.cc1101[0].CCMode);
    ELECHOUSE_cc1101.setModulation(cfg.cc1101[0].Modulation);
    ELECHOUSE_cc1101.setMHZ(cfg.cc1101[0].MHz);
    ELECHOUSE_cc1101.setDeviation(cfg.cc1101[0].Deviation);
    ELECHOUSE_cc1101.setChannel(cfg.cc1101[0].Channel);
    ELECHOUSE_cc1101.setChsp(cfg.cc1101[0].Chsp);
    ELECHOUSE_cc1101.setRxBW(cfg.cc1101[0].RxBW);
    ELECHOUSE_cc1101.setDRate(cfg.cc1101[0].DRate);
    ELECHOUSE_cc1101.setPA(cfg.cc1101[0].PA);
    ELECHOUSE_cc1101.setSyncMode(cfg.cc1101[0].SyncMode);
    ELECHOUSE_cc1101.setSyncWord(cfg.cc1101[0].SyncWord1, cfg.cc1101[0].SyncWord2);
    ELECHOUSE_cc1101.setAdrChk(cfg.cc1101[0].AdrChk);
    ELECHOUSE_cc1101.setAddr(cfg.cc1101[0].Addr);
    ELECHOUSE_cc1101.setWhiteData(cfg.cc1101[0].WhiteData);
    ELECHOUSE_cc1101.setPktFormat(cfg.cc1101[0].PktFormat);
    ELECHOUSE_cc1101.setLengthConfig(cfg.cc1101[0].LengthConfig);
    ELECHOUSE_cc1101.setPacketLength(cfg.cc1101[0].PacketLength);
    ELECHOUSE_cc1101.setCrc(cfg.cc1101[0].Crc);
    ELECHOUSE_cc1101.setCRC_AF(cfg.cc1101[0].CRC_AF);
    ELECHOUSE_cc1101.setDcFilterOff(cfg.cc1101[0].DcFilterOff);
    ELECHOUSE_cc1101.setManchester(cfg.cc1101[0].Manchester);
    ELECHOUSE_cc1101.setFEC(cfg.cc1101[0].FEC);
    ELECHOUSE_cc1101.setPRE(cfg.cc1101[0].PRE);
    ELECHOUSE_cc1101.setPQT(cfg.cc1101[0].PQT);
    ELECHOUSE_cc1101.setAppendStatus(cfg.cc1101[0].AppendStatus);
    DOLOG(F("CCMode:"));
    DOLOG(cfg.cc1101[0].CCMode);
    DOLOG(F(",Modulation:"));
    DOLOG(cfg.cc1101[0].Modulation);
    DOLOG(F(",MHz:"));
    DOLOG(cfg.cc1101[0].MHz);
    DOLOG(F(",Deviation:"));
    DOLOG(cfg.cc1101[0].Deviation);
    DOLOG(F(",Channel:"));
    DOLOG(cfg.cc1101[0].Channel);
    DOLOG(F(",Chsp:"));
    DOLOG(cfg.cc1101[0].Chsp);
    DOLOG(F(",RxBW:"));
    DOLOG(cfg.cc1101[0].RxBW);
    DOLOG(F(",DRate:"));
    DOLOG(cfg.cc1101[0].DRate);
    DOLOG(F(",PA:"));
    DOLOG(cfg.cc1101[0].PA);
    DOLOG(F(",SyncMode:"));
    DOLOG(cfg.cc1101[0].SyncMode);
    DOLOG(F(",SyncWord1:"));
    DOLOG(cfg.cc1101[0].SyncWord1);
    DOLOG(F(",SyncWord2:"));
    DOLOG(cfg.cc1101[0].SyncWord2);
    DOLOG(F(",AdrChk:"));
    DOLOG(cfg.cc1101[0].AdrChk);
    DOLOG(F(",Addr:"));
    DOLOG(cfg.cc1101[0].Addr);
    DOLOG(F(",WhiteData:"));
    DOLOG(cfg.cc1101[0].WhiteData);
    DOLOG(F(",PktFormat:"));
    DOLOG(cfg.cc1101[0].PktFormat);
    DOLOG(F(",LengthConfig:"));
    DOLOG(cfg.cc1101[0].LengthConfig);
    DOLOG(F(",PacketLength:"));
    DOLOG(cfg.cc1101[0].PacketLength);
    DOLOG(F(",Crc:"));
    DOLOG(cfg.cc1101[0].Crc);
    DOLOG(F(",CRC_AF:"));
    DOLOG(cfg.cc1101[0].CRC_AF);
    DOLOG(F(",DcFilterOff:"));
    DOLOG(cfg.cc1101[0].DcFilterOff);
    DOLOG(F(",Manchester:"));
    DOLOG(cfg.cc1101[0].Manchester);
    DOLOG(F(",FEC:"));
    DOLOG(cfg.cc1101[0].FEC);
    DOLOG(F(",PRE:"));
    DOLOG(cfg.cc1101[0].PRE);
    DOLOG(F(",PQT:"));
    DOLOG(cfg.cc1101[0].PQT);
    DOLOG(F(",AppendStatus:"));
    DOLOG(cfg.cc1101[0].AppendStatus);
    DOLOGLN();
    if(cfg.cc1101[0].is_sender)
      ELECHOUSE_cc1101.SetTx();
    else
      ELECHOUSE_cc1101.SetRx();
    cc1101_changed[0] = 0;
    cc1101_initialized[0] = 1;
  }

  if(millis() - led_last_check > 500){
    if(cfg.cc1101[0].is_sender){
      #ifdef DEBUG
      if(cfg.do_debug){
        DOLOG(F("LED BLINK SENDER:"));
        DOLOG(LED);
        DOLOG(F(","));
        DOLOGLN(led_status);
      }
      #endif
      digitalWrite(LED, led_status);
      if(led_status == HIGH)
        led_status = LOW;
      else
        led_status = HIGH;
    } else {
      #ifdef DEBUG
      if(cfg.do_debug){
        DOLOG(F("LED BLINK RECEIVER:"));
        DOLOG(LED);
        DOLOG(F(","));
        DOLOGLN(HIGH);
      }
      #endif
      digitalWrite(LED, HIGH);
    }
    led_last_check = millis();
  }

  if(cc1101_enabled[0]){
    if(cfg.cc1101[0].is_sender){
      int buffer_len = strlen((char *)&uart_buffer);
      if(buffer_len > 0){
        if(cfg.do_verbose){
            DOLOG(F("SEND BUFFER["));
            DOLOG(buffer_len);
            DOLOG(F("]>>"));
            DOLOG(uart_buffer);
            DOLOGLN(F("<<"));
        }
        ELECHOUSE_cc1101.SendData((char *)&uart_buffer, buffer_len);
        memset((char *)&uart_buffer, 0, sizeof(uart_buffer));
      }
    } else if(!cfg.cc1101[0].is_sender){
      if(cfg.do_verbose){
        if(millis() - last_logline > 5000){
          DOLOG(F("Rssi: "));
          DOLOG(ELECHOUSE_cc1101.getRssi());
          DOLOG(F(", LQI: "));
          DOLOGLN(ELECHOUSE_cc1101.getLqi());
          last_logline = millis();
        }
      }
      if(ELECHOUSE_cc1101.CheckReceiveFlag()){
        DOLOG(F("CHECK RECEIVE BUFFER"));
        if(ELECHOUSE_cc1101.CheckCRC()){
          int len = ELECHOUSE_cc1101.ReceiveData((byte *)&in_buffer);
          in_buffer[len] = '\0';
          Serial.println((char *)&in_buffer);
          memset(in_buffer, 0, sizeof(in_buffer));
        }
      }
    }
  }

  // just wifi check
  if(millis() - last_wifi_check > 500){
    if(WiFi.status() == WL_CONNECTED){
      if(!logged_wifi_status){
        DOLOG(F("WiFi connected: "));
        DOLOGLN(WiFi.localIP());
        logged_wifi_status = 1;
      }
    }
    last_wifi_check = millis();
  }
}

void setup_cfg(){
  // EEPROM read
  EEPROM.begin(sizeof(cfg));
  EEPROM.get(CFG_EEPROM, cfg);
  // was (or needs) initialized?
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION){
    // clear
    memset(&cfg, 0, sizeof(cfg));
    // reinit
    cfg.initialized = CFGINIT;
    cfg.version     = CFGVERSION;
    cfg.do_verbose  = 1;
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    // write
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
  if(cfg.cc1101[0].initialized != CFGINIT || cfg.cc1101[0].version != CFGVERSION){
    // clear
    memset(&cfg.cc1101[0], 0, sizeof(cc1101_cfg_t));
    // reinit
    cfg.cc1101[0].initialized = CFGINIT;
    cfg.cc1101[0].version     = CFGVERSION;
    // write
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
}

void setup_wifi(){
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_pass) == 0)
    return;
  if(WiFi.status() == WL_CONNECTED)
    return;

  // connect to Wi-Fi
  DOLOG(F("Connecting to "));
  DOLOGLN(cfg.wifi_ssid);
  WiFi.persistent(false);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
}

