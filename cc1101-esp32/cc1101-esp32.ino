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

typedef struct cc1101_cfg_regs_t {
  uint8_t IOCFG2     ;
  uint8_t IOCFG1     ;
  uint8_t IOCFG0     ;
  uint8_t FIFOTHR    ;
  uint8_t SYNC1      ;
  uint8_t SYNC0      ;
  uint8_t PKTLEN     ;
  uint8_t PKTCTRL1   ;
  uint8_t PKTCTRL0   ;
  uint8_t ADDR       ;
  uint8_t CHANNR     ;
  uint8_t FSCTRL1    ;
  uint8_t FSCTRL0    ;
  uint8_t FREQ2      ;
  uint8_t FREQ1      ;
  uint8_t FREQ0      ;
  uint8_t MDMCFG4    ;
  uint8_t MDMCFG3    ;
  uint8_t MDMCFG2    ;
  uint8_t MDMCFG1    ;
  uint8_t MDMCFG0    ;
  uint8_t DEVIATN    ;
  uint8_t MCSM2      ;
  uint8_t MCSM1      ;
  uint8_t MCSM0      ;
  uint8_t FOCCFG     ;
  uint8_t BSCFG      ;
  uint8_t AGCCTRL2   ;
  uint8_t AGCCTRL1   ;
  uint8_t AGCCTRL0   ;
  uint8_t WOREVT1    ;
  uint8_t WOREVT0    ;
  uint8_t WORCTRL    ;
  uint8_t FREND1     ;
  uint8_t FREND0     ;
  uint8_t FSCAL3     ;
  uint8_t FSCAL2     ;
  uint8_t FSCAL1     ;
  uint8_t FSCAL0     ;
  uint8_t RCCTRL1    ;
  uint8_t RCCTRL0    ;
  uint8_t FSTEST     ;
  uint8_t PTEST      ;
  uint8_t AGCTEST    ;
  uint8_t TEST2      ;
  uint8_t TEST1      ;
  uint8_t TEST0      ;
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
    cc1101_cfg_t tmp_cc1101_cf;
    int r = sscanf(p, "%d,%d,%d,%lf,%lf,%d,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        &tmp_cc1101_cf.is_sender,
        &tmp_cc1101_cf.CCMode,
        &tmp_cc1101_cf.Modulation,
        &tmp_cc1101_cf.MHz,
        &tmp_cc1101_cf.Deviation,
        &tmp_cc1101_cf.Channel,
        &tmp_cc1101_cf.Chsp,
        &tmp_cc1101_cf.RxBW,
        &tmp_cc1101_cf.DRate,
        &tmp_cc1101_cf.PA,
        &tmp_cc1101_cf.SyncMode,
        &tmp_cc1101_cf.SyncWord1,
        &tmp_cc1101_cf.SyncWord2,
        &tmp_cc1101_cf.AdrChk,
        &tmp_cc1101_cf.Addr,
        &tmp_cc1101_cf.WhiteData,
        &tmp_cc1101_cf.PktFormat,
        &tmp_cc1101_cf.LengthConfig,
        &tmp_cc1101_cf.PacketLength,
        &tmp_cc1101_cf.Crc,
        &tmp_cc1101_cf.CRC_AF,
        &tmp_cc1101_cf.DcFilterOff,
        &tmp_cc1101_cf.Manchester,
        &tmp_cc1101_cf.FEC,
        &tmp_cc1101_cf.PRE,
        &tmp_cc1101_cf.PQT,
        &tmp_cc1101_cf.AppendStatus);
    DOLOG(F("GOT CFG 3: "));
    DOLOGLN(r);
    if(r == 0){
      s->GetSerial()->println(F("cc1101 cfg nr fields wrong"));
      s->GetSerial()->println(F("ERROR"));
      return;
    }
    DOLOGLN(F("GOT CFG 4"));
    if(   (tmp_cc1101_cf.is_sender == 1  || tmp_cc1101_cf.is_sender == 0)
       && (tmp_cc1101_cf.CCMode == 1     || tmp_cc1101_cf.CCMode == 0)
       && (tmp_cc1101_cf.Modulation >= 0 || tmp_cc1101_cf.Modulation <= 4)
       && (tmp_cc1101_cf.Channel >= 0    && tmp_cc1101_cf.Channel <= 255)
       && (    tmp_cc1101_cf.PA == -30 
            || tmp_cc1101_cf.PA == -20
            || tmp_cc1101_cf.PA == -15
            || tmp_cc1101_cf.PA == -10
            || tmp_cc1101_cf.PA ==  -6
            || tmp_cc1101_cf.PA ==   0
            || tmp_cc1101_cf.PA ==   5
            || tmp_cc1101_cf.PA ==   7
            || tmp_cc1101_cf.PA ==  10
            || tmp_cc1101_cf.PA ==  11
            || tmp_cc1101_cf.PA ==  12
       )
       && (tmp_cc1101_cf.SyncMode >= 0     && tmp_cc1101_cf.SyncMode <= 7)
       && (tmp_cc1101_cf.SyncWord1 >= 0    && tmp_cc1101_cf.SyncWord1 <= 255)
       && (tmp_cc1101_cf.SyncWord2 >= 0    && tmp_cc1101_cf.SyncWord2 <= 255)
       && (tmp_cc1101_cf.AdrChk >= 0       || tmp_cc1101_cf.AdrChk <= 3)
       && (tmp_cc1101_cf.Addr >= 0         || tmp_cc1101_cf.Addr <= 255)
       && (tmp_cc1101_cf.WhiteData == 0    || tmp_cc1101_cf.WhiteData <= 1)
       && (tmp_cc1101_cf.PktFormat >= 0    || tmp_cc1101_cf.PktFormat <= 3)
       && (tmp_cc1101_cf.LengthConfig >= 0 || tmp_cc1101_cf.LengthConfig <= 3)
       && (tmp_cc1101_cf.PacketLength >= 0 || tmp_cc1101_cf.PacketLength <= 255)
       && (tmp_cc1101_cf.Crc == 0          || tmp_cc1101_cf.Crc == 1)
       && (tmp_cc1101_cf.CRC_AF == 0       || tmp_cc1101_cf.CRC_AF == 1)
       && (tmp_cc1101_cf.DcFilterOff == 0  || tmp_cc1101_cf.DcFilterOff == 1)
       && (tmp_cc1101_cf.Manchester == 0   || tmp_cc1101_cf.Manchester == 1)
       && (tmp_cc1101_cf.FEC == 0          || tmp_cc1101_cf.FEC == 1)
       && (tmp_cc1101_cf.PRE >= 0          || tmp_cc1101_cf.PRE <= 7)
       && (tmp_cc1101_cf.PQT >= 0          || tmp_cc1101_cf.PQT <= 4)
       && (tmp_cc1101_cf.AppendStatus == 1 || tmp_cc1101_cf.AppendStatus == 0)
    ){
      if(tmp_cc1101_cf.is_sender){
        DOLOGLN(F("SENDER"));
      } else {
        DOLOGLN(F("RECEIVER"));
      }
      /* keep/store */
      if(0 != memcmp(&cfg.cc1101[0], &tmp_cc1101_cf, sizeof(cc1101_cfg_t))){
        DOLOGLN(F("Config changed, saving to EEPROM"));
        memcpy((char *)&cfg.cc1101[0], &tmp_cc1101_cf, sizeof(cc1101_cfg_t));
        EEPROM.put(CFG_EEPROM, cfg);
        EEPROM.commit();
        cc1101_changed[0] = 1;
      } else {
        DOLOGLN(F("Config not changed, not saving to EEPROM"));
      }
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

  delay(5000);

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

  SPI.begin();
  ELECHOUSE_cc1101.Init();
}

void loop(){
  // any new AT command? on USB uart
  ATSc.ReadSerial();

  // check cc1101 status
  if(millis() - last_cc1101_check[0] > 5000){
    cc1101_read_cfg(&cfg.cc1101[0]);
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
    if(cc1101_changed[0]){
      DOLOGLN(F("CC1101 settings changed"));
    } else {
      DOLOGLN(F("CC1101 config apply at init"));
    }
    ELECHOUSE_cc1101.setGDO(0, 1);
    ELECHOUSE_cc1101.setCCMode(cfg.cc1101[0].CCMode);
    ELECHOUSE_cc1101.setModulation(cfg.cc1101[0].Modulation);
    //ELECHOUSE_cc1101.setMHZ(cfg.cc1101[0].MHz);
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
    if(cfg.do_verbose){
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
    }
    if(cfg.cc1101[0].is_sender)
      ELECHOUSE_cc1101.SetTx(cfg.cc1101[0].MHz);
    else
      ELECHOUSE_cc1101.SetRx(cfg.cc1101[0].MHz);
    // bookeeping
    cc1101_changed[0]     = 0;
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
        /* use "byte" as type, and convert to that, the ELECHOUSE_cc1101 lib behaves differently */
        ELECHOUSE_cc1101.SendData((byte *)&uart_buffer, (byte)buffer_len);
        memset((char *)&uart_buffer, 0, sizeof(uart_buffer));
      }
    } else if(!cfg.cc1101[0].is_sender){
      if(cfg.do_verbose){
        if(millis() - last_logline > 5000){
          DOLOG(F("Rssi: "));
          DOLOGLN(ELECHOUSE_cc1101.getRssi());
          last_logline = millis();
        }
      }
      if(ELECHOUSE_cc1101.CheckReceiveFlag()){
        if(cfg.do_verbose){
          uint8_t lqi = ELECHOUSE_cc1101.getLqi();
          DOLOG(F("CHECK RECEIVE BUFFER: lqi="));
          DOLOG(lqi);
          DOLOG(F(",crc="));
          DOLOGLN(bitRead(lqi,7));
        }
        int len = ELECHOUSE_cc1101.ReceiveData((byte *)&in_buffer);
        in_buffer[len] = '\0';
        if(cfg.do_verbose){
          DOLOG(F("RECEIVED["));
          DOLOG(strlen((char *)&in_buffer));
          DOLOG(F("]>>"));
        }
        Serial.println((char *)&in_buffer);
        DOLOGLN(F("<<"));
        memset(in_buffer, 0, sizeof(in_buffer));
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

#define IOCFG2     0x00
#define IOCFG1     0x01
#define IOCFG0     0x02
#define FIFOTHR    0x03
#define SYNC1      0x04
#define SYNC0      0x05
#define PKTLEN     0x06
#define PKTCTRL1   0x07
#define PKTCTRL0   0x08
#define ADDR       0x09
#define CHANNR     0x0A
#define FSCTRL1    0x0B
#define FSCTRL0    0x0C
#define FREQ2      0x0D
#define FREQ1      0x0E
#define FREQ0      0x0F
#define MDMCFG4    0x10
#define MDMCFG3    0x11
#define MDMCFG2    0x12
#define MDMCFG1    0x13
#define MDMCFG0    0x14
#define DEVIATN    0x15
#define MCSM2      0x16
#define MCSM1      0x17
#define MCSM0      0x18
#define FOCCFG     0x19
#define BSCFG      0x1A
#define AGCCTRL2   0x1B
#define AGCCTRL1   0x1C
#define AGCCTRL0   0x1D
#define WOREVT1    0x1E
#define WOREVT0    0x1F
#define WORCTRL    0x20
#define FREND1     0x21
#define FREND0     0x22
#define FSCAL3     0x23
#define FSCAL2     0x24
#define FSCAL1     0x25
#define FSCAL0     0x26
#define RCCTRL1    0x27
#define RCCTRL0    0x28
#define FSTEST     0x29
#define PTEST      0x2A
#define AGCTEST    0x2B
#define TEST2      0x2C
#define TEST1      0x2D
#define TEST0      0x2E

#define CC1101_BURST         0x40
#define CC1101_READ          0x80

void cc1101_read_cfg(cc1101_cfg_t *c){
  cc1101_cfg_regs_t r_buffer;
  uint8_t *r_ptr = (uint8_t *)&r_buffer;
  uint16_t n, v;
  uint16_t status = 0;
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  /* per cc1101 datasheet, check MISO being low before continueing after CS set LOW */
  while(digitalRead(MISO)) doYIELD;
  status = SPI.transfer(n|CC1101_BURST|CC1101_READ);
  DOLOG("STATUS:"); DOLOGLN(status);
  for(n=0; n<sizeof(r_buffer); n++){
    doYIELD;
    *r_ptr++ = v = SPI.transfer(0);
    if(cfg.do_verbose){
      char info_str[25];
      int r = snprintf((char *)&info_str, 25, "0x%02x=%d", n, v);
      DOLOGLN(info_str);
    }
  }
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
  SPI.end();
  return;
}

uint16_t cc1101_spi_addr(uint8_t address, uint8_t value){
  #ifdef DEBUG
  Serial.print(address);
  Serial.print(F(","));
  Serial.print(value);
  Serial.print(F(","));
  Serial.print(value);
  Serial.println();
  #endif
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SS, LOW);
  /* per cc1101 datasheet, check MISO being low before continuing after CS set LOW */
  while(digitalRead(MISO)) doYIELD;
  uint16_t v = SPI.transfer16((address << 8) & value);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();
  return v;
}

