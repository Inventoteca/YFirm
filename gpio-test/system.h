#ifndef SYSTEM_H
#define SYSTEM_H

//15 seconds WDT
#define   PRESS               LOW
#define   WDT_TIMEOUT         15
#define   BUFFER_TOPIC_SIZE   100
#define   BUFFER_MSG_SIZE     1024
#define   FILE_SIZE           BUFFER_MSG_SIZE

#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Arduino.h>
#include <esp_task_wdt.h>
#include "esp_attr.h"
#include "SPIFFS.h"
#include "FS.h"


#include "pins.h"
#include "filespiffs.h"
#include "wifiservice.h"
#include "counter.h"
//#include "clock.h"
#include "mqttservice.h"
#include "version.h"

// --------------------------- variables para spiffs
extern JsonObject obj;
extern StaticJsonDocument<FILE_SIZE> doc;
extern StaticJsonDocument<FILE_SIZE> doc_status;


extern const char *filename;
extern const char *filedefault;
extern volatile bool saveConfig;
extern File file;

// ---------------------------- variables para contadores
extern long flag_moneda;
extern long flag_premio;
extern long flag_bolsa;

// ----------------------------- variables para mqtt
extern WiFiClient espClient;
extern PubSubClient Mclient;
extern WiFiClient client_http;
extern const char* publish_topic; 
extern const char* subcribe_topic; 
extern char buffer_union_publish[BUFFER_TOPIC_SIZE]; 
extern char buffer_union_subcribe[BUFFER_TOPIC_SIZE];
extern char buffer_msg[BUFFER_MSG_SIZE];
extern const char* client_id;

// ----------------------------- variables para wifi
extern bool correct;
extern int wifi_trys;
extern boolean isSaved;


// ----------------------------- generales
extern bool factory_press;
extern unsigned long factory_time;
extern unsigned long prev_factory_time;
//extern bool reset_time;
//extern byte localAddress;     // address of this device
extern unsigned long mainRefresh;
extern unsigned long mainTime;
extern const uint32_t connectTimeoutMs;
extern unsigned long  s_timestamp;
extern int buttonState;

//void IRAM_ATTR factory_reset3();
void system_init();
void reset_config();
//void check_reset();
bool strToBool(String str);
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void loadConfig();

#endif
