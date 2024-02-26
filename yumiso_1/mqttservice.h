#ifndef MQTTSERVICE_H
#define MQTTSERVICE_H

#include "system.h"

extern WiFiClient espClient;
extern PubSubClient Mclient;
extern WiFiClient client_http;

extern const char* root_topic;
extern const char* publish_topic; 
extern const char* subscribe_topic; 

extern const char* config_topic;
extern const char* wild_topic;
extern const char* gps_topic;
extern const char* status_topic;
extern const char* log_topic;
extern const char* reporte_topic;

extern char buffer_union_publish[FILE_SIZE]; 
extern char buffer_union_subscribe[FILE_SIZE];
extern char buffer_msg[FILE_SIZE];
extern char buffer_msg_status[STATUS_SIZE];

extern volatile boolean send_log;

void callback(char* topic, byte* payload, unsigned int length);
bool reconnect();
void mqtt_init();
bool mqtt_check();
void mqtt_send();
void mqtt_send_file(String file_to_send);



#endif  //
