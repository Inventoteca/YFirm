#ifndef MQTTSERVICE_H
#define MQTTSERVICE_H

#include "system.h"

void callback(char* topic, byte* payload, unsigned int length);
bool reconnect();
bool mqtt_init();
bool mqtt_check();
void mqtt_send();


#endif  //
