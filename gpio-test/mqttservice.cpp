#include "mqttservice.h"

// -------------------------------------------------- mqtt_init
bool mqtt_init()
{
  //if (obj["enable_mqtt"].as<bool>())
  {

    Serial.println("{\"mqtt\":\"init\"}");
    Mclient.setBufferSize(BUFFER_MSG_SIZE);
    Mclient.setServer(obj["mqtt_server"].as<const char*>(), obj["mqtt_port"].as<unsigned int>());
    Mclient.setCallback(callback);
    return true;

  }
  //return false;


}

// ------------------------------------------------- mqtt_check
bool mqtt_check()
{
  // MQTT Enable
  //if (mqtt_init())
  {
    esp_task_wdt_reset();
    if (!Mclient.connected())
    {
      if (reconnect())
        return true;
      else
        return false;

    }
    else
    {
      Mclient.loop();
      return true;
    }
  }

}


//---------------------------------------------------- mqtt_send
void mqtt_send()
{
  //if (mqtt_init())
  {
    //Mclient.publish(buffer_union_publish, buffer_msg);

    strcat(strcpy(buffer_union_publish, client_id), publish_topic);
    //snprintf(buffer_msg, sizeof(buffer_msg), "%ld", flag_bolsa);

    serializeJson(obj, buffer_msg);
    Serial.print(buffer_msg);
    Mclient.publish(buffer_union_publish, buffer_msg);

  }
}

//--------------------------------------------------- callback
void callback(char* topic, byte* payload, unsigned int length)
{
  char jsonPayload[length + 1]; // +1 para el carácter nulo
  memcpy(jsonPayload, payload, length);
  jsonPayload[length] = '\0'; // Agrega el carácter nulo al final
  Serial.print("Message arrived: ");

  //if (obj["test"].as<bool>())
  {
    Serial.print(topic);
    Serial.print("<-- ");
    Serial.print(jsonPayload); // Imprime el payload como cadena
  }
  Serial.println();

 // StaticJsonDocument<LIST_SIZE> doc_m;
 // DeserializationError error = deserializeJson(doc_m, jsonPayload);
}


//--------------------------------------------------- reconnect
bool reconnect()
{
  //if (mqtt_init())
  {
    strcat(strcpy(buffer_union_subcribe, client_id), subcribe_topic);

    if (!Mclient.connected())
    {
      Serial.print("Attempting MQTT connection...");
      if (Mclient.connect(client_id /*, mqttUser, mqttPassword*/))
      {
        Serial.println("connected");
        Mclient.subscribe(buffer_union_subcribe);
        return true;
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(Mclient.state());
        Serial.println(" try in the next");
        return false;
      }
    }
    else
    {
      return true;
    }
  }

}
