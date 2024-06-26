#include "mqttservice.h"

WiFiClient espClient;
PubSubClient Mclient(espClient);
WiFiClient client_http;

const char* root_topic = "maquinas/";
const char* publish_topic = "/out";
const char* subscribe_topic = "/in";
const char* config_topic = "/config";
const char* log_topic = "/log";
const char* gps_topic = "/gps";
const char* status_topic = "/status";
const char* reporte_topic = "/reporte";
const char* wild_topic = "/#";
const char* id_premios = "24";
const char* id_juegos = "23";

char buffer_union_publish[FILE_SIZE];
char buffer_union_subscribe[FILE_SIZE];
char buffer_msg[FILE_SIZE];
char buffer_msg_status[STATUS_SIZE];


volatile boolean send_log = false;
volatile boolean send_reporte = false;


// -------------------------------------------------- mqtt_init
void mqtt_init()
{
  //if (obj["enable_mqtt"].as<bool>())
  {

    Serial.println("{\"mqtt\":\"init\"}");
    Mclient.setBufferSize(FILE_SIZE);
    Mclient.setServer(obj["mqtt_server"].as<const char*>(), obj["mqtt_port"].as<unsigned int>());
    Mclient.setCallback(callback);
    Mclient.setKeepAlive((mainTime / 1000) * 3);
  }


}

// ------------------------------------------------- mqtt_check
bool mqtt_check()
{
  // MQTT Enable
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

//---------------------------------------------------- mqtt_send_reporte
void mqtt_send_reporte()
{

  Serial.println("{\"mqtt_reporte\":\"sending\"}");

  //saveNewlog();
  strcpy(buffer_union_publish, root_topic);
  strcat(buffer_union_publish, obj["id"].as<const char*>());
  strcat(buffer_union_publish, publish_topic);
  strcat(buffer_union_publish, reporte_topic);

  //JsonArray logObject = obj_log;
  //size_t serializedLength = measureJson(logObject) + 1;
  if (!obj["yuser"].isNull()) // No es null
  {
    char tempBuffer[20];

    const char* tokenValue = obj["yuser"].as<const char*>();
    //serializeJson(obj["token"], tempBuffer);

    strcpy(buffer_msg_status, obj["id"].as<const char*>());
    strcat(buffer_msg_status, " ");
    strcat(buffer_msg_status, tokenValue);
    strcat(buffer_msg_status, " [");
    strcat(buffer_msg_status, id_premios);
    strcat(buffer_msg_status, ":");
    snprintf(tempBuffer, sizeof(tempBuffer), "%ld", flag_stock); 
    strcat(buffer_msg_status, tempBuffer);  // status_doc["gift"] = flag_stock;
    strcat(buffer_msg_status, ",");
    strcat(buffer_msg_status, id_juegos);
    strcat(buffer_msg_status, ":");
    snprintf(tempBuffer, sizeof(tempBuffer), "%ld", flag_games); 
    strcat(buffer_msg_status, tempBuffer);
    strcat(buffer_msg_status, "]");

    Mclient.publish(buffer_union_publish, buffer_msg_status);
    //Mclient.publish(buffer_union_publish, buffer_msg);
  }
  else
  {
    Serial.println("{\"mqtt_reporte\":\"error\"}");
  }

}

//---------------------------------------------------- mqtt_send
void mqtt_send()
{

  Serial.println("{\"mqtt_status\":\"sending\"}");

  //saveNewlog();
  strcpy(buffer_union_publish, root_topic);
  strcat(buffer_union_publish, obj["id"].as<const char*>());
  strcat(buffer_union_publish, publish_topic);
  strcat(buffer_union_publish, gps_topic);

  //JsonArray logObject = obj_log;
  //size_t serializedLength = measureJson(logObject) + 1;
  char tempBuffer[STATUS_SIZE];
  serializeJson(status_doc, tempBuffer);
  strcpy(buffer_msg_status, tempBuffer);

  Mclient.publish(buffer_union_publish, buffer_msg_status);
  //Mclient.publish(buffer_union_publish, buffer_msg);
}


// --------------------------------------------------- mqtt_send_file
void mqtt_send_file(String file_to_send)
{
  send_log = false;
  File file = SD.open(file_to_send);
  if (!file) {
    Serial.print("Error al abrir el archivo: ");
    Serial.println(file_to_send);
    return;
  }

  strcpy(buffer_union_publish, root_topic);
  strcat(buffer_union_publish, obj["id"].as<const char*>());
  //strcat(buffer_union_publish, publish_topic);
  strcat(buffer_union_publish, publish_topic);
  strcat(buffer_union_publish, file_to_send.c_str());


  while (file.available())
  {
    String line = file.readStringUntil('\n');
    if (line.length() > 0) {

      // Publicar la línea
      Mclient.publish(buffer_union_publish, line.c_str(), line.length());
      delay(100); // Pequeña pausa para evitar saturar el cliente MQTT
      esp_task_wdt_reset();
    }
  }

  file.close();
  Serial.println("Archivo enviado");
}



//--------------------------------------------------- callback
void callback(char* topic, byte* payload, unsigned int length)
{
  esp_task_wdt_reset();
  char jsonPayload[length + 1]; // +1 para el carácter nulo
  memcpy(jsonPayload, payload, length);
  jsonPayload[length] = '\0'; // Agrega el carácter nulo al final
  Serial.print("Message arrived: ");

  if (obj["test"].as<bool>())
  {
    Serial.print(topic);
    Serial.print("<-- ");
    Serial.print(jsonPayload); // Imprime el payload como cadena
  }
  Serial.println();

  //strcpy(buffer_union_subscribe, root_topic);
  //strcat(buffer_union_subscribe, obj["id"].as<const char*>());
  //strcat(buffer_union_subscribe, subscribe_topic);
  //strcat(buffer_union_subscribe, config_topic);


  if (strcmp(topic, strcat(strcat(strcat(strcpy(buffer_union_subscribe, root_topic), obj["id"].as<const char*>()), subscribe_topic), config_topic)) == 0)
  {
    StaticJsonDocument<FILE_SIZE> conf_mqtt_doc;
    Serial.println("Config Update");

    DeserializationError error = deserializeJson(conf_mqtt_doc, jsonPayload);

    if (error) {
      Serial.print("deserialize conf_mqtt_doc failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Recorrer cada par clave-valor en el JSON recibido
    //for (JsonPair p : conf_mqtt_doc.as<JsonObject>())
    for (JsonPair p : conf_mqtt_doc.as<JsonObject>())
    {
      String key = p.key().c_str(); // Obtener la clave como String
      JsonVariant value = p.value(); // Obtener el valor

      // Asegúrate de que la clave no esté vacía
      if (!key.isEmpty() && (!value.isNull()))
      {
        // Aquí puedes agregar verificaciones específicas de tipo si es necesario
        // Por ejemplo, para un string:
        if (value.is<const char*>())
        {
          obj[key] = value.as<String>();
          Serial.println("const char");
        }
        // Repite para otros tipos según sea necesario, por ejemplo, para un booleano:
        else if (value.is<bool>())
        {
          obj[key] = value.as<bool>();
          Serial.println("bool");
        }
        else if (value.is<String>())
        {
          obj[key] = value.as<String>();
          Serial.println("String");
        }
        // Si el valor es de un tipo complejo como otro objeto, puedes simplemente asignarlo
        else  {
          obj[key] = value;
          Serial.println("UK");
        }

        if (key == "yuser")
        {
          send_reporte = true;
          saveConfig = true;
        }
        else if (key == "test")
        {
          send_log = true;
          saveConfig = false;
        }
        else
          saveConfig = true;
      }
    }

    // Serializar y mostrar el JSON actualizado
    serializeJson(conf_mqtt_doc, Serial);
    Serial.println();


    send_log = true;
    return;
  }
  //else if()


  return;
}



//--------------------------------------------------- reconnect
bool reconnect()
{
  bool recsta = false;

  //strcat(strcpy(buffer_union_subcribe, obj["id"].as<const char*>()), subcribe_topic);
  strcpy(buffer_union_subscribe, root_topic);
  strcat(buffer_union_subscribe, obj["id"].as<const char*>());
  strcat(buffer_union_subscribe, subscribe_topic);
  const char* macAddress = "mac";
  //const char* macAddress = getMACAddress();

  if (!Mclient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "yumiso-";
    clientId += String(random(0xffff), HEX);

    // Verificar si las claves 'mqtt_user' y 'mqtt_password' existen y no son null
    if (obj.containsKey("mqtt_user") && !obj["mqtt_user"].isNull() && obj.containsKey("mqtt_pass") && !obj["mqtt_pass"].isNull())
    {

      const char* mqtt_user = obj["mqtt_user"].as<const char*>();
      const char* mqtt_pass = obj["mqtt_pass"].as<const char*>();

      // Configurar usuario y contraseña
      if (Mclient.connect(clientId.c_str(), mqtt_user, mqtt_pass))
      {
        Serial.println("connected whit user/pass");
        //Mclient.subscribe(strcat(strcat(strcpy(buffer_union_subscribe, obj["id"].as<const char*>()), subscribe_topic), wild_topic));
        //Mclient.subscribe(strcat(strcat(strcat(strcpy(buffer_union_subscribe, root_topic), obj["id"].as<const char*>()), subscribe_topic)),wild_topic);
        strcpy(buffer_union_subscribe, root_topic);
        strcat(buffer_union_subscribe, obj["id"].as<const char*>());
        strcat(buffer_union_subscribe, subscribe_topic);
        strcat(buffer_union_subscribe, wild_topic);

        Mclient.subscribe(buffer_union_subscribe);
        recsta =  true;
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(Mclient.state());
        Serial.println(" try in the next");
        recsta =  false;
      }
    }
    else
    {
      if (Mclient.connect(clientId.c_str()))
      {
        Serial.println("connected, NO user/pass");
        //Mclient.subscribe(strcat(strcat(strcpy(buffer_union_subscribe, obj["id"].as<const char*>()), subscribe_topic), wild_topic));
        strcpy(buffer_union_subscribe, root_topic);
        strcat(buffer_union_subscribe, obj["id"].as<const char*>());
        strcat(buffer_union_subscribe, subscribe_topic);
        strcat(buffer_union_subscribe, wild_topic);

        Mclient.subscribe(buffer_union_subscribe);
        recsta =  true;
      }
      else
      {
        Serial.print("failed, rc=");
        Serial.print(Mclient.state());
        Serial.println(" try in the next");

        //STATE &= ~(1 << 0);                 // MQTT error
        recsta =  false;
      }
    }


  }
  return recsta;
}
