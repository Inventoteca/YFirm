#include "firebasedb.h"

// Define Firebase Data object
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
//FirebaseJson events_json;
FirebaseJson conf;
FirebaseJson data_json;
String nodeName;
volatile bool updated = true;
volatile bool dataChanged = false;
volatile bool nullData = false;
//volatile bool saveConfig = false;
volatile bool fire_stream = false;
volatile bool update_events = false;
String route = "/panels/" + obj["id"].as<String>();// + "/actual";
FirebaseData stream;



// -------------------------------------------------------------- readCounterValue
void readCounterValue(String collection, String document, String field) {
  // Asegúrate de que obj está correctamente inicializado y contiene "storage_id"
  String project_id = obj["storage_id"].as<String>();
  
  if (Firebase.Firestore.getDocument(&fbdo, project_id.c_str(), "", collection.c_str(), document.c_str())) {
    //FirebaseJson json = fbdo.jsonData();  // Crea un nuevo objeto JSON y copia los datos
    //FirebaseJsonData jsonData;
    
    //if (json.get(jsonData, field.c_str())) 
    //{
      //if (jsonData.typeNum == FirebaseJson::JSON_TYPE_INT) 
      //{
        //int counterValue = jsonData.intValue;
        Serial.printf("Counter value for %s:\n", field.c_str());
      //} 
      //else 
      //{
        //Serial.printf("Field %s is not of type int\n", field.c_str());
     // }
    //} 
    //else 
    //{
      //Serial.printf("Field %s not found in document %s\n", field.c_str(), document.c_str());
    //}
  } 
  else 
  {
    Serial.printf("Error reading document: %s\n", fbdo.errorReason().c_str());
  }
}



// ----------------------------------- OTA The Firebase Storage download callback function
void fcsDownloadCallback(FCS_DownloadStatusInfo info)
{
  esp_task_wdt_reset();

  if (info.status == fb_esp_fcs_download_status_init)
  {
    Serial.printf("update %s (%d)\n", info.remoteFileName.c_str(), info.fileSize);
    obj["updated"] = true;
    obj["registered"] = false;
    saveConfig = true;
    //json.clear();
    //json.set("updated", true);
    Serial.println("{\"update_firmware\":true}");
  }
  else if (info.status == fb_esp_fcs_download_status_download)
  {
    Serial.printf("Done %d%s, Time %d ms\n", (int)info.progress, "%", info.elapsedTime);
    char bufferf[100];
    sprintf(bufferf, "Done %d%s", (int)info.progress, "%");
    //oled_display_text(bufferf);
  }
  else if (info.status == fb_esp_fcs_download_status_complete)
  {
    Serial.println("Completed.");
    Serial.println();

    Serial.println("{\"save_last_config\":true}");
    obj["updated"] = true;
    obj["registered"] = false;
    saveConfig = true;
    saveConfigData();
    //loadConfig();
    Serial.println("Restarting...\n\n");
    ESP.restart();

  }
  else if (info.status == fb_esp_fcs_download_status_error)
  {
    Serial.printf("Update fail, %s\n", info.errorMsg.c_str());
  }
}



//------------------------------------------------------------------ connectFirebase
void connectFirebase()
{
  // Firebase
  Serial.println("{\"starting_firebase\":true}");
  esp_task_wdt_reset();
  if (!Firebase.ready()) // Add more filters
  {
    // Timeout, prevent to program halt
    config.timeout.wifiReconnect = 5 * 1000; // 10 Seconds to 2 min (10 * 1000)
    config.timeout.socketConnection = 30 * 1000; // 1 sec to 1 min (30 * 1000)
    config.timeout.sslHandshake = 2 * 60 * 1000; // 1 sec to 2 min (2 * 60 * 1000)
    config.timeout.rtdbKeepAlive = 20 * 1000;    // 20 sec to 2 min (45 * 1000)
    config.timeout.rtdbStreamReconnect = 1 * 1000;  //1 sec to 1 min (1 * 1000)
    config.timeout.rtdbStreamError = 3 * 1000;    // 3 sec to 30 sec (3 * 1000)
    config.timeout.serverResponse = 1 * 1000;    //Server response read timeout in ms 1 sec - 1 min ( 10 * 1000).

    Serial.println("{\"firebase_init\":true}");
    config.api_key = obj["key"].as<String>();

    /* Assign the user sign in credentials */
    auth.user.email = obj["email"].as<String>();
    auth.user.password = obj["firepass"].as<String>();

    /* Assign the RTDB URL (required) */
    config.database_url = obj["url"].as<String>();

    Serial.println("{\"connecting_firebase\":true}");
    Firebase.begin(&config, &auth);
    esp_task_wdt_reset();
    Firebase.reconnectWiFi(true);

    unsigned long startTime = millis();
    esp_task_wdt_reset();
    while (!Firebase.ready())
    {
      if (millis() - startTime > mainTime)
      {
        Serial.println("Failed to connect to Firebase within timeout period");
        break; // Salir del bucle si no se puede conectar a Firebase después de TIMEOUT_DURATION milisegundos
      }
      delay(100); // Esperar un poco antes de comprobar de nuevo, para no bloquear completamente el bucle
    }

  }


  if (updated)
  {
    updated = false;
    String storage_id = obj["storage_id"].as<String>();
    //String key;
     // key = p.key().c_str(); // Obtener la clave
    String firname = obj["firmware"].as<String>();
    String path = "firmware/" + firname;

    // Pasar storage_id y path como const char* usando .c_str()
    const char* c_storage_id = storage_id.c_str();
    const char* c_path = path.c_str();

    Serial.println("{\"new_firmware\":true}");
    Serial.println(c_path);
    if (!Firebase.Storage.downloadOTA(&fbdo, c_storage_id, c_path, fcsDownloadCallback))
      Serial.println(fbdo.errorReason());
  }
}

// ---------------------------------------------------------------------------------------------------- streamTimeoutCallback
void streamTimeoutCallback(bool timeout)
{
  if (!stream.httpConnected())
    Serial.printf("{\"stream_error\":\"%s\"}\n\n", stream.errorReason().c_str());
}
