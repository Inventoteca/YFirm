#include "system.h"


void setup()
{
  Serial.begin(115200);
  Serial.println("gpio-test");
  Serial.print("Version:"); Serial.println(VERSION);


  // SPIFFS Init
  if (!SPIFFS.begin(true)) {
    Serial.println("{\"spiffs\":false}");
    return;
  } else {
    Serial.println("{\"spiffs\":true}");
    Cfg_get(/*NULL*/);  // Load File from spiffs
    loadConfig();       // Load and update behaivor of system
  }

  pinMode(cont_monedas, INPUT_PULLUP);
  pinMode(cont_premios, INPUT_PULLUP);
  pinMode(I_maq_onoff, INPUT);

  // WatchDog Timer
  //esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  //esp_task_wdt_add(NULL);                //add current thread to WDT watch

  attachInterrupt(cont_monedas, botonpress_monedas, FALLING);
  attachInterrupt(cont_premios, botonpress_premios, FALLING);
  doc["ver"] = VERSION;

  system_init();

}

void loop()
{
  if (millis() - mainRefresh > mainTime)
  {
    doc_status["maquina_ON"] = bool(digitalRead(I_maq_onoff));
    serializeJson(doc_status,Serial);
    Serial.println();
    //mqtt_send();

    
    mainRefresh = millis();
  }

  if (obj["enable_wifi"].as<bool>())
  {
    // ----------------------------------------- check internet
    if ((millis() - s_timestamp) >= connectTimeoutMs) // check to an interval of time
    {
      checkServer();
      s_timestamp = millis();
    }
  }


  // ----------------------------------------- save new data
  if (saveConfig)  // Data change
  {
    Serial.println("{\"upload_config\":true}");
    saveConfigData();
    loadConfig();
    saveConfig = false;
    //ESP.restart();
  }
}
