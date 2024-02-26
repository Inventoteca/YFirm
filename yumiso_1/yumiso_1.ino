#include "system.h"



// ------------------------------------------------------ (CONFIG) setup
void setup()
{
  system_init();
}


// ------------------------------------------------------ loop
void loop()
{
  // PRead button for report
  buttonState = digitalRead(BT_REPORT);
  status_doc["maquina_ON"] = bool(digitalRead(I_maq_onoff));
  status_doc["enlapsed_time"] = millis() / 1000;
  status_doc["total"] = obj["total"].as<long>();
  status_doc["init_bag"] = obj["init_bag"].as<long>();
  status_doc["bag"] = flag_bolsa;
  status_doc["games"] = obj["games"].as<long>();
  status_doc["total_gift"] = obj["total_gift"].as<long>();
  status_doc["init_gift"] = obj["init_gift"].as<long>();
  status_doc["gift"] = flag_stock;


  // ---------------------------------------------------------------- MAIN TIME
  if (millis() - mainRefresh > mainTime)
  {
    mainRefresh = millis();

    gps_update();
    read_clock();


    // ----------------------------------------- check internet

    if (wifi_check())
    {
      update_clock();
      if (mqtt_check())
      {
        // ------------------------------------------- Send Log STATUS
        if (send_log  == true)
        {
          //mqtt_send_file(file_to_send);
          send_log = false;
          mqtt_send();
          status_doc["status"] = "running";
        }

        // ------------------------------------------- Send STATUS
        //if (send_log == true)
        //{
        //mqtt_send();
        //send_log = false;
        //}

        // ------------------------------------------- Send LIST
        //if (send_list == true)
        //{
        //mqtt_send_list();
        //send_list = false;
        //}
      }
    }

    if (obj["test"].as<bool>() == true)
    {
      serializeJson(status_doc, Serial);
      Serial.println();
    }

  }







  // ----------------------------------------- save new data
  if (saveConfig == true)  // Data change
  {
    saveConfig = false;

    Serial.println("{\"upload_config\":true}");
    saveConfigData();
    loadConfig();
    send_log  = true;     // Enviar por MQTT status
    mainRefresh = millis();
  }


  // leer boton para imprimir reporte diario
  // Si el botón cambia de no presionado a presionado
  if (lastButtonState == HIGH && buttonState == LOW)
  {
    Serial.println("PUSH");
    buttonPressTime = millis();

  }

  // Si el botón cambia de presionado a no presionado
  if (lastButtonState == LOW && buttonState == HIGH)
  {
    if (millis() - buttonPressTime < longPressDuration)
    {
      Serial.println("Short press detected!");
      read_clock();
    }
    else if (millis() - buttonPressTime >= (4 * longPressDuration)) // Reinicio de fabrica
    {
      Serial.println("Super Long press detected!");
      reset_config();

    }
    else if (millis() - buttonPressTime >= longPressDuration) // Reinicia el log
    {

      Serial.println("Long press detected!");
      wifiAP(true);
    }


  }

  lastButtonState = buttonState;

  esp_task_wdt_reset();
}
