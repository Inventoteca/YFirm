#include "wifiservice.h"

//// -------------------------------------------------------------------- Wifi_disconnected
//void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info)
//{
//  //Serial.println("Disconnected from WIFI access point");
//  //Serial.print("WiFi lost connection. Reason: ");
//  //Serial.println(info.disconnected.reason);
//  String auxssid = obj["ssid"].as<String>();
//  if (auxssid.length() == 0)
//  {
//    Serial.println("Reconnecting...");
//    WiFi.begin(obj["ssid"].as<const char*>(), obj["pass"].as<const char*>());
//  }
//  //return;
//}



// ------------------------------------------------------------------------------------------------------- checkServer
void checkServer()
{

  //if ((millis() - s_timestamp) >= connectTimeoutMs) // check to an interval of time
  {
    String auxssid = obj["ssid"].as<String>();


    // ------------------ Wifi Connected
    if (WiFi.status() == WL_CONNECTED)
    {
      //Serial.println("{\"wifi\":\"connected\"}");
      doc_status["wifi_connected"] = true;
      //if (obj["enable_rtc"].as<bool>())
      //  update_clock();


      if (obj["registered_wifi"] == false)
      {
        obj["registered_wifi"] = true;
        obj["count_wifi"] = 0;

        Serial.println(saveJSonToAFile(&obj, filename) ? "{\"file_saved_new_wifi\":true}" : "{\"file_saved\":false}");
        Serial.print("{\"wifi\":{\"ssid\":\"");
        Serial.print(obj["ssid"].as<const char*>());
        Serial.println("\"}}");
      }

      // MQTT Enable
      mqtt_check();

      //if (!Mclient.connected()) {
      //  reconnect();
      //}
      //Mclient.loop();

      // comprueba que no haya mensajes pendientes de entrada y mantiene la conexión con el server
      //      if (obj["mqtt"]["enable"].as<bool>())
      //      {
      //        if (!client.connected())
      //        {
      //          Serial.println("{\"mqtt_server\":\"reconnect\"}");
      //          blk = false;
      //          reconnect();
      //        }
      //        // Blink led status on printOut
      //        else
      //        {
      //          //Serial.println("{\"mqtt_server\":\"connected\"}");
      //          //if (obj["neodisplay"]["enable"].as<bool>())
      //          //{
      //          //  display1.updatePoint(obj["neodisplay"]["status"].as<int>(), 0, 255, 0);
      //          //  display1.show();
      //          // }
      //          blk = !blk;
      //        }
      //
      //      }

    }

    else //wifi not connected
    {
      doc_status["wifi_connected"] = false;
    }

  }
}



// ----------------------------------------------------------------------------------------- neoConfig
/*void neoConfig()
  {

  //if (wifi_config == false) // Si aun no se inicia la config
  if (!WiFi.smartConfigDone() && (smart_config == false))
  {
    //if (obj["wifi"]["sta"]["count"] > 0)
    //{
    //  obj["wifi"]["sta"]["count"] = 0;
    //  Serial.println(saveJSonToAFile(&obj, filename) ? "{\"file_saved_wifi_count\":true}" : "{\"file_saved\":false}");
    //}

    WiFi.disconnect(true);
    WiFi.onEvent(WiFiEvent);

    //WiFi.mode(WIFI_STA);

    //if (obj["neodisplay"]["enable"].as<bool>())
    //{
    //display1.updatePoint(obj["neodisplay"]["status"], white); //no funciona
    //display1.updatePoint(obj["neodisplay"]["status"], 255, 255, 255); //no funciona, la libreria es u_int

    //display1.updatePoint(obj["neodisplay"]["status"].as<int>(), 255, 255, 255);
    //display1.show();

    //color_status[0] = 255;
    //color_status[1] = 255;
    //color_status[2] = 255;
    //}

    smart_config = WiFi.beginSmartConfig(SC_TYPE_ESPTOUCH_V2);

    Serial.print("{\"SmartConfig\":");
    Serial.print(smart_config);
    Serial.println("}");

    //while (!WiFi.smartConfigDone());
    //if (!wifi_config) ESP.restart();
  }
  //else // Configuracion iniciada
  //{
  //Serial.print("Wait conection response");
  //if (WiFi.smartConfigDone()) // Configuracion correcta
  //{
  //WiFi.stopSmartConfig();
  // wifi_config = false;
  // wifi_trys = 0;
  //Serial.print("SmartConfig Started Done");
  //}

  }*/
