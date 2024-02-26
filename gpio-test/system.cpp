#include "system.h"

// ---------------------- variables para spifss
JsonObject obj;

StaticJsonDocument<FILE_SIZE> doc;
StaticJsonDocument<FILE_SIZE> doc_status;
const char* filename = "/config.json";
const char *filedefault = "/default.json";
volatile bool saveConfig = false;
File file;

// --------------------- variables para contadorer
long flag_moneda;
long flag_premio;
long flag_bolsa;

//------------------------- variables para mqtt
WiFiClient espClient;
PubSubClient Mclient(espClient);
WiFiClient client_http;
const char* publish_topic = "/mensajes";
const char* subcribe_topic = "/puertas";
char buffer_union_publish[BUFFER_TOPIC_SIZE];       // solo para armar el topico, puede ser menor
char buffer_union_subcribe[BUFFER_TOPIC_SIZE];      // solo para armar el topico, puede ser menor
char buffer_msg[BUFFER_MSG_SIZE];                 // buffer para el mensaje entrante y saliente
const char* client_id = "maquina00018";


// ---------------------------- variables para wifi
bool correct = false;
int wifi_trys;
boolean isSaved = false;


// -------------------------------- variables generales
bool factory_press = false;
unsigned long factory_time = 0;
unsigned long prev_factory_time = 0;
bool reset_time = false;
int buttonState = 0;


unsigned long mainRefresh = obj["mainTime"].as<uint32_t>();
unsigned long mainTime = 1000;

const uint32_t connectTimeoutMs = 10000;
unsigned long  s_timestamp;


void system_init()
{
  //delay(100);
  Serial.begin(115200);
  //delay(5000);
  //I2C_Init(); Serial.println("i2c_Init");// Slave mode
  Serial.println("Yumiso");
  Serial.print("Version:"); Serial.println(VERSION);

  if (spiffs_init())
  {
    loadConfig();       // Load and update behaivor of system
    //mqtt_init();
    //wifi_init();
    //mqtt_check();
    //rtcUpdated = false;
    //ntpConnected = false;
    //init_clock();        // I2C for clock
  }
  //gps_init();

  // WatchDog Timer
  //esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  //esp_task_wdt_add(NULL);
  //pinMode(BT_REPORT, INPUT);
}

// ----------------------------------------------------------------------------------------------- factory_reset3 change
/*void IRAM_ATTR factory_reset3()
  {
  if ((factory_press == false) && (digitalRead(FACTORY_BT) == PRESS))
  {
    Serial.println("{\"R_bt\":\"push\"}");
    factory_press = true;
    factory_time = millis();
    return;

  }
  else if((digitalRead(FACTORY_BT) == !PRESS))
  {
    prev_factory_time = millis();
    reset_time = true;
    //Serial.println("{\"reset_button\":\"released\"}");
    return;
  }

  }*/


// --------------------------------------------------------------------------------------------- check_reset
/*void check_reset()
  {
  // Force Factory to input
  pinMode(FACTORY_BT, INPUT);

  if (reset_time)
  {
    Serial.print("{\"reset_time\":"); Serial.print(prev_factory_time - factory_time); Serial.println("}");
    if ((prev_factory_time - factory_time) > 5000)
    {
      reset_config();
    }
    else
      Serial.println("{\"reset\":\"fail\"}");
    factory_press = false;
    reset_time = false;
  }

  // ------------------------------------------------------reboot time es en horas
  int reboot_time = obj["reboot_time"].as<unsigned int>();
  if (reboot_time < 1)
    reboot_time = 24;
  // Si han pasado más de 24 horas del reset anterior o el tiempo en reboot time
  if (millis() - tiempoInicio >=  (reboot_time * 60 * 60 * 1000))
    //if (millis() - tiempoInicio >=  (reboot_time  * 1000))
  { // Comparar el tiempo actual con el tiempo de inicio
    Serial.print("{\"reboot_time\":"); Serial.print(obj["reboot_time"].as<unsigned int>()); Serial.println("}");
    tiempoInicio = millis();  // Actualizar el tiempo de inicio
    ESP.restart();  // Reiniciar el ESP32
  }

  //------------------------------------------------------ restart from command
  if (obj["restart"].as<bool>())
  {
    Serial.println("{\"reboot\":true}");
    SendData();
    obj["restart"] = false;
    Serial.println(saveJSonToAFile(&obj, filename) ? "{\"reboot_save\":true}" : "{\"reboot_save\":false}");
    //delay(2000);
    ESP.restart();
  }

  }*/

//----------------------------------------------------------------------------------------------------------- reset_config
void reset_config()
{
  WiFi.disconnect();
  //WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);

  //obj["ssid"] = "";
  //obj["pass"] = "";
  //obj["enable_wifi"] = true;
  //obj["count_wifi"] = 0;
  //obj["registered_wifi"] = false;
  obj = getJSonFromFile(&doc, filedefault);
  Serial.println(saveJSonToAFile(&obj, filename) ? "{\"factory_reset\":true}" : "{\"factory_reset\":false}");
  delay(2000);
  ESP.restart();

}


// ------------------------------------------- strtoBool
/*bool strToBool(String str)
  {
  if (str == "true" || str == "1") {
    return true;
  } else if (str == "false" || str == "0") {
    return false;
  } else {
    // handle invalid input
    return false;
  }
  }*/

// ---------------------------------------------------------------------------------------------------- loadConfig
// Update a new config in the file an change behivor
void loadConfig()
{
  // ----------- Load Counters
  flag_moneda = obj["total"];
  flag_premio = obj["gift"];
  //flag_bolsa = obj["bag"];
  flag_bolsa = (obj["total"].as<long>()) - (obj["init_bag"].as<long>());

  doc_status["bolsa"] = flag_bolsa;
  doc_status["total"] = obj["total"];
  doc_status["init_bag"] = obj["init_bag"];
  doc_status["gift"] = obj["gift"];

  // ------------- ID
  String s_aux = obj["id"].as<String>();
  int len = s_aux.length();
  // check for id or mac is the config.json file
  if ((len == 0))
  {
    //(i_aux == 0)
    //Serial.println("{\"update_id\":true}");
    doc_status["update_id"] = true;
    Serial.print("{\"ID\":\"");
    Serial.print(WiFi.macAddress());
    Serial.println("\"}");

    obj["id"].set( WiFi.macAddress());

    Serial.println(saveJSonToAFile(&obj, filename) ? "{\"id_file_saved\":true}" : "{\"id_file_saved\":false}" );
  }

  // ----------------------- WiFi STA
  if (obj["enable_wifi"].as<bool>() == true && (WiFi.status() != WL_CONNECTED))
  {
    WiFi.mode(WIFI_STA);
    String auxssid = obj["ssid"].as<String>();
    String auxpass = obj["pass"].as<String>();
    doc_status["wifi_ebale"] = true; 

    WiFi.begin(obj["ssid"].as<const char*>(), obj["pass"].as<const char*>());
    Serial.print("{\"wifi\":{\"ssid\":\"");
    Serial.print(obj["ssid"].as<const char*>());
    Serial.println("\"}}");
    doc_status["wifi_ssid"] = obj["ssid"];

  }
  else if (obj["enable_wifi"].as<bool>() == false)
  {
    //
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    //Serial.println("{\"wifi\":{\"enable\":false}}");
    doc_status["wifi_ebale"] = false;
  }



  //----------------- RTC
  // if (obj["enable_rtc"].as<bool>())
  // {
  //   rtcUpdated = false;
  //   ntpConnected = false;
  //   init_clock();
  //}


  //------------------- mqtt
  if (obj["enable_mqtt"].as<bool>())
  {
    mqtt_init();
  }

  //if (obj["enable_mqtt"].as<bool>())
  //{
  //client.setBufferSize(1024);
  //client.setServer(obj["mqtt_server"].as<const char*>(), obj["mqtt_port"].as<unsigned int>());
  //client.setServer(obj["mqtt"]["broker"].as<const char*>(),1883);
  //client.setServer("inventoteca.com", 1883);
  //client.setCallback(callback);
  // Serial.println(obj["mqtt"]["port"].as<unsigned int>());

  //}


  // -------------------------------- mainTime
  // refres time
  JsonVariant objTime = obj["mainTime"];
  if (objTime.isNull())
  {
    Serial.println("{\"mainTime\":NULL}");
    mainTime = 1000;
  }
  else
  {
    mainTime = obj["mainTime"].as<uint32_t>();
    Serial.print("{\"mainTime\":");
    Serial.print(mainTime);
    Serial.println("}");
  }
  mainRefresh = mainTime + 1;


  Serial.println("{\"config\":true}");

}
