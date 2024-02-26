#include "system.h"


bool buttonState = LOW;
bool lastButtonState = LOW;
unsigned long buttonPressTime = 0;
const unsigned long longPressDuration = 5000;

bool factory_press = false;
unsigned long factory_time = 0;
unsigned long prev_factory_time = 0;
bool reset_time = false;
volatile bool found_client = false;


unsigned long mainRefresh = obj["mainTime"].as<uint32_t>();
unsigned long mainTime = 1000;

// firebase
const uint32_t connectTimeoutMs = 10000;
unsigned long  s_timestamp;
unsigned long startTime = 0;



// ----------------------------------------GPS intervalos para gps
unsigned long previousMillisGPS = 0;  // Variable para almacenar la última vez que se ejecutó el evento
const long intervalGPS = 60000;  // Intervalo en milisegundos (60,000 milisegundos = 1 minuto)
unsigned long currentMillisGPS;
String gps_name_file;
String gps_str;

// ------------------------------------- wifi flag
bool server_running = false;

volatile uint32_t pesos;


// -----------------------------------------------------------system_ init
void system_init()
{

  delay(100);
  Serial.begin(115200);
  Serial.println("Yumiso");
  Serial.print("Version:"); Serial.println(VERSION);
  //delay(5000);


  delay(100);
  I2C_Init();
  Serial.println("i2c_Init");


  status_doc["ver"] = VERSION;



  if (spiffs_init())
  {
    Cfg_get();  // Load File from spiffs
    loadConfig();       // Load and update behaivor of system
    wifi_init();
    mqtt_init();
    mqtt_check();
    rtcUpdated = false;
    ntpConnected = false;
    init_clock();        // I2C for clock
  }

  gps_init();

  // WatchDog Timer
  esp_task_wdt_init(WDT_TIMEOUT, true);  //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);

  pinMode(cont_monedas, INPUT_PULLUP);
  pinMode(cont_premios, INPUT_PULLUP);
  pinMode(I_maq_onoff, INPUT);               //add current thread to WDT watch

  attachInterrupt(cont_monedas, botonpress_monedas, FALLING);
  attachInterrupt(cont_premios, botonpress_premios, FALLING);
  pinMode(BT_REPORT, INPUT_PULLUP);


  buttonState = LOW;
  lastButtonState = HIGH;
  send_log = true;            // Send initial value on start
  status_doc["status"] = "power on";
}

//----------------------------------------------------------------------------------------------------------- reset_config
void reset_config()
{
  obj.clear();
  obj = getJSonFromFile(SPIFFS, &doc, filedefault);
  saveConfig = true;
  Serial.println("{\"default_config\":true}");
  saveConfigData();
  ESP.restart();
}


// --------------------------------------------------------------------------------------------------- Cfg_get
/*static*/ void Cfg_get(/*struct jsonrpc_request * r*/)
//  {"method":"Config.Get"}
{
  // ----------------------------------------------------- open file to load config
  obj = getJSonFromFile(SPIFFS, &doc, fileconfig);
  if (obj.size() == 0)
  {
    Serial.println("{\"config_file\":\"empty\"}");
    obj = getJSonFromFile(SPIFFS, &doc, filedefault);
    Serial.println(saveJSonToAFile(SPIFFS, &obj, fileconfig) ? "{\"file_default_restore\":true}" : "{\"file_default_restore\":false}");
  }




  if (obj["test"].as<bool>() == true)
  {
    // Comment for production
    serializeJson(obj, Serial);
    Serial.println();

  }

}



// ---------------------------------------------------------------------------------------------------- loadConfig
// Update a new config in the file an change behivor
void loadConfig()
{
  // ----------- Load Counters
  Serial.println("{\"loadConfig\":true}");

  rtc_enable = obj["enable_rtc"].as<bool>();
  ntp_enable = obj["enable_ntp"].as<bool>();
  flag_moneda = obj["total"].as<long>();
  flag_premio = obj["total_gift"].as<long>();
  flag_bolsa = (obj["total"].as<long>()) - (obj["init_bag"].as<long>());
  flag_stock = (obj["total_gift"].as<long>()) - (obj["init_gift"].as<long>());
  

  if (obj["costo"].isNull())
  {
    obj["costo"] = 1;
    saveConfig = true;
  }
  
  costo = obj["costo"].as<long>();

  updated = obj["updated"].as<bool>();

  if (/*(!obj["reboot"].isNull()) && */(obj["reboot"].as<bool>() == true))
  {
    obj["reboot"] = false;
    Serial.println("{\"reboot_upload\":true}");
    saveConfigData();
    Serial.println("{\"reboot\":true}");
    ESP.restart();
  }

  // ------------- ID
  String s_aux = obj["id"].as<String>();
  status_doc["id"] = obj["id"];
  int len = s_aux.length();
  // check for id or mac is the config.json file
  if ((len == 0))
  {
    //(i_aux == 0)
    Serial.println("{\"update_id\":true}");
    Serial.print("{\"ID\":\"");
    //Serial.print(WiFi.macAddress());
    Serial.println("\"}");

    //obj["id"].set( WiFi.macAddress());

    Serial.println(saveJSonToAFile(SPIFFS, &obj, fileconfig) ? "{\"id_file_saved\":true}" : "{\"id_file_saved\":false}" );
  }


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
