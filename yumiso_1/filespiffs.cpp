#include "filespiffs.h"

JsonObject obj;
StaticJsonDocument<FILE_SIZE> doc;
StaticJsonDocument<FILE_SIZE> status_doc;



const char* fileconfig = "/config.json";
const char *filedefault = "/default.json";
String file_to_send;
volatile bool saveConfig = false;

File file;




// ------------------------------------------------------------------------------------- spiffs_init
bool spiffs_init()
{
  // SPIFFS Init
  if (!SPIFFS.begin(true)) 
  {
    Serial.println("{\"spiffs\":false}");
    return false;
  } 
  else 
  {
    Serial.println("{\"spiffs\":true}");
    return true;
  }
}


// ----------------------------------------------------------------------------------------- saveJSonToAFile
bool saveJSonToAFile(fs::FS &fs, JsonObject * doc, const char * path)
{
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  Serial.println(F("Open file in write mode"));
  file = fs.open(path, FILE_WRITE);
  if (file) {
    Serial.print(F("Filename --> "));
    Serial.println(path);

    Serial.print(F("Start write..."));

    serializeJson(*doc, file);

    Serial.print(F("..."));
    //close the file:
    file.close();
    Serial.println(F("done."));

    return true;
  } else {
    // if the file didn't open, print an error:
    Serial.print(F("Error opening "));
    Serial.println(path);
    file.close();
    return false;
  }
}


// ------------------------------------------------------------------------------------------------ getJsonFromFile

JsonObject getJSonFromFile(fs::FS &fs, StaticJsonDocument<FILE_SIZE> *doc, const char * path, bool forceCleanONJsonError)
{
  // open the file for reading:
  //file = LittleFS.open(filename, "r");

  file = fs.open(path);
  if (file)
  {
    Serial.print("Opening File");
    Serial.println(path);

    size_t size = file.size();
    //Serial.println(size);

    if (size > FILE_SIZE)
    {
      Serial.println("Too large file");

    }

    DeserializationError error = deserializeJson(*doc, file);
    if (error)
    {
      // if the file didn't open, print an error:
      Serial.print(F("Error parsing JSON "));
      Serial.println(error.c_str());

      if (forceCleanONJsonError)
      {
        Serial.println("{\"force_empty_json\": true}");
        return doc->to<JsonObject>();
      }
    }

    // close the file:
    file.close();
    Serial.println("{\"json_loaded\": true}");

    return doc->as<JsonObject>();
  } else {
    // if the file didn't open, print an error:
    Serial.print(F("Error opening (or file not exists) "));
    Serial.println(path);

    //Serial.println(F("Empty json created"));
    Serial.println("{\"empty_json\": true}");
    file.close();
    return doc->to<JsonObject>();
  }

}


// --------------------------------------------------------------------------------------------- saveConfigData
void saveConfigData()
{
  Serial.println(saveJSonToAFile(SPIFFS, &obj, fileconfig) ? "{\"config_update_spiffs\":true}" : "{\"conifg_update_spiffs\":false}");
  if (obj["test"].as<bool>())
    serializeJson(obj, Serial);
}
