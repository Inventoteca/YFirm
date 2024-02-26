#ifndef FILESPIFFS_H
#define FILESPIFFS_H

#define FILE_SIZE       2048
#define STATUS_SIZE     2048

#include "system.h"


extern JsonObject obj;
extern StaticJsonDocument<FILE_SIZE> doc;
extern StaticJsonDocument<FILE_SIZE> status_doc;


extern const char *fileconfig;
extern const char *filedefault;
extern String file_to_send;
extern volatile bool saveConfig;

extern File file;

JsonObject getJSonFromFile(fs::FS &fs,StaticJsonDocument<FILE_SIZE> *doc, const char * path,bool forceCleanONJsonError = true);
bool saveJSonToAFile(fs::FS &fs,JsonObject * doc, const char * path);
void saveConfigData();
bool spiffs_init();


#endif  // FILESPIFFS_H
