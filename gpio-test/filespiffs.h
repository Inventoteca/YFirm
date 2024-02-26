#ifndef FILESPIFFS_H
#define FILESPIFFS_H

#include  "system.h"

bool spiffs_init();
JsonObject getJSonFromFile(/*DynamicJsonDocument *doc*/ StaticJsonDocument<FILE_SIZE> *doc, String filename,bool forceCleanONJsonError = true);
/*static*/ void Cfg_get(/*struct jsonrpc_request * r*/);
bool saveJSonToAFile(JsonObject * doc, String filename);
void saveConfigData();

#endif  // FILESPIFFS_H
