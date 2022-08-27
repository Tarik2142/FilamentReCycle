#include "liteSettings.h"
//#define DEBUG

void liteSettings::initFS() {
  if (!LittleFS.begin()) {
#ifdef DEBUG
    Serial.println("[read]SPIFFS mount failed");
#endif
  }
}

bool liteSettings::fileEx(char* file) {
  return LittleFS.exists(file);
}

bool liteSettings::delConf(char* filename) {
  if (fileEx(filename)) {
    return LittleFS.remove(filename);
  } else {
    return false;
  }
}

bool liteSettings::delConf() {
  if (fileEx(filename)) {
    return LittleFS.remove(filename);
  } else {
    return false;
  }
}