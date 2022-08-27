#include "LittleFS.h"
//#define DEBUG
template <typename T>
bool liteSettings::save(const T& input_struct) {
  const uint16_t len = sizeof(T);
#ifdef DEBUG
  Serial.printf("\n[save] input_struct len = %d", len);
  Serial.printf("\n[save] filename = %s", filename);
#endif
  File config = LittleFS.open(filename, "w");
  if (!config) {
#ifdef DEBUG
    Serial.println("File open failed");
#endif
    return false;
  }
  uint16_t bytesWrited = config.write((byte*)&input_struct, len);
  config.close();
#ifdef DEBUG
  Serial.printf("\n[save] bytesWrited = %d", bytesWrited);
#endif
  if (bytesWrited != len) {
#ifdef DEBUG
    Serial.println("writen != sizeof");
#endif
    return false;
  } else {
    return true;
  }
}

template <typename T>
bool liteSettings::read(T& input_struct) {
  const uint16_t len = sizeof(T);
#ifdef DEBUG
  Serial.printf("\n[read] input_struct len = %d", len);
  Serial.printf("\n[read] filename = %s", filename);
#endif
  if (!fileEx(filename)) {
#ifdef DEBUG
    Serial.println("[read]Config file not exsist");
#endif
    if (!save(input_struct)) {
      return false;
    }
  }
  File config = LittleFS.open(filename, "r");
  if (!config) {
    return false;
#ifdef DEBUG
    Serial.println("[read]File open failed");
#endif
  }
  uint16_t bytesRead = config.read((byte*)(void*)&input_struct, len);
  config.close();
#ifdef DEBUG
  Serial.printf("\n[read] bytesRead = %d", bytesRead);
#endif
  if (bytesRead != len) {
#ifdef DEBUG
    Serial.println("Read != sizeof");
#endif
    return false;
  } else {
    return true;
  }
}
