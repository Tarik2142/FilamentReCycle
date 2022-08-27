// settings manager
// 29.01.2020
// UPD 20.12.2020
// Tarik2142 vk.com/tarik2142 tarik2142@gmail.com

#ifndef LITESETTINGS_H
#define LITESETTINGS_H

#include <Arduino.h>

class liteSettings {
 private:
  void initFS();
  bool fileEx(char* file);

 public:
  char* filename = "liteConfig.cfg";
  liteSettings(char* filename) {
    this->filename = filename;
    initFS();
  };
  liteSettings() {
    initFS();
  };
  template <typename T>
  bool save(const T& input_struct);

  template <typename T>
  bool read(T& input_struct);
  bool delConf(char* filename);
  bool delConf();
};
#include "liteSettings.ipp"
#endif
