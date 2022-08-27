#include <Arduino.h>
#include "liteSettings.h"

struct test_t {
  char val[32];
} test = {"TEST"};

liteSettings settings;

void setup() {
  Serial.begin(115200);
  settings.read(test);
  Serial.printf("\ntest.val=%s", test.val);
  //delay(1000);
  strcpy(test.val, "NEW5");
  settings.save(test);
}

void loop() {
  // put your main code here, to run repeatedly:
}
