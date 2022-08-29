/***********************************************
1. SPI Interface Inatruction
      clockPin --> SCK(EN)
      latchPin --> CS(RS)
      dataPin --> SID(RW)
 2. Connection:
    1)Turn the BL_ON Switch to the "ON" side;
    2)Turn the PBS_ON Switch to the "SPI" side

Method1:
      LCD                   Arduino
      EN                 Digital Pin 2
      RS                 Digital Pin 7
      RW                 Digital Pin 10
      VCC                     5V
      GND                     GND;

Method2:
      LCD                          Arduino
      SCK       clockPin(defined in the "initDriverPin" function)
      CS        latchPin(defined in the "initDriverPin" function)
      SID       dataPin (defined in the "initDriverPin" function)3-cd, 4-cs,
5-ck VCC                            5V GND                           GND
***********************************************/
/*
 * arg 1: pin: Analog pin
 * arg 2: vcc: Input voltage
 * arg 3: analogReference: reference voltage. Typically the same as vcc, but not
 * always (ie ESP8266=1.0) arg 4: adcMax: The maximum analog-to-digital convert
 * value returned by analogRead (1023 or 4095) arg 5: seriesResistor: The ohms
 * value of the fixed resistor (based on your hardware setup, usually 10k) arg
 * 6: thermistorNominal: Resistance at nominal temperature (will be documented
 * with the thermistor, usually 10k)
 * arg 7: temperatureNominal: Temperature for
 * nominal resistance in celcius (will be documented with the thermistor, assume
 * 25 if not stated)
 * arg 8: bCoef: Beta coefficient (or constant) of the
 * thermistor (will be documented with the thermistor, typically 3380, 3435, or
 * 3950)
 * arg 9: samples: Number of analog samples to average (for smoothing) arg
 * 10: sampleDelay: Milliseconds between samples (for smoothing)
 */

#include "ESP8266WiFi.h"
#include "LCD12864RSPI.h"
#include "PID_v1.h"
#include "Ticker.h"
#include "liteSettings.h"
#include "thermistor.h"
//#include "LittleFS.h"

#define DEBOUNCE_TIME 50
#define KP 2
#define KI 5
#define KD 1
//#define DEBUG

// PROGMEM const unsigned char img[] = {
//     0x00, 0x40, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x60, 0x80, 0x80,
//     0x80, 0x80, 0x80, 0x80, 0x00, 0x88, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80,
//     0x80, 0x90, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x50, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x84,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x31, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x49,
//     0x00, 0x00, 0x00, 0x00, 0x06, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x22, 0x49, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x31, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x7E, 0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
//     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
//     0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xED, 0xC0, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0xED, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void updLcd();
void updPid();
void filamentSense();
void btnUp();
void btnDown();
void btnOk();
void updBtn();
void handlelongBtn();
void saveSettings();

volatile uint16_t filamentLen = 0;
double Setpoint, Input, Output;
bool menuItemSelect = 0;
bool motorOn = 0;
bool heaterOn = 1;
const uint8_t pin_pidOut = D0;
const uint8_t pin_pidIn = A0;
const uint8_t pin_stepperPwm = D8;
const uint8_t pin_filamentSense = D1;
const uint8_t pin_btnUp = D2;
const uint8_t pin_btnDown = D3;
const uint8_t pin_btnOk = D4;
const uint16_t WindowSize = 5000;
int8_t menu = 0;

enum BUTTON { BTN_UP,
              BTN_DOWN,
              BTN_OK,
              BTN_NONE };

volatile BUTTON current_btn = BTN_NONE;
BUTTON last_btn = BTN_NONE;

struct settings_t {
    bool autostop;
    int autostopLen;
    uint8_t stepperSpeed;
    uint8_t targetTemp;
} settings = {false, 400, 0, 200};

Ticker* tmrLcdUpd;
Ticker* tmrPidUpd;
Ticker* tmrBtnLongPress;
Ticker* tmrSaveSettings;
PID* extruderPID;
Thermistor* thermistor;
liteSettings Settings;
// Ticker tmrBtnUpd(updBtn, 200 , 0, MILLIS);

void setup() {
    WiFi.mode(WIFI_OFF);
    LCDA.initDriverPin(D7, D6, D5);
    LCDA.Initialise();  // INIT SCREEN
    delay(100);
    // LCDA.DrawFullScreen(img); // LOGO
    LCDA.DisplayString(0, 0, "     LOADING    ", 16);  //
    // delay(5000);
    pinMode(pin_pidOut, OUTPUT);
    pinMode(pin_pidIn, INPUT);
    pinMode(pin_filamentSense, INPUT);
    pinMode(pin_btnUp, INPUT_PULLUP);
    pinMode(pin_btnDown, INPUT_PULLUP);
    pinMode(pin_btnOk, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pin_filamentSense), filamentSense, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_btnOk), btnOk, FALLING);
    attachInterrupt(digitalPinToInterrupt(pin_btnDown), btnDown, FALLING);
    attachInterrupt(digitalPinToInterrupt(pin_btnUp), btnUp, FALLING);
    Setpoint = settings.targetTemp;

    thermistor = new Thermistor(A0, 3.3, 1.0, 1023, 220000, 100000, 25, 3950, 5, 20);
    extruderPID = new PID(&Input, &Output, &Setpoint, KP, KI, KD, DIRECT);
    tmrLcdUpd = new Ticker(updLcd, 1000, 0, MILLIS);
    tmrPidUpd = new Ticker(updPid, 100, 0, MILLIS);
    tmrBtnLongPress = new Ticker(handlelongBtn, 500, 0, MILLIS);
    tmrSaveSettings = new Ticker(saveSettings, 60 * 1000, 0, MILLIS);

    extruderPID->SetOutputLimits(0, WindowSize);
    extruderPID->SetMode(AUTOMATIC);
    tmrLcdUpd->start();
    tmrPidUpd->start();
    const bool readRes = Settings.read(settings);
#ifdef DEBUG
    Serial.begin(115200);
    Serial.printf("SETTINGS REAAD: %d", readRes);
#endif
}

void loop() {
    tmrLcdUpd->update();
    tmrPidUpd->update();
    tmrBtnLongPress->update();
    updBtn();
    tmrSaveSettings->update();
}

//"T:100/100 ONF <-"
//"LEN:1000       <"
//"SPD:1000  ONF  <"
//"AS:1000   ONF <-"

void updLcd() {
    char line1[17];
    char line2[17];
    const char pointer = '<';
    const uint8_t pointerIndex = 14;
    const uint8_t editIndex = 15;
    const char edit = '-';
    const char ON[] = "ON ";
    const char OFF[] = "OFF";
    switch (menu) {
        case 0 ... 1:
            char temp[3];
            dtostrf(Input, 3, 0, temp);
            sprintf(line1, "T:%s/%d %s   ", temp, settings.targetTemp, heaterOn ? ON : OFF);
            sprintf(line2, "LEN:%d          ", filamentLen);
            break;
        // case 1:
        //     sprintf(line1, "LEN:%d         ", filamentLen);
        //     sprintf(line2, "SPD:%d  %s   ", stepperSpeed, motorOn ? ON : OFF);
        //     break;
        case 2 ... 3:
            sprintf(line1, "SPD:%d  %s      ", settings.stepperSpeed, motorOn ? ON : OFF);
            sprintf(line2, "AS:%d   %s    ", settings.autostopLen, settings.autostop ? ON : OFF);
            break;
        default:
            break;
    }

    const bool line = menu % 2;
    if (!line) {
        if (menuItemSelect) {
            line1[editIndex] = edit;
            line1[pointerIndex] = pointer;
        } else {
            line1[editIndex] = pointer;
        }
#ifdef DEBUG
        Serial.println("line 1");
#endif
    } else {
        if (menuItemSelect) {
            line2[editIndex] = edit;
            line2[pointerIndex] = pointer;
        } else {
            line2[editIndex] = pointer;
        }
#ifdef DEBUG
        Serial.println("line 2");
#endif
    }
    LCDA.CLEAR();
    delay(10);
    LCDA.DisplayString(0, 0, line1, 16);
    delay(10);
    LCDA.DisplayString(1, 0, line2, 16);

    // LCDA.DisplayString(0, 0, "  150/200   100m", 16); //
    // LCDA.DisplayString(1, 0, " 100%           ", 16);
    // LCDA.DrawFullScreen(img); // LOGO
}

void updPid() {
    static unsigned long windowStartTime;
    Input = thermistor->readTempC();
    extruderPID->Compute();
    unsigned long now = millis();
    if (now - windowStartTime > WindowSize) {  // time to shift the Relay Window
        windowStartTime += WindowSize;
    }
    if (Output > now - windowStartTime)
        digitalWrite(pin_pidOut, HIGH);
    else
        digitalWrite(pin_pidOut, LOW);
}

//"T:100/100 ONF <-"
//"LEN:1000       <"
//"SPD:1000  ONF  <"
//"AUTOSTOP  ONF <-"

void handleMenu(BUTTON btn, bool longPress) {
    const uint8_t five = 5;
#ifdef DEBUG
    Serial.printf("longPress %d BTN: %d \n", longPress, btn);
#endif
    switch (btn) {
        case BTN_OK:
            if (longPress) {
                switch (menu) {
                    case 0:
                        heaterOn = !heaterOn;
#ifdef DEBUG
                        Serial.println("!heater");
#endif
                        break;

                    case 2:
                        motorOn = !motorOn;
#ifdef DEBUG
                        Serial.println("!motor");
#endif
                        break;

                    case 3:
                        settings.autostop = !settings.autostop;
#ifdef DEBUG
                        Serial.println("!autostop");
#endif
                        tmrSaveSettings->start();
                        break;

                    default:
                        break;
                }
            } else {
                menuItemSelect = !menuItemSelect;
            }

            break;
        case BTN_UP:
            if (menuItemSelect) {
                switch (menu) {
                    case 0:  // extruder temp
                        if (longPress) {
                            settings.targetTemp += five;
#ifdef DEBUG
                            Serial.println("TEMP += 5");
#endif
                        } else {
                            settings.targetTemp++;
#ifdef DEBUG
                            Serial.println("TEMP ++");
#endif
                        }
                        Setpoint = settings.targetTemp;
                        tmrSaveSettings->start();
                        break;
                    case 2:  // extruder speed
                        if (longPress) {
                            settings.stepperSpeed += five;
#ifdef DEBUG
                            Serial.println("STEPPER SPD += 5");
#endif
                        } else {
                            settings.stepperSpeed++;
                        }
                        tmrSaveSettings->start();
                        break;
                    case 1:  // filament len
                        filamentLen = 0;
                        break;
                    case 3:
                        if (longPress) {
                            settings.autostopLen += (five * 2);
                        } else {
                            settings.autostopLen++;
#ifdef DEBUG
                            autostopLen = 400;
#endif
                        }
                        tmrSaveSettings->start();
                        break;
                }
            } else {
                if (menu < 3) menu++;
            }

            break;
        case BTN_DOWN:
            if (menuItemSelect) {
                switch (menu) {
                    case 0:  // extruder temp
                        if (longPress) {
                            settings.targetTemp -= five;
                        } else {
                            settings.targetTemp--;
                        }
                        Setpoint = settings.targetTemp;
                        tmrSaveSettings->start();
                        break;
                    case 2:  // extruder speed
                        if (longPress) {
                            settings.stepperSpeed -= five;
                        } else {
                            settings.stepperSpeed--;
                        }
                        tmrSaveSettings->start();
                        break;
                    case 1:  // filament len
                        filamentLen = 0;
                        break;
                    case 3:
                        if (longPress) {
                            settings.autostopLen -= five;
                        } else {
                            settings.autostopLen--;
                        }
                        tmrSaveSettings->start();
                        break;
                }
            } else {
                if (menu > 0) menu--;
            }
            break;

        default:
            break;
    }
#ifdef DEBUG
    Serial.printf("menu: %d \n", menu);
    if (!menu % 2) {
        Serial.println("!menu % 2");
    } else {
        Serial.println("menu % 2");
    }
#endif
}

void updBtn() {
    if (current_btn != BTN_NONE) {
        handleMenu(current_btn, 0);
        last_btn = current_btn;
        current_btn = BTN_NONE;
        tmrBtnLongPress->start();
    }
}

IRAM_ATTR bool debounce() {
    volatile static unsigned long lastFire = 0;
    if (millis() - lastFire < DEBOUNCE_TIME) {  // Debounce
        return 0;
    }
    lastFire = millis();
    return 1;
}

IRAM_ATTR void filamentSense() {
    volatile static unsigned long lastFire_filament = 0;
    if (millis() - lastFire_filament < DEBOUNCE_TIME) {  // Debounce
        return;
    }
    lastFire_filament = millis();
    filamentLen++;
}

IRAM_ATTR void btnOk() {
    if (debounce()) current_btn = BTN_OK;
}

IRAM_ATTR void btnUp() {
    if (debounce()) current_btn = BTN_UP;
}

IRAM_ATTR void btnDown() {
    if (debounce()) current_btn = BTN_DOWN;
}

void handlelongBtn() {
    if (!digitalRead(pin_btnDown) || !digitalRead(pin_btnOk) || !digitalRead(pin_btnUp)) {
        handleMenu(last_btn, true);
    } else {
        tmrBtnLongPress->stop();
#ifdef DEBUG
        Serial.println("STOP LONG PRESS");
#endif
    }
}

void saveSettings() {
    const bool saveRes = Settings.save(settings);
    tmrSaveSettings->stop();
#ifdef DEBUG
    Serial.printf("SAVE SETTINGS: %d", saveRes);
#endif
}