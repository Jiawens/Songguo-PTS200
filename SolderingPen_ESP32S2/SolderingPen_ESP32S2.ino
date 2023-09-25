#include <Wire.h>

#include "SparkFun_LIS2DH12.h" 
#include <U8g2lib.h>

#include <ESP32AnalogRead.h>
#include "esp_adc_cal.h"

#include <PID_v1.h>

#include "PTS200_16.h"

#define SCREEN_OFFSET     3

#define BUTTON_DELAY      5

#define TIP_TEMP_PIN      1
#define VIN_PIN           6
#define BUZZER_PIN        3
#define BUTTON_PIN        0
#define BUTTON_P_PIN      4
#define BUTTON_N_PIN      2
#define CONTROL_PIN       5

#define PD_CFG_0          16
#define PD_CFG_1          17
#define PD_CFG_2          18

#define CONTROL_CHANNEL   2
#define CONTROL_FREQ      200
#define CONTROL_FREQ_20V  1000
#define CONTROL_RES       8

#define TIME2SETTLE       5000
#define TIME2SETTLE_20V   2000

#define WAKEUP_THRESHOLD  10

#define ACCEL_SAMPLES 32
#define TIP_TEMP_SAMPLES 16

#define BUTTON_PRESSED(x) (!digitalRead(x))
#define WAIT_UNTIL_BUTTON_RELEASED(x) while(debounced_button_pressed(x)) {;}

bool debounced_button_pressed(uint8_t pin) {
  if(!BUTTON_PRESSED(pin)) {
    return false;
  }
  delay(BUTTON_DELAY);
  return BUTTON_PRESSED(pin);
}

double aggKp = 11, aggKi = 0.5, aggKd = 1;
double consKp = 11, consKi = 3, consKd = 5;

uint16_t SleepTemp = 50;
uint16_t BaseTemp = 320;
uint16_t BoostTempOffset = 100;

uint16_t time2sleep = 10;
uint16_t time2off = 30;
uint16_t time2base = 60;

volatile bool handleMoved;

double tip_pwm_out, CurrentTemp, TargetTemp;

enum {
  ModeOff,
  ModeSleep,
  ModeBase,
  ModeBoost
} mode;

PID ctrl(&CurrentTemp, &tip_pwm_out, &TargetTemp, consKp, consKi, consKd, P_ON_E, DIRECT);

SPARKFUN_LIS2DH12 accel;

U8G2_SH1107_64X128_F_HW_I2C u8g2(U8G2_R1, /* reset= */ 7);

ESP32AnalogRead adc_tip_temp;
ESP32AnalogRead adc_vin;

void setup() {
  Serial.begin(115200);

  adc_tip_temp.attach(TIP_TEMP_PIN);
  adc_vin.attach(VIN_PIN);

  pinMode(TIP_TEMP_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_P_PIN, INPUT_PULLUP);
  pinMode(BUTTON_N_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(PD_CFG_0, OUTPUT);
  pinMode(PD_CFG_1, OUTPUT);
  pinMode(PD_CFG_2, OUTPUT);

  PD_Request(3);

  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(AUTOMATIC);

  Serial.println("Soldering Pen");
  Wire.begin();
  Wire.setClock(400000);
  if (accel.begin() == false) {
    delay(500);
    Serial.println("Accelerometer not detected.");
  }

  u8g2.begin();
  u8g2.enableUTF8Print();
}

void loop() {
  static uint32_t sleep_time_ms = 0;
  static uint32_t base_time_ms = 0;
  static uint32_t boost_time_ms = 0;

  handleMoved = handleMoved || accel_check(); //Like SET in a RS Latch
  CurrentTemp = read_tip_temp();

  switch(mode){
    case ModeOff:
      if(debounced_button_pressed(BUTTON_PIN)) {
        WAIT_UNTIL_BUTTON_RELEASED(BUTTON_PIN);
        mode = ModeBase;
        TargetTemp = BaseTemp;
        ctrl.SetTunings(consKp, consKi, consKd);
        base_time_ms = millis();
      }
    break;
    case ModeSleep:
      if(debounced_button_pressed(BUTTON_PIN)) {
        WAIT_UNTIL_BUTTON_RELEASED(BUTTON_PIN);
        mode = ModeBase;
        TargetTemp = BaseTemp;
        ctrl.SetTunings(consKp, consKi, consKd);
        base_time_ms = millis();
      }
      if(handleMoved) {
        handleMoved = false;
        mode = ModeBase;
        ctrl.SetTunings(consKp, consKi, consKd);
        base_time_ms = millis();
      }
      if(millis() - sleep_time_ms > time2off * 1000) {
        mode = ModeOff;
        TargetTemp = 0;
      }
    break;
    case ModeBase:
      if(debounced_button_pressed(BUTTON_PIN)) {
        WAIT_UNTIL_BUTTON_RELEASED(BUTTON_PIN);
        long m = millis();
        while((!debounced_button_pressed(BUTTON_PIN)) && millis()-m<300) {
          ;
        }
        if (millis()-m<300) {
          mode = ModeOff;
          WAIT_UNTIL_BUTTON_RELEASED(BUTTON_PIN);
          break;
        } else {
          mode = ModeBoost;
          TargetTemp = BaseTemp + BoostTempOffset;
          ctrl.SetTunings(aggKp, aggKi, aggKd);
          boost_time_ms = millis();
        }
      }
      if(handleMoved) {
        handleMoved = false;
        base_time_ms = millis();
      }
      if(millis() - base_time_ms > time2sleep * 1000) {
        mode = ModeSleep;
        TargetTemp = SleepTemp;
        sleep_time_ms = millis();
      }
    break;
    case ModeBoost:
      if(debounced_button_pressed(BUTTON_PIN)) {
        WAIT_UNTIL_BUTTON_RELEASED(BUTTON_PIN);
        mode = ModeBase;
        TargetTemp = BaseTemp;
        ctrl.SetTunings(consKp, consKi, consKd);
        base_time_ms = millis();
      }
      if(millis() - boost_time_ms > time2base * 1000) {
        mode = ModeBase;
        TargetTemp = BaseTemp;
        ctrl.SetTunings(consKp, consKi, consKd);
        base_time_ms = millis();
      }
    break;
  }

  ctrl.Compute();
  ledc_set_duty(tip_pwm_out);

  refresh_screen();
}

bool accel_check() {
  static uint16_t accels[32][3];
  static uint8_t accelIndex = 0;

  if (accel.available()) {
    accels[accelIndex][0] = accel.getRawX() + 32768;
    accels[accelIndex][1] = accel.getRawY() + 32768;
    accels[accelIndex][2] = accel.getRawZ() + 32768;
    accelIndex++;
    if (accelIndex == ACCEL_SAMPLES) {
      accelIndex = 0;
      uint64_t avg[3] = {0, 0, 0};
      for (int i = 0; i < ACCEL_SAMPLES; i++) {
        avg[0] += accels[i][0];
        avg[1] += accels[i][1];
        avg[2] += accels[i][2];
      }
      avg[0] /= ACCEL_SAMPLES;
      avg[1] /= ACCEL_SAMPLES;
      avg[2] /= ACCEL_SAMPLES;
      uint64_t var[3] = {0, 0, 0};
      for (int i = 0; i < ACCEL_SAMPLES; i++) {
        var[0] += (accels[i][0] - avg[0]) * (accels[i][0] - avg[0]);
        var[1] += (accels[i][1] - avg[1]) * (accels[i][1] - avg[1]);
        var[2] += (accels[i][2] - avg[2]) * (accels[i][2] - avg[2]);
      }
      var[0] /= ACCEL_SAMPLES;
      var[1] /= ACCEL_SAMPLES;
      var[2] /= ACCEL_SAMPLES;

      int varThreshold = WAKEUP_THRESHOLD * 10000;

      if (var[0] > varThreshold || var[1] > varThreshold || var[2] > varThreshold) {
        return true;
      }
    }
  }
  return false;
}

uint16_t read_tip_temp() {
  static uint16_t sample_window[TIP_TEMP_SAMPLES];
  static uint8_t sample_window_index = 0;

  ledc_set_duty(0);
  if (abs(getVin_mv() / 1000 - 20)<=1) {
    delayMicroseconds(TIME2SETTLE_20V);
  } else {
    delayMicroseconds(TIME2SETTLE);
  }
  sample_window[sample_window_index] = constrain(0.4432 * adc_tip_temp.readMiliVolts() + 29.665, 20, 1000);
  ledc_set_duty(tip_pwm_out);
  sample_window_index++;
  if(sample_window_index == TIP_TEMP_SAMPLES) {
    sample_window_index = 0;
  }
  uint16_t ret=0;
  for(uint8_t i=0;i<TIP_TEMP_SAMPLES;i++) {
    ret += sample_window[i];
  }
  return ret/TIP_TEMP_SAMPLES;
}

void refresh_screen() {
  u8g2.clearBuffer();
  u8g2.setFont(PTS200_16);
  u8g2.setFontPosTop();
  u8g2.setCursor(0, 0 + SCREEN_OFFSET);
  u8g2.print("@ ");
  u8g2.print(TargetTemp, 0);

  u8g2.setCursor(112, 0 + SCREEN_OFFSET);
  u8g2.print(F(":)"));

  uint16_t vin_v = getVin_mv() / 1000;
  u8g2.setCursor(0, 50);
  u8g2.print((uint16_t)accel.getTemperature());
  u8g2.print(F("C"));
  u8g2.setCursor(99, 50);
  u8g2.printf("%02d", (uint16_t)vin_v);
  u8g2.print(F("V"));
  
  u8g2.drawBox(33, 50, (tip_pwm_out+1)/4, 16);

  u8g2.setFont(u8g2_font_freedoomr25_tn);
  u8g2.setFontPosTop();
  u8g2.setCursor(37, 18);
  if (CurrentTemp > 500 || CurrentTemp == 45)
    u8g2.print(F("000"));
  else
    u8g2.printf("%03d", (uint16_t)CurrentTemp);
  u8g2.sendBuffer();
}

uint16_t getVin_mv() {
  uint16_t vin = 0;
  for (uint8_t i = 0; i < 4; i++) {
    vin += adc_vin.readMiliVolts();
  }
  vin = vin / 4;
  vin = vin * 31.3;
  return vin;
}

void PD_Request(uint8_t v) {
  switch (v) {
    case 0:
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, LOW);
      digitalWrite(PD_CFG_2, LOW);
      break;
    case 1:
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, LOW);
      digitalWrite(PD_CFG_2, HIGH);
      break;
    case 2:
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, HIGH);
      break;
    case 3:
      digitalWrite(PD_CFG_0, LOW);
      digitalWrite(PD_CFG_1, HIGH);
      digitalWrite(PD_CFG_2, LOW);
      break;
  }

  ledc_init(abs(getVin_mv() / 1000 - 20)<=1?CONTROL_FREQ_20V:CONTROL_FREQ);
}

void ledc_init(uint16_t freq) {
  ledcSetup(CONTROL_CHANNEL, freq, CONTROL_RES);
  ledcAttachPin(CONTROL_PIN, CONTROL_CHANNEL);
  ledc_set_duty(0);
}

void ledc_set_duty(uint8_t duty) {
  ledcWrite(CONTROL_CHANNEL, duty);
}