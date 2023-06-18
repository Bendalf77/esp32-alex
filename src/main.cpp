#define USE_WIFI
#define USE_ULP

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#ifdef USE_ULP
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "esp32s2/ulp.h"
#endif
#include <Arduino.h>
#ifdef USE_WIFI
#include <WiFi.h>
#include "private.h"
#endif

constexpr uint8_t BTN_PIN = 0;
constexpr uint8_t LED_PIN = 21;

constexpr bool BTN_LEVEL = LOW;
constexpr bool LED_LEVEL = LOW;

constexpr uint32_t BLINK_TIME = 25; // 25 ms.
#ifdef USE_ULP
constexpr uint32_t SLEEP_TIME = 50000; // 1/20 sec.

enum { BTN_HOLD, RTC_DATA_MAX };
#endif

static void rtc_gpio_input_init(gpio_num_t pin) {
  rtc_gpio_init(pin);
  rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis(pin);
  rtc_gpio_pulldown_dis(pin);
  rtc_gpio_hold_en(pin);
}

/*
static void rtc_gpio_output_init(gpio_num_t pin) {
  rtc_gpio_init(pin);
  rtc_gpio_set_direction(pin, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_pullup_dis(pin);
  rtc_gpio_pulldown_dis(pin);
}

static void gpio_input_init(gpio_num_t pin) {
  gpio_config_t cfg;

  cfg.pin_bit_mask = BIT(pin);
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&cfg);
}
*/

static void gpio_output_init(gpio_num_t pin) {
  gpio_config_t cfg;

  cfg.pin_bit_mask = BIT(pin);
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&cfg);
}

#ifdef USE_ULP
static esp_err_t ulp_init() {
  enum { OK_BTN, EXIT, WAKE_UP, WAKE_WAIT, RETURN };

  constexpr uint8_t BTN_RTC = BTN_PIN;

  constexpr uint16_t DATA_OFFSET = 0;
  constexpr uint16_t CODE_OFFSET = RTC_DATA_MAX;
  constexpr uint16_t DEBOUNCE = 2;

  const ulp_insn_t program[] = {
    I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, RTC_GPIO_IN_NEXT_S + BTN_RTC),

    I_MOVR(R1, R0),
    I_MOVI(R2, DATA_OFFSET),
    I_MOVI(R3, 0),
    I_ANDI(R0, R1, BIT(BTN_RTC)),
    M_BXZ(OK_BTN),
    I_MOVI(R0, 0),
    I_STL(R0, R2, BTN_HOLD),
    M_BX(EXIT),

    M_LABEL(OK_BTN),
    I_LDL(R0, R2, BTN_HOLD),
    M_BG(EXIT, DEBOUNCE - 1),
    I_ADDI(R0, R0, 1),
    I_STL(R0, R2, BTN_HOLD),
    M_BL(EXIT, DEBOUNCE),
    I_MOVI(R3, 1),

    M_LABEL(EXIT),
    I_SUBI(R3, R3, 0),
    M_BXZ(RETURN),

    M_LABEL(WAKE_UP),
    I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
    I_ANDI(R0, R0, 1),
    M_BXZ(WAKE_WAIT),
    M_BX(RETURN),

    M_LABEL(WAKE_WAIT),
    I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP_S, RTC_CNTL_RDY_FOR_WAKEUP_S),
    I_ANDI(R0, R0, 1),
    M_BXZ(WAKE_WAIT),
    I_WAKE(),

    M_LABEL(RETURN),
    I_HALT()
  };

  size_t size = sizeof(program) / sizeof(ulp_insn_t);
  esp_err_t err;

  RTC_SLOW_MEM[BTN_HOLD] = 0;
  err = ulp_process_macros_and_load(CODE_OFFSET, program, &size);
  if (err == ESP_OK) {
    ulp_set_wakeup_period(0, SLEEP_TIME);
    err = ulp_run(CODE_OFFSET);
  }
  return err;
}
#endif

void setup() {
  gpio_output_init((gpio_num_t)LED_PIN);
//  gpio_set_level((gpio_num_t)LED_PIN, LED_LEVEL);
  rtc_gpio_input_init((gpio_num_t)BTN_PIN);

  Serial.begin(115200);
  Serial.println();

#ifdef USE_ULP
  if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
    esp_err_t err;

    err = ulp_init();
    if (err == ESP_OK) {
      Serial.println("Init ULP successfull");
    } else {
      Serial.printf("ULP init error 0x%X!\n", err);
      Serial.flush();
//      esp_restart();
      esp_deep_sleep_start();
    }
  } else {
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP)
      Serial.println("Waked up by button (ULP)");
  }
#endif

  for (uint8_t i = 0; i < 5; ++i) { // 5 sec.
    gpio_set_level((gpio_num_t)LED_PIN, LED_LEVEL);
    delay(BLINK_TIME);
    gpio_set_level((gpio_num_t)LED_PIN, ! LED_LEVEL);
    delay(1000 - BLINK_TIME);
  }

#ifdef USE_WIFI
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  while (! WiFi.isConnected()) {
    Serial.print('.');
    gpio_set_level((gpio_num_t)LED_PIN, LED_LEVEL);
    delay(BLINK_TIME);
    gpio_set_level((gpio_num_t)LED_PIN, ! LED_LEVEL);
    delay(500 - BLINK_TIME);
  }
  Serial.print(" OK (IP ");
  Serial.print(WiFi.localIP());
  Serial.println(')');
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
#endif

  Serial.println("Going to sleep...");
  Serial.flush();

#ifdef USE_ULP
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_sleep_enable_ulp_wakeup();
#endif
  esp_sleep_enable_timer_wakeup(15000000); // 15 sec.
  esp_sleep_config_gpio_isolate();
  esp_deep_sleep_disable_rom_logging();
  esp_deep_sleep_start();
}

void loop() {}
