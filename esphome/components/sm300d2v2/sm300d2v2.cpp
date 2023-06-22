#include "sm300d2v2.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sm300d2v2 {
static const char *const TAG = "sm300d2v2";
static const uint8_t SM300D2_RESPONSE_LENGTH = 17;

void SM300D2Sensor::update() {
  uint8_t response[SM300D2_RESPONSE_LENGTH];
  uint8_t peeked;
  // uint8_t previous_byte = 0;
  // adding timer for trigger between datasets.
  uint32_t previous_time = 0;
  // uint32_t start_time = millis();

  while (this->available() > 0) {
    this->peek_byte(&peeked);

    // Check if approx half-second delay has elapsed since the previous byte
    if (millis() - previous_time >= 300 && peeked == 0x01) {
      break;  // Exit the loop if the desired byte and delay are satisfied
    }

    // previous_byte = peeked;
    previous_time = millis();
    this->read();
    // Reset the start time whenever a byte is registered
    // start_time = millis();
  }
  //Determining the number of bytes recieved and setting SM300D2_RESPONSE_LENGTH to make correction for v2 hardware
  size_t SM300D2_RESPONSE_LENGTH = sizeof(response) / sizeof(response[0]);

  // Reading in the bytes into response
  uint8_t last_16_bytes[16];
  // bool read_success = read_array(response, SM300D2_RESPONSE_LENGTH);
  bool read_success = read_array(last_16_bytes, sizeof(last_16_bytes));
  // If no data recieved within 1 sec. Throw error
  if (!read_success) {
    ESP_LOGW(TAG, "Reading data from SM300D2 failed!");
    status_set_warning();
    return;
  }

  this->status_clear_warning();
  ESP_LOGD(TAG, "Successfully read SM300D2 data %u", last_16_bytes);

  // Sensor Data Only
  const uint16_t co2 = (last_16_bytes[0] * 256) + last_16_bytes[1];
  const uint16_t formaldehyde = (last_16_bytes[2] * 256) + last_16_bytes[3];
  const uint16_t tvoc = (last_16_bytes[4] * 256) + last_16_bytes[5];
  const uint16_t pm_2_5 = (last_16_bytes[6] * 256) + last_16_bytes[7];
  const uint16_t pm_10_0 = (last_16_bytes[8] * 256) + last_16_bytes[9];
  // A negative value is indicated by adding 0x80 (128) to the temperature value
  const float temperature = ((last_16_bytes[10] + (last_16_bytes[11] * 0.1f)) > 128)
                                ? (((last_16_bytes[10] + (last_16_bytes[11] * 0.1f)) - 128) * -1)
                                : last_16_bytes[10] + (last_16_bytes[11] * 0.1f);
  const float humidity = last_16_bytes[12] + (last_16_bytes[13] * 0.1);

  // Report out to log and publish to HA
  // ESP_LOGD(TAG, "Received Addr: %u", addr);
  // if (this->addr_sensor_ != nullptr)
  //  this->addr_sensor_->publish_state(addr);
  // ESP_LOGD(TAG, "Received Function Type: %u", function);
  // if (this->function_sensor_ != nullptr)
  //  this->function_sensor_->publish_state(function);
  ESP_LOGD(TAG, "Received CO₂: %u ppm", co2);
  if (this->co2_sensor_ != nullptr)
    this->co2_sensor_->publish_state(co2);
  ESP_LOGD(TAG, "Received Formaldehyde: %u µg/m³", formaldehyde);
  if (this->formaldehyde_sensor_ != nullptr)
    this->formaldehyde_sensor_->publish_state(formaldehyde);
  ESP_LOGD(TAG, "Received TVOC: %u µg/m³", tvoc);
  if (this->tvoc_sensor_ != nullptr)
    this->tvoc_sensor_->publish_state(tvoc);
  ESP_LOGD(TAG, "Received PM2.5: %u µg/m³", pm_2_5);
  if (this->pm_2_5_sensor_ != nullptr)
    this->pm_2_5_sensor_->publish_state(pm_2_5);
  ESP_LOGD(TAG, "Received PM10: %u µg/m³", pm_10_0);
  if (this->pm_10_0_sensor_ != nullptr)
    this->pm_10_0_sensor_->publish_state(pm_10_0);
  ESP_LOGD(TAG, "Received Temperature: %.2f °C", temperature);
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);
  ESP_LOGD(TAG, "Received Humidity: %.2f percent", humidity);
  if (this->humidity_sensor_ != nullptr)
    this->humidity_sensor_->publish_state(humidity);
}

void SM300D2Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "SM300D2:");
  LOG_SENSOR("  ", "CO2", this->co2_sensor_);
  LOG_SENSOR("  ", "Formaldehyde", this->formaldehyde_sensor_);
  LOG_SENSOR("  ", "TVOC", this->tvoc_sensor_);
  LOG_SENSOR("  ", "PM2.5", this->pm_2_5_sensor_);
  LOG_SENSOR("  ", "PM10", this->pm_10_0_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  this->check_uart_settings(9600);
}

}  // namespace sm300d2v2
}  // namespace esphome
