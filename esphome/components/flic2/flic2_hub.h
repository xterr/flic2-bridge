#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#ifdef USE_API
#include "esphome/components/api/custom_api_device.h"
#endif

#include <vector>
#include <string>

extern "C" {
#include "flic2_manager.h"
}

namespace esphome {
namespace flic2 {

class Flic2Hub : public Component, public esp32_ble_tracker::ESPBTDeviceListener {
 public:
  Flic2Hub();

  void setup() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  void set_pairing_button_pin(InternalGPIOPin *pin) { pairing_button_pin_ = pin; }

  void start_pairing(uint32_t duration_seconds = 30);
  void stop_pairing();
  void unpair_all();

  bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override;
  void on_scan_end() override;

  bool is_pairing() const { return pairing_active_; }
  int get_connected_count() const;
  std::string get_paired_buttons_info() const;
  std::string get_last_event() const { return last_event_; }

 protected:
  static void on_button_event_static(uint8_t *bd_addr,
                                     enum Flic2EventButtonEventType event_type,
                                     enum Flic2EventButtonEventClass event_class,
                                     bool was_queued);
  static void on_paired_static(uint8_t *bd_addr, const char *name, const char *serial);
  static void on_unpaired_static(uint8_t *bd_addr);
  static void on_battery_update_static(uint8_t *bd_addr, uint16_t millivolt);
  static void on_connection_change_static(uint8_t *bd_addr, bool connected);

  void on_button_event(const uint8_t *bd_addr,
                       Flic2EventButtonEventType event_type,
                       Flic2EventButtonEventClass event_class,
                       bool was_queued);
  void on_paired(const uint8_t *bd_addr, const char *name, const char *serial);
  void on_unpaired(const uint8_t *bd_addr);
  void on_battery_update(const uint8_t *bd_addr, uint16_t millivolt);
  void on_connection_change(const uint8_t *bd_addr, bool connected);

  void fire_event(const std::string &event_type, const uint8_t *bd_addr,
                  const char *name = nullptr, uint16_t battery_mv = 0);

  static std::string format_mac(const uint8_t *bd_addr);
  static std::string generate_button_name(const uint8_t *bd_addr);

  void check_pairing_button();

  static Flic2Hub *instance_;

  InternalGPIOPin *pairing_button_pin_{nullptr};

  bool initialized_{false};
  bool pairing_active_{false};
  uint32_t pairing_end_time_{0};
  std::string last_event_;

  bool button_pressed_{false};
  uint32_t button_press_start_{0};
  static const uint32_t LONG_PRESS_MS = 3000;

  struct ButtonInfo {
    uint8_t mac[6];
    std::string name;
    std::string serial;
    bool connected;
    uint16_t battery_mv;
  };
  std::vector<ButtonInfo> buttons_;

  ButtonInfo *find_button(const uint8_t *bd_addr);
  ButtonInfo *add_button(const uint8_t *bd_addr, const char *name, const char *serial);
};

template<typename... Ts>
class StartPairingAction : public Action<Ts...>, public Parented<Flic2Hub> {
 public:
  void set_duration(uint32_t duration) { duration_ = duration; }

  void play(Ts... x) override {
    this->parent_->start_pairing(duration_);
  }

 protected:
  uint32_t duration_{30};
};

template<typename... Ts>
class StopPairingAction : public Action<Ts...>, public Parented<Flic2Hub> {
 public:
  void play(Ts... x) override {
    this->parent_->stop_pairing();
  }
};

template<typename... Ts>
class UnpairAllAction : public Action<Ts...>, public Parented<Flic2Hub> {
 public:
  void play(Ts... x) override {
    this->parent_->unpair_all();
  }
};

}  // namespace flic2
}  // namespace esphome
