#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

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

/**
 * Flic2Hub - Central hub component that manages Flic2 buttons.
 *
 * Features:
 * - Auto-discovery of Flic 2 buttons in pairing mode
 * - Automatic pairing (no MAC address configuration needed)
 * - Fires events to Home Assistant for button presses
 * - Physical pairing button support
 * - Persistent storage of paired buttons
 */
class Flic2Hub : public Component {
 public:
  Flic2Hub();

  void setup() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::BLUETOOTH; }

  // Configuration
  void set_pairing_button_pin(InternalGPIOPin *pin) { pairing_button_pin_ = pin; }

  // Pairing control
  void start_pairing(uint32_t duration_seconds = 30);
  void stop_pairing();
  void unpair_all();

  // Status queries (for lambdas in YAML)
  bool is_pairing() const { return pairing_active_; }
  int get_connected_count() const;
  std::string get_paired_buttons_info() const;
  std::string get_last_event() const { return last_event_; }

 protected:
  // Static callbacks from flic2_manager (C code)
  static void on_button_event_static(uint8_t *bd_addr,
                                     enum Flic2EventButtonEventType event_type,
                                     enum Flic2EventButtonEventClass event_class,
                                     bool was_queued);
  static void on_paired_static(uint8_t *bd_addr, const char *name, const char *serial);
  static void on_unpaired_static(uint8_t *bd_addr);
  static void on_battery_update_static(uint8_t *bd_addr, uint16_t millivolt);
  static void on_connection_change_static(uint8_t *bd_addr, bool connected);

  // Instance callbacks
  void on_button_event(const uint8_t *bd_addr,
                       Flic2EventButtonEventType event_type,
                       Flic2EventButtonEventClass event_class,
                       bool was_queued);
  void on_paired(const uint8_t *bd_addr, const char *name, const char *serial);
  void on_unpaired(const uint8_t *bd_addr);
  void on_battery_update(const uint8_t *bd_addr, uint16_t millivolt);
  void on_connection_change(const uint8_t *bd_addr, bool connected);

  // Fire event to Home Assistant
  void fire_event(const std::string &event_type, const uint8_t *bd_addr,
                  const char *name = nullptr, uint16_t battery_mv = 0);

  // Helper to format MAC address
  static std::string format_mac(const uint8_t *bd_addr);

  // Generate button name from MAC
  static std::string generate_button_name(const uint8_t *bd_addr);

  // Physical pairing button handling
  void check_pairing_button();

  // Singleton for static callbacks
  static Flic2Hub *instance_;

  // Configuration
  InternalGPIOPin *pairing_button_pin_{nullptr};

  // State
  bool initialized_{false};
  bool pairing_active_{false};
  uint32_t pairing_end_time_{0};
  std::string last_event_;

  // Physical button state
  bool button_pressed_{false};
  uint32_t button_press_start_{0};
  static const uint32_t LONG_PRESS_MS = 3000;  // 3 seconds for pairing

  // Track button info for status
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

// ============================================
// Actions for automation
// ============================================

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
