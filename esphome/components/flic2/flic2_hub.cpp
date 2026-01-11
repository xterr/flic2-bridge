#include "flic2_hub.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <cstring>

namespace esphome {
namespace flic2 {

static const char *const TAG = "flic2";

// Singleton instance for static callbacks
Flic2Hub *Flic2Hub::instance_ = nullptr;

Flic2Hub::Flic2Hub() {
  instance_ = this;
}

void Flic2Hub::setup() {
  ESP_LOGE(TAG, "######### FLIC2 BUILD 2025-01-11-v1 #########");

  // Setup physical pairing button if configured
  if (pairing_button_pin_ != nullptr) {
    pairing_button_pin_->setup();
    pairing_button_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    ESP_LOGI(TAG, "Physical pairing button configured on GPIO%d", pairing_button_pin_->get_pin());
  }

  ESP_LOGI(TAG, "Calling flic2_manager_init...");

  // Set up callbacks for flic2_manager
  flic2_callbacks_t callbacks = {
      .on_button_event = on_button_event_static,
      .on_paired = on_paired_static,
      .on_unpaired = on_unpaired_static,
      .on_battery_update = on_battery_update_static,
      .on_connection_change = on_connection_change_static,
  };

  // Initialize the Flic2 manager
  esp_err_t err = flic2_manager_init(&callbacks);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize Flic2 manager: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "flic2_manager_init succeeded!");
  initialized_ = true;

  // Load paired buttons from storage
  uint8_t paired_addrs[FLIC2_MAX_BUTTONS][6];
  int paired_count = flic2_manager_get_paired(paired_addrs, FLIC2_MAX_BUTTONS);

  for (int i = 0; i < paired_count; i++) {
    // Get button name from storage
    char name_buf[24] = {0};
    if (!flic2_manager_get_name(paired_addrs[i], name_buf)) {
      // Generate name from MAC if not stored
      snprintf(name_buf, sizeof(name_buf), "Flic %02X%02X",
               paired_addrs[i][4], paired_addrs[i][5]);
    }

    ButtonInfo *info = add_button(paired_addrs[i], name_buf, nullptr);

    // Try to connect to paired button
    flic2_manager_connect(paired_addrs[i]);
  }

  ESP_LOGI(TAG, "Flic2 Hub initialized with %d paired buttons", paired_count);
}

void Flic2Hub::loop() {
  if (!initialized_) {
    return;
  }

  uint32_t now = millis();

  // Process Flic2 manager tasks
  flic2_manager_loop();

  // Check pairing timeout
  if (pairing_active_ && now >= pairing_end_time_) {
    stop_pairing();
  }

  // Check physical pairing button
  check_pairing_button();
}

void Flic2Hub::dump_config() {
  ESP_LOGCONFIG(TAG, "Flic2 Hub:");
  if (pairing_button_pin_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Pairing Button: GPIO%d", pairing_button_pin_->get_pin());
  }
  ESP_LOGCONFIG(TAG, "  Paired Buttons: %d", buttons_.size());
  for (const auto &button : buttons_) {
    ESP_LOGCONFIG(TAG, "    - %s (%s) %s",
                  button.name.c_str(),
                  format_mac(button.mac).c_str(),
                  button.connected ? "Connected" : "Disconnected");
  }
}

void Flic2Hub::start_pairing(uint32_t duration_seconds) {
  if (!initialized_) {
    ESP_LOGW(TAG, "Cannot start pairing - not initialized");
    return;
  }

  ESP_LOGI(TAG, "Starting pairing mode for %u seconds...", duration_seconds);

  pairing_active_ = true;
  pairing_end_time_ = millis() + (duration_seconds * 1000);

  flic2_manager_start_scan(duration_seconds);
}

void Flic2Hub::stop_pairing() {
  if (pairing_active_) {
    ESP_LOGI(TAG, "Stopping pairing mode");
    pairing_active_ = false;
    flic2_manager_stop_scan();
  }
}

void Flic2Hub::unpair_all() {
  ESP_LOGW(TAG, "Unpairing all buttons...");

  for (auto &button : buttons_) {
    flic2_manager_unpair(button.mac);
  }

  buttons_.clear();
  ESP_LOGI(TAG, "All buttons unpaired");
}

int Flic2Hub::get_connected_count() const {
  int count = 0;
  for (const auto &button : buttons_) {
    if (button.connected) {
      count++;
    }
  }
  return count;
}

std::string Flic2Hub::get_paired_buttons_info() const {
  if (buttons_.empty()) {
    return "No buttons paired";
  }

  std::string info;
  for (size_t i = 0; i < buttons_.size(); i++) {
    if (i > 0) info += ", ";
    info += buttons_[i].name;
    info += buttons_[i].connected ? " ✓" : " ✗";
  }
  return info;
}

void Flic2Hub::check_pairing_button() {
  if (pairing_button_pin_ == nullptr) {
    return;
  }

  // Button is active LOW (pulled up, pressed = low)
  bool pressed = !pairing_button_pin_->digital_read();
  uint32_t now = millis();

  if (pressed && !button_pressed_) {
    // Button just pressed
    button_pressed_ = true;
    button_press_start_ = now;
  } else if (!pressed && button_pressed_) {
    // Button released
    button_pressed_ = false;

    // Check if it was a long press
    if ((now - button_press_start_) >= LONG_PRESS_MS) {
      if (pairing_active_) {
        stop_pairing();
      } else {
        start_pairing(30);
      }
    }
  }
}

// ============================================
// Static Callbacks (from C code)
// ============================================

void Flic2Hub::on_button_event_static(uint8_t *bd_addr,
                                       enum Flic2EventButtonEventType event_type,
                                       enum Flic2EventButtonEventClass event_class,
                                       bool was_queued) {
  if (instance_) {
    instance_->on_button_event(bd_addr, event_type, event_class, was_queued);
  }
}

void Flic2Hub::on_paired_static(uint8_t *bd_addr, const char *name, const char *serial) {
  if (instance_) {
    instance_->on_paired(bd_addr, name, serial);
  }
}

void Flic2Hub::on_unpaired_static(uint8_t *bd_addr) {
  if (instance_) {
    instance_->on_unpaired(bd_addr);
  }
}

void Flic2Hub::on_battery_update_static(uint8_t *bd_addr, uint16_t millivolt) {
  if (instance_) {
    instance_->on_battery_update(bd_addr, millivolt);
  }
}

void Flic2Hub::on_connection_change_static(uint8_t *bd_addr, bool connected) {
  if (instance_) {
    instance_->on_connection_change(bd_addr, connected);
  }
}

// ============================================
// Instance Callbacks
// ============================================

void Flic2Hub::on_button_event(const uint8_t *bd_addr,
                                Flic2EventButtonEventType event_type,
                                Flic2EventButtonEventClass event_class,
                                bool was_queued) {
  ButtonInfo *button = find_button(bd_addr);
  const char *name = button ? button->name.c_str() : nullptr;
  uint16_t battery = button ? button->battery_mv : 0;

  // Map event type to event name
  const char *event_name = nullptr;
  switch (event_type) {
    case FLIC2_EVENT_BUTTON_EVENT_TYPE_UP:
      event_name = "flic2_up";
      break;
    case FLIC2_EVENT_BUTTON_EVENT_TYPE_DOWN:
      event_name = "flic2_down";
      break;
    case FLIC2_EVENT_BUTTON_EVENT_TYPE_CLICK:
      event_name = "flic2_click";
      break;
    case FLIC2_EVENT_BUTTON_EVENT_TYPE_SINGLE_CLICK:
      event_name = "flic2_single_click";
      break;
    case FLIC2_EVENT_BUTTON_EVENT_TYPE_DOUBLE_CLICK:
      event_name = "flic2_double_click";
      break;
    case FLIC2_EVENT_BUTTON_EVENT_TYPE_HOLD:
      event_name = "flic2_hold";
      break;
  }

  if (event_name) {
    ESP_LOGI(TAG, "Button %s: %s", name ? name : format_mac(bd_addr).c_str(), event_name);
    fire_event(event_name, bd_addr, name, battery);
  }
}

void Flic2Hub::on_paired(const uint8_t *bd_addr, const char *name, const char *serial) {
  ESP_LOGI(TAG, "Button paired: %s (%s)", name ? name : "unnamed", serial ? serial : "");

  // Add to our list if not already present
  ButtonInfo *info = find_button(bd_addr);
  if (!info) {
    info = add_button(bd_addr, name, serial);
  } else {
    if (name) info->name = name;
    if (serial) info->serial = serial;
  }

  // Fire paired event
  fire_event("flic2_paired", bd_addr, name, 0);

  // Stop pairing mode after successful pair
  stop_pairing();
}

void Flic2Hub::on_unpaired(const uint8_t *bd_addr) {
  ESP_LOGW(TAG, "Button unpaired: %s", format_mac(bd_addr).c_str());

  // Remove from our list
  for (auto it = buttons_.begin(); it != buttons_.end(); ++it) {
    if (memcmp(it->mac, bd_addr, 6) == 0) {
      fire_event("flic2_unpaired", bd_addr, it->name.c_str(), 0);
      buttons_.erase(it);
      break;
    }
  }
}

void Flic2Hub::on_battery_update(const uint8_t *bd_addr, uint16_t millivolt) {
  ButtonInfo *button = find_button(bd_addr);
  if (button) {
    button->battery_mv = millivolt;
    ESP_LOGD(TAG, "Button %s battery: %u mV", button->name.c_str(), millivolt);

    // Fire battery event
    fire_event("flic2_battery", bd_addr, button->name.c_str(), millivolt);
  }
}

void Flic2Hub::on_connection_change(const uint8_t *bd_addr, bool connected) {
  ButtonInfo *button = find_button(bd_addr);
  if (button) {
    button->connected = connected;
    ESP_LOGI(TAG, "Button %s %s", button->name.c_str(),
             connected ? "connected" : "disconnected");

    fire_event(connected ? "flic2_connected" : "flic2_disconnected",
               bd_addr, button->name.c_str(), 0);
  }
}

// ============================================
// Event Firing
// ============================================

void Flic2Hub::fire_event(const std::string &event_type, const uint8_t *bd_addr,
                           const char *name, uint16_t battery_mv) {
  std::string mac_str = format_mac(bd_addr);
  std::string button_name = name ? name : generate_button_name(bd_addr);

  // Log the event - HA can trigger automations based on ESPHome logs or state changes
  ESP_LOGI(TAG, "Event: %s | button=%s | mac=%s | battery=%umV",
           event_type.c_str(), button_name.c_str(), mac_str.c_str(), battery_mv);

  // Store last event for text sensor access - HA can watch this state
  last_event_ = event_type + " | " + button_name;

  // Note: ESPHome events to HA can be implemented later using:
  // 1. ESPHome's event component (esphome 2024.1+)
  // 2. Custom services that HA can listen to
  // 3. State change triggers on the last_event text sensor
}

// ============================================
// Helpers
// ============================================

std::string Flic2Hub::format_mac(const uint8_t *bd_addr) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           bd_addr[5], bd_addr[4], bd_addr[3],
           bd_addr[2], bd_addr[1], bd_addr[0]);
  return std::string(buf);
}

std::string Flic2Hub::generate_button_name(const uint8_t *bd_addr) {
  char buf[16];
  snprintf(buf, sizeof(buf), "Flic %02X%02X", bd_addr[4], bd_addr[5]);
  return std::string(buf);
}

Flic2Hub::ButtonInfo *Flic2Hub::find_button(const uint8_t *bd_addr) {
  for (auto &button : buttons_) {
    if (memcmp(button.mac, bd_addr, 6) == 0) {
      return &button;
    }
  }
  return nullptr;
}

Flic2Hub::ButtonInfo *Flic2Hub::add_button(const uint8_t *bd_addr, const char *name, const char *serial) {
  // Check if already exists
  ButtonInfo *existing = find_button(bd_addr);
  if (existing) {
    return existing;
  }

  // Add new
  ButtonInfo info;
  memcpy(info.mac, bd_addr, 6);
  info.name = name ? name : generate_button_name(bd_addr);
  info.serial = serial ? serial : "";
  info.connected = false;
  info.battery_mv = 0;

  buttons_.push_back(info);
  return &buttons_.back();
}

}  // namespace flic2
}  // namespace esphome
