# Flic 2 Bridge for ESPHome

ESP32-based bridge to connect Flic 2 Bluetooth buttons to Home Assistant via ESPHome.

## Features

- Auto-discovery of Flic 2 buttons (no manual MAC address configuration)
- Pairing via Home Assistant button or physical ESP32 button (GPIO0)
- Supports click, double-click, and hold events
- Battery level monitoring
- Persistent pairing (survives reboots)

## Installation

Add to your ESPHome YAML:

```yaml
external_components:
  - source: github://xterr/flic2-bridge@main

esp32:
  board: esp32dev
  framework:
    type: esp-idf
    version: recommended
    sdkconfig_options:
      CONFIG_BT_ENABLED: y
      CONFIG_BTDM_CTRL_MODE_BLE_ONLY: y
      CONFIG_BT_NIMBLE_ENABLED: n
      CONFIG_BT_BLUEDROID_ENABLED: y
      CONFIG_BTDM_CTRL_BLE_MAX_CONN: "5"

flic2:
  id: flic2_hub
  pairing_button_pin: GPIO0  # Optional: physical pairing button

button:
  - platform: template
    name: "Flic2 Start Pairing"
    on_press:
      - flic2.start_pairing:
          id: flic2_hub
          duration: 30s

text_sensor:
  - platform: template
    name: "Flic2 Last Event"
    lambda: |-
      return id(flic2_hub)->get_last_event();
    update_interval: 1s
```

## Pairing a Flic 2 Button

1. Press **"Flic2 Start Pairing"** button in Home Assistant
2. Hold your Flic 2 button for **7+ seconds** until LED flashes
3. Wait for pairing to complete
4. Button events will appear in the "Last Event" sensor

## Requirements

- ESP32 board
- ESPHome with ESP-IDF framework (not Arduino)
- Flic 2 buttons

## License

GPL v3 (due to flic2lib dependency)
