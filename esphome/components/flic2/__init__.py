import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins, automation
from esphome.components import esp32_ble_tracker
from esphome.const import CONF_ID, CONF_DURATION
from esphome.core import coroutine_with_priority

DEPENDENCIES = ["esp32_ble_tracker"]
AUTO_LOAD = ["api"]
CODEOWNERS = ["@xterr"]

CONF_PAIRING_BUTTON_PIN = "pairing_button_pin"

flic2_ns = cg.esphome_ns.namespace("flic2")
Flic2Hub = flic2_ns.class_(
    "Flic2Hub",
    cg.Component,
    esp32_ble_tracker.ESPBTDeviceListener,
)

StartPairingAction = flic2_ns.class_("StartPairingAction", automation.Action)
StopPairingAction = flic2_ns.class_("StopPairingAction", automation.Action)
UnpairAllAction = flic2_ns.class_("UnpairAllAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Flic2Hub),
            cv.Optional(CONF_PAIRING_BUTTON_PIN): pins.gpio_input_pin_schema,
        }
    )
    .extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


@coroutine_with_priority(1.0)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await esp32_ble_tracker.register_ble_device(var, config)

    if CONF_PAIRING_BUTTON_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_PAIRING_BUTTON_PIN])
        cg.add(var.set_pairing_button_pin(pin))


@automation.register_action(
    "flic2.start_pairing",
    StartPairingAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Flic2Hub),
            cv.Optional(CONF_DURATION, default="30s"): cv.positive_time_period_seconds,
        }
    ),
)
async def start_pairing_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_duration(config[CONF_DURATION].total_seconds))
    return var


@automation.register_action(
    "flic2.stop_pairing",
    StopPairingAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Flic2Hub),
        }
    ),
)
async def stop_pairing_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "flic2.unpair_all",
    UnpairAllAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(Flic2Hub),
        }
    ),
)
async def unpair_all_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
