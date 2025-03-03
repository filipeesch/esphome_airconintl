import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

AUTO_LOAD = ["sensor", "climate"]

# Declare the namespace and the C++ class for your component.
aircon_ns = cg.esphome_ns.namespace("aircon_climate")
AirconClimate = aircon_ns.class_("AirconClimate", climate.Climate, cg.Component)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(AirconClimate),
    cv.GenerateID("uart"): cv.use_id(uart.UARTComponent),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uart_var = await cg.get_variable(config["uart"])
    var = cg.new_Pvariable(config[CONF_ID], uart_var)

    await cg.register_component(var, config)
    await climate.register_climate(var, config)
