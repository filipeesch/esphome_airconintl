import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor, uart
from esphome.const import CONF_ID

DEPENDENCIES = ["climate", "uart"]
AUTO_LOAD = ["sensor"]

# Define sensor configuration keys
CONF_COMPRESSOR_FREQUENCY = "compressor_frequency"
CONF_COMPRESSOR_FREQUENCY_SETTING = "compressor_frequency_setting"
CONF_COMPRESSOR_FREQUENCY_SEND = "compressor_frequency_send"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_OUTDOOR_CONDENSER_TEMPERATURE = "outdoor_condenser_temperature"
CONF_COMPRESSOR_EXHAUST_TEMPERATURE = "compressor_exhaust_temperature"
CONF_TARGET_EXHAUST_TEMPERATURE = "target_exhaust_temperature"
CONF_INDOOR_PIPE_TEMPERATURE = "indoor_pipe_temperature"
CONF_INDOOR_HUMIDITY_SETTING = "indoor_humidity_setting"
CONF_INDOOR_HUMIDITY_STATUS = "indoor_humidity_status"

# Declare the namespace and the C++ class for your component.
aircon_ns = cg.esphome_ns.namespace("aircon_climate")
AirconClimate = aircon_ns.class_("AirconClimate", climate.Climate, cg.Component)

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(AirconClimate),
    cv.GenerateID("uart"): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_COMPRESSOR_FREQUENCY): sensor.sensor_schema(
        unit_of_measurement="Hz",
        accuracy_decimals=0,
        device_class="frequency",
        state_class="measurement",
    ),
    cv.Optional(CONF_COMPRESSOR_FREQUENCY_SETTING): sensor.sensor_schema(
        unit_of_measurement="Hz",
        accuracy_decimals=0,
        device_class="frequency",
        state_class="measurement",
    ),
    cv.Optional(CONF_COMPRESSOR_FREQUENCY_SEND): sensor.sensor_schema(
        unit_of_measurement="Hz",
        accuracy_decimals=0,
        device_class="frequency",
        state_class="measurement",
    ),
    cv.Optional(CONF_OUTDOOR_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_OUTDOOR_CONDENSER_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_COMPRESSOR_EXHAUST_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_TARGET_EXHAUST_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_INDOOR_PIPE_TEMPERATURE): sensor.sensor_schema(
        unit_of_measurement="°C",
        accuracy_decimals=1,
        device_class="temperature",
        state_class="measurement",
    ),
    cv.Optional(CONF_INDOOR_HUMIDITY_SETTING): sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=0,
        device_class="humidity",
        state_class="measurement",
    ),
    cv.Optional(CONF_INDOOR_HUMIDITY_STATUS): sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=0,
        device_class="humidity",
        state_class="measurement",
    ),
}).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

async def to_code(config):
    uart_var = await cg.get_variable(config["uart"])
    var = cg.new_Pvariable(config[CONF_ID], uart_var)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    await climate.register_climate(var, config)
    
    if CONF_COMPRESSOR_FREQUENCY in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_FREQUENCY])
        cg.add(var.set_compressor_frequency_sensor(sens))
    if CONF_COMPRESSOR_FREQUENCY_SETTING in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_FREQUENCY_SETTING])
        cg.add(var.set_compressor_frequency_setting_sensor(sens))
    if CONF_COMPRESSOR_FREQUENCY_SEND in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_FREQUENCY_SEND])
        cg.add(var.set_compressor_frequency_send_sensor(sens))
    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(var.set_outdoor_temperature_sensor(sens))
    if CONF_OUTDOOR_CONDENSER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_CONDENSER_TEMPERATURE])
        cg.add(var.set_outdoor_condenser_temperature_sensor(sens))
    if CONF_COMPRESSOR_EXHAUST_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_EXHAUST_TEMPERATURE])
        cg.add(var.set_compressor_exhaust_temperature_sensor(sens))
    if CONF_TARGET_EXHAUST_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TARGET_EXHAUST_TEMPERATURE])
        cg.add(var.set_target_exhaust_temperature_sensor(sens))
    if CONF_INDOOR_PIPE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_PIPE_TEMPERATURE])
        cg.add(var.set_indoor_pipe_temperature_sensor(sens))
    if CONF_INDOOR_HUMIDITY_SETTING in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_HUMIDITY_SETTING])
        cg.add(var.set_indoor_humidity_setting_sensor(sens))
    if CONF_INDOOR_HUMIDITY_STATUS in config:
        sens = await sensor.new_sensor(config[CONF_INDOOR_HUMIDITY_STATUS])
        cg.add(var.set_indoor_humidity_status_sensor(sens))
