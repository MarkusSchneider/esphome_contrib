import esphome.codegen as cg
from esphome.components import sensor, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_INTERVAL

CODEOWNERS = ["@MarkusSchneider"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

mbus_ns = cg.esphome_ns.namespace("mbus")
MBus = mbus_ns.class_("MBus", cg.Component)
MBusSensor = mbus_ns.class_("MBusSensor", cg.Component, sensor.Sensor)
MULTI_CONF = False

CONF_MBUS_ID = "mbus_id"
CONF_SECONDARY_ADDRESS = "secondary_address"
CONF_DATA_INDEX = "data_index"
CONF_FACTOR = "factor"
CONF_SENSORS = "sensors"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_MBUS_ID): cv.declare_id(MBus),
            cv.Optional(CONF_SECONDARY_ADDRESS, default=0): cv.hex_uint64_t,
            cv.Optional(CONF_INTERVAL, default="1min"): cv.positive_time_period_seconds,
            cv.Optional(CONF_SENSORS): cv.ensure_list(
                sensor.sensor_schema(MBusSensor)
                .extend(cv.COMPONENT_SCHEMA)
                .extend(
                    {
                        cv.Required(CONF_DATA_INDEX): cv.positive_int,
                        cv.Optional(CONF_FACTOR, default=0.0): cv.positive_float,
                    }
                )
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    cg.add_global(mbus_ns.using)
    var = cg.new_Pvariable(config[CONF_MBUS_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_secondary_address(config[CONF_SECONDARY_ADDRESS]))
    cg.add(var.set_interval(config[CONF_INTERVAL]))

    if CONF_SENSORS in config:
        for sens_config in config[CONF_SENSORS]:
            sens = cg.new_Pvariable(
                sens_config[CONF_ID],
                sens_config[CONF_DATA_INDEX],
                sens_config[CONF_FACTOR],
            )
            await cg.register_component(sens, sens_config)
            await sensor.register_sensor(sens, sens_config)
            cg.add(var.add_sensor(sens))
