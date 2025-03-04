#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "commands.h"

namespace esphome
{
    namespace aircon_climate
    {

        static const char *TAG = "aircon_climate.component";

        typedef struct _Device_Status
        {
            uint8_t header[16];

            uint8_t wind_status; // air volume
            uint8_t sleep_status;

            uint8_t direction_status : 2; // wind direction
            uint8_t run_status : 2;
            uint8_t mode_status : 4;

            // 4
            uint8_t indoor_temperature_setting;
            uint8_t indoor_temperature_status;
            uint8_t indoor_pipe_temperature;
            // 7
            int8_t indoor_humidity_setting;
            int8_t indoor_humidity_status;

            uint8_t somatosensory_temperature; // sensible temperature
            // 10
            uint8_t somatosensory_compensation_ctrl : 3;
            uint8_t somatosensory_compensation : 5;
            // 11
            uint8_t temperature_Fahrenheit : 3; // fahrenheit display

            uint8_t temperature_compensation : 5;

            // 12
            uint8_t timer;

            // 13
            uint8_t hour;
            // 14
            uint8_t minute;
            // 15
            uint8_t poweron_hour;
            // 16
            uint8_t poweron_minute;
            // 17
            uint8_t poweroff_hour;
            // 18
            uint8_t poweroff_minute;
            // 19
            uint8_t wind_door : 4;
            uint8_t drying : 4;
            // 20
            uint8_t dual_frequency : 1;
            uint8_t efficient : 1;
            uint8_t low_electricity : 1; // save electricity
            uint8_t low_power : 1;       // energy saving
            uint8_t heat : 1;            // heating air
            uint8_t nature : 1;          // natural wind
            uint8_t left_right : 1;      // horizontal swing
            uint8_t up_down : 1;         // vertical swing

            // 21
            uint8_t smoke : 1; // smoke removal
            uint8_t voice : 1;
            uint8_t mute : 1;
            uint8_t smart_eye : 1;
            uint8_t outdoor_clear : 1; // outdoor cleaning
            uint8_t indoor_clear : 1;  // indoor cleaning
            uint8_t swap : 1;          // Change the wind
            uint8_t dew : 1;           // fresh

            // 22
            uint8_t indoor_electric : 1;
            uint8_t right_wind : 1;
            uint8_t left_wind : 1;
            uint8_t filter_reset : 1;
            uint8_t indoor_led : 1;
            uint8_t indicate_led : 1;
            uint8_t display_led : 1;
            uint8_t back_led : 1;

            // 23
            uint8_t indoor_eeprom : 1; // eeprom
            uint8_t sample : 1;
            uint8_t rev23 : 4;
            uint8_t time_lapse : 1;
            uint8_t auto_check : 1; // self-test

            // 24
            uint8_t indoor_outdoor_communication : 1;
            uint8_t indoor_zero_voltage : 1;
            uint8_t indoor_bars : 1;
            uint8_t indoor_machine_run : 1;
            uint8_t indoor_water_pump : 1;
            uint8_t indoor_humidity_sensor : 1;
            uint8_t indoor_temperature_pipe_sensor : 1;
            uint8_t indoor_temperature_sensor : 1;

            // 25
            uint8_t rev25 : 3;
            uint8_t eeprom_communication : 1;
            uint8_t electric_communication : 1;
            uint8_t keypad_communication : 1;
            uint8_t display_communication : 1;

            // 26
            uint8_t compressor_frequency;
            // 27
            uint8_t compressor_frequency_setting;
            // 28
            uint8_t compressor_frequency_send;
            // 29
            int8_t outdoor_temperature;
            // 30
            int8_t outdoor_condenser_temperature;
            // 31
            int8_t compressor_exhaust_temperature;
            // 32
            int8_t target_exhaust_temperature;
            // 33
            uint8_t expand_threshold;
            // 34
            uint8_t UAB_HIGH;
            // 35
            uint8_t UAB_LOW;
            // 36
            uint8_t UBC_HIGH;
            // 37
            uint8_t UBC_LOW;
            // 38
            uint8_t UCA_HIGH;
            // 39
            uint8_t UCA_LOW;
            // 40
            uint8_t IAB;
            // 41
            uint8_t IBC;
            // 42
            uint8_t ICA;
            // 43
            uint8_t generatrix_voltage_high;
            // 44
            uint8_t generatrix_voltage_low;
            // 45
            uint8_t IUV;
            // 46
            uint8_t wind_machine : 3;
            uint8_t outdoor_machine : 1;
            uint8_t four_way : 1;
            uint8_t rev46 : 3;

            // 47
            uint8_t rev47;
            // 48
            uint8_t rev48;
            // 49
            uint8_t rev49;
            // 50
            uint8_t rev50;
            // 51
            uint8_t rev51;
            // 52
            uint8_t rev52;
            // 53
            uint8_t rev53;
            // 54
            uint8_t rev54;
            // 55
            uint8_t rev55;
            // 56
            uint8_t rev56;

            uint8_t extra[6];
            uint16_t chk_sum;
            uint8_t foooter[2];
        } Device_Status;

        class AirconClimate : public PollingComponent, public climate::Climate, public uart::UARTDevice
        {
        public:
            AirconClimate(uart::UARTComponent *parent) : PollingComponent(30000),
                                                         UARTDevice(parent) {}

            void set_compressor_frequency_sensor(sensor::Sensor *sensor)
            {
                this->compressor_frequency = sensor;
            }

            void set_compressor_frequency_setting_sensor(sensor::Sensor *sensor)
            {
                this->compressor_frequency_setting = sensor;
            }

            void set_compressor_frequency_send_sensor(sensor::Sensor *sensor)
            {
                this->compressor_frequency_send = sensor;
            }

            void set_outdoor_temperature_sensor(sensor::Sensor *sensor)
            {
                this->outdoor_temperature = sensor;
            }

            void set_outdoor_condenser_temperature_sensor(sensor::Sensor *sensor)
            {
                this->outdoor_condenser_temperature = sensor;
            }

            void set_compressor_exhaust_temperature_sensor(sensor::Sensor *sensor)
            {
                this->compressor_exhaust_temperature = sensor;
            }

            void set_target_exhaust_temperature_sensor(sensor::Sensor *sensor)
            {
                this->target_exhaust_temperature = sensor;
            }

            void set_indoor_pipe_temperature_sensor(sensor::Sensor *sensor)
            {
                this->indoor_pipe_temperature = sensor;
            }

            void set_indoor_humidity_setting_sensor(sensor::Sensor *sensor)
            {
                this->indoor_humidity_setting = sensor;
            }

            void set_indoor_humidity_status_sensor(sensor::Sensor *sensor)
            {
                this->indoor_humidity_status = sensor;
            }

            void setup() override
            {
                // compressor_frequency.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // compressor_frequency_setting.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // compressor_frequency_send.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // outdoor_temperature.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // outdoor_condenser_temperature.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // compressor_exhaust_temperature.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // target_exhaust_temperature.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // indoor_pipe_temperature.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // indoor_humidity_setting.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                // indoor_humidity_status.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
                update();
            }

            void dump_config() override
            {
                ESP_LOGCONFIG(TAG, "Aircon Climate component");
            }

            void loop() override
            {
                int msg_size = 0;

                while (available())
                {
                    msg_size = update_status(read());

                    if (msg_size > 0)
                    {
                        ESP_LOGD(TAG, "compf: %d compf_set: %d compf_snd: %d",
                                 status->compressor_frequency,
                                 status->compressor_frequency_setting,
                                 status->compressor_frequency_send);

                        ESP_LOGD(TAG, "out_temp: %d out_cond_temp: %d comp_exh_temp: %d comp_exh_temp_tgt: %d",
                                 status->outdoor_temperature,
                                 status->outdoor_condenser_temperature,
                                 status->compressor_exhaust_temperature,
                                 status->target_exhaust_temperature);

                        ESP_LOGD(TAG, "indoor_pipe_temp: %d", status->indoor_pipe_temperature);
                        ESP_LOGD(TAG, "indor_humid_set: %d indoor_humid: %d",
                                 status->indoor_humidity_setting,
                                 status->indoor_humidity_status);

                        ESP_LOGD(TAG,
                                 "indicate_led: %d\n"
                                 "electric_communication: %d\n"
                                 "indoor_electric: %d\n"
                                 "outdoor_machine: %d\n"
                                 "indoor_machine_run: %d\n"
                                 "low_electricity: %d\n"
                                 "indoor_electric (again): %d",
                                 status->indicate_led,
                                 status->electric_communication,
                                 status->indoor_electric,
                                 status->outdoor_machine,
                                 status->indoor_machine_run,
                                 status->low_electricity,
                                 status->indoor_electric);

                        target_temperature = status->indoor_temperature_setting;
                        current_temperature = status->indoor_temperature_status;

                        // Determine if the compressor is running
                        bool comp_running = (status->compressor_frequency > 0);

                        // Update swing mode based on left/right and up/down bits
                        if (status->left_right && status->up_down)
                            swing_mode = climate::CLIMATE_SWING_BOTH;
                        else if (status->left_right)
                            swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
                        else if (status->up_down)
                            swing_mode = climate::CLIMATE_SWING_VERTICAL;
                        else
                            swing_mode = climate::CLIMATE_SWING_OFF;

                        // Set mode and action based on run and mode status
                        if (status->run_status == 0)
                        {
                            mode = climate::CLIMATE_MODE_OFF;
                            action = climate::CLIMATE_ACTION_OFF;
                        }
                        else if (status->mode_status == 0)
                        {
                            mode = climate::CLIMATE_MODE_FAN_ONLY;
                            action = climate::CLIMATE_ACTION_FAN;
                        }
                        else if (status->mode_status == 1)
                        {
                            mode = climate::CLIMATE_MODE_HEAT;
                            action = comp_running ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
                        }
                        else if (status->mode_status == 2)
                        {
                            mode = climate::CLIMATE_MODE_COOL;
                            action = comp_running ? climate::CLIMATE_ACTION_COOLING : climate::CLIMATE_ACTION_IDLE;
                        }
                        else if (status->mode_status == 3)
                        {
                            mode = climate::CLIMATE_MODE_DRY;
                            action = comp_running ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_IDLE;
                        }

                        // Set fan mode based on wind status
                        if (status->wind_status == 18)
                            fan_mode = climate::CLIMATE_FAN_HIGH;
                        else if (status->wind_status == 14)
                            fan_mode = climate::CLIMATE_FAN_MEDIUM;
                        else if (status->wind_status == 10)
                            fan_mode = climate::CLIMATE_FAN_LOW;
                        else if (status->wind_status == 2)
                            fan_mode = climate::CLIMATE_FAN_QUIET;
                        else if (status->wind_status == 0)
                            fan_mode = climate::CLIMATE_FAN_AUTO;

                        // Save target temperature for mode switching
                        if (this->mode == climate::CLIMATE_MODE_COOL && target_temperature > 0)
                            cool_tgt_temp = target_temperature;
                        else if (this->mode == climate::CLIMATE_MODE_HEAT && target_temperature > 0)
                            heat_tgt_temp = target_temperature;
                    }
                }

                // Send any pending messages
                blocking_send(0, 0);
            }

            void update() override
            {
                request_update();

                this->publish_state();

                // Update sensors
                set_sensor(compressor_frequency, status->compressor_frequency);
                set_sensor(compressor_frequency_setting, status->compressor_frequency_setting);
                set_sensor(compressor_frequency_send, status->compressor_frequency_send);
                set_sensor(outdoor_temperature, status->outdoor_temperature);
                set_sensor(outdoor_condenser_temperature, status->outdoor_condenser_temperature);
                set_sensor(compressor_exhaust_temperature, status->compressor_exhaust_temperature);
                set_sensor(target_exhaust_temperature, status->target_exhaust_temperature);
                set_sensor(indoor_pipe_temperature, status->indoor_pipe_temperature);
                set_sensor(indoor_humidity_setting, status->indoor_humidity_setting);
                set_sensor(indoor_humidity_status, status->indoor_humidity_status);
            }

            void control(const climate::ClimateCall &call) override
            {
                if (call.get_mode().has_value())
                {
                    // Save target temperature since it gets messed up by the mode switch command
                    if (this->mode == climate::CLIMATE_MODE_COOL && target_temperature > 0)
                    {
                        cool_tgt_temp = target_temperature;
                    }
                    else if (this->mode == climate::CLIMATE_MODE_HEAT && target_temperature > 0)
                    {
                        heat_tgt_temp = target_temperature;
                    }

                    // User requested mode change
                    climate::ClimateMode md = *call.get_mode();

                    if (md != climate::CLIMATE_MODE_OFF && this->mode == climate::CLIMATE_MODE_OFF)
                    {
                        blocking_send(on, sizeof(on));
                    }

                    switch (md)
                    {
                    case climate::CLIMATE_MODE_OFF:
                        blocking_send(off, sizeof(off));
                        break;
                    case climate::CLIMATE_MODE_COOL:
                        blocking_send(mode_cool, sizeof(mode_cool));
                        set_temp(cool_tgt_temp);
                        break;
                    case climate::CLIMATE_MODE_HEAT:
                        blocking_send(mode_heat, sizeof(mode_heat));
                        set_temp(heat_tgt_temp);
                        break;
                    case climate::CLIMATE_MODE_FAN_ONLY:
                        blocking_send(mode_fan, sizeof(mode_fan));
                        break;
                    case climate::CLIMATE_MODE_DRY:
                        blocking_send(mode_dry, sizeof(mode_dry));
                        break;
                    default:
                        break;
                    }

                    // Publish updated state
                    this->mode = md;
                    this->publish_state();
                }

                if (call.get_target_temperature().has_value())
                {
                    // User requested target temperature change
                    float temp = *call.get_target_temperature();

                    set_temp(temp);

                    // Send target temp to climate
                    target_temperature = temp;
                    this->publish_state();
                }

                if (call.get_fan_mode().has_value())
                {
                    climate::ClimateFanMode fm = *call.get_fan_mode();
                    switch (fm)
                    {
                    case climate::CLIMATE_FAN_AUTO:
                        blocking_send(speed_auto, sizeof(speed_auto));
                        break;
                    case climate::CLIMATE_FAN_LOW:
                        blocking_send(speed_low, sizeof(speed_low));
                        break;
                    case climate::CLIMATE_FAN_MEDIUM:
                        blocking_send(speed_med, sizeof(speed_med));
                        break;
                    case climate::CLIMATE_FAN_HIGH:
                        blocking_send(speed_max, sizeof(speed_max));
                        break;
                    case climate::CLIMATE_FAN_QUIET:
                        blocking_send(speed_mute, sizeof(speed_mute));
                        break;
                    default:
                        break;
                    }
                    fan_mode = fm;
                    this->publish_state();
                }

                if (call.get_swing_mode().has_value())
                {
                    climate::ClimateSwingMode sm = *call.get_swing_mode();

                    if (sm == climate::CLIMATE_SWING_OFF)
                    {
                        if (climate::CLIMATE_SWING_BOTH == swing_mode)
                        {
                            blocking_send(vert_swing, sizeof(vert_swing));
                            blocking_send(hor_swing, sizeof(hor_swing));
                        }
                        else if (climate::CLIMATE_SWING_VERTICAL == swing_mode)
                        {
                            blocking_send(vert_swing, sizeof(vert_swing));
                        }
                        else if (climate::CLIMATE_SWING_HORIZONTAL == swing_mode)
                        {
                            blocking_send(hor_swing, sizeof(hor_swing));
                        }
                    }
                    else if (sm == climate::CLIMATE_SWING_BOTH)
                    {
                        if (climate::CLIMATE_SWING_OFF == swing_mode)
                        {
                            blocking_send(vert_swing, sizeof(vert_swing));
                            blocking_send(hor_swing, sizeof(hor_swing));
                        }
                        else if (climate::CLIMATE_SWING_VERTICAL == swing_mode)
                        {
                            blocking_send(hor_swing, sizeof(hor_swing));
                        }
                        else if (climate::CLIMATE_SWING_HORIZONTAL == swing_mode)
                        {
                            blocking_send(vert_swing, sizeof(vert_swing));
                        }
                    }
                    else if (sm == climate::CLIMATE_SWING_VERTICAL)
                    {
                        if (climate::CLIMATE_SWING_BOTH == swing_mode)
                        {
                            blocking_send(hor_swing, sizeof(hor_swing));
                        }
                        else if (climate::CLIMATE_SWING_HORIZONTAL == swing_mode)
                        {
                            blocking_send(hor_swing, sizeof(hor_swing));
                            blocking_send(vert_swing, sizeof(vert_swing));
                        }
                    }
                    else if (sm == climate::CLIMATE_SWING_HORIZONTAL)
                    {
                        if (climate::CLIMATE_SWING_BOTH == swing_mode)
                        {
                            blocking_send(vert_swing, sizeof(vert_swing));
                        }
                        else if (climate::CLIMATE_SWING_VERTICAL == swing_mode)
                        {
                            blocking_send(vert_swing, sizeof(vert_swing));
                            blocking_send(hor_swing, sizeof(hor_swing));
                        }
                    }

                    swing_mode = sm;
                    this->publish_state();
                }

                if (call.get_preset().has_value())
                {
                    climate::ClimatePreset pre = *call.get_preset();
                    switch (pre)
                    {
                    case climate::CLIMATE_PRESET_NONE:
                        blocking_send(turbo_off, sizeof(turbo_off));
                        blocking_send(energysave_off, sizeof(energysave_off));
                        break;
                    case climate::CLIMATE_PRESET_BOOST:
                        blocking_send(turbo_on, sizeof(turbo_on));
                        break;
                    case climate::CLIMATE_PRESET_ECO:
                        blocking_send(energysave_on, sizeof(energysave_on));
                        break;
                    default:
                        break;
                    }

                    preset = pre;
                    this->publish_state();
                }
            }

            climate::ClimateTraits traits() override
            {
                // The capabilities of the climate device
                auto traits = climate::ClimateTraits();
                traits.set_supports_current_temperature(true);
                traits.set_visual_min_temperature(16);
                traits.set_visual_max_temperature(30);
                traits.set_visual_temperature_step(1);
                traits.set_supported_modes({
                    climate::CLIMATE_MODE_OFF,
                    climate::CLIMATE_MODE_COOL,
                    climate::CLIMATE_MODE_HEAT,
                    climate::CLIMATE_MODE_FAN_ONLY,
                    climate::CLIMATE_MODE_DRY,
                });
                traits.set_supported_swing_modes({climate::CLIMATE_SWING_OFF,
                                                  climate::CLIMATE_SWING_BOTH,
                                                  climate::CLIMATE_SWING_VERTICAL,
                                                  climate::CLIMATE_SWING_HORIZONTAL});
                traits.set_supported_fan_modes({
                    climate::CLIMATE_FAN_AUTO,
                    climate::CLIMATE_FAN_LOW,
                    climate::CLIMATE_FAN_MEDIUM,
                    climate::CLIMATE_FAN_HIGH,
                    climate::CLIMATE_FAN_QUIET,
                });
                traits.set_supported_presets({climate::CLIMATE_PRESET_NONE,
                                              climate::CLIMATE_PRESET_BOOST,
                                              climate::CLIMATE_PRESET_ECO});
                traits.set_supports_action(true);
                return traits;
            }

            sensor::Sensor *compressor_frequency;
            sensor::Sensor *compressor_frequency_setting;
            sensor::Sensor *compressor_frequency_send;
            sensor::Sensor *outdoor_temperature;
            sensor::Sensor *outdoor_condenser_temperature;
            sensor::Sensor *compressor_exhaust_temperature;
            sensor::Sensor *target_exhaust_temperature;
            sensor::Sensor *indoor_pipe_temperature;
            sensor::Sensor *indoor_humidity_setting;
            sensor::Sensor *indoor_humidity_status;

        private:
            float heat_tgt_temp = 16.1111f;
            float cool_tgt_temp = 26.6667f;
            static const int UART_BUF_SIZE = 128;
            Device_Status *status = new Device_Status();
            bool wait_for_rx = false;

            // Handle bytes form the UART to build a complete message
            int update_status(const uint8_t input)
            {
                static char buf[UART_BUF_SIZE] = {0};
                static int buf_idx = 0;
                static int msg_size = 0;
                static uint16_t checksum = 0;
                static bool f4_detect = false;
                bool reset = false;

                // Put the byte in the buffer
                if (!f4_detect)
                    buf[buf_idx++] = input;
                else
                    f4_detect = false;

                // The checksum is computed from byte index 2 to msg_size - 4
                if ((buf_idx > 2 && buf_idx < 6) || (buf_idx < msg_size - 4))
                {
                    checksum += buf[buf_idx - 1];
                }

                // Make sure we don't ever overflow the buffer
                if (buf_idx >= UART_BUF_SIZE)
                {
                    reset = true;
                }
                else if (buf_idx == 1) // Search for frame start byte 1
                {
                    if (input != 0xF4)
                    {
                        reset = true;
                    }
                }
                else if (buf_idx == 2) // Search for frame start byte 2
                {
                    if (input != 0xF5)
                    {
                        reset = true;
                    }
                }
                else if (buf_idx == 3) // Search for message mode byte (1 = repsonse)
                {
                    if (input != 0x01)
                    {
                        reset = true;
                    }
                }
                else if (buf_idx == 4) // Search for message type (we only handle 0x40)
                {
                    if (input != 0x40)
                    {
                        reset = true;
                    }
                }
                else if (buf_idx == 5) // get message size
                {
                    msg_size = input + 9; // add header and footer bytes + this byte
                }
                else if (buf_idx == msg_size - 2) // third to last byte (end of checksum)
                {
                    uint16_t rxd_checksum = buf[msg_size - 4];
                    rxd_checksum = rxd_checksum << 8;
                    rxd_checksum |= buf[msg_size - 3];
                    if (rxd_checksum != checksum)
                    {
                        ESP_LOGE(
                            "aircon_climate",
                            "CRC check failed. Computed: %d Received: %d",
                            checksum,
                            rxd_checksum);
                        reset = true;
                    }
                }
                else if (buf_idx == msg_size - 1) // second to last byte
                {
                    if (input != 0xF4)
                    {
                        reset = true;
                    }
                }
                else if (buf_idx == msg_size) // last byte
                {
                    if (input != 0xFB)
                    {
                        reset = true;
                    }
                    else
                    {
                        int msg_size_cpy = msg_size;
                        ESP_LOGD(
                            "aircon_climate",
                            "Received %d bytes.",
                            msg_size);
                        memcpy((void *)status, buf, msg_size);
                        buf_idx = 0;
                        msg_size = 0;
                        checksum = 0;
                        wait_for_rx = false;
                        return msg_size_cpy;
                    }
                }
                else if (!f4_detect && input == 0xF4)
                {
                    f4_detect = true;
                }

                // Reset the static variables if we failed any of the conditions.
                if (reset)
                {
                    ESP_LOGD("aircon_climate", "Resetting RX buffer.");
                    buf_idx = 0;
                    msg_size = 0;
                    checksum = 0;
                    wait_for_rx = false;
                }

                return 0;
            }

            // This function buffers messages to be sent to the AC.
            // Send messages one at a time and wait for each acknowledgement.
            void blocking_send(uint8_t buf[], size_t sz)
            {
                static uint8_t insert = 0;
                static uint8_t read = 0;
                static uint8_t hold_buf[8][64] = {0};
                static size_t sz_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
                static uint8_t num_buffered = 0;
                static uint32_t start_time = 0;

                if (sz > 0)
                {
                    ESP_LOGD("aircon_climate",
                             "Buffering message. Read: %d Insert: %d",
                             read,
                             insert);
                    memcpy(hold_buf[insert], buf, sz);
                    sz_buf[insert] = sz;
                    insert = (insert + 1) & 7;
                    if (num_buffered == 0)
                    {
                        start_time = millis();
                    }
                    num_buffered++;
                }

                // Return if we're waiting. If we get to seven messages buffered,
                // there's probably an issue and we shouldn't block anymore.
                if (num_buffered < 8 && wait_for_rx && (millis() - start_time < 500))
                {
                    return;
                }

                if (sz_buf[read] > 0)
                {
                    ESP_LOGD("aircon_climate",
                             "Sending message. Read: %d Insert: %d",
                             read,
                             insert);
                    write_array(hold_buf[read], sz_buf[read]);
                    flush();
                    sz_buf[read] = 0;
                    read = (read + 1) & 7;
                    wait_for_rx = true;
                    num_buffered--;
                    start_time = millis();
                }
            }

            // Get status from the AC
            void request_update()
            {
                uint8_t req_stat[] = {
                    0xF4, 0xF5, 0x00, 0x40,
                    0x0C, 0x00, 0x00, 0x01,
                    0x01, 0xFE, 0x01, 0x00,
                    0x00, 0x66, 0x00, 0x00,
                    0x00, 0x01, 0xB3, 0xF4,
                    0xFB};

                ESP_LOGD("aircon_climate", "Requesting update.");
                blocking_send(req_stat, sizeof(req_stat));
            }

            // Update sensors when the value has actually changed.
            void set_sensor(sensor::Sensor *sensor, float value)
            {
                if (sensor == nullptr)
                    return;

                if (!sensor->has_state() || sensor->get_raw_state() != value)
                    sensor->publish_state(value);
            }

            void set_display_state(bool state)
            {
                if (state)
                {
                    blocking_send(display_on, sizeof(display_on));
                }
                else
                {
                    blocking_send(display_off, sizeof(display_off));
                }
            }

            // Set the temperature
            void set_temp(float temp)
            {
                int t = static_cast<int>(temp);
                switch (t)
                {
                case 16:
                    blocking_send(temp_16_C, sizeof(temp_16_C));
                    break;
                case 17:
                    blocking_send(temp_17_C, sizeof(temp_17_C));
                    break;
                case 18:
                    blocking_send(temp_18_C, sizeof(temp_18_C));
                    break;
                case 19:
                    blocking_send(temp_19_C, sizeof(temp_19_C));
                    break;
                case 20:
                    blocking_send(temp_20_C, sizeof(temp_20_C));
                    break;
                case 21:
                    blocking_send(temp_21_C, sizeof(temp_21_C));
                    break;
                case 22:
                    blocking_send(temp_22_C, sizeof(temp_22_C));
                    break;
                case 23:
                    blocking_send(temp_23_C, sizeof(temp_23_C));
                    break;
                case 24:
                    blocking_send(temp_24_C, sizeof(temp_24_C));
                    break;
                case 25:
                    blocking_send(temp_25_C, sizeof(temp_25_C));
                    break;
                case 26:
                    blocking_send(temp_26_C, sizeof(temp_26_C));
                    break;
                case 27:
                    blocking_send(temp_27_C, sizeof(temp_27_C));
                    break;
                case 28:
                    blocking_send(temp_28_C, sizeof(temp_28_C));
                    break;
                case 29:
                    blocking_send(temp_29_C, sizeof(temp_29_C));
                    break;
                case 30:
                    blocking_send(temp_30_C, sizeof(temp_30_C));
                    break;
                default:
                    break;
                }
            }
        };

    }
}