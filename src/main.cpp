// Oceans Dream Instruments
// Upgraded to SensESP version 3.0.0
// Revised build January 2025 rev 6

// Now includes additional 1-wire sensor for future use
// Now includes temperature interpreter for future use
// Now includes modified RPM code
// Now includes INA219 sensor for fuel tank level

#include <memory>
#include <WiFi.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp_onewire/onewire_temperature.h"

using namespace sensesp;

// Custom Temperature Interpreter Class
class TemperatureInterpreter : public CurveInterpolator {
 public:
  TemperatureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
    add_sample(CurveInterpolator::Sample(20, 393.15));
    add_sample(CurveInterpolator::Sample(30, 383.15));
    add_sample(CurveInterpolator::Sample(40, 373.15));
    add_sample(CurveInterpolator::Sample(55, 363.15));
    add_sample(CurveInterpolator::Sample(70, 353.15));
    add_sample(CurveInterpolator::Sample(100, 343.15));
    add_sample(CurveInterpolator::Sample(140, 333.15));
    add_sample(CurveInterpolator::Sample(200, 323.15));
    add_sample(CurveInterpolator::Sample(300, 317.15));
    add_sample(CurveInterpolator::Sample(400, 313.15));
  }
};

// Custom Fuel Flow Interpreter Class
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to M^3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, M^3/s));
    add_sample(CurveInterpolator::Sample(600, 0.0000000694));
    add_sample(CurveInterpolator::Sample(1000, 0.000000125));
    add_sample(CurveInterpolator::Sample(1500, 0.000000222));
    add_sample(CurveInterpolator::Sample(1800, 0.000000284));
    add_sample(CurveInterpolator::Sample(2000, 0.000000347));
    add_sample(CurveInterpolator::Sample(2200, 0.000000484));
    add_sample(CurveInterpolator::Sample(2400, 0.000000620));
    add_sample(CurveInterpolator::Sample(2600, 0.000000757));
    add_sample(CurveInterpolator::Sample(2800, 0.000000893));
    add_sample(CurveInterpolator::Sample(3000, 0.00000103));
    add_sample(CurveInterpolator::Sample(3200, 0.00000124));
  }
};

// Global sensor objects
Adafruit_BME280 bme280;
Adafruit_INA219 ina219;

// BME280 sensor callback functions
float read_temp_callback() {
  return (bme280.readTemperature() + 273.15);
}

float read_pressure_callback() {
  return (bme280.readPressure());
}

float read_humidity_callback() {
  return (bme280.readHumidity());
}

// INA219 sensor callback function
float read_current_callback() {
  return (ina219.getCurrent_mA() / 1000);
}

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging();

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Oceans-Dream-Instruments")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("pi", "roborough")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_uptime_sensor()
                    // ->enable_ota("raspberry")
                    ->get_app();

  // Initialize hardware sensors
  if (!bme280.begin(0x76)) {
    ESP_LOGE(__FILE__, "Could not find a valid BME280 sensor, check wiring!");
  }

  if (!ina219.begin()) {
    ESP_LOGE(__FILE__, "Failed to find INA219 chip");
  }

  /// Fuel Gauge Sensor ///
  // Create a RepeatSensor with float output that reads the current
  // using the function defined above.
  auto fuel_level = std::make_shared<RepeatSensor<float>>(300000, read_current_callback);

  // Send the level to the Signal K server as a Float
  auto fuel_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.fuelTank.level",
      "/Fuel/Level/sk_path"
  );

  ConfigItem(fuel_sk_output)
      ->set_title("Fuel Tank Level SK Output Path")
      ->set_description("Signal K path for fuel tank level")
      ->set_sort_order(100);

  fuel_level->connect_to(fuel_sk_output);

      /// 1-Wire Temp Sensors ///
  auto dts = new sensesp::onewire::DallasTemperatureSensors(19);

  // Exhaust Temperature - propulsion/engine/exhaustTemperature
  auto exhaust_temp = std::make_shared<sensesp::onewire::OneWireTemperature>(dts, 10000, "/Exhaust Temperature/oneWire");
  auto exhaust_linear = std::make_shared<Linear>(1.0, 0.0, "/Exhaust Temperature/linear");
  auto exhaust_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.exhaustTemperature",
      "/Exhaust Temperature/sk_path"
  );

  ConfigItem(exhaust_temp)
      ->set_title("Exhaust Temperature OneWire Sensor")
      ->set_description("OneWire temperature sensor for exhaust")
      ->set_sort_order(199);

  ConfigItem(exhaust_linear)
      ->set_title("Exhaust Temperature Calibration")
      ->set_description("Linear calibration for exhaust temperature")
      ->set_sort_order(200);

  ConfigItem(exhaust_sk_output)
      ->set_title("Exhaust Temperature SK Output Path")
      ->set_description("Signal K path for exhaust temperature")
      ->set_sort_order(201);

  exhaust_temp->connect_to(exhaust_linear)->connect_to(exhaust_sk_output);

  // Alternator Temperature - electrical/alternator/temperature
  auto alternator_temp = std::make_shared<sensesp::onewire::OneWireTemperature>(dts, 10000, "/Alternator Temperature/oneWire");
  auto alternator_linear = std::make_shared<Linear>(1.0, 0.0, "/Alternator Temperature/linear");
  auto alternator_sk_output = std::make_shared<SKOutput<float>>(
      "electrical.alternator.temperature",
      "/Alternator Temperature/sk_path"
  );

  ConfigItem(alternator_temp)
      ->set_title("Alternator Temperature OneWire Sensor")
      ->set_description("OneWire temperature sensor for alternator")
      ->set_sort_order(299);

  ConfigItem(alternator_linear)
      ->set_title("Alternator Temperature Calibration")
      ->set_description("Linear calibration for alternator temperature")
      ->set_sort_order(300);

  ConfigItem(alternator_sk_output)
      ->set_title("Alternator Temperature SK Output Path")
      ->set_description("Signal K path for alternator temperature")
      ->set_sort_order(301);

  alternator_temp->connect_to(alternator_linear)->connect_to(alternator_sk_output);

  // Oil Temperature - propulsion/engine/oilTemperature
  auto oil_temp = std::make_shared<sensesp::onewire::OneWireTemperature>(dts, 10000, "/Oil Temperature/oneWire");
  auto oil_linear = std::make_shared<Linear>(1.0, 0.0, "/Oil Temperature/linear");
  auto oil_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.oilTemperature",
      "/Oil Temperature/sk_path"
  );

  ConfigItem(oil_temp)
      ->set_title("Oil Temperature OneWire Sensor")
      ->set_description("OneWire temperature sensor for oil")
      ->set_sort_order(399);

  ConfigItem(oil_linear)
      ->set_title("Oil Temperature Calibration")
      ->set_description("Linear calibration for oil temperature")
      ->set_sort_order(400);

  ConfigItem(oil_sk_output)
      ->set_title("Oil Temperature SK Output Path")
      ->set_description("Signal K path for oil temperature")
      ->set_sort_order(401);

  oil_temp->connect_to(oil_linear)->connect_to(oil_sk_output);

  // Coolant Temperature - propulsion/engine/coolantTemperature
  auto coolant_temp = std::make_shared<sensesp::onewire::OneWireTemperature>(dts, 10000, "/Coolant Temperature/oneWire");
  auto coolant_linear = std::make_shared<Linear>(1.0, 0.0, "/Coolant Temperature/linear");
  auto coolant_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.coolantTemperature",
      "/Coolant Temperature/sk_path"
  );

  ConfigItem(coolant_temp)
      ->set_title("Coolant Temperature OneWire Sensor")
      ->set_description("OneWire temperature sensor for coolant")
      ->set_sort_order(499);

  ConfigItem(coolant_linear)
      ->set_title("Coolant Temperature Calibration")
      ->set_description("Linear calibration for coolant temperature")
      ->set_sort_order(500);

  ConfigItem(coolant_sk_output)
      ->set_title("Coolant Temperature SK Output Path")
      ->set_description("Signal K path for coolant temperature")
      ->set_sort_order(501);

  coolant_temp->connect_to(coolant_linear)->connect_to(coolant_sk_output);

  /// RPM Application ///
  const float multiplier = 1.0 / 11.0;
  const unsigned int read_delay = 2000;
  const uint8_t rpm_pin = 16;

  auto rpm_sensor = std::make_shared<DigitalInputCounter>(rpm_pin, INPUT_PULLUP, RISING, read_delay);

  // RPM frequency calculation and output
  auto frequency_calibrate = std::make_shared<Frequency>(multiplier, "/Engine RPM/calibrate");
  auto rpm_moving_avg = std::make_shared<MovingAverage>(2, 1.0, "/Engine RPM/movingAVG");
  auto rpm_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.revolutions",
      "/Engine RPM/sk_path"
  );

  ConfigItem(frequency_calibrate)
      ->set_title("RPM Frequency Calibration")
      ->set_description("Frequency multiplier for RPM calculation")
      ->set_sort_order(600);

  ConfigItem(rpm_moving_avg)
      ->set_title("RPM Moving Average")
      ->set_description("Moving average filter for RPM")
      ->set_sort_order(601);

  ConfigItem(rpm_sk_output)
      ->set_title("Engine RPM SK Output Path")
      ->set_description("Signal K path for engine RPM")
      ->set_sort_order(602);

  rpm_sensor->connect_to(frequency_calibrate)->connect_to(rpm_moving_avg)->connect_to(rpm_sk_output);

  // Fuel flow calculation from RPM
  auto fuel_frequency = std::make_shared<Frequency>(6.0, "/Engine Fuel/frequency");
  auto fuel_moving_avg = std::make_shared<MovingAverage>(4, 1.0, "/Engine Fuel/movingAVG");
  auto fuel_curve = std::make_shared<FuelInterpreter>("/Engine Fuel/curve");
  auto fuel_flow_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.fuel.rate",
      "/Engine Fuel/sk_path"
  );

  ConfigItem(fuel_frequency)
      ->set_title("Fuel Flow Frequency")
      ->set_description("Frequency multiplier for fuel flow calculation")
      ->set_sort_order(700);

  ConfigItem(fuel_moving_avg)
      ->set_title("Fuel Flow Moving Average")
      ->set_description("Moving average filter for fuel flow")
      ->set_sort_order(701);

  ConfigItem(fuel_curve)
      ->set_title("Fuel Flow Curve Interpolation")
      ->set_description("RPM to fuel flow curve interpolation")
      ->set_sort_order(702);

  ConfigItem(fuel_flow_sk_output)
      ->set_title("Engine Fuel Flow SK Output Path")
      ->set_description("Signal K path for engine fuel flow rate")
      ->set_sort_order(703);

  rpm_sensor->connect_to(fuel_frequency)->connect_to(fuel_moving_avg)->connect_to(fuel_curve)->connect_to(fuel_flow_sk_output);

  /// BME280 SENSOR CODE - Temp/Humidity/Pressure Sensor ///
  // Create RepeatSensors with float output that read the BME280
  // using the functions defined above.
  auto engine_room_temp = std::make_shared<RepeatSensor<float>>(10000, read_temp_callback);
  auto engine_room_pressure = std::make_shared<RepeatSensor<float>>(10000, read_pressure_callback);
  auto engine_room_humidity = std::make_shared<RepeatSensor<float>>(10000, read_humidity_callback);

  auto temp_sk_output = std::make_shared<SKOutput<float>>("environment.engineBay.temperature", "/Engine Room/Temperature/sk_path");
  auto pressure_sk_output = std::make_shared<SKOutput<float>>("environment.engineBay.pressure", "/Engine Room/Pressure/sk_path");
  auto humidity_sk_output = std::make_shared<SKOutput<float>>("environment.engineBay.relativeHumidity", "/Engine Room/Humidity/sk_path");

  ConfigItem(temp_sk_output)
      ->set_title("Engine Room Temperature SK Output Path")
      ->set_description("Signal K path for engine room temperature")
      ->set_sort_order(800);

  ConfigItem(pressure_sk_output)
      ->set_title("Engine Room Pressure SK Output Path")
      ->set_description("Signal K path for engine room pressure")
      ->set_sort_order(801);

  ConfigItem(humidity_sk_output)
      ->set_title("Engine Room Humidity SK Output Path")
      ->set_description("Signal K path for engine room humidity")
      ->set_sort_order(802);

  // Send the sensor data to the Signal K server
  engine_room_temp->connect_to(temp_sk_output);
  engine_room_pressure->connect_to(pressure_sk_output);
  engine_room_humidity->connect_to(humidity_sk_output);

  /// Engine Temp Config ///
  const float Vin = 3.5;
  const float R1 = 120.0;
  const uint8_t analog_pin = 36;
  const unsigned int analog_read_interval = 2000;

  auto analog_input = std::make_shared<AnalogInput>(analog_pin, analog_read_interval);

  auto analog_voltage = std::make_shared<AnalogVoltage>(Vin, 1.0, 0.0, "/Engine Temp/voltage");
  auto voltage_divider = std::make_shared<VoltageDividerR2>(R1, Vin, "/Engine Temp/sender");
  auto temp_curve = std::make_shared<TemperatureInterpreter>("/Engine Temp/curve");
  auto temp_calibrate = std::make_shared<Linear>(1.0, 0.9, "/Engine Temp/calibrate");
  auto temp_moving_avg = std::make_shared<MovingAverage>(4, 1.0, "/Engine Temp/movingAVG");
  auto engine_temp_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.temperature",
      "/Engine Temp/sk_path"
  );
  auto engine_temp_raw_sk_output = std::make_shared<SKOutput<float>>(
      "propulsion.engine.temperature.raw",
      "/Engine Temp Raw/sk_path"
  );

  ConfigItem(analog_voltage)
      ->set_title("Engine Temperature Analog Voltage")
      ->set_description("Analog voltage conversion for engine temperature")
      ->set_sort_order(900);

  ConfigItem(voltage_divider)
      ->set_title("Engine Temperature Voltage Divider")
      ->set_description("Voltage divider calculation for temperature sender")
      ->set_sort_order(901);

  ConfigItem(temp_curve)
      ->set_title("Engine Temperature Curve")
      ->set_description("Temperature curve interpolation")
      ->set_sort_order(902);

  ConfigItem(temp_calibrate)
      ->set_title("Engine Temperature Calibration")
      ->set_description("Linear calibration for engine temperature")
      ->set_sort_order(903);

  ConfigItem(temp_moving_avg)
      ->set_title("Engine Temperature Moving Average")
      ->set_description("Moving average filter for engine temperature")
      ->set_sort_order(904);

  ConfigItem(engine_temp_sk_output)
      ->set_title("Engine Temperature SK Output Path")
      ->set_description("Signal K path for engine temperature")
      ->set_sort_order(905);

  ConfigItem(engine_temp_raw_sk_output)
      ->set_title("Engine Temperature Raw SK Output Path")
      ->set_description("Signal K path for raw engine temperature")
      ->set_sort_order(906);

  analog_input->connect_to(analog_voltage)->connect_to(voltage_divider)->connect_to(temp_curve)->connect_to(temp_calibrate)->connect_to(temp_moving_avg)->connect_to(engine_temp_sk_output);
  analog_input->connect_to(analog_voltage)->connect_to(voltage_divider)->connect_to(engine_temp_raw_sk_output);

  /// Bilge Monitor ///
  const uint8_t bilge_pin = 17;
  const unsigned int bilge_read_delay = 5000;

  auto bilge = std::make_shared<DigitalInputState>(bilge_pin, INPUT_PULLUP, bilge_read_delay);

  auto int_to_string_function = [](int input) -> String {
    if (input == 1) {
      return "Water in bilge";
    }
    else { // input == 0
      return "Bilge clear";
    }
  };

  auto int_to_string_transform = std::make_shared<LambdaTransform<int, String>>(int_to_string_function);
  auto bilge_sk_output = std::make_shared<SKOutput<String>>("notification.bilge", "/Bilge/Notification/sk_path");
  auto bilge_raw_sk_output = std::make_shared<SKOutput<float>>("notification.bilge.raw", "/Bilge/Raw/sk_path");

  ConfigItem(bilge_sk_output)
      ->set_title("Bilge Notification SK Output Path")
      ->set_description("Signal K path for bilge notification")
      ->set_sort_order(1000);

  ConfigItem(bilge_raw_sk_output)
      ->set_title("Bilge Raw SK Output Path")
      ->set_description("Signal K path for raw bilge status")
      ->set_sort_order(1001);

  bilge->connect_to(int_to_string_transform)->connect_to(bilge_sk_output);
  bilge->connect_to(bilge_raw_sk_output);

  ESP_LOGI(__FILE__, "Setup completed successfully");

  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
