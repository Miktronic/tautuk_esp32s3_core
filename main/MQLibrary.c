#include "MQLibrary.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TauTuk";

uint8_t _firstFlag = 0;
    
float _VOLT_RESOLUTION  = 0; // if 3.3v use 3.3
uint8_t _ADC_Bit_Resolution = 12;
uint8_t _regressionMethod = 1; // 1 -> Exponential || 2 -> Linear
uint8_t _retries = 5;
uint8_t _retry_interval = 10;

float _R1 = 0;
float _R2 = 0;
float _RL = 10; //Value in KiloOhms
float _a  = 0;
float _b  = 0;
float _sensor_volt = 0;
float _R0 = 0; 
float RS_air = 0;
float _ratio = 0;
float _PPM = 0;
float _RS_Calc = 0;  

int adc_raw     = 0;
int voltage     = 0;

#define ADC1_CHAN0                  ADC_CHANNEL_0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_unit_init_cfg_t init_config1 = {
  .unit_id = ADC_UNIT_1,
};
//-------------ADC1 Config---------------//
adc_oneshot_chan_cfg_t config = {
  .atten = EXAMPLE_ADC_ATTEN,
  .bitwidth = ADC_BITWIDTH_DEFAULT,
};

uint8_t do_calibration1_chan0;
adc_cali_handle_t adc1_cali_chan0_handle = NULL;

void initADC(void)
{
//-------------ADC1 Init---------------//
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration1_chan0 = app_adc_calibration_init(ADC_UNIT_1, ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
}

static bool app_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void app_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void getVoltage(void)
{
    int adc_voltage = 0;
    
    voltage = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", (ADC_UNIT_1 + 1), ADC1_CHAN0, adc_raw);
    if (do_calibration1_chan0) {
      for (int i = 0; i < _retries; i++){
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw, &adc_voltage));
        voltage = voltage + adc_voltage;
        vTaskDelay(_retry_interval / portTICK_PERIOD_MS);
      }  

      voltage = voltage / _retries;
      ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", (ADC_UNIT_1 + 1), ADC1_CHAN0, voltage);
    }

    if (_R2 > 0){
      voltage = voltage * (_R1 + _R2) / _R2;
    }

    //return voltage;
}

void update(void)
{
  getVoltage();
  _sensor_volt = voltage;
  ESP_LOGI(TAG, "Sensor Voltage: %f mV", _sensor_volt);
}

void setA(float a) {
  _a = a;
}
void setB(float b) {
  _b = b;
}
void setRL(float RL) {
  if(_R2 > 0){
    _RL = (((_R1 + _R2) * RL) / (RL + _R1 + _R2));
  } else {
    _RL = RL;
  }
}
void setR0(float R0) {
  _R0 = R0;
}
void setR1(float R1) {
  _R1 = R1;
}
void setR2(float R2) {
  _R2 = R2;
}
void setVoltResolution(float voltage_resolution)
{
  _VOLT_RESOLUTION = voltage_resolution;
}
void setRegressionMethod(int regressionMethod)
{
  _regressionMethod = regressionMethod;
}
float validateEcuation(float ratioInput)
{
  if(_regressionMethod == 1) _PPM= _a*pow(ratioInput, _b);
  else 
  {
      // https://jayconsystems.com/blog/understanding-a-gas-sensor
      double ppm_log = (log10(ratioInput)-_b)/_a; //Get ppm value in linear scale according to the the ratio value  
      _PPM = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  //Serial.println("Regression Method: "); Serial.println(_regressionMethod);
  //Serial.println("Result: "); Serial.println(_PPM);
  return _PPM;  
}
float readSensor(bool isMQ303A, float correctionFactor, bool injected)
{
  //More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  if(isMQ303A) {
    _VOLT_RESOLUTION = _VOLT_RESOLUTION - 0.45; //Calculations for RS using mq303a sensor look wrong #42
  }
  _RS_Calc = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Get value of RS in a gas
  if(_RS_Calc < 0)  _RS_Calc = 0; //No negative values accepted.
  if(!injected) _ratio = _RS_Calc / _R0;   // Get ratio RS_gas/RS_air
  _ratio += correctionFactor;
  if(_ratio <= 0)  _ratio = 0; //No negative values accepted or upper datasheet recomendation.
  if(_regressionMethod == 1) _PPM= _a*pow(_ratio, _b); // <- Source excel analisis https://github.com/miguel5612/MQSensorsLib_Docs/tree/master/Internal_design_documents
  else 
  {
    // https://jayconsystems.com/blog/understanding-a-gas-sensor <- Source of linear ecuation
    double ppm_log = (log10(_ratio)-_b)/_a; //Get ppm value in linear scale according to the the ratio value  
    _PPM = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  if(_PPM < 0)  _PPM = 0; //No negative values accepted or upper datasheet recomendation.
  //if(_PPM > 10000) _PPM = 99999999; //No negative values accepted or upper datasheet recomendation.
  return _PPM;
}
float readSensorR0Rs()
{
  //More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  _RS_Calc = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Get value of RS in a gas
  if(_RS_Calc < 0)  _RS_Calc = 0; //No negative values accepted.
  _ratio = _R0/_RS_Calc;   // Get ratio RS_air/RS_gas <- INVERTED for MQ-131 issue 28 https://github.com/miguel5612/MQSensorsLib/issues/28
  if(_ratio <= 0)  _ratio = 0; //No negative values accepted or upper datasheet recomendation.
  if(_regressionMethod == 1) _PPM= _a*pow(_ratio, _b); // <- Source excel analisis https://github.com/miguel5612/MQSensorsLib_Docs/tree/master/Internal_design_documents
  else 
  {
    // https://jayconsystems.com/blog/understanding-a-gas-sensor <- Source of linear ecuation
    double ppm_log = (log10(_ratio)-_b)/_a; //Get ppm value in linear scale according to the the ratio value  
    _PPM = pow(10, ppm_log); //Convert ppm value to log scale  
  }
  if(_PPM < 0)  _PPM = 0; //No negative values accepted or upper datasheet recomendation.
  //if(_PPM > 10000) _PPM = 99999999; //No negative values accepted or upper datasheet recomendation.
  return _PPM;
}
float calibrate(float ratioInCleanAir) {
  //More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  /*
  V = I x R 
  VRL = [VC / (RS + RL)] x RL 
  VRL = (VC x RL) / (RS + RL) 
  Así que ahora resolvemos para RS: 
  VRL x (RS + RL) = VC x RL
  (VRL x RS) + (VRL x RL) = VC x RL 
  (VRL x RS) = (VC x RL) - (VRL x RL)
  RS = [(VC x RL) - (VRL x RL)] / VRL
  RS = [(VC x RL) / VRL] - RL
  */
  float RS_air; //Define variable for sensor resistance
  float R0; //Define variable for R0
  RS_air = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Calculate RS in fresh air
  if(RS_air < 0)  RS_air = 0; //No negative values accepted.
  R0 = RS_air/ratioInCleanAir; //Calculate R0 
  if(R0 < 0)  R0 = 0; //No negative values accepted.
  return R0;
}

float setRsR0RatioGetPPM(float value)
{
  _ratio = value;
  return readSensor(false, 0, true);
}
float getRS()
{
  //More explained in: https://jayconsystems.com/blog/understanding-a-gas-sensor
  _RS_Calc = ((_VOLT_RESOLUTION*_RL)/_sensor_volt)-_RL; //Get value of RS in a gas
  if(_RS_Calc < 0)  _RS_Calc = 0; //No negative values accepted.
  return _RS_Calc;
}