#ifndef MQLibrary_H
  #define MQLibrary_H

  #include <stdint.h>
  #include <stdio.h>
  #include <stdlib.h>
  #include "esp_adc/adc_oneshot.h"
  #include "esp_adc/adc_cali.h"
  #include "esp_adc/adc_cali_scheme.h"
  #include "soc/soc_caps.h"
  #include <math.h>

  void initADC(void);
  static bool app_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
  static void app_adc_calibration_deinit(adc_cali_handle_t handle);
  void getVoltage(void);
  void update(void);
  void setA(float a);
  void setB(float b);
  void setRL(float RL);
  void setR0(float R0);
  void setR1(float R1);
  void setR2(float R2);
  void setVoltResolution(float voltage_resolution);
  void setRegressionMethod(int regressionMethod);
  float validateEcuation(float ratioInput);
  float readSensor(bool isMQ303A,float correctionFactor, bool injected);
  float readSensorR0Rs();
  float calibrate(float ratioInCleanAir);
  float setRsR0RatioGetPPM(float value);
  float getRS();
  
#endif //MQLibrary_H