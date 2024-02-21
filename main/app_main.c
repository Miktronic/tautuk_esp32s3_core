/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "cJSON.h"

#include "app_ble.h"
#include "MQLibrary.h"
#include "aws_iot_mqtt.h"

const char *TAG = "TauTuk";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLUE_LED_GPIO   GPIO_NUM_19
#define RED_LED_GPIO    GPIO_NUM_17
#define GREEN_LED_GPIO  GPIO_NUM_18

#define Voltage_Resolution      5000 // Calibration the resistor divider
#define RatioMQ135CleanAir      3.6

#define R1                      10
#define R2                      10
#define RL                      10

#define PACKET_SEND_FREQUENCY       120000
#define GAS_DELAY                   5000
#define GAS_DATA_POINTS             5

#define BLINK_PERIOD     1000

float CO        = 0.0;
float Alcohol   = 0.0;
float CO2       = 0.0;
float Toluen    = 0.0;
float NH4       = 0.0;
float Aceton    = 0.0;

static uint8_t s_led_state = 0;

char id[10] = "#TT0000001";
char mqtt_payload[256] = "";

char ESP_WIFI_SSID[32] = "518";
char ESP_WIFI_PASS[64] =  "caiyuangungun";
char device_id[16];
char client_cert[] = "-----BEGIN CERTIFICATE-----\nMIIDdzCCAl+gAwIBAgIUe3gOF+eVuVZC9kooCkWWB3ZcRE8wDQYJKoZIhvcNAQEL\nBQAwfjELMAkGA1UEBhMCVVMxEzARBgNVBAgMCldhc2hpbmd0b24xEDAOBgNVBAcM\nB1NlYXR0bGUxGDAWBgNVBAoMD0FtYXpvbi5jb20gSW5jLjEgMB4GA1UECwwXQW1h\nem9uIElvVCBQcm92aXNpb25pbmcxDDAKBgNVBAUTAzEuMDAeFw0yNDAyMjAxNTE0\nNDFaFw0yNDAyMjAxNTIxNDFaMEsxSTBHBgNVBAMMQDBlOWU1MDRiZTc0Mzc1MGVi\nMThhMTM5ZGM5ZThlMDMzYTQ2ZDIxZTBhNDkzMTM2MjRhMzdiOWM5NzA3NDBhOTIw\nggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQC+Y1orfgKSn2M7+iUcVQu5\nJj7iwK4A3Wgwm2qqxP3miryXNar2C5bF9i+pWxxJ8ejo5QdLa3RBJZvcpBvt4EEM\nvcIi5ZNQVqZ4cmXq2ZVz+Z4GfQBIDalzkpKJG5p/b66ezMf9o/83FOInWLEKgTnK\nZoenEV/mxZG0z7XrAKvNGpfxGRtCt97NmuBRUErUcpl/gZk0lYRt6aPAX4o9vYKY\nNBQc+zDoJ2wfbDEs3trteX3BbQ/DgPbBwtFlDeYiJK8YrhZRoiuyI7Zaaj3wEq7O\nU1gEIh1vCEkcqqsdKwgFVXrwBNFNU2ou/zlZs92JARkvHGSs9oZ752IoNKQKBked\nAgMBAAGjIDAeMAwGA1UdEwEB/wQCMAAwDgYDVR0PAQH/BAQDAgeAMA0GCSqGSIb3\nDQEBCwUAA4IBAQC7UnJe1pIWXhAOn+JjkGv8aX+FW4pMVIrl6esnDH3/aqky4FgS\nuFUHDm7BGqyt5m/1sjoKruP41dn+RIHyVEgNHvR+7NBDWu4Ej1kqwNpbLoSR+WdO\nHgur9NdzBFZ4k8ILh4Ecx4fmwDIxX/eNOjB90Nj5QCjyU4lBt5CvjOECS1MWkF+5\nUb1XTGSW75JWAUkjw2nOEaTekP3ptUnXPLNuLf6DiWyajkyQweJokTDtK5HDQ9a4\nyiLFIxeLcNYcdFdNpP23gyzrdAF0vwyAJyDXBXQ7wIN5mGPzA2Q+U93O8DdQz3aB\nI4ky8ak+HBjCNZsXTlcp2/xZFWLrOAPASeAc\n-----END CERTIFICATE-----\n";

char client_key[] = "-----BEGIN RSA PRIVATE KEY-----\nMIIEowIBAAKCAQEAvmNaK34Ckp9jO/olHFULuSY+4sCuAN1oMJtqqsT95oq8lzWq\n9guWxfYvqVscSfHo6OUHS2t0QSWb3KQb7eBBDL3CIuWTUFameHJl6tmVc/meBn0A\nSA2pc5KSiRuaf2+unszH/aP/NxTiJ1ixCoE5ymaHpxFf5sWRtM+16wCrzRqX8Rkb\nQrfezZrgUVBK1HKZf4GZNJWEbemjwF+KPb2CmDQUHPsw6CdsH2wxLN7a7Xl9wW0P\nw4D2wcLRZQ3mIiSvGK4WUaIrsiO2Wmo98BKuzlNYBCIdbwhJHKqrHSsIBVV68ATR\nTVNqLv85WbPdiQEZLxxkrPaGe+diKDSkCgZHnQIDAQABAoIBAFO+adw1mjYaS9EG\n76ntsrFnJEQjUqZovTqcMigZbEErB1dPsPp3EIPVDRPmUqQn9zXx/+ppwoIhY9SC\njxJsENDk6u7koo60/pZ+Y1wRBw8zXhImi6gl8kI216Pdc3DYlCufkdx8dXcqBWwf\nNAiV3PCtdr/DAbcFcljhAHj1G5ykcazxOmdH2uQj4Pc0q3jR5BFLNgWyp08LI4v5\n1BMm3tR/sA7/U4kYSqrusvh9TT2ZcllaZftSPREw+5A8mtYsr53Z4PVeoJkPwTHE\nRQyp+Ingq0UGk4F2jGVT1gh3fUlMBAYhfsF5dtBJz57XlezQox9nWxTrciCb1o1Y\nVDa6RLUCgYEA9NT+fO7QVJD/qr4HRnH7Bqy1CwpkekdOMvkTsMK34R+1k7l4ocft\n4M8XfVrxn7Thd4I1520wl+Q6tIUtiWTSMNPhjxI0+qch4lL89kbHkxDcSGyi/Kj+\nykBS52Uz1R4U9SO6Ju36mA814RTzbGUzK7G9g9ktBY58zYwCNla/9LsCgYEAxxKY\nAASwAh/YHHplcBMrWAF9lih7AXxvEWi7k1C9+4ZzJa94jgno0m1nDzVOgKZy/i9m\nw2c2MjOBBPdR0dcyEDLCE9xCzhAgyuCdXGXjxbcmLy7CtpjjSKMYvq4+4ouFoyEU\nI844BbarrPcv4t6jOLwAQGN6lOY5L3V6kbptm4cCgYAgnvB5fOhNHDS0bzVQ8Ybc\n0M4edngEwtNsfztcZdVSLYNn92JXS+gp9+3NSfy/pr4TykmcWDQNSN95hfUXRVOs\nJc773RUqAHLHUP9bYPreYXS4QaFFwM6R7BgftKA/WQ65ytTWswaclAo9vyjf3GwA\n+mGh26HgB6ghSvJQgyZ4WQKBgQCgGckbAqoXG0swJBOenhwWzCQXdnjuygd9ZdCt\neptARXIn1cZL3ZZcXdYrugBnoPYMjUzFTCWfHCLgPpOUAtDljBzf4h3sIbZt01Vb\nMSqNIW3ZNm8scSshiOHmwLYcdn9Eod7TqQ8PMUlCcw4VFF8KytAc+KRmSf2luIKZ\n3ug10wKBgDQ9GITZraKmBQKwMh6SWBLujr20h4VZk6oOC06lRCoDK6K/aXZUiPVq\nY8de9iYZ8g5ScnYEiiychobisd664AHBLBDtmm5G7mZ2dcTUP68hA2OgkUcFVVOf\nlyPUhxcVCKiA9PPWtSSHJx62FCWq86Ax5NO04hyUSa+KjNnLISOx\n-----END RSA PRIVATE KEY-----\n";

char root_cert[] = "-----BEGIN CERTIFICATE-----\n\
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n\
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n\
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n\
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n\
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n\
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n\
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n\
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n\
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n\
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n\
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n\
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n\
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n\
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n\
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n\
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n\
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n\
rqXRfboQnoZsG4q5WTP468SQvvG5\n\
-----END CERTIFICATE-----\n";

/*********************** Function Declaration ********************/
static void blink_led(void);
static void configure_led(void);
static int read_sensor(void);
static void createStringFromJson(char *str);

/********************** Fundion Definition **************************************/

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLUE_LED_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLUE_LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLUE_LED_GPIO, GPIO_MODE_OUTPUT);
}

static void createStringFromJson(char *str)
{
    cJSON *root;
	root = cJSON_CreateObject();
	cJSON_AddStringToObject(root, "id", id);
	cJSON_AddNumberToObject(root, "CO", CO);
	cJSON_AddNumberToObject(root, "Alcohol", Alcohol);
    cJSON_AddNumberToObject(root, "CO2", (CO2 + 400));
    cJSON_AddNumberToObject(root, "Toluen", Toluen);
    cJSON_AddNumberToObject(root, "NH4", NH4);
    cJSON_AddNumberToObject(root, "Aceton", Aceton);
	char *my_json_string = cJSON_Print(root);
    strcpy(str, my_json_string);
	ESP_LOGI(TAG, "my_json_string:\n%s",my_json_string);
	cJSON_Delete(root);
}

static int read_sensor(void)
{
    update(); // Update data, the arduino will read the voltage from the analog pin

    setA(605.18); setB(-3.937); // Configure the equation to calculate CO concentration value
    CO = readSensor(false, 0.0, false); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

    setA(77.255); setB(-3.18); //Configure the equation to calculate Alcohol concentration value
    Alcohol = readSensor(false, 0.0, false); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup

    setA(110.47); setB(-2.862); // Configure the equation to calculate CO2 concentration value
    CO2 = readSensor(false, 0.0, false); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

    setA(44.947); setB(-3.445); // Configure the equation to calculate Toluen concentration value
    Toluen = readSensor(false, 0.0, false); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  
    setA(102.2 ); setB(-2.473); // Configure the equation to calculate NH4 concentration value
    NH4 = readSensor(false, 0.0, false); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

    setA(34.668); setB(-3.369); // Configure the equation to calculate Aceton concentration value
    
    Aceton = readSensor(false, 0.0, false); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    printf("|   %f",CO); 
    printf("   |   %f",Alcohol);
    // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
/*
Motivation:
We have added 400 PPM because when the library is calibrated it assumes the current state of the
air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.
https://www.lavanguardia.com/natural/20190514/462242832581/concentracion-dioxido-cabono-co2-atmosfera-bate-record-historia-humanidad.html
*/
    printf("   |   %f",(CO2 + 400)); 
    printf("   |   %f",Toluen); 
    printf("   |   %f",NH4); 
    printf("   |   %f",Aceton);
    printf("   |\n"); 

    //createStringFromJson(mqtt_payload);

    sprintf(
        mqtt_payload, 
        "{\"id\":%s, \"CO\":%f, \"Alcohol\":%f, \"CO2\":%f, \"Toluen\":%f, \"NH4\":%f, \"Aceton\": %f}", 
        id, CO, Alcohol, CO2, Toluen, NH4, Aceton
    );

/*
Exponential regression:
GAS      | a      | b
CO       | 605.18 | -3.937  
Alcohol  | 77.255 | -3.18 
CO2      | 110.47 | -2.862
Toluen  | 44.947 | -3.445
NH4      | 102.2  | -2.473
Aceton  | 34.668 | -3.369
*/
    return 1;
}

void mq_sensor_task(void *pvPara)
{
        initADC();
    
    //Set the Power voltage of the MQ-135 sensor
    setVoltResolution(Voltage_Resolution);
    //Set math model to calculate the PPM concentration and the value of constants
    setRegressionMethod(1); //_PPM =  a*ratio^b
    
  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */

/*****************************  MQ Init ********************************************/ 
//Remarks: Configure ADC module
/************************************************************************************/ 
    setR1(R1);
    setR2(R2);
    //RL should be configurated after setting R1 and R2
    setRL(RL);

/*****************************  MQ CAlibration ********************************************/ 
// Explanation: 
// In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
// and on clean air (Calibration conditions), setting up R0 value.
// We recomend executing this routine only on setup in laboratory conditions.
// This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
// Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
    ESP_LOGI(TAG, "Calibrating please wait.");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      update(); // Update data, the arduino will read the voltage from the analog pin
      calcR0 += calibrate(RatioMQ135CleanAir);
      printf(".");
    }
    setR0(calcR0/10);
    ESP_LOGI(TAG, "calcR0 = %f,  done!.", calcR0);
  
    if(isinf(calcR0)) {
        ESP_LOGW(TAG, "Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); 
        while(1);
    }
    if(calcR0 == 0){
        ESP_LOGW(TAG, "Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); 
        while(1);
    }
/*****************************  MQ CAlibration ********************************************/ 

    ESP_LOGI(TAG, "** Values from MQ-135 ****");
    printf("|    CO   |  Alcohol |   CO2  |  Toluen  |  NH4  |  Aceton  |\n");  
    
    for (;;) {
        //ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        //blink_led();
        /* Toggle the LED state */
        //s_led_state = !s_led_state;
        read_sensor();     
        vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %"PRIu32" bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    provisioning_fn(); 

    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());

    /* Configure the peripheral according to the LED type */
    configure_led();

    xTaskCreate(&aws_iot_demo_main, "aws_iot_mqtt", 4096, NULL, 0, NULL);
    //xTaskCreate(&mq_sensor_task, "mq_sensor_task", 4096, NULL, 0, NULL);
}
