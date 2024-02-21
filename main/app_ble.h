#ifndef APP_BLE_H_
#define APP_BLE_H_

    #include "esp_gap_ble_api.h"
    #include "esp_gatts_api.h"
    #include "esp_bt_defs.h"
    #include "esp_bt_main.h"
    #include "esp_gatt_common_api.h"

    #include "protocol_examples_common.h"

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/semphr.h"
    #include "freertos/queue.h"
    #include "freertos/event_groups.h"

    #include "esp_wifi.h"

    #include "nvs.h"
    #include "nvs_flash.h"

    #include "esp_event.h"
    #include "esp_netif.h"
    #include "esp_bt.h"
    #include "esp_chip_info.h"
    #include "esp_flash.h"


    #include "lwip/sockets.h"
    #include "lwip/dns.h"
    #include "lwip/netdb.h"
    #include "lwip/err.h"
    #include "lwip/sys.h"

    #include "esp_err.h"
    #include "esp_log.h"
    #include "sdkconfig.h"

    esp_err_t provisioning_fn(void);
#endif