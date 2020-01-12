#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "main";


// PIN assignments
#define PIN_LED             4
#define PIN_GPS_CS          2
#define PIN_SPI_READY      13
#define PIN_CLK            18
#define PIN_MISO           19
#define PIN_MOSI           23

// SPI
#define SPI_FREQ      1000000

// Misc
#define UART_BUF_SIZE     256
#define NMEA_BUF_SIZE     512

// GPS
#define GPS_ON "ATGPSON\r"

#define TX_BUFFER_LEN 200
#define RX_BUFFER_LEN 200

void app_main() {
    esp_err_t ret;
    char tx_buffer[TX_BUFFER_LEN];
    char rx_buffer[TX_BUFFER_LEN];
    spi_device_handle_t spi = 0;

    // Sleep for a bit so the monitor can connect to the serial port.
    ESP_LOGI(TAG, "Started (sleeping 3 sec)");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Good to go");

    memset(tx_buffer, 0, sizeof(tx_buffer));
    memset(rx_buffer, 0, sizeof(tx_buffer));
    
    ESP_LOGI(TAG, "Configuring SPI bus");
    spi_bus_config_t buscfg={.miso_io_num = PIN_MISO,
                             .mosi_io_num = PIN_MOSI,
                             .sclk_io_num = PIN_CLK,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = -1
    };

    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    

    ESP_LOGI(TAG, "Configuring SPI device");
    spi_device_interface_config_t devcfg={.clock_speed_hz = SPI_FREQ,
                                          .spics_io_num = PIN_GPS_CS,
                                          .queue_size = 1,
                                          .mode = 0
    };
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // Send transaction to SPI
    ESP_LOGI(TAG, "Transmitting SPI command");

    sprintf(tx_buffer, GPS_ON);
    
    static spi_transaction_t trans;

    trans.flags = 0;
    trans.length = strlen(GPS_ON);
    trans.rxlength = 0;
    trans.tx_buffer = (uint8_t *)tx_buffer;
    trans.rx_buffer = (uint8_t *)rx_buffer;

    ESP_LOGI(TAG, "flags = %d, length = %d, rxlength = %d, txbuffer = '%s'",
             trans.flags, trans.length, trans.rxlength, (char *) trans.tx_buffer);

    ret=spi_device_polling_transmit(spi, &trans); 
    assert(ret==ESP_OK); 

    ESP_LOGI(TAG, "rx_buffer = '%s'", rx_buffer);
    ESP_LOGI(TAG, "rx_data = '%s'", (char *)trans.rx_data);
    
    for (;;) {
        ESP_LOGI(TAG, "Tick");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    
}
