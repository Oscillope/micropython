// Original version from https://github.com/adafruit/Adafruit_NeoPixel
// Modifications by dpgeorge to support auto-CPU-frequency detection

// This is a mash-up of the Due show() code + insights from Michael Miller's
// ESP8266 work for the NeoPixelBus library: github.com/Makuna/NeoPixelBus
// Needs to be a separate .c file to enforce ICACHE_RAM_ATTR execution.

#include "py/mpconfig.h"
#include "py/mphal.h"
#include "driver/spi_master.h"
#include "modesp.h"

#define MAX_TRANSFER_SIZE 3*3*512

spi_device_handle_t spi;

uint32_t spi_encode(uint8_t color) {
    uint32_t out = 0;
    for (uint8_t mask = 0x80; mask; mask >>= 1) {
        out = out << 3;
        if (color & mask) {
            out = out | 0B110; // ws2812 "1"
        } else {
            out = out | 0B100; // ws2812 "0"
        }
    }
    return out;
}

void IRAM_ATTR esp_neopixel_init(uint8_t pin, uint8_t timing) {
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .mosi_io_num=pin,
        .miso_io_num=-1,
        .sclk_io_num=-1,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=3*3*512         //3 spi-bits per bit * 3 color bytes * 512 leds
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=24*1000*100,            //Clock out at 2.4 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,               //CS pin
        .queue_size=32,                          //We want to be able to queue 32 transactions at a time
    };
    if (!timing) {
            devcfg.clock_speed_hz = 12*1000*100;
    }
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void IRAM_ATTR esp_neopixel_write(uint8_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t timing) {
    int i = 0;
    uint8_t pixBuf[MAX_TRANSFER_SIZE] = { 0 };
    for (i = 0; i < numBytes; i++) {
        pixBuf[i * 3] = spi_encode(*pixels + i) & 0x00ffffff;
    }
    struct spi_transaction_t trans = (struct spi_transaction_t){
        .length = numBytes * 3,
        .tx_buffer = &pixBuf
    };
    spi_device_queue_trans(spi, &trans, 0);
}
