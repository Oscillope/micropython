// Original version from https://github.com/adafruit/Adafruit_NeoPixel
// Modifications by dpgeorge to support auto-CPU-frequency detection

// This is a mash-up of the Due show() code + insights from Michael Miller's
// ESP8266 work for the NeoPixelBus library: github.com/Makuna/NeoPixelBus
// Needs to be a separate .c file to enforce ICACHE_RAM_ATTR execution.

#include "py/mpconfig.h"
#include "py/mphal.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "modesp.h"

#define MAX_TRANSFER_SIZE 3*3*400         //3 spi-bits per bit * 3 color bytes * 256 leds

static spi_device_handle_t spi;
static uint8_t* pixBuf = NULL;

// This table generated based on the algorithm at:
// https://www.rogerclark.net/arduino-stm32-neopixels-ws2812b-using-spi-dma/
static const uint32_t spi_encode[0xff] = {
0x24499200, 0x26499200, 0x34499200, 0x36499200, 0xa4499200, 0xa6499200, 0xb4499200, 0xb6499200,
0x244d9200, 0x264d9200, 0x344d9200, 0x364d9200, 0xa44d9200, 0xa64d9200, 0xb44d9200, 0xb64d9200,
0x24699200, 0x26699200, 0x34699200, 0x36699200, 0xa4699200, 0xa6699200, 0xb4699200, 0xb6699200,
0x246d9200, 0x266d9200, 0x346d9200, 0x366d9200, 0xa46d9200, 0xa66d9200, 0xb46d9200, 0xb66d9200,
0x24499300, 0x26499300, 0x34499300, 0x36499300, 0xa4499300, 0xa6499300, 0xb4499300, 0xb6499300,
0x244d9300, 0x264d9300, 0x344d9300, 0x364d9300, 0xa44d9300, 0xa64d9300, 0xb44d9300, 0xb64d9300,
0x24699300, 0x26699300, 0x34699300, 0x36699300, 0xa4699300, 0xa6699300, 0xb4699300, 0xb6699300,
0x246d9300, 0x266d9300, 0x346d9300, 0x366d9300, 0xa46d9300, 0xa66d9300, 0xb46d9300, 0xb66d9300,
0x24499a00, 0x26499a00, 0x34499a00, 0x36499a00, 0xa4499a00, 0xa6499a00, 0xb4499a00, 0xb6499a00,
0x244d9a00, 0x264d9a00, 0x344d9a00, 0x364d9a00, 0xa44d9a00, 0xa64d9a00, 0xb44d9a00, 0xb64d9a00,
0x24699a00, 0x26699a00, 0x34699a00, 0x36699a00, 0xa4699a00, 0xa6699a00, 0xb4699a00, 0xb6699a00,
0x246d9a00, 0x266d9a00, 0x346d9a00, 0x366d9a00, 0xa46d9a00, 0xa66d9a00, 0xb46d9a00, 0xb66d9a00,
0x24499b00, 0x26499b00, 0x34499b00, 0x36499b00, 0xa4499b00, 0xa6499b00, 0xb4499b00, 0xb6499b00,
0x244d9b00, 0x264d9b00, 0x344d9b00, 0x364d9b00, 0xa44d9b00, 0xa64d9b00, 0xb44d9b00, 0xb64d9b00,
0x24699b00, 0x26699b00, 0x34699b00, 0x36699b00, 0xa4699b00, 0xa6699b00, 0xb4699b00, 0xb6699b00,
0x246d9b00, 0x266d9b00, 0x346d9b00, 0x366d9b00, 0xa46d9b00, 0xa66d9b00, 0xb46d9b00, 0xb66d9b00,
0x2449d200, 0x2649d200, 0x3449d200, 0x3649d200, 0xa449d200, 0xa649d200, 0xb449d200, 0xb649d200,
0x244dd200, 0x264dd200, 0x344dd200, 0x364dd200, 0xa44dd200, 0xa64dd200, 0xb44dd200, 0xb64dd200,
0x2469d200, 0x2669d200, 0x3469d200, 0x3669d200, 0xa469d200, 0xa669d200, 0xb469d200, 0xb669d200,
0x246dd200, 0x266dd200, 0x346dd200, 0x366dd200, 0xa46dd200, 0xa66dd200, 0xb46dd200, 0xb66dd200,
0x2449d300, 0x2649d300, 0x3449d300, 0x3649d300, 0xa449d300, 0xa649d300, 0xb449d300, 0xb649d300,
0x244dd300, 0x264dd300, 0x344dd300, 0x364dd300, 0xa44dd300, 0xa64dd300, 0xb44dd300, 0xb64dd300,
0x2469d300, 0x2669d300, 0x3469d300, 0x3669d300, 0xa469d300, 0xa669d300, 0xb469d300, 0xb669d300,
0x246dd300, 0x266dd300, 0x346dd300, 0x366dd300, 0xa46dd300, 0xa66dd300, 0xb46dd300, 0xb66dd300,
0x2449da00, 0x2649da00, 0x3449da00, 0x3649da00, 0xa449da00, 0xa649da00, 0xb449da00, 0xb649da00,
0x244dda00, 0x264dda00, 0x344dda00, 0x364dda00, 0xa44dda00, 0xa64dda00, 0xb44dda00, 0xb64dda00,
0x2469da00, 0x2669da00, 0x3469da00, 0x3669da00, 0xa469da00, 0xa669da00, 0xb469da00, 0xb669da00,
0x246dda00, 0x266dda00, 0x346dda00, 0x366dda00, 0xa46dda00, 0xa66dda00, 0xb46dda00, 0xb66dda00,
0x2449db00, 0x2649db00, 0x3449db00, 0x3649db00, 0xa449db00, 0xa649db00, 0xb449db00, 0xb649db00,
0x244ddb00, 0x264ddb00, 0x344ddb00, 0x364ddb00, 0xa44ddb00, 0xa64ddb00, 0xb44ddb00, 0xb64ddb00,
0x2469db00, 0x2669db00, 0x3469db00, 0x3669db00, 0xa469db00, 0xa669db00, 0xb469db00, 0xb669db00,
0x246ddb00, 0x266ddb00, 0x346ddb00, 0x366ddb00, 0xa46ddb00, 0xa66ddb00, 0xb46ddb00
};

void IRAM_ATTR esp_neopixel_init(uint8_t pin, uint8_t timing) {
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .mosi_io_num=pin,
        .miso_io_num=-1,
        .sclk_io_num=-1,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=MAX_TRANSFER_SIZE,
        .flags=SPICOMMON_BUSFLAG_MASTER
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1200000,                //Clock out at 1.2 MHz (400kHz * 3)
        .mode=0,                                //SPI mode 0
        .command_bits=0,
        .address_bits=0,
        .dummy_bits=0,
        .spics_io_num=-1,                       //CS pin
        .queue_size=16,                         //We want to be able to queue 1 transaction at a time
    };
    if (timing) {
        // Set clock to 2.4 MHz (800kHz * 3)
        devcfg.clock_speed_hz = 2400000;
    }
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    // Allocate a big ol' buffer, of DMA-able memory
    pixBuf = heap_caps_malloc(MAX_TRANSFER_SIZE, MALLOC_CAP_DMA);
}

void IRAM_ATTR esp_neopixel_write(uint8_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t timing) {
    if (pixBuf) {
        // Use DMA transfer
        int i = 0;
        for (i = 0; i < numBytes; i++) {
            *((uint32_t*)(pixBuf + (i * 3))) = spi_encode[pixels[i]] >> 8;
        }
        //ESP_LOGI("espneopixel", "pixBuf %p pixels %p numBytes %u", pixBuf, pixels, numBytes);
        struct spi_transaction_t trans = (struct spi_transaction_t){
            .length = numBytes * 3 * 8, // Length is in bits
            .tx_buffer = pixBuf
        };
        spi_device_queue_trans(spi, &trans, 0);
    } else {
         // Use bit-banging method
        uint8_t *p, *end, pix, mask;
        uint32_t t, time0, time1, period, c, startTime, pinMask;

        pinMask   = 1 << pin;
        p         =  pixels;
        end       =  p + numBytes;
        pix       = *p++;
        mask      = 0x80;
        startTime = 0;

        uint32_t fcpu = ets_get_cpu_frequency() * 1000000;

        if (timing == 1) {
            // 800 KHz
            time0 = (fcpu * 0.35) / 1000000; // 0.35us
            time1 = (fcpu * 0.8) / 1000000; // 0.8us
            period = (fcpu * 1.25) / 1000000; // 1.25us per bit
        } else {
            // 400 KHz
            time0 = (fcpu * 0.5) / 1000000; // 0.35us
            time1 = (fcpu * 1.2) / 1000000; // 0.8us
            period = (fcpu * 2.5) / 1000000; // 1.25us per bit
        }

        uint32_t irq_state = mp_hal_quiet_timing_enter();
        for (t = time0;; t = time0) {
            if (pix & mask) t = time1;                                  // Bit high duration
            while (((c = mp_hal_ticks_cpu()) - startTime) < period);    // Wait for bit start
            GPIO_REG_WRITE(GPIO_OUT_W1TS_REG, pinMask);                 // Set high
            startTime = c;                                              // Save start time
            while (((c = mp_hal_ticks_cpu()) - startTime) < t);         // Wait high duration
            GPIO_REG_WRITE(GPIO_OUT_W1TC_REG, pinMask);                 // Set low
            if (!(mask >>= 1)) {                                        // Next bit/byte
                if(p >= end) break;
                pix  = *p++;
                mask = 0x80;
            }
        }
        while ((mp_hal_ticks_cpu() - startTime) < period); // Wait for last bit
        mp_hal_quiet_timing_exit(irq_state);
    }
}
