// Original version from https://github.com/adafruit/Adafruit_NeoPixel
// Modifications by dpgeorge to support auto-CPU-frequency detection

// This is a mash-up of the Due show() code + insights from Michael Miller's
// ESP8266 work for the NeoPixelBus library: github.com/Makuna/NeoPixelBus
// Needs to be a separate .c file to enforce ICACHE_RAM_ATTR execution.

#include "py/mpconfig.h"
#include "py/mphal.h"
#include "driver/i2s.h"
#include "modesp.h"

#define MAX_TRANSFER_SIZE 1024         //3 spi-bits per bit * 3 color bytes * 256 leds

static uint8_t* pixBuf = NULL;

// This table generated based on the algorithm at:
// https://www.rogerclark.net/arduino-stm32-neopixels-ws2812b-using-spi-dma/
static const uint32_t spi_encode[0xff] = {
0x00924924, 0x00924926, 0x00924934, 0x00924936, 0x009249a4, 0x009249a6, 0x009249b4, 0x009249b6,
0x00924d24, 0x00924d26, 0x00924d34, 0x00924d36, 0x00924da4, 0x00924da6, 0x00924db4, 0x00924db6,
0x00926924, 0x00926926, 0x00926934, 0x00926936, 0x009269a4, 0x009269a6, 0x009269b4, 0x009269b6,
0x00926d24, 0x00926d26, 0x00926d34, 0x00926d36, 0x00926da4, 0x00926da6, 0x00926db4, 0x00926db6,
0x00934924, 0x00934926, 0x00934934, 0x00934936, 0x009349a4, 0x009349a6, 0x009349b4, 0x009349b6,
0x00934d24, 0x00934d26, 0x00934d34, 0x00934d36, 0x00934da4, 0x00934da6, 0x00934db4, 0x00934db6,
0x00936924, 0x00936926, 0x00936934, 0x00936936, 0x009369a4, 0x009369a6, 0x009369b4, 0x009369b6,
0x00936d24, 0x00936d26, 0x00936d34, 0x00936d36, 0x00936da4, 0x00936da6, 0x00936db4, 0x00936db6,
0x009a4924, 0x009a4926, 0x009a4934, 0x009a4936, 0x009a49a4, 0x009a49a6, 0x009a49b4, 0x009a49b6,
0x009a4d24, 0x009a4d26, 0x009a4d34, 0x009a4d36, 0x009a4da4, 0x009a4da6, 0x009a4db4, 0x009a4db6,
0x009a6924, 0x009a6926, 0x009a6934, 0x009a6936, 0x009a69a4, 0x009a69a6, 0x009a69b4, 0x009a69b6,
0x009a6d24, 0x009a6d26, 0x009a6d34, 0x009a6d36, 0x009a6da4, 0x009a6da6, 0x009a6db4, 0x009a6db6,
0x009b4924, 0x009b4926, 0x009b4934, 0x009b4936, 0x009b49a4, 0x009b49a6, 0x009b49b4, 0x009b49b6,
0x009b4d24, 0x009b4d26, 0x009b4d34, 0x009b4d36, 0x009b4da4, 0x009b4da6, 0x009b4db4, 0x009b4db6,
0x009b6924, 0x009b6926, 0x009b6934, 0x009b6936, 0x009b69a4, 0x009b69a6, 0x009b69b4, 0x009b69b6,
0x009b6d24, 0x009b6d26, 0x009b6d34, 0x009b6d36, 0x009b6da4, 0x009b6da6, 0x009b6db4, 0x009b6db6,
0x00d24924, 0x00d24926, 0x00d24934, 0x00d24936, 0x00d249a4, 0x00d249a6, 0x00d249b4, 0x00d249b6,
0x00d24d24, 0x00d24d26, 0x00d24d34, 0x00d24d36, 0x00d24da4, 0x00d24da6, 0x00d24db4, 0x00d24db6,
0x00d26924, 0x00d26926, 0x00d26934, 0x00d26936, 0x00d269a4, 0x00d269a6, 0x00d269b4, 0x00d269b6,
0x00d26d24, 0x00d26d26, 0x00d26d34, 0x00d26d36, 0x00d26da4, 0x00d26da6, 0x00d26db4, 0x00d26db6,
0x00d34924, 0x00d34926, 0x00d34934, 0x00d34936, 0x00d349a4, 0x00d349a6, 0x00d349b4, 0x00d349b6,
0x00d34d24, 0x00d34d26, 0x00d34d34, 0x00d34d36, 0x00d34da4, 0x00d34da6, 0x00d34db4, 0x00d34db6,
0x00d36924, 0x00d36926, 0x00d36934, 0x00d36936, 0x00d369a4, 0x00d369a6, 0x00d369b4, 0x00d369b6,
0x00d36d24, 0x00d36d26, 0x00d36d34, 0x00d36d36, 0x00d36da4, 0x00d36da6, 0x00d36db4, 0x00d36db6,
0x00da4924, 0x00da4926, 0x00da4934, 0x00da4936, 0x00da49a4, 0x00da49a6, 0x00da49b4, 0x00da49b6,
0x00da4d24, 0x00da4d26, 0x00da4d34, 0x00da4d36, 0x00da4da4, 0x00da4da6, 0x00da4db4, 0x00da4db6,
0x00da6924, 0x00da6926, 0x00da6934, 0x00da6936, 0x00da69a4, 0x00da69a6, 0x00da69b4, 0x00da69b6,
0x00da6d24, 0x00da6d26, 0x00da6d34, 0x00da6d36, 0x00da6da4, 0x00da6da6, 0x00da6db4, 0x00da6db6,
0x00db4924, 0x00db4926, 0x00db4934, 0x00db4936, 0x00db49a4, 0x00db49a6, 0x00db49b4, 0x00db49b6,
0x00db4d24, 0x00db4d26, 0x00db4d34, 0x00db4d36, 0x00db4da4, 0x00db4da6, 0x00db4db4, 0x00db4db6,
0x00db6924, 0x00db6926, 0x00db6934, 0x00db6936, 0x00db69a4, 0x00db69a6, 0x00db69b4, 0x00db69b6,
0x00db6d24, 0x00db6d26, 0x00db6d34, 0x00db6d36, 0x00db6da4, 0x00db6da6, 0x00db6db4};
uint32_t convert(uint8_t data)
{
  uint32_t out=0;
  for(uint8_t mask = 0x80; mask; mask >>= 1)  
  {
    out=out<<3;
    if (data & mask)
    {
      out = out | 0b110;//Bit high
    }
    else
    {
      out = out | 0b100;// bit low
    }
  }
  return out;
}

void IRAM_ATTR esp_neopixel_init(uint8_t pin, uint8_t timing) {
    static const i2s_config_t i2s_config = {
         .mode = I2S_MODE_MASTER | I2S_MODE_TX,
         .sample_rate = 35191,
         .bits_per_sample = 32,
         .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
         .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
         .intr_alloc_flags = 0, // default interrupt priority
         .dma_buf_count = 8,
         .dma_buf_len = MAX_TRANSFER_SIZE,
         .use_apll = true,
         .fixed_mclk = 2252252
    };
    static const i2s_pin_config_t pin_config = {
        .bck_io_num = 26,
        .ws_io_num = 25,
        .data_out_num = 22,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(0, &i2s_config, 0, NULL);
    i2s_set_pin(0, &pin_config);

    // Allocate a big ol' buffer, of DMA-able memory
    pixBuf = heap_caps_malloc(MAX_TRANSFER_SIZE, MALLOC_CAP_DMA);
    //int i;
    //for (i = 0; i < MAX_TRANSFER_SIZE/4; i++) {
    //        pixBuf[i * 4] = i;
    //}
}

void IRAM_ATTR esp_neopixel_write(uint8_t pin, uint8_t *pixels, uint32_t numBytes, uint8_t timing) {
    if (pixBuf) {
        // Use DMA transfer
        int i = 0;
        size_t written;
        i2s_zero_dma_buffer(0);
        for (i = 0; i < numBytes; i++) {
            pixBuf[i * 3] = spi_encode[pixels[i]] & 0x00ffffff;
            //pixBuf[i * 3] = convert(*(pixels + i));
        }
        i2s_write(0, pixBuf, MAX_TRANSFER_SIZE, &written, 500);
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
