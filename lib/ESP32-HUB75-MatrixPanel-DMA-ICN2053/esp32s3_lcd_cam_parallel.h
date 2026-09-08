/*
 * ESP32-S3 LCD_CAM Parallel Output Driver
 *
 * Drop-in replacement for esp32_i2s_parallel_v2 on ESP32-S3, which has no
 * I2S LCD parallel mode -- it uses the dedicated LCD_CAM peripheral (driven
 * via GDMA) for DMA-driven parallel output instead.
 *
 * This header exposes the same external API as esp32_i2s_parallel_v2.h so
 * the rest of this library (icn2053.cpp/leddrivers.cpp -- this repo's own
 * ICN1065/ICN2053 packet protocol and row scheduler) needs no changes to
 * build for S3. The implementation (esp32s3_lcd_cam_parallel.cpp) is a
 * deliberately faithful, register-for-register port of
 * mrcodetastic/ESP32-HUB75-MatrixPanel-DMA's proven S3 backend
 * (src/platforms/esp32s3/gdma_lcd_parallel16.cpp) -- see that file's header
 * comment for the two unavoidable adaptations (descriptor type, enabling
 * his own commented-out callback) needed to plug into this repo's existing
 * protocol layer, and why nothing else was changed from his values.
 */

#pragma once

#if defined(CONFIG_IDF_TARGET_ESP32S3)

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include <soc/lldesc.h>

#ifdef __cplusplus
extern "C" {
#endif

// ESP32-S3 LCD_CAM source clock: PLL_F160M, matching
// mrcodetastic/ESP32-HUB75-MatrixPanel-DMA's proven backend exactly
// (lcd_clk_sel=3 in esp32s3_lcd_cam_parallel.cpp).
#define I2S_PARALLEL_CLOCK_HZ 160000000L
#define DMA_MAX (4096-4)

typedef enum
{
  I2S_PARALLEL_WIDTH_8,
  I2S_PARALLEL_WIDTH_16,
  I2S_PARALLEL_WIDTH_24,
  I2S_PARALLEL_WIDTH_MAX
} i2s_parallel_cfg_bits_t;

typedef struct
{
    int gpio_bus[24];   // The parallel GPIOs to use, set gpio to -1 to disable
    int gpio_clk;
    int sample_rate;    // Pixel clock frequency in Hz
    int sample_width;
    int desccount_a;
    lldesc_t * lldesc_a;
    int desccount_b;
    lldesc_t * lldesc_b;
    bool clkphase;        // Clock signal phase inversion
} i2s_parallel_config_t;

// Port type for API compatibility (ESP32-S3 LCD_CAM is a single peripheral, port is ignored)
typedef enum {
    I2S_NUM_0 = 0,
    I2S_NUM_1 = 1,
    I2S_NUM_MAX
} i2s_port_t;

static inline int i2s_parallel_get_memory_width(i2s_port_t port, i2s_parallel_cfg_bits_t width)
{
  (void)port;
  switch(width)
  {
    case I2S_PARALLEL_WIDTH_8:
      return 1;
    case I2S_PARALLEL_WIDTH_16:
      return 2;
    case I2S_PARALLEL_WIDTH_24:
      return 4;
    default:
      return -ESP_ERR_INVALID_ARG;
  }
}

// DMA Linked List Creation (same signature as ESP32 version)
void link_dma_desc(volatile lldesc_t* dmadesc, volatile lldesc_t* prevdmadesc, void* memory, size_t size);

// LCD_CAM Parallel Output Functions (same API as I2S parallel)
esp_err_t   i2s_parallel_driver_install(i2s_port_t port, i2s_parallel_config_t* conf);
esp_err_t   i2s_parallel_send_dma(i2s_port_t port, lldesc_t* dma_descriptor);
esp_err_t   i2s_parallel_stop_dma(i2s_port_t port);

// Callback function for when DMA chain has been sent (EOF interrupt)
typedef void (*callback)(void);
void setShiftCompleteCallback(callback f);
void i2s_parallel_set_previous_buffer_not_free();
bool i2s_parallel_is_previous_buffer_free();

#ifdef __cplusplus
}
#endif

#endif // CONFIG_IDF_TARGET_ESP32S3
