/*
 * ESP32-S3 LCD_CAM Parallel Output Driver
 * 
 * Drop-in replacement for esp32_i2s_parallel_v2 on ESP32-S3.
 * The ESP32-S3 does not support I2S LCD parallel mode; instead it uses the
 * dedicated LCD_CAM peripheral for DMA-driven parallel output.
 *
 * This file exposes the same API as esp32_i2s_parallel_v2.h so the rest of
 * the HUB75 library can remain unchanged.
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

#define I2S_PARALLEL_CLOCK_HZ 160000000L  // ESP32-S3 LCD_CAM source clock (PLL_F160M)
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
