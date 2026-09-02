/*
 * ESP32-S3 LCD_CAM Parallel Output Driver
 *
 * Implements the same API as esp32_i2s_parallel_v2.cpp using the ESP32-S3
 * LCD_CAM peripheral + GDMA channel for 16-bit parallel DMA output.
 *
 * Key differences from ESP32 I2S approach:
 *   - Uses LCD_CAM peripheral (not I2S)
 *   - Uses GDMA (General DMA) instead of the legacy I2S DMA engine
 *   - Source clock is 160 MHz (LCD_CLK from PLL_F160M)
 *   - GPIO matrix signal base: LCD_DATA_OUT0_IDX .. LCD_DATA_OUT15_IDX
 *   - Clock output: LCD_PCLK_IDX
 *   - DMA descriptors: same lldesc_t format (ESP-IDF keeps compatibility)
 *
 * Interrupt strategy:
 *   We use the GDMA OUT_EOF interrupt (not LCD_CAM lcd_trans_done) because:
 *   - GDMA keeps traversing circular descriptor chains automatically
 *   - OUT_EOF fires every time a descriptor with eof=1 is reached
 *   - lcd_trans_done is a one-shot signal that stops the peripheral
 *
 *   After each OUT_EOF, we must re-trigger lcd_start because the LCD_CAM
 *   peripheral stops consuming data when it runs out. The GDMA itself
 *   continues looping, but the LCD output stalls without the re-trigger.
 *
 * References:
 *   - ESP32-S3 Technical Reference Manual, Chapter "LCD_CAM" and "GDMA"
 *   - esp-idf components/hal/lcd_hal.c
 *   - esp-idf components/esp_lcd/src/esp_lcd_panel_io_i80.c
 */

#if defined(CONFIG_IDF_TARGET_ESP32S3)

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_reg.h>
#include <soc/lcd_cam_struct.h>
#include <soc/gdma_reg.h>
#include <soc/gdma_struct.h>
#include <soc/gdma_channel.h>
#include <esp_intr_alloc.h>

#include "esp32s3_lcd_cam_parallel.h"

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static callback shiftCompleteCallback = NULL;
static volatile bool previousBufferFree = true;
static intr_handle_t gdma_intr_handle = NULL;
static int gdma_channel_idx = 0;  // GDMA TX channel index for LCD_CAM

void setShiftCompleteCallback(callback f) 
{
  shiftCompleteCallback = f;
}

bool i2s_parallel_is_previous_buffer_free() 
{
  return previousBufferFree;
}

void i2s_parallel_set_previous_buffer_not_free()
{
  previousBufferFree = false;
}

// ---------------------------------------------------------------------------
// GDMA OUT_EOF Interrupt Handler
//
// Fires every time a descriptor with eof=1 is completed by the GDMA engine.
// GDMA continues traversing the linked list (including circular chains)
// automatically — it does NOT stop on EOF, only generates the interrupt.
//
// We re-trigger the LCD_CAM to keep it pulling data from GDMA continuously.
// ---------------------------------------------------------------------------

static void IRAM_ATTR gdma_out_eof_isr(void* arg)
{
  // Read GDMA TX channel 0 interrupt status
  uint32_t status = GDMA.channel[gdma_channel_idx].out.int_st.val;
  // Clear all pending OUT interrupts
  GDMA.channel[gdma_channel_idx].out.int_clr.val = status;

  // OUT_EOF: bit 1
  if (status & 0x02)
  {
    previousBufferFree = true;

    // Re-trigger LCD_CAM so it keeps consuming data from GDMA.
    // Without this, the LCD peripheral stalls after the first batch.
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;

    if (shiftCompleteCallback)
      shiftCompleteCallback();
  }
}

// ---------------------------------------------------------------------------
// GPIO Setup Helper
// ---------------------------------------------------------------------------

static void iomux_set_signal(int gpio, int signal)
{
  if (gpio < 0) return;
  esp_rom_gpio_pad_select_gpio(gpio);
  gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(gpio, signal, false, false);
}

// ---------------------------------------------------------------------------
// DMA Linked List (same as ESP32 version)
// ---------------------------------------------------------------------------

void link_dma_desc(volatile lldesc_t* dmadesc, volatile lldesc_t* prevdmadesc, void* memory, size_t size)
{
  if (size > DMA_MAX) size = DMA_MAX;

  dmadesc->size = size;
  dmadesc->length = size;
  dmadesc->buf = (uint8_t*)memory;
  dmadesc->eof = 0;
  dmadesc->sosf = 0;
  dmadesc->owner = 1;
  dmadesc->qe.stqe_next = NULL;
  dmadesc->offset = 0;

  if (prevdmadesc)
    prevdmadesc->qe.stqe_next = (lldesc_t*)dmadesc;
}

// ---------------------------------------------------------------------------
// LCD_CAM Parallel Driver Install
// ---------------------------------------------------------------------------

esp_err_t i2s_parallel_driver_install(i2s_port_t port, i2s_parallel_config_t* conf)
{
  (void)port;  // ESP32-S3 has only one LCD_CAM peripheral

  if (conf->sample_width < I2S_PARALLEL_WIDTH_8 || conf->sample_width >= I2S_PARALLEL_WIDTH_MAX)
    return ESP_ERR_INVALID_ARG;

  if (conf->sample_rate < 1 || conf->sample_rate > (I2S_PARALLEL_CLOCK_HZ / 2))
    return ESP_ERR_INVALID_ARG;

  // ---- Enable LCD_CAM peripheral clock ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // ---- Configure LCD_CAM clock ----
  // LCD_CLK = PLL_F160M / clkm_div_num
  // We want: sample_rate = 160 MHz / div
  uint32_t clk_div = I2S_PARALLEL_CLOCK_HZ / conf->sample_rate;
  if (clk_div < 2) clk_div = 2;
  if (clk_div > 256) clk_div = 256;

  LCD_CAM.lcd_clock.lcd_clk_sel = 2;              // PLL_F160M clock source
  LCD_CAM.lcd_clock.lcd_clkm_div_num = clk_div;
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;       // Use divider
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;          // PCLK idle low
  LCD_CAM.lcd_clock.lcd_ck_out_edge = conf->clkphase ? 1 : 0;

  // ---- Configure LCD_CAM user settings ----
  LCD_CAM.lcd_user.val = 0;
  LCD_CAM.lcd_user.lcd_2byte_en = (conf->sample_width >= I2S_PARALLEL_WIDTH_16) ? 1 : 0;
  LCD_CAM.lcd_user.lcd_byte_order = 0;            // No byte swap
  LCD_CAM.lcd_user.lcd_bit_order = 0;             // MSB first
  LCD_CAM.lcd_user.lcd_dout = 1;                  // Output mode
  LCD_CAM.lcd_user.lcd_always_out_en = 1;         // Keep output lines active
  LCD_CAM.lcd_user.lcd_cmd = 0;                   // No command phase
  LCD_CAM.lcd_user.lcd_dummy = 0;                 // No dummy phase
  // lcd_dout_cyclelen: number of output cycles minus 1.
  // Set to max (0x1FFF = 8191) so each LCD "transaction" outputs 8192 words
  // before needing a re-trigger. The GDMA OUT_EOF ISR re-triggers automatically.
  LCD_CAM.lcd_user.lcd_dout_cyclelen = 0x1FFF;

  // ---- Configure LCD miscellaneous ----
  LCD_CAM.lcd_misc.val = 0;
  LCD_CAM.lcd_misc.lcd_afifo_threshold_num = 11;  // FIFO threshold
  LCD_CAM.lcd_misc.lcd_cd_idle_edge = 0;
  LCD_CAM.lcd_misc.lcd_cd_cmd_set = 0;
  LCD_CAM.lcd_misc.lcd_cd_dummy_set = 0;
  LCD_CAM.lcd_misc.lcd_cd_data_set = 0;

  // ---- GDMA Channel Configuration ----
  gdma_channel_idx = 0;  // Use GDMA TX channel 0 for LCD output

  // Enable GDMA peripheral
  periph_module_enable(PERIPH_GDMA_MODULE);

  // Reset GDMA TX channel
  GDMA.channel[gdma_channel_idx].out.conf0.out_rst = 1;
  GDMA.channel[gdma_channel_idx].out.conf0.out_rst = 0;

  // Configure GDMA TX channel
  GDMA.channel[gdma_channel_idx].out.conf0.out_data_burst_en = 1;
  GDMA.channel[gdma_channel_idx].out.conf0.outdscr_burst_en = 1;
  // out_eof_mode = 1: EOF when data has been popped from FIFO (i.e. actually sent)
  GDMA.channel[gdma_channel_idx].out.conf0.out_eof_mode = 1;
  GDMA.channel[gdma_channel_idx].out.conf0.out_auto_wrback = 0;

  // Connect GDMA TX channel to LCD_CAM peripheral (peripheral ID = 5 on ESP32-S3)
  GDMA.channel[gdma_channel_idx].out.peri_sel.sel = 5;

  // ---- Setup GDMA OUT_EOF interrupt ----
  // Enable out_eof interrupt (bit 1) on this GDMA channel
  GDMA.channel[gdma_channel_idx].out.int_ena.val = 0;
  GDMA.channel[gdma_channel_idx].out.int_ena.out_eof = 1;
  // Clear any pending
  GDMA.channel[gdma_channel_idx].out.int_clr.val = 0xFFFFFFFF;

  esp_err_t err = esp_intr_alloc(ETS_DMA_OUT_CH0_INTR_SOURCE,
                                 ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1,
                                 gdma_out_eof_isr, NULL,
                                 &gdma_intr_handle);
  if (err != ESP_OK) return err;

  // ---- Setup GPIO output signals ----
  int bus_width;
  switch ((i2s_parallel_cfg_bits_t)conf->sample_width) {
    case I2S_PARALLEL_WIDTH_8:  bus_width = 8;  break;
    case I2S_PARALLEL_WIDTH_16: bus_width = 16; break;
    case I2S_PARALLEL_WIDTH_24: bus_width = 24; break;
    default: return ESP_ERR_INVALID_ARG;
  }

  // Map data GPIOs to LCD_DATA_OUT0..15
  for (int i = 0; i < bus_width; i++)
  {
    iomux_set_signal(conf->gpio_bus[i], LCD_DATA_OUT0_IDX + i);
  }
  // Map clock GPIO to LCD_PCLK
  iomux_set_signal(conf->gpio_clk, LCD_PCLK_IDX);

  // Invert clock if phase requested
  if (conf->clkphase)
    GPIO.func_out_sel_cfg[conf->gpio_clk].inv_sel = 1;

  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Start DMA Transfer
// ---------------------------------------------------------------------------

esp_err_t i2s_parallel_send_dma(i2s_port_t port, lldesc_t* dma_descriptor)
{
  (void)port;

  // Stop ongoing transfer
  LCD_CAM.lcd_user.lcd_start = 0;

  // Reset GDMA TX channel
  GDMA.channel[gdma_channel_idx].out.conf0.out_rst = 1;
  GDMA.channel[gdma_channel_idx].out.conf0.out_rst = 0;

  // Clear pending GDMA interrupts and re-enable OUT_EOF
  GDMA.channel[gdma_channel_idx].out.int_clr.val = 0xFFFFFFFF;
  GDMA.channel[gdma_channel_idx].out.int_ena.out_eof = 1;

  // Set DMA out-link descriptor address (20 LSBs)
  GDMA.channel[gdma_channel_idx].out.link.addr = ((uint32_t)dma_descriptor) & 0xFFFFF;
  // Start GDMA (it will traverse the linked list, including circular chains)
  GDMA.channel[gdma_channel_idx].out.link.start = 1;

  // Reset the LCD FIFO to sync with fresh DMA data
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  LCD_CAM.lcd_misc.lcd_afifo_reset = 0;

  // Trigger LCD_CAM to start consuming data from GDMA
  LCD_CAM.lcd_user.lcd_update = 1;
  LCD_CAM.lcd_user.lcd_start = 1;

  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Stop DMA Transfer
// ---------------------------------------------------------------------------

esp_err_t i2s_parallel_stop_dma(i2s_port_t port)
{
  (void)port;

  // Stop LCD output
  LCD_CAM.lcd_user.lcd_start = 0;

  // Stop GDMA
  GDMA.channel[gdma_channel_idx].out.link.stop = 1;

  // Disable GDMA EOF interrupt to prevent spurious callbacks
  GDMA.channel[gdma_channel_idx].out.int_ena.val = 0;
  GDMA.channel[gdma_channel_idx].out.int_clr.val = 0xFFFFFFFF;

  return ESP_OK;
}

#endif // CONFIG_IDF_TARGET_ESP32S3
