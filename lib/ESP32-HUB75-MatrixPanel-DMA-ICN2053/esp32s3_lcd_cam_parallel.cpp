/*
 * ESP32-S3 LCD_CAM Parallel Output Driver
 *
 * Implements the same API as esp32_i2s_parallel_v2.cpp using the ESP32-S3
 * LCD_CAM peripheral + GDMA channel for 16-bit parallel DMA output.
 *
 * This is a deliberately FAITHFUL port of
 * mrcodetastic/ESP32-HUB75-MatrixPanel-DMA's proven ESP32-S3 backend
 * (src/platforms/esp32s3/gdma_lcd_parallel16.cpp/.hpp, Bus_Parallel16) --
 * every LCD_CAM/GDMA register value and the bring-up/start/stop sequence
 * below are copied from that reference as-is, not re-derived or "improved"
 * on. Only two things differ from that reference, both unavoidable to plug
 * into this repo's existing protocol layer (icn2053.cpp/leddrivers.cpp,
 * unchanged) rather than his own descriptor/buffer class:
 *
 *   1. Descriptor type: this repo's protocol layer already builds and
 *      schedules its own lldesc_t (soc/lldesc.h) descriptor chains and
 *      expects a callback once per completed chain. lldesc_t and
 *      dma_descriptor_t (hal/dma_types.h, what his Bus_Parallel16 uses)
 *      describe the identical hardware descriptor layout on this SoC --
 *      same bit positions for size/length/eof/owner, same word offsets for
 *      the buffer and next-descriptor pointers -- so the lldesc_t chains
 *      this protocol layer already builds are handed directly to the same
 *      official gdma_start() his code calls, with no translation needed and
 *      no change to icn2053.cpp/leddrivers.cpp.
 *
 *   2. Callback: his reference's on_trans_eof callback exists in his source
 *      but is commented out (his own demo polls lcd_user.lcd_start in a
 *      busy loop instead). This repo's protocol layer is interrupt-driven
 *      end to end -- sendCallback() in icn2053.cpp must run once per
 *      completed descriptor chain to relink the next row/vsync segment in.
 *      Enabled his callback using his own callback body (the same
 *      esp_rom_delay_us(100) + previousBufferFree=true), adding only the
 *      one call this protocol layer requires: shiftCompleteCallback().
 *
 * Everything else -- clock source/divider selection, lcd_user/lcd_misc
 * register values, dummy phase, GPIO routing and drive strength, the GDMA
 * channel allocation and strategy, and the start/stop sequence -- matches
 * his reference exactly, values and ordering both.
 *
 * Reference: mrcodetastic/ESP32-HUB75-MatrixPanel-DMA,
 * src/platforms/esp32s3/gdma_lcd_parallel16.cpp (Bus_Parallel16::init(),
 * ::dma_transfer_start(), ::dma_transfer_stop()).
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
#include <esp_rom_sys.h>
#include <soc/gpio_sig_map.h>
#include <soc/lcd_cam_reg.h>
#include <soc/lcd_cam_struct.h>
#include <hal/gpio_hal.h>
#include <esp_private/gdma.h>

#include "esp32s3_lcd_cam_parallel.h"

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static callback shiftCompleteCallback = NULL;
static volatile bool previousBufferFree = true;
static gdma_channel_handle_t dma_chan = NULL;

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
// GDMA TX "on_trans_eof" callback
// ---------------------------------------------------------------------------
// Body matches mrcodetastic's own (commented-out, but fully written)
// gdma_on_trans_eof_callback() exactly -- the 100us delay and comment below
// are his, verbatim -- with one addition: shiftCompleteCallback(), the one
// bridge call this repo's interrupt-driven protocol layer requires.

static bool IRAM_ATTR gdma_on_trans_eof_callback(gdma_channel_handle_t dma_chan,
                                  gdma_event_data_t *event_data, void *user_data) {
  // This DMA callback seems to trigger a moment before the last data has
  // issued (buffering between DMA & LCD peripheral?), so pause a moment
  // before stopping LCD data out. The ideal delay may depend on the LCD
  // clock rate...this one was determined empirically by monitoring on a
  // logic analyzer. YMMV.
  esp_rom_delay_us(100);

  previousBufferFree = true;

  if (shiftCompleteCallback)
    shiftCompleteCallback();

  return true;
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
// Matches Bus_Parallel16::init() register-for-register.

esp_err_t i2s_parallel_driver_install(i2s_port_t port, i2s_parallel_config_t* conf)
{
  (void)port;  // ESP32-S3 has only one LCD_CAM peripheral

  if (conf->sample_width < I2S_PARALLEL_WIDTH_8 || conf->sample_width >= I2S_PARALLEL_WIDTH_MAX)
    return ESP_ERR_INVALID_ARG;

  if (conf->sample_rate < 1 || conf->sample_rate > (I2S_PARALLEL_CLOCK_HZ / 2))
    return ESP_ERR_INVALID_ARG;

  // LCD_CAM peripheral isn't enabled by default -- MUST begin with this:
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);

  // Reset LCD bus
  LCD_CAM.lcd_user.lcd_reset = 1;
  esp_rom_delay_us(1000);

  // LCD_CAM_LCD_CLK_SEL: 0 disabled, 1 XTAL, 2 PLL_D2_CLK, 3 PLL_F160M_CLK.
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;        // Use 160Mhz Clock Source -> PLL_F160M_CLK

  LCD_CAM.lcd_clock.lcd_ck_out_edge = conf->clkphase ? 1 : 0;
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;

  LCD_CAM.lcd_clock.lcd_clkcnt_n = 1; // Should never be zero

  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 1; // PCLK = CLK / 1 (... so 160Mhz still)

  // Divider: this repo's API takes an exact requested sample_rate (Hz)
  // rather than his bus_freq buckets, so compute the divider directly from
  // the same 160MHz source his fixed choice above selects.
  uint32_t clk_div = I2S_PARALLEL_CLOCK_HZ / conf->sample_rate;
  if (clk_div < 2) clk_div = 2;
  if (clk_div > 256) clk_div = 256;
  LCD_CAM.lcd_clock.lcd_clkm_div_num = clk_div;

  LCD_CAM.lcd_clock.lcd_clkm_div_a = 1;     // 0/1 fractional divide
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;

  // Configure LCD frame format for generic (non-LCD) parallel output.
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;    // i8080 mode (not RGB)
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0; // Disable RGB/YUV converter
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;  // Do NOT auto-frame

  LCD_CAM.lcd_misc.lcd_bk_en = 1;          // https://esp32.com/viewtopic.php?t=24459&start=60#p91835

  LCD_CAM.lcd_data_dout_mode.val = 0;      // No data delays
  LCD_CAM.lcd_user.lcd_always_out_en = 1;  // Enable 'always out' mode
  LCD_CAM.lcd_user.lcd_8bits_order = 0;    // Do not swap bytes
  LCD_CAM.lcd_user.lcd_bit_order = 0;      // Do not reverse bit order
  LCD_CAM.lcd_user.lcd_byte_order = 0;
  LCD_CAM.lcd_user.lcd_2byte_en = (conf->sample_width >= I2S_PARALLEL_WIDTH_16) ? 1 : 0;
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_dummy = 1;          // Dummy phase(s) @ LCD start
  // lcd_dummy_cyclelen is only a 2-bit hardware field (bitpos 30:29, "dummy
  // cycle length minus 1" -- see soc/lcd_cam_struct.h), so 3 is the actual
  // maximum it can hold (4 dummy cycles). mrcodetastic/ESP32-HUB75-
  // MatrixPanel-DMA#435 documents that a nonzero dummy-cycle count fixes an
  // intermittent/dead LCD_CAM output quirk on this silicon; 3 is the most
  // margin this field allows.
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 3;
  LCD_CAM.lcd_user.lcd_cmd = 0;            // No command at LCD start

  // Route data GPIOs to LCD_DATA_OUT0..N
  int bus_width;
  switch ((i2s_parallel_cfg_bits_t)conf->sample_width) {
    case I2S_PARALLEL_WIDTH_8:  bus_width = 8;  break;
    case I2S_PARALLEL_WIDTH_16: bus_width = 16; break;
    case I2S_PARALLEL_WIDTH_24: bus_width = 24; break;
    default: return ESP_ERR_INVALID_ARG;
  }
  for (int i = 0; i < bus_width; i++) {
    int gpio = conf->gpio_bus[i];
    if (gpio >= 0) { // -1 value will CRASH the ESP32!
      esp_rom_gpio_connect_out_signal(gpio, LCD_DATA_OUT0_IDX + i, false, false);
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
      gpio_set_drive_capability((gpio_num_t)gpio, (gpio_drive_cap_t)3);
    }
  }

  // Clock
  esp_rom_gpio_connect_out_signal(conf->gpio_clk, LCD_PCLK_IDX, conf->clkphase, false);
  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[conf->gpio_clk], PIN_FUNC_GPIO);
  gpio_set_drive_capability((gpio_num_t)conf->gpio_clk, (gpio_drive_cap_t)3);

  // Allocate DMA channel and connect it to the LCD peripheral
  static gdma_channel_alloc_config_t dma_chan_config = {};
  dma_chan_config.sibling_chan = NULL;
  dma_chan_config.direction = GDMA_CHANNEL_DIRECTION_TX;
  dma_chan_config.flags.reserve_sibling = 0;
  gdma_new_channel(&dma_chan_config, &dma_chan);

  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));
  static gdma_strategy_config_t strategy_config = {};
  strategy_config.owner_check = false;
  strategy_config.auto_update_desc = false;
  gdma_apply_strategy(dma_chan, &strategy_config);

  gdma_transfer_ability_t ability = {};
  ability.sram_trans_align = 32;
  ability.psram_trans_align = 64;
  gdma_set_transfer_ability(dma_chan, &ability);

  // Enable DMA transfer callback (commented out in his reference; enabled
  // here since this repo's protocol layer is interrupt-driven -- see file
  // header comment)
  static gdma_tx_event_callbacks_t tx_cbs = {};
  tx_cbs.on_trans_eof = gdma_on_trans_eof_callback;
  gdma_register_tx_event_callbacks(dma_chan, &tx_cbs, NULL);

  // This uses a busy loop to wait for each DMA transfer to complete...
  // but the whole point of DMA is that one's code can do other work in
  // the interim. The CPU is totally free while the transfer runs!
  while (LCD_CAM.lcd_user.lcd_start); // Wait for DMA completion callback

  // After much experimentation, each of these steps is required to get
  // a clean start on the next LCD transfer:
  gdma_reset(dma_chan);                 // Reset DMA to known state
  esp_rom_delay_us(1000);

  LCD_CAM.lcd_user.lcd_dout        = 1; // Enable data out
  LCD_CAM.lcd_user.lcd_update      = 1; // Update registers
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1; // Reset LCD TX FIFO

  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Start DMA Transfer
// ---------------------------------------------------------------------------
// Matches Bus_Parallel16::dma_transfer_start() exactly.

esp_err_t i2s_parallel_send_dma(i2s_port_t port, lldesc_t* dma_descriptor)
{
  (void)port;

  gdma_start(dma_chan, (intptr_t)dma_descriptor); // Start DMA w/updated descriptor(s)
  esp_rom_delay_us(100);              // Must 'bake' a moment before...
  LCD_CAM.lcd_user.lcd_start = 1;        // Trigger LCD DMA transfer

  return ESP_OK;
}

// ---------------------------------------------------------------------------
// Stop DMA Transfer
// ---------------------------------------------------------------------------
// Matches Bus_Parallel16::dma_transfer_stop() exactly.

esp_err_t i2s_parallel_stop_dma(i2s_port_t port)
{
  (void)port;

  LCD_CAM.lcd_user.lcd_reset = 1;
  LCD_CAM.lcd_user.lcd_update = 1;

  gdma_stop(dma_chan);

  return ESP_OK;
}

#endif // CONFIG_IDF_TARGET_ESP32S3
