/*
 * LCD_CAM DMA Test Suite for ESP32-S3
 *
 * Tests the esp32s3_lcd_cam_parallel driver in isolation to verify:
 *   1. Driver installation (peripheral + GDMA init)
 *   2. DMA descriptor linking (lldesc_t chain)
 *   3. DMA buffer allocation and content
 *   4. DMA transfer start + EOF callback fires
 *   5. DMA stop
 *   6. Clock divider calculation
 *   7. Continuous (looped) descriptor chain
 *
 * Build with: pio test -e esp32-s3-devkitc-1
 * Or as a standalone sketch: pio run -e esp32-s3-test
 *
 * Connect a logic analyzer to the CLK pin (GPIO2) to observe output.
 * No HUB75 panel required for these tests.
 */

#include <Arduino.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3_lcd_cam_parallel.h"
#else
#include "esp32_i2s_parallel_v2.h"
#endif

// ─── Test Configuration ────────────────────────────────────────────────────────
// Use minimal pin set for testing (only CLK and one data line needed)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define TEST_CLK_PIN  2
  #define TEST_DATA_PIN 4   // Just one data pin to verify output
#else
  #define TEST_CLK_PIN  16
  #define TEST_DATA_PIN 25
#endif

#define TEST_DMA_CLOCK_HZ  5000000   // 5 MHz — slow enough to probe easily
#define TEST_BUF_WORDS     64        // 64 x 16-bit words per DMA buffer
#define TEST_BUF_BYTES     (TEST_BUF_WORDS * sizeof(uint16_t))

// ─── Test State ────────────────────────────────────────────────────────────────
static volatile uint32_t callback_count = 0;
static volatile uint32_t callback_time_us = 0;

static void IRAM_ATTR test_dma_callback()
{
  callback_count++;
  callback_time_us = micros();
}

// ─── Helpers ───────────────────────────────────────────────────────────────────
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
  if (cond) { \
    Serial.printf("  [PASS] %s\n", msg); \
    tests_passed++; \
  } else { \
    Serial.printf("  [FAIL] %s\n", msg); \
    tests_failed++; \
  } \
} while(0)

#define TEST_ASSERT_EQ(a, b, msg) do { \
  if ((a) == (b)) { \
    Serial.printf("  [PASS] %s (got %d)\n", msg, (int)(a)); \
    tests_passed++; \
  } else { \
    Serial.printf("  [FAIL] %s (expected %d, got %d)\n", msg, (int)(b), (int)(a)); \
    tests_failed++; \
  } \
} while(0)

// ─── Test 1: DMA Descriptor Linking ───────────────────────────────────────────
void test_descriptor_linking()
{
  Serial.println("\n=== Test 1: DMA Descriptor Linking ===");

  // Allocate a chain of 4 descriptors
  lldesc_t descs[4];
  uint8_t buf0[128], buf1[128], buf2[128], buf3[64];

  memset(descs, 0, sizeof(descs));

  link_dma_desc(&descs[0], NULL,      buf0, sizeof(buf0));
  link_dma_desc(&descs[1], &descs[0], buf1, sizeof(buf1));
  link_dma_desc(&descs[2], &descs[1], buf2, sizeof(buf2));
  link_dma_desc(&descs[3], &descs[2], buf3, sizeof(buf3));

  // Verify chain pointers
  TEST_ASSERT(descs[0].qe.stqe_next == &descs[1], "desc[0]->next == desc[1]");
  TEST_ASSERT(descs[1].qe.stqe_next == &descs[2], "desc[1]->next == desc[2]");
  TEST_ASSERT(descs[2].qe.stqe_next == &descs[3], "desc[2]->next == desc[3]");
  TEST_ASSERT(descs[3].qe.stqe_next == NULL,      "desc[3]->next == NULL (end)");

  // Verify sizes
  TEST_ASSERT_EQ(descs[0].size, 128, "desc[0].size == 128");
  TEST_ASSERT_EQ(descs[3].size, 64,  "desc[3].size == 64");

  // Verify buffer pointers
  TEST_ASSERT(descs[0].buf == buf0, "desc[0].buf points to buf0");
  TEST_ASSERT(descs[3].buf == buf3, "desc[3].buf points to buf3");

  // Verify owner bit (for DMA)
  TEST_ASSERT_EQ(descs[0].owner, 1, "desc[0].owner == 1 (DMA owns)");

  // Test DMA_MAX clamping
  lldesc_t big_desc;
  uint8_t* big_buf = (uint8_t*)malloc(DMA_MAX + 100);
  TEST_ASSERT(big_buf != NULL, "big_buf heap allocated for clamping test");
  if (big_buf) {
    link_dma_desc(&big_desc, NULL, big_buf, DMA_MAX + 100);
    TEST_ASSERT(big_desc.size <= DMA_MAX, "big buffer clamped to DMA_MAX");
    free(big_buf);
  }
}

// ─── Test 2: Circular Descriptor Chain ────────────────────────────────────────
void test_circular_chain()
{
  Serial.println("\n=== Test 2: Circular Descriptor Chain ===");

  lldesc_t descs[3];
  uint8_t bufs[3][128];
  memset(descs, 0, sizeof(descs));

  link_dma_desc(&descs[0], NULL,      bufs[0], 128);
  link_dma_desc(&descs[1], &descs[0], bufs[1], 128);
  link_dma_desc(&descs[2], &descs[1], bufs[2], 128);

  // Make circular: last -> first
  descs[2].qe.stqe_next = &descs[0];
  descs[2].eof = 1;  // trigger interrupt at end of loop

  TEST_ASSERT(descs[2].qe.stqe_next == &descs[0], "circular: desc[2]->next == desc[0]");
  TEST_ASSERT_EQ(descs[2].eof, 1, "desc[2].eof == 1 (interrupt trigger)");

  // Walk the chain and verify we loop back
  lldesc_t* current = &descs[0];
  for (int i = 0; i < 6; i++) {  // walk 2 full loops
    current = (lldesc_t*)current->qe.stqe_next;
  }
  TEST_ASSERT(current == &descs[0], "after 6 steps, back at desc[0] (2 loops)");
}

// ─── Test 3: Buffer Allocation (DMA-capable) ─────────────────────────────────
void test_buffer_allocation()
{
  Serial.println("\n=== Test 3: DMA Buffer Allocation ===");

  // Allocate DMA-capable memory (like the library does)
  size_t free_before = heap_caps_get_free_size(MALLOC_CAP_DMA);
  Serial.printf("  DMA heap free before: %d bytes\n", free_before);

  uint16_t* dma_buf = (uint16_t*)heap_caps_calloc(TEST_BUF_WORDS, sizeof(uint16_t), MALLOC_CAP_DMA);
  TEST_ASSERT(dma_buf != NULL, "DMA buffer allocated successfully");

  size_t free_after = heap_caps_get_free_size(MALLOC_CAP_DMA);
  Serial.printf("  DMA heap free after: %d bytes (used %d)\n", free_after, free_before - free_after);
  TEST_ASSERT(free_before - free_after >= TEST_BUF_BYTES, "heap usage >= buffer size");

  // Fill with test pattern
  for (int i = 0; i < TEST_BUF_WORDS; i++) {
    dma_buf[i] = (uint16_t)(0xA500 | (i & 0xFF));
  }
  TEST_ASSERT_EQ(dma_buf[0], 0xA500, "buffer[0] == 0xA500");
  TEST_ASSERT_EQ(dma_buf[63], 0xA53F, "buffer[63] == 0xA53F");

  // Verify alignment (DMA requires word alignment)
  TEST_ASSERT(((uint32_t)dma_buf & 0x3) == 0, "DMA buffer is 4-byte aligned");

  heap_caps_free(dma_buf);

  // Verify we can allocate descriptor memory too
  lldesc_t* desc = (lldesc_t*)heap_caps_calloc(4, sizeof(lldesc_t), MALLOC_CAP_DMA);
  TEST_ASSERT(desc != NULL, "DMA descriptor array allocated");
  TEST_ASSERT(((uint32_t)desc & 0x3) == 0, "descriptor array is 4-byte aligned");
  heap_caps_free(desc);
}

// ─── Test 4: Driver Install ──────────────────────────────────────────────────
void test_driver_install()
{
  Serial.println("\n=== Test 4: Driver Install ===");

  // Set up a minimal config — only CLK and one data pin
  i2s_parallel_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));

  // Set all gpio_bus entries to -1 (disabled)
  for (int i = 0; i < 24; i++) cfg.gpio_bus[i] = -1;

  // Enable just one data pin for verification
  cfg.gpio_bus[0] = TEST_DATA_PIN;
  cfg.gpio_clk = TEST_CLK_PIN;
  cfg.sample_rate = TEST_DMA_CLOCK_HZ;
  cfg.sample_width = I2S_PARALLEL_WIDTH_16;
  cfg.clkphase = false;

  // Allocate a small DMA buffer and link descriptors
  uint16_t* buf = (uint16_t*)heap_caps_calloc(TEST_BUF_WORDS, sizeof(uint16_t), MALLOC_CAP_DMA);
  TEST_ASSERT(buf != NULL, "test buffer allocated for driver install");

  lldesc_t desc;
  link_dma_desc(&desc, NULL, buf, TEST_BUF_BYTES);
  desc.eof = 1;  // trigger interrupt at end
  desc.qe.stqe_next = &desc;  // loop for continuous output

  cfg.desccount_a = 1;
  cfg.lldesc_a = &desc;
  cfg.desccount_b = 0;
  cfg.lldesc_b = NULL;

  // Install driver
  setShiftCompleteCallback(test_dma_callback);

  esp_err_t err = i2s_parallel_driver_install(I2S_NUM_1, &cfg);
  TEST_ASSERT(err == ESP_OK, "i2s_parallel_driver_install() returned ESP_OK");

  heap_caps_free(buf);
}

// ─── Test 5: DMA Transfer + Callback ─────────────────────────────────────────
void test_dma_transfer()
{
  Serial.println("\n=== Test 5: DMA Transfer + Callback ===");

  // Allocate buffer with a known pattern
  uint16_t* buf = (uint16_t*)heap_caps_calloc(TEST_BUF_WORDS, sizeof(uint16_t), MALLOC_CAP_DMA);
  TEST_ASSERT(buf != NULL, "transfer test buffer allocated");
  if (!buf) return;

  // Fill with alternating pattern: bit0 set on even words (data pin toggles)
  for (int i = 0; i < TEST_BUF_WORDS; i++) {
    buf[i] = (i & 1) ? 0x0000 : 0x0001;  // toggle data pin 0
  }

  // Set up descriptor chain: 1 buffer, looped, with EOF
  lldesc_t desc;
  link_dma_desc(&desc, NULL, buf, TEST_BUF_BYTES);
  desc.eof = 1;
  desc.qe.stqe_next = &desc;  // loop

  // Reset callback counter
  callback_count = 0;
  callback_time_us = 0;
  i2s_parallel_set_previous_buffer_not_free();

  uint32_t t_start = micros();

  // Start DMA
  esp_err_t err = i2s_parallel_send_dma(I2S_NUM_1, &desc);
  TEST_ASSERT(err == ESP_OK, "i2s_parallel_send_dma() returned ESP_OK");

  // Wait for at least one callback (timeout: 500ms)
  uint32_t timeout = millis() + 500;
  while (callback_count == 0 && millis() < timeout) {
    delay(1);
  }

  uint32_t elapsed_us = callback_time_us - t_start;

  TEST_ASSERT(callback_count > 0, "DMA EOF callback fired at least once");
  Serial.printf("  Callbacks received: %d (first after %d us)\n", callback_count, elapsed_us);

  // At 5 MHz, 64 words = 64 clocks = 12.8 us per loop
  // We should get many callbacks in 500ms
  if (callback_count > 0) {
    TEST_ASSERT(elapsed_us < 10000, "first callback within 10ms (expected <1ms)");
  }

  // Wait a bit longer and check continuous callbacks
  uint32_t count_before = callback_count;
  delay(50);
  uint32_t count_after = callback_count;
  TEST_ASSERT(count_after > count_before, "callbacks continue (DMA is looping)");
  Serial.printf("  Callbacks in 50ms: %d\n", count_after - count_before);

  // Test previousBufferFree flag
  TEST_ASSERT(i2s_parallel_is_previous_buffer_free() == true, "previousBufferFree set by callback");

  heap_caps_free(buf);
}

// ─── Test 6: DMA Stop ─────────────────────────────────────────────────────────
void test_dma_stop()
{
  Serial.println("\n=== Test 6: DMA Stop ===");

  esp_err_t err = i2s_parallel_stop_dma(I2S_NUM_1);
  TEST_ASSERT(err == ESP_OK, "i2s_parallel_stop_dma() returned ESP_OK");

  // After stop, no more callbacks should fire
  uint32_t count_before = callback_count;
  delay(50);
  uint32_t count_after = callback_count;

  TEST_ASSERT(count_after == count_before, "no new callbacks after DMA stop");
  Serial.printf("  Callbacks after stop (50ms wait): %d (should be 0)\n", count_after - count_before);
}

// ─── Test 7: Multi-Descriptor Transfer ────────────────────────────────────────
void test_multi_descriptor()
{
  Serial.println("\n=== Test 7: Multi-Descriptor Chain Transfer ===");

  // Allocate 3 buffers with distinct patterns
  const int NUM_DESCS = 3;
  uint16_t* bufs[NUM_DESCS];
  lldesc_t descs[NUM_DESCS];

  for (int i = 0; i < NUM_DESCS; i++) {
    bufs[i] = (uint16_t*)heap_caps_calloc(TEST_BUF_WORDS, sizeof(uint16_t), MALLOC_CAP_DMA);
    TEST_ASSERT(bufs[i] != NULL, "multi-desc buffer allocated");
    // Fill each buffer with a unique pattern
    for (int j = 0; j < TEST_BUF_WORDS; j++) {
      bufs[i][j] = (uint16_t)((i << 8) | j);
    }
  }

  // Link: desc[0] -> desc[1] -> desc[2] -> desc[0] (loop)
  link_dma_desc(&descs[0], NULL,      bufs[0], TEST_BUF_BYTES);
  link_dma_desc(&descs[1], &descs[0], bufs[1], TEST_BUF_BYTES);
  link_dma_desc(&descs[2], &descs[1], bufs[2], TEST_BUF_BYTES);
  descs[2].qe.stqe_next = &descs[0];  // circular
  descs[2].eof = 1;                     // interrupt after full chain

  // Reset and start
  callback_count = 0;
  esp_err_t err = i2s_parallel_send_dma(I2S_NUM_1, &descs[0]);
  TEST_ASSERT(err == ESP_OK, "multi-desc send_dma returned ESP_OK");

  // Wait for callbacks
  delay(100);
  TEST_ASSERT(callback_count > 0, "multi-desc: callbacks received");
  Serial.printf("  Multi-desc callbacks in 100ms: %d\n", callback_count);

  // Each loop is 3*64 words @ 5MHz = 192 clocks = 38.4 us
  // Expect many loops in 100ms
  TEST_ASSERT(callback_count > 100, "multi-desc: >100 loops in 100ms (high throughput)");

  // Stop
  i2s_parallel_stop_dma(I2S_NUM_1);

  for (int i = 0; i < NUM_DESCS; i++) heap_caps_free(bufs[i]);
}

// ─── Test 8: Descriptor Re-linking (simulates row switching) ──────────────────
void test_descriptor_relinking()
{
  Serial.println("\n=== Test 8: Descriptor Re-linking (Row Switch Simulation) ===");

  // This simulates what the library does in sendCBRow/sendCBVsync:
  // dynamically relinking descriptor chains during DMA operation

  lldesc_t suffix_desc, data_desc_a, data_desc_b;
  uint16_t* suffix_buf = (uint16_t*)heap_caps_calloc(32, sizeof(uint16_t), MALLOC_CAP_DMA);
  uint16_t* data_buf_a = (uint16_t*)heap_caps_calloc(32, sizeof(uint16_t), MALLOC_CAP_DMA);
  uint16_t* data_buf_b = (uint16_t*)heap_caps_calloc(32, sizeof(uint16_t), MALLOC_CAP_DMA);

  TEST_ASSERT(suffix_buf && data_buf_a && data_buf_b, "relink buffers allocated");

  // Fill with identifiable patterns
  for (int i = 0; i < 32; i++) {
    suffix_buf[i] = 0xFF00;
    data_buf_a[i] = 0x00AA;
    data_buf_b[i] = 0x0055;
  }

  // Set up: suffix loops to itself initially
  link_dma_desc(&suffix_desc, NULL, suffix_buf, 64);
  suffix_desc.eof = 1;
  suffix_desc.qe.stqe_next = &suffix_desc;  // self-loop

  link_dma_desc(&data_desc_a, NULL, data_buf_a, 64);
  data_desc_a.eof = 1;
  data_desc_a.qe.stqe_next = &suffix_desc;  // data_a -> suffix

  link_dma_desc(&data_desc_b, NULL, data_buf_b, 64);
  data_desc_b.eof = 1;
  data_desc_b.qe.stqe_next = &suffix_desc;  // data_b -> suffix

  // Start with suffix looping
  callback_count = 0;
  esp_err_t err = i2s_parallel_send_dma(I2S_NUM_1, &suffix_desc);
  TEST_ASSERT(err == ESP_OK, "relink: initial suffix loop started");
  delay(20);
  TEST_ASSERT(callback_count > 0, "relink: suffix looping generates callbacks");

  // Now relink: suffix -> data_a -> suffix (simulates sendCBRow)
  suffix_desc.qe.stqe_next = &data_desc_a;
  delay(20);
  uint32_t count_with_data = callback_count;
  TEST_ASSERT(count_with_data > 0, "relink: data_a in chain generates callbacks");

  // Switch to data_b (simulates double-buffer row switch)
  suffix_desc.qe.stqe_next = &data_desc_b;
  delay(20);
  TEST_ASSERT(callback_count > count_with_data, "relink: data_b switch generates callbacks");

  // Return to self-loop
  suffix_desc.qe.stqe_next = &suffix_desc;
  delay(10);

  i2s_parallel_stop_dma(I2S_NUM_1);
  Serial.printf("  Total callbacks during relink test: %d\n", callback_count);

  heap_caps_free(suffix_buf);
  heap_caps_free(data_buf_a);
  heap_caps_free(data_buf_b);
}

// ─── Test 9: Clock Frequency Verification ────────────────────────────────────
void test_clock_frequency()
{
  Serial.println("\n=== Test 9: Clock Frequency (Timing) ===");

  // We can indirectly verify clock by measuring how fast a known buffer
  // completes. At 5 MHz with 64 16-bit words, each loop = 64/5MHz = 12.8 us

  uint16_t* buf = (uint16_t*)heap_caps_calloc(256, sizeof(uint16_t), MALLOC_CAP_DMA);
  TEST_ASSERT(buf != NULL, "clock test buffer allocated");
  if (!buf) return;

  for (int i = 0; i < 256; i++) buf[i] = 0xAAAA;

  lldesc_t desc;
  link_dma_desc(&desc, NULL, buf, 512);  // 256 words = 512 bytes
  desc.eof = 1;
  desc.qe.stqe_next = &desc;

  callback_count = 0;
  uint32_t t_start = micros();
  i2s_parallel_send_dma(I2S_NUM_1, &desc);
  delay(100);  // let it run for 100ms
  i2s_parallel_stop_dma(I2S_NUM_1);
  uint32_t t_end = micros();
  uint32_t elapsed = t_end - t_start;
  uint32_t loops = callback_count;

  Serial.printf("  Elapsed: %d us, Loops: %d\n", elapsed, loops);

  // At 5 MHz, 256 words per loop = 51.2 us/loop
  // In 100ms we expect ~1953 loops
  // Allow wide tolerance (500-5000) due to interrupt latency and startup
  if (loops > 0) {
    uint32_t us_per_loop = elapsed / loops;
    Serial.printf("  Measured: ~%d us/loop (expected ~51 us at 5MHz)\n", us_per_loop);
    TEST_ASSERT(us_per_loop > 20 && us_per_loop < 200, "loop timing in expected range (20-200 us)");
  } else {
    TEST_ASSERT(false, "no loops completed - DMA may not be running");
  }

  heap_caps_free(buf);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════════

void setup()
{
  Serial.begin(115200);
  delay(2000);  // wait for serial monitor

  Serial.println("\n╔══════════════════════════════════════════════════════╗");
  Serial.println("║   LCD_CAM / I2S Parallel DMA Test Suite             ║");
  Serial.println("║   ESP32-S3 Super Mini                               ║");
  Serial.println("╚══════════════════════════════════════════════════════╝");

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  Serial.println("Platform: ESP32-S3 (LCD_CAM + GDMA)");
#else
  Serial.println("Platform: ESP32 (I2S LCD mode)");
#endif

  Serial.printf("Test CLK pin: GPIO%d\n", TEST_CLK_PIN);
  Serial.printf("Test DATA pin: GPIO%d\n", TEST_DATA_PIN);
  Serial.printf("Test clock: %d Hz\n", TEST_DMA_CLOCK_HZ);
  Serial.printf("DMA_MAX: %d bytes\n", DMA_MAX);
  Serial.printf("Free DMA heap: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_DMA));

  // Run tests in order (some depend on prior state)
  test_descriptor_linking();
  test_circular_chain();
  test_buffer_allocation();
  test_driver_install();
  test_dma_transfer();
  test_dma_stop();
  test_multi_descriptor();
  test_descriptor_relinking();
  test_clock_frequency();

  // Summary
  Serial.println("\n══════════════════════════════════════════════════════");
  Serial.printf("Results: %d PASSED, %d FAILED\n", tests_passed, tests_failed);
  Serial.println("══════════════════════════════════════════════════════");

  if (tests_failed == 0) {
    Serial.println("✓ ALL TESTS PASSED — DMA layer is functional");
  } else {
    Serial.println("✗ SOME TESTS FAILED — check output above");
  }
}

void loop()
{
  delay(10000);
}
