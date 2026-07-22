# ESP32 HUB75 Driver for ICN1065 SPWM Panels

A from-scratch ESP32 Arduino driver for HUB75 LED matrix panels that use the **ICN1065** (ICND1065) Scramble-PWM driver IC. It uses the ESP32's I2S peripheral in 16-bit parallel mode with DMA linked-list descriptors for zero-CPU pixel output, and inherits Adafruit GFX so all text and drawing primitives work out of the box.

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [The ICN1065 Chip — What It Is and Why It Matters](#2-the-icn1065-chip--what-it-is-and-why-it-matters)
3. [The ICN1065 Serial Protocol](#3-the-icn1065-serial-protocol)
4. [How the ESP32 Drives HUB75 — I2S in Parallel Mode](#4-how-the-esp32-drives-hub75--i2s-in-parallel-mode)
5. [The DMA Buffer Pipeline](#5-the-dma-buffer-pipeline)
6. [The Frame Buffer — Where You Draw](#6-the-frame-buffer--where-you-draw)
7. [Coordinate Mapping: Physical Panel ↔ Frame Buffer](#7-coordinate-mapping-physical-panel--frame-buffer)
8. [VirtualMatrixPanel — The Drawing Interface](#8-virtualmatrixpanel--the-drawing-interface)
9. [GPIO Wiring](#9-gpio-wiring)
10. [Configuration Reference](#10-configuration-reference)
11. [API Reference](#11-api-reference)
12. [Project File Map](#12-project-file-map)
13. [Key Bugs Found and Fixed](#13-key-bugs-found-and-fixed)
14. [How to Add a New Panel Type](#14-how-to-add-a-new-panel-type)

---

## 1. Hardware Overview

### The panel

- **Physical size:** 64 columns × 32 rows
- **Scan rate:** 1/8 scan — only 3 row-address lines (A, B, C) are needed, selecting among 8 scan addresses
- **Driver IC:** ICN1065 (ICND1065), a Scramble-PWM LED driver — not a plain shift register

### The microcontroller

- **ESP32** (DOIT DevKit V1 or equivalent)
- Uses the **I2S peripheral** (I2S_NUM_1) in 16-bit LCD/parallel mode — not for audio at all. This peripheral can clock out a 16-bit bus at a fixed frequency, driven entirely by DMA. The CPU does not bit-bang any signals.

### The HUB75 connector signals

A standard HUB75 connector carries:

| Signal | Purpose |
|--------|---------|
| R1, G1, B1 | Serial RGB data — upper half of panel |
| R2, G2, B2 | Serial RGB data — lower half of panel |
| A, B, C | Row address (3 bits → 8 scan addresses for 1/8-scan) |
| LAT | Latch: rising edge commits shift-register data to outputs |
| OE | Output Enable: active HIGH on ICN1065 (triggers row advance) |
| CLK | Shift-clock for serial data |

All 13 of these signals are driven simultaneously by one 16-bit I2S parallel clock cycle. Each 16-bit word output by DMA encodes all signals at once.

---

## 2. The ICN1065 Chip — What It Is and Why It Matters

### It is NOT a plain shift register

Most cheap HUB75 panels use simple 74HC595-style shift registers: you clock in bits, latch, and the LED is either on or off. Brightness is achieved by rapidly cycling through bit-planes (Binary Code Modulation, BCM). The ICN1065 is fundamentally different.

### Scramble-PWM (SPWM)

The ICN1065 is a **Scramble-PWM** driver. Instead of receiving a 1-bit on/off value per LED, it receives a **16-bit grayscale word per LED per refresh cycle**. The chip's internal hardware then generates a PWM signal at the hardware level — no external BCM loop required.

The bottom 12 of those 16 bits carry actual grayscale data (`gclk_bits = 12`, bits 15..12 must be zero). This gives 4096 levels of brightness per LED channel, with the ICN1065 automatically Scrambleing the PWM pulses to reduce EMI (that is the "Scramble" in Scramble-PWM).

### Physical structure

Each ICN1065 chip has **16 output channels** (24-pin SSOP package, per the Chipone datasheet). For a 64-column panel, you need 4 ICN1065 chips daisy-chained per row (4 × 16 = 64 channels per chain, matching the panel's column count). The chips are connected in two separate serial chains:
- Chain 1 carries R1G1B1 data (upper half of the panel)
- Chain 2 carries R2G2B2 data (lower half of the panel)

Data is shifted in MSB-first over CLK. After all 16 bits per output (× 64 outputs = 1024 CLK cycles per chain) are clocked in, a LATCH pulse with a specific width commits the data to the internal SPWM registers.

### Scan addressing

With 1/8 scan and 32 rows, 8 scan addresses drive 32 rows total:
- Each scan address activates a group of 4 physical rows (FOUR_SCAN mode)
- R1G1B1 and R2G2B2 split the panel vertically, each chain handling 16 rows
- OE pulses advance the internal row counter within the ICN1065

---

## 3. The ICN1065 Serial Protocol

The ICN1065 is initialized once at startup by writing to its 38 internal configuration registers. After that, every frame sends fresh SPWM grayscale data for all LEDs.

### Commands are encoded as LAT pulse widths

The ICN1065 does not have a separate command/data line. It distinguishes between data and commands solely by counting how many CLK cycles the LAT line stays high. The defined pulse lengths are:

| Constant | LAT-high CLK cycles | Meaning |
|----------|-------------------|---------|
| `ICN1065_V_SYNC` | 3 | Vertical sync — resets the internal row counter |
| `ICN1065_PRE_ACT1` | 11 | Pre-activation 1 — unlock config write |
| `ICN1065_PRE_ACT2` | 14 | Pre-activation 2 — confirm config write |
| `ICN1065_REG_LATCH` | 5 | Register data latch — commits one config word |

Between each of these commands, 8 CLK cycles of padding (`ICN1065_CLK_PAD = 8`) are inserted with LAT low.

### Frame prefix: VSYNC + register write

At the start of every frame, the driver sends a **prefix** buffer containing:

```
VSYNC (3 LAT clocks)
  → 8 CLK pad
PRE_ACT1 (11 LAT clocks)
  → 8 CLK pad
PRE_ACT2 (14 LAT clocks)
  → 8 CLK pad
Wrapper 0 (0x00AA data, 5 LAT clocks)
Wrapper 1 (0x01AA data, 5 LAT clocks)
Config register N data (5 LAT clocks)
Wrapper 2 (0x0055 data, 5 LAT clocks)
Wrapper 3 (0x0155 data, 5 LAT clocks)
```

The VSYNC resets the ICN1065 row counter to zero. The PRE_ACT sequence unlocks register writes. Then the five-block sequence (four fixed wrapper words surrounding the register data) writes one of the 38 config registers. The wrappers are magic numbers required by the ICN1065 protocol to distinguish register writes from pixel data.

The 38 config registers are written one per frame, cycling through all of them over 38 frames. This means the panel's full configuration is refreshed approximately every 38 display frames.

### Config register 2: scan line count

Register index 2 (constant `ICN1065_CFG_SCAN = 2`) sets the number of active scan lines:

```
register_value = 0x0200 | (rows_per_frame - 1)
```

With `rows_per_frame = 8`, this writes `0x0207`.

### Pixel data

After the prefix, pixel data for one row is clocked in as 128 × 16-bit SPWM words per chain. Each word encodes one bit from each channel's 16-bit SPWM value, bit-serial, MSB first. The bit-serial packing loop in `prepareDmaRows()` works like this:

```cpp
for j = 15 downto 0:          // 16 SPWM bits per pixel
    if pixel.r MSB set: mask |= BIT_R1
    if pixel.g MSB set: mask |= BIT_G1
    if pixel.b MSB set: mask |= BIT_B1
    if pixel2.r MSB set: mask |= BIT_R2
    ...
    dma_word[offset] = mask    // all 6 color bits in one 16-bit word
    shift all channels left 1
```

The brightness table converts 8-bit color components to 12-bit SPWM values (stored sign-extended in int16_t so MSB-test works as `< 0`).

### Frame suffix: row scanning

After all pixel rows are output, the driver sends a **suffix** buffer that continuously cycles through all 8 row addresses with OE pulses. The OE pulse advances the ICN1065's internal scan counter. Each row address slot is exactly 128 CLK cycles wide (`ICN1065_ROW_OE_LEN = 128`):

```
ICN1065_ROW_OE_CNT = 1    → 1 OE pulse (2 CLK cycles: rising + falling edge)
ICN1065_ROW_OE_ADD_LEN = 126  → 126 CLK cycles of hold
Total = 1×2 + 126 = 128 CLK cycles per row slot
```

This is the most important tuning parameter for this panel. See [Section 13](#13-key-bugs-found-and-fixed) for why.

---

## 4. How the ESP32 Drives HUB75 — I2S in Parallel Mode

### Why I2S?

The ESP32 has no dedicated LCD or parallel-bus peripheral with DMA. However, the I2S peripheral (designed for audio) has a mode where it outputs a 16-bit parallel bus clocked at a fixed frequency — exactly what we need to drive a HUB75 panel.

### I2S configuration

- **Peripheral:** I2S_NUM_1
- **Mode:** LCD/parallel, 16-bit
- **Clock:** 5 MHz (`HZ_5M`) — each CLK cycle is 200 ns
- **Clock phase:** `CLK_POZITIVE` — data is valid on the rising edge
- **DMA:** Linked-list descriptors, automatically reloaded (circular buffer)

The I2S peripheral in this mode outputs one 16-bit word per CLK cycle. Each word's bits map to the physical GPIO pins:

```
Bit  0  → R1  (GPIO 25)
Bit  1  → G1  (GPIO 26)
Bit  2  → B1  (GPIO 27)
Bit  3  → R2  (GPIO 14)
Bit  4  → G2  (GPIO 12)
Bit  5  → B2  (GPIO 13)
Bit  6  → LAT (GPIO 4)
Bit  7  → OE  (GPIO 15)
Bit  8  → A   (GPIO 2)
Bit  9  → B   (GPIO 32)
Bit 10  → C   (GPIO 17)
Bit 11  → D   (not connected, -1)
Bit 12  → E   (not connected, -1)
Bits 13-15 → unused
```

### The byte-swap quirk

The ESP32 I2S DMA engine byte-swaps every 16-bit word before outputting it. This means if you want to output the word `0xAABB`, the DMA buffer must contain `0xBBAA`. The library compensates for this everywhere by XOR-ing buffer offsets with 1 (`buffer[offset ^ 1]`).

---

## 5. The DMA Buffer Pipeline

There is no single frame buffer that DMA reads directly. Instead, there are three distinct DMA regions that the library chains together:

```
┌─────────────────────────────────────────────────────────────────┐
│                         DMA CHAIN LOOP                          │
│                                                                 │
│   [SUFFIX: row scanning]  ←──────────────────────────────────┐  │
│         │                                                    │  │
│         │ (on EOF interrupt: CPU prepares one row)           │  │
│         ↓                                                    │  │
│   [ROW DATA: one row of pixel data + LAT]                    │  │
│         │                                                    │  │
│         └──→ [PREFIX: VSYNC + config register write] ──→ ───┘  │
│              (only sent after all 8 rows are done)              │
└─────────────────────────────────────────────────────────────────┘
```

### 1. Suffix buffer

The suffix is the steady-state DMA loop. It continuously cycles through all 8 row addresses, asserting OE for each one. The ICN1065 advances its row counter on each OE pulse. The suffix runs in a tight circular DMA loop (`dmadesc_suffix[desc_suffix][last].qe.stqe_next = &dmadesc_suffix[desc_suffix][0]`).

There are two sets of suffix descriptors (`ICN1065_DSUFFIX_CNT = 2`) so one can be in use while the other is being swapped during a frame flip.

### 2. Row data buffer

There is one row data buffer (`row_data_cnt = 1`). When the DMA ISR fires (end of a suffix descriptor), the CPU fills this buffer with one row's pixel data via `prepareDmaRows()`, then splices it into the DMA chain between two suffix descriptors. The panel sees: `...suffix rows...` → `pixel row N` → `...suffix rows...`.

The row data buffer contains:
- 128 CLK cycles of R1G1B1/R2G2B2 bit-serial pixel data
- A LAT pulse at the end (with correct width to mean "data latch")
- OE and address bits throughout

### 3. Prefix buffer

After all 8 pixel rows have been transmitted for a frame, the driver splices in the prefix buffer, which contains the VSYNC + config register write sequence. This resets the ICN1065 row counter and writes one config register.

### Interrupt-driven row pipelining

The callback `sendCallback()` is called from the I2S DMA EOF interrupt. It:

1. Checks which row was just sent
2. Prepares the next row in the row data buffer (reads from frame buffer, converts color, packs SPWM bits)
3. Splices the row buffer into the DMA chain at the correct point
4. After row 7: splices the prefix buffer instead

This means pixel data is prepared at the interrupt level, one row at a time, in sync with the DMA output. The CPU load for this is modest because `prepareDmaRows()` only processes one row of 128 pixels per interrupt.

---

## 6. The Frame Buffer — Where You Draw

### Structure

The **frame buffer** is separate from the DMA output buffers. It lives in regular (non-DMA) heap RAM and stores raw **RGB565 color values** — not SPWM data. Conversion to SPWM happens in `prepareDmaRows()` at the moment of output.

```c
typedef uint32_t vbuffer_t;

typedef struct {
    vbuffer_t* frameBits[2];  // two frame buffers (only one used when double_buff=OFF)
    size_t len;               // size in vbuffer_t words of one frame
    size_t row_len;           // width of one row in vbuffer_t words
    size_t subframe_len;      // size of one subframe (half the full frame)
    uint8_t subframe_cnt;
} frame_buffer_t;
```

### Dimensions

With the current configuration (`mx_width=128, mx_height=16`):

```
row_len    = (128 × 2 bytes + 3) / 4 = 64   uint32_t words per row
           = 128  uint16_t slots per row
subframe_len = 64 × 16 = 1024  uint32_t words
len          = 1024  uint32_t words (one complete frame)
```

Although `vbuffer_t` is `uint32_t`, the frame buffer is always accessed as `uint16_t*`. Each uint16_t slot holds one RGB565 color value.

### Two sub-frames within the frame buffer

The 16 rows of the frame buffer are split into two halves:

```
Rows  0..7  → pixel1 data  (read as R1G1B1 → drives physical rows  0..7  and 16..23)
Rows  8..15 → pixel2 data  (read as R2G2B2 → drives physical rows  8..15 and 24..31)
```

In `prepareDmaRows()` for scan address `row_offset` (0..7):
```cpp
pixel1 ← frame_buffer[row_offset][offset_x]         // rows 0..7
pixel2 ← frame_buffer[row_offset + 8][offset_x]     // rows 8..15
```

Pixel1 → R1G1B1 serial chain → upper-half LED rows
Pixel2 → R2G2B2 serial chain → lower-half LED rows

### Column interleaving (bank)

Within each frame buffer row, the 128 uint16_t slots do not map 1:1 to physical columns. Physical column `x` occupies **two** adjacent slots, one per bank:

```
Slot  2x+0  (even, bank=0):  color for the "lower" row group at column x
Slot  2x+1  (odd,  bank=1):  color for the "upper" row group at column x
```

This interleaving exists because `prepareDmaRows()` reads every slot sequentially and alternately encodes it into the R1/R2 SPWM streams. The bank determines which physical row a given color ends up on — see Section 7 for the exact mapping.

---

## 7. Coordinate Mapping: Physical Panel ↔ Frame Buffer

This is the most complex part of the driver, and where most bugs hide.

### The goal

You call `drawPixel(x, y, color)` with physical panel coordinates, where (0,0) is the top-left LED. The driver must store `color` at the correct `(dma_x, dma_y)` position in the frame buffer so that `prepareDmaRows()` picks it up and routes it to the right ICN1065 output.

### Physical row grouping

With 1/8 scan, 32 rows, and the ICN1065's two serial chains:

| Physical rows | Chain | Frame buffer rows | Bank |
|--------------|-------|------------------|------|
| 0 .. 7 | R1G1B1 | 0 .. 7 | 1 (odd DMA cols) |
| 8 .. 15 | R1G1B1 (lower) | 0 .. 7 | 0 (even DMA cols) |
| 16 .. 23 | R2G2B2 | 8 .. 15 | 1 (odd DMA cols) |
| 24 .. 31 | R2G2B2 (lower) | 8 .. 15 | 0 (even DMA cols) |

### The mapping formula

```
bank  = ((y & 8) == 0) ? 1 : 0
dma_x = 2 * x + bank
dma_y = (y & 7) | (((y >> 4) & 1) * 8)
```

Walking through the four row groups:

**y = 0..7** (`y & 8 = 0` → bank=1, `y >> 4 = 0`):
```
dma_x = 2x + 1   (odd slot)
dma_y = y         (rows 0..7 of frame buffer)
```
Stored at frame_buffer[y][2x+1]. `prepareDmaRows(row_offset=y)` reads slot 2x+1 → pixel1 → R1G1B1 → physical row y.

**y = 8..15** (`y & 8 = 8` → bank=0, `y >> 4 = 0`):
```
dma_x = 2x + 0   (even slot)
dma_y = y & 7     (maps y=8 → dma_y=0, y=9 → dma_y=1, etc.)
```
Stored at frame_buffer[y-8][2x]. `prepareDmaRows(row_offset=y-8)` reads slot 2x → pixel1 (same row_offset as y=0..7 but different column!) → R1G1B1 lower → physical row y.

**y = 16..23** (`y & 8 = 0` → bank=1, `y >> 4 = 1`):
```
dma_x = 2x + 1   (odd slot)
dma_y = (y & 7) | 8   (maps y=16 → dma_y=8, y=17 → dma_y=9, etc.)
```
Stored at frame_buffer[y-8][2x+1]. `prepareDmaRows(row_offset=y-16)` reads from dma_y=y-8, which is in the second half (rows 8..15) → pixel2 → R2G2B2 → physical row y.

**y = 24..31** (`y & 8 = 8` → bank=0, `y >> 4 = 1`):
```
dma_x = 2x + 0
dma_y = (y & 7) | 8   (maps y=24 → dma_y=8, etc.)
```

### Why this panel needs a custom mapping

The library's built-in `fillRectBufferVirtual()` also computes a (dma_x, dma_y) mapping, but it uses a **different formula** designed for panels with an interleaved scan order:

```cpp
// Library's formula (WRONG for this panel):
int16_t ry_in_band = ry & 7;
dma_y = (ry >> 4) * 8 + ((ry_in_band & 3) << 1) + ((ry_in_band >> 2) & 1);
```

The `((ry_in_band & 3) << 1) + ((ry_in_band >> 2) & 1)` is a 3-bit left-rotation. It assumes scan addresses are ordered 0,4,1,5,2,6,3,7 in the physical panel. This panel uses **linear** scan order (scan address k → physical row k), so the correct `dma_y` is simply `y & 7` (with the high bit from `y >> 4`).

The library's formula gives the wrong answer for y=1..6 (off by a factor of 2 for rows 1-3, then correcting at rows 4-7). This `VirtualMatrixPanel` overrides the mapping with the correct formula.

### Verifying the mapping

A complete truth table for y=0..15 (the range used with `height=16`):

| y | bank | dma_y | Physical row |
|---|------|-------|-------------|
| 0 | 1 | 0 | 0 |
| 1 | 1 | 1 | 1 |
| 2 | 1 | 2 | 2 |
| 3 | 1 | 3 | 3 |
| 4 | 1 | 4 | 4 |
| 5 | 1 | 5 | 5 |
| 6 | 1 | 6 | 6 |
| 7 | 1 | 7 | 7 |
| 8 | 0 | 0 | 8 |
| 9 | 0 | 1 | 9 |
| 10 | 0 | 2 | 10 |
| 11 | 0 | 3 | 11 |
| 12 | 0 | 4 | 12 |
| 13 | 0 | 5 | 13 |
| 14 | 0 | 6 | 14 |
| 15 | 0 | 7 | 15 |

---

## 8. VirtualMatrixPanel — The Drawing Interface

`VirtualMatrixPanel` is a thin wrapper over `MatrixPanel_DMA` that:

1. Accepts physical (x, y) coordinates from user code
2. Applies the coordinate mapping formula (Section 7)
3. Calls `MatrixPanel_DMA::writePixelDMA(dma_x, dma_y, color)` to store the color in the frame buffer
4. Inherits `Adafruit_GFX` so all text, shapes, and bitmap drawing work via the single `drawPixel` override

### Construction

```cpp
// Full 32-row access (ICN1065_ROW_OE_CNT must be 1):
VirtualMatrixPanel vdisplay(*dma_display, 64, 32);

// 16-row access only (e.g. when testing or when OE_CNT=2):
VirtualMatrixPanel vdisplay(*dma_display, 64, 16);
```

The third parameter is the virtual panel height. It must not exceed the physical panel height. With `height=32` you can address all 32 rows. With `height=16` only rows 0..15 are accessible.

### drawPixel flow

```
vdisplay.drawPixel(x, y, color565)
  → VirtualMatrixPanel::drawPixel()
    → getCoords(x, y)          // applies the mapping formula
    → display->writePixelDMA(dma_x, dma_y, color)
      → frame_buffer[dma_y][dma_x] = color   // stored as raw RGB565
```

The frame buffer stores raw RGB565. The SPWM conversion happens later in `prepareDmaRows()` when the DMA ISR runs.

### flipDMABuffer

```cpp
vdisplay->flipDMABuffer();
```

Sends the current frame buffer contents to the panel. With `double_buff=OFF` (current config), this calls `sendFrame(true)` — it **blocks** until all rows have been transmitted to the ICN1065. This ensures the display is updated completely before the function returns.

### All Adafruit GFX primitives work

Because `VirtualMatrixPanel` inherits `Adafruit_GFX` and overrides `drawPixel`, all of these work:

```cpp
vdisplay->drawFastHLine(0, 0, 64, color565(255, 0, 0));    // horizontal line
vdisplay->drawFastVLine(0, 0, 32, color565(0, 255, 0));    // vertical line
vdisplay->fillRect(10, 5, 20, 10, color565(0, 0, 255));    // filled rectangle
vdisplay->drawCircle(32, 16, 10, color565(255, 255, 0));   // circle
vdisplay->setCursor(2, 2);
vdisplay->setTextColor(color565(255, 255, 255));
vdisplay->setTextSize(1);
vdisplay->print("Hello");                                  // text
vdisplay->drawChar(2, 2, 'A', color565(255,255,0), 0, 3); // large character
```

---

## 9. GPIO Wiring

```
ESP32 GPIO  │  HUB75 Signal  │  Function
────────────┼────────────────┼─────────────────────────────
    25      │      R1        │  Red,   upper half
    26      │      G1        │  Green, upper half
    27      │      B1        │  Blue,  upper half
    14      │      R2        │  Red,   lower half
    12      │      G2        │  Green, lower half
    13      │      B2        │  Blue,  lower half
     2      │       A        │  Row address bit 0
    32      │       B        │  Row address bit 1
    17      │       C        │  Row address bit 2
    —       │       D        │  Not connected (1/8 scan only needs A,B,C)
    —       │       E        │  Not connected
     4      │      LAT       │  Latch
    15      │      OE        │  Output Enable (active HIGH on ICN1065)
    16      │      CLK       │  Shift clock
```

> **Important:** The ICN1065's OE is **active HIGH**, opposite to the active-LOW OE on traditional HUB75 panels with shift-register drivers. Do not mix up OE polarity if using a different driver IC.

### PCB color channel swap

On this specific panel PCB, the HUB75 connector signals are routed to non-standard LED positions. The physical wiring is:

```
HUB75 R1 signal → Blue LEDs on PCB
HUB75 G1 signal → Red LEDs on PCB
HUB75 B1 signal → Green LEDs on PCB
```

The driver compensates for this in `prepareDmaRows()` by swapping the channel assignments in the bit-packing loop:

```cpp
if (pixel1.r < 0) mask |= BIT_G1;   // software R → hardware G1 → Red LED
if (pixel1.g < 0) mask |= BIT_B1;   // software G → hardware B1 → Green LED
if (pixel1.b < 0) mask |= BIT_R1;   // software B → hardware R1 → Blue LED
```

If your panel does not have this swap, change these assignments back to `r→R1, g→G1, b→B1`.

---

## 10. Configuration Reference

### `hub75_cfg_t` fields (set in `main.cpp`)

```cpp
hub75_cfg_t mxconfig = {
    .mx_width       = 128,      // DMA width = physical width × 2 (column interleave)
    .mx_height      = 16,       // DMA height = physical height / 2 (row groups)
    .mx_count_width = 1,        // number of panels chained horizontally
    .mx_count_height = 1,       // number of panels chained vertically
    .gpio = { ... },            // see GPIO section above
    .driver         = ICN1065,  // selects ICN1065 protocol throughout
    .clk_freq       = HZ_5M,    // 5 MHz I2S clock
    .clk_phase      = CLK_POZITIVE, // data valid on rising edge
    .color_depth    = COLORx16, // 16-bit SPWM mode (full ICN1065 capability)
    .double_buff    = DOUBLE_BUFF_OFF,  // single frame buffer
    .double_dma_buff = DOUBLE_BUFF_ON, // two DMA suffix descriptor sets
    .decoder_INT595 = false,    // no external 74HC595 row decoder
    .phys_width     = 64,       // actual physical panel width in pixels
    .phys_height    = 32,       // actual physical panel height in pixels
};
```

### `ICN1065_ROW_OE_CNT` — the critical timing parameter

Located in `ESP32-HUB75-MatrixPanel-DMA-leddrivers.h`:

```cpp
enum{ICN1065_ROW_OE_CNT = 1};      // number of OE pulses per row slot
enum{ICN1065_ROW_OE_ADD_LEN = 126};// hold time after OE pulses
// Total row slot = 1×2 + 126 = 128 CLK cycles
```

**This value controls whether rows are paired or independent:**

| OE_CNT | Behavior |
|--------|---------|
| 1 | Each scan address lights exactly one row group — 32 independent rows |
| 2 | Two OE pulses per address — activates two row groups simultaneously, pairing rows k and k+4 |

The panel was originally set to `OE_CNT=2`, which caused `drawPixel(x, 0)` to light both row 0 and row 4. Setting `OE_CNT=1` gives full independent row control.

**The total row slot length must stay at 128 CLK cycles.** If you change `OE_CNT`, adjust `OE_ADD_LEN` accordingly: `OE_ADD_LEN = 128 - OE_CNT * 2`.

### Color depth: `COLORx16`

This enables 16-bit SPWM mode. In this mode:
- The frame buffer stores raw RGB565 values
- `brightness_table[]` converts 8-bit color components to 16-bit SPWM grayscale values
- The ICN1065 receives the full 12-bit grayscale range
- The `USE_COLORx16` preprocessor flag must be defined (it is, in `ESP32-HUB75-MatrixPanel-DMA-config.h`)

### Brightness

```cpp
dma_display->setPanelBrightness(10);   // 0..255, affects all colors uniformly
```

Internally, this scales the `brightness_table[]` used in SPWM conversion. Value 255 = full brightness. The panel is very bright; values of 5..30 are typical for indoor use.

---

## 11. API Reference

### MatrixPanel_DMA (low-level)

```cpp
// Initialization
MatrixPanel_DMA *display = new MatrixPanel_DMA(config);
display->begin();

// Drawing (coordinates are in DMA space — use VirtualMatrixPanel instead)
display->drawPixel(x, y, color565);
display->drawPixelRGB888(x, y, r, g, b);
display->fillScreen(color565);
display->fillScreenRGB888(r, g, b);
display->clearScreen();

// Brightness
display->setPanelBrightness(0..255);
display->setBrightness8(0..255);         // alias

// Frame output
display->flipDMABuffer();                // send frame, block until done
display->sendFrame(bool waitSend, bool autoVsync);  // lower-level
display->waitDmaReady();                 // spin until DMA is ready

// Misc
display->invertDisplay(bool negative);
display->setMirrorX(bool);
display->setMirrorY(bool);
display->stopDMAoutput();                // blanks panel permanently until reboot
```

### VirtualMatrixPanel (user-facing)

```cpp
VirtualMatrixPanel vdisplay(*display, width, height);

// Core pixel operations
vdisplay.drawPixel(x, y, color565);
vdisplay.drawPixelRGB888(x, y, r, g, b);
vdisplay.fillScreen(color565);
vdisplay.clearScreen();

// Frame output
vdisplay.flipDMABuffer();               // always call this after drawing

// Adafruit GFX (all of these work automatically)
vdisplay.drawFastHLine(x, y, w, color);
vdisplay.drawFastVLine(x, y, h, color);
vdisplay.fillRect(x, y, w, h, color);
vdisplay.drawRect(x, y, w, h, color);
vdisplay.drawCircle(x, y, r, color);
vdisplay.fillCircle(x, y, r, color);
vdisplay.drawLine(x0, y0, x1, y1, color);
vdisplay.drawTriangle(...);
vdisplay.setCursor(x, y);
vdisplay.setTextColor(color);
vdisplay.setTextSize(n);
vdisplay.print("text");
vdisplay.drawChar(x, y, c, fg, bg, size);

// Utility
vdisplay.setPanelBrightness(0..255);
vdisplay.color565(r, g, b);             // pack RGB888 → RGB565
vdisplay.width();                       // virtual width
vdisplay.height();                      // virtual height
```

### Color helpers (global)

```cpp
uint16_t color565(uint8_t r, uint8_t g, uint8_t b);  // pack to RGB565
void color565to888(uint16_t color, uint8_t& r, uint8_t& g, uint8_t& b); // unpack
```

---

## 12. Project File Map

```
HUB75_1065L1.0/
├── src/
│   └── main.cpp                         User application
│
├── lib/
│   ├── ESP32-HUB75-MatrixPanel-DMA-ICN2053/    Main driver library
│   │   ├── ESP32-HUB75-MatrixPanel-DMA.h        Public API header
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-icn2053.h  Full class definition, types
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-icn2053.cpp DMA engine, buffer init,
│   │   │                                           prepareDmaRows, sendCallback
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-config.h  Compile-time defaults, bit defs
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-leddrivers.h  ICN1065/ICN2053 constants
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-leddrivers.cpp ICN1065 register tables,
│   │   │                                              prefix/suffix buffer fill,
│   │   │                                              OE/address pattern generator
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-draw.cpp  High-level draw calls,
│   │   │                                          writePixelDMA
│   │   ├── ESP32-HUB75-MatrixPanel-DMA-lowdraw.cpp fillRectBufferVirtual,
│   │   │                                            fillRectFrameBuffer,
│   │   │                                            setPanelBrightness
│   │   ├── ESP32-HUB75-VirtualMatrixPanel.h      Coordinate remapping +
│   │   │                                          Adafruit GFX interface
│   │   ├── esp32_i2s_parallel_v2.h/.cpp          I2S peripheral setup and DMA
│   │   ├── color_convert.h / color-convert.cpp    RGB565 ↔ RGB888 helpers
│   │   └── ESP32-HUB75-MatrixPanel-DMA-types.h   hub75_cfg_t and related types
│   │
│   └── ICND1065_SPWM/                  Reference implementation (NOT compiled)
│       ├── DMD_SPWM_DRIVER.h           STM32-based ICN1065 driver — protocol ref
│       ├── DMD_RGB.h
│       └── DMD_RGB.cpp
│
└── platformio.ini                       Build configuration
```

### Key files to edit for common tasks

| Task | File |
|------|------|
| Change GPIO pins | `src/main.cpp` (`hub75_cfg_t.gpio`) |
| Change OE pulse count | `lib/ESP32-HUB75-MatrixPanel-DMA-ICN2053/ESP32-HUB75-MatrixPanel-DMA-leddrivers.h` (`ICN1065_ROW_OE_CNT`) |
| Fix coordinate mapping | `lib/ESP32-HUB75-MatrixPanel-DMA-ICN2053/ESP32-HUB75-VirtualMatrixPanel.h` (`getCoords`) |
| Fix color channel order | `lib/ESP32-HUB75-MatrixPanel-DMA-ICN2053/ESP32-HUB75-MatrixPanel-DMA-icn2053.cpp` (`prepareDmaRows`) |
| Change clock speed | `src/main.cpp` (`hub75_cfg_t.clk_freq`) |
| Change ICN1065 register values | `lib/ESP32-HUB75-MatrixPanel-DMA-ICN2053/ESP32-HUB75-MatrixPanel-DMA-leddrivers.cpp` (`ICN1065_REG_VALUE[]`) |

---

## 13. Key Bugs Found and Fixed

### Bug 1: Row pairing — `ICN1065_ROW_OE_CNT = 2`

**Symptom:** `drawPixel(x, 0, white)` lit physical rows 0 AND 4 simultaneously. Every pixel lit two rows 4 apart.

**Root cause:** The OE timing parameter was set to `ICN1065_ROW_OE_CNT = 2`, generating two OE pulses per scan address slot. Each OE pulse advances the ICN1065's internal scan counter by one step. With two pulses, two consecutive row groups are activated per scan address, pairing them.

**Fix:** Set `ICN1065_ROW_OE_CNT = 1`. One OE pulse per address slot → one row group activated → 32 independent rows.

```cpp
// In ESP32-HUB75-MatrixPanel-DMA-leddrivers.h:
enum{ICN1065_ROW_OE_CNT = 1};
enum{ICN1065_ROW_OE_ADD_LEN = 126};   // keep total = 128
```

---

### Bug 2: Coordinate mapping — needed the linear-scan variant, not the interleaved one

**Symptom:** After fixing Bug 1, `drawPixel(0,0)` correctly lit row 0, `drawPixel(0,15)` correctly lit row 15, but `drawPixel(0,1)` lit row 2 and `drawPixel(0,2)` lit row 4.

**Root cause:** The default `VP_FOUR_SCAN_32PX_HIGH` formula in `VirtualMatrixPanel::getCoords()` (matching the main library's `fillRectBufferVirtual()`) encodes a 3-bit rotation of `ry_in_band`. This is the correct formula for panels wired with **interleaved** scan addressing (addresses ordered 0,4,1,5,2,6,3,7) — it is not itself wrong, it's just a different variant than this panel needs:

```cpp
// Interleaved-scan variant — correct for panels addressed 0,4,1,5,2,6,3,7 (not this one):
dma_y = (ry >> 4) * 8 + ((ry_in_band & 3) << 1) + ((ry_in_band >> 2) & 1);
```

This panel uses **linear** scan order (address k → row k) instead, so it needs a different mapping variant.

**Fix:** Use the linear-scan mapping:

```cpp
// Correct — linear scan order:
int16_t bank = ((out_y & 8) == 0) ? 1 : 0;
out_x = 2 * out_x + bank;
out_y = (int16_t)(out_y & 7) | (int16_t)(((out_y >> 4) & 1) * 8);
```

---

### Bug 3: Color channel mismatch

**Symptom:** Setting color `(255, 0, 0)` (red) displayed as a different color.

**Root cause:** The PCB routes HUB75 signals to LED colors in a non-standard order. The R1 GPIO pin on the connector drives the blue LEDs on this PCB, not the red LEDs.

**Fix:** Swap the channel mapping in `prepareDmaRows()`:

```cpp
// Physical wiring: R1→Blue, G1→Red, B1→Green on this PCB
if (pixel1.r < 0) mask |= BIT_G1;   // software red → G1 pin → Red LED
if (pixel1.g < 0) mask |= BIT_B1;   // software green → B1 pin → Green LED
if (pixel1.b < 0) mask |= BIT_R1;   // software blue → R1 pin → Blue LED
```

---

### Bug 4: `flipDMABuffer()` showing wrong frame

**Symptom:** In a loop setting different colors, the displayed color was always one frame behind — each color appeared one iteration late.

**Root cause:** `flipDMABuffer()` called `sendFrame()` (non-blocking). The DMA ISR continued reading from the frame buffer asynchronously. By the time the next `drawPixel()` call ran, the ISR had not finished reading the previous frame, so the new pixel data overwrite caused visual glitches.

**Fix:** Call `sendFrame(true)` (blocking) inside `flipDMABuffer()`:

```cpp
void MatrixPanel_DMA::flipDMABuffer() {
    sendFrame(true);   // waitSend=true: block until all rows transmitted
}
```

---

## 14. How to Add a New Panel Type

If you have a different HUB75 panel with ICN1065 but different physical row ordering, here is how to figure out the correct mapping.

### Step 1: Confirm OE_CNT

Set `ICN1065_ROW_OE_CNT = 1`. Call `drawPixel(0, 0, white)`. If **exactly one row** lights up, OE_CNT=1 is correct. If **two rows** light up simultaneously, experiment with `OE_CNT` to find the right value.

### Step 2: Map out the scan order

With `OE_CNT=1`, call `drawPixel(0, y, white)` for y=0..15 one at a time. Record which physical row on the panel each y value lights. You are looking for the function `f(y) = physical_row`.

### Step 3: Determine bank and dma_y

For each y, note:
- `bank = 1` if it uses odd DMA columns (rows 0..7 and 16..23 of the physical panel)
- `bank = 0` if it uses even DMA columns (rows 8..15 and 24..31)

From the pattern, derive:
- The bank formula: which bit of y (bit 3 for this panel, i.e., `y & 8`) determines bank
- The dma_y formula: how y maps to the frame buffer row index

### Step 4: Update `VirtualMatrixPanel::getCoords()`

Edit the `VP_FOUR_SCAN_32PX_HIGH` case in `ESP32-HUB75-VirtualMatrixPanel.h` with your derived formulas.

### Step 5: Check color channels

Draw a red-only pixel. If the wrong LED color lights up, trace the PCB and adjust the `BIT_R1/G1/B1` assignments in the `prepareDmaRows()` bit-packing loop in `ESP32-HUB75-MatrixPanel-DMA-icn2053.cpp`.

---

## Build and Flash

```bash
# Install PlatformIO (if not installed)
pip install platformio

# Build
pio run

# Flash
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

The project targets `esp32doit-devkit-v1`. To use a different ESP32 board, change the `board` field in `platformio.ini`.

---

## Quick Start Example

```cpp
#include "ESP32-HUB75-MatrixPanel-DMA.h"
#include "ESP32-HUB75-VirtualMatrixPanel.h"

MatrixPanel_DMA *display = nullptr;
VirtualMatrixPanel *panel = nullptr;

void setup() {
    hub75_cfg_t cfg = {
        .mx_width  = 128, .mx_height  = 16,
        .mx_count_width = 1, .mx_count_height = 1,
        .gpio = { .r1=25,.g1=26,.b1=27,.r2=14,.g2=12,.b2=13,
                  .a=2,.b=32,.c=17,.d=-1,.e=-1,.lat=4,.oe=15,.clk=16 },
        .driver = ICN1065,
        .clk_freq = HZ_5M, .clk_phase = CLK_POZITIVE,
        .color_depth = COLORx16,
        .double_buff = DOUBLE_BUFF_OFF, .double_dma_buff = DOUBLE_BUFF_ON,
        .decoder_INT595 = false,
        .phys_width = 64, .phys_height = 32,
    };

    display = new MatrixPanel_DMA(cfg);
    display->begin();
    display->setPanelBrightness(10);

    panel = new VirtualMatrixPanel(*display, 64, 32);  // full 32 rows
    panel->clearScreen();
    panel->flipDMABuffer();
}

void loop() {
    panel->clearScreen();

    // Red horizontal line across the top
    panel->drawFastHLine(0, 0, 64, color565(255, 0, 0));

    // Yellow 'A' at scale 3
    panel->drawChar(2, 4, 'A', color565(255, 255, 0), color565(0,0,0), 3);

    // Blue border
    panel->drawRect(0, 0, 64, 32, color565(0, 0, 255));

    panel->flipDMABuffer();
    delay(1000);
}
```
