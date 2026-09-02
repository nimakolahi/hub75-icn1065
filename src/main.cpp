#include <Arduino.h>
#include "ESP32-HUB75-MatrixPanel-DMA.h"
#include "ESP32-HUB75-VirtualMatrixPanel.h"
#include "Fonts/FreeSans9pt7b_IT.h"
#include "utf8_latin1.h"


void drawBitmapCenteredTop(const uint16_t* bitmap, uint16_t w, uint16_t h);
void scrollTextInBox(const char* text,
                     int16_t boxX, int16_t boxY, int16_t boxW, int16_t boxH,
                     const GFXfont* font, uint8_t textSize,
                     uint16_t fg, uint16_t bg,
                     float pxPerSecond, uint32_t durationMs);
MatrixPanel_DMA *dma_display = nullptr;
VirtualMatrixPanel *vdisplay = nullptr;

// Local stopwatch: counts up from 00:00 the moment the board boots.
static uint32_t timerStartMs = 0;

// ── Panel grid — change only these four values ────────────────────────────────
enum { SINGLE_PANEL_W = 64  };  // physical width of one panel (pixels)
enum { SINGLE_PANEL_H = 64  };  // physical height of one panel (pixels)
enum { PANELS_X       = 1   };  // number of panels in a horizontal row
enum { PANELS_Y       = 1   };  // number of panels stacked vertically
// NOTE: PANELS_Y > 1 requires additional coordinate mapping work in
//       VirtualMatrixPanel::getCoords — currently only PANELS_Y = 1 is supported.

// ── Derived — do not edit ─────────────────────────────────────────────────────
enum { VIRTUAL_PANEL_W  = SINGLE_PANEL_W * PANELS_X };  // total drawable width
enum { VIRTUAL_PANEL_H  = SINGLE_PANEL_H * PANELS_Y };  // total drawable height
enum { DMA_PANEL_W      = SINGLE_PANEL_W * 2         };  // DMA width of ONE panel (column interleave ×2)
enum { DMA_PANEL_H      = SINGLE_PANEL_H / 2         };  // DMA height of ONE panel (row groups /2)


// ── Custom 64x64 / 1/16-scan pixel mapping (panel geometry "Option B") ────────
// Physical layout of this panel:
//   * 1/16 scan  → 16 addressed scan lines (A0..A3 through the 595 decoder)
//   * 2 parallel halves (strings): R1/G1/B1 drives the UPPER 32-row string,
//     R2/G2/B2 drives the LOWER 32-row string  → selected by the DMA row half
//   * 2 bands per string: within a 32-row string, one addressed line lights two
//     rows 16 apart (band 0 = rows 0..15, band 1 = rows 16..31 of that string)
//     → selected by the COLUMN-INTERLEAVE bit (dma_x LSB)
//   16 scan x 2 strings x 2 bands = 64 rows.
//
// DMA buffer is 128 wide x 32 tall. Decomposition of a virtual (x,y):
//   string = y / 32            // 0 = upper (R1 zone, DMA rows 0..15)
//                              // 1 = lower (R2 zone, DMA rows 16..31)
//   row_in_string = y % 32     // 0..31 within that string
//   band = row_in_string / 16  // 0 or 1  → interleaved column (dma_x LSB)
//   scan = row_in_string % 16  // 0..15   → addressed scan line
//   dma_x = 2*x + band         // 0..127
//   dma_y = scan + string*16   // 0..31   (string picks the R1/R2 zone)
//
// The earlier symptom (only the middle 32 rows lit + every pixel doubled) was
// the BAND bit not reaching the hardware: both bands got identical data, so
// only band 0 of each string lit and every pixel appeared twice. Encoding band
// into dma_x (the column interleave) is what separates the two bands.
//
// Bypasses VirtualMatrixPanel::getCoords and writes the DMA framebuffer directly.
// Mapping derived from the four-zone test observation:
//   * colors alternated EVERY COLUMN with dma_x = 2*x+band  → the band is NOT an
//     adjacent-column interleave. It selects which 64-wide HALF of the 128-wide
//     DMA row the pixel lands in:  dma_x = x + band*64  (bands 64 columns apart,
//     not adjacent).
//   * upper physical half showed the lower-string colors (y>=32) and vice-versa
//     → the string bit that picks the DMA row zone was INVERTED. Flip it.
//   * yellow at virtual (0,0) landed at the RIGHTMOST column → x is mirrored
//     within its 64-wide band:  local = 63 - x.
static inline void drawPixel64(int16_t x, int16_t y, uint16_t color)
{
  if (x < 0 || x >= VIRTUAL_PANEL_W || y < 0 || y >= VIRTUAL_PANEL_H) return;
  // Mapping that produced CORRECT four-zone stripes (red/green/blue/white,
  // full-height, right order). The (y, y-1) doubling seen later is a HARDWARE
  // scan artifact (proved by the raw DMA walker: one DMA row lights two physical
  // rows), NOT a mapping error — it's handled separately by ICN1065_ROW_OE_CNT,
  // so this mapping only needs to place the PRIMARY row correctly.
  int16_t string_ = y / 32;             // 0 = upper input, 1 = lower input
  int16_t row_in_string = y % 32;       // 0..31
  int16_t band = row_in_string / 16;    // 0..1 → which 64-wide half of DMA row
  int16_t scan = row_in_string % 16;    // 0..15 → addressed scan line
  scan = 15 - scan;                     // scan inverted within the 16-row zone
                                        //  (marker at virtual y=0 landed at row 15)
  int16_t local = (SINGLE_PANEL_W - 1) - x;        // x mirrored within the band
  int16_t dma_x = local + band * SINGLE_PANEL_W;   // 0..127
  int16_t dma_y = scan + (1 - string_) * 16;       // 0..31, string inverted
  dma_display->writePixelDMA(dma_x, dma_y, color);
}


void setup() {
  delay(1000);  // wait for serial monitor to open
  Serial.begin(115200);
  delay(1000);  // wait for serial monitor to open
  hub75_cfg_t mxconfig = {
    .mx_width = DMA_PANEL_W,       // single-panel DMA width — library multiplies by count
    .mx_height = DMA_PANEL_H,      // single-panel DMA height
    .mx_count_width = PANELS_X,
    .mx_count_height = PANELS_Y,
    .gpio = {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
      // ESP32-S3 Super Mini pin mapping
      // "Safe" GPIOs: 1, 2, 4, 5, 6, 7, 8, 15, 16, 17, 18, 21
      // "Fine" GPIOs (JTAG): 3, 39, 40, 41
      .r1 = 4,
      .g1 = 5,
      .b1 = 6,
      .r2 = 7,
      .g2 = 15,
      .b2 = 16,
      .a = 17,
      .b = 18,
      .c = 8,
      .d = -1,
      .e = -1,
      .lat = 21,
      .oe = 1,
      .clk = 2,
#else
      // Original ESP32 pin mapping
      .r1 = 25,
      .g1 = 26,
      .b1 = 27,
      .r2 = 14,
      .g2 = 12,
      .b2 = 13,
      .a = 2,
      .b = 32,
      .c = 17,
      .d = -1,
      .e = -1,
      .lat = 4,
      .oe = 15,
      .clk = 16,
#endif
    },
    .driver = ICN1065,
#ifndef PANEL_CLK
#define PANEL_CLK HZ_10M   // override via build flag: HZ_5M / HZ_10M / HZ_13M / HZ_20M
#endif
    .clk_freq = PANEL_CLK, 
    .clk_phase = CLK_POZITIVE,
    .color_depth = COLORx16,
    .double_buff = DOUBLE_BUFF_ON, 
    .double_dma_buff = DOUBLE_BUFF_ON,
    .decoder_INT595 = true,
    .phys_width = SINGLE_PANEL_W,
    .phys_height = SINGLE_PANEL_H,
    //.scan_rate = 
  };

  dma_display = new MatrixPanel_DMA(mxconfig);
  if (!dma_display || !dma_display->begin()) {
    Serial.println("Matrix init failed");
    while (true) {
      delay(1000);
    }
  }

  // The panel has 32 physical rows but only 16 are independently addressable.
  // Each logical row k is displayed at physical rows k and k+4 simultaneously (hardware).
  vdisplay = new VirtualMatrixPanel(*dma_display, VIRTUAL_PANEL_W, VIRTUAL_PANEL_H, SINGLE_PANEL_W, SINGLE_PANEL_H, VP_NORMAL_SCAN);

  dma_display->setPanelBrightness(100);
  dma_display->fillScreenRGB888(0, 0, 0);
  dma_display->flipDMABuffer();

  timerStartMs = millis();
}

// Draw a PROGMEM RGB565 bitmap at (x, y).
void drawBitmap(int16_t x, int16_t y,
                const uint16_t* bitmap, uint16_t w, uint16_t h)
{
    for (int16_t row = 0; row < h; row++)
        for (int16_t col = 0; col < w; col++)
            vdisplay->drawPixel(x + col, y + row,
                                pgm_read_word(&bitmap[row * w + col]));
}

// Draw bitmap horizontally centred, starting at the top of the display.
void drawBitmapCenteredTop(const uint16_t* bitmap, uint16_t w, uint16_t h)
{
    int16_t x = ((int16_t)VIRTUAL_PANEL_W - (int16_t)w) / 2;
    drawBitmap(x, 0, bitmap, w, h);
}

// ── Scrolling text inside a clipped box ───────────────────────────────────────
// Scrolls `text` right-to-left inside the rectangle (boxX, boxY, boxW, boxH).
// Only pixels inside that rectangle are touched — the rest of the screen is left
// as-is — because the text is rendered into an off-screen canvas and only the box
// region is copied out (this is what clips the text to the box).
//
//   text        : string to scroll (Latin-1; run UTF-8 through utf8ToLatin1 first)
//   boxX, boxY  : top-left corner of the box on the virtual display
//   boxW, boxH  : box size in pixels
//   font        : GFX font pointer, e.g. &FreeSans9pt8b_IT (nullptr = built-in 6x8)
//   textSize    : integer scale factor (1, 2, 3 …)
//   fg, bg      : text colour and box background (RGB565, e.g. color565(255,255,255))
//   pxPerSecond : scroll speed in pixels per second (higher = faster)
//   durationMs  : how long the animation runs before the function returns
//
// The text loops as a marquee (re-enters from the right) for the whole duration.
void scrollTextInBox(const char* text,
                     int16_t boxX, int16_t boxY, int16_t boxW, int16_t boxH,
                     const GFXfont* font, uint8_t textSize,
                     uint16_t fg, uint16_t bg,
                     float pxPerSecond, uint32_t durationMs)
{
    GFXcanvas16 canvas(boxW, boxH);
    if (!canvas.getBuffer()) return;            // not enough heap for this box size

    canvas.setFont(font);
    canvas.setTextSize(textSize);
    canvas.setTextColor(fg);
    canvas.setTextWrap(false);

    // Measure the text so we know its width and how to centre it vertically.
    int16_t tx, ty; uint16_t tw, th;
    canvas.getTextBounds(text, 0, 0, &tx, &ty, &tw, &th);
    int16_t cursorY = (boxH - (int16_t)th) / 2 - ty;   // vertical centre

    const int32_t cycle = (int32_t)tw + boxW;   // travel per loop (text width + a full-box gap)

    // ── double_buff = ON version ──────────────────────────────────────────────
    // With CPU-side double buffering, flipDMABuffer() = wait for the previous
    // send, SWAP front/back framebuffers, then start sending the frame we just
    // drew (async) — rendering overlaps the DMA transfer, and we can never
    // overwrite a frame mid-send.
    // Consequences for this loop:
    //  * flips ALTERNATE buffers, so the box must be re-blitted EVERY iteration
    //    (the buffer we draw into holds the frame from TWO flips ago);
    //  * the canvas is only re-rendered when the text actually moved (cheap);
    //  * we still flip every iteration = the steady, continuous vsync stream the
    //    working receiver card uses (60 fps nonstop).
    // NOTE for callers: any static background (bitmaps etc.) must be drawn into
    // BOTH buffers before scrolling — draw it, flipDMABuffer(), draw it again —
    // otherwise it blinks at half the flip rate.
    uint32_t start = millis();
    int16_t  lastX = INT16_MIN;
    while (millis() - start < durationMs) {
        uint32_t elapsed   = millis() - start;
        int32_t  travelled = (int32_t)((float)elapsed * pxPerSecond / 1000.0f);
        int16_t  x         = boxW - (int16_t)(travelled % cycle);

        if (x != lastX) {                       // text moved → re-render the canvas
            lastX = x;
            canvas.fillScreen(bg);
            canvas.setCursor(x, cursorY);
            canvas.print(text);
        }

        // Blit the box into the CURRENT back buffer (every iteration — buffers alternate).
        for (int16_t row = 0; row < boxH; row++)
        {
          for (int16_t col = 0; col < boxW; col++)
                vdisplay->drawPixel(boxX + col, boxY + row, canvas.getPixel(col, row));

        // Swap buffers and send (waits for the previous send internally).
        if (row % 4 == 0) vdisplay->flipDMABuffer();
      }
    }
}

inline uint8_t randomByte() {
    return (uint8_t)(esp_random() & 0xFF);
}

inline char utf8CharToLatin1(const char* utf8char) {
    char buf[4];
    utf8ToLatin1(utf8char, buf, sizeof(buf));
    return buf[0];
}

void loop() {
// ── Stopwatch display ─────────────────────────────────────────────────────────
// Counts up from 00:00 since boot. The digits only get RE-RENDERED into the
// offscreen canvas when the displayed second actually changes (cheap), but
// the canvas is blitted + flipped EVERY loop() pass, continuously — same
// structure as scrollTextInBox. This isn't about buffer staleness (a 2-pass
// redraw already fixed that); it's because ICN1065_VSYNC_BEFORE_DATA's extra
// vsync send causes a brief real blank each time it fires, which is invisible
// inside a continuous commit stream (scrolling) but reads as a distinct
// flash-then-black when commits are sparse (a lone commit once a second).
// Committing continuously — even when the digits haven't changed — keeps the
// panel in the same "steady stream" regime scrolling already benefits from.
//
// NOTE: the canvas is blitted to the panel through drawPixel64 (the custom
// 64x64 / 1/16-scan mapping for this ICN1065 panel), not vdisplay->drawPixel.
static uint32_t lastShownSec = UINT32_MAX;
static GFXcanvas16 clockCanvas(VIRTUAL_PANEL_W, VIRTUAL_PANEL_H - 12);
if (!clockCanvas.getBuffer()) return;   // not enough heap for this canvas size

uint32_t elapsedSec = (millis() - timerStartMs) / 1000;
if (elapsedSec != lastShownSec) {
  lastShownSec = elapsedSec;
  uint32_t minutes = (elapsedSec / 60) % 100;   // wraps at 99:59
  uint32_t seconds = elapsedSec % 60;

  clockCanvas.setFont(&FreeSans9pt8b_IT);
  clockCanvas.setTextSize(1);
  clockCanvas.fillScreen(color565(0, 0, 0));
  clockCanvas.setCursor(0, 40 - 12);   // canvas-local Y (canvas starts at panel Y=12)
  clockCanvas.setTextColor(color565(5, 255, 5));
  clockCanvas.printf("%02u:%02u", minutes, seconds);
}

// Blit the WHOLE canvas + one atomic flip, every iteration, unconditionally.
// Uses drawPixel64 (custom 64x64 / 1/16-scan mapping) instead of the built-in
// VirtualMatrixPanel presets, which don't match this panel.
for (int16_t row = 0; row < clockCanvas.height(); row++)
    for (int16_t col = 0; col < clockCanvas.width(); col++)
        drawPixel64(col, row + 12, clockCanvas.getPixel(col, row));
dma_display->flipDMABuffer();

}