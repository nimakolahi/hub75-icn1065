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
enum { SINGLE_PANEL_H = 32  };  // physical height of one panel (pixels)
enum { PANELS_X       = 2   };  // number of panels in a horizontal row
enum { PANELS_Y       = 4   };  // number of panels stacked vertically
// NOTE: PANELS_Y > 1 requires additional coordinate mapping work in
//       VirtualMatrixPanel::getCoords — currently only PANELS_Y = 1 is supported.

// ── Derived — do not edit ─────────────────────────────────────────────────────
enum { VIRTUAL_PANEL_W  = SINGLE_PANEL_W * PANELS_X };  // total drawable width
enum { VIRTUAL_PANEL_H  = SINGLE_PANEL_H * PANELS_Y };  // total drawable height
enum { DMA_PANEL_W      = SINGLE_PANEL_W * 2         };  // DMA width of ONE panel (column interleave ×2)
enum { DMA_PANEL_H      = SINGLE_PANEL_H / 2         };  // DMA height of ONE panel (row groups /2)


void setup() {
  Serial.begin(115200);

  hub75_cfg_t mxconfig = {
    .mx_width = DMA_PANEL_W,       // single-panel DMA width — library multiplies by count
    .mx_height = DMA_PANEL_H,      // single-panel DMA height
    .mx_count_width = PANELS_X,
    .mx_count_height = PANELS_Y,
    .gpio = {
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
    },
    .driver = ICN1065,
    .clk_freq = HZ_10M,   // EXPERIMENT 9: 2x DCLK → commits ~13 ms instead of ~26 ms
                          // (chip max 25 MHz; working card runs 12.5 MHz). Requires the
                          // widened OE pulse (ICN1065_OE_PULSE_EXTRA) to meet twROW 280ns.
                          // Fallback ladder: HZ_5M here → then ICN1065_OE_PULSE_EXTRA 0.
    .clk_phase = CLK_POZITIVE,
    .color_depth = COLORx16,
    .double_buff = DOUBLE_BUFF_ON, 
    .double_dma_buff = DOUBLE_BUFF_ON,
    .decoder_INT595 = false,
    .phys_width = SINGLE_PANEL_W,
    .phys_height = SINGLE_PANEL_H,
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
  vdisplay = new VirtualMatrixPanel(*dma_display, VIRTUAL_PANEL_W, VIRTUAL_PANEL_H, SINGLE_PANEL_W, SINGLE_PANEL_H);

  dma_display->setPanelBrightness(255);
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
// Counts up from 00:00 since boot. Only redraws when the displayed second
// actually changes, so this doesn't hammer the panel every loop() pass.
static uint32_t lastShownSec = UINT32_MAX;

uint32_t elapsedSec = (millis() - timerStartMs) / 1000;
if (elapsedSec != lastShownSec) {
  lastShownSec = elapsedSec;
  uint32_t minutes = (elapsedSec / 60) % 100;   // wraps at 99:59
  uint32_t seconds = elapsedSec % 60;

  vdisplay->setFont(&FreeSans9pt8b_IT);
  vdisplay->setTextSize(2);
  vdisplay->fillRect(0, 12, vdisplay->width(), vdisplay->height(), color565(0, 0, 0));
  vdisplay->setCursor(0, 40);
  vdisplay->setTextColor(color565(255, 255, 255));
  vdisplay->printf("%02u:%02u", minutes, seconds);
  vdisplay->flipDMABuffer();
}
}