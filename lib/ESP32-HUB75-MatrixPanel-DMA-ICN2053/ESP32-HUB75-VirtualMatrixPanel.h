#ifndef ESP32_HUB75_VIRTUAL_MATRIX_PANEL_H
#define ESP32_HUB75_VIRTUAL_MATRIX_PANEL_H

/*
 * VirtualMatrixPanel for ESP32-HUB75-MatrixPanel-DMA (ICN2053/ICN1065 variant).
 *
 * Inherits Adafruit_GFX so all text/drawing primitives work out of the box:
 *   vdisplay.setCursor(0, 0);
 *   vdisplay.setTextColor(color565(255, 255, 0));
 *   vdisplay.print("Hi");
 *   vdisplay.flipDMABuffer();
 *
 * For a 64x32 1/8-scan ICN1065 panel with ICN1065_ROW_OE_CNT=1, all 32 rows
 * are independently addressable:
 *   VirtualMatrixPanel vdisplay(*dma_display, 64, 32);
 * With ICN1065_ROW_OE_CNT=2 the driver pairs rows k and k+4 per OE pulse.
 */

#include "ESP32-HUB75-MatrixPanel-DMA.h"

#ifndef NO_GFX
  #include "Adafruit_GFX.h"
#endif

enum VirtualPanelScanRate {
  VP_NORMAL_SCAN,
  VP_FOUR_SCAN_32PX_HIGH,
  VP_FOUR_SCAN_16PX_HIGH,
};

#ifndef NO_GFX
class VirtualMatrixPanel : public Adafruit_GFX {
#else
class VirtualMatrixPanel {
#endif

public:
#ifndef NO_GFX
  VirtualMatrixPanel(MatrixPanel_DMA &disp,
                     uint16_t panel_virtual_width,
                     uint16_t panel_virtual_height,
                     uint16_t single_panel_width,
                     uint16_t single_panel_height,
                     VirtualPanelScanRate scan_rate = VP_FOUR_SCAN_32PX_HIGH)
    : Adafruit_GFX(panel_virtual_width, panel_virtual_height),
      display(&disp),
      scan_rate(scan_rate),
      panel_pixel_base(panel_virtual_width),
      spw(single_panel_width),
      sph(single_panel_height)
  {}
#else
  VirtualMatrixPanel(MatrixPanel_DMA &disp,
                     uint16_t panel_virtual_width,
                     uint16_t panel_virtual_height,
                     uint16_t single_panel_width,
                     uint16_t single_panel_height,
                     VirtualPanelScanRate scan_rate = VP_FOUR_SCAN_32PX_HIGH)
    : display(&disp),
      virt_w(panel_virtual_width),
      virt_h(panel_virtual_height),
      scan_rate(scan_rate),
      panel_pixel_base(panel_virtual_width),
      spw(single_panel_width),
      sph(single_panel_height)
  {}
#endif

  // Adafruit_GFX virtual — all text/line/shape primitives call this.
  void drawPixel(int16_t x, int16_t y, uint16_t color) override;

  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
  void fillScreen(uint16_t color) { display->fillScreen(color); }
  void clearScreen()              { display->clearScreen(); }
  void flipDMABuffer()            { display->flipDMABuffer(); }
  void setPanelBrightness(int b)  { display->setPanelBrightness(b); }

  uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ::color565(r, g, b); }

#ifdef NO_GFX
  inline uint16_t width()  const { return virt_w; }
  inline uint16_t height() const { return virt_h; }
#endif

  void setPanelPixelBase(uint8_t base) { panel_pixel_base = base; }

private:
  MatrixPanel_DMA *display;
#ifdef NO_GFX
  uint16_t virt_w;
  uint16_t virt_h;
#endif
  VirtualPanelScanRate scan_rate;
  uint8_t panel_pixel_base;
  uint16_t spw;  // single panel width  (physical pixels)
  uint16_t sph;  // single panel height (physical pixels)

  bool getCoords(int16_t x, int16_t y, int16_t &out_x, int16_t &out_y);
};

inline bool VirtualMatrixPanel::getCoords(int16_t x, int16_t y,
                                           int16_t &out_x, int16_t &out_y)
{
#ifndef NO_GFX
  if (x < 0 || x >= _width || y < 0 || y >= _height)
    return false;
#else
  if (x < 0 || x >= (int16_t)virt_w || y < 0 || y >= (int16_t)virt_h)
    return false;
#endif

  out_x = x;
  out_y = y;

  switch (scan_rate) {
    case VP_FOUR_SCAN_32PX_HIGH: {
      // Decompose virtual (x,y) into panel grid position + local pixel.
      // panels_x = total_width / single_panel_width (e.g. 128/64 = 2).
      // Panels are chained left-to-right, top-to-bottom in the DMA shift register:
      //   panel_idx = panel_row * panels_x + panel_col
      // Each panel occupies (spw*2) DMA columns (column interleave ×2).
#ifndef NO_GFX
      int16_t panels_x   = (int16_t)(_width  / spw);
#else
      int16_t panels_x   = (int16_t)(virt_w  / spw);
#endif
      int16_t panel_col  = out_x / (int16_t)spw;
      int16_t panel_row  = out_y / (int16_t)sph;
      int16_t local_x    = out_x % (int16_t)spw;
      int16_t local_y    = out_y % (int16_t)sph;
      int16_t panel_idx  = panel_row * panels_x + panel_col;
      int16_t bank       = ((local_y & 8) == 0) ? 1 : 0;
      out_x = panel_idx * (int16_t)(spw * 2) + 2 * local_x + bank;
      out_y = (int16_t)(local_y & 7) | (int16_t)(((local_y >> 4) & 1) * 8);
      break;
    }

    case VP_FOUR_SCAN_16PX_HIGH: {
      int16_t bank = ((out_y & 4) == 0) ? 1 : 0;
      out_x = 2 * out_x + bank;
      out_y = (int16_t)(((out_y >> 3) & 1) * 4) + (int16_t)(2 * (out_y & 1));
      break;
    }

    default:
      break;
  }

  return true;
}

inline void VirtualMatrixPanel::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  int16_t dx, dy;
  if (!getCoords(x, y, dx, dy)) return;
  display->writePixelDMA(dx, dy, color);
}

inline void VirtualMatrixPanel::drawPixelRGB888(int16_t x, int16_t y,
                                                 uint8_t r, uint8_t g, uint8_t b)
{
  drawPixel(x, y, color565(r, g, b));
}

#endif // ESP32_HUB75_VIRTUAL_MATRIX_PANEL_H
