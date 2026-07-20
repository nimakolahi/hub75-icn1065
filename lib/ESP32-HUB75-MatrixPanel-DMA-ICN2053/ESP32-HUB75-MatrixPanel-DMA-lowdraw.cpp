#include "ESP32-HUB75-MatrixPanel-DMA.h"

#ifndef _swap_int16
#define _swap_int16(a, b)\
  {                      \
    int16_t t = a;       \
    a = b;               \
    b = t;               \
  }
#endif

typedef union 
{
  uint8_t* p8;
  uint16_t* p16;
  uint32_t* p32;
  vbuffer_t* pXX;
}uniptr_t;


void MatrixPanel_DMA::fillRectFrameBuffer(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
  if ( !initialized ) 
  { 
    #ifdef SERIAL_DEBUG 
    Serial.println(F("Cannot updateMatrixDMABuffer as setup failed!"));
    #endif         
    return;
  }

  uniptr_t addr;
  int offset_y = y1*frame_buffer.row_len;
  int x;
  y2 -= y1;
  x2 -= x1;  
  int16_t _x2 = x2;
  while (y2 >= 0) 
  {
    switch (dma_buff.color_bits)  
    {
      #ifdef USE_COLORx16
      case -16:
        addr.pXX = &frame_buffer.frameBits[back_buffer_id][offset_y];
      break;
      #endif
    } 
    x = x1;
    while (x2 >= 0) 
    {
      switch (dma_buff.color_bits)  
      {
        #ifdef USE_COLORx16
        case -16:
          addr.p16[x] = (uint16_t)CurColor;
        break;
        #endif
        default:
          drawUserPixel(x1,y1,CurRGB);
        break;
      } 
      x++;
      x2--;
    }
    y1++;
    offset_y += frame_buffer.row_len;
    x2 = _x2;
    y2--;
  }
}


void MatrixPanel_DMA::fillRectBuffer(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if (frame_buffer.len != 0)
  {
    if (!m_cfg.double_buff) 
      waitDmaReady(); //первичный буфер без двойной буферизации, нужно ждать конца его вывода
    fillRectFrameBuffer(x, y, x + w - 1, y + h - 1);    
  }else
  {
    //shift
    //fillRectDMABuffer(x, y, x + w - 1, y + h - 1);
  }
}

void MatrixPanel_DMA::fillRectBufferVirtual(int16_t x, int16_t y, int16_t w, int16_t h)
{    
  //пропуск скрытого
  //en: if the rectangle is completely outside the display boundaries, skip drawing.
  if(x < 0) 
  {
    w += x;
    x = 0;    
  }
  if (w <= 0) return;
  if(y < 0) 
  {
    h += y;
    y = 0;
  }
  if (h <= 0) return;
  
  //поворот
  //en: if the display is rotated, swap the x and y coordinates and the width and height.
  if (rotation & 1)
  {
    _swap_int16(x,y);
    _swap_int16(h,w);
  }
  //пропуск скрытого
  if ((x >= WIDTH)||(y >= HEIGHT)) 
  {
    return;
  }
  
  //en: if the rectangle exceeds the display boundaries, adjust its size to fit within the display.
  if (x + w >= WIDTH) w = WIDTH - x;
  if (y + h >= HEIGHT) h = HEIGHT - y;

  //if (!top_down_chain)
  //mirror_x = 1;
  //mirror_y = 1;
  if(mirror_x)
  {
    //En: for non-serpentine, mirror_x inverts each panel; for serpentine, mirror_x inverts odd rows of panels.
    x = WIDTH - x - w;
  }

  if(mirror_y)
  {
    //En: for non-serpentine, mirror_y inverts each panel; for serpentine, mirror_y inverts odd rows of panels.
    y = HEIGHT - y - h;
  }

  if (!virtual_draw)
  {
    if (m_cfg.phys_width && m_cfg.phys_height) {
      // 1/8-scan remap: physical (phys_width x phys_height) → DMA (2*phys_width x phys_height/2)
      // Columns interleaved: even DMA col = bank A (rows 8-15, 24-31),
      //                       odd  DMA col = bank B (rows 0-7,  16-23)
      // fb_row → physical: physical = (fb >> 1) | ((fb & 1) << 2)  (3-bit left-rotate)
      // Inverse (physical → fb_row): dma_y = ((ry_in_band & 3) << 1) | ((ry_in_band >> 2) & 1)  (3-bit right-rotate)
      // dma_x = 2*x + ((y & 8) == 0 ? 1 : 0)
      if (frame_buffer.len == 0) return;
      if (!m_cfg.double_buff) waitDmaReady();
      for (int16_t ry = y; ry < y + h; ry++) {
        int16_t dma_x0 = (2 * x) + (int16_t)((ry & 8) == 0 ? 1 : 0);
        int16_t ry_in_band = ry & 7;
        int16_t dma_y  = (int16_t)(ry >> 4) * 8 + (int16_t)((ry_in_band & 3) << 1) + (int16_t)((ry_in_band >> 2) & 1);
        uniptr_t addr;
        addr.pXX = &frame_buffer.frameBits[back_buffer_id][dma_y * frame_buffer.row_len];
        for (int16_t rx = 0; rx < w; rx++) {
          addr.p16[dma_x0 + 2 * rx] = (uint16_t)CurColor;
        }
      }
      return;
    }
    //панели в одну строку
    fillRectBuffer(x,y,w,h);
    return;
  }

  int16_t coord_x,coord_y;
  uint8_t row = (y / m_cfg.mx_height); //a non indexed 0 row number
  //int16_t _h;
  
  if (serpentine_chain & row & 1)
  {
    coord_x = (m_cfg.mx_count_height - row)*WIDTH - x - w;
    coord_y = (row+1)*m_cfg.mx_height - y;
    y = coord_y;
    if (y > h) y = h;
    coord_y -= y;
  }else
  {
    coord_x = (m_cfg.mx_count_height - row - 1)*WIDTH + x;
    coord_y = y - row*m_cfg.mx_height;
    y = m_cfg.mx_height - coord_y;   
    if (y > h) y = h;
  }

  //смещение следующих прямоугольников
  x = (x << 1) + w;  
  while(true)
  {
    if (!top_down_chain)
    {
      //коррекция под порядок соединения в столбце
      fillRectBuffer(pixels_per_row - coord_x - w,coord_y,w,y);
    }else
    {
      fillRectBuffer(coord_x,coord_y,w,y);
    }
    //return;
    //setColor(255,0,0);
    //Serial.print(coord_x);Serial.print(":");Serial.print(coord_y);Serial.print("-");Serial.print(w);Serial.print(":");Serial.println(y);
    h -= y;
    if (h <= 0) break;
    row++;
    y = m_cfg.mx_height;
    if (y > h) y = h;
    if(serpentine_chain)
    {
      if(row & 1)
      {
        //нечетные строки панелей (нумерация от 0)
        coord_x -= x;
        coord_y = m_cfg.mx_height - y;
      }else      
      {
        //четые строки панелей (нумерация от 0)
        coord_x -= (WIDTH<<1) - x;
        coord_y = 0;
      }
    }else 
    {
      coord_x -= WIDTH;
      coord_y = m_cfg.mx_height - y;
    }
  }  
}

void MatrixPanel_DMA::setRotate(rotate_t _rotate) 
{
	// We don't support rotation by degrees.
  setRotation(_rotate); 
  setMirrorX(show_mirror_x);
  setMirrorY(show_mirror_y);  
}

void MatrixPanel_DMA::setMirrorX(bool _mirror_x) 
{
  show_mirror_x = _mirror_x;
  if (rotation&1)
  {
    mirror_y = (rotation >> 1)^_mirror_x;
  }else
  {
    mirror_x = (rotation >> 1)^_mirror_x^top_down_chain^1;	
  }
}

void MatrixPanel_DMA::setMirrorY(bool _mirror_y) 
{
  show_mirror_y = _mirror_y;
  if (rotation&1)
  {
    mirror_x = (rotation >> 1)^_mirror_y^top_down_chain;	
  }else
	{  
    mirror_y = (rotation >> 1)^_mirror_y;	
  }
}

void MatrixPanel_DMA::setPanelBrightness(int b)
{
  if ( !initialized ) 
  { 
    #ifdef SERIAL_DEBUG 
    Serial.println(F("Cannot updateMatrixDMABuffer as setup failed!"));
    #endif         
    return;
  }

  waitDmaReady();
  #ifdef SERIAL_DEBUG 
    Serial.print("Set brightness ");
    Serial.println(b);
  #endif    
  //Serial.println(b);
  if (brightness_table != NULL)
  {
    uint32_t g;    
    for(int i = BRIGHT_TABLE_SIZE-1; i >= 0; i--)
    {
      brightness_table[i] = (lumConvTab[b]|(lumConvTab[b]<<8))*i/(BRIGHT_TABLE_SIZE-1);
    }
  }else
  {
    //shift
  }  
}

void MatrixPanel_DMA::setBrightness8(uint8_t b)
{
  brightness = b;
  setPanelBrightness(b);
}

void MatrixPanel_DMA::invertDisplay(bool negative)
{
  waitDmaReady();
  negative_panel = negative;
  sendFrame();
}
