/*
  Various LED Driver chips might need some specific code for initialisation/control logic 

*/

#include <Arduino.h>
#include "ESP32-HUB75-MatrixPanel-DMA-leddrivers.h"
#include "ESP32-HUB75-MatrixPanel-DMA.h"

#define offset_prefix  dma_buff.all_row_data_cnt
#define offset_suffix  dma_buff.all_row_data_cnt + dma_buff.frame_prefix_cnt

//#define CLK_PULSE          digitalWrite(_cfg.gpio.clk, HIGH); digitalWrite(_cfg.gpio.clk, LOW);
enum{CLK_PULSE_DELAY = 1};
enum{CMD_DELAY = 10};
enum{LINE_DELAY = 50};

//массив команд записи в регистры
const uint8_t ICN2053_REG_CMD[ICN2053_REG_CNT] = {
    ICN2053_WR_CFG1, 
    ICN2053_WR_CFG2, 
    ICN2053_WR_CFG3,
    ICN2053_WR_CFG4,
    ICN2053_WR_DBG,
};

//массив значений для команд записи в регистры
const driver_rgb_t ICN2053_REG_VALUE[ICN2053_REG_CNT] = {
    {ICN2053_CFG1_R,ICN2053_CFG1_G,ICN2053_CFG1_B},
    {ICN2053_CFG2_R,ICN2053_CFG2_G,ICN2053_CFG2_B},
    {ICN2053_CFG3_R,ICN2053_CFG3_G,ICN2053_CFG3_B},
    {ICN2053_CFG4_R,ICN2053_CFG4_G,ICN2053_CFG4_B},
    {ICN2053_DBG_x, ICN2053_DBG_x, ICN2053_DBG_x},
};

//array of values ​​for commands to write to registers
const driver_rgb_t ICN1065_REG_VALUE[ICN1065_REG_CNT] = {
    {0x00aa, 0x00aa, 0x00aa},
    {0x01aa, 0x01aa, 0x01aa},
    {0x022a, 0x022a, 0x022a},
    {0x0335, 0x0335, 0x0335},
    {0x0412, 0x0412, 0x0412},
    {0x0500, 0x0500, 0x0500},
    {0x0601, 0x0601, 0x0601},
    {0x0720, 0x0720, 0x0720},
    {0x0c18, 0x0c18, 0x0c18},
    {0x0d01, 0x0d01, 0x0d01},
    {0x0e86, 0x0e86, 0x0e86},
    {0x0f01, 0x0f01, 0x0f01},
    {0x1040, 0x1040, 0x1040},
    {0x1127, 0x1127, 0x1127},
    {0x1200, 0x1200, 0x1200},
    {0x1300, 0x1300, 0x1300},
    {0x1400, 0x1400, 0x1400},
    {0x1500, 0x1500, 0x1500},
    {0x1600, 0x1600, 0x1600},
    {0x1800, 0x1800, 0x1800},
    {0x1906, 0x1906, 0x1906},
    {0x1c60, 0x1c60, 0x1c60},
    {0x1dca, 0x1dca, 0x1dca},
    {0x1e73, 0x1e73, 0x1e73},
    {0x1f00, 0x1f00, 0x1f00},
    {0x2000, 0x2000, 0x2000},
    {0x2100, 0x2100, 0x2100},
    {0x2200, 0x2200, 0x2200},
    {0x2300, 0x2300, 0x2300},
    {0x2400, 0x2400, 0x2400},
    {0x2500, 0x2500, 0x2500},
    {0x2600, 0x2600, 0x2600},
    {0x2700, 0x2700, 0x2700},
    {0x7000, 0x7000, 0x7000},
    {0x7100, 0x7100, 0x7100},
    {0x7200, 0x7200, 0x7200},
    {0x7300, 0x7300, 0x7300},
    {0x74a0, 0x74a0, 0x74a0}
};

//массив значений для команд записи в регистры
const driver_rgb_t ICN2038_REG_VALUE[ICN2038_REG_CNT] = {
    {ICN2038_REG1_VALUE,ICN2038_REG1_VALUE,ICN2038_REG1_VALUE},
    {ICN2038_REG2_VALUE,ICN2038_REG2_VALUE,ICN2038_REG2_VALUE},
};


//установка последовательности бит в буфере в буфере
int data_set(ESP32_I2S_DMA_STORAGE_TYPE* buffer, int offset,  ESP32_I2S_DMA_STORAGE_TYPE data_mask, int len)
{	
	while(len > 0)
	{
		buffer[offset^1] |= data_mask;
    offset++;
    len--;
	}
	return offset;
}

//сброс последовательности бит в буфере в буфере
int data_clr(ESP32_I2S_DMA_STORAGE_TYPE* buffer, int offset,  ESP32_I2S_DMA_STORAGE_TYPE data_mask, int len)
{
	while(len > 0)
	{
		buffer[offset^1] &= ~data_mask;
    offset++;
    len--;
	}
	return offset;
}

//устновка значения регистра\пикселя в буфере
int setDataRegBuffer(ESP32_I2S_DMA_STORAGE_TYPE* buffer, int offset, const driver_rgb_t* regs_data)
{
  driver_reg_t r,g,b;
  ESP32_I2S_DMA_STORAGE_TYPE d;
	r = regs_data->r;
	g = regs_data->g;
	b = regs_data->b;
	for (int i = DRIVER_BITS; i > 0; i--)
	{
		d = 0;
		if (r < 0) d |= BIT_R1|BIT_R2; r <<= 1;
		if (g < 0) d |= BIT_G1|BIT_G2; g <<= 1;
		if (b < 0) d |= BIT_B1|BIT_B2; b <<= 1;

    int j = offset^1;
		buffer[j] = (buffer[j] & ~BITMASK_RGB12) | d;
		offset++;
	}  
  return offset;
}

//запись регистров\заливка строки пикселов в буфере
int setDataRegBuffer_n(ESP32_I2S_DMA_STORAGE_TYPE* buffer, int offset, driver_rgb_t* regs_data, int regs_cnt)
{
	int i;
  //Serial.println(regs_cnt);
	for(i = regs_cnt-1; i >= 0; i--)
	{
    //Serial.println(offset);
    offset = setDataRegBuffer(buffer, offset, regs_data);
	}
  return offset;
}

//получение маски значения адреса строки
ESP32_I2S_DMA_STORAGE_TYPE getAddrBits(uint8_t addr)
{
  ESP32_I2S_DMA_STORAGE_TYPE BIT_ADDR[ROW_ADDR_BITS] = {BIT_A, BIT_B, BIT_C, BIT_D, BIT_E};
  ESP32_I2S_DMA_STORAGE_TYPE res = 0;
  int i;
  for (i = 0; i < ROW_ADDR_BITS; i++)
  {
    if (addr & 1) res |= BIT_ADDR[i];
    addr >>= 1;
  }
  return res;
}

//установка LAT в конце подстрок в буфере
int setLatRowBuffer(ESP32_I2S_DMA_STORAGE_TYPE* buffer, int offset, int subrow_cnt, int subrow_len)
{
  int i = offset + subrow_len-1;
  while(subrow_cnt > 0)
  {
    buffer[i^1] |= BIT_LAT;
    i += subrow_len + SUBROW_ADD_LEN;
    subrow_cnt--;
  }
  return offset;
}

//заполнение буфера регенерации строк в буфере
//возвращает смещения начала кадра (для стыковки буферов разной длины к суффиксу)
int icn20xxsetOEaddrBuffer(ESP32_I2S_DMA_STORAGE_TYPE* buffer, int offset, uint8_t row_cnt, size_t buffer_len, int frame_offset,
                          int row_oe_cnt, int row_oe_add_len, bool decoder_INT595 = false)
{
  int row_oe_len = row_oe_cnt * 2 + row_oe_add_len;
  enum {DELAY_INT595_START = 2};
  int DELAY_INT595_LEN = row_oe_add_len / 2 - DELAY_INT595_START;
  int DELAY_INT595_CLK = row_oe_add_len / 4 - DELAY_INT595_START;
  ESP32_I2S_DMA_STORAGE_TYPE data;
  ESP32_I2S_DMA_STORAGE_TYPE start_INT595 = 0;
  uint8_t addr;
  int row_offset;
  int oe_cnt;
  int clk_INT595_cnt = 255; //длительность клока в тактах
  //вычисление начальных значений счетчиков
  if (frame_offset >= (row_oe_len << ROW_ADDR_BITS)) frame_offset = 0;
  addr = frame_offset / row_oe_len;
  row_offset = frame_offset % row_oe_len;
  if (row_offset > row_oe_cnt * 2) oe_cnt = 0;
  else oe_cnt = (row_oe_cnt - (row_offset >> 1));

  if(!decoder_INT595) data = getAddrBits(addr);
  else data = 0;
 
  while (offset < buffer_len)
  {
    if(row_offset == row_oe_len)
    {      
      addr++;
      if (addr >= row_cnt) 
      {
        addr = 0;
        frame_offset = 0;
      }else if (addr == row_cnt - 1) //два канала: R1G1B1 и R2G2B2
      {
        start_INT595 = BIT_SDI;
      }
      if(!decoder_INT595) data = getAddrBits(addr);
      else data = 0;
      row_offset = 0;
      oe_cnt = row_oe_cnt;
    }

    if(decoder_INT595)
    {     
      if (oe_cnt > 0) 
      {
        buffer[offset^1] = BIT_OE;
        oe_cnt--;
        if (oe_cnt == 0)
        {
          clk_INT595_cnt = -DELAY_INT595_START;
        }
      }else 
      {
        if (clk_INT595_cnt <= DELAY_INT595_LEN)
        {
          clk_INT595_cnt++; 
          if(clk_INT595_cnt >= 0)    
          {
            if (clk_INT595_cnt == DELAY_INT595_LEN)
            {
              start_INT595 = 0;
              data = 0;
            }else
            {       
              data = start_INT595 | BIT_RCK;
              if (clk_INT595_cnt >= DELAY_INT595_CLK) data |= BIT_DTK;                     
            }
          }          
        }         
        buffer[offset^1] = data;      
      }
      row_offset++;
      frame_offset++;
      offset++;
      if (offset == buffer_len) break;
      buffer[offset^1] = data;      
    }else
    {
      if (oe_cnt > 0) 
      {
        buffer[offset^1] = data | BIT_OE;
        oe_cnt--;
      }else buffer[offset^1] = data;
      row_offset++;
      frame_offset++;
      offset++;
      if (offset == buffer_len) break;
      buffer[offset^1] = data;
    }
    
    row_offset++;
    frame_offset++;
    offset++;
  }
  return frame_offset++;
}

//установка заголовка кадра в буфере
int icn20xxsetVSyncBuffer(ESP32_I2S_DMA_STORAGE_TYPE* vsync_buffer, int offset, uint8_t driver, bool leds_enable, bool vsync)
{	  
  if (driver == ICN1065)
  {
    // VSYNC + pre-active command train for ICN1065 class drivers
    offset = data_set(vsync_buffer, offset, BIT_LAT, ICN1065_V_SYNC);
    offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN1065_CLK_PAD);
    offset = data_set(vsync_buffer, offset, BIT_LAT, ICN1065_PRE_ACT1);
    offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN1065_CLK_PAD);
    offset = data_set(vsync_buffer, offset, BIT_LAT, ICN1065_PRE_ACT2);
    offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN1065_CLK_PAD);
    return offset;
  }

  //if (vsync_buffer == NULL) return;
  offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN2053_CMD_DELAY);
  offset = data_set(vsync_buffer, offset, BIT_LAT, ICN2053_PRE_ACT);
  offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN2053_CMD_DELAY);
  int n = ICN2053_CMD_DELAY;
  if (leds_enable)
  {
    offset = data_set(vsync_buffer, offset, BIT_LAT, ICN2053_EN_OP);
    n += ICN2053_DIS_OP - ICN2053_EN_OP;
  }else
  {
    offset = data_set(vsync_buffer, offset, BIT_LAT, ICN2053_DIS_OP);
  }
  offset = data_clr(vsync_buffer, offset, BIT_LAT, n);
  if (leds_enable)
    offset = data_set(vsync_buffer, offset, BIT_LAT, ICN2053_V_SYNC);
  else
    offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN2053_V_SYNC);
  offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN2053_CMD_DELAY);
  offset = data_set(vsync_buffer, offset, BIT_LAT, ICN2053_PRE_ACT);
  offset = data_clr(vsync_buffer, offset, BIT_LAT, ICN2053_CMD_DELAY);
  return offset;
}

//установка заголовка кадра
void MatrixPanel_DMA::icn2053setVSync(bool leds_enable, bool vsync)
{
  // For ICN1065 the vsync train sits AFTER the inserted quiet region (see
  // ICN1065_PRE_VSYNC_QUIET in leddrivers.h). The quiet words stay all-zero
  // from calloc: OE low (row lit, no scan pulses), LAT low.
  int vsync_offs = FRAME_ADD_LEN;
  if (m_cfg.driver == ICN1065) vsync_offs += ICN1065_PRE_VSYNC_QUIET;
  icn20xxsetVSyncBuffer(dma_buff.rowBits[offset_prefix], vsync_offs, m_cfg.driver, leds_enable, vsync);
}

//setting the values ​​of configuration registers
void MatrixPanel_DMA::icn2053setReg(uint8_t reg_idx, driver_rgb_t* regs_data)
{
  if (m_cfg.driver == ICN1065)
  {
    driver_rgb_t wr;
    int wr_offs = FRAME_ADD_LEN + ICN1065_PREFIX_START_LEN;

    wr.r = ICN1065_WRAPPER_0; wr.g = ICN1065_WRAPPER_0; wr.b = ICN1065_WRAPPER_0;
    setDataRegBuffer_n(dma_buff.rowBits[offset_prefix], wr_offs, &wr, driver_cnt);
    data_clr(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - DRIVER_BITS, BIT_LAT, DRIVER_BITS);
    data_set(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - ICN1065_REG_LATCH, BIT_LAT, ICN1065_REG_LATCH);

    wr_offs += pixels_per_row;
    wr.r = ICN1065_WRAPPER_1; wr.g = ICN1065_WRAPPER_1; wr.b = ICN1065_WRAPPER_1;
    setDataRegBuffer_n(dma_buff.rowBits[offset_prefix], wr_offs, &wr, driver_cnt);
    data_clr(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - DRIVER_BITS, BIT_LAT, DRIVER_BITS);
    data_set(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - ICN1065_REG_LATCH, BIT_LAT, ICN1065_REG_LATCH);

    wr_offs += pixels_per_row;
    setDataRegBuffer_n(dma_buff.rowBits[offset_prefix], wr_offs, regs_data, driver_cnt);
    data_clr(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - DRIVER_BITS, BIT_LAT, DRIVER_BITS);
    data_set(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - ICN1065_REG_LATCH, BIT_LAT, ICN1065_REG_LATCH);

    wr_offs += pixels_per_row;
    wr.r = ICN1065_WRAPPER_2; wr.g = ICN1065_WRAPPER_2; wr.b = ICN1065_WRAPPER_2;
    setDataRegBuffer_n(dma_buff.rowBits[offset_prefix], wr_offs, &wr, driver_cnt);
    data_clr(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - DRIVER_BITS, BIT_LAT, DRIVER_BITS);
    data_set(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - ICN1065_REG_LATCH, BIT_LAT, ICN1065_REG_LATCH);

    wr_offs += pixels_per_row;
    wr.r = ICN1065_WRAPPER_3; wr.g = ICN1065_WRAPPER_3; wr.b = ICN1065_WRAPPER_3;
    setDataRegBuffer_n(dma_buff.rowBits[offset_prefix], wr_offs, &wr, driver_cnt);
    data_clr(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - DRIVER_BITS, BIT_LAT, DRIVER_BITS);
    data_set(dma_buff.rowBits[offset_prefix], wr_offs + pixels_per_row - ICN1065_REG_LATCH, BIT_LAT, ICN1065_REG_LATCH);
    return;
  }

  setDataRegBuffer_n(dma_buff.rowBits[offset_prefix], FRAME_ADD_LEN + ICN2053_PREFIX_START_LEN, regs_data, driver_cnt);
  data_clr(dma_buff.rowBits[offset_prefix], dma_buff.frame_prefix_len - DRIVER_BITS, BIT_LAT, DRIVER_BITS);
  data_set(dma_buff.rowBits[offset_prefix], dma_buff.frame_prefix_len - ICN2053_REG_CMD[reg_idx], BIT_LAT, ICN2053_REG_CMD[reg_idx]);
}

//инициализация буферов и дескрипторов
// Widen each ROW/OE scan pulse by ICN1065_OE_PULSE_EXTRA ticks after its rising
// edge, to satisfy the ICND1065's 280 ns twROW minimum at faster DCLK.
// Contiguous high extension only — edge count and edge positions are unchanged.
// Walks in TEMPORAL order: index t^1 because the I2S peripheral swaps each
// 16-bit word pair.
#if ICN1065_OE_PULSE_EXTRA > 0
static void icn1065WidenOEPulses(ESP32_I2S_DMA_STORAGE_TYPE* buffer, size_t len)
{
  bool prev_high = false;   // widening at t=0 is safe either way (never adds an edge)
  for (size_t t = 0; t < len; t++)
  {
    bool high = (buffer[t ^ 1] & BIT_OE) != 0;
    if (high && !prev_high)
      for (size_t k = t + 1; k <= t + ICN1065_OE_PULSE_EXTRA && k < len; k++)
        buffer[k ^ 1] |= BIT_OE;
    prev_high = (buffer[t ^ 1] & BIT_OE) != 0;
  }
}

// Additional widening for the address-0/scan-restart OE pulse ONLY, applied
// AFTER icn1065WidenOEPulses() has already widened every pulse in the buffer
// (including this one) to the regular ICN1065_OE_LEN_REGULAR width. Per
// board707 (github.com/board707/DMD_STM32#217, logic-analyzer capture against
// the datasheet), the pulse marking a scan restart must be
// ICN1065_OE_LEN_RESTART (12) clocks, not 4 — this tops up just the FIRST OE
// edge found in the buffer with the remaining ticks. Same contiguous-high,
// no-new-edge technique as icn1065WidenOEPulses (iron rule preserved).
//
// Only safe to call on buffers where the first OE edge really is a scan
// restart: offset_prefix and offset_suffix are both generated starting at
// frame_offset=0 (address 0), so their first edge always qualifies. Do NOT
// call this on the per-row data buffers — a restart can land mid-buffer there
// and this function has no address context to know where.
static void icn1065WidenRestartOEPulse(ESP32_I2S_DMA_STORAGE_TYPE* buffer, size_t len)
{
  enum { EXTRA_RESTART = ICN1065_OE_LEN_RESTART - ICN1065_OE_LEN_REGULAR };
  bool prev_high = false;
  for (size_t t = 0; t < len; t++)
  {
    bool high = (buffer[t ^ 1] & BIT_OE) != 0;
    if (high && !prev_high)
    {
      for (size_t k = t + 1; k <= t + EXTRA_RESTART && k < len; k++)
        buffer[k ^ 1] |= BIT_OE;
      return;   // only the first edge in the buffer — every later pulse is left alone
    }
    prev_high = high;
  }
}
#endif

void MatrixPanel_DMA::icn2053initBuffers()
{
  int row_oe_cnt;
  int row_oe_add_len;
  int frame_vsync_len;
  int d_suffix_cnt;
  if (m_cfg.driver == ICN1065)
  {
    row_oe_cnt = ICN1065_ROW_OE_CNT;
    row_oe_add_len = ICN1065_ROW_OE_ADD_LEN;
    frame_vsync_len = ICN1065_PREFIX_START_LEN;  // quiet region + vsync train
    d_suffix_cnt = ICN1065_DSUFFIX_CNT;
  }else
  {
    row_oe_cnt = ICN2053_ROW_OE_CNT;
    row_oe_add_len = ICN2053_ROW_OE_ADD_LEN;
    frame_vsync_len = ICN2053_VSYNC_LEN;
    d_suffix_cnt = ICN2053_DSUFFIX_CNT;
  }

  //заполняем регенерацию строк в буферах данных
  #ifdef SERIAL_DEBUG  
  Serial.print("DMA buffers init: rows  ");
  #endif     
  int frame_offset_data = 0; //индекс смещения относительно начала кадра
  int row_offset = 0;
  for(int buf_id = m_cfg.double_dma_buff; buf_id >= 0; buf_id--)
  {
    frame_offset_data = 0;
    for(int row = 0; row < dma_buff.row_data_cnt; row++)
    {
      #ifdef SERIAL_DEBUG
      Serial.print(row); Serial.print(" ");
      #endif
      frame_offset_data = icn20xxsetOEaddrBuffer(dma_buff.rowBits[row_offset + row], 0, rows_per_frame, dma_buff.row_data_len,
                                                 frame_offset_data, row_oe_cnt, row_oe_add_len, m_cfg.decoder_INT595);
      setLatRowBuffer(dma_buff.rowBits[row_offset + row],0,DRIVER_BITS,pixels_per_row);
#if ICN1065_OE_PULSE_EXTRA > 0
      if (m_cfg.driver == ICN1065)
        icn1065WidenOEPulses(dma_buff.rowBits[row_offset + row], dma_buff.row_data_len);
#endif
    }
    dmadesc_data[buf_id][desc_data_cnt-1].eof = true;
    dmadesc_data[buf_id][desc_data_cnt-1].qe.stqe_next = &dmadesc_ext[ICN2053_EXT_DATA];
    row_offset += dma_buff.row_data_cnt;
  }

  frame_offset_data %= dma_buff.frame_suffix_len;

  #ifdef SERIAL_DEBUG  
  Serial.println("\r\nDMA buffers init: calculate ext descriptors");
  #endif     
  frame_offset_data *= SIZE_DMA_TYPE;
  //находим дескриптор куда входит текущее смещение frame_offset, чтобы из него взять параметры буфера
  int desk_idx_datain = frame_offset_data/DMA_MAX;
  //находим смещение в буфере для получения адреса буфера переходного дескриптора
  int offset_bufer_datain = frame_offset_data % DMA_MAX;

  //индекс смещения относительно начала кадра
  int frame_offset_prefix = icn20xxsetOEaddrBuffer(dma_buff.rowBits[offset_prefix], FRAME_ADD_LEN + frame_vsync_len,
                                                    rows_per_frame, dma_buff.frame_prefix_len, 0, row_oe_cnt, row_oe_add_len, m_cfg.decoder_INT595);
#if ICN1065_OE_PULSE_EXTRA > 0
  if (m_cfg.driver == ICN1065)
  {
    icn1065WidenOEPulses(dma_buff.rowBits[offset_prefix], dma_buff.frame_prefix_len);
    // Prefix generation above starts fresh at address 0 (frame_offset=0), so
    // its first OE edge is always the scan-restart pulse — see
    // icn1065WidenRestartOEPulse() for why this is safe here.
    icn1065WidenRestartOEPulse(dma_buff.rowBits[offset_prefix], dma_buff.frame_prefix_len);
  }
#endif

  frame_offset_prefix %= dma_buff.frame_suffix_len;
  frame_offset_prefix *= SIZE_DMA_TYPE;
  //находим дескриптор куда входит текущее смещение frame_offset, чтобы из него взять параметры буфера
  int desk_idx_prefixin = frame_offset_prefix/DMA_MAX;
  //находим смещение в буфере для получения адреса буфера переходного дескриптора
  int offset_bufer_prefixin = frame_offset_prefix % DMA_MAX;;
  //настариваем выход префикса на вход в суффикс
  dmadesc_prefix[desc_prefix_cnt-1].eof= true;
  dmadesc_prefix[desc_prefix_cnt-1].qe.stqe_next = &dmadesc_ext[ICN2053_EXT_PREFIX];

  #ifdef SERIAL_DEBUG  
  Serial.println("DMA buffers init: oe_addr suffix buffers");
  #endif     
  //заполняем буфер регененерации строк
  icn20xxsetOEaddrBuffer(dma_buff.rowBits[offset_suffix], 0, rows_per_frame, dma_buff.frame_suffix_len, 0,
                         row_oe_cnt, row_oe_add_len, m_cfg.decoder_INT595);
#if ICN1065_OE_PULSE_EXTRA > 0
  if (m_cfg.driver == ICN1065)
  {
    icn1065WidenOEPulses(dma_buff.rowBits[offset_suffix], dma_buff.frame_suffix_len);
    // Suffix generation above also starts fresh at address 0 (frame_offset=0),
    // AND this is a static buffer the DMA re-reads on every loop iteration, so
    // baking the wider restart pulse in once is correct on every replay too.
    icn1065WidenRestartOEPulse(dma_buff.rowBits[offset_suffix], dma_buff.frame_suffix_len);
  }
#endif

  //int desk_idx_next;
  //заполняем регенерацию строк в буфере суффиксов
  for (int desc_suffix = d_suffix_cnt - 1; desc_suffix >= 0; desc_suffix--)
  {
    #ifdef SERIAL_DEBUG  
    Serial.print("DMA descriptors init for suffix: "); Serial.println(desc_suffix);
    #endif     
    //заполняем дескрипторы регененерации строк 
    dmadesc_suffix[desc_suffix][desc_suffix_cnt-1].eof = true;
    dmadesc_suffix[desc_suffix][desc_suffix_cnt-1].qe.stqe_next = &dmadesc_suffix[desc_suffix][0];
  }      

  #ifdef SERIAL_DEBUG  
  Serial.println("DMA ext descriptor init for data");
  #endif     
  //заполняем дескриптор входа данных строк
  dmadesc_ext[ICN2053_EXT_DATA].buf = &dmadesc_suffix[0][desk_idx_datain].buf[offset_bufer_datain];  
  dmadesc_ext[ICN2053_EXT_DATA].size = dmadesc_suffix[0][desk_idx_datain].size - offset_bufer_datain;
  dmadesc_ext[ICN2053_EXT_DATA].length = dmadesc_ext[ICN2053_EXT_DATA].size;
  //следующий дескриптор после переходного
  if (desk_idx_datain == (desc_suffix_cnt - 1))
  {
    //Serial.println("D");
    //dmadesc_ext[ICN2053_EXT_DATA].eof = true;
    data_to_suffix = 0;
  }else data_to_suffix = desk_idx_datain + 1;
  dmadesc_ext[ICN2053_EXT_DATA].qe.stqe_next = &dmadesc_suffix[0][data_to_suffix];
  
  #ifdef SERIAL_DEBUG  
  Serial.println("DMA ext descriptor init for prefix: ");
  #endif     
  //заполняем дескриптор входа префикса
  dmadesc_ext[ICN2053_EXT_PREFIX].buf = &dmadesc_suffix[0][desk_idx_prefixin].buf[offset_bufer_prefixin];  
  dmadesc_ext[ICN2053_EXT_PREFIX].size = dmadesc_suffix[0][desk_idx_prefixin].size - offset_bufer_prefixin;
  dmadesc_ext[ICN2053_EXT_PREFIX].length = dmadesc_ext[ICN2053_EXT_PREFIX].size;
  //следующий дескриптор после переходного
  if (desk_idx_prefixin == (desc_suffix_cnt - 1))
  {
    //Serial.println("P");
    //dmadesc_ext[ICN2053_EXT_PREFIX].eof = true;
    prefix_to_suffix = 0;
  }else prefix_to_suffix = desk_idx_prefixin + 1;
  dmadesc_ext[ICN2053_EXT_PREFIX].qe.stqe_next = &dmadesc_suffix[0][prefix_to_suffix];
}

void setDriverReg(driver_reg_t& driver_reg, driver_reg_t value, driver_reg_t mask, uint8_t offset)
{
  driver_reg = (driver_reg & (~mask))|((value << offset) & mask);
}

void setDriverRegRGB(driver_rgb_t* driver_reg, driver_reg_t value, driver_reg_t mask, uint8_t offset)
{
  setDriverReg(driver_reg->r, value, mask, offset);
  setDriverReg(driver_reg->g, value, mask, offset);
  setDriverReg(driver_reg->b, value, mask, offset);
}

#ifdef DIRECT_DRIVER_INIT


enum{NO_REG = 255}; //заглушка для отправки данных без команды по LAT

void preset_pin_out_low(int8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void preset_pin_out_high(int8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

void preset_pins_out_low(const hub75_pins_t& gpio)
{
  preset_pin_out_low(gpio.r1); preset_pin_out_low(gpio.g1); preset_pin_out_low(gpio.b1);
  preset_pin_out_low(gpio.r2); preset_pin_out_low(gpio.g2); preset_pin_out_low(gpio.b2);
  preset_pin_out_low(gpio.lat); preset_pin_out_low(gpio.clk); preset_pin_out_high(gpio.oe);
}

//позитивный клок
void write_clk(int8_t pin)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(CLK_PULSE_DELAY);
  digitalWrite(pin, LOW);
  delayMicroseconds(CLK_PULSE_DELAY);
}

void write_clk_n(int8_t pin, int n)
{
  while (n > 0)
  {
    write_clk(pin);
    n--;
  }
}

//негативный клок
void write_nclk(int8_t pin)
{
  digitalWrite(pin, LOW);
  delayMicroseconds(CLK_PULSE_DELAY);
  digitalWrite(pin, HIGH);
  delayMicroseconds(CLK_PULSE_DELAY);
}

void write_nclk_n(int8_t pin, int n)
{
  while (n > 0)
  {
    write_nclk(pin);
    n--;
  }
}

//запись регистров RGB совмещенная с записью данных
void icn2053_write_reg(const hub75_pins_t& gpio, uint8_t reg, const driver_rgb_t* data_regs)
{
  driver_rgb_t data_reg;
  int i;

  if (data_regs != NULL)
  {
    memcpy(&data_reg, data_regs, sizeof(data_reg));
    i = 16;
  }else
  {
    i = reg;
  }

  while (i > 0)
  {
    //отправка бита данных
    if (data_regs != NULL)
    {            
      digitalWrite(gpio.r1, (uint16_t)data_reg.r & 1);
      digitalWrite(gpio.g1, (uint16_t)data_reg.g & 1);
      digitalWrite(gpio.b1, (uint16_t)data_reg.b & 1);
      digitalWrite(gpio.r2, (uint16_t)data_reg.r & 1);
      digitalWrite(gpio.g2, (uint16_t)data_reg.g & 1);
      digitalWrite(gpio.b2, (uint16_t)data_reg.b & 1);
      data_reg.r >>= 1;
      data_reg.g >>= 1;
      data_reg.b >>= 1;
            
    }
    //запуск счета номера регистра
    if(reg == i)
    {
      digitalWrite(gpio.lat, HIGH);
    }
    //такт
    write_clk(gpio.clk);
    i--;
  }
  digitalWrite(gpio.lat, LOW);
  //delayMicroseconds(CMD_DELAY);
}

//отправка префикса
void icn2053_write_prefix(const hub75_pins_t& gpio)
{
  icn2053_write_reg(gpio, ICN2053_PRE_ACT, NULL);    
}

//запись линии регистров
void icn20xx_write_regs(const hub75_pins_t& gpio, uint8_t reg, const driver_rgb_t* data_regs, int cnt_regs)
{
  int i = cnt_regs - 1;
  while (i > 0)
  {
    icn2053_write_reg(gpio, NO_REG, data_regs);
      i--;
  }
  icn2053_write_reg(gpio, reg, data_regs);
}

//инициализация буферов и дескрипторов
void icn2038_init(const hub75_pins_t& gpio, int driver_cnt)
{
  preset_pins_out_low(gpio);
  digitalWrite(gpio.oe, HIGH);


  //Send Data to control register REG2 (enable LED output)
  icn20xx_write_regs(gpio, ICN2038_WR_REG2, &ICN2038_REG_VALUE[ICN2038_REG2], driver_cnt);
  write_clk_n(gpio.clk,16);
  //Send Data to control register REG1
  icn20xx_write_regs(gpio, ICN2038_WR_REG1, &ICN2038_REG_VALUE[ICN2038_REG1], driver_cnt);
  //blank data regs to keep matrix clear after manipulations
  write_clk_n(gpio.clk,driver_cnt*DRIVER_BITS);
  digitalWrite(gpio.oe, LOW);//Enable Display
}


void icn2053_init(const hub75_pins_t& gpio, int driver_cnt)
{
  preset_pins_out_low(gpio);

  int i;
  //запись регистров конфигурации
  for(i = 0; i < ICN2053_REG_CNT; i++) 
  {
    icn2053_write_prefix(gpio);
    write_clk_n(gpio.clk,16);
    icn20xx_write_regs(gpio, ICN2053_REG_CMD[i], &ICN2053_REG_VALUE[i], driver_cnt);
    write_clk_n(gpio.clk,16);
  }
}

void MatrixPanel_DMA::shiftDriverInit()
{
  switch (m_cfg.driver)
  {
    case MBI5124:
      // MBI5124 chips must be clocked with positive-edge, since it's LAT signal
      //resets on clock's rising edge while high
      //https://github.com/mrfaptastic/ESP32-HUB75-MatrixPanel-I2S-DMA/files/5952216/5a542453754da.pdf
      m_cfg.clk_phase = CLK_NEGATIVE;
      break;
    case ICN2038:
      icn2038_init(m_cfg.gpio,driver_cnt);
    case ICN2053:
      icn2053_init(m_cfg.gpio,driver_cnt);
      break;
    default: //SHIFTREG
      break;    
  }
}

#endif
