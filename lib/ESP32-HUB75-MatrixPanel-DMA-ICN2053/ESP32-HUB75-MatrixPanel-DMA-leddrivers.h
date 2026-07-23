#ifndef _ESP32_HUB75_MATRIXPANEL_DMA_LEDDRIVERS
#define _ESP32_HUB75_MATRIXPANEL_DMA_LEDDRIVERS

#include "stdbool.h"
#include "stdint.h"

//размерность регистров драйверов
typedef int16_t driver_reg_t;
//битность регистров
enum{DRIVER_BITS = sizeof(driver_reg_t)*8};
//enum{DRIVER_BITS = 16};

//структура данных регистров конфигурации на каждый канал цвета
typedef struct 
{
 driver_reg_t r;
 driver_reg_t g;
 driver_reg_t b;
}driver_rgb_t;

//------------------------------------------------ опционально ---------------------------------------------------
//выдержка между командами (для контроля через логический нализатор - опциональна)
enum{ICN2053_CMD_DELAY = 16};
//добавление зоны после каждой подстроки (для контроля через логический нализатор - опциональна)
enum{SUBROW_ADD_LEN = 8};
//добавление зоны после каждой строки (для контроля через логический нализатор - опциональна)
enum{ROW_ADD_LEN = 8};
//добавление зоны после каждого фрейма (для контроля через логический нализатор - опциональна)
enum{FRAME_ADD_LEN = 16};

//параметры длительности lat для SHIFT драйверов
enum{MAX_LAT_BLANKING = 4};
enum{DEFAULT_LAT_BLANKING = 1};

//------------------------------------ не менять (кроме значений регистров) ---------------------------------------

//битность адреса строки
enum{ROW_ADDR_BITS = 5};

//------------------------------------ ICN2038 ---------------------------------------
enum{
  ICN2038_REG1 = 0,
  ICN2038_REG2,
  ICN2038_REG_CNT
};

//биты регистров  для ICN2038
enum{
  ICN2038_WR_REG1 = 12,
  ICN2038_WR_REG2 = 13,
};

enum{
  // this sets global matrix brightness power
  ICN2038_REG1_VALUE = 0b0000011111100000,
  // a single bit enables the matrix output
  ICN2038_REG2_VALUE = 0b0000000001000000,
};


//------------------------------------ ICN2053 ---------------------------------------
//коды команд для ICN2053 (длина LAT)
enum{    
  ICN2053_DATA_LATCH = 1,
  ICN2053_WR_DBG = 2,     //PRE_ACT +
  ICN2053_V_SYNC = 3,
  ICN2053_WR_CFG1 = 4,    //PRE_ACT + 
  ICN2053_RD_CFG1 = 5,
  ICN2053_WR_CFG2 = 6,    //PRE_ACT +
  ICN2053_RD_CFG2 = 7,
  ICN2053_WR_CFG3 = 8,    //PRE_ACT + 
  ICN2053_RD_CFG3 = 9,
  ICN2053_WR_CFG4 = 10,   //PRE_ACT + 
  ICN2053_RD_CFG4 = 11,
  ICN2053_EN_OP = 12,     //PRE_ACT + 
  ICN2053_DIS_OP = 13, 
  ICN2053_PRE_ACT = 14,
};

//биты регистра DBG для ICN2053, взято с LEDVISON
enum{
  ICN2053_DBG_x = 0x08, //????
};

//биты регистра CFG1 для ICN2053, взято с LEDVISON
enum{    
  ICN2053_CFG1_WOBPS_MASK = 0x4000, //bad point detection current adjusment (0..1)
  ICN2053_CFG1_WOBPS_OFFSET = 14,
  ICN2053_CFG1_WOBPS = 0, //0: - sync

  //число активных строк драйвера - НЕ МЕНЯТЬ!!! - уставнавливается при инициализации через ИЛИ
  ICN2053_CFG1_LC_MASK = 0x1F00, //line count (0..31): 1..32 - sync
  ICN2053_CFG1_LC_OFFSET = 8,
  ICN2053_CFG1_LC = 19,    //15:

  ICN2053_CFG1_LGS_MASK = 0x00C0, //low gray spot (0..3) - sync
  ICN2053_CFG1_LGS_OFFSET = 6,
  ICN2053_CFG1_LGS = 1,   //1 

  ICN2053_CFG1_RRM_MASK = 0x0030, //refresh rate multiplayer (0..3): 1x, 2x, 4x, 8x - sync
  ICN2053_CFG1_RRM_OFFSET = 4,
  ICN2053_CFG1_RRM = 3,   //3

  ICN2053_CFG1_PWMR_MASK = 0x0008, //PWM reverce (0..1) - sync
  ICN2053_CFG1_PWMR_OFFSET = 3, 
  ICN2053_CFG1_PWMR = 0,  //0

  ICN2053_CFG1_R = (ICN2053_CFG1_WOBPS << 14) + (ICN2053_CFG1_LC << 8) + (ICN2053_CFG1_LGS << 6) + (ICN2053_CFG1_RRM << 4) + (ICN2053_CFG1_PWMR << 3),
  ICN2053_CFG1_G = ICN2053_CFG1_R,
  ICN2053_CFG1_B = ICN2053_CFG1_R,
};

//биты регистра CFG2 для ICN2053, взято с LEDVISON
enum{
  //порог уровня черного
  ICN2053_CFG2_BL_MASK = 0x7C00, //blanking level (0..31)
  ICN2053_CFG2_BL_OFFSET = 10,
  ICN2053_CFG2_BL_R = 31,//31, //31
  ICN2053_CFG2_BL_G = 28,//28, //28
  ICN2053_CFG2_BL_B = 23,//23, //23

  ICN2053_CFG2_CURRENT_MASK = 0x003E, //blanking enhancement (4..31): 13%..199%, 16 = 100%
  ICN2053_CFG2_CURRENT_OFFSET = 1,
  ICN2053_CFG2_CURRENT_R = 13, //13
  ICN2053_CFG2_CURRENT_G = 13, //13
  ICN2053_CFG2_CURRENT_B = 13, //13

  ICN2053_CFG2_BE_MASK = 0x0001, //blanking enhancement (0..1): 1,0
  ICN2053_CFG2_BE_OFFSET = 0,
  ICN2053_CFG2_BE_R = 1, //1
  ICN2053_CFG2_BE_G = 1, //1
  ICN2053_CFG2_BE_B = 1, //1
  //уровни яркости - получены эксперементально
  ICN2053_CFG2_BRIGHT_MASK = 0x03E0, //(0..15): 1..16
  ICN2053_CFG2_BRIGHT_OFFSET = 5,
  ICN2053_CFG2_BRIGHT = 14,//0x0380, 

  ICN2053_CFG2_R = (ICN2053_CFG2_BL_R << 10) + (ICN2053_CFG2_CURRENT_R << 1) + (ICN2053_CFG2_BE_R << 0) + (ICN2053_CFG2_BRIGHT << 5),
  ICN2053_CFG2_G = (ICN2053_CFG2_BL_G << 10) + (ICN2053_CFG2_CURRENT_G << 1) + (ICN2053_CFG2_BE_G << 0) + (ICN2053_CFG2_BRIGHT << 5),
  ICN2053_CFG2_B = (ICN2053_CFG2_BL_B << 10) + (ICN2053_CFG2_CURRENT_B << 1) + (ICN2053_CFG2_BE_B << 0) + (ICN2053_CFG2_BRIGHT << 5),
};

//биты регистра CFG3 для ICN2053, взято с LEDVISON
enum{
  ICN2053_CFG3_LGWB_MASK = 0x00f0, //low gray white balance (0..15): 15..0
  ICN2053_CFG3_LGWB_OFFSET = 4,
  ICN2053_CFG3_LGWB_R = 4, //4
  ICN2053_CFG3_LGWB_G = 4, //4
  ICN2053_CFG3_LGWB_B = 4, //4

  ICN2053_CFG3_BLE_MASK = 0x0004, //blanking level enable (0..1): 0,1
  ICN2053_CFG3_BLE_OFFSET = 2,
  ICN2053_CFG3_BLE_R = 1, //1
  ICN2053_CFG3_BLE_G = 1, //1
  ICN2053_CFG3_BLE_B = 1, //1

  ICN2053_CFG3_BASE = 0x04003, //взято с LEDVISION

  ICN2053_CFG3_R = (ICN2053_CFG3_LGWB_R << 4) + (ICN2053_CFG3_BLE_R << 2) + ICN2053_CFG3_BASE,
  ICN2053_CFG3_G = (ICN2053_CFG3_LGWB_G << 4) + (ICN2053_CFG3_BLE_G << 2) + ICN2053_CFG3_BASE,
  ICN2053_CFG3_B = (ICN2053_CFG3_LGWB_B << 4) + (ICN2053_CFG3_BLE_B << 2) + ICN2053_CFG3_BASE,
};

//биты регистра CFG4 для ICN2053, взято с LEDVISON
enum{
  ICN2053_CFG4_LGWBE_MASK = 0x4000, //low gray white balance enable(0..1): 0,1
  ICN2053_CFG4_LGWBE_OFFSET = 14,
  ICN2053_CFG4_LGWBE_R = 0, //0
  ICN2053_CFG4_LGWBE_G = 0, //0
  ICN2053_CFG4_LGWBE_B = 0, //0

  ICN2053_CFG4_FLO_MASK = 0x0070, //first line optimization (0..4): 0,4,5,6,7
  ICN2053_CFG4_FLO_OFFSET = 4,
  ICN2053_CFG4_FLO_R = 4, //4
  ICN2053_CFG4_FLO_G = 4, //4
  ICN2053_CFG4_FLO_B = 4, //4

  ICN2053_CFG4_BASE = 0x0e00, //взято с LEDVISION

  ICN2053_CFG4_R = (ICN2053_CFG4_LGWBE_R << 14) + (ICN2053_CFG4_FLO_R << 4) + ICN2053_CFG4_BASE,
  ICN2053_CFG4_G = (ICN2053_CFG4_LGWBE_G << 14) + (ICN2053_CFG4_FLO_G << 4) + ICN2053_CFG4_BASE,
  ICN2053_CFG4_B = (ICN2053_CFG4_LGWBE_B << 14) + (ICN2053_CFG4_FLO_B << 4) + ICN2053_CFG4_BASE,
};

//индексы массивов данных для регистров для ICN2053
enum{
  ICN2053_CFG1 = 0,
  ICN2053_CFG2,
  ICN2053_CFG3,
  ICN2053_CFG4,
  ICN2053_DBG,
  ICN2053_REG_CNT
};
//массив команд записи в регистры
extern const uint8_t ICN2053_REG_CMD[ICN2053_REG_CNT];
//массив значений для команд записи в регистры
extern const driver_rgb_t ICN2053_REG_VALUE[ICN2053_REG_CNT];

//------------------------------------ ICN1065 ---------------------------------------
// ICN1065 command lengths encoded by LAT pulse width
enum {
  ICN1065_V_SYNC = 3,
  ICN1065_PRE_ACT1 = 11,
  ICN1065_PRE_ACT2 = 14,
  ICN1065_REG_LATCH = 5,
  ICN1065_CLK_PAD = 8
};

// ICN1065 configuration table
enum {
  ICN1065_CFG_SCAN = 2,
  ICN1065_REG_CNT = 38
};
extern const driver_rgb_t ICN1065_REG_VALUE[ICN1065_REG_CNT];

// ICN1065 register write wrappers
enum {
  ICN1065_WRAPPER_0 = 0x00aa,
  ICN1065_WRAPPER_1 = 0x01aa,
  ICN1065_WRAPPER_2 = 0x0055,
  ICN1065_WRAPPER_3 = 0x0155
};

//длина начла префикса вертикальной синхронизации и обновления регистров конфигурации для ICN2053
enum{
  ICN2053_VSYNC_LEN = ((ICN2053_CMD_DELAY + 
    ICN2053_PRE_ACT + ICN2053_CMD_DELAY + ICN2053_EN_OP + ICN2053_CMD_DELAY + 
    ICN2053_V_SYNC + ICN2053_CMD_DELAY + 1)/2)*2,
  ICN2053_PREFIX_START_LEN = ((ICN2053_VSYNC_LEN + ICN2053_PRE_ACT + ICN2053_CMD_DELAY + 1)/2)*2,
};

// ICN1065: quiet region inserted into the prefix BEFORE the vsync train, so row
// scanning is paused (~141 µs at 5 MHz) while the vsync/pre-active command train
// latches into the driver chips — matching both the DMD_STM32 reference
// (stop_GCLK before load_config_regs) and a working ICND1065L receiver card
// (captured via logic analyzer), which also pause scanning around the train.
// Edge-count neutral: no OE pulses are added or removed, only all-zero idle
// words inserted, so the chip's row pointer can't drift. Set to 0 to disable.
#define ICN1065_PRE_VSYNC_QUIET 704

//length of the start of the vertical synchronization prefix and update of the configuration registers for ICN1065
enum {
  ICN1065_VSYNC_LEN = ((ICN1065_V_SYNC + ICN1065_CLK_PAD +
                        ICN1065_PRE_ACT1 + ICN1065_CLK_PAD +
                        ICN1065_PRE_ACT2 + ICN1065_CLK_PAD + 1) / 2) * 2,
  // the quiet region sits BEFORE the vsync train inside the prefix; register
  // writes and the prefix OE pattern start after quiet + train
  ICN1065_PREFIX_START_LEN = ICN1065_PRE_VSYNC_QUIET + ICN1065_VSYNC_LEN
};

//индексы переходных дескрипторов для ICN2053
enum{
  ICN2053_EXT_PREFIX = 0,
  ICN2053_EXT_DATA,
  ICN2053_DESCEXT_CNT,
};

enum{ICN2038_PREFIX_CNT = 0}; //число DMA буферов префиксов
enum{ICN2038_SUFFIX_CNT = 2}; //число DMA буферов суффиксов
enum{ICN2038_DSUFFIX_CNT = 2}; //число наборов DMA дескрипторов на суффикс

//именованные константы для ICN2053
enum{ICN2053_PREFIX_CNT = 1}; //число DMA буферов префиксов
enum{ICN2053_SUFFIX_CNT = 1}; //число DMA буферов суффиксов
enum{ICN2053_DSUFFIX_CNT = 2}; //число наборов DMA дескрипторов на суффикс
//теоретически при двойной буферизации DMA вывода чем больше строк тем быстрее вывод кадра (меньше надо ждать циклов окончания регенерации), 
//но при этом больше потребление памяти

enum{ICN2053_ROW_OE_CNT = 138}; //число OE импульсов для переключения строки
enum{ICN2053_ROW_OE_ADD_LEN = 30}; //пауза между строчными импульсами OE (для 138 - опциональна, для 595 - нужна)
enum{ICN2053_ROW_OE_LEN = ICN2053_ROW_OE_CNT*2 + ICN2053_ROW_OE_ADD_LEN}; //общая длина в такатах на одну строку OE (*2 - подъем+спад)

// Widen every ROW/OE scan pulse by this many extra ticks after its rising edge
// (contiguous high → no new edge; edge count and positions unchanged, so the
// chip's row pointer can't desync). The stock pulse is only 1 tick wide.
//
// SUPERSEDED as of July 2026 (per board707, github.com/board707/DMD_STM32#217,
// logic-analyzer capture against the datasheet): the real spec is a
// CLOCK-CYCLE COUNT, not an absolute-time target — every regular row-address
// OE pulse must be ICN1065_OE_LEN_REGULAR (4) clocks wide, independent of
// clk_freq. EXTRA is fixed at 3 (base 1 tick + 3 = 4) regardless of clock
// speed. See ICN1065_OE_LEN_RESTART below for the address-0/scan-restart
// pulse, which needs a DIFFERENT width (12 clocks) — handled separately by
// icn1065WidenRestartOEPulse() in the .cpp.
// Set to 0 for the original 1-tick pulse.
#define ICN1065_OE_PULSE_EXTRA 3

// Explicit clock-cycle-count targets (per board707's logic-analyzer read of the
// datasheet, github.com/board707/DMD_STM32#217): the OE pulse marking a scan
// restart (address wraps back to 0) must be 12 clocks; every other row-address
// OE pulse must be 4 clocks (== 1 base tick + ICN1065_OE_PULSE_EXTRA above).
// Only offset_prefix and offset_suffix currently get the restart-specific
// widening (icn1065WidenRestartOEPulse() in the .cpp) — both are generated
// starting fresh at address 0, so their first OE edge is always the restart
// pulse. NOT yet applied to the per-row data buffers, where a restart can land
// mid-buffer and there's no cheap way to know where without more plumbing.
#define ICN1065_OE_LEN_REGULAR 4
#define ICN1065_OE_LEN_RESTART 12

// board707 (github.com/board707/DMD_STM32#217) asked why pixel data appears to
// go out before the config/vsync train in a logic capture. This codebase's
// normal commit order is data-first, vsync-last (shift, then latch — see
// sendCallback() in icn2053.cpp).
//
// v1 (ADDITIVE, superseded): fired an EXTRA vsync/config send before data,
// keeping the normal trailing vsync too — two full config-train events per
// commit. Measurably reduced shadow-trail on continuous commits (scrolling),
// but caused a real, visible blank on sparse commits (this repo's once-per-
// second stopwatch: text flashes on then goes black every update). Even
// switching the stopwatch to commit continuously only reduced the flicker,
// never eliminated it — consistent with TWO glitch-causing config events per
// commit rather than one.
//
// v2 (TRUE REORDER, current): fires vsync/config before data via sendVsync(),
// then sends data via sendFrame(false, false) with autoVsync=false to SKIP
// the normal trailing vsync. One config-train event per commit, same as
// stock — just relocated to before the data instead of after. This still
// answers board707's original question (config genuinely precedes data), but
// without doubling the per-commit config-event count. Tradeoff: data shifted
// in during a commit isn't latched until the NEXT commit's leading vsync (a
// one-commit-late latch) — imperceptible in a continuous commit stream,
// unconfirmed whether it matters for very sparse/one-shot commits.
// UNTESTED ON HARDWARE as of this edit — see loop()'s continuous-commit
// stopwatch for the test case. Set to 0 to disable (stock: data-first,
// single trailing vsync, no reorder at all).
#define ICN1065_VSYNC_BEFORE_DATA 1

//named constants for ICN1065
enum{ICN1065_PREFIX_CNT = 1}; //number of DMA prefix buffers
enum{ICN1065_SUFFIX_CNT = 1}; //number of DMA suffix buffers
enum{ICN1065_DSUFFIX_CNT = 2}; //number of sets of DMA descriptors per suffix
//for ICN1065 8S panels often requires two OE pulses per address position
//diagnostic: 1 OE pulse to test row-group pairing; total window = 1*2 + 126 = 128 ticks
enum{ICN1065_ROW_OE_CNT = 1};
enum{ICN1065_ROW_OE_ADD_LEN = 126};
enum{ICN1065_ROW_OE_LEN = ICN1065_ROW_OE_CNT*2 + ICN1065_ROW_OE_ADD_LEN};

//функции заполнения значения регистра конфигурации на 1 и 3 канала
void setDriverReg(driver_reg_t& driver_reg, driver_reg_t value, driver_reg_t mask, uint8_t offset);
void setDriverRegRGB(driver_rgb_t* driver_reg, driver_reg_t value, driver_reg_t mask, uint8_t offset);

#endif
