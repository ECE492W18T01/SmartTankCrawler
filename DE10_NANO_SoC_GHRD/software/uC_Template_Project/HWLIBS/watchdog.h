#ifndef  __WATCHDOG__
#define  __WATCHDOG__
#include  <lib_def.h>

#define  WDT1_BASE  (*((CPU_REG32 *)0xFFD03000))

#define WDT1_CR (*(( CPU_REG32 *) (WDT1_BASE + 0x0)))
#define WDT1_TORR (*(( CPU_REG32 *) (WDT1_BASE + 0x4)))
#define WDT1_CRR (*(( CPU_REG32 *) (WDT1_BASE + 0xC)))
#define WDT_INTERUPT_MODE DEF_BIT_01 // or with this
#define WDT_SYSRESET_MODE ~DEF_BIT_01 //and with this

#define WDT_ENABLE DEF_BIT_00 // or with this
#define WDT_DISABLE ~DEF_BIT_00 // and with this

#define WTD_TIMEOUR_INIT_CLEAR (~DEF_BIT_07 & ~DEF_BIT_06 & ~DEF_BIT_05 & ~DEF_BIT_04) // and with this
#define WTD_TIMEOUT_INIT  (0x5 << 4) // // (1/osc1_clk) * 2 ^ (16 + WTD_TIMEOUT_INIT) = time to bite, or with this
#define WTD_TIMEOUT_RESET_CLEAR   (~DEF_BIT_03 & ~DEF_BIT_02 & ~DEF_BIT_01 & ~DEF_BIT_00) // and with this
#define WTD_TIMEOUT_RESET  (0x5) // (1/osc1_clk) * 2 ^ (16 + WTD_TIMEOUT_RESET) = time to bite, or with this

#define WD_DBG_PAUSE_REG  (*((CPU_REG32 *)0xFFD08010))
#define WD_DBG_PAUSE_REG_CLEAR (~DEF_BIT_03 & ~DEF_BIT_02)

void init_communication_watchdog();
void enable_communication_watchdog();
void disable_communication_watchdog();
void reset_communication_watchdog();
void WTD1_ISR_handeler(CPU_INT32U cpu_id);

#endif
