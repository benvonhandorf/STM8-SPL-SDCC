; STM8_TSL_RC_ACQUISITION.ASM generated from: ..\..\..\..\..\LIBRARIES\STM8S_TOUCHSENSING_DRIVER\SRC\STM8_TSL_RC_ACQUISITION.C


$MODESTM8 ; STM8 code
;$@$ROMMODEL(LARGE)
NAME    STM8_TSL_RC_ACQUISITION

?D0?STM8_TSL_RC_ACQUISITION            SEGMENT PAGE0 INSECTION0
?ZD0?STM8_TSL_RC_ACQUISITION           SEGMENT ZDATA0 INSECTION0
?DT?STM8_TSL_RC_ACQUISITION            SEGMENT DATA INSECTION0
?ZDT?STM8_TSL_RC_ACQUISITION           SEGMENT ZDATA INSECTION0
?PR?STM8_TSL_RC_ACQUISITION            SEGMENT CODE INSECTION0
?FC??TSL_IO_SW_BURST_TESTSYNCSHIFT?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_SW_BURST_WAIT_VIL?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_SW_BURST_WAIT_VIH?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_SW_SPREAD_SPECTRUM?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_INIT?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_CLAMP?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_SW_BURST_START_TIMER?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_SW_BURST_STOP_TIMER?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
?FC??TSL_IO_ACQUISITION?STM8_TSL_RC_ACQUISITION SEGMENT FCODE INSECTION
        EXTRN   PAGE0  (?BH)
        EXTRN   FCODE  (?TSL_IO_Clamp)
        EXTRN   DATA  (IT_Sync_Flags)
        EXTRN   PAGE0  (pKeyStruct)
        EXTRN   PAGE0  (?CH)
        EXTRN   PAGE0  (?CL)
        EXTRN   FCODE  (?C?mulu3232)
        EXTRN   FCODE  (?C?mv4_bc2isa)
        EXTRN   FCODE  (?C?mv4_isa2bc)
        EXTRN   FCODE  (?C?srl328)
        EXTRN   FCODE  (?C?mv4_pg2sk1)
        EXTRN   FCODE  (?C?mv4_ix2sk)
        EXTRN   FCODE  (?C?add3232)
        EXTRN   FCODE  (?C?mv4_bc2ix)
        EXTRN   FCODE  (?C?mv4_ix2bc)
        PUBLIC  Table_SCKEY_BITS
        PUBLIC  AcquisitionBitMask
        PUBLIC  SamplingShifter
        PUBLIC  CounterStop
        PUBLIC  sTouchIO
        PUBLIC  FinalMeasurementValue
        PUBLIC  ?TSL_IO_SW_Spread_Spectrum
        PUBLIC  ?TSL_IO_SW_Burst_Stop_Timer
        PUBLIC  ?TSL_IO_SW_Burst_Start_Timer
        PUBLIC  ?TSL_IO_SW_Burst_Wait_Vih
        PUBLIC  ?TSL_IO_SW_Burst_Wait_Vil
        PUBLIC  ?TSL_IO_SW_Burst_TestSyncShift
        PUBLIC  ?TSL_IO_Acquisition
        PUBLIC  ?TSL_IO_Clamp
        PUBLIC  ?TSL_IO_Init
         PUBLIC  _ICE_DUMMY_
          _ICE_DUMMY_	EQU   0

TIMACQ_CNTR        DATA  05328H

        RSEG  ?D0?STM8_TSL_RC_ACQUISITION
?D0?STM8_TSL_RC_ACQUISITION?BASE:

        RSEG  ?ZD0?STM8_TSL_RC_ACQUISITION
?ZD0?STM8_TSL_RC_ACQUISITION?BASE:
        FinalMeasurementValue: DS  004H
        sTouchIO: DS  00AH
        CounterStop: DS  002H
        SamplingShifter: DS  001H
        AcquisitionBitMask: DS  001H

        RSEG  ?DT?STM8_TSL_RC_ACQUISITION
?DT?STM8_TSL_RC_ACQUISITION?BASE:

        RSEG  ?ZDT?STM8_TSL_RC_ACQUISITION
?ZDT?STM8_TSL_RC_ACQUISITION?BASE:
        SpreadCounter: DS  001H

        RSEG  ?PR?STM8_TSL_RC_ACQUISITION
?PR?STM8_TSL_RC_ACQUISITION?BASE:
Table_SCKEY_BITS:
        DB  002H, 004H, 008H, 080H, 001H
; ; #endif
; ; 
; ; /* Includes ------------------------------------------------------------------*/
; ; #include "stm8_tsl_rc_acquisition.h"
; ; #include "stm8_tsl_services.h"
; ; 
; ; 
; ; /*#if defined(__RCSTM8__)
; ; #pragma INITSTATICVAR
; ; #endif*/
; ; 
; ; /* Memory section ------------------------------------------------------------*/
; ; /* Private typedef -----------------------------------------------------------*/
; ; /* Private define ------------------------------------------------------------*/
; ; /* Private macro -------------------------------------------------------------*/
; ; /* Private variables ---------------------------------------------------------*/
; ; /* Private function prototypes -----------------------------------------------*/
; ; /* Private functions ---------------------------------------------------------*/
; ; /* Public functions ----------------------------------------------------------*/
; ; #if defined(_COSMIC_) && defined(USE_PRAGMA_SECTION)
; ; 
; ; 
; ; /* Private typedef -----------------------------------------------------------*/
; ; /* Private define ------------------------------------------------------------*/
; ; /* Private macro -------------------------------------------------------------*/
; ; /* Private variables ---------------------------------------------------------*/
; ; u32 TINY FinalMeasurementValue;
; ; TOUCH_PORT_Info_T TINY sTouchIO;
; ; u16 TINY CounterStop;
; ; u8 TINY SamplingShifter;
; ; u8 TINY AcquisitionBitMask;
; ; static u8 SpreadCounter;
; ; 
; ; /* Private function prototypes -----------------------------------------------*/
; ; /* Private functions ---------------------------------------------------------*/
; ; /* Public functions ----------------------------------------------------------*/
; ; 
; ; /* This table is always defined */
; ; CONST u8 Table_SCKEY_BITS[
; ;   SCKEY_P1_KEY_COUNT
; ; #if NUMBER_OF_SINGLE_CHANNEL_PORTS > 1
; ;   + SCKEY_P2_KEY_COUNT
; ; #endif
; ; #if NUMBER_OF_SINGLE_CHANNEL_PORTS > 2
; ; ] =
; ;   {
; ;     SCKEY_P1_A /* Always defined */
; ; #if SCKEY_P1_KEY_COUNT > 1
; ;     , SCKEY_P1_B
; ; #endif
; ; #if SCKEY_P1_KEY_COUNT > 2
; ;     , SCKEY_P1_C
; ; #endif
; ; #if SCKEY_P1_KEY_COUNT > 3
; ;     , SCKEY_P1_D
; ; #endif
; ; #if SCKEY_P1_KEY_COUNT > 4
; ; #if SCKEY_P1_KEY_COUNT > 5
; ; #if SCKEY_P1_KEY_COUNT > 6
; ; #if SCKEY_P1_KEY_COUNT > 7
; ; 
; ; #if NUMBER_OF_SINGLE_CHANNEL_PORTS > 1
; ;     , SCKEY_P2_A
; ; #if SCKEY_P2_KEY_COUNT > 1
; ; #if SCKEY_P2_KEY_COUNT > 2
; ; #if SCKEY_P2_KEY_COUNT > 3
; ; #if SCKEY_P2_KEY_COUNT > 4
; ; #if SCKEY_P2_KEY_COUNT > 5
; ; #if SCKEY_P2_KEY_COUNT > 6
; ; #if SCKEY_P2_KEY_COUNT > 7
; ; #endif
; ; 
; ; #if NUMBER_OF_SINGLE_CHANNEL_PORTS > 2
; ;   };
; ; 
; ; #if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
; ; 
; ; #if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
; ; 
; ; 
; ; 
; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Execute a loop for calibrated number of cycles to allow over sampling
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************/
; ; void TSL_IO_SW_Burst_TestSyncShift(void)
; ; {
; ; 
; ; #if defined(_COSMIC_)
; ; #pragma ASM
; ;   ld a, SamplingShifter;// 3 cycles loop if jump executed; 2 cycles loop if not.
; ; ShiftLoopVih:
; ;   dec a;// 1 cycle
; ;   jrne ShiftLoopVih
; ; #pragma ENDASM
; ; #endif
; ; 
; ; }

        RSEG  ?FC??TSL_IO_SW_BURST_TESTSYNCSHIFT?STM8_TSL_RC_ACQUISITION
?TSL_IO_SW_Burst_TestSyncShift:
; #pragma asm
        ld a, SamplingShifter;// 3 cycles loop if jump executed; 2 cycles loop if not.
        ShiftLoopVih:
        dec a;// 1 cycle
        jrne ShiftLoopVih
; #pragma endasm
                        ;$@$C_ROUTINE(?TSL_IO_SW_Burst_TestSyncShift)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(270)
        RETF    

; END OF ?TSL_IO_SW_Burst_TestSyncShift
;; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Local sub function to wait for vil on RC structure.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************/
; ; void TSL_IO_SW_Burst_Wait_Vil(void)
; ; {
; ; 
; ; #if defined(_COSMIC_)
; ; #pragma ASM
; ;   ld a, AcquisitionBitMask
; ;   ldw x, sTouchIO   ;// Input data register ...
; ;   incw x
; ;   ;// Loop = 1 + 1 + 2 + 2 + 2 cycles = 8 cycles
; ; WaitForVil:
; ; // To be sure that the loop last 8 cycles the first instruction must be a 1-byte instruction
; ; // This is to be sure it is fully fetched in 1 cycle. The second instruction must be shorter than 4 bytes.
; ; // If this not the case, the code must be aligned.
; ;   bcp a, (x)  ;// 1 cycles
; ;   jreq EndWaitForVil
; ;   ldw y, TIMACQ_CNTR ;// 2 cycles; hw counter also used for timeout ...
; ;   cpw y, #0E00h    ;// 2 cycles; Timeout compare
; ;   jrult WaitForVil
; ; EndWaitForVil:
; ; #pragma ENDASM
; ; 
; ; #endif
; ; }

        RSEG  ?FC??TSL_IO_SW_BURST_WAIT_VIL?STM8_TSL_RC_ACQUISITION
?TSL_IO_SW_Burst_Wait_Vil:
; #pragma asm
        ld a, AcquisitionBitMask
        ldw x, sTouchIO   ;// Input data register ...
        incw x
        ;// Loop = 1 + 1 + 2 + 2 + 2 cycles = 8 cycles
        WaitForVil:
        
        
        
        bcp a, (x)  ;// 1 cycles
        jreq EndWaitForVil
        ldw y, TIMACQ_CNTR ;// 2 cycles; hw counter also used for timeout ...
        cpw y, #0E00h    ;// 2 cycles; Timeout compare
        jrult WaitForVil
        EndWaitForVil:
; #pragma endasm
                        ;$@$C_ROUTINE(?TSL_IO_SW_Burst_Wait_Vil)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(335)
        RETF    

; END OF ?TSL_IO_SW_Burst_Wait_Vil
;; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Local sub function to wait for vih on RC structure.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   *******************************************************************************/
; ; void TSL_IO_SW_Burst_Wait_Vih(void)
; ; {
; ; #if defined(_COSMIC_)
; ; #pragma ASM
; ;   ld a, AcquisitionBitMask
; ;   ldw x, sTouchIO   ;// Input data register ...
; ;   incw x
; ;   ;// Loop = 1 + 1 + 2 + 2 + 2 cycles = 8 cycles
; ; WaitForVih:
; ; // To be sure that the loop last 8 cycles the first instruction must be a 1-byte instruction
; ; // This is to be sure it is fully fetched in 1 cycle. The second instruction must be shorter than 4 bytes.
; ; // If this not the case, the code must be aligned.
; ;   bcp a, (x)  ;// 1 cycles
; ;   jrne EndWaitForVih
; ;   ldw y, TIMACQ_CNTR ;// 2 cycles; hw counter also used for timeout ...
; ;   cpw y, #0E00h    ;// 2 cycles; Timeout compare
; ;   jrult WaitForVih
; ; EndWaitForVih:
; ; 
; ; #pragma ENDASM
; ; 
; ; #endif
; ; }

        RSEG  ?FC??TSL_IO_SW_BURST_WAIT_VIH?STM8_TSL_RC_ACQUISITION
?TSL_IO_SW_Burst_Wait_Vih:
; #pragma asm
        ld a, AcquisitionBitMask
        ldw x, sTouchIO   ;// Input data register ...
        incw x
        ;// Loop = 1 + 1 + 2 + 2 + 2 cycles = 8 cycles
        WaitForVih:
        
        
        
        bcp a, (x)  ;// 1 cycles
        jrne EndWaitForVih
        ldw y, TIMACQ_CNTR ;// 2 cycles; hw counter also used for timeout ...
        cpw y, #0E00h    ;// 2 cycles; Timeout compare
        jrult WaitForVih
        EndWaitForVih:
        
; #pragma endasm
                        ;$@$C_ROUTINE(?TSL_IO_SW_Burst_Wait_Vih)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(400)
        RETF    

; END OF ?TSL_IO_SW_Burst_Wait_Vih
;; ; 
; ; 
; ; #if defined(_COSMIC_) && defined(USE_PRAGMA_SECTION)
; ; 
; ; #if SPREAD_SPECTRUM
; ; /**
; ;   ******************************************************************************
; ;   * @brief Spread Spectrum waiting routine. A variable delay is
; ;   * inserted between each set of 8 measures.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * SPREAD_SPECTRUM must be defined in the configuration file
; ;   ******************************************************************************
; ;   */
; ; 
; ; void TSL_IO_SW_Spread_Spectrum(void)
; ; {
; ;   u8 i;
; ; 
; ;   SpreadCounter++;
; ; 
; ;   if (SpreadCounter == SPREAD_COUNTER_MAX)
; ;   {
; ;     SpreadCounter = SPREAD_COUNTER_MIN;
; ;   }
; ; 
; ;   for (i = SpreadCounter; i; i--) {}}

        RSEG  ?FC??TSL_IO_SW_SPREAD_SPECTRUM?STM8_TSL_RC_ACQUISITION
?TSL_IO_SW_Spread_Spectrum:
                        ;$@$C_ROUTINE(?TSL_IO_SW_Spread_Spectrum)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(420)
        PUSH    A
                        ;$@$C_SOURCE_LINE(424)
        INC     SpreadCounter
                        ;$@$C_SOURCE_LINE(426)
        LD      A,#014H
        CP      A,SpreadCounter
        JRNE    ?NXT_0003
                        ;$@$C_SOURCE_LINE(428)
        MOV     SpreadCounter,#001H
?NXT_0003:
                        ;$@$C_SOURCE_LINE(431)
        LD      A,SpreadCounter
        LD      (001H,SP),A                        ; [ i ]
?FOR_0001:
        TNZ     (001H,SP)                          ; [ i ]
        JREQ    ?EPILOG_0003
        DEC     (001H,SP)                          ; [ i ]
        JRA     ?FOR_0001
?EPILOG_0003:
        POP     A
        RETF    

; END OF ?TSL_IO_SW_Spread_Spectrum
;; ; #endif /* SPREAD_SPECTRUM */
; ; 
; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Init for I/Os used in the application. Used for SW I/O toggling.
; ;   * If the hardware cell is used, the responsability of the I/O configuration is
; ;   * given to the user layer.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************
; ;   */
; ; void TSL_IO_Init(void)
; ; {
; ; 
; ;   // LOADREF pin
; ;   ((GPIO_TypeDef *)(LOADREF_PORT_ADDR))->CR1 |= LOADREF_BIT;
; ;   ((GPIO_TypeDef *)(LOADREF_PORT_ADDR))->DDR |= LOADREF_BIT;
; ;   ((GPIO_TypeDef *)(LOADREF_PORT_ADDR))->ODR &= (u8)(~LOADREF_BIT);
; ; 
; ;   // Single-Channel and Multi-channels pins
; ; #if GPIOA_ELECTRODES_MASK > 0
; ; #if GPIOB_ELECTRODES_MASK > 0
; ; #if GPIOC_ELECTRODES_MASK > 0
; ;   ((GPIO_TypeDef *)(GPIOC_BaseAddress))->CR1 |= GPIOC_ELECTRODES_MASK;
; ; #endif
; ; #if GPIOD_ELECTRODES_MASK > 0
; ;   ((GPIO_TypeDef *)(GPIOD_BaseAddress))->CR1 |= GPIOD_ELECTRODES_MASK;
; ; #endif
; ; #if GPIOE_ELECTRODES_MASK > 0
; ; #if GPIOF_ELECTRODES_MASK > 0
; ; #if GPIOG_ELECTRODES_MASK > 0
; ; #if GPIOH_ELECTRODES_MASK > 0
; ; #if GPIOI_ELECTRODES_MASK > 0
; ; 
; ;   TSL_IO_Clamp();
; ; 
; ;   /* Init Timer used to measure VIH/VIL */
; ;   TIMACQ->PSCR = 0;
; ;   TIMACQ->CR1 = 0x01;
; ; 
; ; }

        RSEG  ?FC??TSL_IO_INIT?STM8_TSL_RC_ACQUISITION
?TSL_IO_Init:
                        ;$@$C_ROUTINE(?TSL_IO_Init)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(451)
        BSET    0501CH,#004H
                        ;$@$C_SOURCE_LINE(452)
        BSET    0501BH,#004H
                        ;$@$C_SOURCE_LINE(453)
        BRES    05019H,#004H
                        ;$@$C_SOURCE_LINE(463)
        LD      A,#08EH
        OR      A,0500DH
        LD      0500DH,A
                        ;$@$C_SOURCE_LINE(466)
        LD      A,#07DH
        OR      A,05012H
        LD      05012H,A
                        ;$@$C_SOURCE_LINE(484)
        CALLF   ?TSL_IO_Clamp
                        ;$@$C_SOURCE_LINE(487)
        CLR     0532AH
                        ;$@$C_SOURCE_LINE(488)
        MOV     05320H,#001H
                        ;$@$C_SOURCE_LINE(490)
        RETF    

; END OF ?TSL_IO_Init
;; ; 
; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Put All Sensing I/Os in ouput mode at 0.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************
; ;   */
; ; void TSL_IO_Clamp(void)
; ; {
; ; 
; ;   // LOADREF pin
; ;   ((GPIO_TypeDef *)(LOADREF_PORT_ADDR))->ODR &= (u8)(~LOADREF_BIT);
; ; 
; ;   // Single-Channel and Multi-channels pins
; ; #if GPIOA_ELECTRODES_MASK > 0
; ; #if GPIOB_ELECTRODES_MASK > 0
; ; #if GPIOC_ELECTRODES_MASK > 0
; ;   ((GPIO_TypeDef *)(GPIOC_BaseAddress))->ODR &= (u8)(~GPIOC_ELECTRODES_MASK);
; ;   ((GPIO_TypeDef *)(GPIOC_BaseAddress))->DDR |= GPIOC_ELECTRODES_MASK;
; ; #endif
; ; #if GPIOD_ELECTRODES_MASK > 0
; ;   ((GPIO_TypeDef *)(GPIOD_BaseAddress))->ODR &= (u8)(~GPIOD_ELECTRODES_MASK);
; ;   ((GPIO_TypeDef *)(GPIOD_BaseAddress))->DDR |= GPIOD_ELECTRODES_MASK;
; ; #endif
; ; #if GPIOE_ELECTRODES_MASK > 0
; ; #if GPIOF_ELECTRODES_MASK > 0
; ; #if GPIOG_ELECTRODES_MASK > 0
; ; #if GPIOH_ELECTRODES_MASK > 0
; ; #if GPIOI_ELECTRODES_MASK > 0
; ; 
; ; }

        RSEG  ?FC??TSL_IO_CLAMP?STM8_TSL_RC_ACQUISITION
?TSL_IO_Clamp:
                        ;$@$C_ROUTINE(?TSL_IO_Clamp)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(507)
        BRES    05019H,#004H
                        ;$@$C_SOURCE_LINE(519)
        LD      A,#071H
        AND     A,0500AH
        LD      0500AH,A
                        ;$@$C_SOURCE_LINE(520)
        LD      A,#08EH
        OR      A,0500CH
        LD      0500CH,A
                        ;$@$C_SOURCE_LINE(523)
        LD      A,#082H
        AND     A,0500FH
        LD      0500FH,A
                        ;$@$C_SOURCE_LINE(524)
        LD      A,#07DH
        OR      A,05011H
        LD      05011H,A
                        ;$@$C_SOURCE_LINE(547)
        RETF    

; END OF ?TSL_IO_Clamp
;; ; 
; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Local sub function to restart timer for acquisition.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************
; ;   */
; ; void TSL_IO_SW_Burst_Start_Timer(void)
; ; {
; ; 
; ;   TIMACQ->EGR |= 0x01;
; ; 
; ; }

        RSEG  ?FC??TSL_IO_SW_BURST_START_TIMER?STM8_TSL_RC_ACQUISITION
?TSL_IO_SW_Burst_Start_Timer:
                        ;$@$C_ROUTINE(?TSL_IO_SW_Burst_Start_Timer)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(563)
        BSET    05324H,#000H
                        ;$@$C_SOURCE_LINE(565)
        RETF    

; END OF ?TSL_IO_SW_Burst_Start_Timer
;; ; 
; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Local sub function to get timer count for acquisition.
; ;   * @par Parameters:
; ;   * None
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************
; ;   */
; ; void TSL_IO_SW_Burst_Stop_Timer(void)
; ; {
; ; #if defined(_COSMIC_)
; ; #pragma ASM
; ;   ld a, TIMACQ_CNTR
; ;   ld CounterStop, a
; ;   ld a, TIMACQ_CNTR + 1
; ;   ld CounterStop + 1, a
; ; #pragma ENDASM
; ; 
; ; #endif
; ; }

        RSEG  ?FC??TSL_IO_SW_BURST_STOP_TIMER?STM8_TSL_RC_ACQUISITION
?TSL_IO_SW_Burst_Stop_Timer:
; #pragma asm
        ld a, TIMACQ_CNTR
        ld CounterStop, a
        ld a, TIMACQ_CNTR + 1
        ld CounterStop + 1, a
; #pragma endasm
                        ;$@$C_ROUTINE(?TSL_IO_SW_Burst_Stop_Timer)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(602)
        RETF    

; END OF ?TSL_IO_SW_Burst_Stop_Timer
;; ; 
; ; 
; ; /**
; ;   ******************************************************************************
; ;   * @brief Handles RC charge / discharge timing measurement.
; ;   * @param[in] AcqNumber Number of times the acquisition is done.
; ;   * @param[in] AdjustmentLevel Used to adjust the measured level.
; ;   * @retval void None
; ;   * @par Required preconditions:
; ;   * None
; ;   ******************************************************************************
; ;   */
; ; void TSL_IO_Acquisition(u8 AcqNumber, u8 AdjustmentLevel)
; ; {
; ; 
; ;   u16 MaxMeasurement, MinMeasurement, CumulatedMeasurement, Measurement;
; ;   u8 MeasRejected, RejectionCounter;
; ;   u8 AcqLoopIndex;
; ;   u32 tmpval;
; ; 
; ;   AcquisitionBitMask = sTouchIO.AcqMask;
; ; 
; ;   MinMeasurement = 0;
; ;   MaxMeasurement = 0;
; ;   FinalMeasurementValue = 0;
; ;   RejectionCounter = 0;
; ; 
; ;   /* Whole acquisition synchronisation */
; ;   /* The IT_Sync_Flag.start must be set to 1 inside an IT or it will loop forever */
; ; #if IT_SYNC
; ;   if (IT_Sync_Flags.one_acquisition_sync_enable)
; ;   {
; ;     IT_Sync_Flags.start = 0;
; ;     while (IT_Sync_Flags.start == 0);
; ;   }
; ; #endif
; ; 
; ; 
; ;   for (AcqLoopIndex = 0; AcqLoopIndex < AcqNumber; AcqLoopIndex++)
; ;   {
; ;     /* single measurement synchronisation */
; ;     /* The IT_Sync_Flag.start must be set to 1 inside an IT or it will loop forever */
; ; #if IT_SYNC
; ;     if (IT_Sync_Flags.one_measure_sync_enable)
; ;     {
; ;       IT_Sync_Flags.start = 0;
; ;       while (IT_Sync_Flags.start == 0);
; ;     }
; ; #endif
; ; 
; ;     do
; ;     {
; ; 
; ;       MeasRejected = 0;
; ;       CumulatedMeasurement = 0;
; ; 
; ;       for (SamplingShifter = SAMPLING_SHIFTER_LOOP_START;
; ;            SamplingShifter < (SAMPLING_SHIFTER_NB_LOOPS + SAMPLING_SHIFTER_LOOP_START);
; ;            SamplingShifter++)
; ;       {
; ;         /* VIH measurement */
; ;         disableInterrupts();
; ;         sTouchIO.PORT_ADDR->ODR &= (u8)(~sTouchIO.DriveMask);
; ;         sTouchIO.PORT_ADDR->DDR |= sTouchIO.DriveMask;
; ;         sTouchIO.PORT_ADDR->CR1 &= (u8)(~sTouchIO.DriveMask);
; ;         ((GPIO_TypeDef *)(LOADREF_PORT_ADDR))->ODR |= LOADREF_BIT;
; ;         enableInterrupts();
; ; 
; ;         /* Single charge Aquisition synchronisation */
; ;         /* The IT_Sync_Flag.start must be set to 1 inside an IT or it will loop forever */
; ; #if IT_SYNC
; ;         if (IT_Sync_Flags.one_charge_sync_enable)
; ;         {
; ;           IT_Sync_Flags.start = 0;
; ;           while (IT_Sync_Flags.start == 0);
; ;         }
; ; #endif
; ; 
; ; #if SPREAD_SPECTRUM
; ;         TSL_IO_SW_Spread_Spectrum();
; ; #endif
; ; 
; ;         disableInterrupts();
; ;         TSL_IO_SW_Burst_Start_Timer();
; ;         sTouchIO.PORT_ADDR->DDR &= (u8)(~sTouchIO.DriveMask);
; ;         TSL_IO_SW_Burst_TestSyncShift();
; ;         TSL_IO_SW_Burst_Wait_Vih();
; ;         TSL_IO_SW_Burst_Stop_Timer();
; ; 
; ;         Measurement = CounterStop;
; ; 
; ;         /* VIL measurement */
; ;         sTouchIO.PORT_ADDR->ODR |= sTouchIO.DriveMask;
; ;         sTouchIO.PORT_ADDR->DDR |= sTouchIO.DriveMask;
; ;         sTouchIO.PORT_ADDR->CR1 |= sTouchIO.DriveMask;
; ;         ((GPIO_TypeDef *)(LOADREF_PORT_ADDR))->ODR &= (u8)(~LOADREF_BIT);
; ;         enableInterrupts();
; ; 
; ;         /* Single charge Aquisition synchronization */
; ;         /* The IT_Sync_Flag.start must be set to 1 inside an IT or it will loop forever */
; ; #if IT_SYNC
; ;         if (IT_Sync_Flags.one_charge_sync_enable)
; ;         {
; ;           IT_Sync_Flags.start = 0;
; ;           while (IT_Sync_Flags.start == 0);
; ;         }
; ; #endif
; ; 
; ; #if SPREAD_SPECTRUM
; ;         TSL_IO_SW_Spread_Spectrum();
; ; #endif
; ; 
; ;         disableInterrupts();
; ;         TSL_IO_SW_Burst_Start_Timer();
; ;         sTouchIO.PORT_ADDR->CR1 &= (u8)(~sTouchIO.DriveMask);
; ;         sTouchIO.PORT_ADDR->DDR &= (u8)(~sTouchIO.DriveMask);
; ;         TSL_IO_SW_Burst_TestSyncShift();
; ;         TSL_IO_SW_Burst_Wait_Vil();
; ;         TSL_IO_SW_Burst_Stop_Timer();
; ; 
; ;         Measurement += CounterStop;
; ; 
; ;         CumulatedMeasurement += Measurement;
; ; 
; ;         /* Calculation of the min/max limits */
; ;         if (SamplingShifter == SAMPLING_SHIFTER_LOOP_START)
; ;         {
; ;           tmpval = (u32)((u32)Measurement * MAX_MEAS_COEFF);
; ;           MaxMeasurement = (u16)((u16)(tmpval >> 8) + NB_CYCLES_VIHVIL_LOOP);
; ;           tmpval = (u32)((u32)Measurement * MIN_MEAS_COEFF);
; ;           MinMeasurement = (u16)((u16)(tmpval >> 8) - NB_CYCLES_VIHVIL_LOOP);
; ;         }
; ;         else /* Compare measurement with min/max limit */
; ;         {
; ;           if ((Measurement < MinMeasurement) || (Measurement > MaxMeasurement))
; ;           {
; ;             MeasRejected++;
; ;             RejectionCounter++;
; ;             break; // Out from 'for SamplingShifter' loop !!!
; ;           }
; ;         }
; ; 
; ;       } /* for SamplingShifter... */
; ; 
; ;     }
; ;     while (MeasRejected && (RejectionCounter <= MAX_REJECTED_MEASUREMENTS));
; ; 
; ;     if (MeasRejected == 0)
; ;     {
; ;       FinalMeasurementValue += CumulatedMeasurement;
; ;     }
; ;     else // RejectionCounter > MAX_REJECTED_MEASUREMENTS
; ;     {
; ;       break; // Out from 'for AcqLoopIndex' loop !!!
; ;     }
; ; 
; ;   } /* for AcqLoopIndex... */
; ; 
; ;   TSL_IO_Clamp(); // To avoid consumption
; ;   enableInterrupts();
; ; 
; ;   *sTouchIO.RejectedNb = RejectionCounter;
; ; 
; ;   if (RejectionCounter <= MAX_REJECTED_MEASUREMENTS)
; ;   {
; ;     FinalMeasurementValue = (u32)(FinalMeasurementValue >> 3); /* Division by SAMPLING_SHIFTER_NB_LOOPS */
; ;     while (AdjustmentLevel--)
; ;     {
; ;       FinalMeasurementValue = (u32)(FinalMeasurementValue >> 1);
; ;     }
; ;     *sTouchIO.Measurement = (u16)FinalMeasurementValue;
; ;   }
; ;   else // Too many rejected measures...
; ;   {
; ; #if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
; ;       pKeyStruct->Setting.b.NOISE = 1; /* Warning: Application layer must reset this flag */
; ; #if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
; ;   }
; ; 
; ; }

        RSEG  ?FC??TSL_IO_ACQUISITION?STM8_TSL_RC_ACQUISITION
?TSL_IO_Acquisition:
                        ;$@$C_ROUTINE(?TSL_IO_Acquisition)
                        ;$@$C_SOURCE_FILE(..\..\..\..\..\libraries\stm8s_touchsensing_driver\src\stm8_tsl_rc_acquisition.c)
                        ;$@$C_SOURCE_LINE(615)
        PUSH    A
        SUB     SP,#00FH
                        ;$@$C_SOURCE_LINE(623)
        MOV     AcquisitionBitMask,sTouchIO + 02H
                        ;$@$C_SOURCE_LINE(625)
        CLRW    X
        LDW     (003H,SP),X                        ; [ MinMeasurement ]
                        ;$@$C_SOURCE_LINE(626)
        LDW     (001H,SP),X                        ; [ MaxMeasurement ]
                        ;$@$C_SOURCE_LINE(627)
        LDW     FinalMeasurementValue,X
        LDW     FinalMeasurementValue + 02H,X
                        ;$@$C_SOURCE_LINE(628)
        CLR     (00AH,SP)                          ; [ RejectionCounter ]
                        ;$@$C_SOURCE_LINE(633)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#000H,?NXT_0020
                        ;$@$C_SOURCE_LINE(635)
        BRES    IT_Sync_Flags,#3
?WHILE_0005:
                        ;$@$C_SOURCE_LINE(636)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#003H,?WHILE_0005
?NXT_0020:
                        ;$@$C_SOURCE_LINE(641)
        CLR     (00BH,SP)                          ; [ AcqLoopIndex ]
?FOR_0004:
        LD      A,(00BH,SP)                        ; [ AcqLoopIndex ]
        CP      A,(010H,SP)                        ; [ AcqNumber ]
        JRULT   ?LAB_0004
        JP      ?NXT_0021
?LAB_0004:
                        ;$@$C_SOURCE_LINE(646)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#001H,?DO_0001
                        ;$@$C_SOURCE_LINE(648)
        BRES    IT_Sync_Flags,#3
?WHILE_0006:
                        ;$@$C_SOURCE_LINE(649)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#003H,?WHILE_0006
?DO_0001:
                        ;$@$C_SOURCE_LINE(656)
        CLR     (009H,SP)                          ; [ MeasRejected ]
                        ;$@$C_SOURCE_LINE(657)
        CLRW    X
        LDW     (005H,SP),X                        ; [ CumulatedMeasurement ]
                        ;$@$C_SOURCE_LINE(659)
        MOV     SamplingShifter,#001H
?FOR_0005:
                        ;$@$C_SOURCE_LINE(664)
        SIM     
                        ;$@$C_SOURCE_LINE(665)
        LDW     X,sTouchIO
        LD      A,sTouchIO + 03H
        CPL     A
        AND     A,(X)
        LD      (X),A
                        ;$@$C_SOURCE_LINE(666)
        INCW    X
        INCW    X
        LD      A,(X)
        OR      A,sTouchIO + 03H
        LD      (X),A
                        ;$@$C_SOURCE_LINE(667)
        LDW     X,sTouchIO
        ADDW    X,#00003H
        LD      A,sTouchIO + 03H
        CPL     A
        AND     A,(X)
        LD      (X),A
                        ;$@$C_SOURCE_LINE(668)
        BSET    05019H,#004H
                        ;$@$C_SOURCE_LINE(669)
        RIM     
                        ;$@$C_SOURCE_LINE(674)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#002H,?NXT_0027
                        ;$@$C_SOURCE_LINE(676)
        BRES    IT_Sync_Flags,#3
?WHILE_0007:
                        ;$@$C_SOURCE_LINE(677)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#003H,?WHILE_0007
?NXT_0027:
                        ;$@$C_SOURCE_LINE(682)
        CALLF   ?TSL_IO_SW_Spread_Spectrum
                        ;$@$C_SOURCE_LINE(685)
        SIM     
                        ;$@$C_SOURCE_LINE(686)
        CALLF   ?TSL_IO_SW_Burst_Start_Timer
                        ;$@$C_SOURCE_LINE(687)
        LDW     X,sTouchIO
        INCW    X
        INCW    X
        LD      A,sTouchIO + 03H
        CPL     A
        AND     A,(X)
        LD      (X),A
                        ;$@$C_SOURCE_LINE(688)
        CALLF   ?TSL_IO_SW_Burst_TestSyncShift
                        ;$@$C_SOURCE_LINE(689)
        CALLF   ?TSL_IO_SW_Burst_Wait_Vih
                        ;$@$C_SOURCE_LINE(690)
        CALLF   ?TSL_IO_SW_Burst_Stop_Timer
                        ;$@$C_SOURCE_LINE(692)
        LDW     X,CounterStop
        LDW     (007H,SP),X                        ; [ Measurement ]
                        ;$@$C_SOURCE_LINE(695)
        LDW     X,sTouchIO
        LD      A,(X)
        OR      A,sTouchIO + 03H
        LD      (X),A
                        ;$@$C_SOURCE_LINE(696)
        INCW    X
        INCW    X
        LD      A,(X)
        OR      A,sTouchIO + 03H
        LD      (X),A
                        ;$@$C_SOURCE_LINE(697)
        LDW     X,sTouchIO
        ADDW    X,#00003H
        LD      A,(X)
        OR      A,sTouchIO + 03H
        LD      (X),A
                        ;$@$C_SOURCE_LINE(698)
        BRES    05019H,#004H
                        ;$@$C_SOURCE_LINE(699)
        RIM     
                        ;$@$C_SOURCE_LINE(704)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#002H,?NXT_0028
                        ;$@$C_SOURCE_LINE(706)
        BRES    IT_Sync_Flags,#3
?WHILE_0008:
                        ;$@$C_SOURCE_LINE(707)
        LD      A,IT_Sync_Flags
        BTJF    IT_Sync_Flags,#003H,?WHILE_0008
?NXT_0028:
                        ;$@$C_SOURCE_LINE(712)
        CALLF   ?TSL_IO_SW_Spread_Spectrum
                        ;$@$C_SOURCE_LINE(715)
        SIM     
                        ;$@$C_SOURCE_LINE(716)
        CALLF   ?TSL_IO_SW_Burst_Start_Timer
                        ;$@$C_SOURCE_LINE(717)
        LDW     X,sTouchIO
        ADDW    X,#00003H
        LD      A,sTouchIO + 03H
        CPL     A
        AND     A,(X)
        LD      (X),A
                        ;$@$C_SOURCE_LINE(718)
        LDW     X,sTouchIO
        INCW    X
        INCW    X
        LD      A,sTouchIO + 03H
        CPL     A
        AND     A,(X)
        LD      (X),A
                        ;$@$C_SOURCE_LINE(719)
        CALLF   ?TSL_IO_SW_Burst_TestSyncShift
                        ;$@$C_SOURCE_LINE(720)
        CALLF   ?TSL_IO_SW_Burst_Wait_Vil
                        ;$@$C_SOURCE_LINE(721)
        CALLF   ?TSL_IO_SW_Burst_Stop_Timer
                        ;$@$C_SOURCE_LINE(723)
        LDW     X,CounterStop
        ADDW    X,(007H,SP)                        ; [ Measurement ]
        LDW     (007H,SP),X                        ; [ Measurement ]
                        ;$@$C_SOURCE_LINE(725)
        ADDW    X,(005H,SP)                        ; [ CumulatedMeasurement ]
        LDW     (005H,SP),X                        ; [ CumulatedMeasurement ]
                        ;$@$C_SOURCE_LINE(728)
        LD      A,SamplingShifter
        DEC     A
        JRNE    ?ELSE_0014
                        ;$@$C_SOURCE_LINE(730)
        LDW     X,#00126H
        PUSHW   X
        CLRW    X
        PUSHW   X
        LDW     X,(00BH,SP)                        ; [ Measurement ]
        LDW     ?CH,X
        CLRW    X
        LDW     ?BH,X
        CALLF   ?C?mulu3232
        LD      A,#00BH
        CALLF   ?C?mv4_bc2isa
                        ;$@$C_SOURCE_LINE(731)
        LD      A,#008H
        CALLF   ?C?srl328
        LDW     X,?CH.w
        ADDW    X,#00008H
        LDW     (001H,SP),X                        ; [ MaxMeasurement ]
                        ;$@$C_SOURCE_LINE(732)
        CALLF   ?C?mv4_pg2sk1
?DATASTART_0001:
        DB      0D9H
?DATAEND_0001:
        LDW     X,(00BH,SP)                        ; [ Measurement ]
        LDW     ?CH,X
        CLRW    X
        LDW     ?BH,X
        CALLF   ?C?mulu3232
        LD      A,#00BH
        CALLF   ?C?mv4_bc2isa
                        ;$@$C_SOURCE_LINE(733)
        LD      A,#008H
        CALLF   ?C?srl328
        LDW     X,?CH.w
        ADDW    X,#0FFF8H
        LDW     (003H,SP),X                        ; [ MinMeasurement ]
        JRA     ?NXT_0026
?ELSE_0014:
                        ;$@$C_SOURCE_LINE(737)
        LDW     X,(007H,SP)                        ; [ Measurement ]
        CPW     X,(003H,SP)                        ; [ MinMeasurement ]
        JRULT   ?LAB_0002
        LDW     X,(001H,SP)                        ; [ MaxMeasurement ]
        CPW     X,(007H,SP)                        ; [ Measurement ]
        JRUGE   ?NXT_0026
?LAB_0002:
                        ;$@$C_SOURCE_LINE(739)
        INC     (009H,SP)                          ; [ MeasRejected ]
                        ;$@$C_SOURCE_LINE(740)
        INC     (00AH,SP)                          ; [ RejectionCounter ]
                        ;$@$C_SOURCE_LINE(741)
        JRA     ?TEST_0001
?NXT_0026:
                        ;$@$C_SOURCE_LINE(659)
        INC     SamplingShifter
        LD      A,#009H
        CP      A,SamplingShifter
        JRULE   ?LAB_0005
        JP      ?FOR_0005
?LAB_0005:
?TEST_0001:
        TNZ     (009H,SP)                          ; [ MeasRejected ]
        JREQ    ?NXT_0024
        LD      A,#005H
        CP      A,(00AH,SP)                        ; [ RejectionCounter ]
        JRULT   ?LAB_0006
        JP      ?DO_0001
?LAB_0006:
?NXT_0024:
                        ;$@$C_SOURCE_LINE(750)
        TNZ     (009H,SP)                          ; [ MeasRejected ]
        JRNE    ?NXT_0021
                        ;$@$C_SOURCE_LINE(752)
        LDW     X,#FinalMeasurementValue
        CALLF   ?C?mv4_ix2sk
        LDW     X,(009H,SP)                        ; [ CumulatedMeasurement ]
        LDW     ?CH,X
        CLRW    X
        LDW     ?BH,X
        CALLF   ?C?add3232
        LDW     X,#HIGH(FinalMeasurementValue)
        CALLF   ?C?mv4_bc2ix
                        ;$@$C_SOURCE_LINE(641)
        INC     (00BH,SP)                          ; [ AcqLoopIndex ]
        JP      ?FOR_0004
?NXT_0021:
                        ;$@$C_SOURCE_LINE(761)
        CALLF   ?TSL_IO_Clamp
                        ;$@$C_SOURCE_LINE(762)
        RIM     
                        ;$@$C_SOURCE_LINE(764)
        LD      A,(00AH,SP)                        ; [ RejectionCounter ]
        LD      [sTouchIO.w + 06H],A
                        ;$@$C_SOURCE_LINE(766)
        LD      A,#005H
        CP      A,(00AH,SP)                        ; [ RejectionCounter ]
        JRULT   ?ELSE_0017
                        ;$@$C_SOURCE_LINE(768)
        LDW     X,#FinalMeasurementValue
        CALLF   ?C?mv4_ix2bc
        LD      A,#003H
?WHILE_0009:
                        ;$@$C_SOURCE_LINE(769)
        CALLF   ?C?srl328
        LDW     X,#HIGH(FinalMeasurementValue)
        CALLF   ?C?mv4_bc2ix
        LD      A,(014H,SP)                        ; [ AdjustmentLevel ]
        DEC     (014H,SP)                          ; [ AdjustmentLevel ]
        TNZ     A
        JREQ    ?NXT_0033
                        ;$@$C_SOURCE_LINE(771)
        CALLF   ?C?mv4_ix2bc
        LD      A,#001H
        JRA     ?WHILE_0009
?NXT_0033:
                        ;$@$C_SOURCE_LINE(773)
        LDW     X,sTouchIO + 04H
        LDW     Y,FinalMeasurementValue + 02H
        LDW     (X),Y
        JRA     ?EPILOG_0008
?ELSE_0017:
                        ;$@$C_SOURCE_LINE(781)
        LDW     X,pKeyStruct
        INCW    X
        LD      A,(X)
        OR      A,#040H
        LD      (X),A
?EPILOG_0008:
                        ;$@$C_SOURCE_LINE(791)
        ADD     SP,#010H
        RETF    

; END OF ?TSL_IO_Acquisition
;; ; 
; ; /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
; 

         END
