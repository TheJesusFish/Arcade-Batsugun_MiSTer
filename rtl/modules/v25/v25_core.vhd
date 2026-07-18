-- SPDX-License-Identifier: BSD-3-Clause
--
-- Experimental NEC V25-compatible CPU core.
--
-- Scope of this first pass:
--   * V25-style reset vector state: PS=FFFFh, IP=0000h, active bank=7.
--   * 20-bit byte-wide external memory bus with a ready/valid handshake.
--   * A useful starter subset of 8086/V20 opcodes.
--   * V25 0Fh register/memory bit operations for TEST1/CLR1/SET1/NOT1.
--   * ModRM memory operands for MOV, TEST, and ALU byte/word forms.
--   * Segment override prefixes for ModRM memory operands.
--   * Segment register MOVs and DS0/DS1 far pointer loads.
--   * Segment PUSH/POP, immediate/rm stack ops, and direct/indirect far CALL/JMP/RET.
--   * PSW transfer/stack ops, including register-bank selection on POP PSW.
--   * RETI interrupt-return stack restore.
--   * BRK/BRKV software interrupt vector entry.
--   * DISPOSE frame tear-down.
--   * Group-3 TEST/NOT/NEG/MUL/IMUL/DIV/IDIV for register and memory operands.
--   * Group-2 shifts/rotates for register and memory operands.
--   * FE-group INC/DEC for byte register and memory operands.
--   * FF-group INC/DEC/CALL/JMP/PUSH for word register and memory operands.
--   * CBW and CWD sign extension.
--   * ADJ4A/ADJ4S/ADJBA/ADJBS and CVTBD/CVTDB decimal adjust/conversion.
--   * ROL4/ROR4 packed-BCD nibble rotate through AL low nibble.
--   * ADD4S/SUB4S/CMP4S packed-BCD string operations.
--   * LOOP/LOOPZ/LOOPNZ/JCXZ short branches.
--   * XLAT byte table lookup.
--   * Non-REP MOVSB/MOVSW with DS0 source, DS1 destination, and DF adjustment.
--   * Non-REP CMPS/MOVS/STOS/LODS/SCAS byte and word string ops.
--   * REP/REPE/REPNE prefixes for string ops.
--   * LOCK prefix consumed as a bus-lock no-op.
--   * Internal data/SFR window selected by IDB, including REGBNK mirroring.
--   * Relocated and fixed-address IDB aliases.
--   * PRC.RAMEN gating for normal CPU data access to lower internal RAM.
--   * FINT clears the least significant active service bit in the internal ISPR byte.
--   * POLL waits on P14/POLL when P14 is configured as an input/control pin.
--   * HLT and STOP enter distinct halted states with first-pass release rules.
--   * IN/OUT byte-wide external I/O bus cycles when IBRK is set.
--   * FPO1/FPO2 compatibility traps through vector 7.
--   * CHKIND bounds check with vector 5 on failure.
--   * V-series PUSH R / POP R register block stack operations.
--   * V-series 69h/6Bh signed immediate multiply.
--   * PREPARE frame setup and DISPOSE frame tear-down.
--   * Timer interval/one-shot and timebase request scaffolds.
--   * DMA memory-memory, one-transfer I/O, and demand-release I/O scaffolds.
--   * Serial transmit/receive-completion and overrun/framing/parity error request scaffolds.
--
-- Still deferred: cycle-exact full prefetch/queue timing, exact DMA bus ownership,
-- nanosecond-exact acknowledge/peripheral pin phase, and remaining exact
-- internal/peripheral timing refinements.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.v25_pkg.all;
use work.v25_timing_pkg.all;

entity v25_core is
  generic (
    RESET_PS   : word_t := x"FFFF";
    RESET_IP   : word_t := x"0000";
    RESET_BANK : natural range 0 to 7 := 7;
    ENABLE_TIMING_THROTTLE : boolean := false;
    FIXED_INSTRUCTION_BUDGET : natural range 0 to 31 := 0;
    ENABLE_PREFETCH_QUEUE : boolean := false
  );
  port (
    clk       : in  std_logic;
    reset_n   : in  std_logic;
    clock_enable : in std_logic := '1';

    mem_valid : out std_logic;
    mem_write : out std_logic;
    mem_addr  : out addr20_t;
    mem_wdata : out byte_t;
    mem_rdata : in  byte_t;
    mem_ready : in  std_logic;

    io_valid  : out std_logic;
    io_write  : out std_logic;
    io_addr   : out word_t;
    io_wdata  : out byte_t;
    io_rdata  : in  byte_t;
    io_ready  : in  std_logic;

    irq_request : in std_logic := '0';
    irq_vector  : in byte_t := x"00";
    int_ack_valid : out std_logic := '0';
    int_ack_second : out std_logic := '0';
    int_ack_ready : in std_logic := '1';
    int_ack_vector_data : in byte_t := x"00";
    int_ack_vector_valid : in std_logic := '0';
    nmi_in      : in std_logic := '0';
    intp0_in    : in std_logic := '0';
    intp1_in    : in std_logic := '0';
    intp2_in    : in std_logic := '0';

    serial0_rx_valid_in : in std_logic := '0';
    serial0_rx_data_in  : in byte_t := x"00";
    serial0_rx_frame_error_in : in std_logic := '0';
    serial0_rx_parity_error_in : in std_logic := '0';
    serial0_rxd_in      : in std_logic := '1';
    serial1_rx_valid_in : in std_logic := '0';
    serial1_rx_data_in  : in byte_t := x"00";
    serial1_rx_frame_error_in : in std_logic := '0';
    serial1_rx_parity_error_in : in std_logic := '0';
    serial1_rxd_in      : in std_logic := '1';

    port0_in : in byte_t := x"00";
    port1_in : in byte_t := x"00";
    port2_in : in byte_t := x"00";
    portt_in : in byte_t := x"00";
    port0_out : out byte_t := x"00";
    port1_out : out byte_t := x"00";
    port2_out : out byte_t := x"00";
    portt_out : out byte_t := x"00";
    port0_mode_control : out byte_t := x"00";
    port1_mode_control : out byte_t := x"00";
    port2_mode_control : out byte_t := x"00";
    timer0_tick_in : in std_logic := '1';
    timer0_md_tick_in : in std_logic := '1';
    timer1_tick_in : in std_logic := '1';
    tout_out       : out std_logic := '1';
    sfr_wait_states : out unsigned(3 downto 0) := (others => '0');
    sfr_ready_extend : out std_logic := '0';
    rfm_refresh_timing_in : in std_logic := '0';

    dma0_control : out byte_t := x"00";
    dma0_mode    : out byte_t := x"00";
    dma0_start   : out std_logic := '0';
    dma0_hold    : out std_logic := '0';
    dmarq0_in    : in std_logic := '0';
    dmaaak0_n_out : out std_logic := '1';
    tc0_n_out    : out std_logic := '1';
    dma1_control : out byte_t := x"00";
    dma1_mode    : out byte_t := x"00";
    dma1_start   : out std_logic := '0';
    dma1_hold    : out std_logic := '0';
    dmarq1_in    : in std_logic := '0';
    dmaaak1_n_out : out std_logic := '1';
    tc1_n_out    : out std_logic := '1';

    serial0_rxb  : out byte_t := x"00";
    serial0_txb  : out byte_t := x"00";
    serial0_mode : out byte_t := x"00";
    serial0_ctrl : out byte_t := x"00";
    serial0_brg  : out byte_t := x"00";
    serial0_enable : out byte_t := x"00";
    serial1_rxb  : out byte_t := x"00";
    serial1_txb  : out byte_t := x"00";
    serial1_mode : out byte_t := x"00";
    serial1_ctrl : out byte_t := x"00";
    serial1_brg  : out byte_t := x"00";
    serial1_enable : out byte_t := x"00";

    standby_control : out byte_t := x"00";
    ram_control     : out byte_t := x"00";
    flag_control    : out byte_t := x"00";
    protect_control : out byte_t := x"00";
    timebase_control : out byte_t := x"00";

    interrupt_mode : out byte_t := x"00";
    interrupt_pending : out byte_t := x"00";
    ext_irq0_control : out byte_t := x"00";
    ext_irq1_control : out byte_t := x"00";
    ext_irq2_control : out byte_t := x"00";
    timer0_irq_control : out byte_t := x"00";
    timer1_irq_control : out byte_t := x"00";
    timer2_irq_control : out byte_t := x"00";
    timer0_control : out byte_t := x"00";
    timer1_control : out byte_t := x"00";
    dma0_irq_control : out byte_t := x"00";
    dma1_irq_control : out byte_t := x"00";
    serial0_error_irq_control : out byte_t := x"00";
    serial0_rx_irq_control : out byte_t := x"00";
    serial0_tx_irq_control : out byte_t := x"00";
    serial1_error_irq_control : out byte_t := x"00";
    serial1_rx_irq_control : out byte_t := x"00";
    serial1_tx_irq_control : out byte_t := x"00";

    halted    : out std_logic;
    stop_mode : out std_logic := '0';
    fault     : out std_logic;

    debug_pc  : out addr20_t;
    debug_psw : out word_t;
    debug_ax  : out word_t;
    debug_cx  : out word_t;
    debug_dx  : out word_t;
    debug_bx  : out word_t;
    debug_sp  : out word_t
  );
end entity;

architecture rtl of v25_core is
  constant TIMING_COUNTER_MAX : natural := 4194303;
  constant PREFETCH_QUEUE_DEPTH : natural := 6;
  constant PREFETCH_FORCE_THRESHOLD : natural := 3;

  type state_t is (
    ST_FETCH_REQ,
    ST_FETCH_WAIT,
    ST_DECODE,
    ST_IMM_REQ,
    ST_IMM_WAIT,
    ST_MODRM_REQ,
    ST_MODRM_WAIT,
    ST_DISP_REQ,
    ST_DISP_WAIT,
    ST_EXECUTE,
    ST_MEM_RD_LO_REQ,
    ST_MEM_RD_LO_WAIT,
    ST_MEM_RD_HI_REQ,
    ST_MEM_RD_HI_WAIT,
    ST_MEM_WR_LO_REQ,
    ST_MEM_WR_LO_WAIT,
    ST_MEM_WR_HI_REQ,
    ST_MEM_WR_HI_WAIT,
    ST_BCD_SRC_RD_REQ,
    ST_BCD_SRC_RD_WAIT,
    ST_BCD_DST_RD_REQ,
    ST_BCD_DST_RD_WAIT,
    ST_BCD_WR_REQ,
    ST_BCD_WR_WAIT,
    ST_INT_VEC_IP_LO_REQ,
    ST_INT_VEC_IP_LO_WAIT,
    ST_INT_VEC_IP_HI_REQ,
    ST_INT_VEC_IP_HI_WAIT,
    ST_INT_VEC_PS_LO_REQ,
    ST_INT_VEC_PS_LO_WAIT,
    ST_INT_VEC_PS_HI_REQ,
    ST_INT_VEC_PS_HI_WAIT,
    ST_EXT_INT_ACK1_REQ,
    ST_EXT_INT_ACK_GAP,
    ST_EXT_INT_ACK2_REQ,
    ST_IO_RD_LO_REQ,
    ST_IO_RD_LO_WAIT,
    ST_IO_RD_HI_REQ,
    ST_IO_RD_HI_WAIT,
    ST_IO_WR_LO_REQ,
    ST_IO_WR_LO_WAIT,
    ST_IO_WR_HI_REQ,
    ST_IO_WR_HI_WAIT,
    ST_DMA_RD_REQ,
    ST_DMA_RD_WAIT,
    ST_DMA_WR_REQ,
    ST_DMA_WR_WAIT,
    ST_MACRO_RD_REQ,
    ST_MACRO_RD_WAIT,
    ST_MACRO_WR_REQ,
    ST_MACRO_WR_WAIT,
    ST_MACRO_FINISH,
    ST_FIELD_RD0_REQ,
    ST_FIELD_RD0_WAIT,
    ST_FIELD_RD1_REQ,
    ST_FIELD_RD1_WAIT,
    ST_FIELD_RD2_REQ,
    ST_FIELD_RD2_WAIT,
    ST_FIELD_RD3_REQ,
    ST_FIELD_RD3_WAIT,
    ST_FIELD_WR0_REQ,
    ST_FIELD_WR0_WAIT,
    ST_FIELD_WR1_REQ,
    ST_FIELD_WR1_WAIT,
    ST_FIELD_WR2_REQ,
    ST_FIELD_WR2_WAIT,
    ST_FIELD_WR3_REQ,
    ST_FIELD_WR3_WAIT,
    ST_PREPARE_COPY_RD_LO_REQ,
    ST_PREPARE_COPY_RD_LO_WAIT,
    ST_PREPARE_COPY_RD_HI_REQ,
    ST_PREPARE_COPY_RD_HI_WAIT,
    ST_MEM_RD_FAR_SEG_LO_REQ,
    ST_MEM_RD_FAR_SEG_LO_WAIT,
    ST_MEM_RD_FAR_SEG_HI_REQ,
    ST_MEM_RD_FAR_SEG_HI_WAIT,
    ST_PUSH_LO_REQ,
    ST_PUSH_LO_WAIT,
    ST_PUSH_HI_REQ,
    ST_PUSH_HI_WAIT,
    ST_PUSH_FAR_SEG_LO_REQ,
    ST_PUSH_FAR_SEG_LO_WAIT,
    ST_PUSH_FAR_SEG_HI_REQ,
    ST_PUSH_FAR_SEG_HI_WAIT,
    ST_POP_LO_REQ,
    ST_POP_LO_WAIT,
    ST_POP_HI_REQ,
    ST_POP_HI_WAIT,
    ST_POP_FAR_SEG_LO_REQ,
    ST_POP_FAR_SEG_LO_WAIT,
    ST_POP_FAR_SEG_HI_REQ,
    ST_POP_FAR_SEG_HI_WAIT,
    ST_POP_PSW_LO_REQ,
    ST_POP_PSW_LO_WAIT,
    ST_POP_PSW_HI_REQ,
    ST_POP_PSW_HI_WAIT,
    ST_POLL_WAIT,
    ST_HALTED,
    ST_FAULT
  );

  type op_kind_t is (
    OP_NONE,
    OP_MOV_R16_IMM,
    OP_MOV_R8_IMM,
    OP_ALU_AX_IMM16,
    OP_ALU_AL_IMM8,
    OP_ADJ4A,
    OP_ADJ4S,
    OP_ADJBA,
    OP_ADJBS,
    OP_CVTBD,
    OP_CVTDB,
    OP_TEST_AL_IMM8,
    OP_TEST_AX_IMM16,
    OP_INC_R16,
    OP_DEC_R16,
    OP_GRP_FE,
    OP_INC_RM8,
    OP_DEC_RM8,
    OP_INC_RM16,
    OP_DEC_RM16,
    OP_JMP_REL8,
    OP_JMP_REL16,
    OP_JMP_FAR_IMM,
    OP_JMP_RM16,
    OP_JMP_M32,
    OP_JCC_REL8,
    OP_LOOP_REL8,
    OP_MOV_RM8_R8,
    OP_MOV_RM16_R16,
    OP_MOV_R8_RM8,
    OP_MOV_R16_RM16,
    OP_MOV_RM8_IMM8,
    OP_MOV_RM16_IMM16,
    OP_MOV_RM16_SREG,
    OP_MOV_SREG_RM16,
    OP_MOV_DS0_R16_MEM32,
    OP_MOV_DS1_R16_MEM32,
    OP_ALU_RM8_R8,
    OP_ALU_RM16_R16,
    OP_ALU_R8_RM8,
    OP_ALU_R16_RM16,
    OP_TEST_RM8_R8,
    OP_TEST_RM16_R16,
    OP_MOV_AL_MOFFS,
    OP_MOV_AX_MOFFS,
    OP_MOV_MOFFS_AL,
    OP_MOV_MOFFS_AX,
    OP_LEA_R16_MEM,
    OP_XLAT,
    OP_MOVS8,
    OP_MOVS16,
    OP_CMPS8,
    OP_CMPS16,
    OP_CMPS8_DST,
    OP_CMPS16_DST,
    OP_STOS8,
    OP_STOS16,
    OP_LODS8,
    OP_LODS16,
    OP_SCAS8,
    OP_SCAS16,
    OP_XCHG_AX_R16,
    OP_XCHG_RM8_R8,
    OP_XCHG_RM16_R16,
    OP_IMUL_R16_RM16_IMM16,
    OP_IMUL_R16_RM16_IMM8,
    OP_GRP_IMM8_RM8,
    OP_GRP_IMM16_RM16,
    OP_GRP_IMM8_RM16_SIGN,
    OP_GRP3_RM8,
    OP_GRP3_RM16,
    OP_GRP_SHIFT_RM8_1,
    OP_GRP_SHIFT_RM16_1,
    OP_GRP_SHIFT_RM8_CL,
    OP_GRP_SHIFT_RM16_CL,
    OP_GRP_SHIFT_RM8_IMM,
    OP_GRP_SHIFT_RM16_IMM,
    OP_PUSH_R16,
    OP_POP_R16,
    OP_PUSH_SREG,
    OP_POP_SREG,
    OP_PUSH_PSW,
    OP_POP_PSW,
    OP_MOV_PSW_AH,
    OP_PUSH_REGS,
    OP_POP_REGS,
    OP_PUSH_RM16,
    OP_PUSH_IMM16,
    OP_PUSH_IMM8_SIGN,
    OP_POP_RM16,
    OP_PREPARE,
    OP_DISPOSE,
    OP_CALL_REL16,
    OP_CALL_FAR_IMM,
    OP_CALL_RM16,
    OP_CALL_M32,
    OP_RET_NEAR,
    OP_RET_NEAR_IMM,
    OP_RET_FAR,
    OP_RET_FAR_IMM,
    OP_RETI,
    OP_BRK,
    OP_BRKV,
    OP_BRKCS,
    OP_RETRBI,
    OP_BTCLR,
    OP_IN_AL_IMM8,
    OP_IN_AX_IMM8,
    OP_OUT_IMM8_AL,
    OP_OUT_IMM8_AX,
    OP_IN_AL_DX,
    OP_IN_AX_DX,
    OP_OUT_DX_AL,
    OP_OUT_DX_AX,
    OP_INM8,
    OP_INM16,
    OP_OUTM8,
    OP_OUTM16,
    OP_FPO1,
    OP_FPO2,
    OP_CHKIND,
    OP_INS_FIELD,
    OP_EXT_FIELD,
    OP_GRP_FF,
    OP_V25_PREFIX,
    OP_V25_BITOP,
    OP_ROL4,
    OP_ROR4,
    OP_MOVSPA,
    OP_MOVSPB,
    OP_TSKSW,
    OP_ADD4S,
    OP_SUB4S,
    OP_CMP4S
  );

  type alu_func_t is (
    ALU_ADD,
    ALU_OR,
    ALU_ADC,
    ALU_SBB,
    ALU_AND,
    ALU_SUB,
    ALU_XOR,
    ALU_CMP
  );

  type seg_select_t is (
    SEG_DEFAULT,
    SEG_PS,
    SEG_SS,
    SEG_DS0,
    SEG_DS1
  );

  type push_mode_t is (
    PUSH_ONLY,
    PUSH_REGS,
    PUSH_PREPARE,
    PUSH_PREPARE_COPY,
    PUSH_PREPARE_TEMP,
    PUSH_INTERRUPT_PSW,
    PUSH_INTERRUPT_PS,
    PUSH_INTERRUPT_PC,
    PUSH_THEN_JUMP,
    PUSH_FAR_THEN_JUMP
  );
  type pop_mode_t is (
    POP_TO_REG,
    POP_TO_MEM,
    POP_TO_PSW,
    POP_TO_REGS,
    POP_TO_IP,
    POP_TO_IP_ADJ,
    POP_TO_SREG,
    POP_TO_IP_FAR,
    POP_TO_IP_FAR_ADJ,
    POP_TO_IP_FAR_PSW
  );
  type rep_mode_t is (REP_NONE, REP_NE, REP_E, REP_NC, REP_C);
  type shift8_result_t is record
    value : byte_t;
    flags : v25_flags_t;
  end record;
  type shift16_result_t is record
    value : word_t;
    flags : v25_flags_t;
  end record;
  type bcd_byte_result_t is record
    value : byte_t;
    carry : std_logic;
  end record;
  type bank_word_t is array (0 to 7) of word_t;
  type internal_data_t is array (0 to 511) of byte_t;
  type prefetch_byte_queue_t is array (0 to PREFETCH_QUEUE_DEPTH - 1) of byte_t;
  type prefetch_word_queue_t is array (0 to PREFETCH_QUEUE_DEPTH - 1) of word_t;

  constant SFR_P0_INDEX    : natural := 16#100#;
  constant SFR_PM0_INDEX   : natural := 16#101#;
  constant SFR_PMC0_INDEX  : natural := 16#102#;
  constant SFR_P1_INDEX    : natural := 16#108#;
  constant SFR_PM1_INDEX   : natural := 16#109#;
  constant SFR_PMC1_INDEX  : natural := 16#10A#;
  constant SFR_P2_INDEX    : natural := 16#110#;
  constant SFR_PM2_INDEX   : natural := 16#111#;
  constant SFR_PMC2_INDEX  : natural := 16#112#;
  constant SFR_PT_INDEX    : natural := 16#138#;
  constant SFR_PMT_INDEX   : natural := 16#13B#;
  constant SFR_INTM_INDEX  : natural := 16#140#;
  constant SFR_EMS0_INDEX  : natural := 16#144#;
  constant SFR_EMS1_INDEX  : natural := 16#145#;
  constant SFR_EMS2_INDEX  : natural := 16#146#;
  constant SFR_EXIC0_INDEX : natural := 16#14C#;
  constant SFR_EXIC1_INDEX : natural := 16#14D#;
  constant SFR_EXIC2_INDEX : natural := 16#14E#;
  constant SFR_RXB0_INDEX  : natural := 16#160#;
  constant SFR_TXB0_INDEX  : natural := 16#162#;
  constant SFR_SRMS0_INDEX : natural := 16#165#;
  constant SFR_STMS0_INDEX : natural := 16#166#;
  constant SFR_SCM0_INDEX  : natural := 16#168#;
  constant SFR_SCC0_INDEX  : natural := 16#169#;
  constant SFR_BRG0_INDEX  : natural := 16#16A#;
  constant SFR_SCE0_INDEX  : natural := 16#16B#;
  constant SFR_SEIC0_INDEX : natural := 16#16C#;
  constant SFR_SRIC0_INDEX : natural := 16#16D#;
  constant SFR_STIC0_INDEX : natural := 16#16E#;
  constant SFR_RXB1_INDEX  : natural := 16#170#;
  constant SFR_TXB1_INDEX  : natural := 16#172#;
  constant SFR_SRMS1_INDEX : natural := 16#175#;
  constant SFR_STMS1_INDEX : natural := 16#176#;
  constant SFR_SCM1_INDEX  : natural := 16#178#;
  constant SFR_SCC1_INDEX  : natural := 16#179#;
  constant SFR_BRG1_INDEX  : natural := 16#17A#;
  constant SFR_SCE1_INDEX  : natural := 16#17B#;
  constant SFR_SEIC1_INDEX : natural := 16#17C#;
  constant SFR_SRIC1_INDEX : natural := 16#17D#;
  constant SFR_STIC1_INDEX : natural := 16#17E#;
  constant SFR_TM0_INDEX   : natural := 16#180#;
  constant SFR_MD0_INDEX   : natural := 16#182#;
  constant SFR_TM1_INDEX   : natural := 16#188#;
  constant SFR_MD1_INDEX   : natural := 16#18A#;
  constant SFR_TMC0_INDEX  : natural := 16#190#;
  constant SFR_TMC1_INDEX  : natural := 16#191#;
  constant SFR_TMMS0_INDEX : natural := 16#194#;
  constant SFR_TMMS1_INDEX : natural := 16#195#;
  constant SFR_TMMS2_INDEX : natural := 16#196#;
  constant SFR_TMIC0_INDEX : natural := 16#19C#;
  constant SFR_TMIC1_INDEX : natural := 16#19D#;
  constant SFR_TMIC2_INDEX : natural := 16#19E#;
  constant SFR_DMAC0_INDEX : natural := 16#1A0#;
  constant SFR_DMAM0_INDEX : natural := 16#1A1#;
  constant SFR_DMAC1_INDEX : natural := 16#1A2#;
  constant SFR_DMAM1_INDEX : natural := 16#1A3#;
  constant SFR_DIC0_INDEX  : natural := 16#1AC#;
  constant SFR_DIC1_INDEX  : natural := 16#1AD#;
  constant SFR_STBC_INDEX  : natural := 16#1E0#;
  constant SFR_RFM_INDEX   : natural := 16#1E1#;
  constant SFR_WTC_INDEX   : natural := 16#1E8#;
  constant SFR_FLAG_INDEX  : natural := 16#1EA#;
  constant SFR_PRC_INDEX   : natural := 16#1EB#;
  constant SFR_TBIC_INDEX  : natural := 16#1EC#;
  constant SFR_ISPR_INDEX  : natural := 16#1FC#;
  constant SFR_IDB_INDEX   : natural := 16#1FF#;

  constant IRQ_VEC_SERIAL0_ERROR : byte_t := x"0C";
  constant IRQ_VEC_NMI           : byte_t := x"02";
  constant IRQ_VEC_SERIAL0_RX    : byte_t := x"0D";
  constant IRQ_VEC_SERIAL0_TX    : byte_t := x"0E";
  constant IRQ_VEC_SERIAL1_ERROR : byte_t := x"10";
  constant IRQ_VEC_SERIAL1_RX    : byte_t := x"11";
  constant IRQ_VEC_SERIAL1_TX    : byte_t := x"12";
  constant IRQ_VEC_DMA0          : byte_t := x"14";
  constant IRQ_VEC_DMA1          : byte_t := x"15";
  constant IRQ_VEC_EX0           : byte_t := x"18";
  constant IRQ_VEC_EX1           : byte_t := x"19";
  constant IRQ_VEC_EX2           : byte_t := x"1A";
  constant IRQ_VEC_TIMER0        : byte_t := x"1C";
  constant IRQ_VEC_TIMER1        : byte_t := x"1D";
  constant IRQ_VEC_TIMER2        : byte_t := x"1E";
  constant IRQ_VEC_TIMEBASE      : byte_t := x"1F";

  subtype wtc_field_t is std_logic_vector(1 downto 0);
  subtype wait_count_t is unsigned(3 downto 0);

  signal state       : state_t := ST_FETCH_REQ;
  signal op_kind     : op_kind_t := OP_NONE;
  signal alu_func    : alu_func_t := ALU_ADD;
  signal opcode      : byte_t := x"00";
  signal modrm       : byte_t := x"00";
  signal imm16       : word_t := x"0000";
  signal imm_needed  : natural range 0 to 4 := 0;
  signal imm_index   : natural range 0 to 3 := 0;
  signal disp16      : word_t := x"0000";
  signal disp_needed : natural range 0 to 2 := 0;
  signal disp_index  : natural range 0 to 1 := 0;
  signal op_reg      : natural range 0 to 15 := 0;
  signal v25_subop   : byte_t := x"00";

  signal regs        : v25_bank_regs_t := (others => (others => (others => '0')));
  signal active_bank : natural range 0 to 7 := RESET_BANK;
  signal previous_bank : natural range 0 to 7 := RESET_BANK;
  function reset_flags_for_bank(bank : natural) return v25_flags_t is
    variable f : v25_flags_t := V25_RESET_FLAGS;
  begin
    f.rb := std_logic_vector(to_unsigned(bank, 3));
    return f;
  end function;
  function reset_saved_psw_values return bank_word_t is
    variable values : bank_word_t;
  begin
    for i in 0 to 7 loop
      values(i) := v25_pack_psw(reset_flags_for_bank(i));
    end loop;
    return values;
  end function;
  function reset_internal_data return internal_data_t is
    variable values : internal_data_t := (others => x"00");
  begin
    values(SFR_PM0_INDEX) := x"FF";
    values(SFR_PM1_INDEX) := x"FF";
    values(SFR_PM2_INDEX) := x"FF";
    values(SFR_EXIC0_INDEX) := x"47";
    values(SFR_EXIC1_INDEX) := x"47";
    values(SFR_EXIC2_INDEX) := x"47";
    values(SFR_SEIC0_INDEX) := x"47";
    values(SFR_SRIC0_INDEX) := x"47";
    values(SFR_STIC0_INDEX) := x"47";
    values(SFR_SEIC1_INDEX) := x"47";
    values(SFR_SRIC1_INDEX) := x"47";
    values(SFR_STIC1_INDEX) := x"47";
    values(SFR_TMIC0_INDEX) := x"47";
    values(SFR_TMIC1_INDEX) := x"47";
    values(SFR_TMIC2_INDEX) := x"47";
    values(SFR_DIC0_INDEX) := x"47";
    values(SFR_DIC1_INDEX) := x"47";
    values(SFR_RFM_INDEX) := x"FC";
    values(SFR_WTC_INDEX) := x"FF";
    values(SFR_WTC_INDEX + 1) := x"FF";
    values(SFR_PRC_INDEX) := x"4E";
    values(SFR_TBIC_INDEX) := x"47";
    values(SFR_IDB_INDEX) := x"FF";
    return values;
  end function;

  signal flags       : v25_flags_t := reset_flags_for_bank(RESET_BANK);
  signal bank_saved_ip : bank_word_t := (others => x"0000");
  signal bank_saved_psw : bank_word_t := reset_saved_psw_values;
  signal bank_vector_ip : bank_word_t := (others => x"0000");
  signal bank_ps : bank_word_t := (others => RESET_PS);
  signal bank_ss : bank_word_t := (others => x"0000");
  signal bank_ds0 : bank_word_t := (others => x"0000");
  signal bank_ds1 : bank_word_t := (others => x"0000");
  signal internal_data : internal_data_t := reset_internal_data;
  signal rfm_rflv_slave : std_logic := '1';
  signal idb_high : byte_t := x"FF";
  signal nmi_prev : std_logic := '0';
  signal nmi_pending : std_logic := '0';
  signal intp0_prev : std_logic := '0';
  signal intp1_prev : std_logic := '0';
  signal intp2_prev : std_logic := '0';
  signal dmarq0_prev : std_logic := '0';
  signal dmarq1_prev : std_logic := '0';
  signal halt_stop_mode : std_logic := '0';
  signal serial0_rx_unread : std_logic := '0';
  signal serial1_rx_unread : std_logic := '0';
  signal timebase_counter : unsigned(19 downto 0) := (others => '0');
  signal timebase_tap_prev : std_logic := '0';
  signal timer_tout_r : std_logic := '1';
  signal tmc0_prev : byte_t := x"00";
  signal timer0_tick_pending : natural range 0 to 255 := 0;
  signal timer0_md_tick_pending : natural range 0 to 255 := 0;
  signal timer1_tick_pending : natural range 0 to 255 := 0;
  signal timing_counter : natural range 0 to TIMING_COUNTER_MAX := 0;
  signal fixed_timing_counter : natural range 0 to 31 := 0;
  signal prefetch_count : natural range 0 to PREFETCH_QUEUE_DEPTH := 0;
  signal prefetch_pending : std_logic := '0';
  signal prefetch_bytes : prefetch_byte_queue_t := (others => x"00");
  signal prefetch_ps_queue : prefetch_word_queue_t := (others => RESET_PS);
  signal prefetch_ip_queue : prefetch_word_queue_t := (others => RESET_IP);
  signal prefetch_pending_ps : word_t := RESET_PS;
  signal prefetch_pending_ip : word_t := RESET_IP;
  signal poll_wait_count : natural range 0 to 4 := 0;
  signal ps          : word_t := RESET_PS;
  signal ss          : word_t := x"0000";
  signal ds0         : word_t := x"0000";
  signal ds1         : word_t := x"0000";
  signal ip          : word_t := RESET_IP;
  signal instr_start_ip : word_t := RESET_IP;
  signal seg_override : seg_select_t := SEG_DEFAULT;
  signal seg_override_consumed : std_logic := '0';
  signal rep_mode : rep_mode_t := REP_NONE;
  signal rep_mode_consumed : std_logic := '0';
  signal rep_timing_loaded : std_logic := '0';

  signal push_value  : word_t := x"0000";
  signal push_mode   : push_mode_t := PUSH_ONLY;
  signal push_regs_index : natural range 0 to 7 := 0;
  signal push_sp_save : word_t := x"0000";
  signal prepare_temp : word_t := x"0000";
  signal prepare_copy_bp : word_t := x"0000";
  signal prepare_level : natural range 0 to 255 := 0;
  signal prepare_remaining : natural range 0 to 255 := 0;
  signal pop_mode    : pop_mode_t := POP_TO_REG;
  signal pop_target  : natural range 0 to 7 := 0;
  signal pop_regs_index : natural range 0 to 7 := 0;
  signal pop_low     : byte_t := x"00";
  signal branch_ip   : word_t := x"0000";
  signal far_seg     : word_t := x"0000";
  signal int_vector_base : addr20_t := (others => '0');
  signal external_irq_vector : byte_t := x"00";
  signal int_target_ip : word_t := x"0000";
  signal int_target_ps : word_t := x"0000";
  signal int_return_ip : word_t := x"0000";
  signal int_ibrk_after : std_logic := '0';
  signal int_ack_valid_r : std_logic := '0';
  signal int_ack_second_r : std_logic := '0';
  signal stack_adjust : word_t := x"0000";
  signal mem_op_addr : addr20_t := (others => '0');
  signal mem_low     : byte_t := x"00";
  signal mem_value   : word_t := x"0000";
  signal mem_is_word : std_logic := '0';
  signal mem_target  : natural range 0 to 7 := 0;
  signal mem_sreg_target : seg_select_t := SEG_DEFAULT;
  signal bcd_src_addr : addr20_t := (others => '0');
  signal bcd_dst_addr : addr20_t := (others => '0');
  signal bcd_src_byte : byte_t := x"00";
  signal bcd_index    : natural range 0 to 127 := 0;
  signal bcd_total    : natural range 0 to 127 := 0;
  signal bcd_carry    : std_logic := '0';
  signal bcd_zero     : std_logic := '1';

  signal mem_valid_r : std_logic := '0';
  signal mem_write_r : std_logic := '0';
  signal mem_addr_r  : addr20_t := (others => '0');
  signal mem_wdata_r : byte_t := x"00";

  signal io_valid_r : std_logic := '0';
  signal io_write_r : std_logic := '0';
  signal io_addr_r  : word_t := x"0000";
  signal io_wdata_r : byte_t := x"00";
  signal io_low     : byte_t := x"00";

  signal dma_active_channel : natural range 0 to 1 := 0;
  signal dma_src_addr       : addr20_t := (others => '0');
  signal dma_dst_addr       : addr20_t := (others => '0');
  signal dma_data           : byte_t := x"00";
  signal dma_word_mode      : std_logic := '0';
  signal dma_high_phase     : std_logic := '0';
  signal dma_source_is_io   : std_logic := '0';
  signal dma_dest_is_io     : std_logic := '0';

  signal macro_irq_ctrl_index : natural range 0 to 511 := SFR_EXIC0_INDEX;
  signal macro_channel_base   : natural range 0 to 63 := 0;
  signal macro_mem_addr       : addr20_t := (others => '0');
  signal macro_sfr_index      : natural range 0 to 511 := SFR_P0_INDEX;
  signal macro_data           : byte_t := x"00";
  signal macro_word_mode      : std_logic := '0';
  signal macro_high_phase     : std_logic := '0';
  signal macro_search_mode    : std_logic := '0';

  signal field_addr   : addr20_t := (others => '0');
  signal field_b0     : byte_t := x"00";
  signal field_b1     : byte_t := x"00";
  signal field_b2     : byte_t := x"00";
  signal field_b3     : byte_t := x"00";
  signal field_offset : natural range 0 to 15 := 0;
  signal field_length : natural range 1 to 16 := 1;
  signal field_reg    : natural range 0 to 7 := 0;

  signal fault_r     : std_logic := '0';

  function opcode_to_alu(op : byte_t) return alu_func_t is
  begin
    case op is
      when x"04" | x"05" => return ALU_ADD;
      when x"0C" | x"0D" => return ALU_OR;
      when x"14" | x"15" => return ALU_ADC;
      when x"1C" | x"1D" => return ALU_SBB;
      when x"24" | x"25" => return ALU_AND;
      when x"2C" | x"2D" => return ALU_SUB;
      when x"34" | x"35" => return ALU_XOR;
      when others         => return ALU_CMP;
    end case;
  end function;

  function group_to_alu(group_bits : std_logic_vector(2 downto 0)) return alu_func_t is
  begin
    case group_bits is
      when "000" => return ALU_ADD;
      when "001" => return ALU_OR;
      when "010" => return ALU_ADC;
      when "011" => return ALU_SBB;
      when "100" => return ALU_AND;
      when "101" => return ALU_SUB;
      when "110" => return ALU_XOR;
      when others => return ALU_CMP;
    end case;
  end function;

  function opcode_timing_budget(op : byte_t) return natural is
    variable op_u : unsigned(7 downto 0) := unsigned(op);
  begin
    if op = x"90" then
      return v25_clocks_nop;
    elsif op_u >= to_unsigned(16#91#, 8) and op_u <= to_unsigned(16#97#, 8) then
      return v25_clocks_xchg_acc_reg16;
    elsif op = x"FA" then
      return v25_clocks_di;
    elsif op = x"FB" then
      return v25_clocks_ei;
    elsif op = x"F0" then
      return v25_clocks_buslock;
    elsif op = x"64" or op = x"65" or op = x"F2" or op = x"F3" then
      return v25_clocks_repeat_prefix;
    elsif op = x"26" or op = x"2E" or op = x"36" or op = x"3E" then
      return v25_clocks_segment_prefix;
    elsif op = x"06" or op = x"0E" or op = x"16" or op = x"1E" then
      return v25_clocks_push_sreg(false, 0);
    elsif op = x"07" or op = x"17" or op = x"1F" then
      return v25_clocks_pop_sreg(0);
    elsif op = x"04" or op = x"0C" or op = x"14" or op = x"1C" or
          op = x"24" or op = x"2C" or op = x"34" or op = x"3C" then
      return v25_clocks_alu_reg_imm(false);
    elsif op = x"27" then
      return v25_clocks_adj4a;
    elsif op = x"2F" then
      return v25_clocks_adj4s;
    elsif op = x"37" then
      return v25_clocks_adjba;
    elsif op = x"3F" then
      return v25_clocks_adjbs;
    elsif op = x"05" or op = x"0D" or op = x"15" or op = x"1D" or
          op = x"25" or op = x"2D" or op = x"35" or op = x"3D" then
      return v25_clocks_alu_reg_imm(true);
    elsif op_u >= to_unsigned(16#40#, 8) and op_u <= to_unsigned(16#4F#, 8) then
      return v25_clocks_incdec_reg(true);
    elsif op_u >= to_unsigned(16#50#, 8) and op_u <= to_unsigned(16#57#, 8) then
      return v25_clocks_push_reg16(false, 0);
    elsif op_u >= to_unsigned(16#58#, 8) and op_u <= to_unsigned(16#5F#, 8) then
      return v25_clocks_pop_reg16(0);
    elsif op = x"60" then
      return v25_clocks_push_regs(false, 0);
    elsif op = x"61" then
      return v25_clocks_pop_regs(false, 0);
    elsif op = x"68" then
      return v25_clocks_push_imm16(false, 0);
    elsif op = x"6A" then
      return v25_clocks_push_imm8(false, 0);
    elsif op = x"9A" then
      return v25_clocks_call_far(false, 0);
    elsif op = x"9C" then
      return v25_clocks_push_psw(false, 0);
    elsif op = x"9D" then
      return v25_clocks_pop_psw(0);
    elsif op = x"9E" then
      return v25_clocks_mov_psw_ah;
    elsif op = x"9F" then
      return v25_clocks_mov_ah_psw;
    elsif op = x"98" then
      return v25_clocks_cvtbw;
    elsif op = x"99" then
      return v25_clocks_cvtwl;
    elsif op = x"A8" then
      return v25_clocks_test_acc_imm(false);
    elsif op = x"A9" then
      return v25_clocks_test_acc_imm(true);
    elsif op = x"A0" then
      return v25_clocks_mov_acc_dmem(false, 0);
    elsif op = x"A1" then
      return v25_clocks_mov_acc_dmem(true, 0);
    elsif op = x"A2" then
      return v25_clocks_mov_dmem_acc(false, false, 0);
    elsif op = x"A3" then
      return v25_clocks_mov_dmem_acc(true, false, 0);
    elsif op = x"A4" then
      return v25_clocks_movk_single(false, false, 0);
    elsif op = x"A5" then
      return v25_clocks_movk_single(true, false, 0);
    elsif op = x"A6" then
      return v25_clocks_cmpk_single(false, false, 0);
    elsif op = x"A7" then
      return v25_clocks_cmpk_single(true, false, 0);
    elsif op = x"AA" then
      return v25_clocks_stm_single(false, false, 0);
    elsif op = x"AB" then
      return v25_clocks_stm_single(true, false, 0);
    elsif op = x"AC" then
      return v25_clocks_ldm_single(false, 0);
    elsif op = x"AD" then
      return v25_clocks_ldm_single(true, 0);
    elsif op = x"AE" then
      return v25_clocks_cmpm_single(false, 0);
    elsif op = x"AF" then
      return v25_clocks_cmpm_single(true, 0);
    elsif op_u >= to_unsigned(16#B0#, 8) and op_u <= to_unsigned(16#B7#, 8) then
      return v25_clocks_mov_reg_imm(false);
    elsif op_u >= to_unsigned(16#B8#, 8) and op_u <= to_unsigned(16#BF#, 8) then
      return v25_clocks_mov_reg_imm(true);
    elsif op = x"C2" then
      return v25_clocks_ret_near_pop(0);
    elsif op = x"C3" then
      return v25_clocks_ret_near(0);
    elsif op = x"C9" then
      return v25_clocks_dispose(0);
    elsif op = x"CA" then
      return v25_clocks_ret_far_pop(0);
    elsif op = x"CB" then
      return v25_clocks_ret_far(0);
    elsif op = x"CF" then
      return v25_clocks_reti(false, 0);
    elsif op = x"D4" then
      return v25_clocks_cvtbd;
    elsif op = x"D5" then
      return v25_clocks_cvtdb;
    elsif op = x"E4" then
      return v25_clocks_in_acc_imm(false, 0);
    elsif op = x"E5" then
      return v25_clocks_in_acc_imm(true, 0);
    elsif op = x"E6" then
      return v25_clocks_out_imm_acc(false, 0);
    elsif op = x"E7" then
      return v25_clocks_out_imm_acc(true, 0);
    elsif op = x"E8" then
      return v25_clocks_call_near(false, 0);
    elsif op = x"E9" then
      return v25_clocks_branch_near;
    elsif op = x"EA" then
      return v25_clocks_branch_far;
    elsif op = x"EB" then
      return v25_clocks_branch_short;
    elsif op = x"EC" then
      return v25_clocks_in_acc_dw(false, 0);
    elsif op = x"ED" then
      return v25_clocks_in_acc_dw(true, 0);
    elsif op = x"EE" then
      return v25_clocks_out_dw_acc(false, 0);
    elsif op = x"EF" then
      return v25_clocks_out_dw_acc(true, 0);
    elsif op = x"6C" then
      return v25_clocks_inm_single(false, false, 0);
    elsif op = x"6D" then
      return v25_clocks_inm_single(true, false, 0);
    elsif op = x"6E" then
      return v25_clocks_outm_single(false, false, 0);
    elsif op = x"6F" then
      return v25_clocks_outm_single(true, false, 0);
    end if;

    return 0;
  end function;

  function v25_subop_timing_budget(subop : byte_t) return natural is
  begin
    case subop is
      when x"2D" =>
        return v25_clocks_brkcs;
      when x"25" =>
        return v25_clocks_movspa;
      when x"95" =>
        return v25_clocks_movspb;
      when x"91" =>
        return v25_clocks_retrbi;
      when x"92" =>
        return v25_clocks_fint;
      when x"94" =>
        return v25_clocks_tsksw;
      when others =>
        return 0;
    end case;
  end function;

  function v25_bit_timing_op(subop : byte_t) return v25_bit_timing_op_t is
  begin
    case subop(2 downto 1) is
      when "00" =>
        return V25_TIM_TEST1;
      when "01" =>
        return V25_TIM_CLR1;
      when "10" =>
        return V25_TIM_SET1;
      when others =>
        return V25_TIM_NOT1;
    end case;
  end function;

  function modrm_timing_budget(
    kind : op_kind_t;
    alu_op : alu_func_t;
    subop : byte_t;
    modrm_value : byte_t
  ) return natural is
    variable ea_clocks : natural := v25_ea_clocks(modrm_value);
    variable reg_operand : boolean := modrm_value(7 downto 6) = "11";
    variable group_alu : alu_func_t := group_to_alu(modrm_value(5 downto 3));
    variable bit_op : v25_bit_timing_op_t := v25_bit_timing_op(subop);
  begin
    case kind is
      when OP_MOV_RM8_R8 =>
        if reg_operand then
          return v25_clocks_mov_reg_reg;
        end if;
        return v25_clocks_mov_mem_reg(false, false, ea_clocks, 0);

      when OP_MOV_RM16_R16 | OP_MOV_RM16_SREG =>
        if reg_operand then
          if kind = OP_MOV_RM16_SREG then
            return v25_clocks_mov_reg_sreg;
          end if;
          return v25_clocks_mov_reg_reg;
        end if;
        if kind = OP_MOV_RM16_SREG then
          return v25_clocks_mov_mem_sreg(false, ea_clocks, 0);
        end if;
        return v25_clocks_mov_mem_reg(true, false, ea_clocks, 0);

      when OP_MOV_R8_RM8 =>
        if reg_operand then
          return v25_clocks_mov_reg_reg;
        end if;
        return v25_clocks_mov_reg_mem(false, ea_clocks, 0);

      when OP_MOV_R16_RM16 | OP_MOV_SREG_RM16 =>
        if reg_operand then
          if kind = OP_MOV_SREG_RM16 then
            return v25_clocks_mov_sreg_reg;
          end if;
          return v25_clocks_mov_reg_reg;
        end if;
        if kind = OP_MOV_SREG_RM16 then
          return v25_clocks_mov_sreg_mem(ea_clocks, 0);
        end if;
        return v25_clocks_mov_reg_mem(true, ea_clocks, 0);

      when OP_LEA_R16_MEM =>
        if reg_operand then
          return 0;
        end if;
        return v25_clocks_ldea(ea_clocks);

      when OP_MOV_DS0_R16_MEM32 | OP_MOV_DS1_R16_MEM32 =>
        if reg_operand then
          return 0;
        end if;
        return v25_clocks_mov_ds_reg_mem32(ea_clocks, 0);

      when OP_MOV_RM8_IMM8 =>
        if reg_operand then
          return v25_clocks_mov_reg_imm(false);
        end if;
        return v25_clocks_mov_mem_imm(false, false, ea_clocks, 0);

      when OP_MOV_RM16_IMM16 =>
        if reg_operand then
          return v25_clocks_mov_reg_imm(true);
        end if;
        return v25_clocks_mov_mem_imm(true, false, ea_clocks, 0);

      when OP_XCHG_RM8_R8 =>
        if reg_operand then
          return v25_clocks_xchg_reg_reg;
        end if;
        return v25_clocks_xchg_mem_reg(false, false, ea_clocks, 0);

      when OP_XCHG_RM16_R16 =>
        if reg_operand then
          return v25_clocks_xchg_reg_reg;
        end if;
        return v25_clocks_xchg_mem_reg(true, false, ea_clocks, 0);

      when OP_ALU_RM8_R8 =>
        if reg_operand then
          return v25_clocks_alu_reg_reg;
        elsif alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(false, ea_clocks, 0);
        end if;
        return v25_clocks_alu_mem_reg(false, false, ea_clocks, 0);

      when OP_ALU_RM16_R16 =>
        if reg_operand then
          return v25_clocks_alu_reg_reg;
        elsif alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(true, ea_clocks, 0);
        end if;
        return v25_clocks_alu_mem_reg(true, false, ea_clocks, 0);

      when OP_ALU_R8_RM8 =>
        if reg_operand then
          return v25_clocks_alu_reg_reg;
        elsif alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(false, ea_clocks, 0);
        end if;
        return v25_clocks_alu_reg_mem(false, ea_clocks, 0);

      when OP_ALU_R16_RM16 =>
        if reg_operand then
          return v25_clocks_alu_reg_reg;
        elsif alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(true, ea_clocks, 0);
        end if;
        return v25_clocks_alu_reg_mem(true, ea_clocks, 0);

      when OP_GRP_IMM8_RM8 =>
        if reg_operand then
          return v25_clocks_alu_reg_imm(false);
        elsif group_alu = ALU_CMP then
          return v25_clocks_cmp_mem_imm(false, ea_clocks, 0);
        elsif group_alu = ALU_ADD or group_alu = ALU_ADC or
              group_alu = ALU_SBB or group_alu = ALU_SUB then
          return v25_clocks_addsub_mem_imm(false, false, ea_clocks, 0);
        end if;
        return v25_clocks_logic_mem_imm(false, false, ea_clocks, 0);

      when OP_GRP_IMM16_RM16 | OP_GRP_IMM8_RM16_SIGN =>
        if reg_operand then
          return v25_clocks_alu_reg_imm(true);
        elsif group_alu = ALU_CMP then
          return v25_clocks_cmp_mem_imm(true, ea_clocks, 0);
        elsif group_alu = ALU_ADD or group_alu = ALU_ADC or
              group_alu = ALU_SBB or group_alu = ALU_SUB then
          return v25_clocks_addsub_mem_imm(true, false, ea_clocks, 0);
        end if;
        return v25_clocks_logic_mem_imm(true, false, ea_clocks, 0);

      when OP_TEST_RM8_R8 =>
        if reg_operand then
          return v25_clocks_test_reg_reg;
        end if;
        return v25_clocks_test_mem_reg(false, ea_clocks, 0);

      when OP_TEST_RM16_R16 =>
        if reg_operand then
          return v25_clocks_test_reg_reg;
        end if;
        return v25_clocks_test_mem_reg(true, ea_clocks, 0);

      when OP_GRP3_RM8 =>
        if modrm_value(5 downto 3) = "000" then
          if reg_operand then
            return v25_clocks_test_reg_imm(false);
          end if;
          return v25_clocks_test_mem_imm(false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "010" or modrm_value(5 downto 3) = "011" then
          if reg_operand then
            return v25_clocks_notneg_reg;
          end if;
          return v25_clocks_notneg_mem(false, false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "100" then
          if reg_operand then
            return v25_clocks_mulu_reg(false);
          end if;
          return v25_clocks_mulu_mem(false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "101" then
          if reg_operand then
            return v25_clocks_mul_reg(false, true);
          end if;
          return v25_clocks_mul_mem(false, ea_clocks, 0, true);
        elsif modrm_value(5 downto 3) = "110" then
          if reg_operand then
            return v25_clocks_divu_reg(false);
          end if;
          return v25_clocks_divu_mem(false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "111" then
          if reg_operand then
            return v25_clocks_div_reg(false, true);
          end if;
          return v25_clocks_div_mem(false, ea_clocks, 0, true);
        end if;

      when OP_GRP3_RM16 =>
        if modrm_value(5 downto 3) = "000" then
          if reg_operand then
            return v25_clocks_test_reg_imm(true);
          end if;
          return v25_clocks_test_mem_imm(true, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "010" or modrm_value(5 downto 3) = "011" then
          if reg_operand then
            return v25_clocks_notneg_reg;
          end if;
          return v25_clocks_notneg_mem(true, false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "100" then
          if reg_operand then
            return v25_clocks_mulu_reg(true);
          end if;
          return v25_clocks_mulu_mem(true, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "101" then
          if reg_operand then
            return v25_clocks_mul_reg(true, true);
          end if;
          return v25_clocks_mul_mem(true, ea_clocks, 0, true);
        elsif modrm_value(5 downto 3) = "110" then
          if reg_operand then
            return v25_clocks_divu_reg(true);
          end if;
          return v25_clocks_divu_mem(true, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "111" then
          if reg_operand then
            return v25_clocks_div_reg(true, true);
          end if;
          return v25_clocks_div_mem(true, ea_clocks, 0, true);
        end if;

      when OP_IMUL_R16_RM16_IMM8 =>
        if reg_operand then
          return v25_clocks_imul_imm_reg(true, true);
        end if;
        return v25_clocks_imul_imm_mem(true, ea_clocks, 0, true);

      when OP_IMUL_R16_RM16_IMM16 =>
        if reg_operand then
          return v25_clocks_imul_imm_reg(false, true);
        end if;
        return v25_clocks_imul_imm_mem(false, ea_clocks, 0, true);

      when OP_GRP_SHIFT_RM8_1 =>
        if reg_operand then
          return v25_clocks_shift_reg_1;
        end if;
        return v25_clocks_shift_mem_1(false, false, ea_clocks, 0);

      when OP_GRP_SHIFT_RM16_1 =>
        if reg_operand then
          return v25_clocks_shift_reg_1;
        end if;
        return v25_clocks_shift_mem_1(true, false, ea_clocks, 0);

      when OP_GRP_FE =>
        if modrm_value(5 downto 3) = "000" or modrm_value(5 downto 3) = "001" then
          if reg_operand then
            return v25_clocks_incdec_reg(false);
          end if;
          return v25_clocks_incdec_mem(false, false, ea_clocks, 0);
        end if;

      when OP_GRP_FF =>
        if modrm_value(5 downto 3) = "000" or modrm_value(5 downto 3) = "001" then
          if reg_operand then
            return v25_clocks_incdec_reg(true);
          end if;
          return v25_clocks_incdec_mem(true, false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "010" then
          if reg_operand then
            return v25_clocks_call_near(false, 0);
          end if;
          return v25_clocks_call_memptr16(false, ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "011" then
          if not reg_operand then
            return v25_clocks_call_memptr32(false, ea_clocks, 0);
          end if;
        elsif modrm_value(5 downto 3) = "100" then
          if reg_operand then
            return v25_clocks_branch_regptr16;
          end if;
          return v25_clocks_branch_memptr16(ea_clocks, 0);
        elsif modrm_value(5 downto 3) = "101" then
          if not reg_operand then
            return v25_clocks_branch_memptr32(ea_clocks, 0);
          end if;
        elsif modrm_value(5 downto 3) = "110" then
          if reg_operand then
            return v25_clocks_push_reg16(false, 0);
          end if;
          return v25_clocks_push_mem16(false, ea_clocks, 0);
        end if;

      when OP_POP_RM16 =>
        if modrm_value(5 downto 3) = "000" then
          if reg_operand then
            return v25_clocks_pop_reg16(0);
          end if;
          return v25_clocks_pop_mem16(false, ea_clocks, 0);
        end if;

      when OP_CHKIND =>
        return v25_clocks_chkind(ea_clocks, 0);

      when OP_ROL4 =>
        if reg_operand then
          return v25_clocks_rol4_reg;
        end if;
        return v25_clocks_rol4_mem(false, ea_clocks, 0);

      when OP_ROR4 =>
        if reg_operand then
          return v25_clocks_ror4_reg;
        end if;
        return v25_clocks_ror4_mem(false, ea_clocks, 0);

      when OP_V25_BITOP =>
        if reg_operand then
          if subop(3) = '1' then
            return v25_clocks_bitop_reg_imm(bit_op, subop(0) = '1');
          end if;
          return v25_clocks_bitop_reg_cl(bit_op, subop(0) = '1');
        elsif subop(3) = '1' then
          return v25_clocks_bitop_mem_imm(bit_op, subop(0) = '1', false, ea_clocks, 0);
        end if;
        return v25_clocks_bitop_mem_cl(bit_op, subop(0) = '1', false, ea_clocks, 0);

      when others =>
        return 0;
    end case;

    return 0;
  end function;

  function is_v25_bitop_subop(subop : byte_t) return boolean is
  begin
    return unsigned(subop) >= to_unsigned(16#10#, 8)
      and unsigned(subop) <= to_unsigned(16#1F#, 8);
  end function;

  function op_needs_imm_after_modrm(kind : op_kind_t; subop : byte_t; modrm_value : byte_t)
    return boolean is
  begin
    return kind = OP_MOV_RM8_IMM8 or
      kind = OP_MOV_RM16_IMM16 or
      kind = OP_IMUL_R16_RM16_IMM16 or
      kind = OP_IMUL_R16_RM16_IMM8 or
      kind = OP_GRP_IMM8_RM8 or
      kind = OP_GRP_IMM16_RM16 or
      kind = OP_GRP_IMM8_RM16_SIGN or
      (kind = OP_GRP3_RM8 and modrm_value(5 downto 3) = "000") or
      (kind = OP_GRP3_RM16 and modrm_value(5 downto 3) = "000") or
      kind = OP_GRP_SHIFT_RM8_IMM or
      kind = OP_GRP_SHIFT_RM16_IMM or
      ((kind = OP_INS_FIELD or kind = OP_EXT_FIELD) and (subop = x"39" or subop = x"3B")) or
      (kind = OP_V25_BITOP and subop(3) = '1');
  end function;

  function is_repeatable_string_op(kind : op_kind_t) return boolean is
  begin
    return kind = OP_MOVS8 or kind = OP_MOVS16 or
      kind = OP_CMPS8 or kind = OP_CMPS16 or
      kind = OP_STOS8 or kind = OP_STOS16 or
      kind = OP_LODS8 or kind = OP_LODS16 or
      kind = OP_SCAS8 or kind = OP_SCAS16 or
      kind = OP_INM8 or kind = OP_INM16 or
      kind = OP_OUTM8 or kind = OP_OUTM16;
  end function;

  function repeat_continues(
    mode : rep_mode_t;
    kind : op_kind_t;
    flags_after : v25_flags_t;
    cx_after : word_t
  ) return boolean is
  begin
    if mode = REP_NONE or cx_after = x"0000" then
      return false;
    end if;

    if kind = OP_CMPS8_DST or kind = OP_CMPS16_DST or
       kind = OP_SCAS8 or kind = OP_SCAS16 then
      if mode = REP_E then
        return flags_after.zf = '1';
      elsif mode = REP_NE then
        return flags_after.zf = '0';
      elsif mode = REP_C then
        return flags_after.cf = '1';
      elsif mode = REP_NC then
        return flags_after.cf = '0';
      end if;
    end if;

    return mode = REP_E or mode = REP_NE;
  end function;

  function repeat_start_kind(kind : op_kind_t) return op_kind_t is
  begin
    case kind is
      when OP_CMPS8_DST =>
        return OP_CMPS8;
      when OP_CMPS16_DST =>
        return OP_CMPS16;
      when others =>
        return kind;
    end case;
  end function;

  function is_memory_string_op(kind : op_kind_t) return boolean is
    variable start_kind : op_kind_t := repeat_start_kind(kind);
  begin
    return start_kind = OP_MOVS8 or start_kind = OP_MOVS16 or
      start_kind = OP_CMPS8 or start_kind = OP_CMPS16 or
      start_kind = OP_STOS8 or start_kind = OP_STOS16 or
      start_kind = OP_LODS8 or start_kind = OP_LODS16 or
      start_kind = OP_SCAS8 or start_kind = OP_SCAS16;
  end function;

  function is_primitive_io_block_op(kind : op_kind_t) return boolean is
  begin
    return kind = OP_INM8 or kind = OP_INM16 or
      kind = OP_OUTM8 or kind = OP_OUTM16;
  end function;

  function repeat_prefix_applies(mode : rep_mode_t; kind : op_kind_t) return boolean is
  begin
    if mode = REP_NONE then
      return false;
    end if;

    if kind = OP_CMPS8 or kind = OP_CMPS16 or
       kind = OP_SCAS8 or kind = OP_SCAS16 then
      return true;
    end if;

    return mode = REP_E or mode = REP_NE;
  end function;

  function repeat_uses_conditional_stop(kind : op_kind_t) return boolean is
    variable start_kind : op_kind_t := repeat_start_kind(kind);
  begin
    return start_kind = OP_CMPS8 or start_kind = OP_CMPS16 or
      start_kind = OP_SCAS8 or start_kind = OP_SCAS16;
  end function;

  function wtc_field_wait_states(field : wtc_field_t) return natural is
  begin
    case field is
      when "00" =>
        return 0;
      when "01" =>
        return 1;
      when others =>
        return 2;
    end case;
  end function;

  function current_io_wait_states(values : internal_data_t) return natural is
  begin
    return wtc_field_wait_states(values(SFR_WTC_INDEX + 1)(7 downto 6));
  end function;

  function wtc_memory_field(values : internal_data_t; addr_value : addr20_t) return wtc_field_t is
  begin
    case addr_value(19 downto 17) is
      when "000" =>
        return values(SFR_WTC_INDEX)(1 downto 0);
      when "001" =>
        return values(SFR_WTC_INDEX)(3 downto 2);
      when "010" =>
        return values(SFR_WTC_INDEX)(5 downto 4);
      when "011" =>
        return values(SFR_WTC_INDEX)(7 downto 6);
      when "100" =>
        return values(SFR_WTC_INDEX + 1)(1 downto 0);
      when "101" =>
        return values(SFR_WTC_INDEX + 1)(3 downto 2);
      when others =>
        return values(SFR_WTC_INDEX + 1)(5 downto 4);
    end case;
  end function;

  function current_memory_wait_states(values : internal_data_t; addr_value : addr20_t) return natural is
  begin
    return wtc_field_wait_states(wtc_memory_field(values, addr_value));
  end function;

  function current_max_memory_wait_states(values : internal_data_t) return natural is
    variable result     : natural := 0;
    variable field_wait : natural := 0;
  begin
    result := wtc_field_wait_states(values(SFR_WTC_INDEX)(1 downto 0));

    field_wait := wtc_field_wait_states(values(SFR_WTC_INDEX)(3 downto 2));
    if field_wait > result then
      result := field_wait;
    end if;

    field_wait := wtc_field_wait_states(values(SFR_WTC_INDEX)(5 downto 4));
    if field_wait > result then
      result := field_wait;
    end if;

    field_wait := wtc_field_wait_states(values(SFR_WTC_INDEX)(7 downto 6));
    if field_wait > result then
      result := field_wait;
    end if;

    field_wait := wtc_field_wait_states(values(SFR_WTC_INDEX + 1)(1 downto 0));
    if field_wait > result then
      result := field_wait;
    end if;

    field_wait := wtc_field_wait_states(values(SFR_WTC_INDEX + 1)(3 downto 2));
    if field_wait > result then
      result := field_wait;
    end if;

    field_wait := wtc_field_wait_states(values(SFR_WTC_INDEX + 1)(5 downto 4));
    if field_wait > result then
      result := field_wait;
    end if;

    return result;
  end function;

  function string_single_timing_budget(
    kind : op_kind_t;
    mem_wait_states : natural;
    mem_onchip : boolean
  ) return natural is
    variable start_kind : op_kind_t := repeat_start_kind(kind);
  begin
    case start_kind is
      when OP_MOVS8 =>
        return v25_clocks_movk_single(false, mem_onchip, mem_wait_states);
      when OP_MOVS16 =>
        return v25_clocks_movk_single(true, mem_onchip, mem_wait_states);
      when OP_CMPS8 =>
        return v25_clocks_cmpk_single(false, mem_onchip, mem_wait_states);
      when OP_CMPS16 =>
        return v25_clocks_cmpk_single(true, mem_onchip, mem_wait_states);
      when OP_STOS8 =>
        return v25_clocks_stm_single(false, mem_onchip, mem_wait_states);
      when OP_STOS16 =>
        return v25_clocks_stm_single(true, mem_onchip, mem_wait_states);
      when OP_LODS8 =>
        return v25_clocks_ldm_single(false, mem_wait_states);
      when OP_LODS16 =>
        return v25_clocks_ldm_single(true, mem_wait_states);
      when OP_SCAS8 =>
        return v25_clocks_cmpm_single(false, mem_wait_states);
      when OP_SCAS16 =>
        return v25_clocks_cmpm_single(true, mem_wait_states);
      when others =>
        return 0;
    end case;
  end function;

  function repeat_iteration_timing_budget(
    kind : op_kind_t;
    mem_wait_states : natural;
    mem_onchip : boolean
  ) return natural is
    variable start_kind : op_kind_t := repeat_start_kind(kind);
  begin
    case start_kind is
      when OP_MOVS8 =>
        return v25_clocks_movk_repeat(false, mem_onchip, mem_wait_states, 1) -
          v25_clocks_movk_repeat(false, mem_onchip, mem_wait_states, 0);
      when OP_MOVS16 =>
        return v25_clocks_movk_repeat(true, mem_onchip, mem_wait_states, 1) -
          v25_clocks_movk_repeat(true, mem_onchip, mem_wait_states, 0);
      when OP_CMPS8 =>
        return v25_clocks_cmpk_repeat(false, mem_onchip, mem_wait_states, 1) -
          v25_clocks_cmpk_repeat(false, mem_onchip, mem_wait_states, 0);
      when OP_CMPS16 =>
        return v25_clocks_cmpk_repeat(true, mem_onchip, mem_wait_states, 1) -
          v25_clocks_cmpk_repeat(true, mem_onchip, mem_wait_states, 0);
      when OP_STOS8 =>
        return v25_clocks_stm_repeat(false, mem_onchip, mem_wait_states, 1) -
          v25_clocks_stm_repeat(false, mem_onchip, mem_wait_states, 0);
      when OP_STOS16 =>
        return v25_clocks_stm_repeat(true, mem_onchip, mem_wait_states, 1) -
          v25_clocks_stm_repeat(true, mem_onchip, mem_wait_states, 0);
      when OP_LODS8 =>
        return v25_clocks_ldm_repeat(false, mem_wait_states, 1) -
          v25_clocks_ldm_repeat(false, mem_wait_states, 0);
      when OP_LODS16 =>
        return v25_clocks_ldm_repeat(true, mem_wait_states, 1) -
          v25_clocks_ldm_repeat(true, mem_wait_states, 0);
      when OP_SCAS8 =>
        return v25_clocks_cmpm_repeat(false, mem_wait_states, 1) -
          v25_clocks_cmpm_repeat(false, mem_wait_states, 0);
      when OP_SCAS16 =>
        return v25_clocks_cmpm_repeat(true, mem_wait_states, 1) -
          v25_clocks_cmpm_repeat(true, mem_wait_states, 0);
      when others =>
        return 0;
    end case;
  end function;

  function primitive_io_single_timing_budget(
    kind : op_kind_t;
    wait_states : natural;
    mem_onchip : boolean
  ) return natural is
  begin
    case kind is
      when OP_INM8 =>
        return v25_clocks_inm_single(false, mem_onchip, wait_states);
      when OP_INM16 =>
        return v25_clocks_inm_single(true, mem_onchip, wait_states);
      when OP_OUTM8 =>
        return v25_clocks_outm_single(false, mem_onchip, wait_states);
      when OP_OUTM16 =>
        return v25_clocks_outm_single(true, mem_onchip, wait_states);
      when others =>
        return 0;
    end case;
  end function;

  function primitive_io_iteration_timing_budget(
    kind : op_kind_t;
    wait_states : natural;
    mem_onchip : boolean
  ) return natural is
  begin
    case kind is
      when OP_INM8 =>
        return v25_clocks_inm_repeat(false, mem_onchip, wait_states, 1) -
          v25_clocks_inm_repeat(false, mem_onchip, wait_states, 0);
      when OP_INM16 =>
        return v25_clocks_inm_repeat(true, mem_onchip, wait_states, 1) -
          v25_clocks_inm_repeat(true, mem_onchip, wait_states, 0);
      when OP_OUTM8 =>
        return v25_clocks_outm_repeat(false, mem_onchip, wait_states, 1) -
          v25_clocks_outm_repeat(false, mem_onchip, wait_states, 0);
      when OP_OUTM16 =>
        return v25_clocks_outm_repeat(true, mem_onchip, wait_states, 1) -
          v25_clocks_outm_repeat(true, mem_onchip, wait_states, 0);
      when others =>
        return 0;
    end case;
  end function;

  function timing_saturating_add(current_value : natural; add_value : natural) return natural is
  begin
    if current_value > TIMING_COUNTER_MAX - add_value then
      return TIMING_COUNTER_MAX;
    end if;
    return current_value + add_value;
  end function;

  function repeat_timing_budget(
    kind : op_kind_t;
    mode : rep_mode_t;
    cx_start : word_t;
    io_wait_states : natural
  ) return natural is
    variable iterations : natural;
  begin
    if not repeat_prefix_applies(mode, kind) then
      return 0;
    end if;

    if repeat_uses_conditional_stop(kind) or is_memory_string_op(kind) or
       is_primitive_io_block_op(kind) then
      iterations := 0;
    else
      iterations := to_integer(unsigned(cx_start));
    end if;

    case kind is
      when OP_MOVS8 =>
        return v25_clocks_movk_repeat(false, false, 0, iterations);
      when OP_MOVS16 =>
        return v25_clocks_movk_repeat(true, false, 0, iterations);
      when OP_CMPS8 =>
        return v25_clocks_cmpk_repeat(false, false, 0, iterations);
      when OP_CMPS16 =>
        return v25_clocks_cmpk_repeat(true, false, 0, iterations);
      when OP_STOS8 =>
        return v25_clocks_stm_repeat(false, false, 0, iterations);
      when OP_STOS16 =>
        return v25_clocks_stm_repeat(true, false, 0, iterations);
      when OP_LODS8 =>
        return v25_clocks_ldm_repeat(false, 0, iterations);
      when OP_LODS16 =>
        return v25_clocks_ldm_repeat(true, 0, iterations);
      when OP_SCAS8 =>
        return v25_clocks_cmpm_repeat(false, 0, iterations);
      when OP_SCAS16 =>
        return v25_clocks_cmpm_repeat(true, 0, iterations);
      when OP_INM8 =>
        return v25_clocks_inm_repeat(false, false, io_wait_states, iterations);
      when OP_INM16 =>
        return v25_clocks_inm_repeat(true, false, io_wait_states, iterations);
      when OP_OUTM8 =>
        return v25_clocks_outm_repeat(false, false, io_wait_states, iterations);
      when OP_OUTM16 =>
        return v25_clocks_outm_repeat(true, false, io_wait_states, iterations);
      when others =>
        return 0;
    end case;
  end function;

  function io_timing_budget(kind : op_kind_t; wait_states : natural) return natural is
  begin
    case kind is
      when OP_IN_AL_IMM8 =>
        return v25_clocks_in_acc_imm(false, wait_states);
      when OP_IN_AX_IMM8 =>
        return v25_clocks_in_acc_imm(true, wait_states);
      when OP_OUT_IMM8_AL =>
        return v25_clocks_out_imm_acc(false, wait_states);
      when OP_OUT_IMM8_AX =>
        return v25_clocks_out_imm_acc(true, wait_states);
      when OP_IN_AL_DX =>
        return v25_clocks_in_acc_dw(false, wait_states);
      when OP_IN_AX_DX =>
        return v25_clocks_in_acc_dw(true, wait_states);
      when OP_OUT_DX_AL =>
        return v25_clocks_out_dw_acc(false, wait_states);
      when OP_OUT_DX_AX =>
        return v25_clocks_out_dw_acc(true, wait_states);
      when OP_INM8 =>
        return v25_clocks_inm_single(false, false, wait_states);
      when OP_INM16 =>
        return v25_clocks_inm_single(true, false, wait_states);
      when OP_OUTM8 =>
        return v25_clocks_outm_single(false, false, wait_states);
      when OP_OUTM16 =>
        return v25_clocks_outm_single(true, false, wait_states);
      when others =>
        return 0;
    end case;
  end function;

  function direct_moffs_timing_budget(
    kind : op_kind_t;
    wait_states : natural;
    internal_selected : boolean
  ) return natural is
  begin
    case kind is
      when OP_MOV_AL_MOFFS =>
        return v25_clocks_mov_acc_dmem(false, wait_states);
      when OP_MOV_AX_MOFFS =>
        return v25_clocks_mov_acc_dmem(true, wait_states);
      when OP_MOV_MOFFS_AL =>
        return v25_clocks_mov_dmem_acc(false, internal_selected, wait_states);
      when OP_MOV_MOFFS_AX =>
        return v25_clocks_mov_dmem_acc(true, internal_selected, wait_states);
      when others =>
        return 0;
    end case;
  end function;

  function direct_stack_timing_budget(
    kind : op_kind_t;
    wait_states : natural;
    internal_selected : boolean
  ) return natural is
  begin
    case kind is
      when OP_PUSH_R16 =>
        return v25_clocks_push_reg16(internal_selected, wait_states);
      when OP_PUSH_SREG =>
        return v25_clocks_push_sreg(internal_selected, wait_states);
      when OP_PUSH_PSW =>
        return v25_clocks_push_psw(internal_selected, wait_states);
      when OP_PUSH_REGS =>
        return v25_clocks_push_regs(internal_selected, wait_states);
      when OP_PUSH_IMM8_SIGN =>
        return v25_clocks_push_imm8(internal_selected, wait_states);
      when OP_PUSH_IMM16 =>
        return v25_clocks_push_imm16(internal_selected, wait_states);
      when OP_POP_R16 =>
        return v25_clocks_pop_reg16(wait_states);
      when OP_POP_SREG =>
        return v25_clocks_pop_sreg(wait_states);
      when OP_POP_PSW =>
        return v25_clocks_pop_psw(wait_states);
      when OP_POP_REGS =>
        return v25_clocks_pop_regs(internal_selected, wait_states);
      when OP_DISPOSE =>
        return v25_clocks_dispose(wait_states);
      when OP_CALL_REL16 =>
        return v25_clocks_call_near(internal_selected, wait_states);
      when OP_CALL_FAR_IMM =>
        return v25_clocks_call_far(internal_selected, wait_states);
      when OP_RET_NEAR =>
        return v25_clocks_ret_near(wait_states);
      when OP_RET_NEAR_IMM =>
        return v25_clocks_ret_near_pop(wait_states);
      when OP_RET_FAR =>
        return v25_clocks_ret_far(wait_states);
      when OP_RET_FAR_IMM =>
        return v25_clocks_ret_far_pop(wait_states);
      when OP_RETI =>
        return v25_clocks_reti(internal_selected, wait_states);
      when others =>
        return 0;
    end case;
  end function;

  function v25_selected_seg(
    sel : seg_select_t;
    default_seg : word_t;
    ps_value : word_t;
    ss_value : word_t;
    ds0_value : word_t;
    ds1_value : word_t
  ) return word_t is
  begin
    case sel is
      when SEG_PS =>
        return ps_value;
      when SEG_SS =>
        return ss_value;
      when SEG_DS0 =>
        return ds0_value;
      when SEG_DS1 =>
        return ds1_value;
      when others =>
        return default_seg;
    end case;
  end function;

  function v25_sreg_select(sreg_bits : std_logic_vector(1 downto 0)) return seg_select_t is
  begin
    case sreg_bits is
      when "00" =>
        return SEG_DS1;
      when "01" =>
        return SEG_PS;
      when "10" =>
        return SEG_SS;
      when others =>
        return SEG_DS0;
    end case;
  end function;

  function v25_sreg_value(
    sel : seg_select_t;
    ps_value : word_t;
    ss_value : word_t;
    ds0_value : word_t;
    ds1_value : word_t
  ) return word_t is
  begin
    case sel is
      when SEG_PS =>
        return ps_value;
      when SEG_SS =>
        return ss_value;
      when SEG_DS0 =>
        return ds0_value;
      when SEG_DS1 =>
        return ds1_value;
      when others =>
        return x"0000";
    end case;
  end function;

  function alu_result8(func : alu_func_t; a : byte_t; b : byte_t; carry_in : std_logic)
    return byte_t is
    variable r : byte_t;
  begin
    case func is
      when ALU_ADD =>
        r := std_logic_vector(unsigned(a) + unsigned(b));
      when ALU_OR =>
        r := a or b;
      when ALU_ADC =>
        r := std_logic_vector(unsigned(a) + unsigned(b));
        if carry_in = '1' then
          r := std_logic_vector(unsigned(r) + 1);
        end if;
      when ALU_SBB =>
        r := std_logic_vector(unsigned(a) - unsigned(b));
        if carry_in = '1' then
          r := std_logic_vector(unsigned(r) - 1);
        end if;
      when ALU_AND =>
        r := a and b;
      when ALU_SUB =>
        r := std_logic_vector(unsigned(a) - unsigned(b));
      when ALU_XOR =>
        r := a xor b;
      when ALU_CMP =>
        r := a;
    end case;

    return r;
  end function;

  function alu_result16(func : alu_func_t; a : word_t; b : word_t; carry_in : std_logic)
    return word_t is
    variable r : word_t;
  begin
    case func is
      when ALU_ADD =>
        r := std_logic_vector(unsigned(a) + unsigned(b));
      when ALU_OR =>
        r := a or b;
      when ALU_ADC =>
        r := std_logic_vector(unsigned(a) + unsigned(b));
        if carry_in = '1' then
          r := std_logic_vector(unsigned(r) + 1);
        end if;
      when ALU_SBB =>
        r := std_logic_vector(unsigned(a) - unsigned(b));
        if carry_in = '1' then
          r := std_logic_vector(unsigned(r) - 1);
        end if;
      when ALU_AND =>
        r := a and b;
      when ALU_SUB =>
        r := std_logic_vector(unsigned(a) - unsigned(b));
      when ALU_XOR =>
        r := a xor b;
      when ALU_CMP =>
        r := a;
    end case;

    return r;
  end function;

  function alu_flags8(func : alu_func_t; old_flags : v25_flags_t; a : byte_t; b : byte_t)
    return v25_flags_t is
  begin
    case func is
      when ALU_ADD =>
        return v25_add_flags8(old_flags, a, b, '0');
      when ALU_OR | ALU_AND | ALU_XOR =>
        return v25_logic_flags8(old_flags, alu_result8(func, a, b, old_flags.cf));
      when ALU_ADC =>
        return v25_add_flags8(old_flags, a, b, old_flags.cf);
      when ALU_SBB =>
        return v25_sub_flags8(old_flags, a, b, old_flags.cf);
      when ALU_SUB | ALU_CMP =>
        return v25_sub_flags8(old_flags, a, b, '0');
    end case;
  end function;

  function alu_flags16(func : alu_func_t; old_flags : v25_flags_t; a : word_t; b : word_t)
    return v25_flags_t is
  begin
    case func is
      when ALU_ADD =>
        return v25_add_flags16(old_flags, a, b, '0');
      when ALU_OR | ALU_AND | ALU_XOR =>
        return v25_logic_flags16(old_flags, alu_result16(func, a, b, old_flags.cf));
      when ALU_ADC =>
        return v25_add_flags16(old_flags, a, b, old_flags.cf);
      when ALU_SBB =>
        return v25_sub_flags16(old_flags, a, b, old_flags.cf);
      when ALU_SUB | ALU_CMP =>
        return v25_sub_flags16(old_flags, a, b, '0');
    end case;
  end function;

  function shift_count_for_op(kind : op_kind_t; bank_regs : v25_word_regs_t; imm : word_t) return natural is
  begin
    case kind is
      when OP_GRP_SHIFT_RM8_1 | OP_GRP_SHIFT_RM16_1 =>
        return 1;
      when OP_GRP_SHIFT_RM8_CL | OP_GRP_SHIFT_RM16_CL =>
        return to_integer(unsigned(bank_regs(V25_REG_CX)(4 downto 0)));
      when others =>
        return to_integer(unsigned(imm(4 downto 0)));
    end case;
  end function;

  function resolved_modrm_timing_budget(
    kind : op_kind_t;
    alu_op : alu_func_t;
    subop : byte_t;
    modrm_value : byte_t;
    wait_states : natural;
    internal_selected : boolean;
    count : natural
  ) return natural is
    variable ea_clocks : natural := v25_ea_clocks(modrm_value);
    variable group_alu : alu_func_t := group_to_alu(modrm_value(5 downto 3));
    variable bit_op : v25_bit_timing_op_t := v25_bit_timing_op(subop);
  begin
    if not v25_modrm_is_memory(modrm_value) then
      return 0;
    end if;

    case kind is
      when OP_MOV_RM8_R8 =>
        return v25_clocks_mov_mem_reg(false, internal_selected, ea_clocks, wait_states);
      when OP_MOV_RM16_R16 | OP_MOV_RM16_SREG =>
        if kind = OP_MOV_RM16_SREG then
          return v25_clocks_mov_mem_sreg(internal_selected, ea_clocks, wait_states);
        end if;
        return v25_clocks_mov_mem_reg(true, internal_selected, ea_clocks, wait_states);
      when OP_MOV_R8_RM8 =>
        return v25_clocks_mov_reg_mem(false, ea_clocks, wait_states);
      when OP_MOV_R16_RM16 | OP_MOV_SREG_RM16 =>
        if kind = OP_MOV_SREG_RM16 then
          return v25_clocks_mov_sreg_mem(ea_clocks, wait_states);
        end if;
        return v25_clocks_mov_reg_mem(true, ea_clocks, wait_states);
      when OP_MOV_RM8_IMM8 =>
        return v25_clocks_mov_mem_imm(false, internal_selected, ea_clocks, wait_states);
      when OP_MOV_RM16_IMM16 =>
        return v25_clocks_mov_mem_imm(true, internal_selected, ea_clocks, wait_states);

      when OP_XCHG_RM8_R8 =>
        return v25_clocks_xchg_mem_reg(false, internal_selected, ea_clocks, wait_states);
      when OP_XCHG_RM16_R16 =>
        return v25_clocks_xchg_mem_reg(true, internal_selected, ea_clocks, wait_states);

      when OP_LEA_R16_MEM =>
        return v25_clocks_ldea(ea_clocks);
      when OP_MOV_DS0_R16_MEM32 | OP_MOV_DS1_R16_MEM32 =>
        return v25_clocks_mov_ds_reg_mem32(ea_clocks, wait_states);

      when OP_ALU_RM8_R8 =>
        if alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(false, ea_clocks, wait_states);
        end if;
        return v25_clocks_alu_mem_reg(false, internal_selected, ea_clocks, wait_states);
      when OP_ALU_RM16_R16 =>
        if alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(true, ea_clocks, wait_states);
        end if;
        return v25_clocks_alu_mem_reg(true, internal_selected, ea_clocks, wait_states);
      when OP_ALU_R8_RM8 =>
        if alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(false, ea_clocks, wait_states);
        end if;
        return v25_clocks_alu_reg_mem(false, ea_clocks, wait_states);
      when OP_ALU_R16_RM16 =>
        if alu_op = ALU_CMP then
          return v25_clocks_cmp_mem_reg(true, ea_clocks, wait_states);
        end if;
        return v25_clocks_alu_reg_mem(true, ea_clocks, wait_states);
      when OP_GRP_IMM8_RM8 =>
        if group_alu = ALU_CMP then
          return v25_clocks_cmp_mem_imm(false, ea_clocks, wait_states);
        elsif group_alu = ALU_ADD or group_alu = ALU_ADC or
              group_alu = ALU_SBB or group_alu = ALU_SUB then
          return v25_clocks_addsub_mem_imm(false, internal_selected, ea_clocks, wait_states);
        end if;
        return v25_clocks_logic_mem_imm(false, internal_selected, ea_clocks, wait_states);
      when OP_GRP_IMM16_RM16 | OP_GRP_IMM8_RM16_SIGN =>
        if group_alu = ALU_CMP then
          return v25_clocks_cmp_mem_imm(true, ea_clocks, wait_states);
        elsif group_alu = ALU_ADD or group_alu = ALU_ADC or
              group_alu = ALU_SBB or group_alu = ALU_SUB then
          return v25_clocks_addsub_mem_imm(true, internal_selected, ea_clocks, wait_states);
        end if;
        return v25_clocks_logic_mem_imm(true, internal_selected, ea_clocks, wait_states);

      when OP_TEST_RM8_R8 =>
        return v25_clocks_test_mem_reg(false, ea_clocks, wait_states);
      when OP_TEST_RM16_R16 =>
        return v25_clocks_test_mem_reg(true, ea_clocks, wait_states);
      when OP_GRP3_RM8 =>
        case modrm_value(5 downto 3) is
          when "000" =>
            return v25_clocks_test_mem_imm(false, ea_clocks, wait_states);
          when "010" | "011" =>
            return v25_clocks_notneg_mem(false, internal_selected, ea_clocks, wait_states);
          when "100" =>
            return v25_clocks_mulu_mem(false, ea_clocks, wait_states);
          when "101" =>
            return v25_clocks_mul_mem(false, ea_clocks, wait_states, true);
          when "110" =>
            return v25_clocks_divu_mem(false, ea_clocks, wait_states);
          when "111" =>
            return v25_clocks_div_mem(false, ea_clocks, wait_states, true);
          when others =>
            return 0;
        end case;
      when OP_GRP3_RM16 =>
        case modrm_value(5 downto 3) is
          when "000" =>
            return v25_clocks_test_mem_imm(true, ea_clocks, wait_states);
          when "010" | "011" =>
            return v25_clocks_notneg_mem(true, internal_selected, ea_clocks, wait_states);
          when "100" =>
            return v25_clocks_mulu_mem(true, ea_clocks, wait_states);
          when "101" =>
            return v25_clocks_mul_mem(true, ea_clocks, wait_states, true);
          when "110" =>
            return v25_clocks_divu_mem(true, ea_clocks, wait_states);
          when "111" =>
            return v25_clocks_div_mem(true, ea_clocks, wait_states, true);
          when others =>
            return 0;
        end case;
      when OP_IMUL_R16_RM16_IMM8 =>
        return v25_clocks_imul_imm_mem(true, ea_clocks, wait_states, true);
      when OP_IMUL_R16_RM16_IMM16 =>
        return v25_clocks_imul_imm_mem(false, ea_clocks, wait_states, true);

      when OP_GRP_SHIFT_RM8_1 =>
        return v25_clocks_shift_mem_1(false, internal_selected, ea_clocks, wait_states);
      when OP_GRP_SHIFT_RM16_1 =>
        return v25_clocks_shift_mem_1(true, internal_selected, ea_clocks, wait_states);
      when OP_GRP_SHIFT_RM8_CL =>
        return v25_clocks_shift_mem_cl(false, internal_selected, ea_clocks, wait_states, count);
      when OP_GRP_SHIFT_RM16_CL =>
        return v25_clocks_shift_mem_cl(true, internal_selected, ea_clocks, wait_states, count);
      when OP_GRP_SHIFT_RM8_IMM =>
        return v25_clocks_shift_mem_imm(false, internal_selected, ea_clocks, wait_states, count);
      when OP_GRP_SHIFT_RM16_IMM =>
        return v25_clocks_shift_mem_imm(true, internal_selected, ea_clocks, wait_states, count);

      when OP_GRP_FE =>
        if modrm_value(5 downto 3) = "000" or modrm_value(5 downto 3) = "001" then
          return v25_clocks_incdec_mem(false, internal_selected, ea_clocks, wait_states);
        end if;
      when OP_GRP_FF =>
        case modrm_value(5 downto 3) is
          when "000" | "001" =>
            return v25_clocks_incdec_mem(true, internal_selected, ea_clocks, wait_states);
          when "010" =>
            return v25_clocks_call_memptr16(internal_selected, ea_clocks, wait_states);
          when "011" =>
            return v25_clocks_call_memptr32(internal_selected, ea_clocks, wait_states);
          when "100" =>
            return v25_clocks_branch_memptr16(ea_clocks, wait_states);
          when "101" =>
            return v25_clocks_branch_memptr32(ea_clocks, wait_states);
          when "110" =>
            return v25_clocks_push_mem16(internal_selected, ea_clocks, wait_states);
          when others =>
            return 0;
        end case;
      when OP_POP_RM16 =>
        if modrm_value(5 downto 3) = "000" then
          return v25_clocks_pop_mem16(internal_selected, ea_clocks, wait_states);
        end if;

      when OP_CHKIND =>
        return v25_clocks_chkind(ea_clocks, wait_states);
      when OP_ROL4 =>
        return v25_clocks_rol4_mem(internal_selected, ea_clocks, wait_states);
      when OP_ROR4 =>
        return v25_clocks_ror4_mem(internal_selected, ea_clocks, wait_states);
      when OP_V25_BITOP =>
        if subop(3) = '1' then
          return v25_clocks_bitop_mem_imm(bit_op, subop(0) = '1',
            internal_selected, ea_clocks, wait_states);
        end if;
        return v25_clocks_bitop_mem_cl(bit_op, subop(0) = '1',
          internal_selected, ea_clocks, wait_states);

      when others =>
        return 0;
    end case;

    return 0;
  end function;

  function shift_rotate8(
    group_bits : std_logic_vector(2 downto 0);
    value : byte_t;
    count : natural;
    old_flags : v25_flags_t
  ) return shift8_result_t is
    variable result : shift8_result_t;
    variable r      : byte_t := value;
    variable f      : v25_flags_t := old_flags;
    variable c      : std_logic := old_flags.cf;
    variable c_in   : std_logic;
  begin
    if count = 0 then
      result.value := value;
      result.flags := old_flags;
      return result;
    end if;

    for i in 1 to 31 loop
      if i <= count then
        case group_bits is
          when "000" =>
            c := r(7);
            r := r(6 downto 0) & r(7);
          when "001" =>
            c := r(0);
            r := r(0) & r(7 downto 1);
          when "010" =>
            c_in := c;
            c := r(7);
            r := r(6 downto 0) & c_in;
          when "011" =>
            c_in := c;
            c := r(0);
            r := c_in & r(7 downto 1);
          when "100" | "110" =>
            c := r(7);
            r := r(6 downto 0) & '0';
          when "101" =>
            c := r(0);
            r := '0' & r(7 downto 1);
          when others =>
            c := r(0);
            r := r(7) & r(7 downto 1);
        end case;
      end if;
    end loop;

    f.cf := c;
    case group_bits is
      when "000" =>
        if count = 1 then
          f.oflag := r(7) xor c;
        end if;
      when "001" =>
        if count = 1 then
          f.oflag := r(7) xor r(6);
        end if;
      when "010" =>
        if count = 1 then
          f.oflag := r(7) xor c;
        end if;
      when "011" =>
        if count = 1 then
          f.oflag := r(7) xor r(6);
        end if;
      when "100" | "110" =>
        f.sf := r(7);
        if r = x"00" then
          f.zf := '1';
        else
          f.zf := '0';
        end if;
        f.pf := v25_even_parity8(r);
        if count = 1 then
          f.oflag := r(7) xor c;
        end if;
      when "101" =>
        f.sf := r(7);
        if r = x"00" then
          f.zf := '1';
        else
          f.zf := '0';
        end if;
        f.pf := v25_even_parity8(r);
        if count = 1 then
          f.oflag := value(7);
        end if;
      when others =>
        f.sf := r(7);
        if r = x"00" then
          f.zf := '1';
        else
          f.zf := '0';
        end if;
        f.pf := v25_even_parity8(r);
        if count = 1 then
          f.oflag := '0';
        end if;
    end case;

    result.value := r;
    result.flags := f;
    return result;
  end function;

  function shift_rotate16(
    group_bits : std_logic_vector(2 downto 0);
    value : word_t;
    count : natural;
    old_flags : v25_flags_t
  ) return shift16_result_t is
    variable result : shift16_result_t;
    variable r      : word_t := value;
    variable f      : v25_flags_t := old_flags;
    variable c      : std_logic := old_flags.cf;
    variable c_in   : std_logic;
  begin
    if count = 0 then
      result.value := value;
      result.flags := old_flags;
      return result;
    end if;

    for i in 1 to 31 loop
      if i <= count then
        case group_bits is
          when "000" =>
            c := r(15);
            r := r(14 downto 0) & r(15);
          when "001" =>
            c := r(0);
            r := r(0) & r(15 downto 1);
          when "010" =>
            c_in := c;
            c := r(15);
            r := r(14 downto 0) & c_in;
          when "011" =>
            c_in := c;
            c := r(0);
            r := c_in & r(15 downto 1);
          when "100" | "110" =>
            c := r(15);
            r := r(14 downto 0) & '0';
          when "101" =>
            c := r(0);
            r := '0' & r(15 downto 1);
          when others =>
            c := r(0);
            r := r(15) & r(15 downto 1);
        end case;
      end if;
    end loop;

    f.cf := c;
    case group_bits is
      when "000" =>
        if count = 1 then
          f.oflag := r(15) xor c;
        end if;
      when "001" =>
        if count = 1 then
          f.oflag := r(15) xor r(14);
        end if;
      when "010" =>
        if count = 1 then
          f.oflag := r(15) xor c;
        end if;
      when "011" =>
        if count = 1 then
          f.oflag := r(15) xor r(14);
        end if;
      when "100" | "110" =>
        f.sf := r(15);
        if r = x"0000" then
          f.zf := '1';
        else
          f.zf := '0';
        end if;
        f.pf := v25_even_parity8(r(7 downto 0));
        if count = 1 then
          f.oflag := r(15) xor c;
        end if;
      when "101" =>
        f.sf := r(15);
        if r = x"0000" then
          f.zf := '1';
        else
          f.zf := '0';
        end if;
        f.pf := v25_even_parity8(r(7 downto 0));
        if count = 1 then
          f.oflag := value(15);
        end if;
      when others =>
        f.sf := r(15);
        if r = x"0000" then
          f.zf := '1';
        else
          f.zf := '0';
        end if;
        f.pf := v25_even_parity8(r(7 downto 0));
        if count = 1 then
          f.oflag := '0';
        end if;
    end case;

    result.value := r;
    result.flags := f;
    return result;
  end function;

  function bcd_add_byte(dst : byte_t; src : byte_t; carry_in : std_logic) return bcd_byte_result_t is
    variable result : bcd_byte_result_t;
    variable digit  : natural range 0 to 19;
    variable carry  : std_logic := carry_in;
  begin
    digit := to_integer(unsigned(dst(3 downto 0))) + to_integer(unsigned(src(3 downto 0)));
    if carry = '1' then
      digit := digit + 1;
    end if;
    if digit > 9 then
      digit := digit - 10;
      carry := '1';
    else
      carry := '0';
    end if;
    result.value(3 downto 0) := std_logic_vector(to_unsigned(digit, 4));

    digit := to_integer(unsigned(dst(7 downto 4))) + to_integer(unsigned(src(7 downto 4)));
    if carry = '1' then
      digit := digit + 1;
    end if;
    if digit > 9 then
      digit := digit - 10;
      carry := '1';
    else
      carry := '0';
    end if;
    result.value(7 downto 4) := std_logic_vector(to_unsigned(digit, 4));
    result.carry := carry;
    return result;
  end function;

  function bcd_sub_byte(dst : byte_t; src : byte_t; borrow_in : std_logic) return bcd_byte_result_t is
    variable result : bcd_byte_result_t;
    variable digit  : integer range -10 to 9;
    variable borrow : std_logic := borrow_in;
  begin
    digit := to_integer(unsigned(dst(3 downto 0))) - to_integer(unsigned(src(3 downto 0)));
    if borrow = '1' then
      digit := digit - 1;
    end if;
    if digit < 0 then
      digit := digit + 10;
      borrow := '1';
    else
      borrow := '0';
    end if;
    result.value(3 downto 0) := std_logic_vector(to_unsigned(digit, 4));

    digit := to_integer(unsigned(dst(7 downto 4))) - to_integer(unsigned(src(7 downto 4)));
    if borrow = '1' then
      digit := digit - 1;
    end if;
    if digit < 0 then
      digit := digit + 10;
      borrow := '1';
    else
      borrow := '0';
    end if;
    result.value(7 downto 4) := std_logic_vector(to_unsigned(digit, 4));
    result.carry := borrow;
    return result;
  end function;

  function push_regs_value(bank_regs : v25_word_regs_t; idx : natural; saved_sp : word_t) return word_t is
  begin
    case idx is
      when 0 => return bank_regs(V25_REG_AX);
      when 1 => return bank_regs(V25_REG_CX);
      when 2 => return bank_regs(V25_REG_DX);
      when 3 => return bank_regs(V25_REG_BX);
      when 4 => return saved_sp;
      when 5 => return bank_regs(V25_REG_BP);
      when 6 => return bank_regs(V25_REG_SI);
      when others => return bank_regs(V25_REG_DI);
    end case;
  end function;

  function interrupt_vector_base(vector : byte_t) return addr20_t is
  begin
    return std_logic_vector(shift_left(resize(unsigned(vector), 20), 2));
  end function;

  function v25_internal_data_selected(addr : addr20_t; idb_value : byte_t; prc_value : byte_t) return boolean is
  begin
    if addr = x"FFFFF" then
      return true;
    end if;

    if addr(19 downto 12) /= idb_value or addr(11) /= '1' then
      return false;
    end if;

    -- Hardware manual Figure 3-5: SFR data accesses always take precedence;
    -- lower internal RAM data accesses only do so when PRC.RAMEN is set.
    return addr(8) = '1' or prc_value(6) = '1';
  end function;

  function v25_internal_data_index(addr : addr20_t) return natural is
  begin
    return to_integer(unsigned(addr(8 downto 0)));
  end function;

  function v25_sfr_index(sfr_number : byte_t) return natural is
  begin
    return 256 + to_integer(unsigned(sfr_number));
  end function;

  function port_read_value(values : internal_data_t; latch_index : natural; mode_index : natural; pins : byte_t)
    return byte_t is
  begin
    return (values(latch_index) and not values(mode_index)) or (pins and values(mode_index));
  end function;

  function poll_pin_waiting(values : internal_data_t; port1_pins : byte_t) return boolean is
  begin
    if values(SFR_PMC1_INDEX)(4) = '1' or values(SFR_PM1_INDEX)(4) = '1' then
      return port1_pins(4) = '1';
    end if;

    return false;
  end function;

  function flag_sfr_value(current_flags : v25_flags_t) return byte_t is
    variable value : byte_t := x"00";
  begin
    value(3) := current_flags.f0;
    value(5) := current_flags.f1;
    return value;
  end function;

  function interrupt_request_control_index(idx : natural) return boolean is
  begin
    case idx is
      when SFR_EXIC0_INDEX | SFR_EXIC1_INDEX | SFR_EXIC2_INDEX |
        SFR_SEIC0_INDEX | SFR_SRIC0_INDEX | SFR_STIC0_INDEX |
        SFR_SEIC1_INDEX | SFR_SRIC1_INDEX | SFR_STIC1_INDEX |
        SFR_TMIC0_INDEX | SFR_TMIC1_INDEX | SFR_TMIC2_INDEX |
        SFR_DIC0_INDEX | SFR_DIC1_INDEX | SFR_TBIC_INDEX =>
        return true;
      when others =>
        return false;
    end case;
  end function;

  function irq_control_visible_value(values : internal_data_t; idx : natural) return byte_t is
    variable value : byte_t := values(idx);
  begin
    case idx is
      when SFR_EXIC1_INDEX | SFR_EXIC2_INDEX =>
        value(2 downto 0) := "111";
      when SFR_TMIC1_INDEX | SFR_TMIC2_INDEX =>
        value(2 downto 0) := "111";
      when SFR_DIC1_INDEX =>
        value(2 downto 0) := "111";
      when SFR_SRIC0_INDEX | SFR_STIC0_INDEX |
        SFR_SRIC1_INDEX | SFR_STIC1_INDEX =>
        value(2 downto 0) := "111";
      when SFR_TBIC_INDEX =>
        value(5 downto 4) := "00";
        value(2 downto 0) := "111";
      when others =>
        null;
    end case;

    if interrupt_request_control_index(idx) then
      value(3) := '0';
    end if;
    return value;
  end function;

  function read_internal_byte(
    values     : internal_data_t;
    current_flags : v25_flags_t;
    bank_regs  : v25_bank_regs_t;
    vector_ip  : bank_word_t;
    saved_ip   : bank_word_t;
    saved_psw  : bank_word_t;
    ps_values  : bank_word_t;
    ss_values  : bank_word_t;
    ds0_values : bank_word_t;
    ds1_values : bank_word_t;
    port0_pins : byte_t;
    port1_pins : byte_t;
    port2_pins : byte_t;
    portt_pins : byte_t;
    serial0_rxd_pin : std_logic;
    serial1_rxd_pin : std_logic;
    idx        : natural
  ) return byte_t is
    variable bank        : natural range 0 to 7;
    variable offset      : natural range 0 to 31;
    variable word_offset : natural range 0 to 30;
    variable w           : word_t;
  begin
    if idx >= 256 then
      case idx is
        when SFR_P0_INDEX =>
          return port_read_value(values, SFR_P0_INDEX, SFR_PM0_INDEX, port0_pins);
        when SFR_P1_INDEX =>
          return port_read_value(values, SFR_P1_INDEX, SFR_PM1_INDEX, port1_pins);
        when SFR_P2_INDEX =>
          return port_read_value(values, SFR_P2_INDEX, SFR_PM2_INDEX, port2_pins);
        when SFR_PT_INDEX =>
          return port_read_value(values, SFR_PT_INDEX, SFR_PMT_INDEX, portt_pins);
        when SFR_DMAM0_INDEX | SFR_DMAM1_INDEX =>
          w := x"00" & values(idx);
          w(2) := '0';
          return w(7 downto 0);
        when SFR_SCE0_INDEX =>
          w := x"00" & values(idx);
          w(7) := serial0_rxd_pin;
          return w(7 downto 0);
        when SFR_SCE1_INDEX =>
          w := x"00" & values(idx);
          w(7) := serial1_rxd_pin;
          return w(7 downto 0);
        when SFR_STBC_INDEX =>
          w := x"00" & values(idx);
          w(7 downto 1) := (others => '0');
          return w(7 downto 0);
        when SFR_FLAG_INDEX =>
          return flag_sfr_value(current_flags);
        when others =>
          if interrupt_request_control_index(idx) then
            return irq_control_visible_value(values, idx);
          end if;
      end case;
      return values(idx);
    end if;

    bank := idx / 32;
    offset := idx mod 32;
    word_offset := offset - (offset mod 2);

    case word_offset is
      when 0 =>
        -- The V25 stack can use the nominally reserved word at each bank base.
        -- Keep reads coherent with the byte RAM written through this window.
        w := values((bank * 32) + 1) & values(bank * 32);
      when 2 =>
        w := vector_ip(bank);
      when 4 =>
        w := saved_psw(bank);
      when 6 =>
        w := saved_ip(bank);
      when 8 =>
        w := ds0_values(bank);
      when 10 =>
        w := ss_values(bank);
      when 12 =>
        w := ps_values(bank);
      when 14 =>
        w := ds1_values(bank);
      when 16 =>
        w := bank_regs(bank)(V25_REG_DI);
      when 18 =>
        w := bank_regs(bank)(V25_REG_SI);
      when 20 =>
        w := bank_regs(bank)(V25_REG_BP);
      when 22 =>
        w := bank_regs(bank)(V25_REG_SP);
      when 24 =>
        w := bank_regs(bank)(V25_REG_BX);
      when 26 =>
        w := bank_regs(bank)(V25_REG_DX);
      when 28 =>
        w := bank_regs(bank)(V25_REG_CX);
      when 30 =>
        w := bank_regs(bank)(V25_REG_AX);
      when others =>
        w := x"0000";
    end case;

    if (offset mod 2) = 0 then
      return w(7 downto 0);
    end if;

    return w(15 downto 8);
  end function;

  function stored_internal_byte(idx : natural; value : byte_t; current_value : byte_t) return byte_t is
    variable stored : byte_t := value;
  begin
    case idx is
      when SFR_PT_INDEX | SFR_RXB0_INDEX | SFR_RXB1_INDEX |
        SFR_SCE0_INDEX | SFR_SCE1_INDEX | SFR_ISPR_INDEX =>
        stored := current_value;
      when SFR_STBC_INDEX =>
        stored := "0000000" & (current_value(0) or value(0));
      when SFR_RFM_INDEX =>
        stored(7) := current_value(7);
      when SFR_FLAG_INDEX =>
        stored := x"00";
        stored(3) := value(3);
        stored(5) := value(5);
      when SFR_INTM_INDEX =>
        stored(7) := '0';
        stored(5) := '0';
        stored(3) := '0';
        stored(1) := '0';
      when SFR_DMAM0_INDEX | SFR_DMAM1_INDEX =>
        stored(1 downto 0) := "00";
      when SFR_DMAC0_INDEX | SFR_DMAC1_INDEX =>
        stored(7 downto 6) := "00";
        stored(3 downto 2) := "00";
      when SFR_TMC1_INDEX =>
        stored(5 downto 0) := "000000";
      when SFR_SCC0_INDEX | SFR_SCC1_INDEX =>
        stored(7 downto 4) := "0000";
      when SFR_PRC_INDEX =>
        stored(7) := '0';
        stored(5 downto 4) := "00";
      when SFR_EMS0_INDEX | SFR_EMS1_INDEX | SFR_EMS2_INDEX |
        SFR_SRMS0_INDEX | SFR_STMS0_INDEX |
        SFR_SRMS1_INDEX | SFR_STMS1_INDEX |
        SFR_TMMS0_INDEX | SFR_TMMS1_INDEX | SFR_TMMS2_INDEX =>
        stored(3) := '0';
      when SFR_EXIC1_INDEX | SFR_EXIC2_INDEX =>
        stored(3) := '0';
        stored(2 downto 0) := "111";
      when SFR_TMIC1_INDEX | SFR_TMIC2_INDEX =>
        stored(3) := '0';
        stored(2 downto 0) := "111";
      when SFR_SEIC0_INDEX | SFR_SEIC1_INDEX =>
        stored(5) := '0';
        stored(3) := '0';
      when SFR_SRIC0_INDEX | SFR_STIC0_INDEX |
        SFR_SRIC1_INDEX | SFR_STIC1_INDEX =>
        stored(3) := '0';
        stored(2 downto 0) := "111";
      when SFR_DIC0_INDEX =>
        stored(5) := '0';
        stored(3) := '0';
      when SFR_DIC1_INDEX =>
        stored(5) := '0';
        stored(3) := '0';
        stored(2 downto 0) := "111";
      when SFR_TBIC_INDEX =>
        stored(5 downto 4) := "00";
        stored(3) := '0';
        stored(2 downto 0) := "111";
      when others =>
        null;
    end case;
    if interrupt_request_control_index(idx) then
      stored(3) := '0';
    end if;
    return stored;
  end function;

  function wtc_selected_field(
    values     : internal_data_t;
    mem_active : std_logic;
    addr_value : addr20_t;
    io_active  : std_logic
  ) return wtc_field_t is
  begin
    if io_active = '1' then
      return values(SFR_WTC_INDEX + 1)(7 downto 6);
    end if;

    if mem_active = '1' then
      return wtc_memory_field(values, addr_value);
    end if;

    return values(SFR_WTC_INDEX)(1 downto 0);
  end function;

  function wtc_wait_count(field : wtc_field_t) return wait_count_t is
  begin
    return to_unsigned(wtc_field_wait_states(field), wait_count_t'length);
  end function;

  function wtc_ready_extend(field : wtc_field_t) return std_logic is
  begin
    if field = "11" then
      return '1';
    end if;
    return '0';
  end function;

  procedure write_internal_byte(
    signal values         : inout internal_data_t;
    signal current_flags  : inout v25_flags_t;
    signal idb_value      : inout byte_t;
    signal vector_ip      : inout bank_word_t;
    signal saved_ip       : inout bank_word_t;
    signal saved_psw      : inout bank_word_t;
    signal ps_values      : inout bank_word_t;
    signal ss_values      : inout bank_word_t;
    signal ds0_values     : inout bank_word_t;
    signal ds1_values     : inout bank_word_t;
    signal bank_regs      : inout v25_bank_regs_t;
    signal rfm_slave      : inout std_logic;
    signal ps_out         : out word_t;
    signal ss_out         : out word_t;
    signal ds0_out        : out word_t;
    signal ds1_out        : out word_t;
    current_bank          : in natural range 0 to 7;
    idx                   : in natural;
    value                 : in byte_t
  ) is
    variable bank        : natural range 0 to 7;
    variable offset      : natural range 0 to 31;
    variable word_offset : natural range 0 to 30;
    variable w           : word_t;
  begin
    values(idx) <= stored_internal_byte(idx, value, values(idx));

    if idx = SFR_RFM_INDEX then
      rfm_slave <= value(7);
    end if;

    if idx = SFR_FLAG_INDEX then
      current_flags.f0 <= value(3);
      current_flags.f1 <= value(5);
    end if;

    if idx = SFR_IDB_INDEX then
      idb_value <= value;
    end if;

    if idx = SFR_TXB0_INDEX and values(SFR_SCM0_INDEX)(7) = '1' then
      values(SFR_STIC0_INDEX)(7) <= '1';
    elsif idx = SFR_TXB1_INDEX and values(SFR_SCM1_INDEX)(7) = '1' then
      values(SFR_STIC1_INDEX)(7) <= '1';
    elsif idx = SFR_SCM0_INDEX and value(7) = '1' then
      values(SFR_STIC0_INDEX)(7) <= '1';
    elsif idx = SFR_SCM1_INDEX and value(7) = '1' then
      values(SFR_STIC1_INDEX)(7) <= '1';
    end if;

    if idx < 256 then
      bank := idx / 32;
      offset := idx mod 32;
      word_offset := offset - (offset mod 2);

      case word_offset is
        when 2 =>
          w := vector_ip(bank);
        when 4 =>
          w := saved_psw(bank);
        when 6 =>
          w := saved_ip(bank);
        when 8 =>
          w := ds0_values(bank);
        when 10 =>
          w := ss_values(bank);
        when 12 =>
          w := ps_values(bank);
        when 14 =>
          w := ds1_values(bank);
        when 16 =>
          w := bank_regs(bank)(V25_REG_DI);
        when 18 =>
          w := bank_regs(bank)(V25_REG_SI);
        when 20 =>
          w := bank_regs(bank)(V25_REG_BP);
        when 22 =>
          w := bank_regs(bank)(V25_REG_SP);
        when 24 =>
          w := bank_regs(bank)(V25_REG_BX);
        when 26 =>
          w := bank_regs(bank)(V25_REG_DX);
        when 28 =>
          w := bank_regs(bank)(V25_REG_CX);
        when 30 =>
          w := bank_regs(bank)(V25_REG_AX);
        when others =>
          w := x"0000";
      end case;

      if (offset mod 2) = 0 then
        w(7 downto 0) := value;
      else
        w(15 downto 8) := value;
      end if;

      case word_offset is
        when 2 =>
          vector_ip(bank) <= w;
        when 4 =>
          saved_psw(bank) <= w;
        when 6 =>
          saved_ip(bank) <= w;
        when 8 =>
          ds0_values(bank) <= w;
          if bank = current_bank then
            ds0_out <= w;
          end if;
        when 10 =>
          ss_values(bank) <= w;
          if bank = current_bank then
            ss_out <= w;
          end if;
        when 12 =>
          ps_values(bank) <= w;
          if bank = current_bank then
            ps_out <= w;
          end if;
        when 14 =>
          ds1_values(bank) <= w;
          if bank = current_bank then
            ds1_out <= w;
          end if;
        when 16 =>
          bank_regs(bank)(V25_REG_DI) <= w;
        when 18 =>
          bank_regs(bank)(V25_REG_SI) <= w;
        when 20 =>
          bank_regs(bank)(V25_REG_BP) <= w;
        when 22 =>
          bank_regs(bank)(V25_REG_SP) <= w;
        when 24 =>
          bank_regs(bank)(V25_REG_BX) <= w;
        when 26 =>
          bank_regs(bank)(V25_REG_DX) <= w;
        when 28 =>
          bank_regs(bank)(V25_REG_CX) <= w;
        when 30 =>
          bank_regs(bank)(V25_REG_AX) <= w;
        when others =>
          null;
      end case;
    end if;
  end procedure;

  procedure write_internal_word(
    signal values         : inout internal_data_t;
    signal current_flags  : inout v25_flags_t;
    signal idb_value      : inout byte_t;
    signal vector_ip      : inout bank_word_t;
    signal saved_ip       : inout bank_word_t;
    signal saved_psw      : inout bank_word_t;
    signal ps_values      : inout bank_word_t;
    signal ss_values      : inout bank_word_t;
    signal ds0_values     : inout bank_word_t;
    signal ds1_values     : inout bank_word_t;
    signal bank_regs      : inout v25_bank_regs_t;
    signal rfm_slave      : inout std_logic;
    signal ps_out         : out word_t;
    signal ss_out         : out word_t;
    signal ds0_out        : out word_t;
    signal ds1_out        : out word_t;
    current_bank          : in natural range 0 to 7;
    idx                   : in natural;
    value                 : in word_t
  ) is
    variable hi_idx      : natural range 0 to 511;
    variable bank        : natural range 0 to 7;
    variable offset      : natural range 0 to 31;
  begin
    hi_idx := (idx + 1) mod 512;

    values(idx) <= stored_internal_byte(idx, value(7 downto 0), values(idx));
    values(hi_idx) <= stored_internal_byte(hi_idx, value(15 downto 8), values(hi_idx));

    if idx = SFR_RFM_INDEX then
      rfm_slave <= value(7);
    elsif hi_idx = SFR_RFM_INDEX then
      rfm_slave <= value(15);
    end if;

    if idx = SFR_FLAG_INDEX then
      current_flags.f0 <= value(3);
      current_flags.f1 <= value(5);
    elsif hi_idx = SFR_FLAG_INDEX then
      current_flags.f0 <= value(11);
      current_flags.f1 <= value(13);
    end if;

    if idx = SFR_IDB_INDEX then
      idb_value <= value(7 downto 0);
    elsif hi_idx = SFR_IDB_INDEX then
      idb_value <= value(15 downto 8);
    end if;

    if idx = SFR_TXB0_INDEX and values(SFR_SCM0_INDEX)(7) = '1' then
      values(SFR_STIC0_INDEX)(7) <= '1';
    elsif idx = SFR_TXB1_INDEX and values(SFR_SCM1_INDEX)(7) = '1' then
      values(SFR_STIC1_INDEX)(7) <= '1';
    elsif idx = SFR_SCM0_INDEX and value(7) = '1' then
      values(SFR_STIC0_INDEX)(7) <= '1';
    elsif idx = SFR_SCM1_INDEX and value(7) = '1' then
      values(SFR_STIC1_INDEX)(7) <= '1';
    end if;
    if hi_idx = SFR_TXB0_INDEX and values(SFR_SCM0_INDEX)(7) = '1' then
      values(SFR_STIC0_INDEX)(7) <= '1';
    elsif hi_idx = SFR_TXB1_INDEX and values(SFR_SCM1_INDEX)(7) = '1' then
      values(SFR_STIC1_INDEX)(7) <= '1';
    elsif hi_idx = SFR_SCM0_INDEX and value(15) = '1' then
      values(SFR_STIC0_INDEX)(7) <= '1';
    elsif hi_idx = SFR_SCM1_INDEX and value(15) = '1' then
      values(SFR_STIC1_INDEX)(7) <= '1';
    end if;

    if idx < 256 and (idx mod 2) = 0 then
      bank := idx / 32;
      offset := idx mod 32;

      case offset is
        when 2 =>
          vector_ip(bank) <= value;
        when 4 =>
          saved_psw(bank) <= value;
        when 6 =>
          saved_ip(bank) <= value;
        when 8 =>
          ds0_values(bank) <= value;
          if bank = current_bank then
            ds0_out <= value;
          end if;
        when 10 =>
          ss_values(bank) <= value;
          if bank = current_bank then
            ss_out <= value;
          end if;
        when 12 =>
          ps_values(bank) <= value;
          if bank = current_bank then
            ps_out <= value;
          end if;
        when 14 =>
          ds1_values(bank) <= value;
          if bank = current_bank then
            ds1_out <= value;
          end if;
        when 16 =>
          bank_regs(bank)(V25_REG_DI) <= value;
        when 18 =>
          bank_regs(bank)(V25_REG_SI) <= value;
        when 20 =>
          bank_regs(bank)(V25_REG_BP) <= value;
        when 22 =>
          bank_regs(bank)(V25_REG_SP) <= value;
        when 24 =>
          bank_regs(bank)(V25_REG_BX) <= value;
        when 26 =>
          bank_regs(bank)(V25_REG_DX) <= value;
        when 28 =>
          bank_regs(bank)(V25_REG_CX) <= value;
        when 30 =>
          bank_regs(bank)(V25_REG_AX) <= value;
        when others =>
          null;
      end case;
    end if;
  end procedure;

  function timer_word(values : internal_data_t; idx : natural) return word_t is
  begin
    return values(idx + 1) & values(idx);
  end function;

  procedure set_timer_word(
    signal values : inout internal_data_t;
    idx           : in natural;
    value         : in word_t
  ) is
  begin
    values(idx) <= value(7 downto 0);
    values(idx + 1) <= value(15 downto 8);
  end procedure;

  procedure tick_timer_unit(
    signal values     : inout internal_data_t;
    signal timer_tout : inout std_logic;
    timer0_tick       : in std_logic;
    timer0_md_tick    : in std_logic;
    timer1_tick       : in std_logic
  ) is
    variable tm0       : word_t;
    variable md0       : word_t;
    variable tm1       : word_t;
    variable md1       : word_t;
    variable next_word : word_t;
    variable tmc0      : byte_t;
    variable tmc1      : byte_t;
    variable timer0_interval : boolean;
  begin
    tm0 := timer_word(values, SFR_TM0_INDEX);
    md0 := timer_word(values, SFR_MD0_INDEX);
    tm1 := timer_word(values, SFR_TM1_INDEX);
    md1 := timer_word(values, SFR_MD1_INDEX);
    tmc0 := values(SFR_TMC0_INDEX);
    tmc1 := values(SFR_TMC1_INDEX);
    timer0_interval := tmc0(1 downto 0) = "00";

    if timer0_tick = '1' and timer0_interval then
      if tmc0(7) = '1' and md0 /= x"0000" then
        if tm0 = x"0000" then
          set_timer_word(values, SFR_TM0_INDEX, md0);
        else
          next_word := std_logic_vector(unsigned(tm0) - 1);
          if next_word = x"0000" then
            values(SFR_TMIC0_INDEX)(7) <= '1';
            if tmc0(3) = '1' then
              timer_tout <= not timer_tout;
            end if;
            set_timer_word(values, SFR_TM0_INDEX, md0);
          else
            set_timer_word(values, SFR_TM0_INDEX, next_word);
          end if;
        end if;
      end if;
    elsif tmc0(1 downto 0) = "01" then
      if timer0_tick = '1' and tmc0(7) = '1' and tm0 /= x"0000" then
        next_word := std_logic_vector(unsigned(tm0) - 1);
        set_timer_word(values, SFR_TM0_INDEX, next_word);
        if next_word = x"0000" then
          values(SFR_TMIC0_INDEX)(7) <= '1';
          values(SFR_TMC0_INDEX)(7) <= '0';
          if tmc0(3) = '1' then
            timer_tout <= not timer_tout;
          end if;
        end if;
      end if;

      if timer0_md_tick = '1' and tmc0(5) = '1' and md0 /= x"0000" then
        next_word := std_logic_vector(unsigned(md0) - 1);
        set_timer_word(values, SFR_MD0_INDEX, next_word);
        if next_word = x"0000" then
          values(SFR_TMIC1_INDEX)(7) <= '1';
          values(SFR_TMC0_INDEX)(5) <= '0';
        end if;
      end if;
    end if;

    if timer1_tick = '1' and tmc1(7) = '1' and md1 /= x"0000" then
      if tm1 = x"0000" then
        set_timer_word(values, SFR_TM1_INDEX, md1);
      else
        next_word := std_logic_vector(unsigned(tm1) - 1);
        if next_word = x"0000" then
          if timer0_interval then
            values(SFR_TMIC1_INDEX)(7) <= '1';
          end if;
          values(SFR_TMIC2_INDEX)(7) <= '1';
          set_timer_word(values, SFR_TM1_INDEX, md1);
        else
          set_timer_word(values, SFR_TM1_INDEX, next_word);
        end if;
      end if;
    end if;
  end procedure;

  function dma_channel_base(channel : natural) return natural is
  begin
    return channel * 8;
  end function;

  function dma_dmac_index(channel : natural) return natural is
  begin
    if channel = 0 then
      return SFR_DMAC0_INDEX;
    end if;
    return SFR_DMAC1_INDEX;
  end function;

  function dma_dmam_index(channel : natural) return natural is
  begin
    if channel = 0 then
      return SFR_DMAM0_INDEX;
    end if;
    return SFR_DMAM1_INDEX;
  end function;

  function dma_dic_index(channel : natural) return natural is
  begin
    if channel = 0 then
      return SFR_DIC0_INDEX;
    end if;
    return SFR_DIC1_INDEX;
  end function;

  function dma_phys_addr(segment_hi : byte_t; offset : word_t) return addr20_t is
  begin
    return std_logic_vector(shift_left(resize(unsigned(segment_hi), 20), 12) + resize(unsigned(offset), 20));
  end function;

  function dma_mode_io_to_mem(mode : std_logic_vector(2 downto 0)) return boolean is
  begin
    return mode = "001" or mode = "101";
  end function;

  function dma_mode_mem_to_io(mode : std_logic_vector(2 downto 0)) return boolean is
  begin
    return mode = "010" or mode = "110";
  end function;

  function dma_mode_demand(mode : std_logic_vector(2 downto 0)) return boolean is
  begin
    return mode = "001" or mode = "010";
  end function;

  function dma_mode_one_transfer(mode : std_logic_vector(2 downto 0)) return boolean is
  begin
    return mode = "101" or mode = "110";
  end function;

  function dma_transfer_pending(
    values      : internal_data_t;
    channel     : natural;
    dmarq_edge  : std_logic;
    dmarq_level : std_logic
  ) return boolean is
    variable dmam_value : byte_t;
    variable mode       : std_logic_vector(2 downto 0);
    variable base       : natural;
  begin
    dmam_value := values(dma_dmam_index(channel));
    mode := dmam_value(7 downto 5);
    base := dma_channel_base(channel);
    if dmam_value(3) /= '1' or timer_word(values, base + 6) = x"0000" then
      return false;
    end if;

    if (mode = "000" or mode = "100") and (dmam_value(2) = '1' or dmarq_edge = '1') then
      return true;
    end if;

    if dma_mode_demand(mode) and dmarq_level = '1' then
      return true;
    end if;

    if dma_mode_one_transfer(mode) and dmarq_edge = '1' then
      return true;
    end if;

    return false;
  end function;

  function dma_io_ack_active(
    current_state  : state_t;
    active_channel : natural;
    channel        : natural;
    source_is_io   : std_logic;
    dest_is_io     : std_logic
  ) return std_logic is
  begin
    if active_channel = channel then
      if source_is_io = '1' and
        (current_state = ST_DMA_RD_REQ or current_state = ST_DMA_RD_WAIT) then
        return '1';
      elsif dest_is_io = '1' and
        (current_state = ST_DMA_WR_REQ or current_state = ST_DMA_WR_WAIT) then
        return '1';
      end if;
    end if;
    return '0';
  end function;

  function macro_control_supported(ctrl_index : natural) return boolean is
  begin
    return ctrl_index = SFR_EXIC0_INDEX or
      ctrl_index = SFR_EXIC1_INDEX or
      ctrl_index = SFR_EXIC2_INDEX or
      ctrl_index = SFR_SRIC0_INDEX or
      ctrl_index = SFR_STIC0_INDEX or
      ctrl_index = SFR_SRIC1_INDEX or
      ctrl_index = SFR_STIC1_INDEX or
      ctrl_index = SFR_TMIC0_INDEX or
      ctrl_index = SFR_TMIC1_INDEX or
      ctrl_index = SFR_TMIC2_INDEX;
  end function;

  function macro_control_index(ctrl_index : natural) return natural is
  begin
    case ctrl_index is
      when SFR_EXIC0_INDEX =>
        return SFR_EMS0_INDEX;
      when SFR_EXIC1_INDEX =>
        return SFR_EMS1_INDEX;
      when SFR_EXIC2_INDEX =>
        return SFR_EMS2_INDEX;
      when SFR_SRIC0_INDEX =>
        return SFR_SRMS0_INDEX;
      when SFR_STIC0_INDEX =>
        return SFR_STMS0_INDEX;
      when SFR_SRIC1_INDEX =>
        return SFR_SRMS1_INDEX;
      when SFR_STIC1_INDEX =>
        return SFR_STMS1_INDEX;
      when SFR_TMIC0_INDEX =>
        return SFR_TMMS0_INDEX;
      when SFR_TMIC1_INDEX =>
        return SFR_TMMS1_INDEX;
      when SFR_TMIC2_INDEX =>
        return SFR_TMMS2_INDEX;
      when others =>
        return SFR_EMS0_INDEX;
    end case;
  end function;

  function macro_normal_pending(values : internal_data_t; ctrl_index : natural) return boolean is
    variable macro_ctrl_index : natural;
    variable mode             : std_logic_vector(2 downto 0);
  begin
    if not macro_control_supported(ctrl_index) then
      return false;
    end if;

    macro_ctrl_index := macro_control_index(ctrl_index);
    mode := values(macro_ctrl_index)(7 downto 5);
    return values(ctrl_index)(5) = '1' and
      (mode = "000" or mode = "001" or mode = "100");
  end function;

  procedure start_interrupt_bank_switch(
    signal values        : inout internal_data_t;
    signal saved_ip      : inout bank_word_t;
    signal saved_psw     : inout bank_word_t;
    signal ps_values     : inout bank_word_t;
    signal ss_values     : inout bank_word_t;
    signal ds0_values    : inout bank_word_t;
    signal ds1_values    : inout bank_word_t;
    signal flags_out     : out v25_flags_t;
    signal previous_bank_out : out natural range 0 to 7;
    signal active_bank_out   : out natural range 0 to 7;
    signal ip_out        : out word_t;
    signal ps_out        : out word_t;
    signal ss_out        : out word_t;
    signal ds0_out       : out word_t;
    signal ds1_out       : out word_t;
    service_index        : in natural;
    priority             : in natural range 0 to 7;
    current_ip           : in word_t;
    current_flags        : in v25_flags_t;
    current_bank         : in natural range 0 to 7;
    current_ps           : in word_t;
    current_ss           : in word_t;
    current_ds0          : in word_t;
    current_ds1          : in word_t;
    vector_ip            : in bank_word_t
  ) is
    variable ispr_mask  : unsigned(7 downto 0);
    variable next_flags : v25_flags_t;
  begin
    ispr_mask := shift_left(to_unsigned(1, 8), priority);
    values(SFR_ISPR_INDEX) <= std_logic_vector(unsigned(values(SFR_ISPR_INDEX)) or ispr_mask);
    values(service_index)(7) <= '0';
    saved_ip(priority) <= current_ip;
    saved_psw(priority) <= v25_pack_psw(current_flags);
    ps_values(current_bank) <= current_ps;
    ss_values(current_bank) <= current_ss;
    ds0_values(current_bank) <= current_ds0;
    ds1_values(current_bank) <= current_ds1;

    next_flags := current_flags;
    next_flags.rb := std_logic_vector(to_unsigned(priority, 3));
    next_flags.iflag := '0';
    next_flags.ibrk := '0';
    flags_out <= next_flags;

    previous_bank_out <= current_bank;
    active_bank_out <= priority;
    ip_out <= vector_ip(priority);
    ps_out <= ps_values(priority);
    ss_out <= ss_values(priority);
    ds0_out <= ds0_values(priority);
    ds1_out <= ds1_values(priority);
  end procedure;

  function dma_next_offset(offset : word_t; mode_bits : std_logic_vector(1 downto 0); step : natural) return word_t is
  begin
    case mode_bits is
      when "01" =>
        return std_logic_vector(unsigned(offset) + to_unsigned(step, 16));
      when "10" =>
        return std_logic_vector(unsigned(offset) - to_unsigned(step, 16));
      when others =>
        return offset;
    end case;
  end function;

  function irq_control_requests(control : byte_t) return boolean is
  begin
    return control(7) = '1' and control(6) = '0';
  end function;

  function irq_priority_allowed(values : internal_data_t; priority : natural) return boolean is
  begin
    for active_priority in 0 to 7 loop
      if values(SFR_ISPR_INDEX)(active_priority) = '1' and priority >= active_priority then
        return false;
      end if;
    end loop;
    return true;
  end function;

  procedure consider_irq_source(
    values        : in internal_data_t;
    ctrl_index    : in natural;
    group_pri     : in natural;
    scan_pri      : in natural;
    vector_value  : in byte_t;
    take          : inout boolean;
    vector        : inout byte_t;
    priority      : inout natural;
    service_index : inout natural
  ) is
  begin
    if not take and group_pri = scan_pri and irq_control_requests(values(ctrl_index)) then
      take := true;
      vector := vector_value;
      priority := group_pri;
      service_index := ctrl_index;
    end if;
  end procedure;

  procedure select_internal_irq(
    values        : in internal_data_t;
    take          : out boolean;
    vector        : out byte_t;
    priority      : out natural;
    service_index : out natural
  ) is
    variable timer_priority   : natural range 0 to 7;
    variable dma_priority     : natural range 0 to 7;
    variable ext_priority     : natural range 0 to 7;
    variable serial0_priority : natural range 0 to 7;
    variable serial1_priority : natural range 0 to 7;
    variable take_v           : boolean;
    variable vector_v         : byte_t;
    variable priority_v       : natural;
    variable service_index_v  : natural;
  begin
    take_v := false;
    vector_v := x"00";
    priority_v := 7;
    service_index_v := SFR_ISPR_INDEX;

    timer_priority := to_integer(unsigned(values(SFR_TMIC0_INDEX)(2 downto 0)));
    dma_priority := to_integer(unsigned(values(SFR_DIC0_INDEX)(2 downto 0)));
    ext_priority := to_integer(unsigned(values(SFR_EXIC0_INDEX)(2 downto 0)));
    serial0_priority := to_integer(unsigned(values(SFR_SEIC0_INDEX)(2 downto 0)));
    serial1_priority := to_integer(unsigned(values(SFR_SEIC1_INDEX)(2 downto 0)));

    for scan_pri in 0 to 7 loop
      if irq_priority_allowed(values, scan_pri) then
        consider_irq_source(values, SFR_TMIC0_INDEX, timer_priority, scan_pri,
          IRQ_VEC_TIMER0, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_TMIC1_INDEX, timer_priority, scan_pri,
          IRQ_VEC_TIMER1, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_TMIC2_INDEX, timer_priority, scan_pri,
          IRQ_VEC_TIMER2, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_DIC0_INDEX, dma_priority, scan_pri,
          IRQ_VEC_DMA0, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_DIC1_INDEX, dma_priority, scan_pri,
          IRQ_VEC_DMA1, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_EXIC0_INDEX, ext_priority, scan_pri,
          IRQ_VEC_EX0, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_EXIC1_INDEX, ext_priority, scan_pri,
          IRQ_VEC_EX1, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_EXIC2_INDEX, ext_priority, scan_pri,
          IRQ_VEC_EX2, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_SEIC0_INDEX, serial0_priority, scan_pri,
          IRQ_VEC_SERIAL0_ERROR, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_SRIC0_INDEX, serial0_priority, scan_pri,
          IRQ_VEC_SERIAL0_RX, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_STIC0_INDEX, serial0_priority, scan_pri,
          IRQ_VEC_SERIAL0_TX, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_SEIC1_INDEX, serial1_priority, scan_pri,
          IRQ_VEC_SERIAL1_ERROR, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_SRIC1_INDEX, serial1_priority, scan_pri,
          IRQ_VEC_SERIAL1_RX, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_STIC1_INDEX, serial1_priority, scan_pri,
          IRQ_VEC_SERIAL1_TX, take_v, vector_v, priority_v, service_index_v);
        consider_irq_source(values, SFR_TBIC_INDEX, 7, scan_pri,
          IRQ_VEC_TIMEBASE, take_v, vector_v, priority_v, service_index_v);
      end if;

      if take_v then
        take := take_v;
        vector := vector_v;
        priority := priority_v;
        service_index := service_index_v;
        return;
      end if;
    end loop;

    take := take_v;
    vector := vector_v;
    priority := priority_v;
    service_index := service_index_v;
  end procedure;
begin
  mem_valid <= mem_valid_r;
  mem_write <= mem_write_r;
  mem_addr  <= mem_addr_r;
  mem_wdata <= mem_wdata_r;

  io_valid <= io_valid_r;
  io_write <= io_write_r;
  io_addr  <= io_addr_r;
  io_wdata <= io_wdata_r;
  int_ack_valid <= int_ack_valid_r;
  int_ack_second <= int_ack_second_r;

  halted <= '1' when state = ST_HALTED else '0';
  stop_mode <= halt_stop_mode when state = ST_HALTED else '0';
  fault  <= fault_r;

  debug_pc  <= v25_phys_addr(ps, ip);
  debug_psw <= v25_pack_psw(flags);
  debug_ax  <= regs(active_bank)(V25_REG_AX);
  debug_cx  <= regs(active_bank)(V25_REG_CX);
  debug_dx  <= regs(active_bank)(V25_REG_DX);
  debug_bx  <= regs(active_bank)(V25_REG_BX);
  debug_sp  <= regs(active_bank)(V25_REG_SP);

  port0_out <= internal_data(SFR_P0_INDEX);
  port1_out <= internal_data(SFR_P1_INDEX);
  port2_out <= internal_data(SFR_P2_INDEX);
  portt_out <= internal_data(SFR_PT_INDEX);
  port0_mode_control <= internal_data(SFR_PMC0_INDEX);
  port1_mode_control <= internal_data(SFR_PMC1_INDEX);
  port2_mode_control <= internal_data(SFR_PMC2_INDEX);
  -- Hardware manual Figure 9-4: ENTO=0 fixes TOUT at the inactive ALV level.
  tout_out <= internal_data(SFR_P1_INDEX)(5) when internal_data(SFR_PMC1_INDEX)(5) = '0' else
    not internal_data(SFR_TMC0_INDEX)(2) when internal_data(SFR_TMC0_INDEX)(3) = '0' else
    timer_tout_r;
  sfr_wait_states <= wtc_wait_count(
    wtc_selected_field(internal_data, mem_valid_r, mem_addr_r, io_valid_r));
  sfr_ready_extend <= wtc_ready_extend(
    wtc_selected_field(internal_data, mem_valid_r, mem_addr_r, io_valid_r));
  dma0_control <= internal_data(SFR_DMAC0_INDEX);
  dma0_mode <= internal_data(SFR_DMAM0_INDEX);
  dma0_start <= internal_data(SFR_DMAM0_INDEX)(2);
  dma0_hold <= internal_data(SFR_DMAM0_INDEX)(3);
  dmaaak0_n_out <= '0' when internal_data(SFR_PMC2_INDEX)(1) = '1' and
    dma_io_ack_active(state, dma_active_channel, 0, dma_source_is_io, dma_dest_is_io) = '1' else '1';
  tc0_n_out <= '0' when internal_data(SFR_PMC2_INDEX)(2) = '1' and
    internal_data(SFR_DIC0_INDEX)(7) = '1' else '1';
  dma1_control <= internal_data(SFR_DMAC1_INDEX);
  dma1_mode <= internal_data(SFR_DMAM1_INDEX);
  dma1_start <= internal_data(SFR_DMAM1_INDEX)(2);
  dma1_hold <= internal_data(SFR_DMAM1_INDEX)(3);
  dmaaak1_n_out <= '0' when internal_data(SFR_PMC2_INDEX)(4) = '1' and
    dma_io_ack_active(state, dma_active_channel, 1, dma_source_is_io, dma_dest_is_io) = '1' else '1';
  tc1_n_out <= '0' when internal_data(SFR_PMC2_INDEX)(5) = '1' and
    internal_data(SFR_DIC1_INDEX)(7) = '1' else '1';
  serial0_rxb <= internal_data(SFR_RXB0_INDEX);
  serial0_txb <= internal_data(SFR_TXB0_INDEX);
  serial0_mode <= internal_data(SFR_SCM0_INDEX);
  serial0_ctrl <= internal_data(SFR_SCC0_INDEX);
  serial0_brg <= internal_data(SFR_BRG0_INDEX);
  serial0_enable <= internal_data(SFR_SCE0_INDEX);
  serial1_rxb <= internal_data(SFR_RXB1_INDEX);
  serial1_txb <= internal_data(SFR_TXB1_INDEX);
  serial1_mode <= internal_data(SFR_SCM1_INDEX);
  serial1_ctrl <= internal_data(SFR_SCC1_INDEX);
  serial1_brg <= internal_data(SFR_BRG1_INDEX);
  serial1_enable <= internal_data(SFR_SCE1_INDEX);
  standby_control <= internal_data(SFR_STBC_INDEX);
  ram_control <= internal_data(SFR_RFM_INDEX);
  flag_control <= flag_sfr_value(flags);
  protect_control <= internal_data(SFR_PRC_INDEX);
  timebase_control <= irq_control_visible_value(internal_data, SFR_TBIC_INDEX);
  interrupt_mode <= internal_data(SFR_INTM_INDEX);
  interrupt_pending <= internal_data(SFR_ISPR_INDEX);
  ext_irq0_control <= irq_control_visible_value(internal_data, SFR_EXIC0_INDEX);
  ext_irq1_control <= irq_control_visible_value(internal_data, SFR_EXIC1_INDEX);
  ext_irq2_control <= irq_control_visible_value(internal_data, SFR_EXIC2_INDEX);
  timer0_irq_control <= irq_control_visible_value(internal_data, SFR_TMIC0_INDEX);
  timer1_irq_control <= irq_control_visible_value(internal_data, SFR_TMIC1_INDEX);
  timer2_irq_control <= irq_control_visible_value(internal_data, SFR_TMIC2_INDEX);
  timer0_control <= internal_data(SFR_TMC0_INDEX);
  timer1_control <= internal_data(SFR_TMC1_INDEX);
  dma0_irq_control <= irq_control_visible_value(internal_data, SFR_DIC0_INDEX);
  dma1_irq_control <= irq_control_visible_value(internal_data, SFR_DIC1_INDEX);
  serial0_error_irq_control <= irq_control_visible_value(internal_data, SFR_SEIC0_INDEX);
  serial0_rx_irq_control <= irq_control_visible_value(internal_data, SFR_SRIC0_INDEX);
  serial0_tx_irq_control <= irq_control_visible_value(internal_data, SFR_STIC0_INDEX);
  serial1_error_irq_control <= irq_control_visible_value(internal_data, SFR_SEIC1_INDEX);
  serial1_rx_irq_control <= irq_control_visible_value(internal_data, SFR_SRIC1_INDEX);
  serial1_tx_irq_control <= irq_control_visible_value(internal_data, SFR_STIC1_INDEX);

  process(clk)
    variable op_u          : unsigned(7 downto 0);
    variable rm_idx        : natural range 0 to 7;
    variable src_idx       : natural range 0 to 7;
    variable group_bits    : std_logic_vector(2 downto 0);
    variable a8            : byte_t;
    variable b8            : byte_t;
    variable r8            : byte_t;
    variable a16           : word_t;
    variable b16           : word_t;
    variable r16           : word_t;
    variable cx_next       : word_t;
    variable adjust_low    : boolean;
    variable adjust_high   : boolean;
    variable tmp_nat       : natural;
    variable prod16u       : unsigned(15 downto 0);
    variable prod32u       : unsigned(31 downto 0);
    variable prod16s       : signed(15 downto 0);
    variable prod32s       : signed(31 downto 0);
    variable dividend16u   : unsigned(15 downto 0);
    variable dividend32u   : unsigned(31 downto 0);
    variable quotient16u   : unsigned(15 downto 0);
    variable remainder16u  : unsigned(15 downto 0);
    variable quotient32u   : unsigned(31 downto 0);
    variable remainder32u  : unsigned(31 downto 0);
    variable dividend16s   : signed(15 downto 0);
    variable dividend32s   : signed(31 downto 0);
    variable quotient16s   : signed(15 downto 0);
    variable remainder16s  : signed(15 downto 0);
    variable quotient32s   : signed(31 downto 0);
    variable remainder32s  : signed(31 downto 0);
    variable f             : v25_flags_t;
    variable reg_bank      : v25_word_regs_t;
    variable sp_next       : word_t;
    variable shift_count   : natural range 0 to 31;
    variable shift8        : shift8_result_t;
    variable shift16       : shift16_result_t;
    variable bcd_result    : bcd_byte_result_t;
    variable bit_index     : natural range 0 to 15;
    variable bit_mask8     : unsigned(7 downto 0);
    variable bit_mask16    : unsigned(15 downto 0);
    variable field_window  : unsigned(31 downto 0);
    variable field_shifted : unsigned(31 downto 0);
    variable field_mask32  : unsigned(31 downto 0);
    variable field_mask16  : unsigned(15 downto 0);
    variable field_insert  : unsigned(31 downto 0);
    variable field_next_offset : natural range 0 to 31;
    variable field_bit_pos : natural range 0 to 31;
    variable disp_count    : natural range 0 to 2;
    variable ea_addr       : addr20_t;
    variable seg_sel       : seg_select_t;
    variable seg_value     : word_t;
    variable mem_byte      : byte_t;
    variable trap_started  : boolean;
    variable bit_was_set   : boolean;
    variable branch_taken  : boolean;
    variable internal_irq_take : boolean;
    variable internal_irq_vector : byte_t;
    variable internal_irq_priority : natural range 0 to 7;
    variable internal_irq_service_index : natural range 0 to 511;
    variable irq_ispr_mask : unsigned(7 downto 0);
    variable tb_next        : unsigned(19 downto 0);
    variable tb_tap         : std_logic;
    variable dmarq0_edge_v   : std_logic;
    variable dmarq1_edge_v   : std_logic;
    variable dmarq0_level_v  : std_logic;
    variable dmarq1_level_v  : std_logic;
    variable dma_base_v     : natural range 0 to 15;
    variable dma_dmac_v     : natural;
    variable dma_dmam_v     : natural;
    variable dma_dic_v      : natural;
    variable dma_mode_v      : std_logic_vector(2 downto 0);
    variable dma_tc_v       : word_t;
    variable dma_next_tc    : word_t;
    variable dma_src_offset : word_t;
    variable dma_dst_offset : word_t;
    variable dma_next_src_offset : word_t;
    variable dma_next_dst_offset : word_t;
    variable dma_step       : natural range 1 to 2;
    variable macro_sfr_idx_v : natural range 0 to 511;
    variable macro_msp_v     : word_t;
    variable macro_next_msp_v : word_t;
    variable macro_count_v   : byte_t;
    variable macro_next_count_v : byte_t;
    variable macro_value_v   : byte_t;
    variable macro_step_v    : natural range 1 to 2;
    variable macro_search_hit_v : boolean;
    variable reset_data_v    : internal_data_t;
    variable timing_budget_v  : natural;
    variable timing_wait_v    : natural;
    variable timing_wait2_v   : natural;
    variable timing_addr_v    : addr20_t;
    variable timing_internal_v : boolean;
    variable prefetch_target_ps : word_t;
    variable prefetch_target_ip : word_t;
    variable timer0_pending_v : natural range 0 to 255;
    variable timer0_md_pending_v : natural range 0 to 255;
    variable timer1_pending_v : natural range 0 to 255;
    variable timer0_tick_v : std_logic;
    variable timer0_md_tick_v : std_logic;
    variable timer1_tick_v : std_logic;

    procedure begin_mem_read_low(
      addr : in addr20_t;
      word_access : in std_logic
    ) is
    begin
      mem_op_addr <= addr;
      mem_is_word <= word_access;
      if v25_internal_data_selected(addr, idb_high, internal_data(SFR_PRC_INDEX)) then
        mem_valid_r <= '0';
        mem_write_r <= '0';
      else
        mem_valid_r <= '1';
        mem_write_r <= '0';
        mem_addr_r <= addr;
      end if;
      state <= ST_MEM_RD_LO_WAIT;
    end procedure;

    procedure begin_mem_write_low(
      addr : in addr20_t;
      value : in word_t;
      word_access : in std_logic
    ) is
    begin
      mem_op_addr <= addr;
      mem_value <= value;
      mem_is_word <= word_access;
      if v25_internal_data_selected(addr, idb_high, internal_data(SFR_PRC_INDEX)) then
        mem_valid_r <= '0';
        mem_write_r <= '0';
      else
        mem_valid_r <= '1';
        mem_write_r <= '1';
        mem_addr_r <= addr;
        mem_wdata_r <= value(7 downto 0);
      end if;
      state <= ST_MEM_WR_LO_WAIT;
    end procedure;

    procedure note_internal_read_side_effect(
      idx : in natural
    ) is
    begin
      if idx = SFR_RXB0_INDEX then
        serial0_rx_unread <= '0';
      elsif idx = SFR_RXB1_INDEX then
        serial1_rx_unread <= '0';
      end if;
    end procedure;

    procedure read_internal_index_byte(
      idx   : in natural;
      value : out byte_t
    ) is
    begin
      value := read_internal_byte(
        internal_data,
        flags,
        regs,
        bank_vector_ip,
        bank_saved_ip,
        bank_saved_psw,
        bank_ps,
        bank_ss,
        bank_ds0,
        bank_ds1,
        port0_in,
        port1_in,
        port2_in,
        portt_in,
        serial0_rxd_in,
        serial1_rxd_in,
        idx
      );
      note_internal_read_side_effect(idx);
    end procedure;

    procedure read_internal_data_byte(
      addr : in addr20_t;
      value : out byte_t
    ) is
    begin
      read_internal_index_byte(v25_internal_data_index(addr), value);
    end procedure;

    procedure write_internal_data_byte(
      addr : in addr20_t;
      value : in byte_t
    ) is
    begin
      write_internal_byte(
        internal_data,
        flags,
        idb_high,
        bank_vector_ip,
        bank_saved_ip,
        bank_saved_psw,
        bank_ps,
        bank_ss,
        bank_ds0,
        bank_ds1,
        regs,
        rfm_rflv_slave,
        ps,
        ss,
        ds0,
        ds1,
        active_bank,
        v25_internal_data_index(addr),
        value
      );
    end procedure;

    procedure select_string_memory_timing(
      kind : in op_kind_t;
      wait_states : out natural;
      onchip_access : out boolean
    ) is
      variable start_kind : op_kind_t := repeat_start_kind(kind);
      variable source_addr : addr20_t := (others => '0');
      variable dest_addr   : addr20_t := (others => '0');
      variable dest_wait   : natural := 0;
      variable wait_states_v : natural := 0;
    begin
      wait_states_v := 0;
      onchip_access := false;

      case start_kind is
        when OP_MOVS8 | OP_MOVS16 | OP_CMPS8 | OP_CMPS16 =>
          source_addr := v25_phys_addr(
            v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
            regs(active_bank)(V25_REG_SI)
          );
          dest_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
          wait_states_v := current_memory_wait_states(internal_data, source_addr);
          dest_wait := current_memory_wait_states(internal_data, dest_addr);
          if dest_wait > wait_states_v then
            wait_states_v := dest_wait;
          end if;
          onchip_access :=
            v25_internal_data_selected(source_addr, idb_high, internal_data(SFR_PRC_INDEX)) or
            v25_internal_data_selected(dest_addr, idb_high, internal_data(SFR_PRC_INDEX));

        when OP_STOS8 | OP_STOS16 | OP_SCAS8 | OP_SCAS16 =>
          dest_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
          wait_states_v := current_memory_wait_states(internal_data, dest_addr);
          onchip_access :=
            v25_internal_data_selected(dest_addr, idb_high, internal_data(SFR_PRC_INDEX));

        when OP_LODS8 | OP_LODS16 =>
          source_addr := v25_phys_addr(
            v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
            regs(active_bank)(V25_REG_SI)
          );
          wait_states_v := current_memory_wait_states(internal_data, source_addr);
          onchip_access :=
            v25_internal_data_selected(source_addr, idb_high, internal_data(SFR_PRC_INDEX));

        when others =>
          null;
      end case;
      wait_states := wait_states_v;
    end procedure;

    procedure select_primitive_io_timing(
      kind : in op_kind_t;
      wait_states : out natural;
      onchip_access : out boolean
    ) is
      variable data_addr : addr20_t := (others => '0');
      variable mem_wait  : natural := 0;
      variable wait_states_v : natural := 0;
    begin
      wait_states_v := current_io_wait_states(internal_data);
      onchip_access := false;

      case kind is
        when OP_INM8 | OP_INM16 =>
          data_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));

        when OP_OUTM8 | OP_OUTM16 =>
          data_addr := v25_phys_addr(
            v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
            regs(active_bank)(V25_REG_SI)
          );

        when others =>
          null;
      end case;

      if is_primitive_io_block_op(kind) then
        mem_wait := current_memory_wait_states(internal_data, data_addr);
        if mem_wait > wait_states_v then
          wait_states_v := mem_wait;
        end if;
        onchip_access :=
          v25_internal_data_selected(data_addr, idb_high, internal_data(SFR_PRC_INDEX));
      end if;
      wait_states := wait_states_v;
    end procedure;

    procedure merge_memory_timing(
      addr : in addr20_t;
      wait_states : inout natural;
      onchip_access : inout boolean
    ) is
      variable addr_wait : natural := 0;
    begin
      addr_wait := current_memory_wait_states(internal_data, addr);
      if addr_wait > wait_states then
        wait_states := addr_wait;
      end if;

      onchip_access :=
        onchip_access or
        v25_internal_data_selected(addr, idb_high, internal_data(SFR_PRC_INDEX));
    end procedure;

    procedure merge_memory_word_timing(
      seg_word : in word_t;
      offset_value : in word_t;
      wait_states : inout natural;
      onchip_access : inout boolean
    ) is
    begin
      merge_memory_timing(
        v25_phys_addr(seg_word, offset_value),
        wait_states,
        onchip_access
      );
      merge_memory_timing(
        v25_phys_addr(seg_word, std_logic_vector(unsigned(offset_value) + 1)),
        wait_states,
        onchip_access
      );
    end procedure;

    procedure merge_stack_words_timing(
      first_offset : in word_t;
      word_count : in natural;
      descending : in boolean;
      wait_states : inout natural;
      onchip_access : inout boolean
    ) is
      variable offset_value : word_t := first_offset;
    begin
      for index in 0 to 7 loop
        if index < word_count then
          merge_memory_word_timing(ss, offset_value, wait_states, onchip_access);
          if descending then
            offset_value := std_logic_vector(unsigned(offset_value) - 2);
          else
            offset_value := std_logic_vector(unsigned(offset_value) + 2);
          end if;
        end if;
      end loop;
    end procedure;

    procedure select_stack_words_timing(
      first_offset : in word_t;
      word_count : in natural;
      descending : in boolean;
      wait_states : out natural;
      onchip_access : out boolean
    ) is
      variable wait_states_v : natural := 0;
      variable onchip_access_v : boolean := false;
    begin
      wait_states_v := 0;
      onchip_access_v := false;
      merge_stack_words_timing(
        first_offset,
        word_count,
        descending,
        wait_states_v,
        onchip_access_v
      );
      wait_states := wait_states_v;
      onchip_access := onchip_access_v;
    end procedure;

    procedure select_prepare_timing(
      level : in natural;
      wait_states : out natural;
      onchip_access : out boolean
    ) is
      variable copy_reads : natural := 0;
      variable stack_words : natural := 1;
      variable read_offset : word_t;
      variable max_wait : natural := 0;
      variable wait_states_v : natural := 0;
      variable onchip_access_v : boolean := false;
    begin
      wait_states_v := 0;
      onchip_access_v := false;

      if level = 1 then
        stack_words := 2;
      elsif level > 1 then
        if level < 8 then
          stack_words := level + 1;
          copy_reads := level - 1;
        else
          stack_words := 8;
          copy_reads := 7;
        end if;
      end if;

      merge_stack_words_timing(
        std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2),
        stack_words,
        true,
        wait_states_v,
        onchip_access_v
      );

      read_offset := std_logic_vector(unsigned(regs(active_bank)(V25_REG_BP)) - 2);
      for index in 0 to 6 loop
        if index < copy_reads then
          merge_memory_word_timing(
            ss,
            read_offset,
            wait_states_v,
            onchip_access_v
          );
          read_offset := std_logic_vector(unsigned(read_offset) - 2);
        end if;
      end loop;

      if level >= 8 then
        max_wait := current_max_memory_wait_states(internal_data);
        if max_wait > wait_states_v then
          wait_states_v := max_wait;
        end if;
      end if;

      wait_states := wait_states_v;
      onchip_access := onchip_access_v;
    end procedure;

    procedure merge_interrupt_vector_timing(
      vector_base : in addr20_t;
      wait_states : inout natural;
      onchip_access : inout boolean
    ) is
    begin
      for index in 0 to 3 loop
        merge_memory_timing(
          std_logic_vector(unsigned(vector_base) + index),
          wait_states,
          onchip_access
        );
      end loop;
    end procedure;

    procedure select_interrupt_entry_timing(
      vector_base : in addr20_t;
      wait_states : out natural;
      onchip_access : out boolean
    ) is
      variable wait_states_v : natural := 0;
      variable onchip_access_v : boolean := false;
    begin
      wait_states_v := 0;
      onchip_access_v := false;
      merge_interrupt_vector_timing(vector_base, wait_states_v, onchip_access_v);
      merge_stack_words_timing(
        std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2),
        3,
        true,
        wait_states_v,
        onchip_access_v
      );
      wait_states := wait_states_v;
      onchip_access := onchip_access_v;
    end procedure;

    procedure start_macro_service(
      service_index : in natural
    ) is
      variable ctrl_index   : natural range 0 to 511;
      variable channel_base : natural range 0 to 63;
      variable sfr_index    : natural range 0 to 511;
      variable sfr_value    : byte_t;
    begin
      ctrl_index := macro_control_index(service_index);
      channel_base := to_integer(unsigned(internal_data(ctrl_index)(2 downto 0))) * 8;
      sfr_index := v25_sfr_index(internal_data(channel_base + 1));

      macro_irq_ctrl_index <= service_index;
      macro_channel_base <= channel_base;
      macro_sfr_index <= sfr_index;
      macro_word_mode <= internal_data(ctrl_index)(5);
      macro_high_phase <= '0';
      macro_search_mode <= internal_data(ctrl_index)(7);
      macro_mem_addr <= v25_phys_addr(
        timer_word(internal_data, channel_base + 6),
        timer_word(internal_data, channel_base + 4)
      );

      if internal_data(ctrl_index)(4) = '0' then
        state <= ST_MACRO_RD_REQ;
      else
        sfr_value := read_internal_byte(
          internal_data,
          flags,
          regs,
          bank_vector_ip,
          bank_saved_ip,
          bank_saved_psw,
          bank_ps,
          bank_ss,
          bank_ds0,
          bank_ds1,
          port0_in,
          port1_in,
          port2_in,
          portt_in,
          serial0_rxd_in,
          serial1_rxd_in,
          sfr_index
        );
        macro_data <= sfr_value;
        if sfr_index = SFR_RXB0_INDEX then
          serial0_rx_unread <= '0';
        elsif sfr_index = SFR_RXB1_INDEX then
          serial1_rx_unread <= '0';
        end if;
        state <= ST_MACRO_WR_REQ;
      end if;
    end procedure;

    impure function prefetch_head_matches return boolean is
    begin
      return
        prefetch_count > 0 and
        prefetch_ps_queue(0) = ps and
        prefetch_ip_queue(0) = ip;
    end function;

    procedure clear_prefetch_queue is
    begin
      prefetch_count <= 0;
      prefetch_pending <= '0';
      prefetch_bytes <= (others => x"00");
      prefetch_ps_queue <= (others => RESET_PS);
      prefetch_ip_queue <= (others => RESET_IP);
      prefetch_pending_ps <= RESET_PS;
      prefetch_pending_ip <= RESET_IP;
    end procedure;

    procedure drop_prefetch_head is
    begin
      if prefetch_count > 1 then
        for i in 0 to PREFETCH_QUEUE_DEPTH - 2 loop
          if i < prefetch_count - 1 then
            prefetch_bytes(i) <= prefetch_bytes(i + 1);
            prefetch_ps_queue(i) <= prefetch_ps_queue(i + 1);
            prefetch_ip_queue(i) <= prefetch_ip_queue(i + 1);
          else
            prefetch_bytes(i) <= x"00";
            prefetch_ps_queue(i) <= RESET_PS;
            prefetch_ip_queue(i) <= RESET_IP;
          end if;
        end loop;
        prefetch_bytes(PREFETCH_QUEUE_DEPTH - 1) <= x"00";
        prefetch_ps_queue(PREFETCH_QUEUE_DEPTH - 1) <= RESET_PS;
        prefetch_ip_queue(PREFETCH_QUEUE_DEPTH - 1) <= RESET_IP;
        prefetch_count <= prefetch_count - 1;
      else
        prefetch_bytes <= (others => x"00");
        prefetch_ps_queue <= (others => RESET_PS);
        prefetch_ip_queue <= (others => RESET_IP);
        prefetch_count <= 0;
      end if;
    end procedure;

    procedure enqueue_prefetch_byte(
      fetched_byte : in byte_t
    ) is
    begin
      if prefetch_count < PREFETCH_QUEUE_DEPTH then
        prefetch_bytes(prefetch_count) <= fetched_byte;
        prefetch_ps_queue(prefetch_count) <= prefetch_pending_ps;
        prefetch_ip_queue(prefetch_count) <= prefetch_pending_ip;
        prefetch_count <= prefetch_count + 1;
      end if;
      prefetch_pending <= '0';
    end procedure;

    procedure finish_imm_fetch(
      fetched_byte : in byte_t
    ) is
    begin
      mem_valid_r <= '0';

      case imm_index is
        when 0 =>
          imm16(7 downto 0) <= fetched_byte;
          if ENABLE_TIMING_THROTTLE and op_kind = OP_V25_PREFIX then
            timing_counter <= v25_subop_timing_budget(fetched_byte);
          end if;
        when 1 =>
          imm16(15 downto 8) <= fetched_byte;
        when 2 =>
          far_seg(7 downto 0) <= fetched_byte;
        when others =>
          far_seg(15 downto 8) <= fetched_byte;
      end case;

      ip <= std_logic_vector(unsigned(ip) + 1);

      if imm_index + 1 >= imm_needed then
        imm_index <= 0;
        state     <= ST_EXECUTE;
      else
        imm_index <= imm_index + 1;
        state     <= ST_IMM_REQ;
      end if;
    end procedure;

    procedure finish_modrm_fetch(
      fetched_byte : in byte_t
    ) is
    begin
      mem_valid_r <= '0';
      modrm       <= fetched_byte;
      if ENABLE_TIMING_THROTTLE then
        timing_budget_v := modrm_timing_budget(op_kind, alu_func, v25_subop, fetched_byte);
        if timing_budget_v /= 0 then
          timing_counter <= timing_budget_v;
        end if;
      end if;
      ip          <= std_logic_vector(unsigned(ip) + 1);
      disp16      <= x"0000";
      disp_index  <= 0;
      disp_count  := v25_modrm_disp_size(fetched_byte);
      disp_needed <= disp_count;

      if disp_count /= 0 then
        state <= ST_DISP_REQ;
      elsif op_needs_imm_after_modrm(op_kind, v25_subop, fetched_byte) then
        if op_kind = OP_MOV_RM16_IMM16 or
           op_kind = OP_IMUL_R16_RM16_IMM16 or
           op_kind = OP_GRP_IMM16_RM16 or
           op_kind = OP_GRP3_RM16 then
          imm_needed <= 2;
        else
          imm_needed <= 1;
        end if;
        imm_index <= 0;
        imm16     <= x"0000";
        state     <= ST_IMM_REQ;
      else
        state <= ST_EXECUTE;
      end if;
    end procedure;

    procedure finish_disp_fetch(
      fetched_byte : in byte_t
    ) is
    begin
      mem_valid_r <= '0';

      if disp_index = 0 then
        disp16(7 downto 0) <= fetched_byte;
      else
        disp16(15 downto 8) <= fetched_byte;
      end if;

      ip <= std_logic_vector(unsigned(ip) + 1);

      if disp_index + 1 >= disp_needed then
        disp_index <= 0;
        if op_needs_imm_after_modrm(op_kind, v25_subop, modrm) then
          if op_kind = OP_MOV_RM16_IMM16 or
             op_kind = OP_IMUL_R16_RM16_IMM16 or
             op_kind = OP_GRP_IMM16_RM16 or
             op_kind = OP_GRP3_RM16 then
            imm_needed <= 2;
          else
            imm_needed <= 1;
          end if;
          imm_index <= 0;
          imm16     <= x"0000";
          state     <= ST_IMM_REQ;
        else
          state <= ST_EXECUTE;
        end if;
      else
        disp_index <= disp_index + 1;
        state      <= ST_DISP_REQ;
      end if;
    end procedure;

    procedure finish_opcode_fetch(
      fetched_opcode : in byte_t
    ) is
    begin
      mem_valid_r <= '0';
      opcode <= fetched_opcode;
      if ENABLE_TIMING_THROTTLE then
        timing_counter <= opcode_timing_budget(fetched_opcode);
      end if;
      if FIXED_INSTRUCTION_BUDGET > 0 then
        fixed_timing_counter <= FIXED_INSTRUCTION_BUDGET;
      end if;
      instr_start_ip <= ip;
      ip <= std_logic_vector(unsigned(ip) + 1);
      if seg_override /= SEG_DEFAULT then
        seg_override_consumed <= '1';
      end if;
      if rep_mode /= REP_NONE then
        rep_mode_consumed <= '1';
      end if;
      state <= ST_DECODE;
    end procedure;
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        state       <= ST_FETCH_REQ;
        op_kind     <= OP_NONE;
        alu_func    <= ALU_ADD;
        opcode      <= x"00";
        modrm       <= x"00";
        imm16       <= x"0000";
        imm_needed  <= 0;
        imm_index   <= 0;
        disp16      <= x"0000";
        disp_needed <= 0;
        disp_index  <= 0;
        op_reg      <= 0;
        v25_subop   <= x"00";
        regs        <= (others => (others => (others => '0')));
        active_bank <= RESET_BANK;
        previous_bank <= RESET_BANK;
        bank_saved_ip <= (others => x"0000");
        bank_saved_psw <= reset_saved_psw_values;
        bank_vector_ip <= (others => x"0000");
        bank_ps <= (others => RESET_PS);
        bank_ss <= (others => x"0000");
        bank_ds0 <= (others => x"0000");
        bank_ds1 <= (others => x"0000");
        reset_data_v := reset_internal_data;
        reset_data_v(SFR_DMAC0_INDEX) := internal_data(SFR_DMAC0_INDEX);
        reset_data_v(SFR_DMAC1_INDEX) := internal_data(SFR_DMAC1_INDEX);
        internal_data <= reset_data_v;
        rfm_rflv_slave <= '1';
        idb_high <= x"FF";
        nmi_prev <= nmi_in;
        nmi_pending <= '0';
        intp0_prev <= intp0_in;
        intp1_prev <= intp1_in;
        intp2_prev <= intp2_in;
        dmarq0_prev <= dmarq0_in;
        dmarq1_prev <= dmarq1_in;
        halt_stop_mode <= '0';
        serial0_rx_unread <= '0';
        serial1_rx_unread <= '0';
        timebase_counter <= (others => '0');
        timebase_tap_prev <= '0';
        timer_tout_r <= '1';
        tmc0_prev <= x"00";
        timer0_tick_pending <= 0;
        timer0_md_tick_pending <= 0;
        timer1_tick_pending <= 0;
        timing_counter <= 0;
        fixed_timing_counter <= 0;
        prefetch_count <= 0;
        prefetch_pending <= '0';
        prefetch_bytes <= (others => x"00");
        prefetch_ps_queue <= (others => RESET_PS);
        prefetch_ip_queue <= (others => RESET_IP);
        prefetch_pending_ps <= RESET_PS;
        prefetch_pending_ip <= RESET_IP;
        poll_wait_count <= 0;
        flags       <= reset_flags_for_bank(RESET_BANK);
        ps          <= RESET_PS;
        ss          <= x"0000";
        ds0         <= x"0000";
        ds1         <= x"0000";
        ip          <= RESET_IP;
        instr_start_ip <= RESET_IP;
        seg_override <= SEG_DEFAULT;
        seg_override_consumed <= '0';
        rep_mode <= REP_NONE;
        rep_mode_consumed <= '0';
        rep_timing_loaded <= '0';
        push_value  <= x"0000";
        push_mode   <= PUSH_ONLY;
        push_regs_index <= 0;
        push_sp_save <= x"0000";
        prepare_temp <= x"0000";
        prepare_copy_bp <= x"0000";
        prepare_level <= 0;
        prepare_remaining <= 0;
        pop_mode    <= POP_TO_REG;
        pop_target  <= 0;
        pop_regs_index <= 0;
        pop_low     <= x"00";
        branch_ip   <= x"0000";
        far_seg     <= x"0000";
        int_vector_base <= (others => '0');
        external_irq_vector <= x"00";
        int_target_ip <= x"0000";
        int_target_ps <= x"0000";
        int_return_ip <= x"0000";
        int_ibrk_after <= '0';
        int_ack_valid_r <= '0';
        int_ack_second_r <= '0';
        stack_adjust <= x"0000";
        mem_op_addr <= (others => '0');
        mem_low     <= x"00";
        mem_value   <= x"0000";
        mem_is_word <= '0';
        mem_target  <= 0;
        mem_sreg_target <= SEG_DEFAULT;
        bcd_src_addr <= (others => '0');
        bcd_dst_addr <= (others => '0');
        bcd_src_byte <= x"00";
        bcd_index    <= 0;
        bcd_total    <= 0;
        bcd_carry    <= '0';
        bcd_zero     <= '1';
        mem_valid_r <= '0';
        mem_write_r <= '0';
        mem_addr_r  <= (others => '0');
        mem_wdata_r <= x"00";
        io_valid_r <= '0';
        io_write_r <= '0';
        io_addr_r  <= x"0000";
        io_wdata_r <= x"00";
        io_low     <= x"00";
        dma_active_channel <= 0;
        dma_src_addr <= (others => '0');
        dma_dst_addr <= (others => '0');
        dma_data <= x"00";
        dma_word_mode <= '0';
        dma_high_phase <= '0';
        dma_source_is_io <= '0';
        dma_dest_is_io <= '0';
        macro_irq_ctrl_index <= SFR_EXIC0_INDEX;
        macro_channel_base <= 0;
        macro_mem_addr <= (others => '0');
        macro_sfr_index <= SFR_P0_INDEX;
        macro_data <= x"00";
        macro_word_mode <= '0';
        macro_high_phase <= '0';
        macro_search_mode <= '0';
        field_addr <= (others => '0');
        field_b0 <= x"00";
        field_b1 <= x"00";
        field_b2 <= x"00";
        field_b3 <= x"00";
        field_offset <= 0;
        field_length <= 1;
        field_reg <= 0;
        fault_r     <= '0';
      elsif clock_enable = '1' then
        op_u := unsigned(opcode);
        trap_started := false;
        timer0_pending_v := timer0_tick_pending;
        timer0_md_pending_v := timer0_md_tick_pending;
        timer1_pending_v := timer1_tick_pending;
        if timer0_tick_in = '1' and timer0_pending_v < 255 then
          timer0_pending_v := timer0_pending_v + 1;
        end if;
        if timer0_md_tick_in = '1' and timer0_md_pending_v < 255 then
          timer0_md_pending_v := timer0_md_pending_v + 1;
        end if;
        if timer1_tick_in = '1' and timer1_pending_v < 255 then
          timer1_pending_v := timer1_pending_v + 1;
        end if;
        dmarq0_level_v := dmarq0_in and internal_data(SFR_PMC2_INDEX)(0);
        dmarq1_level_v := dmarq1_in and internal_data(SFR_PMC2_INDEX)(3);
        if dmarq0_prev = '0' and dmarq0_level_v = '1' then
          dmarq0_edge_v := '1';
        else
          dmarq0_edge_v := '0';
        end if;
        if dmarq1_prev = '0' and dmarq1_level_v = '1' then
          dmarq1_edge_v := '1';
        else
          dmarq1_edge_v := '0';
        end if;
        if ENABLE_TIMING_THROTTLE and timing_counter > 0 then
          timing_counter <= timing_counter - 1;
        end if;
        if FIXED_INSTRUCTION_BUDGET > 0 and fixed_timing_counter > 0 then
          fixed_timing_counter <= fixed_timing_counter - 1;
        end if;
        if internal_data(SFR_TMC0_INDEX)(3) = '0' or tmc0_prev(3) = '0' or
          (internal_data(SFR_TMC0_INDEX)(1 downto 0) = "01" and
           tmc0_prev(7) = '0' and internal_data(SFR_TMC0_INDEX)(7) = '1') then
          timer_tout_r <= not internal_data(SFR_TMC0_INDEX)(2);
        end if;
        tmc0_prev <= internal_data(SFR_TMC0_INDEX);

        if rfm_refresh_timing_in = '1' then
          internal_data(SFR_RFM_INDEX)(7) <= rfm_rflv_slave;
        end if;

        if (internal_data(SFR_INTM_INDEX)(0) = '1' and nmi_prev = '0' and nmi_in = '1') or
          (internal_data(SFR_INTM_INDEX)(0) = '0' and nmi_prev = '1' and nmi_in = '0') then
          nmi_pending <= '1';
        end if;
        if (internal_data(SFR_INTM_INDEX)(2) = '1' and intp0_prev = '0' and intp0_in = '1') or
          (internal_data(SFR_INTM_INDEX)(2) = '0' and intp0_prev = '1' and intp0_in = '0') then
          internal_data(SFR_EXIC0_INDEX)(7) <= '1';
        end if;
        if (internal_data(SFR_INTM_INDEX)(4) = '1' and intp1_prev = '0' and intp1_in = '1') or
          (internal_data(SFR_INTM_INDEX)(4) = '0' and intp1_prev = '1' and intp1_in = '0') then
          internal_data(SFR_EXIC1_INDEX)(7) <= '1';
        end if;
        if (internal_data(SFR_INTM_INDEX)(6) = '1' and intp2_prev = '0' and intp2_in = '1') or
          (internal_data(SFR_INTM_INDEX)(6) = '0' and intp2_prev = '1' and intp2_in = '0') then
          internal_data(SFR_EXIC2_INDEX)(7) <= '1';
        end if;
        nmi_prev <= nmi_in;
        intp0_prev <= intp0_in;
        intp1_prev <= intp1_in;
        intp2_prev <= intp2_in;
        dmarq0_prev <= dmarq0_level_v;
        dmarq1_prev <= dmarq1_level_v;

        if serial0_rx_valid_in = '1' and internal_data(SFR_SCM0_INDEX)(6) = '1' then
          internal_data(SFR_RXB0_INDEX) <= serial0_rx_data_in;
          internal_data(SFR_SRIC0_INDEX)(7) <= '1';
          internal_data(SFR_SCE0_INDEX)(1) <= serial0_rx_frame_error_in;
          internal_data(SFR_SCE0_INDEX)(2) <= serial0_rx_parity_error_in;
          if serial0_rx_unread = '1' then
            internal_data(SFR_SCE0_INDEX)(0) <= '1';
            internal_data(SFR_SEIC0_INDEX)(7) <= '1';
          else
            internal_data(SFR_SCE0_INDEX)(0) <= '0';
          end if;
          if serial0_rx_frame_error_in = '1' or serial0_rx_parity_error_in = '1' then
            internal_data(SFR_SEIC0_INDEX)(7) <= '1';
          end if;
          serial0_rx_unread <= '1';
        end if;
        if serial1_rx_valid_in = '1' and internal_data(SFR_SCM1_INDEX)(6) = '1' then
          internal_data(SFR_RXB1_INDEX) <= serial1_rx_data_in;
          internal_data(SFR_SRIC1_INDEX)(7) <= '1';
          internal_data(SFR_SCE1_INDEX)(1) <= serial1_rx_frame_error_in;
          internal_data(SFR_SCE1_INDEX)(2) <= serial1_rx_parity_error_in;
          if serial1_rx_unread = '1' then
            internal_data(SFR_SCE1_INDEX)(0) <= '1';
            internal_data(SFR_SEIC1_INDEX)(7) <= '1';
          else
            internal_data(SFR_SCE1_INDEX)(0) <= '0';
          end if;
          if serial1_rx_frame_error_in = '1' or serial1_rx_parity_error_in = '1' then
            internal_data(SFR_SEIC1_INDEX)(7) <= '1';
          end if;
          serial1_rx_unread <= '1';
        end if;

        case state is
          when ST_FETCH_REQ =>
            if ENABLE_PREFETCH_QUEUE and prefetch_pending = '1' then
              if mem_ready = '1' then
                mem_valid_r <= '0';
                enqueue_prefetch_byte(mem_rdata);
              else
                mem_valid_r <= '1';
                mem_write_r <= '0';
                mem_addr_r <= v25_phys_addr(prefetch_pending_ps, prefetch_pending_ip);
              end if;
            elsif (ENABLE_TIMING_THROTTLE and timing_counter > 0) or
                  (FIXED_INSTRUCTION_BUDGET > 0 and fixed_timing_counter > 0) then
              if ENABLE_PREFETCH_QUEUE and prefetch_count > 0 and not prefetch_head_matches then
                mem_valid_r <= '0';
                clear_prefetch_queue;
              elsif ENABLE_PREFETCH_QUEUE and prefetch_count < PREFETCH_QUEUE_DEPTH then
                prefetch_target_ps := ps;
                if prefetch_count = 0 then
                  prefetch_target_ip := ip;
                else
                  prefetch_target_ip := std_logic_vector(unsigned(prefetch_ip_queue(prefetch_count - 1)) + 1);
                end if;
                mem_valid_r <= '1';
                mem_write_r <= '0';
                mem_addr_r <= v25_phys_addr(prefetch_target_ps, prefetch_target_ip);
                prefetch_pending <= '1';
                prefetch_pending_ps <= prefetch_target_ps;
                prefetch_pending_ip <= prefetch_target_ip;
              else
                mem_valid_r <= '0';
              end if;
            elsif ENABLE_PREFETCH_QUEUE and prefetch_count > 0 and not prefetch_head_matches then
              mem_valid_r <= '0';
              clear_prefetch_queue;
            elsif ENABLE_PREFETCH_QUEUE and prefetch_count < PREFETCH_FORCE_THRESHOLD then
              prefetch_target_ps := ps;
              if prefetch_count = 0 then
                prefetch_target_ip := ip;
              else
                prefetch_target_ip := std_logic_vector(unsigned(prefetch_ip_queue(prefetch_count - 1)) + 1);
              end if;
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r <= v25_phys_addr(prefetch_target_ps, prefetch_target_ip);
              prefetch_pending <= '1';
              prefetch_pending_ps <= prefetch_target_ps;
              prefetch_pending_ip <= prefetch_target_ip;
            else
              timer0_tick_v := '0';
              timer0_md_tick_v := '0';
              timer1_tick_v := '0';
              if timer0_pending_v > 0 then
                timer0_tick_v := '1';
                timer0_pending_v := timer0_pending_v - 1;
              end if;
              if timer0_md_pending_v > 0 then
                timer0_md_tick_v := '1';
                timer0_md_pending_v := timer0_md_pending_v - 1;
              end if;
              if timer1_pending_v > 0 then
                timer1_tick_v := '1';
                timer1_pending_v := timer1_pending_v - 1;
              end if;
              tick_timer_unit(internal_data, timer_tout_r, timer0_tick_v, timer0_md_tick_v, timer1_tick_v);
              tb_next := timebase_counter + 1;
              case internal_data(SFR_PRC_INDEX)(3 downto 2) is
                when "00" =>
                  tb_tap := tb_next(10);
                when "01" =>
                  tb_tap := tb_next(13);
                when "10" =>
                  tb_tap := tb_next(16);
                when others =>
                  if tb_next = to_unsigned(0, 20) then
                    tb_tap := '1';
                  else
                    tb_tap := '0';
                  end if;
              end case;
              if tb_tap = '1' and timebase_tap_prev = '0' then
                internal_data(SFR_TBIC_INDEX)(7) <= '1';
              end if;
              timebase_counter <= tb_next;
              timebase_tap_prev <= tb_tap;

              if seg_override_consumed = '1' then
                seg_override <= SEG_DEFAULT;
                seg_override_consumed <= '0';
              end if;
              if rep_mode_consumed = '1' then
                rep_mode <= REP_NONE;
                rep_mode_consumed <= '0';
                rep_timing_loaded <= '0';
              end if;

              if dma_transfer_pending(internal_data, 0, dmarq0_edge_v, dmarq0_level_v) then
                dma_base_v := dma_channel_base(0);
                dma_mode_v := internal_data(SFR_DMAM0_INDEX)(7 downto 5);
                dma_src_offset := timer_word(internal_data, dma_base_v);
                dma_dst_offset := timer_word(internal_data, dma_base_v + 2);
                dma_active_channel <= 0;
                dma_src_addr <= dma_phys_addr(internal_data(dma_base_v + 5), dma_src_offset);
                dma_dst_addr <= dma_phys_addr(internal_data(dma_base_v + 4), dma_dst_offset);
                dma_word_mode <= internal_data(SFR_DMAM0_INDEX)(4);
                dma_high_phase <= '0';
                if dma_mode_io_to_mem(dma_mode_v) then
                  dma_source_is_io <= '1';
                else
                  dma_source_is_io <= '0';
                end if;
                if dma_mode_mem_to_io(dma_mode_v) then
                  dma_dest_is_io <= '1';
                else
                  dma_dest_is_io <= '0';
                end if;
                if ENABLE_PREFETCH_QUEUE then
                  clear_prefetch_queue;
                end if;
                state <= ST_DMA_RD_REQ;
              elsif dma_transfer_pending(internal_data, 1, dmarq1_edge_v, dmarq1_level_v) then
                dma_base_v := dma_channel_base(1);
                dma_mode_v := internal_data(SFR_DMAM1_INDEX)(7 downto 5);
                dma_src_offset := timer_word(internal_data, dma_base_v);
                dma_dst_offset := timer_word(internal_data, dma_base_v + 2);
                dma_active_channel <= 1;
                dma_src_addr <= dma_phys_addr(internal_data(dma_base_v + 5), dma_src_offset);
                dma_dst_addr <= dma_phys_addr(internal_data(dma_base_v + 4), dma_dst_offset);
                dma_word_mode <= internal_data(SFR_DMAM1_INDEX)(4);
                dma_high_phase <= '0';
                if dma_mode_io_to_mem(dma_mode_v) then
                  dma_source_is_io <= '1';
                else
                  dma_source_is_io <= '0';
                end if;
                if dma_mode_mem_to_io(dma_mode_v) then
                  dma_dest_is_io <= '1';
                else
                  dma_dest_is_io <= '0';
                end if;
                if ENABLE_PREFETCH_QUEUE then
                  clear_prefetch_queue;
                end if;
                state <= ST_DMA_RD_REQ;
              else
                select_internal_irq(internal_data, internal_irq_take, internal_irq_vector,
                  internal_irq_priority, internal_irq_service_index);

                if nmi_pending = '1' then
                  nmi_pending <= '0';
                  if ENABLE_PREFETCH_QUEUE then
                    clear_prefetch_queue;
                  end if;
                  int_vector_base <= interrupt_vector_base(IRQ_VEC_NMI);
                  int_ibrk_after <= '0';
                  int_return_ip <= ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(IRQ_VEC_NMI);
                  state <= ST_INT_VEC_IP_LO_REQ;
                elsif internal_irq_take and macro_normal_pending(internal_data, internal_irq_service_index) then
                  if ENABLE_PREFETCH_QUEUE then
                    clear_prefetch_queue;
                  end if;
                  start_macro_service(internal_irq_service_index);
                elsif internal_irq_take and flags.iflag = '1' then
                  if internal_data(internal_irq_service_index)(5) = '0' and
                    internal_data(internal_irq_service_index)(4) = '1' then
                    if ENABLE_PREFETCH_QUEUE then
                      clear_prefetch_queue;
                    end if;
                    start_interrupt_bank_switch(
                      internal_data,
                      bank_saved_ip,
                      bank_saved_psw,
                      bank_ps,
                      bank_ss,
                      bank_ds0,
                      bank_ds1,
                      flags,
                      previous_bank,
                      active_bank,
                      ip,
                      ps,
                      ss,
                      ds0,
                      ds1,
                      internal_irq_service_index,
                      internal_irq_priority,
                      ip,
                      flags,
                      active_bank,
                      ps,
                      ss,
                      ds0,
                      ds1,
                      bank_vector_ip
                    );
                    state <= ST_FETCH_REQ;
                  else
                    irq_ispr_mask := shift_left(to_unsigned(1, 8), internal_irq_priority);
                    if ENABLE_PREFETCH_QUEUE then
                      clear_prefetch_queue;
                    end if;
                    internal_data(SFR_ISPR_INDEX) <= std_logic_vector(unsigned(internal_data(SFR_ISPR_INDEX)) or irq_ispr_mask);
                    internal_data(internal_irq_service_index)(7) <= '0';
                    int_vector_base <= interrupt_vector_base(internal_irq_vector);
                    int_ibrk_after <= '0';
                    int_return_ip <= ip;
                    mem_valid_r <= '1';
                    mem_write_r <= '0';
                    mem_addr_r  <= interrupt_vector_base(internal_irq_vector);
                    state <= ST_INT_VEC_IP_LO_REQ;
                  end if;
                elsif irq_request = '1' and flags.iflag = '1' then
                  if ENABLE_PREFETCH_QUEUE then
                    clear_prefetch_queue;
                  end if;
                  external_irq_vector <= irq_vector;
                  int_ibrk_after <= '0';
                  int_return_ip <= ip;
                  int_ack_valid_r <= '1';
                  int_ack_second_r <= '0';
                  state <= ST_EXT_INT_ACK1_REQ;
                else
                  if ENABLE_PREFETCH_QUEUE and prefetch_head_matches then
                    finish_opcode_fetch(prefetch_bytes(0));
                    drop_prefetch_head;
                  else
                    if ENABLE_PREFETCH_QUEUE then
                      clear_prefetch_queue;
                    end if;
                    mem_valid_r <= '1';
                    mem_write_r <= '0';
                    mem_addr_r  <= v25_phys_addr(ps, ip);
                    state       <= ST_FETCH_WAIT;
                  end if;
                end if;
              end if;
            end if;

          when ST_FETCH_WAIT =>
            if mem_ready = '1' then
              if ENABLE_PREFETCH_QUEUE then
                clear_prefetch_queue;
              end if;
              finish_opcode_fetch(mem_rdata);
            end if;

          when ST_DECODE =>
            op_u := unsigned(opcode);

            if op_u >= to_unsigned(16#B8#, 8) and op_u <= to_unsigned(16#BF#, 8) then
              op_kind    <= OP_MOV_R16_IMM;
              op_reg     <= to_integer(op_u - to_unsigned(16#B8#, 8));
              imm_needed <= 2;
              imm_index  <= 0;
              imm16      <= x"0000";
              state      <= ST_IMM_REQ;
            elsif op_u >= to_unsigned(16#B0#, 8) and op_u <= to_unsigned(16#B7#, 8) then
              op_kind    <= OP_MOV_R8_IMM;
              op_reg     <= to_integer(op_u - to_unsigned(16#B0#, 8));
              imm_needed <= 1;
              imm_index  <= 0;
              imm16      <= x"0000";
              state      <= ST_IMM_REQ;
            elsif op_u >= to_unsigned(16#40#, 8) and op_u <= to_unsigned(16#47#, 8) then
              op_kind <= OP_INC_R16;
              op_reg  <= to_integer(op_u - to_unsigned(16#40#, 8));
              state   <= ST_EXECUTE;
            elsif op_u >= to_unsigned(16#48#, 8) and op_u <= to_unsigned(16#4F#, 8) then
              op_kind <= OP_DEC_R16;
              op_reg  <= to_integer(op_u - to_unsigned(16#48#, 8));
              state   <= ST_EXECUTE;
            elsif op_u >= to_unsigned(16#50#, 8) and op_u <= to_unsigned(16#57#, 8) then
              op_kind <= OP_PUSH_R16;
              op_reg  <= to_integer(op_u - to_unsigned(16#50#, 8));
              state   <= ST_EXECUTE;
            elsif op_u >= to_unsigned(16#58#, 8) and op_u <= to_unsigned(16#5F#, 8) then
              op_kind <= OP_POP_R16;
              op_reg  <= to_integer(op_u - to_unsigned(16#58#, 8));
              state   <= ST_EXECUTE;
            elsif op_u >= to_unsigned(16#91#, 8) and op_u <= to_unsigned(16#97#, 8) then
              op_kind <= OP_XCHG_AX_R16;
              op_reg  <= to_integer(op_u - to_unsigned(16#90#, 8));
              state   <= ST_EXECUTE;
            elsif op_u >= to_unsigned(16#70#, 8) and op_u <= to_unsigned(16#7F#, 8) then
              op_kind    <= OP_JCC_REL8;
              op_reg     <= to_integer(op_u - to_unsigned(16#70#, 8));
              imm_needed <= 1;
              imm_index  <= 0;
              imm16      <= x"0000";
              state      <= ST_IMM_REQ;
            elsif op_u >= to_unsigned(16#E0#, 8) and op_u <= to_unsigned(16#E3#, 8) then
              op_kind    <= OP_LOOP_REL8;
              op_reg     <= to_integer(op_u - to_unsigned(16#E0#, 8));
              imm_needed <= 1;
              imm_index  <= 0;
              imm16      <= x"0000";
              state      <= ST_IMM_REQ;
            elsif op_u >= to_unsigned(16#D8#, 8) and op_u <= to_unsigned(16#DF#, 8) then
              op_kind <= OP_FPO1;
              state   <= ST_MODRM_REQ;
            else
              case opcode is
                when x"04" | x"0C" | x"14" | x"1C" |
                     x"24" | x"2C" | x"34" | x"3C" =>
                  op_kind    <= OP_ALU_AL_IMM8;
                  alu_func   <= opcode_to_alu(opcode);
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"27" =>
                  op_kind <= OP_ADJ4A;
                  state   <= ST_EXECUTE;

                when x"2F" =>
                  op_kind <= OP_ADJ4S;
                  state   <= ST_EXECUTE;

                when x"37" =>
                  op_kind <= OP_ADJBA;
                  state   <= ST_EXECUTE;

                when x"3F" =>
                  op_kind <= OP_ADJBS;
                  state   <= ST_EXECUTE;

                when x"05" | x"0D" | x"15" | x"1D" |
                     x"25" | x"2D" | x"35" | x"3D" =>
                  op_kind    <= OP_ALU_AX_IMM16;
                  alu_func   <= opcode_to_alu(opcode);
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A8" =>
                  op_kind    <= OP_TEST_AL_IMM8;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A9" =>
                  op_kind    <= OP_TEST_AX_IMM16;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"00" | x"08" | x"10" | x"18" |
                     x"20" | x"28" | x"30" | x"38" =>
                  op_kind  <= OP_ALU_RM8_R8;
                  alu_func <= group_to_alu(opcode(5 downto 3));
                  state    <= ST_MODRM_REQ;

                when x"01" | x"09" | x"11" | x"19" |
                     x"21" | x"29" | x"31" | x"39" =>
                  op_kind  <= OP_ALU_RM16_R16;
                  alu_func <= group_to_alu(opcode(5 downto 3));
                  state    <= ST_MODRM_REQ;

                when x"02" | x"0A" | x"12" | x"1A" |
                     x"22" | x"2A" | x"32" | x"3A" =>
                  op_kind  <= OP_ALU_R8_RM8;
                  alu_func <= group_to_alu(opcode(5 downto 3));
                  state    <= ST_MODRM_REQ;

                when x"03" | x"0B" | x"13" | x"1B" |
                     x"23" | x"2B" | x"33" | x"3B" =>
                  op_kind  <= OP_ALU_R16_RM16;
                  alu_func <= group_to_alu(opcode(5 downto 3));
                  state    <= ST_MODRM_REQ;

                when x"84" =>
                  op_kind <= OP_TEST_RM8_R8;
                  state   <= ST_MODRM_REQ;

                when x"85" =>
                  op_kind <= OP_TEST_RM16_R16;
                  state   <= ST_MODRM_REQ;

                when x"80" =>
                  op_kind <= OP_GRP_IMM8_RM8;
                  state   <= ST_MODRM_REQ;

                when x"81" =>
                  op_kind <= OP_GRP_IMM16_RM16;
                  state   <= ST_MODRM_REQ;

                when x"83" =>
                  op_kind <= OP_GRP_IMM8_RM16_SIGN;
                  state   <= ST_MODRM_REQ;

                when x"69" =>
                  op_kind <= OP_IMUL_R16_RM16_IMM16;
                  state   <= ST_MODRM_REQ;

                when x"6B" =>
                  op_kind <= OP_IMUL_R16_RM16_IMM8;
                  state   <= ST_MODRM_REQ;

                when x"F6" =>
                  op_kind <= OP_GRP3_RM8;
                  state   <= ST_MODRM_REQ;

                when x"F7" =>
                  op_kind <= OP_GRP3_RM16;
                  state   <= ST_MODRM_REQ;

                when x"FE" =>
                  op_kind <= OP_GRP_FE;
                  state   <= ST_MODRM_REQ;

                when x"C0" =>
                  op_kind <= OP_GRP_SHIFT_RM8_IMM;
                  state   <= ST_MODRM_REQ;

                when x"C1" =>
                  op_kind <= OP_GRP_SHIFT_RM16_IMM;
                  state   <= ST_MODRM_REQ;

                when x"D0" =>
                  op_kind <= OP_GRP_SHIFT_RM8_1;
                  state   <= ST_MODRM_REQ;

                when x"D1" =>
                  op_kind <= OP_GRP_SHIFT_RM16_1;
                  state   <= ST_MODRM_REQ;

                when x"D2" =>
                  op_kind <= OP_GRP_SHIFT_RM8_CL;
                  state   <= ST_MODRM_REQ;

                when x"D3" =>
                  op_kind <= OP_GRP_SHIFT_RM16_CL;
                  state   <= ST_MODRM_REQ;

                when x"06" =>
                  op_kind <= OP_PUSH_SREG;
                  mem_sreg_target <= SEG_DS1;
                  state <= ST_EXECUTE;

                when x"0E" =>
                  op_kind <= OP_PUSH_SREG;
                  mem_sreg_target <= SEG_PS;
                  state <= ST_EXECUTE;

                when x"16" =>
                  op_kind <= OP_PUSH_SREG;
                  mem_sreg_target <= SEG_SS;
                  state <= ST_EXECUTE;

                when x"1E" =>
                  op_kind <= OP_PUSH_SREG;
                  mem_sreg_target <= SEG_DS0;
                  state <= ST_EXECUTE;

                when x"07" =>
                  op_kind <= OP_POP_SREG;
                  mem_sreg_target <= SEG_DS1;
                  state <= ST_EXECUTE;

                when x"17" =>
                  op_kind <= OP_POP_SREG;
                  mem_sreg_target <= SEG_SS;
                  state <= ST_EXECUTE;

                when x"1F" =>
                  op_kind <= OP_POP_SREG;
                  mem_sreg_target <= SEG_DS0;
                  state <= ST_EXECUTE;

                when x"88" =>
                  op_kind <= OP_MOV_RM8_R8;
                  state   <= ST_MODRM_REQ;

                when x"86" =>
                  op_kind <= OP_XCHG_RM8_R8;
                  state   <= ST_MODRM_REQ;

                when x"87" =>
                  op_kind <= OP_XCHG_RM16_R16;
                  state   <= ST_MODRM_REQ;

                when x"89" =>
                  op_kind <= OP_MOV_RM16_R16;
                  state   <= ST_MODRM_REQ;

                when x"8A" =>
                  op_kind <= OP_MOV_R8_RM8;
                  state   <= ST_MODRM_REQ;

                when x"8B" =>
                  op_kind <= OP_MOV_R16_RM16;
                  state   <= ST_MODRM_REQ;

                when x"8C" =>
                  op_kind <= OP_MOV_RM16_SREG;
                  state   <= ST_MODRM_REQ;

                when x"8D" =>
                  op_kind <= OP_LEA_R16_MEM;
                  state   <= ST_MODRM_REQ;

                when x"8E" =>
                  op_kind <= OP_MOV_SREG_RM16;
                  state   <= ST_MODRM_REQ;

                when x"8F" =>
                  op_kind <= OP_POP_RM16;
                  state   <= ST_MODRM_REQ;

                when x"C4" =>
                  op_kind <= OP_MOV_DS1_R16_MEM32;
                  state   <= ST_MODRM_REQ;

                when x"C5" =>
                  op_kind <= OP_MOV_DS0_R16_MEM32;
                  state   <= ST_MODRM_REQ;

                when x"C6" =>
                  op_kind <= OP_MOV_RM8_IMM8;
                  state   <= ST_MODRM_REQ;

                when x"C7" =>
                  op_kind <= OP_MOV_RM16_IMM16;
                  state   <= ST_MODRM_REQ;

                when x"D7" =>
                  op_kind <= OP_XLAT;
                  state   <= ST_EXECUTE;

                when x"D4" =>
                  op_kind    <= OP_CVTBD;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"D5" =>
                  op_kind    <= OP_CVTDB;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A0" =>
                  op_kind    <= OP_MOV_AL_MOFFS;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A1" =>
                  op_kind    <= OP_MOV_AX_MOFFS;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A2" =>
                  op_kind    <= OP_MOV_MOFFS_AL;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A3" =>
                  op_kind    <= OP_MOV_MOFFS_AX;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"A4" =>
                  op_kind <= OP_MOVS8;
                  state   <= ST_EXECUTE;

                when x"A5" =>
                  op_kind <= OP_MOVS16;
                  state   <= ST_EXECUTE;

                when x"A6" =>
                  op_kind <= OP_CMPS8;
                  state   <= ST_EXECUTE;

                when x"A7" =>
                  op_kind <= OP_CMPS16;
                  state   <= ST_EXECUTE;

                when x"AA" =>
                  op_kind <= OP_STOS8;
                  state   <= ST_EXECUTE;

                when x"AB" =>
                  op_kind <= OP_STOS16;
                  state   <= ST_EXECUTE;

                when x"AC" =>
                  op_kind <= OP_LODS8;
                  state   <= ST_EXECUTE;

                when x"AD" =>
                  op_kind <= OP_LODS16;
                  state   <= ST_EXECUTE;

                when x"AE" =>
                  op_kind <= OP_SCAS8;
                  state   <= ST_EXECUTE;

                when x"AF" =>
                  op_kind <= OP_SCAS16;
                  state   <= ST_EXECUTE;

                when x"FF" =>
                  op_kind <= OP_GRP_FF;
                  state   <= ST_MODRM_REQ;

                when x"68" =>
                  op_kind    <= OP_PUSH_IMM16;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"66" | x"67" =>
                  op_kind <= OP_FPO2;
                  state   <= ST_MODRM_REQ;

                when x"6A" =>
                  op_kind    <= OP_PUSH_IMM8_SIGN;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"6C" =>
                  op_kind <= OP_INM8;
                  state   <= ST_EXECUTE;

                when x"6D" =>
                  op_kind <= OP_INM16;
                  state   <= ST_EXECUTE;

                when x"6E" =>
                  op_kind <= OP_OUTM8;
                  state   <= ST_EXECUTE;

                when x"6F" =>
                  op_kind <= OP_OUTM16;
                  state   <= ST_EXECUTE;

                when x"60" =>
                  op_kind <= OP_PUSH_REGS;
                  state <= ST_EXECUTE;

                when x"61" =>
                  op_kind <= OP_POP_REGS;
                  state <= ST_EXECUTE;

                when x"62" =>
                  op_kind <= OP_CHKIND;
                  state   <= ST_MODRM_REQ;

                when x"64" =>
                  rep_mode <= REP_NC;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  rep_timing_loaded <= '0';
                  state <= ST_FETCH_REQ;

                when x"65" =>
                  rep_mode <= REP_C;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  rep_timing_loaded <= '0';
                  state <= ST_FETCH_REQ;

                when x"9A" =>
                  op_kind    <= OP_CALL_FAR_IMM;
                  imm_needed <= 4;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  far_seg    <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"EA" =>
                  op_kind    <= OP_JMP_FAR_IMM;
                  imm_needed <= 4;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  far_seg    <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"E8" =>
                  op_kind    <= OP_CALL_REL16;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"E4" =>
                  op_kind    <= OP_IN_AL_IMM8;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"E5" =>
                  op_kind    <= OP_IN_AX_IMM8;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"E6" =>
                  op_kind    <= OP_OUT_IMM8_AL;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"E7" =>
                  op_kind    <= OP_OUT_IMM8_AX;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"E9" =>
                  op_kind    <= OP_JMP_REL16;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"EB" =>
                  op_kind    <= OP_JMP_REL8;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"EC" =>
                  op_kind <= OP_IN_AL_DX;
                  state   <= ST_EXECUTE;

                when x"ED" =>
                  op_kind <= OP_IN_AX_DX;
                  state   <= ST_EXECUTE;

                when x"EE" =>
                  op_kind <= OP_OUT_DX_AL;
                  state   <= ST_EXECUTE;

                when x"EF" =>
                  op_kind <= OP_OUT_DX_AX;
                  state   <= ST_EXECUTE;

                when x"26" =>
                  seg_override <= SEG_DS1;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  state <= ST_FETCH_REQ;

                when x"2E" =>
                  seg_override <= SEG_PS;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  state <= ST_FETCH_REQ;

                when x"36" =>
                  seg_override <= SEG_SS;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  state <= ST_FETCH_REQ;

                when x"3E" =>
                  seg_override <= SEG_DS0;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  state <= ST_FETCH_REQ;

                when x"C3" =>
                  op_kind <= OP_RET_NEAR;
                  state   <= ST_EXECUTE;

                when x"C8" =>
                  op_kind    <= OP_PREPARE;
                  imm_needed <= 3;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  far_seg    <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"C9" =>
                  op_kind <= OP_DISPOSE;
                  state   <= ST_EXECUTE;

                when x"C2" =>
                  op_kind    <= OP_RET_NEAR_IMM;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"CB" =>
                  op_kind <= OP_RET_FAR;
                  state   <= ST_EXECUTE;

                when x"CA" =>
                  op_kind    <= OP_RET_FAR_IMM;
                  imm_needed <= 2;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"CC" =>
                  op_kind <= OP_BRK;
                  imm16   <= x"0003";
                  state   <= ST_EXECUTE;

                when x"CD" =>
                  op_kind    <= OP_BRK;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when x"CE" =>
                  op_kind <= OP_BRKV;
                  state   <= ST_EXECUTE;

                when x"CF" =>
                  op_kind <= OP_RETI;
                  state   <= ST_EXECUTE;

                when x"90" =>
                  state <= ST_FETCH_REQ;

                when x"98" =>
                  if v25_get_reg8(regs(active_bank), 0)(7) = '1' then
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 4, x"FF");
                  else
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 4, x"00");
                  end if;
                  state <= ST_FETCH_REQ;

                when x"99" =>
                  if regs(active_bank)(V25_REG_AX)(15) = '1' then
                    regs(active_bank)(V25_REG_DX) <= x"FFFF";
                  else
                    regs(active_bank)(V25_REG_DX) <= x"0000";
                  end if;
                  state <= ST_FETCH_REQ;

                when x"9B" =>
                  if poll_pin_waiting(internal_data, port1_in) then
                    poll_wait_count <= 4;
                    state <= ST_POLL_WAIT;
                  else
                    state <= ST_FETCH_REQ;
                  end if;

                when x"9C" =>
                  op_kind <= OP_PUSH_PSW;
                  state <= ST_EXECUTE;

                when x"9D" =>
                  op_kind <= OP_POP_PSW;
                  state <= ST_EXECUTE;

                when x"9E" =>
                  op_kind <= OP_MOV_PSW_AH;
                  state <= ST_EXECUTE;

                when x"9F" =>
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), 4, v25_pack_psw(flags)(7 downto 0));
                  state <= ST_FETCH_REQ;

                when x"F0" =>
                  -- Bus locking is not modeled yet; treat LOCK as a consumed prefix.
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  state <= ST_FETCH_REQ;

                when x"F2" =>
                  rep_mode <= REP_NE;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  rep_timing_loaded <= '0';
                  state <= ST_FETCH_REQ;

                when x"F3" =>
                  rep_mode <= REP_E;
                  seg_override_consumed <= '0';
                  rep_mode_consumed <= '0';
                  rep_timing_loaded <= '0';
                  state <= ST_FETCH_REQ;

                when x"F4" =>
                  halt_stop_mode <= '0';
                  state <= ST_HALTED;

                when x"F5" =>
                  flags.cf <= not flags.cf;
                  state <= ST_FETCH_REQ;

                when x"F8" =>
                  flags.cf <= '0';
                  state <= ST_FETCH_REQ;

                when x"F9" =>
                  flags.cf <= '1';
                  state <= ST_FETCH_REQ;

                when x"FA" =>
                  flags.iflag <= '0';
                  state <= ST_FETCH_REQ;

                when x"FB" =>
                  flags.iflag <= '1';
                  state <= ST_FETCH_REQ;

                when x"FC" =>
                  flags.df <= '0';
                  state <= ST_FETCH_REQ;

                when x"FD" =>
                  flags.df <= '1';
                  state <= ST_FETCH_REQ;

                when x"0F" =>
                  op_kind    <= OP_V25_PREFIX;
                  imm_needed <= 1;
                  imm_index  <= 0;
                  imm16      <= x"0000";
                  state      <= ST_IMM_REQ;

                when others =>
                  fault_r <= '1';
                  state   <= ST_FAULT;
              end case;
            end if;

          when ST_IMM_REQ =>
            if ENABLE_PREFETCH_QUEUE and prefetch_head_matches then
              finish_imm_fetch(prefetch_bytes(0));
              drop_prefetch_head;
            else
              if ENABLE_PREFETCH_QUEUE and prefetch_count > 0 then
                clear_prefetch_queue;
              end if;
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= v25_phys_addr(ps, ip);
              state       <= ST_IMM_WAIT;
            end if;

          when ST_IMM_WAIT =>
            if mem_ready = '1' then
              finish_imm_fetch(mem_rdata);
            end if;

          when ST_MODRM_REQ =>
            if ENABLE_PREFETCH_QUEUE and prefetch_head_matches then
              finish_modrm_fetch(prefetch_bytes(0));
              drop_prefetch_head;
            else
              if ENABLE_PREFETCH_QUEUE and prefetch_count > 0 then
                clear_prefetch_queue;
              end if;
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= v25_phys_addr(ps, ip);
              state       <= ST_MODRM_WAIT;
            end if;

          when ST_MODRM_WAIT =>
            if mem_ready = '1' then
              finish_modrm_fetch(mem_rdata);
            end if;

          when ST_DISP_REQ =>
            if ENABLE_PREFETCH_QUEUE and prefetch_head_matches then
              finish_disp_fetch(prefetch_bytes(0));
              drop_prefetch_head;
            else
              if ENABLE_PREFETCH_QUEUE and prefetch_count > 0 then
                clear_prefetch_queue;
              end if;
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= v25_phys_addr(ps, ip);
              state       <= ST_DISP_WAIT;
            end if;

          when ST_DISP_WAIT =>
            if mem_ready = '1' then
              finish_disp_fetch(mem_rdata);
            end if;

          when ST_EXECUTE =>
            if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '0' and
               is_repeatable_string_op(op_kind) and
               repeat_prefix_applies(rep_mode, op_kind) then
              timing_budget_v := repeat_timing_budget(
                op_kind,
                rep_mode,
                regs(active_bank)(V25_REG_CX),
                current_io_wait_states(internal_data)
              );
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
              rep_timing_loaded <= '1';
            end if;

            if ENABLE_TIMING_THROTTLE and
               not (is_repeatable_string_op(op_kind) and repeat_prefix_applies(rep_mode, op_kind)) then
              timing_budget_v := io_timing_budget(op_kind, current_io_wait_states(internal_data));
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
            end if;

            if ENABLE_TIMING_THROTTLE and is_memory_string_op(op_kind) and
               not repeat_prefix_applies(rep_mode, op_kind) then
              select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
              timing_budget_v := string_single_timing_budget(
                op_kind,
                timing_wait_v,
                timing_internal_v
              );
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
            end if;

            if ENABLE_TIMING_THROTTLE and is_primitive_io_block_op(op_kind) and
               not repeat_prefix_applies(rep_mode, op_kind) then
              select_primitive_io_timing(op_kind, timing_wait_v, timing_internal_v);
              timing_budget_v := primitive_io_single_timing_budget(
                op_kind,
                timing_wait_v,
                timing_internal_v
              );
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
            end if;

            if ENABLE_TIMING_THROTTLE and
               (op_kind = OP_MOV_AL_MOFFS or op_kind = OP_MOV_AX_MOFFS or
                op_kind = OP_MOV_MOFFS_AL or op_kind = OP_MOV_MOFFS_AX) then
              ea_addr := v25_phys_addr(
                v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                imm16
              );
              timing_budget_v := direct_moffs_timing_budget(
                op_kind,
                current_memory_wait_states(internal_data, ea_addr),
                v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX))
              );
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
            end if;

            if ENABLE_TIMING_THROTTLE and op_kind = OP_XLAT then
              ea_addr := v25_phys_addr(
                v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                std_logic_vector(
                  unsigned(regs(active_bank)(V25_REG_BX)) +
                  resize(unsigned(v25_get_reg8(regs(active_bank), 0)), 16)
                )
              );
              timing_counter <= v25_clocks_trans(current_memory_wait_states(internal_data, ea_addr));
            end if;

            if ENABLE_TIMING_THROTTLE and
               (op_kind = OP_ADD4S or op_kind = OP_SUB4S or op_kind = OP_CMP4S) then
              tmp_nat := to_integer(unsigned(v25_get_reg8(regs(active_bank), 1)));
              if tmp_nat > 0 and tmp_nat <= 254 then
                if (tmp_nat mod 2) = 1 then
                  tmp_nat := tmp_nat + 1;
                end if;
                tmp_nat := tmp_nat / 2;
                seg_value := v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1);
                ea_addr := v25_phys_addr(seg_value, regs(active_bank)(V25_REG_SI));
                timing_addr_v := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                timing_wait_v := current_memory_wait_states(internal_data, ea_addr);
                timing_wait2_v := current_memory_wait_states(internal_data, timing_addr_v);
                if timing_wait2_v > timing_wait_v then
                  timing_wait_v := timing_wait2_v;
                end if;
                timing_internal_v :=
                  v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or
                  v25_internal_data_selected(timing_addr_v, idb_high, internal_data(SFR_PRC_INDEX));

                if op_kind = OP_ADD4S then
                  timing_counter <= v25_clocks_add4s(timing_internal_v, timing_wait_v, tmp_nat);
                elsif op_kind = OP_SUB4S then
                  timing_counter <= v25_clocks_sub4s(timing_internal_v, timing_wait_v, tmp_nat);
                else
                  timing_counter <= v25_clocks_cmp4s(timing_wait_v, tmp_nat);
                end if;
              end if;
            end if;

            if ENABLE_TIMING_THROTTLE then
              timing_budget_v := 0;
              case op_kind is
                when OP_PUSH_R16 | OP_PUSH_SREG | OP_PUSH_PSW |
                     OP_PUSH_IMM8_SIGN | OP_PUSH_IMM16 | OP_CALL_REL16 =>
                  select_stack_words_timing(
                    std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2),
                    1,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_PUSH_REGS =>
                  select_stack_words_timing(
                    std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2),
                    8,
                    true,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_CALL_FAR_IMM =>
                  select_stack_words_timing(
                    std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 4),
                    2,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_POP_R16 | OP_POP_SREG | OP_POP_PSW |
                     OP_RET_NEAR | OP_RET_NEAR_IMM =>
                  select_stack_words_timing(
                    regs(active_bank)(V25_REG_SP),
                    1,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_POP_REGS =>
                  select_stack_words_timing(
                    regs(active_bank)(V25_REG_SP),
                    8,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_RET_FAR | OP_RET_FAR_IMM =>
                  select_stack_words_timing(
                    regs(active_bank)(V25_REG_SP),
                    2,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_RETI =>
                  select_stack_words_timing(
                    regs(active_bank)(V25_REG_SP),
                    3,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_DISPOSE =>
                  select_stack_words_timing(
                    regs(active_bank)(V25_REG_BP),
                    1,
                    false,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := direct_stack_timing_budget(
                    op_kind,
                    timing_wait_v,
                    timing_internal_v
                  );
                when OP_PREPARE =>
                  tmp_nat := to_integer(unsigned(far_seg(7 downto 0)));
                  select_prepare_timing(
                    tmp_nat,
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := v25_clocks_prepare(
                    tmp_nat,
                    timing_wait_v
                  );
                when OP_BRK =>
                  select_interrupt_entry_timing(
                    interrupt_vector_base(imm16(7 downto 0)),
                    timing_wait_v,
                    timing_internal_v
                  );
                  timing_budget_v := v25_clocks_brk(
                    imm16(7 downto 0) = x"03",
                    timing_internal_v,
                    timing_wait_v
                  );
                when OP_BRKV =>
                  if flags.oflag = '1' then
                    select_interrupt_entry_timing(
                      interrupt_vector_base(x"04"),
                      timing_wait_v,
                      timing_internal_v
                    );
                    timing_budget_v := v25_clocks_brkv(
                      timing_internal_v,
                      timing_wait_v
                    );
                  end if;
                when OP_GRP_FF =>
                  if not v25_modrm_is_memory(modrm) then
                    case modrm(5 downto 3) is
                      when "010" =>
                        select_stack_words_timing(
                          std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2),
                          1,
                          false,
                          timing_wait_v,
                          timing_internal_v
                        );
                        timing_budget_v := v25_clocks_call_near(
                          timing_internal_v,
                          timing_wait_v
                        );
                      when "110" =>
                        select_stack_words_timing(
                          std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2),
                          1,
                          false,
                          timing_wait_v,
                          timing_internal_v
                        );
                        timing_budget_v := v25_clocks_push_reg16(
                          timing_internal_v,
                          timing_wait_v
                        );
                      when others =>
                        null;
                    end case;
                  end if;
                when others =>
                  null;
              end case;
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
            end if;

            if ENABLE_TIMING_THROTTLE and v25_modrm_is_memory(modrm) then
              ea_addr := v25_phys_addr(
                v25_selected_seg(
                  seg_override,
                  v25_modrm_default_seg(ss, ds0, modrm),
                  ps,
                  ss,
                  ds0,
                  ds1
                ),
                v25_modrm_ea(regs(active_bank), modrm, disp16)
              );
              shift_count := shift_count_for_op(op_kind, regs(active_bank), imm16);
              timing_budget_v := resolved_modrm_timing_budget(
                op_kind,
                alu_func,
                v25_subop,
                modrm,
                current_memory_wait_states(internal_data, ea_addr),
                v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)),
                shift_count
              );
              if timing_budget_v /= 0 then
                timing_counter <= timing_budget_v;
              end if;
            end if;

            if rep_mode /= REP_NONE and
               is_repeatable_string_op(op_kind) and
               regs(active_bank)(V25_REG_CX) = x"0000" then
              state <= ST_FETCH_REQ;
            else
              case op_kind is
              when OP_MOV_R16_IMM =>
                regs(active_bank)(op_reg) <= imm16;
                state <= ST_FETCH_REQ;

              when OP_MOV_R8_IMM =>
                regs(active_bank) <= v25_set_reg8(regs(active_bank), op_reg, imm16(7 downto 0));
                state <= ST_FETCH_REQ;

              when OP_ALU_AL_IMM8 =>
                a8 := v25_get_reg8(regs(active_bank), 0);
                b8 := imm16(7 downto 0);
                f := flags;

                case alu_func is
                  when ALU_ADD =>
                    r8 := std_logic_vector(unsigned(a8) + unsigned(b8));
                    f := v25_add_flags8(flags, a8, b8, '0');
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_OR =>
                    r8 := a8 or b8;
                    f := v25_logic_flags8(flags, r8);
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_ADC =>
                    r8 := std_logic_vector(unsigned(a8) + unsigned(b8));
                    if flags.cf = '1' then
                      r8 := std_logic_vector(unsigned(r8) + 1);
                    end if;
                    f := v25_add_flags8(flags, a8, b8, flags.cf);
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_SBB =>
                    r8 := std_logic_vector(unsigned(a8) - unsigned(b8));
                    if flags.cf = '1' then
                      r8 := std_logic_vector(unsigned(r8) - 1);
                    end if;
                    f := v25_sub_flags8(flags, a8, b8, flags.cf);
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_AND =>
                    r8 := a8 and b8;
                    f := v25_logic_flags8(flags, r8);
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_SUB =>
                    r8 := std_logic_vector(unsigned(a8) - unsigned(b8));
                    f := v25_sub_flags8(flags, a8, b8, '0');
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_XOR =>
                    r8 := a8 xor b8;
                    f := v25_logic_flags8(flags, r8);
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                  when ALU_CMP =>
                    f := v25_sub_flags8(flags, a8, b8, '0');
                end case;

                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_ALU_AX_IMM16 =>
                a16 := regs(active_bank)(V25_REG_AX);
                b16 := imm16;
                f := flags;

                case alu_func is
                  when ALU_ADD =>
                    r16 := std_logic_vector(unsigned(a16) + unsigned(b16));
                    f := v25_add_flags16(flags, a16, b16, '0');
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_OR =>
                    r16 := a16 or b16;
                    f := v25_logic_flags16(flags, r16);
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_ADC =>
                    r16 := std_logic_vector(unsigned(a16) + unsigned(b16));
                    if flags.cf = '1' then
                      r16 := std_logic_vector(unsigned(r16) + 1);
                    end if;
                    f := v25_add_flags16(flags, a16, b16, flags.cf);
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_SBB =>
                    r16 := std_logic_vector(unsigned(a16) - unsigned(b16));
                    if flags.cf = '1' then
                      r16 := std_logic_vector(unsigned(r16) - 1);
                    end if;
                    f := v25_sub_flags16(flags, a16, b16, flags.cf);
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_AND =>
                    r16 := a16 and b16;
                    f := v25_logic_flags16(flags, r16);
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_SUB =>
                    r16 := std_logic_vector(unsigned(a16) - unsigned(b16));
                    f := v25_sub_flags16(flags, a16, b16, '0');
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_XOR =>
                    r16 := a16 xor b16;
                    f := v25_logic_flags16(flags, r16);
                    regs(active_bank)(V25_REG_AX) <= r16;
                  when ALU_CMP =>
                    f := v25_sub_flags16(flags, a16, b16, '0');
                end case;

                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_ADJ4A =>
                a8 := v25_get_reg8(regs(active_bank), 0);
                r8 := a8;
                f := flags;
                adjust_low := unsigned(a8(3 downto 0)) > to_unsigned(9, 4) or flags.af = '1';
                adjust_high := unsigned(a8) > to_unsigned(16#99#, 8) or flags.cf = '1';

                if adjust_low then
                  r8 := std_logic_vector(unsigned(r8) + to_unsigned(6, 8));
                  f.af := '1';
                else
                  f.af := '0';
                end if;

                if adjust_high then
                  r8 := std_logic_vector(unsigned(r8) + to_unsigned(16#60#, 8));
                  f.cf := '1';
                else
                  f.cf := '0';
                end if;

                f.sf := r8(7);
                if r8 = x"00" then
                  f.zf := '1';
                else
                  f.zf := '0';
                end if;
                f.pf := v25_even_parity8(r8);
                regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_ADJ4S =>
                a8 := v25_get_reg8(regs(active_bank), 0);
                r8 := a8;
                f := flags;
                adjust_low := unsigned(a8(3 downto 0)) > to_unsigned(9, 4) or flags.af = '1';
                adjust_high := unsigned(a8) > to_unsigned(16#99#, 8) or flags.cf = '1';

                if adjust_low then
                  r8 := std_logic_vector(unsigned(r8) - to_unsigned(6, 8));
                  f.af := '1';
                else
                  f.af := '0';
                end if;

                if adjust_high then
                  r8 := std_logic_vector(unsigned(r8) - to_unsigned(16#60#, 8));
                  f.cf := '1';
                else
                  f.cf := '0';
                end if;

                f.sf := r8(7);
                if r8 = x"00" then
                  f.zf := '1';
                else
                  f.zf := '0';
                end if;
                f.pf := v25_even_parity8(r8);
                regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, r8);
                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_ADJBA =>
                a8 := v25_get_reg8(regs(active_bank), 0);
                reg_bank := regs(active_bank);
                f := flags;
                adjust_low := unsigned(a8(3 downto 0)) > to_unsigned(9, 4) or flags.af = '1';

                if adjust_low then
                  r8 := std_logic_vector(unsigned(a8) + to_unsigned(6, 8));
                  reg_bank(V25_REG_AX)(15 downto 8) :=
                    std_logic_vector(unsigned(regs(active_bank)(V25_REG_AX)(15 downto 8)) + 1);
                  f.af := '1';
                  f.cf := '1';
                else
                  r8 := a8;
                  f.af := '0';
                  f.cf := '0';
                end if;

                reg_bank := v25_set_reg8(reg_bank, 0, r8 and x"0F");
                regs(active_bank) <= reg_bank;
                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_ADJBS =>
                a8 := v25_get_reg8(regs(active_bank), 0);
                reg_bank := regs(active_bank);
                f := flags;
                adjust_low := unsigned(a8(3 downto 0)) > to_unsigned(9, 4) or flags.af = '1';

                if adjust_low then
                  r8 := std_logic_vector(unsigned(a8) - to_unsigned(6, 8));
                  reg_bank(V25_REG_AX)(15 downto 8) :=
                    std_logic_vector(unsigned(regs(active_bank)(V25_REG_AX)(15 downto 8)) - 1);
                  f.af := '1';
                  f.cf := '1';
                else
                  r8 := a8;
                  f.af := '0';
                  f.cf := '0';
                end if;

                reg_bank := v25_set_reg8(reg_bank, 0, r8 and x"0F");
                regs(active_bank) <= reg_bank;
                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_CVTBD =>
                if imm16(7 downto 0) /= x"0A" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  a8 := v25_get_reg8(regs(active_bank), 0);
                  tmp_nat := to_integer(unsigned(a8));
                  r8 := std_logic_vector(to_unsigned(tmp_nat mod 10, 8));
                  reg_bank := v25_set_reg8(regs(active_bank), 0, r8);
                  reg_bank := v25_set_reg8(reg_bank, 4, std_logic_vector(to_unsigned(tmp_nat / 10, 8)));
                  f := flags;
                  f.sf := r8(7);
                  if r8 = x"00" then
                    f.zf := '1';
                  else
                    f.zf := '0';
                  end if;
                  f.pf := v25_even_parity8(r8);
                  regs(active_bank) <= reg_bank;
                  flags <= f;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_CVTDB =>
                if imm16(7 downto 0) /= x"0A" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  a8 := v25_get_reg8(regs(active_bank), 0);
                  b8 := v25_get_reg8(regs(active_bank), 4);
                  tmp_nat := (to_integer(unsigned(b8)) * 10) + to_integer(unsigned(a8));
                  r8 := std_logic_vector(to_unsigned(tmp_nat mod 256, 8));
                  reg_bank := v25_set_reg8(regs(active_bank), 0, r8);
                  reg_bank := v25_set_reg8(reg_bank, 4, x"00");
                  f := flags;
                  f.sf := r8(7);
                  if r8 = x"00" then
                    f.zf := '1';
                  else
                    f.zf := '0';
                  end if;
                  f.pf := v25_even_parity8(r8);
                  regs(active_bank) <= reg_bank;
                  flags <= f;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_TEST_AL_IMM8 =>
                a8 := v25_get_reg8(regs(active_bank), 0);
                r8 := a8 and imm16(7 downto 0);
                flags <= v25_logic_flags8(flags, r8);
                state <= ST_FETCH_REQ;

              when OP_TEST_AX_IMM16 =>
                a16 := regs(active_bank)(V25_REG_AX);
                r16 := a16 and imm16;
                flags <= v25_logic_flags16(flags, r16);
                state <= ST_FETCH_REQ;

              when OP_INC_R16 =>
                a16 := regs(active_bank)(op_reg);
                r16 := std_logic_vector(unsigned(a16) + 1);
                f := v25_add_flags16(flags, a16, x"0001", '0');
                f.cf := flags.cf;
                regs(active_bank)(op_reg) <= r16;
                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_DEC_R16 =>
                a16 := regs(active_bank)(op_reg);
                r16 := std_logic_vector(unsigned(a16) - 1);
                f := v25_sub_flags16(flags, a16, x"0001", '0');
                f.cf := flags.cf;
                regs(active_bank)(op_reg) <= r16;
                flags <= f;
                state <= ST_FETCH_REQ;

              when OP_PUSH_R16 =>
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= regs(active_bank)(op_reg);
                push_mode <= PUSH_ONLY;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_POP_R16 =>
                pop_target <= op_reg;
                pop_mode <= POP_TO_REG;
                state <= ST_POP_LO_REQ;

              when OP_PUSH_SREG =>
                seg_value := v25_sreg_value(mem_sreg_target, ps, ss, ds0, ds1);
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= seg_value;
                push_mode <= PUSH_ONLY;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_POP_SREG =>
                pop_mode <= POP_TO_SREG;
                state <= ST_POP_LO_REQ;

              when OP_PUSH_PSW =>
                r16 := v25_pack_psw(flags);
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= r16;
                push_mode <= PUSH_ONLY;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_POP_PSW =>
                pop_mode <= POP_TO_PSW;
                state <= ST_POP_LO_REQ;

              when OP_MOV_PSW_AH =>
                flags <= v25_update_flags_from_ah(flags, v25_get_reg8(regs(active_bank), 4));
                state <= ST_FETCH_REQ;

              when OP_PUSH_REGS =>
                push_sp_save <= regs(active_bank)(V25_REG_SP);
                push_regs_index <= 0;
                r16 := push_regs_value(regs(active_bank), 0, regs(active_bank)(V25_REG_SP));
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= r16;
                push_mode <= PUSH_REGS;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_POP_REGS =>
                pop_regs_index <= 7;
                pop_mode <= POP_TO_REGS;
                state <= ST_POP_LO_REQ;

              when OP_PUSH_IMM16 =>
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= imm16;
                push_mode <= PUSH_ONLY;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_PUSH_IMM8_SIGN =>
                r16 := v25_sign_extend8(imm16(7 downto 0));
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= r16;
                push_mode <= PUSH_ONLY;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_POP_RM16 =>
                if modrm(5 downto 3) /= "000" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                elsif v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  pop_mode <= POP_TO_MEM;
                  state <= ST_POP_LO_REQ;
                else
                  pop_target <= to_integer(unsigned(modrm(2 downto 0)));
                  pop_mode <= POP_TO_REG;
                  state <= ST_POP_LO_REQ;
                end if;

              when OP_PREPARE =>
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                prepare_level <= to_integer(unsigned(far_seg(7 downto 0)));
                push_value <= regs(active_bank)(V25_REG_BP);
                push_mode <= PUSH_PREPARE;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_DISPOSE =>
                pop_target <= V25_REG_BP;
                pop_mode <= POP_TO_REG;
                regs(active_bank)(V25_REG_SP) <= regs(active_bank)(V25_REG_BP);
                state <= ST_POP_LO_REQ;

              when OP_CALL_REL16 =>
                branch_ip <= std_logic_vector(unsigned(ip) + unsigned(imm16));
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= ip;
                push_mode <= PUSH_THEN_JUMP;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_CALL_FAR_IMM =>
                branch_ip <= imm16;
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 4);
                push_value <= ip;
                push_mode <= PUSH_FAR_THEN_JUMP;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;

              when OP_RET_NEAR =>
                pop_mode <= POP_TO_IP;
                state <= ST_POP_LO_REQ;

              when OP_RET_NEAR_IMM =>
                stack_adjust <= imm16;
                pop_mode <= POP_TO_IP_ADJ;
                state <= ST_POP_LO_REQ;

              when OP_RET_FAR =>
                pop_mode <= POP_TO_IP_FAR;
                state <= ST_POP_LO_REQ;

              when OP_RET_FAR_IMM =>
                stack_adjust <= imm16;
                pop_mode <= POP_TO_IP_FAR_ADJ;
                state <= ST_POP_LO_REQ;

              when OP_RETI =>
                pop_mode <= POP_TO_IP_FAR_PSW;
                state <= ST_POP_LO_REQ;

              when OP_BRK =>
                int_vector_base <= interrupt_vector_base(imm16(7 downto 0));
                int_ibrk_after <= '0';
                int_return_ip <= ip;
                mem_valid_r <= '1';
                mem_write_r <= '0';
                mem_addr_r  <= interrupt_vector_base(imm16(7 downto 0));
                state <= ST_INT_VEC_IP_LO_REQ;

              when OP_BRKV =>
                if flags.oflag = '1' then
                  int_vector_base <= interrupt_vector_base(x"04");
                  int_ibrk_after <= '0';
                  int_return_ip <= ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(x"04");
                  state <= ST_INT_VEC_IP_LO_REQ;
                else
                  state <= ST_FETCH_REQ;
                end if;

              when OP_IN_AL_IMM8 | OP_IN_AX_IMM8 | OP_IN_AL_DX | OP_IN_AX_DX =>
                if flags.ibrk = '0' then
                  int_vector_base <= interrupt_vector_base(x"13");
                  int_ibrk_after <= '1';
                  int_return_ip <= instr_start_ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(x"13");
                  state <= ST_INT_VEC_IP_LO_REQ;
                else
                  if op_kind = OP_IN_AL_IMM8 or op_kind = OP_IN_AX_IMM8 then
                    io_addr_r <= x"00" & imm16(7 downto 0);
                  else
                    io_addr_r <= regs(active_bank)(V25_REG_DX);
                  end if;
                  io_valid_r <= '1';
                  io_write_r <= '0';
                  state <= ST_IO_RD_LO_WAIT;
                end if;

              when OP_OUT_IMM8_AL | OP_OUT_IMM8_AX | OP_OUT_DX_AL | OP_OUT_DX_AX =>
                if flags.ibrk = '0' then
                  int_vector_base <= interrupt_vector_base(x"13");
                  int_ibrk_after <= '1';
                  int_return_ip <= instr_start_ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(x"13");
                  state <= ST_INT_VEC_IP_LO_REQ;
                else
                  if op_kind = OP_OUT_IMM8_AL or op_kind = OP_OUT_IMM8_AX then
                    io_addr_r <= x"00" & imm16(7 downto 0);
                  else
                    io_addr_r <= regs(active_bank)(V25_REG_DX);
                  end if;
                  io_valid_r <= '1';
                  io_write_r <= '1';
                  io_wdata_r <= regs(active_bank)(V25_REG_AX)(7 downto 0);
                  state <= ST_IO_WR_LO_WAIT;
                end if;

              when OP_INM8 | OP_INM16 =>
                if flags.ibrk = '0' then
                  int_vector_base <= interrupt_vector_base(x"13");
                  int_ibrk_after <= '1';
                  int_return_ip <= instr_start_ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(x"13");
                  state <= ST_INT_VEC_IP_LO_REQ;
                elsif seg_override /= SEG_DEFAULT then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  io_addr_r <= regs(active_bank)(V25_REG_DX);
                  io_valid_r <= '1';
                  io_write_r <= '0';
                  state <= ST_IO_RD_LO_WAIT;
                end if;

              when OP_OUTM8 | OP_OUTM16 =>
                if flags.ibrk = '0' then
                  int_vector_base <= interrupt_vector_base(x"13");
                  int_ibrk_after <= '1';
                  int_return_ip <= instr_start_ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(x"13");
                  state <= ST_INT_VEC_IP_LO_REQ;
                else
                  seg_value := v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1);
                  ea_addr := v25_phys_addr(seg_value, regs(active_bank)(V25_REG_SI));
                  mem_op_addr <= ea_addr;
                  if op_kind = OP_OUTM16 then
                    mem_is_word <= '1';
                  else
                    mem_is_word <= '0';
                  end if;
                  state <= ST_MEM_RD_LO_REQ;
                end if;

              when OP_FPO1 | OP_FPO2 =>
                int_vector_base <= interrupt_vector_base(x"07");
                int_ibrk_after <= '0';
                int_return_ip <= instr_start_ip;
                mem_valid_r <= '1';
                mem_write_r <= '0';
                mem_addr_r  <= interrupt_vector_base(x"07");
                state <= ST_INT_VEC_IP_LO_REQ;

              when OP_CHKIND =>
                if v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_target <= to_integer(unsigned(modrm(5 downto 3)));
                  begin_mem_read_low(ea_addr, '1');
                else
                  fault_r <= '1';
                  state <= ST_FAULT;
                end if;

              when OP_INS_FIELD | OP_EXT_FIELD =>
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                if ((v25_subop = x"31" or v25_subop = x"33") and modrm(7 downto 6) /= "11") or
                   ((v25_subop = x"39" or v25_subop = x"3B") and modrm(7 downto 3) /= "11000") or
                   (op_kind = OP_INS_FIELD and seg_override /= SEG_DEFAULT) then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  field_reg <= rm_idx;
                  field_offset <= to_integer(unsigned(v25_get_reg8(regs(active_bank), rm_idx)(3 downto 0)));
                  if v25_subop = x"31" or v25_subop = x"33" then
                    src_idx := to_integer(unsigned(modrm(5 downto 3)));
                    field_length <= to_integer(unsigned(v25_get_reg8(regs(active_bank), src_idx)(3 downto 0))) + 1;
                  else
                    field_length <= to_integer(unsigned(imm16(3 downto 0))) + 1;
                  end if;

                  if op_kind = OP_EXT_FIELD then
                    if v25_subop = x"33" then
                      timing_counter <= v25_clocks_ext_field_reg(true);
                    else
                      timing_counter <= v25_clocks_ext_field_imm(true);
                    end if;
                    seg_value := v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1);
                    ea_addr := v25_phys_addr(seg_value, regs(active_bank)(V25_REG_SI));
                  else
                    if v25_subop = x"31" then
                      timing_counter <= v25_clocks_ins_field_reg(true);
                    else
                      timing_counter <= v25_clocks_ins_field_imm(true);
                    end if;
                    ea_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                  end if;
                  field_addr <= ea_addr;
                  state <= ST_FIELD_RD0_REQ;
                end if;

              when OP_JMP_REL8 =>
                ip <= std_logic_vector(unsigned(ip) + unsigned(v25_sign_extend8(imm16(7 downto 0))));
                state <= ST_FETCH_REQ;

              when OP_JMP_REL16 =>
                ip <= std_logic_vector(unsigned(ip) + unsigned(imm16));
                state <= ST_FETCH_REQ;

              when OP_JMP_FAR_IMM =>
                ip <= imm16;
                ps <= far_seg;
                bank_ps(active_bank) <= far_seg;
                state <= ST_FETCH_REQ;

              when OP_GRP_FE =>
                group_bits := modrm(5 downto 3);
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));

                if group_bits = "000" or group_bits = "001" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    if group_bits = "000" then
                      op_kind <= OP_INC_RM8;
                    else
                      op_kind <= OP_DEC_RM8;
                    end if;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '0');
                  else
                    a8 := v25_get_reg8(regs(active_bank), rm_idx);
                    if group_bits = "000" then
                      r8 := std_logic_vector(unsigned(a8) + 1);
                      f := v25_add_flags8(flags, a8, x"01", '0');
                    else
                      r8 := std_logic_vector(unsigned(a8) - 1);
                      f := v25_sub_flags8(flags, a8, x"01", '0');
                    end if;
                    f.cf := flags.cf;
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                    flags <= f;
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  fault_r <= '1';
                  state <= ST_FAULT;
                end if;

              when OP_GRP_FF =>
                group_bits := modrm(5 downto 3);
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));

                if group_bits = "000" or group_bits = "001" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    if group_bits = "000" then
                      op_kind <= OP_INC_RM16;
                    else
                      op_kind <= OP_DEC_RM16;
                    end if;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '1');
                  else
                    a16 := regs(active_bank)(rm_idx);
                    if group_bits = "000" then
                      r16 := std_logic_vector(unsigned(a16) + 1);
                      f := v25_add_flags16(flags, a16, x"0001", '0');
                    else
                      r16 := std_logic_vector(unsigned(a16) - 1);
                      f := v25_sub_flags16(flags, a16, x"0001", '0');
                    end if;
                    f.cf := flags.cf;
                    regs(active_bank)(rm_idx) <= r16;
                    flags <= f;
                    state <= ST_FETCH_REQ;
                  end if;
                elsif group_bits = "010" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    op_kind <= OP_CALL_RM16;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '1');
                  else
                    branch_ip <= regs(active_bank)(rm_idx);
                    sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                    push_value <= ip;
                    push_mode <= PUSH_THEN_JUMP;
                    regs(active_bank)(V25_REG_SP) <= sp_next;
                    state <= ST_PUSH_LO_REQ;
                  end if;
                elsif group_bits = "011" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    op_kind <= OP_CALL_M32;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '1');
                  else
                    fault_r <= '1';
                    state <= ST_FAULT;
                  end if;
                elsif group_bits = "100" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    op_kind <= OP_JMP_RM16;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '1');
                  else
                    ip <= regs(active_bank)(rm_idx);
                    state <= ST_FETCH_REQ;
                  end if;
                elsif group_bits = "101" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    op_kind <= OP_JMP_M32;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '1');
                  else
                    fault_r <= '1';
                    state <= ST_FAULT;
                  end if;
                elsif group_bits = "110" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    op_kind <= OP_PUSH_RM16;
                    mem_op_addr <= ea_addr;
                    begin_mem_read_low(ea_addr, '1');
                  else
                    sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                    push_value <= regs(active_bank)(rm_idx);
                    push_mode <= PUSH_ONLY;
                    regs(active_bank)(V25_REG_SP) <= sp_next;
                    state <= ST_PUSH_LO_REQ;
                  end if;
                else
                  fault_r <= '1';
                  state <= ST_FAULT;
                end if;

              when OP_JCC_REL8 =>
                branch_taken := v25_jcc_taken(std_logic_vector(to_unsigned(op_reg, 4)), flags);
                if branch_taken then
                  ip <= std_logic_vector(unsigned(ip) + unsigned(v25_sign_extend8(imm16(7 downto 0))));
                end if;
                if ENABLE_TIMING_THROTTLE then
                  timing_counter <= v25_clocks_cond_branch(branch_taken);
                end if;
                state <= ST_FETCH_REQ;

              when OP_LOOP_REL8 =>
                branch_taken := false;
                if op_reg = 3 then
                  if regs(active_bank)(V25_REG_CX) = x"0000" then
                    branch_taken := true;
                    ip <= std_logic_vector(unsigned(ip) + unsigned(v25_sign_extend8(imm16(7 downto 0))));
                  end if;
                else
                  r16 := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  regs(active_bank)(V25_REG_CX) <= r16;

                  if r16 /= x"0000" then
                    if op_reg = 2 or
                       (op_reg = 1 and flags.zf = '1') or
                       (op_reg = 0 and flags.zf = '0') then
                      branch_taken := true;
                      ip <= std_logic_vector(unsigned(ip) + unsigned(v25_sign_extend8(imm16(7 downto 0))));
                    end if;
                  end if;
                end if;
                if ENABLE_TIMING_THROTTLE then
                  if op_reg = 3 then
                    timing_counter <= v25_clocks_bcwz(branch_taken);
                  else
                    timing_counter <= v25_clocks_dbnz(branch_taken);
                  end if;
                end if;
                state <= ST_FETCH_REQ;

              when OP_XCHG_AX_R16 =>
                r16 := regs(active_bank)(V25_REG_AX);
                regs(active_bank)(V25_REG_AX) <= regs(active_bank)(op_reg);
                regs(active_bank)(op_reg) <= r16;
                state <= ST_FETCH_REQ;

              when OP_LEA_R16_MEM =>
                if not v25_modrm_is_memory(modrm) then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  src_idx := to_integer(unsigned(modrm(5 downto 3)));
                  regs(active_bank)(src_idx) <= v25_modrm_ea(regs(active_bank), modrm, disp16);
                  state <= ST_FETCH_REQ;
                end if;

              when OP_XCHG_RM8_R8 | OP_XCHG_RM16_R16 =>
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                src_idx := to_integer(unsigned(modrm(5 downto 3)));

                if not v25_modrm_is_memory(modrm) then
                  if op_kind = OP_XCHG_RM8_R8 then
                    a8 := v25_get_reg8(regs(active_bank), rm_idx);
                    b8 := v25_get_reg8(regs(active_bank), src_idx);
                    reg_bank := regs(active_bank);
                    reg_bank := v25_set_reg8(reg_bank, rm_idx, b8);
                    reg_bank := v25_set_reg8(reg_bank, src_idx, a8);
                    regs(active_bank) <= reg_bank;
                  else
                    a16 := regs(active_bank)(rm_idx);
                    regs(active_bank)(rm_idx) <= regs(active_bank)(src_idx);
                    regs(active_bank)(src_idx) <= a16;
                  end if;
                  state <= ST_FETCH_REQ;
                else
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_target <= src_idx;
                  if op_kind = OP_XCHG_RM16_R16 then
                    begin_mem_read_low(ea_addr, '1');
                  else
                    begin_mem_read_low(ea_addr, '0');
                  end if;
                end if;

              when OP_ALU_RM8_R8 | OP_ALU_R8_RM8 |
                   OP_ALU_RM16_R16 | OP_ALU_R16_RM16 =>
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                src_idx := to_integer(unsigned(modrm(5 downto 3)));

                if not v25_modrm_is_memory(modrm) then
                  if op_kind = OP_ALU_RM8_R8 then
                    a8 := v25_get_reg8(regs(active_bank), rm_idx);
                    b8 := v25_get_reg8(regs(active_bank), src_idx);
                    r8 := alu_result8(alu_func, a8, b8, flags.cf);
                    f := alu_flags8(alu_func, flags, a8, b8);
                    if alu_func /= ALU_CMP then
                      regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                    end if;
                    flags <= f;
                  elsif op_kind = OP_ALU_R8_RM8 then
                    a8 := v25_get_reg8(regs(active_bank), src_idx);
                    b8 := v25_get_reg8(regs(active_bank), rm_idx);
                    r8 := alu_result8(alu_func, a8, b8, flags.cf);
                    f := alu_flags8(alu_func, flags, a8, b8);
                    if alu_func /= ALU_CMP then
                      regs(active_bank) <= v25_set_reg8(regs(active_bank), src_idx, r8);
                    end if;
                    flags <= f;
                  elsif op_kind = OP_ALU_RM16_R16 then
                    a16 := regs(active_bank)(rm_idx);
                    b16 := regs(active_bank)(src_idx);
                    r16 := alu_result16(alu_func, a16, b16, flags.cf);
                    f := alu_flags16(alu_func, flags, a16, b16);
                    if alu_func /= ALU_CMP then
                      regs(active_bank)(rm_idx) <= r16;
                    end if;
                    flags <= f;
                  else
                    a16 := regs(active_bank)(src_idx);
                    b16 := regs(active_bank)(rm_idx);
                    r16 := alu_result16(alu_func, a16, b16, flags.cf);
                    f := alu_flags16(alu_func, flags, a16, b16);
                    if alu_func /= ALU_CMP then
                      regs(active_bank)(src_idx) <= r16;
                    end if;
                    flags <= f;
                  end if;
                  state <= ST_FETCH_REQ;
                else
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_target <= src_idx;
                  if op_kind = OP_ALU_RM16_R16 or op_kind = OP_ALU_R16_RM16 then
                    begin_mem_read_low(ea_addr, '1');
                  else
                    begin_mem_read_low(ea_addr, '0');
                  end if;
                end if;

              when OP_TEST_RM8_R8 | OP_TEST_RM16_R16 =>
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                src_idx := to_integer(unsigned(modrm(5 downto 3)));

                if not v25_modrm_is_memory(modrm) then
                  if op_kind = OP_TEST_RM8_R8 then
                    a8 := v25_get_reg8(regs(active_bank), rm_idx);
                    b8 := v25_get_reg8(regs(active_bank), src_idx);
                    flags <= v25_logic_flags8(flags, a8 and b8);
                  else
                    a16 := regs(active_bank)(rm_idx);
                    b16 := regs(active_bank)(src_idx);
                    flags <= v25_logic_flags16(flags, a16 and b16);
                  end if;
                  state <= ST_FETCH_REQ;
                else
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_target <= src_idx;
                  if op_kind = OP_TEST_RM16_R16 then
                    begin_mem_read_low(ea_addr, '1');
                  else
                    begin_mem_read_low(ea_addr, '0');
                  end if;
                end if;

              when OP_IMUL_R16_RM16_IMM16 | OP_IMUL_R16_RM16_IMM8 =>
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                src_idx := to_integer(unsigned(modrm(5 downto 3)));

                if op_kind = OP_IMUL_R16_RM16_IMM8 then
                  b16 := v25_sign_extend8(imm16(7 downto 0));
                else
                  b16 := imm16;
                end if;

                if not v25_modrm_is_memory(modrm) then
                  prod32s := signed(regs(active_bank)(rm_idx)) * signed(b16);
                  regs(active_bank)(src_idx) <= std_logic_vector(prod32s(15 downto 0));
                  if prod32s >= to_signed(-32768, 32) and prod32s <= to_signed(32767, 32) then
                    flags.cf <= '0';
                    flags.oflag <= '0';
                  else
                    flags.cf <= '1';
                    flags.oflag <= '1';
                  end if;
                  state <= ST_FETCH_REQ;
                else
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_target <= src_idx;
                  begin_mem_read_low(ea_addr, '1');
                end if;

              when OP_MOV_RM8_R8 | OP_MOV_R8_RM8 |
                   OP_MOV_RM16_R16 | OP_MOV_R16_RM16 =>
                rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                src_idx := to_integer(unsigned(modrm(5 downto 3)));

                if not v25_modrm_is_memory(modrm) then
                  case op_kind is
                    when OP_MOV_RM8_R8 =>
                      regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, v25_get_reg8(regs(active_bank), src_idx));
                    when OP_MOV_R8_RM8 =>
                      regs(active_bank) <= v25_set_reg8(regs(active_bank), src_idx, v25_get_reg8(regs(active_bank), rm_idx));
                    when OP_MOV_RM16_R16 =>
                      regs(active_bank)(rm_idx) <= regs(active_bank)(src_idx);
                    when others =>
                      regs(active_bank)(src_idx) <= regs(active_bank)(rm_idx);
                  end case;
                  state <= ST_FETCH_REQ;
                else
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;

                  case op_kind is
                    when OP_MOV_RM8_R8 =>
                      if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                        tmp_nat := v25_internal_data_index(ea_addr);
                        write_internal_byte(
                          internal_data,
                          flags,
                          idb_high,
                          bank_vector_ip,
                          bank_saved_ip,
                          bank_saved_psw,
                          bank_ps,
                          bank_ss,
                          bank_ds0,
                          bank_ds1,
                          regs,
                          rfm_rflv_slave,
                          ps,
                          ss,
                          ds0,
                          ds1,
                          active_bank,
                          tmp_nat,
                          v25_get_reg8(regs(active_bank), src_idx)
                        );
                        state <= ST_FETCH_REQ;
                      else
                        mem_value <= x"00" & v25_get_reg8(regs(active_bank), src_idx);
                        mem_is_word <= '0';
                        mem_valid_r <= '1';
                        mem_write_r <= '1';
                        mem_addr_r  <= ea_addr;
                        mem_wdata_r <= v25_get_reg8(regs(active_bank), src_idx);
                        state <= ST_MEM_WR_LO_WAIT;
                      end if;
                    when OP_MOV_RM16_R16 =>
                      if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                        tmp_nat := v25_internal_data_index(ea_addr);
                        write_internal_word(
                          internal_data,
                          flags,
                          idb_high,
                          bank_vector_ip,
                          bank_saved_ip,
                          bank_saved_psw,
                          bank_ps,
                          bank_ss,
                          bank_ds0,
                          bank_ds1,
                          regs,
                          rfm_rflv_slave,
                          ps,
                          ss,
                          ds0,
                          ds1,
                          active_bank,
                          tmp_nat,
                          regs(active_bank)(src_idx)
                        );
                        state <= ST_FETCH_REQ;
                      else
                        mem_value <= regs(active_bank)(src_idx);
                        mem_is_word <= '1';
                        mem_valid_r <= '1';
                        mem_write_r <= '1';
                        mem_addr_r  <= ea_addr;
                        mem_wdata_r <= regs(active_bank)(src_idx)(7 downto 0);
                        state <= ST_MEM_WR_LO_WAIT;
                      end if;
                    when OP_MOV_R8_RM8 =>
                      if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                        tmp_nat := v25_internal_data_index(ea_addr);
                        read_internal_index_byte(tmp_nat, mem_byte);
                        regs(active_bank) <= v25_set_reg8(
                          regs(active_bank),
                          src_idx,
                          mem_byte
                        );
                        state <= ST_FETCH_REQ;
                      else
                        mem_target <= src_idx;
                        mem_is_word <= '0';
                        mem_valid_r <= '1';
                        mem_write_r <= '0';
                        mem_addr_r  <= ea_addr;
                        state <= ST_MEM_RD_LO_WAIT;
                      end if;
                    when others =>
                      if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                        tmp_nat := v25_internal_data_index(ea_addr);
                        read_internal_index_byte((tmp_nat + 1) mod 512, b8);
                        read_internal_index_byte(tmp_nat, a8);
                        regs(active_bank)(src_idx) <= b8 & a8;
                        state <= ST_FETCH_REQ;
                      else
                        mem_target <= src_idx;
                        mem_is_word <= '1';
                        mem_valid_r <= '1';
                        mem_write_r <= '0';
                        mem_addr_r  <= ea_addr;
                        state <= ST_MEM_RD_LO_WAIT;
                      end if;
                  end case;
                end if;

              when OP_MOV_RM16_SREG =>
                if modrm(5) = '1' then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  seg_sel := v25_sreg_select(modrm(4 downto 3));
                  seg_value := v25_sreg_value(seg_sel, ps, ss, ds0, ds1);

                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    begin_mem_write_low(ea_addr, seg_value, '1');
                  else
                    rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                    regs(active_bank)(rm_idx) <= seg_value;
                    state <= ST_FETCH_REQ;
                  end if;
                end if;

              when OP_MOV_SREG_RM16 =>
                seg_sel := v25_sreg_select(modrm(4 downto 3));

                if modrm(5) = '1' or seg_sel = SEG_PS then
                  fault_r <= '1';
                  state <= ST_FAULT;
                elsif v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_sreg_target <= seg_sel;
                  begin_mem_read_low(ea_addr, '1');
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  case seg_sel is
                    when SEG_SS =>
                      ss <= regs(active_bank)(rm_idx);
                      bank_ss(active_bank) <= regs(active_bank)(rm_idx);
                    when SEG_DS0 =>
                      ds0 <= regs(active_bank)(rm_idx);
                      bank_ds0(active_bank) <= regs(active_bank)(rm_idx);
                    when SEG_DS1 =>
                      ds1 <= regs(active_bank)(rm_idx);
                      bank_ds1(active_bank) <= regs(active_bank)(rm_idx);
                    when others =>
                      fault_r <= '1';
                      state <= ST_FAULT;
                  end case;

                  if seg_sel /= SEG_PS and seg_sel /= SEG_DEFAULT then
                    state <= ST_FETCH_REQ;
                  end if;
                end if;

              when OP_MOV_DS0_R16_MEM32 | OP_MOV_DS1_R16_MEM32 =>
                if not v25_modrm_is_memory(modrm) then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_target <= to_integer(unsigned(modrm(5 downto 3)));
                  if op_kind = OP_MOV_DS0_R16_MEM32 then
                    mem_sreg_target <= SEG_DS0;
                  else
                    mem_sreg_target <= SEG_DS1;
                  end if;
                  begin_mem_read_low(ea_addr, '1');
                end if;

              when OP_MOV_RM8_IMM8 =>
                if modrm(5 downto 3) /= "000" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                elsif v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                    tmp_nat := v25_internal_data_index(ea_addr);
                    write_internal_byte(
                      internal_data,
                      flags,
                      idb_high,
                      bank_vector_ip,
                      bank_saved_ip,
                      bank_saved_psw,
                      bank_ps,
                      bank_ss,
                      bank_ds0,
                      bank_ds1,
                      regs,
                      rfm_rflv_slave,
                      ps,
                      ss,
                      ds0,
                      ds1,
                      active_bank,
                      tmp_nat,
                      imm16(7 downto 0)
                    );
                    state <= ST_FETCH_REQ;
                  else
                    mem_op_addr <= ea_addr;
                    mem_value   <= x"00" & imm16(7 downto 0);
                    mem_is_word <= '0';
                    mem_valid_r <= '1';
                    mem_write_r <= '1';
                    mem_addr_r  <= ea_addr;
                    mem_wdata_r <= imm16(7 downto 0);
                    state <= ST_MEM_WR_LO_WAIT;
                  end if;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, imm16(7 downto 0));
                  state <= ST_FETCH_REQ;
                end if;

              when OP_MOV_RM16_IMM16 =>
                if modrm(5 downto 3) /= "000" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                elsif v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                    tmp_nat := v25_internal_data_index(ea_addr);
                    write_internal_word(
                      internal_data,
                      flags,
                      idb_high,
                      bank_vector_ip,
                      bank_saved_ip,
                      bank_saved_psw,
                      bank_ps,
                      bank_ss,
                      bank_ds0,
                      bank_ds1,
                      regs,
                      rfm_rflv_slave,
                      ps,
                      ss,
                      ds0,
                      ds1,
                      active_bank,
                      tmp_nat,
                      imm16
                    );
                    state <= ST_FETCH_REQ;
                  else
                    mem_op_addr <= ea_addr;
                    mem_value   <= imm16;
                    mem_is_word <= '1';
                    mem_valid_r <= '1';
                    mem_write_r <= '1';
                    mem_addr_r  <= ea_addr;
                    mem_wdata_r <= imm16(7 downto 0);
                    state <= ST_MEM_WR_LO_WAIT;
                  end if;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  regs(active_bank)(rm_idx) <= imm16;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_MOV_AL_MOFFS =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  imm16
                );
                if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                  tmp_nat := v25_internal_data_index(ea_addr);
                  read_internal_index_byte(tmp_nat, mem_byte);
                  regs(active_bank) <= v25_set_reg8(
                    regs(active_bank),
                    0,
                    mem_byte
                  );
                  state <= ST_FETCH_REQ;
                else
                  mem_op_addr <= ea_addr;
                  mem_is_word <= '0';
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= ea_addr;
                  state <= ST_MEM_RD_LO_WAIT;
                end if;

              when OP_MOV_AX_MOFFS =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  imm16
                );
                if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                  tmp_nat := v25_internal_data_index(ea_addr);
                  read_internal_index_byte((tmp_nat + 1) mod 512, b8);
                  read_internal_index_byte(tmp_nat, a8);
                  regs(active_bank)(V25_REG_AX) <= b8 & a8;
                  state <= ST_FETCH_REQ;
                else
                  mem_op_addr <= ea_addr;
                  mem_is_word <= '1';
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= ea_addr;
                  state <= ST_MEM_RD_LO_WAIT;
                end if;

              when OP_MOV_MOFFS_AL =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  imm16
                );
                if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                  tmp_nat := v25_internal_data_index(ea_addr);
                  write_internal_byte(
                    internal_data,
                    flags,
                    idb_high,
                    bank_vector_ip,
                    bank_saved_ip,
                    bank_saved_psw,
                    bank_ps,
                    bank_ss,
                    bank_ds0,
                    bank_ds1,
                    regs,
                    rfm_rflv_slave,
                    ps,
                    ss,
                    ds0,
                    ds1,
                    active_bank,
                    tmp_nat,
                    regs(active_bank)(V25_REG_AX)(7 downto 0)
                  );
                  state <= ST_FETCH_REQ;
                else
                  mem_op_addr <= ea_addr;
                  mem_value <= x"00" & regs(active_bank)(V25_REG_AX)(7 downto 0);
                  mem_is_word <= '0';
                  mem_valid_r <= '1';
                  mem_write_r <= '1';
                  mem_addr_r  <= ea_addr;
                  mem_wdata_r <= regs(active_bank)(V25_REG_AX)(7 downto 0);
                  state <= ST_MEM_WR_LO_WAIT;
                end if;

              when OP_MOV_MOFFS_AX =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  imm16
                );
                if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                  tmp_nat := v25_internal_data_index(ea_addr);
                  write_internal_word(
                    internal_data,
                    flags,
                    idb_high,
                    bank_vector_ip,
                    bank_saved_ip,
                    bank_saved_psw,
                    bank_ps,
                    bank_ss,
                    bank_ds0,
                    bank_ds1,
                    regs,
                    rfm_rflv_slave,
                    ps,
                    ss,
                    ds0,
                    ds1,
                    active_bank,
                    tmp_nat,
                    regs(active_bank)(V25_REG_AX)
                  );
                  state <= ST_FETCH_REQ;
                else
                  mem_op_addr <= ea_addr;
                  mem_value <= regs(active_bank)(V25_REG_AX);
                  mem_is_word <= '1';
                  mem_valid_r <= '1';
                  mem_write_r <= '1';
                  mem_addr_r  <= ea_addr;
                  mem_wdata_r <= regs(active_bank)(V25_REG_AX)(7 downto 0);
                  state <= ST_MEM_WR_LO_WAIT;
                end if;

              when OP_XLAT =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  std_logic_vector(
                    unsigned(regs(active_bank)(V25_REG_BX)) +
                    resize(unsigned(v25_get_reg8(regs(active_bank), 0)), 16)
                  )
                );
                mem_op_addr <= ea_addr;
                mem_is_word <= '0';
                state <= ST_MEM_RD_LO_REQ;

              when OP_MOVS8 | OP_MOVS16 =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  regs(active_bank)(V25_REG_SI)
                );
                mem_op_addr <= ea_addr;
                if op_kind = OP_MOVS16 then
                  mem_is_word <= '1';
                else
                  mem_is_word <= '0';
                end if;
                state <= ST_MEM_RD_LO_REQ;

              when OP_CMPS8 | OP_CMPS16 =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  regs(active_bank)(V25_REG_SI)
                );
                mem_op_addr <= ea_addr;
                if op_kind = OP_CMPS16 then
                  mem_is_word <= '1';
                else
                  mem_is_word <= '0';
                end if;
                state <= ST_MEM_RD_LO_REQ;

              when OP_STOS8 =>
                ea_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                mem_op_addr <= ea_addr;
                mem_value <= x"00" & v25_get_reg8(regs(active_bank), 0);
                mem_is_word <= '0';
                mem_wdata_r <= v25_get_reg8(regs(active_bank), 0);
                state <= ST_MEM_WR_LO_REQ;

              when OP_STOS16 =>
                ea_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                mem_op_addr <= ea_addr;
                mem_value <= regs(active_bank)(V25_REG_AX);
                mem_is_word <= '1';
                mem_wdata_r <= regs(active_bank)(V25_REG_AX)(7 downto 0);
                state <= ST_MEM_WR_LO_REQ;

              when OP_LODS8 | OP_LODS16 =>
                ea_addr := v25_phys_addr(
                  v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1),
                  regs(active_bank)(V25_REG_SI)
                );
                mem_op_addr <= ea_addr;
                if op_kind = OP_LODS16 then
                  mem_is_word <= '1';
                else
                  mem_is_word <= '0';
                end if;
                state <= ST_MEM_RD_LO_REQ;

              when OP_SCAS8 | OP_SCAS16 =>
                ea_addr := v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                mem_op_addr <= ea_addr;
                if op_kind = OP_SCAS16 then
                  mem_is_word <= '1';
                else
                  mem_is_word <= '0';
                end if;
                state <= ST_MEM_RD_LO_REQ;

              when OP_GRP_IMM8_RM8 =>
                group_bits := modrm(5 downto 3);
                alu_func <= group_to_alu(group_bits);

                if v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_is_word <= '0';
                  state <= ST_MEM_RD_LO_REQ;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  a8 := v25_get_reg8(regs(active_bank), rm_idx);
                  b8 := imm16(7 downto 0);
                  r8 := alu_result8(group_to_alu(group_bits), a8, b8, flags.cf);
                  f := alu_flags8(group_to_alu(group_bits), flags, a8, b8);

                  if group_to_alu(group_bits) /= ALU_CMP then
                      regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                  end if;

                  flags <= f;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_GRP_IMM16_RM16 | OP_GRP_IMM8_RM16_SIGN =>
                group_bits := modrm(5 downto 3);
                alu_func <= group_to_alu(group_bits);

                if v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_is_word <= '1';
                  state <= ST_MEM_RD_LO_REQ;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  a16 := regs(active_bank)(rm_idx);
                  if op_kind = OP_GRP_IMM8_RM16_SIGN then
                    b16 := v25_sign_extend8(imm16(7 downto 0));
                  else
                    b16 := imm16;
                  end if;

                  r16 := alu_result16(group_to_alu(group_bits), a16, b16, flags.cf);
                  f := alu_flags16(group_to_alu(group_bits), flags, a16, b16);

                  if group_to_alu(group_bits) /= ALU_CMP then
                      regs(active_bank)(rm_idx) <= r16;
                  end if;

                  flags <= f;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_GRP3_RM8 | OP_GRP3_RM16 =>
                group_bits := modrm(5 downto 3);

                if group_bits = "000" or group_bits = "010" or group_bits = "011" or
                   group_bits = "100" or group_bits = "101" or
                   group_bits = "110" or group_bits = "111" then
                  if v25_modrm_is_memory(modrm) then
                    ea_addr := v25_phys_addr(
                      v25_selected_seg(
                        seg_override,
                        v25_modrm_default_seg(ss, ds0, modrm),
                        ps,
                        ss,
                        ds0,
                        ds1
                      ),
                      v25_modrm_ea(regs(active_bank), modrm, disp16)
                    );
                    mem_op_addr <= ea_addr;
                    if op_kind = OP_GRP3_RM16 then
                      begin_mem_read_low(ea_addr, '1');
                    else
                      begin_mem_read_low(ea_addr, '0');
                    end if;
                  else
                    rm_idx := to_integer(unsigned(modrm(2 downto 0)));

                    if op_kind = OP_GRP3_RM8 then
                      a8 := v25_get_reg8(regs(active_bank), rm_idx);
                      case group_bits is
                        when "000" =>
                          flags <= v25_logic_flags8(flags, a8 and imm16(7 downto 0));
                        when "010" =>
                          regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, not a8);
                        when "011" =>
                          r8 := std_logic_vector(to_unsigned(0, 8) - unsigned(a8));
                          f := v25_sub_flags8(flags, x"00", a8, '0');
                          regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                          flags <= f;
                        when "100" =>
                          prod16u := unsigned(v25_get_reg8(regs(active_bank), 0)) * unsigned(a8);
                          regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod16u);
                          if prod16u(15 downto 8) = x"00" then
                            flags.cf <= '0';
                            flags.oflag <= '0';
                          else
                            flags.cf <= '1';
                            flags.oflag <= '1';
                          end if;
                        when "101" =>
                          prod16s := signed(v25_get_reg8(regs(active_bank), 0)) * signed(a8);
                          regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod16s);
                          if prod16s >= to_signed(-128, 16) and prod16s <= to_signed(127, 16) then
                            flags.cf <= '0';
                            flags.oflag <= '0';
                          else
                            flags.cf <= '1';
                            flags.oflag <= '1';
                          end if;
                        when "110" =>
                          if a8 = x"00" then
                            int_vector_base <= interrupt_vector_base(x"00");
                            int_ibrk_after <= '0';
                            int_return_ip <= ip;
                            mem_valid_r <= '1';
                            mem_write_r <= '0';
                            mem_addr_r  <= interrupt_vector_base(x"00");
                            state <= ST_INT_VEC_IP_LO_REQ;
                            trap_started := true;
                          else
                            dividend16u := unsigned(regs(active_bank)(V25_REG_AX));
                            quotient16u := dividend16u / resize(unsigned(a8), 16);
                            remainder16u := dividend16u rem resize(unsigned(a8), 16);
                            if quotient16u(15 downto 8) /= x"00" then
                              int_vector_base <= interrupt_vector_base(x"00");
                              int_ibrk_after <= '0';
                              int_return_ip <= ip;
                              mem_valid_r <= '1';
                              mem_write_r <= '0';
                              mem_addr_r  <= interrupt_vector_base(x"00");
                              state <= ST_INT_VEC_IP_LO_REQ;
                              trap_started := true;
                            else
                              reg_bank := v25_set_reg8(regs(active_bank), 0, std_logic_vector(quotient16u(7 downto 0)));
                              reg_bank := v25_set_reg8(reg_bank, 4, std_logic_vector(remainder16u(7 downto 0)));
                              regs(active_bank) <= reg_bank;
                            end if;
                          end if;
                        when others =>
                          if a8 = x"00" then
                            int_vector_base <= interrupt_vector_base(x"00");
                            int_ibrk_after <= '0';
                            int_return_ip <= ip;
                            mem_valid_r <= '1';
                            mem_write_r <= '0';
                            mem_addr_r  <= interrupt_vector_base(x"00");
                            state <= ST_INT_VEC_IP_LO_REQ;
                            trap_started := true;
                          else
                            dividend16s := signed(regs(active_bank)(V25_REG_AX));
                            quotient16s := dividend16s / resize(signed(a8), 16);
                            remainder16s := dividend16s rem resize(signed(a8), 16);
                            if quotient16s < to_signed(-128, 16) or quotient16s > to_signed(127, 16) then
                              int_vector_base <= interrupt_vector_base(x"00");
                              int_ibrk_after <= '0';
                              int_return_ip <= ip;
                              mem_valid_r <= '1';
                              mem_write_r <= '0';
                              mem_addr_r  <= interrupt_vector_base(x"00");
                              state <= ST_INT_VEC_IP_LO_REQ;
                              trap_started := true;
                            else
                              reg_bank := v25_set_reg8(regs(active_bank), 0, std_logic_vector(quotient16s(7 downto 0)));
                              reg_bank := v25_set_reg8(reg_bank, 4, std_logic_vector(remainder16s(7 downto 0)));
                              regs(active_bank) <= reg_bank;
                            end if;
                          end if;
                      end case;
                    else
                      a16 := regs(active_bank)(rm_idx);
                      case group_bits is
                        when "000" =>
                          flags <= v25_logic_flags16(flags, a16 and imm16);
                        when "010" =>
                          regs(active_bank)(rm_idx) <= not a16;
                        when "011" =>
                          r16 := std_logic_vector(to_unsigned(0, 16) - unsigned(a16));
                          f := v25_sub_flags16(flags, x"0000", a16, '0');
                          regs(active_bank)(rm_idx) <= r16;
                          flags <= f;
                        when "100" =>
                          prod32u := unsigned(regs(active_bank)(V25_REG_AX)) * unsigned(a16);
                          regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod32u(15 downto 0));
                          regs(active_bank)(V25_REG_DX) <= std_logic_vector(prod32u(31 downto 16));
                          if prod32u(31 downto 16) = x"0000" then
                            flags.cf <= '0';
                            flags.oflag <= '0';
                          else
                            flags.cf <= '1';
                            flags.oflag <= '1';
                          end if;
                        when "101" =>
                          prod32s := signed(regs(active_bank)(V25_REG_AX)) * signed(a16);
                          regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod32s(15 downto 0));
                          regs(active_bank)(V25_REG_DX) <= std_logic_vector(prod32s(31 downto 16));
                          if prod32s >= to_signed(-32768, 32) and prod32s <= to_signed(32767, 32) then
                            flags.cf <= '0';
                            flags.oflag <= '0';
                          else
                            flags.cf <= '1';
                            flags.oflag <= '1';
                          end if;
                        when "110" =>
                          if a16 = x"0000" then
                            int_vector_base <= interrupt_vector_base(x"00");
                            int_ibrk_after <= '0';
                            int_return_ip <= ip;
                            mem_valid_r <= '1';
                            mem_write_r <= '0';
                            mem_addr_r  <= interrupt_vector_base(x"00");
                            state <= ST_INT_VEC_IP_LO_REQ;
                            trap_started := true;
                          else
                            dividend32u := unsigned(regs(active_bank)(V25_REG_DX)) & unsigned(regs(active_bank)(V25_REG_AX));
                            quotient32u := dividend32u / resize(unsigned(a16), 32);
                            remainder32u := dividend32u rem resize(unsigned(a16), 32);
                            if quotient32u(31 downto 16) /= x"0000" then
                              int_vector_base <= interrupt_vector_base(x"00");
                              int_ibrk_after <= '0';
                              int_return_ip <= ip;
                              mem_valid_r <= '1';
                              mem_write_r <= '0';
                              mem_addr_r  <= interrupt_vector_base(x"00");
                              state <= ST_INT_VEC_IP_LO_REQ;
                              trap_started := true;
                            else
                              regs(active_bank)(V25_REG_AX) <= std_logic_vector(quotient32u(15 downto 0));
                              regs(active_bank)(V25_REG_DX) <= std_logic_vector(remainder32u(15 downto 0));
                            end if;
                          end if;
                        when others =>
                          if a16 = x"0000" then
                            int_vector_base <= interrupt_vector_base(x"00");
                            int_ibrk_after <= '0';
                            int_return_ip <= ip;
                            mem_valid_r <= '1';
                            mem_write_r <= '0';
                            mem_addr_r  <= interrupt_vector_base(x"00");
                            state <= ST_INT_VEC_IP_LO_REQ;
                            trap_started := true;
                          else
                            dividend32s := signed(regs(active_bank)(V25_REG_DX)) & signed(regs(active_bank)(V25_REG_AX));
                            quotient32s := dividend32s / resize(signed(a16), 32);
                            remainder32s := dividend32s rem resize(signed(a16), 32);
                            if quotient32s < to_signed(-32768, 32) or quotient32s > to_signed(32767, 32) then
                              int_vector_base <= interrupt_vector_base(x"00");
                              int_ibrk_after <= '0';
                              int_return_ip <= ip;
                              mem_valid_r <= '1';
                              mem_write_r <= '0';
                              mem_addr_r  <= interrupt_vector_base(x"00");
                              state <= ST_INT_VEC_IP_LO_REQ;
                              trap_started := true;
                            else
                              regs(active_bank)(V25_REG_AX) <= std_logic_vector(quotient32s(15 downto 0));
                              regs(active_bank)(V25_REG_DX) <= std_logic_vector(remainder32s(15 downto 0));
                            end if;
                          end if;
                      end case;
                    end if;

                    if not trap_started then
                      state <= ST_FETCH_REQ;
                    end if;
                  end if;
                else
                  fault_r <= '1';
                  state <= ST_FAULT;
                end if;

              when OP_GRP_SHIFT_RM8_1 | OP_GRP_SHIFT_RM8_CL | OP_GRP_SHIFT_RM8_IMM =>
                group_bits := modrm(5 downto 3);
                shift_count := shift_count_for_op(op_kind, regs(active_bank), imm16);
                if ENABLE_TIMING_THROTTLE and not v25_modrm_is_memory(modrm) then
                  if op_kind = OP_GRP_SHIFT_RM8_CL then
                    timing_counter <= v25_clocks_shift_reg_cl(shift_count);
                  elsif op_kind = OP_GRP_SHIFT_RM8_IMM then
                    timing_counter <= v25_clocks_shift_reg_imm(shift_count);
                  end if;
                end if;

                if v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_is_word <= '0';
                  state <= ST_MEM_RD_LO_REQ;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  a8 := v25_get_reg8(regs(active_bank), rm_idx);
                  shift8 := shift_rotate8(group_bits, a8, shift_count, flags);
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, shift8.value);
                  flags <= shift8.flags;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_GRP_SHIFT_RM16_1 | OP_GRP_SHIFT_RM16_CL | OP_GRP_SHIFT_RM16_IMM =>
                group_bits := modrm(5 downto 3);
                shift_count := shift_count_for_op(op_kind, regs(active_bank), imm16);
                if ENABLE_TIMING_THROTTLE and not v25_modrm_is_memory(modrm) then
                  if op_kind = OP_GRP_SHIFT_RM16_CL then
                    timing_counter <= v25_clocks_shift_reg_cl(shift_count);
                  elsif op_kind = OP_GRP_SHIFT_RM16_IMM then
                    timing_counter <= v25_clocks_shift_reg_imm(shift_count);
                  end if;
                end if;

                if v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  begin_mem_read_low(ea_addr, '1');
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  a16 := regs(active_bank)(rm_idx);
                  shift16 := shift_rotate16(group_bits, a16, shift_count, flags);
                  regs(active_bank)(rm_idx) <= shift16.value;
                  flags <= shift16.flags;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_V25_PREFIX =>
                v25_subop <= imm16(7 downto 0);
                if is_v25_bitop_subop(imm16(7 downto 0)) then
                  op_kind <= OP_V25_BITOP;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"28" then
                  op_kind <= OP_ROL4;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"2A" then
                  op_kind <= OP_ROR4;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"25" then
                  op_kind <= OP_MOVSPA;
                  state <= ST_EXECUTE;
                elsif imm16(7 downto 0) = x"2D" then
                  op_kind <= OP_BRKCS;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"20" then
                  op_kind <= OP_ADD4S;
                  state <= ST_EXECUTE;
                elsif imm16(7 downto 0) = x"22" then
                  op_kind <= OP_SUB4S;
                  state <= ST_EXECUTE;
                elsif imm16(7 downto 0) = x"26" then
                  op_kind <= OP_CMP4S;
                  state <= ST_EXECUTE;
                elsif imm16(7 downto 0) = x"31" or imm16(7 downto 0) = x"39" then
                  op_kind <= OP_INS_FIELD;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"33" or imm16(7 downto 0) = x"3B" then
                  op_kind <= OP_EXT_FIELD;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"95" then
                  op_kind <= OP_MOVSPB;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"94" then
                  op_kind <= OP_TSKSW;
                  state <= ST_MODRM_REQ;
                elsif imm16(7 downto 0) = x"92" then
                  tmp_nat := SFR_ISPR_INDEX;
                  r8 := internal_data(tmp_nat);
                  for ispr_bit in 0 to 7 loop
                    if r8(ispr_bit) = '1' then
                      bit_mask8 := shift_left(to_unsigned(1, 8), ispr_bit);
                      r8 := std_logic_vector(unsigned(r8) and not bit_mask8);
                      exit;
                    end if;
                  end loop;
                  internal_data(tmp_nat) <= r8;
                  state <= ST_FETCH_REQ;
                elsif imm16(7 downto 0) = x"91" then
                  op_kind <= OP_RETRBI;
                  state <= ST_EXECUTE;
                elsif imm16(7 downto 0) = x"9C" then
                  op_kind <= OP_BTCLR;
                  imm_needed <= 3;
                  imm_index <= 0;
                  imm16 <= x"0000";
                  far_seg <= x"0000";
                  state <= ST_IMM_REQ;
                elsif imm16(7 downto 0) = x"9E" then
                  halt_stop_mode <= '1';
                  state <= ST_HALTED;
                else
                  fault_r <= '1';
                  state <= ST_FAULT;
                end if;

              when OP_ADD4S | OP_SUB4S | OP_CMP4S =>
                tmp_nat := to_integer(unsigned(v25_get_reg8(regs(active_bank), 1)));
                if tmp_nat = 0 or tmp_nat > 254 then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  if (tmp_nat mod 2) = 1 then
                    tmp_nat := tmp_nat + 1;
                  end if;
                  bcd_total <= tmp_nat / 2;
                  bcd_index <= 0;
                  bcd_carry <= '0';
                  bcd_zero  <= '1';
                  seg_value := v25_selected_seg(seg_override, ds0, ps, ss, ds0, ds1);
                  ea_addr := v25_phys_addr(seg_value, regs(active_bank)(V25_REG_SI));
                  bcd_src_addr <= ea_addr;
                  bcd_dst_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                  state <= ST_BCD_SRC_RD_REQ;
                end if;

              when OP_ROL4 | OP_ROR4 =>
                if modrm(5 downto 3) /= "000" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                elsif v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  begin_mem_read_low(ea_addr, '0');
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  a8 := v25_get_reg8(regs(active_bank), rm_idx);
                  b8 := v25_get_reg8(regs(active_bank), 0);
                  if op_kind = OP_ROL4 then
                    r8 := a8(3 downto 0) & b8(3 downto 0);
                    b8 := b8(7 downto 4) & a8(7 downto 4);
                  else
                    r8 := b8(3 downto 0) & a8(7 downto 4);
                    b8 := b8(7 downto 4) & a8(3 downto 0);
                  end if;
                  reg_bank := v25_set_reg8(regs(active_bank), rm_idx, r8);
                  reg_bank := v25_set_reg8(reg_bank, 0, b8);
                  regs(active_bank) <= reg_bank;
                  state <= ST_FETCH_REQ;
                end if;

              when OP_MOVSPA =>
                ss <= bank_ss(previous_bank);
                bank_ss(active_bank) <= bank_ss(previous_bank);
                regs(active_bank)(V25_REG_SP) <= regs(previous_bank)(V25_REG_SP);
                state <= ST_FETCH_REQ;

              when OP_MOVSPB =>
                if modrm(7 downto 3) /= "11111" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  tmp_nat := to_integer(unsigned(regs(active_bank)(rm_idx)(2 downto 0)));
                  bank_ss(tmp_nat) <= ss;
                  regs(tmp_nat)(V25_REG_SP) <= regs(active_bank)(V25_REG_SP);
                  state <= ST_FETCH_REQ;
                end if;

              when OP_TSKSW =>
                if modrm(7 downto 3) /= "11111" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  tmp_nat := to_integer(unsigned(regs(active_bank)(rm_idx)(2 downto 0)));
                  bank_saved_ip(active_bank) <= ip;
                  bank_saved_psw(active_bank) <= v25_pack_psw(flags);
                  bank_ps(active_bank) <= ps;
                  bank_ss(active_bank) <= ss;
                  bank_ds0(active_bank) <= ds0;
                  bank_ds1(active_bank) <= ds1;
                  f := v25_unpack_psw(bank_saved_psw(tmp_nat));
                  f.rb := std_logic_vector(to_unsigned(tmp_nat, 3));
                  flags <= f;
                  previous_bank <= active_bank;
                  active_bank <= tmp_nat;
                  ip <= bank_saved_ip(tmp_nat);
                  ps <= bank_ps(tmp_nat);
                  ss <= bank_ss(tmp_nat);
                  ds0 <= bank_ds0(tmp_nat);
                  ds1 <= bank_ds1(tmp_nat);
                  state <= ST_FETCH_REQ;
                end if;

              when OP_BRKCS =>
                if modrm(7 downto 3) /= "11000" then
                  fault_r <= '1';
                  state <= ST_FAULT;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  tmp_nat := to_integer(unsigned(regs(active_bank)(rm_idx)(2 downto 0)));
                  bank_saved_ip(tmp_nat) <= ip;
                  bank_saved_psw(tmp_nat) <= v25_pack_psw(flags);
                  bank_ps(active_bank) <= ps;
                  bank_ss(active_bank) <= ss;
                  bank_ds0(active_bank) <= ds0;
                  bank_ds1(active_bank) <= ds1;
                  f := flags;
                  f.rb := std_logic_vector(to_unsigned(tmp_nat, 3));
                  f.iflag := '0';
                  f.ibrk := '0';
                  flags <= f;
                  previous_bank <= active_bank;
                  active_bank <= tmp_nat;
                  ip <= bank_vector_ip(tmp_nat);
                  ps <= bank_ps(tmp_nat);
                  ss <= bank_ss(tmp_nat);
                  ds0 <= bank_ds0(tmp_nat);
                  ds1 <= bank_ds1(tmp_nat);
                  state <= ST_FETCH_REQ;
                end if;

              when OP_RETRBI =>
                f := v25_unpack_psw(bank_saved_psw(active_bank));
                flags <= f;
                bank_ps(active_bank) <= ps;
                bank_ss(active_bank) <= ss;
                bank_ds0(active_bank) <= ds0;
                bank_ds1(active_bank) <= ds1;
                previous_bank <= active_bank;
                tmp_nat := to_integer(unsigned(f.rb));
                active_bank <= tmp_nat;
                ip <= bank_saved_ip(active_bank);
                ps <= bank_ps(tmp_nat);
                ss <= bank_ss(tmp_nat);
                ds0 <= bank_ds0(tmp_nat);
                ds1 <= bank_ds1(tmp_nat);
                state <= ST_FETCH_REQ;

              when OP_BTCLR =>
                tmp_nat := v25_sfr_index(imm16(7 downto 0));
                bit_index := to_integer(unsigned(imm16(10 downto 8)));
                bit_mask8 := shift_left(to_unsigned(1, 8), bit_index);
                bit_was_set := (unsigned(internal_data(tmp_nat)) and bit_mask8) /= to_unsigned(0, 8);
                if bit_was_set then
                  r8 := std_logic_vector(unsigned(internal_data(tmp_nat)) and not bit_mask8);
                  write_internal_byte(
                    internal_data,
                    flags,
                    idb_high,
                    bank_vector_ip,
                    bank_saved_ip,
                    bank_saved_psw,
                    bank_ps,
                    bank_ss,
                    bank_ds0,
                    bank_ds1,
                    regs,
                    rfm_rflv_slave,
                    ps,
                    ss,
                    ds0,
                    ds1,
                    active_bank,
                    tmp_nat,
                    r8
                  );
                  ip <= std_logic_vector(unsigned(ip) + unsigned(v25_sign_extend8(far_seg(7 downto 0))));
                end if;
                if ENABLE_TIMING_THROTTLE then
                  timing_counter <= v25_clocks_btclr(bit_was_set);
                end if;
                state <= ST_FETCH_REQ;

              when OP_V25_BITOP =>
                if v25_modrm_is_memory(modrm) then
                  ea_addr := v25_phys_addr(
                    v25_selected_seg(
                      seg_override,
                      v25_modrm_default_seg(ss, ds0, modrm),
                      ps,
                      ss,
                      ds0,
                      ds1
                    ),
                    v25_modrm_ea(regs(active_bank), modrm, disp16)
                  );
                  mem_op_addr <= ea_addr;
                  mem_is_word <= v25_subop(0);
                  state <= ST_MEM_RD_LO_REQ;
                else
                  rm_idx := to_integer(unsigned(modrm(2 downto 0)));
                  f := flags;

                  if v25_subop(0) = '0' then
                    a8 := v25_get_reg8(regs(active_bank), rm_idx);
                    if v25_subop(3) = '1' then
                      bit_index := to_integer(unsigned(imm16(2 downto 0)));
                    else
                      bit_index := to_integer(unsigned(v25_get_reg8(regs(active_bank), 1)(2 downto 0)));
                    end if;
                    bit_mask8 := shift_left(to_unsigned(1, 8), bit_index);

                    case v25_subop(2 downto 1) is
                      when "00" =>
                        if (unsigned(a8) and bit_mask8) /= to_unsigned(0, 8) then
                          f.zf := '0';
                        else
                          f.zf := '1';
                        end if;
                        f.cf := '0';
                        f.oflag := '0';
                        flags <= f;
                      when "01" =>
                        r8 := std_logic_vector(unsigned(a8) and not bit_mask8);
                        regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                      when "10" =>
                        r8 := std_logic_vector(unsigned(a8) or bit_mask8);
                        regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                      when others =>
                        r8 := std_logic_vector(unsigned(a8) xor bit_mask8);
                        regs(active_bank) <= v25_set_reg8(regs(active_bank), rm_idx, r8);
                    end case;
                  else
                    a16 := regs(active_bank)(rm_idx);
                    if v25_subop(3) = '1' then
                      bit_index := to_integer(unsigned(imm16(3 downto 0)));
                    else
                      bit_index := to_integer(unsigned(v25_get_reg8(regs(active_bank), 1)(3 downto 0)));
                    end if;
                    bit_mask16 := shift_left(to_unsigned(1, 16), bit_index);

                    case v25_subop(2 downto 1) is
                      when "00" =>
                        if (unsigned(a16) and bit_mask16) /= to_unsigned(0, 16) then
                          f.zf := '0';
                        else
                          f.zf := '1';
                        end if;
                        f.cf := '0';
                        f.oflag := '0';
                        flags <= f;
                      when "01" =>
                        regs(active_bank)(rm_idx) <= std_logic_vector(unsigned(a16) and not bit_mask16);
                      when "10" =>
                        regs(active_bank)(rm_idx) <= std_logic_vector(unsigned(a16) or bit_mask16);
                      when others =>
                        regs(active_bank)(rm_idx) <= std_logic_vector(unsigned(a16) xor bit_mask16);
                    end case;
                  end if;

                  state <= ST_FETCH_REQ;
                end if;

              when others =>
                fault_r <= '1';
                state <= ST_FAULT;
              end case;
            end if;

          when ST_MEM_RD_LO_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= mem_op_addr;
            end if;
            state <= ST_MEM_RD_LO_WAIT;

          when ST_MEM_RD_LO_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                tmp_nat := v25_internal_data_index(mem_op_addr);
                mem_byte := read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
tmp_nat
                );
                if tmp_nat = SFR_RXB0_INDEX then
                  serial0_rx_unread <= '0';
                elsif tmp_nat = SFR_RXB1_INDEX then
                  serial1_rx_unread <= '0';
                end if;
              else
                mem_byte := mem_rdata;
              end if;
              mem_low <= mem_byte;

              if mem_is_word = '1' then
                state <= ST_MEM_RD_HI_REQ;
              else
                if op_kind = OP_MOV_R8_RM8 then
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), mem_target, mem_byte);
                  state <= ST_FETCH_REQ;
                elsif op_kind = OP_MOV_AL_MOFFS then
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, mem_byte);
                  state <= ST_FETCH_REQ;
                elsif op_kind = OP_XLAT then
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, mem_byte);
                  state <= ST_FETCH_REQ;
                elsif op_kind = OP_MOVS8 then
                  mem_value <= x"00" & mem_byte;
                  mem_is_word <= '0';
                  mem_op_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                  state <= ST_MEM_WR_LO_REQ;
                elsif op_kind = OP_CMPS8 then
                  mem_value <= x"00" & mem_byte;
                  mem_is_word <= '0';
                  mem_op_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                  op_kind <= OP_CMPS8_DST;
                  state <= ST_MEM_RD_LO_REQ;
                elsif op_kind = OP_CMPS8_DST then
                  f := v25_sub_flags8(flags, mem_value(7 downto 0), mem_byte, '0');
                  flags <= f;
                  reg_bank := regs(active_bank);
                  if flags.df = '1' then
                    reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 1);
                    reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) - 1);
                  else
                    reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 1);
                    reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 1);
                  end if;
                  if rep_mode /= REP_NONE then
                    cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                    reg_bank(V25_REG_CX) := cx_next;
                    if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                       repeat_uses_conditional_stop(op_kind) then
                      select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                      timing_counter <= timing_saturating_add(
                        timing_counter,
                        repeat_iteration_timing_budget(
                          op_kind,
                          timing_wait_v,
                          timing_internal_v
                        )
                      );
                    end if;
                    if repeat_continues(rep_mode, op_kind, f, cx_next) then
                      op_kind <= repeat_start_kind(op_kind);
                      state <= ST_EXECUTE;
                    else
                      state <= ST_FETCH_REQ;
                    end if;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                  regs(active_bank) <= reg_bank;
                elsif op_kind = OP_LODS8 then
                  reg_bank := v25_set_reg8(regs(active_bank), 0, mem_byte);
                  if flags.df = '1' then
                    reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 1);
                  else
                    reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 1);
                  end if;
                  if rep_mode /= REP_NONE then
                    cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                    reg_bank(V25_REG_CX) := cx_next;
                    if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' then
                      select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                      timing_counter <= timing_saturating_add(
                        timing_counter,
                        repeat_iteration_timing_budget(
                          op_kind,
                          timing_wait_v,
                          timing_internal_v
                        )
                      );
                    end if;
                    if repeat_continues(rep_mode, op_kind, flags, cx_next) then
                      state <= ST_EXECUTE;
                    else
                      state <= ST_FETCH_REQ;
                    end if;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                  regs(active_bank) <= reg_bank;
                elsif op_kind = OP_SCAS8 then
                  a8 := v25_get_reg8(regs(active_bank), 0);
                  f := v25_sub_flags8(flags, a8, mem_byte, '0');
                  flags <= f;
                  reg_bank := regs(active_bank);
                  if flags.df = '1' then
                    reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) - 1);
                  else
                    reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 1);
                  end if;
                  if rep_mode /= REP_NONE then
                    cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                    reg_bank(V25_REG_CX) := cx_next;
                    if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                       repeat_uses_conditional_stop(op_kind) then
                      select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                      timing_counter <= timing_saturating_add(
                        timing_counter,
                        repeat_iteration_timing_budget(
                          op_kind,
                          timing_wait_v,
                          timing_internal_v
                        )
                      );
                    end if;
                    if repeat_continues(rep_mode, op_kind, f, cx_next) then
                      state <= ST_EXECUTE;
                    else
                      state <= ST_FETCH_REQ;
                    end if;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                  regs(active_bank) <= reg_bank;
                elsif op_kind = OP_XCHG_RM8_R8 then
                  a8 := v25_get_reg8(regs(active_bank), mem_target);
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), mem_target, mem_byte);
                  mem_value <= x"00" & a8;
                  mem_is_word <= '0';
                  state <= ST_MEM_WR_LO_REQ;
                elsif op_kind = OP_ALU_RM8_R8 then
                  a8 := mem_byte;
                  b8 := v25_get_reg8(regs(active_bank), mem_target);
                  r8 := alu_result8(alu_func, a8, b8, flags.cf);
                  f := alu_flags8(alu_func, flags, a8, b8);
                  flags <= f;
                  if alu_func = ALU_CMP then
                    state <= ST_FETCH_REQ;
                  else
                    mem_value <= x"00" & r8;
                    mem_is_word <= '0';
                    state <= ST_MEM_WR_LO_REQ;
                  end if;
                elsif op_kind = OP_ALU_R8_RM8 then
                  a8 := v25_get_reg8(regs(active_bank), mem_target);
                  b8 := mem_byte;
                  r8 := alu_result8(alu_func, a8, b8, flags.cf);
                  f := alu_flags8(alu_func, flags, a8, b8);
                  flags <= f;
                  if alu_func /= ALU_CMP then
                    regs(active_bank) <= v25_set_reg8(regs(active_bank), mem_target, r8);
                  end if;
                  state <= ST_FETCH_REQ;
                elsif op_kind = OP_TEST_RM8_R8 then
                  a8 := mem_byte;
                  b8 := v25_get_reg8(regs(active_bank), mem_target);
                  flags <= v25_logic_flags8(flags, a8 and b8);
                  state <= ST_FETCH_REQ;
                elsif op_kind = OP_GRP_IMM8_RM8 then
                  group_bits := modrm(5 downto 3);
                  a8 := mem_byte;
                  b8 := imm16(7 downto 0);
                  f := alu_flags8(group_to_alu(group_bits), flags, a8, b8);
                  flags <= f;

                  if group_to_alu(group_bits) = ALU_CMP then
                    state <= ST_FETCH_REQ;
                  else
                    r8 := alu_result8(group_to_alu(group_bits), a8, b8, flags.cf);
                    mem_value <= x"00" & r8;
                    mem_is_word <= '0';
                    state <= ST_MEM_WR_LO_REQ;
                  end if;
                elsif op_kind = OP_GRP3_RM8 then
                  group_bits := modrm(5 downto 3);
                  a8 := mem_byte;

                  case group_bits is
                    when "000" =>
                      flags <= v25_logic_flags8(flags, a8 and imm16(7 downto 0));
                      state <= ST_FETCH_REQ;
                    when "010" =>
                      r8 := not a8;
                      mem_value <= x"00" & r8;
                      mem_is_word <= '0';
                      state <= ST_MEM_WR_LO_REQ;
                    when "011" =>
                      r8 := std_logic_vector(to_unsigned(0, 8) - unsigned(a8));
                      f := v25_sub_flags8(flags, x"00", a8, '0');
                      mem_value <= x"00" & r8;
                      mem_is_word <= '0';
                      flags <= f;
                      state <= ST_MEM_WR_LO_REQ;
                    when "100" =>
                      prod16u := unsigned(v25_get_reg8(regs(active_bank), 0)) * unsigned(a8);
                      regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod16u);
                      if prod16u(15 downto 8) = x"00" then
                        flags.cf <= '0';
                        flags.oflag <= '0';
                      else
                        flags.cf <= '1';
                        flags.oflag <= '1';
                      end if;
                      state <= ST_FETCH_REQ;
                    when "101" =>
                      prod16s := signed(v25_get_reg8(regs(active_bank), 0)) * signed(a8);
                      regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod16s);
                      if prod16s >= to_signed(-128, 16) and prod16s <= to_signed(127, 16) then
                        flags.cf <= '0';
                        flags.oflag <= '0';
                      else
                        flags.cf <= '1';
                        flags.oflag <= '1';
                      end if;
                      state <= ST_FETCH_REQ;
                    when "110" =>
                      if a8 = x"00" then
                        int_vector_base <= interrupt_vector_base(x"00");
                        int_ibrk_after <= '0';
                        int_return_ip <= ip;
                        mem_valid_r <= '1';
                        mem_write_r <= '0';
                        mem_addr_r  <= interrupt_vector_base(x"00");
                        state <= ST_INT_VEC_IP_LO_REQ;
                      else
                        dividend16u := unsigned(regs(active_bank)(V25_REG_AX));
                        quotient16u := dividend16u / resize(unsigned(a8), 16);
                        remainder16u := dividend16u rem resize(unsigned(a8), 16);
                        if quotient16u(15 downto 8) /= x"00" then
                          int_vector_base <= interrupt_vector_base(x"00");
                          int_ibrk_after <= '0';
                          int_return_ip <= ip;
                          mem_valid_r <= '1';
                          mem_write_r <= '0';
                          mem_addr_r  <= interrupt_vector_base(x"00");
                          state <= ST_INT_VEC_IP_LO_REQ;
                        else
                          reg_bank := v25_set_reg8(regs(active_bank), 0, std_logic_vector(quotient16u(7 downto 0)));
                          reg_bank := v25_set_reg8(reg_bank, 4, std_logic_vector(remainder16u(7 downto 0)));
                          regs(active_bank) <= reg_bank;
                          state <= ST_FETCH_REQ;
                        end if;
                      end if;
                    when "111" =>
                      if a8 = x"00" then
                        int_vector_base <= interrupt_vector_base(x"00");
                        int_ibrk_after <= '0';
                        int_return_ip <= ip;
                        mem_valid_r <= '1';
                        mem_write_r <= '0';
                        mem_addr_r  <= interrupt_vector_base(x"00");
                        state <= ST_INT_VEC_IP_LO_REQ;
                      else
                        dividend16s := signed(regs(active_bank)(V25_REG_AX));
                        quotient16s := dividend16s / resize(signed(a8), 16);
                        remainder16s := dividend16s rem resize(signed(a8), 16);
                        if quotient16s < to_signed(-128, 16) or quotient16s > to_signed(127, 16) then
                          int_vector_base <= interrupt_vector_base(x"00");
                          int_ibrk_after <= '0';
                          int_return_ip <= ip;
                          mem_valid_r <= '1';
                          mem_write_r <= '0';
                          mem_addr_r  <= interrupt_vector_base(x"00");
                          state <= ST_INT_VEC_IP_LO_REQ;
                        else
                          reg_bank := v25_set_reg8(regs(active_bank), 0, std_logic_vector(quotient16s(7 downto 0)));
                          reg_bank := v25_set_reg8(reg_bank, 4, std_logic_vector(remainder16s(7 downto 0)));
                          regs(active_bank) <= reg_bank;
                          state <= ST_FETCH_REQ;
                        end if;
                      end if;
                    when others =>
                      fault_r <= '1';
                      state <= ST_FAULT;
                  end case;
                elsif op_kind = OP_INC_RM8 or op_kind = OP_DEC_RM8 then
                  a8 := mem_byte;
                  if op_kind = OP_INC_RM8 then
                    r8 := std_logic_vector(unsigned(a8) + 1);
                    f := v25_add_flags8(flags, a8, x"01", '0');
                  else
                    r8 := std_logic_vector(unsigned(a8) - 1);
                    f := v25_sub_flags8(flags, a8, x"01", '0');
                  end if;

                  f.cf := flags.cf;
                  mem_value <= x"00" & r8;
                  mem_is_word <= '0';
                  flags <= f;
                  state <= ST_MEM_WR_LO_REQ;
                elsif op_kind = OP_GRP_SHIFT_RM8_1 or
                      op_kind = OP_GRP_SHIFT_RM8_CL or
                      op_kind = OP_GRP_SHIFT_RM8_IMM then
                  group_bits := modrm(5 downto 3);
                  shift_count := shift_count_for_op(op_kind, regs(active_bank), imm16);
                  if shift_count = 0 then
                    state <= ST_FETCH_REQ;
                  else
                    shift8 := shift_rotate8(group_bits, mem_byte, shift_count, flags);
                    mem_value <= x"00" & shift8.value;
                    mem_is_word <= '0';
                    flags <= shift8.flags;
                    state <= ST_MEM_WR_LO_REQ;
                  end if;
                elsif op_kind = OP_ROL4 or op_kind = OP_ROR4 then
                  a8 := mem_byte;
                  b8 := v25_get_reg8(regs(active_bank), 0);
                  if op_kind = OP_ROL4 then
                    r8 := a8(3 downto 0) & b8(3 downto 0);
                    b8 := b8(7 downto 4) & a8(7 downto 4);
                  else
                    r8 := b8(3 downto 0) & a8(7 downto 4);
                    b8 := b8(7 downto 4) & a8(3 downto 0);
                  end if;
                  regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, b8);
                  mem_value <= x"00" & r8;
                  mem_is_word <= '0';
                  state <= ST_MEM_WR_LO_REQ;
                elsif op_kind = OP_V25_BITOP then
                  f := flags;
                  a8 := mem_byte;

                  if v25_subop(3) = '1' then
                    bit_index := to_integer(unsigned(imm16(2 downto 0)));
                  else
                    bit_index := to_integer(unsigned(v25_get_reg8(regs(active_bank), 1)(2 downto 0)));
                  end if;
                  bit_mask8 := shift_left(to_unsigned(1, 8), bit_index);

                  case v25_subop(2 downto 1) is
                    when "00" =>
                      if (unsigned(a8) and bit_mask8) /= to_unsigned(0, 8) then
                        f.zf := '0';
                      else
                        f.zf := '1';
                      end if;
                      f.cf := '0';
                      f.oflag := '0';
                      flags <= f;
                      state <= ST_FETCH_REQ;
                    when "01" =>
                      r8 := std_logic_vector(unsigned(a8) and not bit_mask8);
                      mem_value <= x"00" & r8;
                      mem_is_word <= '0';
                      state <= ST_MEM_WR_LO_REQ;
                    when "10" =>
                      r8 := std_logic_vector(unsigned(a8) or bit_mask8);
                      mem_value <= x"00" & r8;
                      mem_is_word <= '0';
                      state <= ST_MEM_WR_LO_REQ;
                    when others =>
                      r8 := std_logic_vector(unsigned(a8) xor bit_mask8);
                      mem_value <= x"00" & r8;
                      mem_is_word <= '0';
                      state <= ST_MEM_WR_LO_REQ;
                  end case;
                elsif op_kind = OP_OUTM8 then
                  io_addr_r <= regs(active_bank)(V25_REG_DX);
                  io_wdata_r <= mem_byte;
                  io_valid_r <= '1';
                  io_write_r <= '1';
                  state <= ST_IO_WR_LO_WAIT;
                else
                  fault_r <= '1';
                  state <= ST_FAULT;
                end if;
              end if;
            end if;

          when ST_MEM_RD_HI_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= std_logic_vector(unsigned(mem_op_addr) + 1);
            end if;
            state <= ST_MEM_RD_HI_WAIT;

          when ST_MEM_RD_HI_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                mem_byte := read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
                  (v25_internal_data_index(mem_op_addr) + 1) mod 512
                );
              else
                mem_byte := mem_rdata;
              end if;
              r16 := mem_byte & mem_low;

              if op_kind = OP_MOV_R16_RM16 then
                regs(active_bank)(mem_target) <= r16;
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_MOV_AX_MOFFS then
                regs(active_bank)(V25_REG_AX) <= r16;
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_MOVS16 then
                mem_value <= r16;
                mem_is_word <= '1';
                mem_op_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                state <= ST_MEM_WR_LO_REQ;
              elsif op_kind = OP_CMPS16 then
                mem_value <= r16;
                mem_is_word <= '1';
                mem_op_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                op_kind <= OP_CMPS16_DST;
                state <= ST_MEM_RD_LO_REQ;
              elsif op_kind = OP_CMPS16_DST then
                f := v25_sub_flags16(flags, mem_value, r16, '0');
                flags <= f;
                reg_bank := regs(active_bank);
                if flags.df = '1' then
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 2);
                  reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) - 2);
                else
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 2);
                  reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 2);
                end if;
                if rep_mode /= REP_NONE then
                  cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  reg_bank(V25_REG_CX) := cx_next;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                     repeat_uses_conditional_stop(op_kind) then
                    select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      repeat_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if repeat_continues(rep_mode, op_kind, f, cx_next) then
                    op_kind <= repeat_start_kind(op_kind);
                    state <= ST_EXECUTE;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  state <= ST_FETCH_REQ;
                end if;
                regs(active_bank) <= reg_bank;
              elsif op_kind = OP_LODS16 then
                reg_bank := regs(active_bank);
                reg_bank(V25_REG_AX) := r16;
                if flags.df = '1' then
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 2);
                else
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 2);
                end if;
                if rep_mode /= REP_NONE then
                  cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  reg_bank(V25_REG_CX) := cx_next;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' then
                    select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      repeat_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if repeat_continues(rep_mode, op_kind, flags, cx_next) then
                    state <= ST_EXECUTE;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  state <= ST_FETCH_REQ;
                end if;
                regs(active_bank) <= reg_bank;
              elsif op_kind = OP_SCAS16 then
                a16 := regs(active_bank)(V25_REG_AX);
                f := v25_sub_flags16(flags, a16, r16, '0');
                flags <= f;
                reg_bank := regs(active_bank);
                if flags.df = '1' then
                  reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) - 2);
                else
                  reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 2);
                end if;
                if rep_mode /= REP_NONE then
                  cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  reg_bank(V25_REG_CX) := cx_next;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                     repeat_uses_conditional_stop(op_kind) then
                    select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      repeat_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if repeat_continues(rep_mode, op_kind, f, cx_next) then
                    state <= ST_EXECUTE;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  state <= ST_FETCH_REQ;
                end if;
                regs(active_bank) <= reg_bank;
              elsif op_kind = OP_XCHG_RM16_R16 then
                a16 := regs(active_bank)(mem_target);
                regs(active_bank)(mem_target) <= r16;
                mem_value <= a16;
                mem_is_word <= '1';
                state <= ST_MEM_WR_LO_REQ;
              elsif op_kind = OP_ALU_RM16_R16 then
                a16 := r16;
                b16 := regs(active_bank)(mem_target);
                r16 := alu_result16(alu_func, a16, b16, flags.cf);
                f := alu_flags16(alu_func, flags, a16, b16);
                flags <= f;
                if alu_func = ALU_CMP then
                  state <= ST_FETCH_REQ;
                else
                  mem_value <= r16;
                  mem_is_word <= '1';
                  state <= ST_MEM_WR_LO_REQ;
                end if;
              elsif op_kind = OP_ALU_R16_RM16 then
                a16 := regs(active_bank)(mem_target);
                b16 := r16;
                r16 := alu_result16(alu_func, a16, b16, flags.cf);
                f := alu_flags16(alu_func, flags, a16, b16);
                flags <= f;
                if alu_func /= ALU_CMP then
                  regs(active_bank)(mem_target) <= r16;
                end if;
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_TEST_RM16_R16 then
                a16 := r16;
                b16 := regs(active_bank)(mem_target);
                flags <= v25_logic_flags16(flags, a16 and b16);
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_CALL_RM16 then
                branch_ip <= r16;
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= ip;
                push_mode <= PUSH_THEN_JUMP;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;
              elsif op_kind = OP_JMP_RM16 then
                ip <= r16;
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_PUSH_RM16 then
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= r16;
                push_mode <= PUSH_ONLY;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;
              elsif op_kind = OP_V25_BITOP then
                f := flags;
                a16 := r16;

                if v25_subop(3) = '1' then
                  bit_index := to_integer(unsigned(imm16(3 downto 0)));
                else
                  bit_index := to_integer(unsigned(v25_get_reg8(regs(active_bank), 1)(3 downto 0)));
                end if;
                bit_mask16 := shift_left(to_unsigned(1, 16), bit_index);

                case v25_subop(2 downto 1) is
                  when "00" =>
                    if (unsigned(a16) and bit_mask16) /= to_unsigned(0, 16) then
                      f.zf := '0';
                    else
                      f.zf := '1';
                    end if;
                    f.cf := '0';
                    f.oflag := '0';
                    flags <= f;
                    state <= ST_FETCH_REQ;
                  when "01" =>
                    mem_value <= std_logic_vector(unsigned(a16) and not bit_mask16);
                    mem_is_word <= '1';
                    state <= ST_MEM_WR_LO_REQ;
                  when "10" =>
                    mem_value <= std_logic_vector(unsigned(a16) or bit_mask16);
                    mem_is_word <= '1';
                    state <= ST_MEM_WR_LO_REQ;
                  when others =>
                    mem_value <= std_logic_vector(unsigned(a16) xor bit_mask16);
                    mem_is_word <= '1';
                    state <= ST_MEM_WR_LO_REQ;
                end case;
              elsif op_kind = OP_MOV_SREG_RM16 then
                case mem_sreg_target is
                  when SEG_SS =>
                    ss <= r16;
                    bank_ss(active_bank) <= r16;
                    state <= ST_FETCH_REQ;
                  when SEG_DS0 =>
                    ds0 <= r16;
                    bank_ds0(active_bank) <= r16;
                    state <= ST_FETCH_REQ;
                  when SEG_DS1 =>
                    ds1 <= r16;
                    bank_ds1(active_bank) <= r16;
                    state <= ST_FETCH_REQ;
                  when others =>
                    fault_r <= '1';
                    state <= ST_FAULT;
                end case;
              elsif op_kind = OP_MOV_DS0_R16_MEM32 or op_kind = OP_MOV_DS1_R16_MEM32 then
                regs(active_bank)(mem_target) <= r16;
                mem_op_addr <= std_logic_vector(unsigned(mem_op_addr) + 2);
                state <= ST_MEM_RD_FAR_SEG_LO_REQ;
              elsif op_kind = OP_CHKIND then
                mem_value <= r16;
                mem_op_addr <= std_logic_vector(unsigned(mem_op_addr) + 2);
                state <= ST_MEM_RD_FAR_SEG_LO_REQ;
              elsif op_kind = OP_CALL_M32 or op_kind = OP_JMP_M32 then
                branch_ip <= r16;
                mem_op_addr <= std_logic_vector(unsigned(mem_op_addr) + 2);
                state <= ST_MEM_RD_FAR_SEG_LO_REQ;
              elsif op_kind = OP_GRP_IMM16_RM16 or op_kind = OP_GRP_IMM8_RM16_SIGN then
                group_bits := modrm(5 downto 3);
                a16 := r16;
                if op_kind = OP_GRP_IMM8_RM16_SIGN then
                  b16 := v25_sign_extend8(imm16(7 downto 0));
                else
                  b16 := imm16;
                end if;

                f := alu_flags16(group_to_alu(group_bits), flags, a16, b16);
                flags <= f;

                if group_to_alu(group_bits) = ALU_CMP then
                  state <= ST_FETCH_REQ;
                else
                  r16 := alu_result16(group_to_alu(group_bits), a16, b16, flags.cf);
                  mem_value <= r16;
                  mem_is_word <= '1';
                  state <= ST_MEM_WR_LO_REQ;
                end if;
              elsif op_kind = OP_GRP3_RM16 then
                group_bits := modrm(5 downto 3);
                a16 := r16;

                case group_bits is
                  when "000" =>
                    flags <= v25_logic_flags16(flags, a16 and imm16);
                    state <= ST_FETCH_REQ;
                  when "010" =>
                    mem_value <= not a16;
                    mem_is_word <= '1';
                    state <= ST_MEM_WR_LO_REQ;
                  when "011" =>
                    r16 := std_logic_vector(to_unsigned(0, 16) - unsigned(a16));
                    f := v25_sub_flags16(flags, x"0000", a16, '0');
                    mem_value <= r16;
                    mem_is_word <= '1';
                    flags <= f;
                    state <= ST_MEM_WR_LO_REQ;
                  when "100" =>
                    prod32u := unsigned(regs(active_bank)(V25_REG_AX)) * unsigned(a16);
                    regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod32u(15 downto 0));
                    regs(active_bank)(V25_REG_DX) <= std_logic_vector(prod32u(31 downto 16));
                    if prod32u(31 downto 16) = x"0000" then
                      flags.cf <= '0';
                      flags.oflag <= '0';
                    else
                      flags.cf <= '1';
                      flags.oflag <= '1';
                    end if;
                    state <= ST_FETCH_REQ;
                  when "101" =>
                    prod32s := signed(regs(active_bank)(V25_REG_AX)) * signed(a16);
                    regs(active_bank)(V25_REG_AX) <= std_logic_vector(prod32s(15 downto 0));
                    regs(active_bank)(V25_REG_DX) <= std_logic_vector(prod32s(31 downto 16));
                    if prod32s >= to_signed(-32768, 32) and prod32s <= to_signed(32767, 32) then
                      flags.cf <= '0';
                      flags.oflag <= '0';
                    else
                      flags.cf <= '1';
                      flags.oflag <= '1';
                    end if;
                    state <= ST_FETCH_REQ;
                  when "110" =>
                    if a16 = x"0000" then
                      int_vector_base <= interrupt_vector_base(x"00");
                      int_ibrk_after <= '0';
                      int_return_ip <= ip;
                      mem_valid_r <= '1';
                      mem_write_r <= '0';
                      mem_addr_r  <= interrupt_vector_base(x"00");
                      state <= ST_INT_VEC_IP_LO_REQ;
                    else
                      dividend32u := unsigned(regs(active_bank)(V25_REG_DX)) & unsigned(regs(active_bank)(V25_REG_AX));
                      quotient32u := dividend32u / resize(unsigned(a16), 32);
                      remainder32u := dividend32u rem resize(unsigned(a16), 32);
                      if quotient32u(31 downto 16) /= x"0000" then
                        int_vector_base <= interrupt_vector_base(x"00");
                        int_ibrk_after <= '0';
                        int_return_ip <= ip;
                        mem_valid_r <= '1';
                        mem_write_r <= '0';
                        mem_addr_r  <= interrupt_vector_base(x"00");
                        state <= ST_INT_VEC_IP_LO_REQ;
                      else
                        regs(active_bank)(V25_REG_AX) <= std_logic_vector(quotient32u(15 downto 0));
                        regs(active_bank)(V25_REG_DX) <= std_logic_vector(remainder32u(15 downto 0));
                        state <= ST_FETCH_REQ;
                      end if;
                    end if;
                  when "111" =>
                    if a16 = x"0000" then
                      int_vector_base <= interrupt_vector_base(x"00");
                      int_ibrk_after <= '0';
                      int_return_ip <= ip;
                      mem_valid_r <= '1';
                      mem_write_r <= '0';
                      mem_addr_r  <= interrupt_vector_base(x"00");
                      state <= ST_INT_VEC_IP_LO_REQ;
                    else
                      dividend32s := signed(regs(active_bank)(V25_REG_DX)) & signed(regs(active_bank)(V25_REG_AX));
                      quotient32s := dividend32s / resize(signed(a16), 32);
                      remainder32s := dividend32s rem resize(signed(a16), 32);
                      if quotient32s < to_signed(-32768, 32) or quotient32s > to_signed(32767, 32) then
                        int_vector_base <= interrupt_vector_base(x"00");
                        int_ibrk_after <= '0';
                        int_return_ip <= ip;
                        mem_valid_r <= '1';
                        mem_write_r <= '0';
                        mem_addr_r  <= interrupt_vector_base(x"00");
                        state <= ST_INT_VEC_IP_LO_REQ;
                      else
                        regs(active_bank)(V25_REG_AX) <= std_logic_vector(quotient32s(15 downto 0));
                        regs(active_bank)(V25_REG_DX) <= std_logic_vector(remainder32s(15 downto 0));
                        state <= ST_FETCH_REQ;
                      end if;
                    end if;
                  when others =>
                    fault_r <= '1';
                    state <= ST_FAULT;
                end case;
              elsif op_kind = OP_IMUL_R16_RM16_IMM16 or op_kind = OP_IMUL_R16_RM16_IMM8 then
                if op_kind = OP_IMUL_R16_RM16_IMM8 then
                  b16 := v25_sign_extend8(imm16(7 downto 0));
                else
                  b16 := imm16;
                end if;

                prod32s := signed(r16) * signed(b16);
                regs(active_bank)(mem_target) <= std_logic_vector(prod32s(15 downto 0));
                if prod32s >= to_signed(-32768, 32) and prod32s <= to_signed(32767, 32) then
                  flags.cf <= '0';
                  flags.oflag <= '0';
                else
                  flags.cf <= '1';
                  flags.oflag <= '1';
                end if;
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_INC_RM16 or op_kind = OP_DEC_RM16 then
                a16 := r16;
                if op_kind = OP_INC_RM16 then
                  r16 := std_logic_vector(unsigned(a16) + 1);
                  f := v25_add_flags16(flags, a16, x"0001", '0');
                else
                  r16 := std_logic_vector(unsigned(a16) - 1);
                  f := v25_sub_flags16(flags, a16, x"0001", '0');
                end if;

                f.cf := flags.cf;
                mem_value <= r16;
                mem_is_word <= '1';
                flags <= f;
                state <= ST_MEM_WR_LO_REQ;
              elsif op_kind = OP_GRP_SHIFT_RM16_1 or
                    op_kind = OP_GRP_SHIFT_RM16_CL or
                    op_kind = OP_GRP_SHIFT_RM16_IMM then
                group_bits := modrm(5 downto 3);
                shift_count := shift_count_for_op(op_kind, regs(active_bank), imm16);
                if shift_count = 0 then
                  state <= ST_FETCH_REQ;
                else
                  shift16 := shift_rotate16(group_bits, r16, shift_count, flags);
                  mem_value <= shift16.value;
                  mem_is_word <= '1';
                  flags <= shift16.flags;
                  state <= ST_MEM_WR_LO_REQ;
                end if;
              elsif op_kind = OP_OUTM16 then
                mem_value <= r16;
                io_addr_r <= regs(active_bank)(V25_REG_DX);
                io_wdata_r <= r16(7 downto 0);
                io_valid_r <= '1';
                io_write_r <= '1';
                state <= ST_IO_WR_LO_WAIT;
              else
                fault_r <= '1';
                state <= ST_FAULT;
              end if;
            end if;

          when ST_MEM_RD_FAR_SEG_LO_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= mem_op_addr;
            end if;
            state <= ST_MEM_RD_FAR_SEG_LO_WAIT;

          when ST_MEM_RD_FAR_SEG_LO_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                mem_low <= read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
                  v25_internal_data_index(mem_op_addr)
                );
              else
                mem_low <= mem_rdata;
              end if;
              state <= ST_MEM_RD_FAR_SEG_HI_REQ;
            end if;

          when ST_MEM_RD_FAR_SEG_HI_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= std_logic_vector(unsigned(mem_op_addr) + 1);
            end if;
            state <= ST_MEM_RD_FAR_SEG_HI_WAIT;

          when ST_MEM_RD_FAR_SEG_HI_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                r16 := read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
                  (v25_internal_data_index(mem_op_addr) + 1) mod 512
                ) & mem_low;
              else
                r16 := mem_rdata & mem_low;
              end if;

              if op_kind = OP_CHKIND then
                a16 := regs(active_bank)(mem_target);
                if unsigned(mem_value) > unsigned(a16) or unsigned(r16) < unsigned(a16) then
                  int_vector_base <= interrupt_vector_base(x"05");
                  int_ibrk_after <= '0';
                  int_return_ip <= ip;
                  mem_valid_r <= '1';
                  mem_write_r <= '0';
                  mem_addr_r  <= interrupt_vector_base(x"05");
                  state <= ST_INT_VEC_IP_LO_REQ;
                else
                  state <= ST_FETCH_REQ;
                end if;
              elsif op_kind = OP_CALL_M32 then
                far_seg <= r16;
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 4);
                push_value <= ip;
                push_mode <= PUSH_FAR_THEN_JUMP;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;
              elsif op_kind = OP_JMP_M32 then
                ip <= branch_ip;
                ps <= r16;
                bank_ps(active_bank) <= r16;
                state <= ST_FETCH_REQ;
              else
                case mem_sreg_target is
                  when SEG_DS0 =>
                    ds0 <= r16;
                    bank_ds0(active_bank) <= r16;
                    state <= ST_FETCH_REQ;
                  when SEG_DS1 =>
                    ds1 <= r16;
                    bank_ds1(active_bank) <= r16;
                    state <= ST_FETCH_REQ;
                  when others =>
                    fault_r <= '1';
                    state <= ST_FAULT;
                end case;
              end if;
            end if;

          when ST_MEM_WR_LO_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= mem_op_addr;
              mem_wdata_r <= mem_value(7 downto 0);
            end if;
            state <= ST_MEM_WR_LO_WAIT;

          when ST_MEM_WR_LO_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                tmp_nat := v25_internal_data_index(mem_op_addr);
                write_internal_byte(
                  internal_data,
                  flags,
                  idb_high,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  regs,
                  rfm_rflv_slave,
                  ps,
                  ss,
                  ds0,
                  ds1,
                  active_bank,
                  tmp_nat,
                  mem_value(7 downto 0)
                );
              end if;
              if mem_is_word = '1' then
                state <= ST_MEM_WR_HI_REQ;
              else
                if op_kind = OP_MOVS8 or op_kind = OP_STOS8 or op_kind = OP_INM8 then
                  reg_bank := regs(active_bank);
                  if flags.df = '1' then
                    if op_kind = OP_MOVS8 then
                      reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 1);
                    end if;
                    reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) - 1);
                  else
                    if op_kind = OP_MOVS8 then
                      reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 1);
                    end if;
                    reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 1);
                  end if;
                  if rep_mode /= REP_NONE then
                    cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                    reg_bank(V25_REG_CX) := cx_next;
                    if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                       (op_kind = OP_MOVS8 or op_kind = OP_STOS8) then
                      select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                      timing_counter <= timing_saturating_add(
                        timing_counter,
                        repeat_iteration_timing_budget(
                          op_kind,
                          timing_wait_v,
                          timing_internal_v
                        )
                      );
                    end if;
                    if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                       op_kind = OP_INM8 then
                      select_primitive_io_timing(op_kind, timing_wait_v, timing_internal_v);
                      timing_counter <= timing_saturating_add(
                        timing_counter,
                        primitive_io_iteration_timing_budget(
                          op_kind,
                          timing_wait_v,
                          timing_internal_v
                        )
                      );
                    end if;
                    if repeat_continues(rep_mode, op_kind, flags, cx_next) then
                      state <= ST_EXECUTE;
                    else
                      state <= ST_FETCH_REQ;
                    end if;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                  regs(active_bank) <= reg_bank;
                else
                  state <= ST_FETCH_REQ;
                end if;
              end if;
            end if;

          when ST_MEM_WR_HI_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= std_logic_vector(unsigned(mem_op_addr) + 1);
              mem_wdata_r <= mem_value(15 downto 8);
            end if;
            state <= ST_MEM_WR_HI_WAIT;

          when ST_MEM_WR_HI_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                tmp_nat := (v25_internal_data_index(mem_op_addr) + 1) mod 512;
                write_internal_byte(
                  internal_data,
                  flags,
                  idb_high,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  regs,
                  rfm_rflv_slave,
                  ps,
                  ss,
                  ds0,
                  ds1,
                  active_bank,
                  tmp_nat,
                  mem_value(15 downto 8)
                );
              end if;
              if op_kind = OP_MOVS16 or op_kind = OP_STOS16 or op_kind = OP_INM16 then
                reg_bank := regs(active_bank);
                if flags.df = '1' then
                  if op_kind = OP_MOVS16 then
                    reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 2);
                  end if;
                  reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) - 2);
                else
                  if op_kind = OP_MOVS16 then
                    reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 2);
                  end if;
                  reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 2);
                end if;
                if rep_mode /= REP_NONE then
                  cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  reg_bank(V25_REG_CX) := cx_next;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                     (op_kind = OP_MOVS16 or op_kind = OP_STOS16) then
                    select_string_memory_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      repeat_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' and
                     op_kind = OP_INM16 then
                    select_primitive_io_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      primitive_io_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if repeat_continues(rep_mode, op_kind, flags, cx_next) then
                    state <= ST_EXECUTE;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  state <= ST_FETCH_REQ;
                end if;
                regs(active_bank) <= reg_bank;
              else
                state <= ST_FETCH_REQ;
              end if;
            end if;

          when ST_BCD_SRC_RD_REQ =>
            if v25_internal_data_selected(bcd_src_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= bcd_src_addr;
            end if;
            state <= ST_BCD_SRC_RD_WAIT;

          when ST_BCD_SRC_RD_WAIT =>
            if v25_internal_data_selected(bcd_src_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(bcd_src_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                bcd_src_byte <= read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
                  v25_internal_data_index(bcd_src_addr)
                );
              else
                bcd_src_byte <= mem_rdata;
              end if;
              state <= ST_BCD_DST_RD_REQ;
            end if;

          when ST_BCD_DST_RD_REQ =>
            if v25_internal_data_selected(bcd_dst_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= bcd_dst_addr;
            end if;
            state <= ST_BCD_DST_RD_WAIT;

          when ST_BCD_DST_RD_WAIT =>
            if v25_internal_data_selected(bcd_dst_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(bcd_dst_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                mem_byte := read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
                  v25_internal_data_index(bcd_dst_addr)
                );
              else
                mem_byte := mem_rdata;
              end if;

              if op_kind = OP_ADD4S then
                bcd_result := bcd_add_byte(mem_byte, bcd_src_byte, bcd_carry);
              else
                bcd_result := bcd_sub_byte(mem_byte, bcd_src_byte, bcd_carry);
              end if;

              bcd_carry <= bcd_result.carry;
              if bcd_result.value /= x"00" then
                bcd_zero <= '0';
              end if;

              if op_kind = OP_CMP4S then
                if bcd_index + 1 >= bcd_total then
                  f := flags;
                  f.cf := bcd_result.carry;
                  if bcd_zero = '1' and bcd_result.value = x"00" then
                    f.zf := '1';
                  else
                    f.zf := '0';
                  end if;
                  flags <= f;
                  state <= ST_FETCH_REQ;
                else
                  bcd_index <= bcd_index + 1;
                  bcd_src_addr <= std_logic_vector(unsigned(bcd_src_addr) + 1);
                  bcd_dst_addr <= std_logic_vector(unsigned(bcd_dst_addr) + 1);
                  state <= ST_BCD_SRC_RD_REQ;
                end if;
              else
                mem_wdata_r <= bcd_result.value;
                state <= ST_BCD_WR_REQ;
              end if;
            end if;

          when ST_BCD_WR_REQ =>
            if v25_internal_data_selected(bcd_dst_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= bcd_dst_addr;
            end if;
            state <= ST_BCD_WR_WAIT;

          when ST_BCD_WR_WAIT =>
            if v25_internal_data_selected(bcd_dst_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(bcd_dst_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_byte(
                  internal_data,
                  flags,
                  idb_high,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  regs,
                  rfm_rflv_slave,
                  ps,
                  ss,
                  ds0,
                  ds1,
                  active_bank,
                  v25_internal_data_index(bcd_dst_addr),
                  mem_wdata_r
                );
              end if;
              if bcd_index + 1 >= bcd_total then
                f := flags;
                f.cf := bcd_carry;
                f.zf := bcd_zero;
                flags <= f;
                state <= ST_FETCH_REQ;
              else
                bcd_index <= bcd_index + 1;
                bcd_src_addr <= std_logic_vector(unsigned(bcd_src_addr) + 1);
                bcd_dst_addr <= std_logic_vector(unsigned(bcd_dst_addr) + 1);
                state <= ST_BCD_SRC_RD_REQ;
              end if;
            end if;

          when ST_INT_VEC_IP_LO_REQ =>
            mem_valid_r <= '1';
            mem_write_r <= '0';
            mem_addr_r  <= int_vector_base;
            state <= ST_INT_VEC_IP_LO_WAIT;

          when ST_INT_VEC_IP_LO_WAIT =>
            if mem_ready = '1' then
              mem_valid_r <= '0';
              mem_low <= mem_rdata;
              state <= ST_INT_VEC_IP_HI_REQ;
            end if;

          when ST_INT_VEC_IP_HI_REQ =>
            mem_valid_r <= '1';
            mem_write_r <= '0';
            mem_addr_r  <= std_logic_vector(unsigned(int_vector_base) + 1);
            state <= ST_INT_VEC_IP_HI_WAIT;

          when ST_INT_VEC_IP_HI_WAIT =>
            if mem_ready = '1' then
              mem_valid_r <= '0';
              int_target_ip <= mem_rdata & mem_low;
              state <= ST_INT_VEC_PS_LO_REQ;
            end if;

          when ST_INT_VEC_PS_LO_REQ =>
            mem_valid_r <= '1';
            mem_write_r <= '0';
            mem_addr_r  <= std_logic_vector(unsigned(int_vector_base) + 2);
            state <= ST_INT_VEC_PS_LO_WAIT;

          when ST_INT_VEC_PS_LO_WAIT =>
            if mem_ready = '1' then
              mem_valid_r <= '0';
              mem_low <= mem_rdata;
              state <= ST_INT_VEC_PS_HI_REQ;
            end if;

          when ST_INT_VEC_PS_HI_REQ =>
            mem_valid_r <= '1';
            mem_write_r <= '0';
            mem_addr_r  <= std_logic_vector(unsigned(int_vector_base) + 3);
            state <= ST_INT_VEC_PS_HI_WAIT;

          when ST_INT_VEC_PS_HI_WAIT =>
            if mem_ready = '1' then
              mem_valid_r <= '0';
              int_target_ps <= mem_rdata & mem_low;
              r16 := v25_pack_psw(flags);
              sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
              push_value <= r16;
              push_mode <= PUSH_INTERRUPT_PSW;
              regs(active_bank)(V25_REG_SP) <= sp_next;
              f := flags;
              f.iflag := '0';
              f.ibrk := int_ibrk_after;
              flags <= f;
              state <= ST_PUSH_LO_REQ;
            end if;

          when ST_EXT_INT_ACK1_REQ =>
            int_ack_valid_r <= '1';
            int_ack_second_r <= '0';
            if int_ack_ready = '1' then
              int_ack_valid_r <= '0';
              int_ack_second_r <= '0';
              state <= ST_EXT_INT_ACK_GAP;
            end if;

          when ST_EXT_INT_ACK_GAP =>
            int_ack_valid_r <= '1';
            int_ack_second_r <= '1';
            state <= ST_EXT_INT_ACK2_REQ;

          when ST_EXT_INT_ACK2_REQ =>
            int_ack_valid_r <= '1';
            int_ack_second_r <= '1';
            if int_ack_ready = '1' then
              int_ack_valid_r <= '0';
              int_ack_second_r <= '0';
              if int_ack_vector_valid = '1' then
                int_vector_base <= interrupt_vector_base(int_ack_vector_data);
                mem_addr_r <= interrupt_vector_base(int_ack_vector_data);
              else
                int_vector_base <= interrupt_vector_base(external_irq_vector);
                mem_addr_r <= interrupt_vector_base(external_irq_vector);
              end if;
              mem_valid_r <= '1';
              mem_write_r <= '0';
              state <= ST_INT_VEC_IP_LO_REQ;
            end if;

          when ST_IO_RD_LO_REQ =>
            io_valid_r <= '1';
            io_write_r <= '0';
            state <= ST_IO_RD_LO_WAIT;

          when ST_IO_RD_LO_WAIT =>
            if io_ready = '1' then
              io_valid_r <= '0';
              if op_kind = OP_IN_AL_IMM8 or op_kind = OP_IN_AL_DX then
                regs(active_bank) <= v25_set_reg8(regs(active_bank), 0, io_rdata);
                state <= ST_FETCH_REQ;
              elsif op_kind = OP_INM8 then
                mem_value <= x"00" & io_rdata;
                mem_is_word <= '0';
                mem_op_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                state <= ST_MEM_WR_LO_REQ;
              else
                io_low <= io_rdata;
                io_addr_r <= std_logic_vector(unsigned(io_addr_r) + 1);
                state <= ST_IO_RD_HI_REQ;
              end if;
            end if;

          when ST_IO_RD_HI_REQ =>
            io_valid_r <= '1';
            io_write_r <= '0';
            state <= ST_IO_RD_HI_WAIT;

          when ST_IO_RD_HI_WAIT =>
            if io_ready = '1' then
              io_valid_r <= '0';
              if op_kind = OP_INM16 then
                mem_value <= io_rdata & io_low;
                mem_is_word <= '1';
                mem_op_addr <= v25_phys_addr(ds1, regs(active_bank)(V25_REG_DI));
                state <= ST_MEM_WR_LO_REQ;
              else
                regs(active_bank)(V25_REG_AX) <= io_rdata & io_low;
                state <= ST_FETCH_REQ;
              end if;
            end if;

          when ST_IO_WR_LO_REQ =>
            io_valid_r <= '1';
            io_write_r <= '1';
            state <= ST_IO_WR_LO_WAIT;

          when ST_IO_WR_LO_WAIT =>
            if io_ready = '1' then
              io_valid_r <= '0';
              if op_kind = OP_OUTM8 then
                reg_bank := regs(active_bank);
                if flags.df = '1' then
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 1);
                else
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 1);
                end if;
                if rep_mode /= REP_NONE then
                  cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  reg_bank(V25_REG_CX) := cx_next;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' then
                    select_primitive_io_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      primitive_io_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if repeat_continues(rep_mode, op_kind, flags, cx_next) then
                    state <= ST_EXECUTE;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  state <= ST_FETCH_REQ;
                end if;
                regs(active_bank) <= reg_bank;
              elsif op_kind = OP_OUT_IMM8_AL or op_kind = OP_OUT_DX_AL then
                state <= ST_FETCH_REQ;
              else
                io_addr_r <= std_logic_vector(unsigned(io_addr_r) + 1);
                if op_kind = OP_OUTM16 then
                  io_wdata_r <= mem_value(15 downto 8);
                else
                  io_wdata_r <= regs(active_bank)(V25_REG_AX)(15 downto 8);
                end if;
                state <= ST_IO_WR_HI_REQ;
              end if;
            end if;

          when ST_IO_WR_HI_REQ =>
            io_valid_r <= '1';
            io_write_r <= '1';
            state <= ST_IO_WR_HI_WAIT;

          when ST_IO_WR_HI_WAIT =>
            if io_ready = '1' then
              io_valid_r <= '0';
              if op_kind = OP_OUTM16 then
                reg_bank := regs(active_bank);
                if flags.df = '1' then
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) - 2);
                else
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 2);
                end if;
                if rep_mode /= REP_NONE then
                  cx_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_CX)) - 1);
                  reg_bank(V25_REG_CX) := cx_next;
                  if ENABLE_TIMING_THROTTLE and rep_timing_loaded = '1' then
                    select_primitive_io_timing(op_kind, timing_wait_v, timing_internal_v);
                    timing_counter <= timing_saturating_add(
                      timing_counter,
                      primitive_io_iteration_timing_budget(
                        op_kind,
                        timing_wait_v,
                        timing_internal_v
                      )
                    );
                  end if;
                  if repeat_continues(rep_mode, op_kind, flags, cx_next) then
                    state <= ST_EXECUTE;
                  else
                    state <= ST_FETCH_REQ;
                  end if;
                else
                  state <= ST_FETCH_REQ;
                end if;
                regs(active_bank) <= reg_bank;
              else
                state <= ST_FETCH_REQ;
              end if;
            end if;

          when ST_DMA_RD_REQ =>
            if dma_source_is_io = '1' then
              io_valid_r <= '1';
              io_write_r <= '0';
              io_addr_r <= x"000" & std_logic_vector(to_unsigned(dma_active_channel, 4));
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= dma_src_addr;
            end if;
            state <= ST_DMA_RD_WAIT;

          when ST_DMA_RD_WAIT =>
            if (dma_source_is_io = '1' and io_ready = '1') or
              (dma_source_is_io = '0' and mem_ready = '1') then
              io_valid_r <= '0';
              mem_valid_r <= '0';
              if dma_source_is_io = '1' then
                dma_data <= io_rdata;
              else
                dma_data <= mem_rdata;
              end if;
              state <= ST_DMA_WR_REQ;
            end if;

          when ST_DMA_WR_REQ =>
            if dma_dest_is_io = '1' then
              io_valid_r <= '1';
              io_write_r <= '1';
              io_addr_r <= x"000" & std_logic_vector(to_unsigned(dma_active_channel, 4));
              io_wdata_r <= dma_data;
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= dma_dst_addr;
              mem_wdata_r <= dma_data;
            end if;
            state <= ST_DMA_WR_WAIT;

          when ST_DMA_WR_WAIT =>
            if (dma_dest_is_io = '1' and io_ready = '1') or
              (dma_dest_is_io = '0' and mem_ready = '1') then
              io_valid_r <= '0';
              mem_valid_r <= '0';
              if dma_word_mode = '1' and dma_high_phase = '0' then
                dma_high_phase <= '1';
                dma_src_addr <= std_logic_vector(unsigned(dma_src_addr) + 1);
                dma_dst_addr <= std_logic_vector(unsigned(dma_dst_addr) + 1);
                state <= ST_DMA_RD_REQ;
              else
                dma_base_v := dma_channel_base(dma_active_channel);
                dma_dmac_v := dma_dmac_index(dma_active_channel);
                dma_dmam_v := dma_dmam_index(dma_active_channel);
                dma_dic_v := dma_dic_index(dma_active_channel);
                dma_mode_v := internal_data(dma_dmam_v)(7 downto 5);
                dma_step := 1;
                if dma_word_mode = '1' then
                  dma_step := 2;
                end if;

                dma_src_offset := timer_word(internal_data, dma_base_v);
                dma_dst_offset := timer_word(internal_data, dma_base_v + 2);
                dma_next_src_offset := dma_next_offset(
                  dma_src_offset,
                  internal_data(dma_dmac_v)(1 downto 0),
                  dma_step
                );
                dma_next_dst_offset := dma_next_offset(
                  dma_dst_offset,
                  internal_data(dma_dmac_v)(5 downto 4),
                  dma_step
                );
                set_timer_word(internal_data, dma_base_v, dma_next_src_offset);
                set_timer_word(internal_data, dma_base_v + 2, dma_next_dst_offset);

                dma_tc_v := timer_word(internal_data, dma_base_v + 6);
                if dma_tc_v = x"0000" then
                  dma_next_tc := x"0000";
                else
                  dma_next_tc := std_logic_vector(unsigned(dma_tc_v) - 1);
                end if;
                set_timer_word(internal_data, dma_base_v + 6, dma_next_tc);

                if dma_next_tc = x"0000" then
                  internal_data(dma_dmam_v)(3) <= '0';
                  internal_data(dma_dmam_v)(2) <= '0';
                  internal_data(dma_dic_v)(7) <= '1';
                  dma_high_phase <= '0';
                  state <= ST_FETCH_REQ;
                elsif dma_mode_v = "100" or
                  (dma_mode_demand(dma_mode_v) and
                   ((dma_active_channel = 0 and dmarq0_in = '1') or
                    (dma_active_channel = 1 and dmarq1_in = '1'))) then
                  dma_high_phase <= '0';
                  dma_src_addr <= dma_phys_addr(internal_data(dma_base_v + 5), dma_next_src_offset);
                  dma_dst_addr <= dma_phys_addr(internal_data(dma_base_v + 4), dma_next_dst_offset);
                  state <= ST_DMA_RD_REQ;
                else
                  internal_data(dma_dmam_v)(2) <= '0';
                  dma_high_phase <= '0';
                  state <= ST_FETCH_REQ;
                end if;
              end if;
            end if;

          when ST_MACRO_RD_REQ =>
            mem_valid_r <= '1';
            mem_write_r <= '0';
            mem_addr_r  <= macro_mem_addr;
            state <= ST_MACRO_RD_WAIT;

          when ST_MACRO_RD_WAIT =>
            if mem_ready = '1' then
              mem_valid_r <= '0';
              macro_data <= mem_rdata;
              write_internal_byte(
                internal_data,
                flags,
                idb_high,
                bank_vector_ip,
                bank_saved_ip,
                bank_saved_psw,
                bank_ps,
                bank_ss,
                  bank_ds0,
                  bank_ds1,
                  regs,
                  rfm_rflv_slave,
                  ps,
                  ss,
                ds0,
                ds1,
                active_bank,
                macro_sfr_index,
                mem_rdata
              );
              if macro_word_mode = '1' and macro_high_phase = '0' then
                macro_high_phase <= '1';
                macro_mem_addr <= std_logic_vector(unsigned(macro_mem_addr) + 1);
                macro_sfr_index <= macro_sfr_index + 1;
                state <= ST_MACRO_RD_REQ;
              else
                state <= ST_MACRO_FINISH;
              end if;
            end if;

          when ST_MACRO_WR_REQ =>
            mem_valid_r <= '1';
            mem_write_r <= '1';
            mem_addr_r  <= macro_mem_addr;
            mem_wdata_r <= macro_data;
            state <= ST_MACRO_WR_WAIT;

          when ST_MACRO_WR_WAIT =>
            if mem_ready = '1' then
              mem_valid_r <= '0';
              if macro_word_mode = '1' and macro_high_phase = '0' then
                macro_sfr_idx_v := macro_sfr_index + 1;
                macro_value_v := read_internal_byte(
                  internal_data,
                  flags,
                  regs,
                  bank_vector_ip,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  port0_in,
                  port1_in,
                  port2_in,
                  portt_in,
                  serial0_rxd_in,
                  serial1_rxd_in,
                  macro_sfr_idx_v
                );
                macro_high_phase <= '1';
                macro_mem_addr <= std_logic_vector(unsigned(macro_mem_addr) + 1);
                macro_sfr_index <= macro_sfr_idx_v;
                macro_data <= macro_value_v;
                state <= ST_MACRO_WR_REQ;
              else
                state <= ST_MACRO_FINISH;
              end if;
            end if;

          when ST_MACRO_FINISH =>
            macro_step_v := 1;
            if macro_word_mode = '1' then
              macro_step_v := 2;
            end if;
            macro_search_hit_v := macro_search_mode = '1' and
              macro_data = internal_data(macro_channel_base + 2);
            macro_msp_v := timer_word(internal_data, macro_channel_base + 4);
            macro_next_msp_v := std_logic_vector(unsigned(macro_msp_v) + to_unsigned(macro_step_v, 16));
            set_timer_word(internal_data, macro_channel_base + 4, macro_next_msp_v);

            macro_count_v := internal_data(macro_channel_base);
            if macro_count_v = x"00" then
              macro_next_count_v := x"FF";
            else
              macro_next_count_v := std_logic_vector(unsigned(macro_count_v) - to_unsigned(1, 8));
            end if;
            internal_data(macro_channel_base) <= macro_next_count_v;

            if macro_next_count_v = x"00" or macro_search_hit_v then
              internal_data(macro_irq_ctrl_index)(5) <= '0';
              internal_data(macro_irq_ctrl_index)(7) <= '1';
            else
              internal_data(macro_irq_ctrl_index)(7) <= '0';
            end if;
            macro_high_phase <= '0';
            macro_search_mode <= '0';
            state <= ST_FETCH_REQ;

          when ST_FIELD_RD0_REQ =>
            if v25_internal_data_selected(field_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= field_addr;
            end if;
            state <= ST_FIELD_RD0_WAIT;

          when ST_FIELD_RD0_WAIT =>
            if v25_internal_data_selected(field_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(field_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(field_addr, mem_byte);
                field_b0 <= mem_byte;
              else
                field_b0 <= mem_rdata;
              end if;
              state <= ST_FIELD_RD1_REQ;
            end if;

          when ST_FIELD_RD1_REQ =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_FIELD_RD1_WAIT;

          when ST_FIELD_RD1_WAIT =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                field_b1 <= mem_byte;
              else
                field_b1 <= mem_rdata;
              end if;
              state <= ST_FIELD_RD2_REQ;
            end if;

          when ST_FIELD_RD2_REQ =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 2);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_FIELD_RD2_WAIT;

          when ST_FIELD_RD2_WAIT =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 2);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                field_b2 <= mem_byte;
              else
                field_b2 <= mem_rdata;
              end if;
              state <= ST_FIELD_RD3_REQ;
            end if;

          when ST_FIELD_RD3_REQ =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 3);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_FIELD_RD3_WAIT;

          when ST_FIELD_RD3_WAIT =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 3);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                field_b3 <= mem_byte;
              else
                mem_byte := mem_rdata;
                field_b3 <= mem_rdata;
              end if;
              field_window := resize(unsigned(field_b0), 32) or
                shift_left(resize(unsigned(field_b1), 32), 8) or
                shift_left(resize(unsigned(field_b2), 32), 16) or
                shift_left(resize(unsigned(mem_byte), 32), 24);

              if op_kind = OP_EXT_FIELD then
                field_shifted := shift_right(field_window, field_offset);
                field_mask16 := (others => '0');
                for i in 0 to 15 loop
                  if i < field_length then
                    field_mask16(i) := '1';
                  end if;
                end loop;

                reg_bank := regs(active_bank);
                reg_bank(V25_REG_AX) := std_logic_vector(field_shifted(15 downto 0) and field_mask16);

                field_next_offset := field_offset + field_length;
                if field_next_offset > 15 then
                  field_next_offset := field_next_offset - 16;
                  reg_bank(V25_REG_SI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SI)) + 2);
                end if;
                reg_bank := v25_set_reg8(reg_bank, field_reg, std_logic_vector(to_unsigned(field_next_offset, 8)));
                regs(active_bank) <= reg_bank;
                state <= ST_FETCH_REQ;
              else
                field_mask32 := (others => '0');
                for i in 0 to 15 loop
                  if i < field_length then
                    field_bit_pos := field_offset + i;
                    field_mask32(field_bit_pos) := '1';
                  end if;
                end loop;

                field_insert := shift_left(resize(unsigned(regs(active_bank)(V25_REG_AX)), 32), field_offset) and field_mask32;
                field_window := (field_window and not field_mask32) or field_insert;
                field_b0 <= std_logic_vector(field_window(7 downto 0));
                field_b1 <= std_logic_vector(field_window(15 downto 8));
                field_b2 <= std_logic_vector(field_window(23 downto 16));
                field_b3 <= std_logic_vector(field_window(31 downto 24));
                state <= ST_FIELD_WR0_REQ;
              end if;
            end if;

          when ST_FIELD_WR0_REQ =>
            if v25_internal_data_selected(field_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= field_addr;
              mem_wdata_r <= field_b0;
            end if;
            state <= ST_FIELD_WR0_WAIT;

          when ST_FIELD_WR0_WAIT =>
            if v25_internal_data_selected(field_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(field_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(field_addr, field_b0);
              end if;
              state <= ST_FIELD_WR1_REQ;
            end if;

          when ST_FIELD_WR1_REQ =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= field_b1;
            end if;
            state <= ST_FIELD_WR1_WAIT;

          when ST_FIELD_WR1_WAIT =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, field_b1);
              end if;
              state <= ST_FIELD_WR2_REQ;
            end if;

          when ST_FIELD_WR2_REQ =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 2);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= field_b2;
            end if;
            state <= ST_FIELD_WR2_WAIT;

          when ST_FIELD_WR2_WAIT =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 2);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, field_b2);
              end if;
              state <= ST_FIELD_WR3_REQ;
            end if;

          when ST_FIELD_WR3_REQ =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 3);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= field_b3;
            end if;
            state <= ST_FIELD_WR3_WAIT;

          when ST_FIELD_WR3_WAIT =>
            ea_addr := std_logic_vector(unsigned(field_addr) + 3);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, field_b3);
              end if;
              reg_bank := regs(active_bank);
              field_next_offset := field_offset + field_length;
              if field_next_offset > 15 then
                field_next_offset := field_next_offset - 16;
                reg_bank(V25_REG_DI) := std_logic_vector(unsigned(regs(active_bank)(V25_REG_DI)) + 2);
              end if;
              reg_bank := v25_set_reg8(reg_bank, field_reg, std_logic_vector(to_unsigned(field_next_offset, 8)));
              regs(active_bank) <= reg_bank;
              state <= ST_FETCH_REQ;
            end if;

          when ST_PREPARE_COPY_RD_LO_REQ =>
            sp_next := std_logic_vector(unsigned(prepare_copy_bp) - 2);
            prepare_copy_bp <= sp_next;
            ea_addr := v25_phys_addr(ss, sp_next);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_PREPARE_COPY_RD_LO_WAIT;

          when ST_PREPARE_COPY_RD_LO_WAIT =>
            ea_addr := v25_phys_addr(ss, prepare_copy_bp);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                mem_low <= mem_byte;
              else
                mem_low <= mem_rdata;
              end if;
              state <= ST_PREPARE_COPY_RD_HI_REQ;
            end if;

          when ST_PREPARE_COPY_RD_HI_REQ =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(prepare_copy_bp) + 1));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_PREPARE_COPY_RD_HI_WAIT;

          when ST_PREPARE_COPY_RD_HI_WAIT =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(prepare_copy_bp) + 1));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                r16 := mem_byte & mem_low;
              else
                r16 := mem_rdata & mem_low;
              end if;
              sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
              push_value <= r16;
              push_mode <= PUSH_PREPARE_COPY;
              regs(active_bank)(V25_REG_SP) <= sp_next;
              state <= ST_PUSH_LO_REQ;
            end if;

          when ST_PUSH_LO_REQ =>
            ea_addr := v25_phys_addr(ss, regs(active_bank)(V25_REG_SP));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= push_value(7 downto 0);
            end if;
            state <= ST_PUSH_LO_WAIT;

          when ST_PUSH_LO_WAIT =>
            ea_addr := v25_phys_addr(ss, regs(active_bank)(V25_REG_SP));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, push_value(7 downto 0));
              end if;
              state <= ST_PUSH_HI_REQ;
            end if;

          when ST_PUSH_HI_REQ =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 1));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= push_value(15 downto 8);
            end if;
            state <= ST_PUSH_HI_WAIT;

          when ST_PUSH_HI_WAIT =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 1));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, push_value(15 downto 8));
              end if;
              if push_mode = PUSH_THEN_JUMP then
                ip <= branch_ip;
                state <= ST_FETCH_REQ;
              elsif push_mode = PUSH_FAR_THEN_JUMP then
                state <= ST_PUSH_FAR_SEG_LO_REQ;
              elsif push_mode = PUSH_INTERRUPT_PSW then
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= ps;
                push_mode <= PUSH_INTERRUPT_PS;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                state <= ST_PUSH_LO_REQ;
              elsif push_mode = PUSH_INTERRUPT_PS then
                sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                push_value <= int_return_ip;
                push_mode <= PUSH_INTERRUPT_PC;
                regs(active_bank)(V25_REG_SP) <= sp_next;
                ps <= int_target_ps;
                bank_ps(active_bank) <= int_target_ps;
                state <= ST_PUSH_LO_REQ;
              elsif push_mode = PUSH_INTERRUPT_PC then
                ip <= int_target_ip;
                state <= ST_FETCH_REQ;
              elsif push_mode = PUSH_REGS then
                if push_regs_index = 7 then
                  state <= ST_FETCH_REQ;
                else
                  push_regs_index <= push_regs_index + 1;
                  r16 := push_regs_value(regs(active_bank), push_regs_index + 1, push_sp_save);
                  sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                  push_value <= r16;
                  regs(active_bank)(V25_REG_SP) <= sp_next;
                  state <= ST_PUSH_LO_REQ;
                end if;
              elsif push_mode = PUSH_PREPARE then
                prepare_temp <= regs(active_bank)(V25_REG_SP);
                if prepare_level = 0 then
                  regs(active_bank)(V25_REG_BP) <= regs(active_bank)(V25_REG_SP);
                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - unsigned(imm16));
                  state <= ST_FETCH_REQ;
                elsif prepare_level = 1 then
                  r16 := regs(active_bank)(V25_REG_SP);
                  sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                  push_value <= r16;
                  push_mode <= PUSH_PREPARE_TEMP;
                  regs(active_bank)(V25_REG_SP) <= sp_next;
                  state <= ST_PUSH_LO_REQ;
                else
                  prepare_copy_bp <= regs(active_bank)(V25_REG_BP);
                  prepare_remaining <= prepare_level - 1;
                  state <= ST_PREPARE_COPY_RD_LO_REQ;
                end if;
              elsif push_mode = PUSH_PREPARE_COPY then
                if prepare_remaining = 1 then
                  sp_next := std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - 2);
                  push_value <= prepare_temp;
                  push_mode <= PUSH_PREPARE_TEMP;
                  regs(active_bank)(V25_REG_SP) <= sp_next;
                  state <= ST_PUSH_LO_REQ;
                else
                  prepare_remaining <= prepare_remaining - 1;
                  state <= ST_PREPARE_COPY_RD_LO_REQ;
                end if;
              elsif push_mode = PUSH_PREPARE_TEMP then
                regs(active_bank)(V25_REG_BP) <= prepare_temp;
                regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) - unsigned(imm16));
                state <= ST_FETCH_REQ;
              else
                state <= ST_FETCH_REQ;
              end if;
            end if;

          when ST_PUSH_FAR_SEG_LO_REQ =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= ps(7 downto 0);
            end if;
            state <= ST_PUSH_FAR_SEG_LO_WAIT;

          when ST_PUSH_FAR_SEG_LO_WAIT =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, ps(7 downto 0));
              end if;
              state <= ST_PUSH_FAR_SEG_HI_REQ;
            end if;

          when ST_PUSH_FAR_SEG_HI_REQ =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 3));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '1';
              mem_addr_r  <= ea_addr;
              mem_wdata_r <= ps(15 downto 8);
            end if;
            state <= ST_PUSH_FAR_SEG_HI_WAIT;

          when ST_PUSH_FAR_SEG_HI_WAIT =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 3));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                write_internal_data_byte(ea_addr, ps(15 downto 8));
              end if;
              ip <= branch_ip;
              ps <= far_seg;
              bank_ps(active_bank) <= far_seg;
              state <= ST_FETCH_REQ;
            end if;

          when ST_POP_LO_REQ =>
            ea_addr := v25_phys_addr(ss, regs(active_bank)(V25_REG_SP));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_POP_LO_WAIT;

          when ST_POP_LO_WAIT =>
            ea_addr := v25_phys_addr(ss, regs(active_bank)(V25_REG_SP));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                pop_low <= mem_byte;
                tmp_nat := v25_internal_data_index(ea_addr);
                if tmp_nat = SFR_RXB0_INDEX then
                  serial0_rx_unread <= '0';
                elsif tmp_nat = SFR_RXB1_INDEX then
                  serial1_rx_unread <= '0';
                end if;
              else
                pop_low <= mem_rdata;
              end if;
              state <= ST_POP_HI_REQ;
            end if;

          when ST_POP_HI_REQ =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 1));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_POP_HI_WAIT;

          when ST_POP_HI_WAIT =>
            ea_addr := v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 1));
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                tmp_nat := v25_internal_data_index(ea_addr);
                if tmp_nat = SFR_RXB0_INDEX then
                  serial0_rx_unread <= '0';
                elsif tmp_nat = SFR_RXB1_INDEX then
                  serial1_rx_unread <= '0';
                end if;
                r16 := mem_byte & pop_low;
              else
                r16 := mem_rdata & pop_low;
              end if;

              case pop_mode is
                when POP_TO_IP =>
                  ip <= r16;
                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                  state <= ST_FETCH_REQ;
                when POP_TO_IP_ADJ =>
                  ip <= r16;
                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + unsigned(stack_adjust) + 2);
                  state <= ST_FETCH_REQ;
                when POP_TO_REG =>
                  regs(active_bank)(pop_target) <= r16;
                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                  state <= ST_FETCH_REQ;
                when POP_TO_MEM =>
                  mem_value <= r16;
                  mem_is_word <= '1';
                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                  state <= ST_MEM_WR_LO_REQ;
                when POP_TO_PSW =>
                  f := v25_unpack_psw(r16);
                  flags <= f;
                  bank_ps(active_bank) <= ps;
                  bank_ss(active_bank) <= ss;
                  bank_ds0(active_bank) <= ds0;
                  bank_ds1(active_bank) <= ds1;
                  previous_bank <= active_bank;
                  tmp_nat := to_integer(unsigned(f.rb));
                  active_bank <= tmp_nat;
                  ps <= bank_ps(tmp_nat);
                  ss <= bank_ss(tmp_nat);
                  ds0 <= bank_ds0(tmp_nat);
                  ds1 <= bank_ds1(tmp_nat);
                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                  state <= ST_FETCH_REQ;
                when POP_TO_REGS =>
                  case pop_regs_index is
                    when 7 =>
                      regs(active_bank)(V25_REG_DI) <= r16;
                    when 6 =>
                      regs(active_bank)(V25_REG_SI) <= r16;
                    when 5 =>
                      regs(active_bank)(V25_REG_BP) <= r16;
                    when 4 =>
                      null;
                    when 3 =>
                      regs(active_bank)(V25_REG_BX) <= r16;
                    when 2 =>
                      regs(active_bank)(V25_REG_DX) <= r16;
                    when 1 =>
                      regs(active_bank)(V25_REG_CX) <= r16;
                    when others =>
                      regs(active_bank)(V25_REG_AX) <= r16;
                  end case;

                  regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                  if pop_regs_index = 0 then
                    state <= ST_FETCH_REQ;
                  else
                    pop_regs_index <= pop_regs_index - 1;
                    state <= ST_POP_LO_REQ;
                  end if;
                when POP_TO_SREG =>
                  case mem_sreg_target is
                    when SEG_SS =>
                      ss <= r16;
                      bank_ss(active_bank) <= r16;
                      regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                      state <= ST_FETCH_REQ;
                    when SEG_DS0 =>
                      ds0 <= r16;
                      bank_ds0(active_bank) <= r16;
                      regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                      state <= ST_FETCH_REQ;
                    when SEG_DS1 =>
                      ds1 <= r16;
                      bank_ds1(active_bank) <= r16;
                      regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2);
                      state <= ST_FETCH_REQ;
                    when others =>
                      fault_r <= '1';
                      state <= ST_FAULT;
                  end case;
                when POP_TO_IP_FAR =>
                  branch_ip <= r16;
                  mem_op_addr <= v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2));
                  state <= ST_POP_FAR_SEG_LO_REQ;
                when POP_TO_IP_FAR_ADJ =>
                  branch_ip <= r16;
                  mem_op_addr <= v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2));
                  state <= ST_POP_FAR_SEG_LO_REQ;
                when POP_TO_IP_FAR_PSW =>
                  branch_ip <= r16;
                  mem_op_addr <= v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 2));
                  state <= ST_POP_FAR_SEG_LO_REQ;
              end case;
            end if;

          when ST_POP_FAR_SEG_LO_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= mem_op_addr;
            end if;
            state <= ST_POP_FAR_SEG_LO_WAIT;

          when ST_POP_FAR_SEG_LO_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(mem_op_addr, mem_byte);
                pop_low <= mem_byte;
              else
                pop_low <= mem_rdata;
              end if;
              state <= ST_POP_FAR_SEG_HI_REQ;
            end if;

          when ST_POP_FAR_SEG_HI_REQ =>
            ea_addr := std_logic_vector(unsigned(mem_op_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_POP_FAR_SEG_HI_WAIT;

          when ST_POP_FAR_SEG_HI_WAIT =>
            ea_addr := std_logic_vector(unsigned(mem_op_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                r16 := mem_byte & pop_low;
              else
                r16 := mem_rdata & pop_low;
              end if;

              ip <= branch_ip;
              ps <= r16;
              bank_ps(active_bank) <= r16;
              if pop_mode = POP_TO_IP_FAR_PSW then
                mem_op_addr <= v25_phys_addr(ss, std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 4));
                state <= ST_POP_PSW_LO_REQ;
              elsif pop_mode = POP_TO_IP_FAR_ADJ then
                regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + unsigned(stack_adjust) + 4);
                state <= ST_FETCH_REQ;
              else
                regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 4);
                state <= ST_FETCH_REQ;
              end if;
            end if;

          when ST_POP_PSW_LO_REQ =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= mem_op_addr;
            end if;
            state <= ST_POP_PSW_LO_WAIT;

          when ST_POP_PSW_LO_WAIT =>
            if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(mem_op_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(mem_op_addr, mem_byte);
                pop_low <= mem_byte;
              else
                pop_low <= mem_rdata;
              end if;
              state <= ST_POP_PSW_HI_REQ;
            end if;

          when ST_POP_PSW_HI_REQ =>
            ea_addr := std_logic_vector(unsigned(mem_op_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
              mem_valid_r <= '0';
              mem_write_r <= '0';
            else
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= ea_addr;
            end if;
            state <= ST_POP_PSW_HI_WAIT;

          when ST_POP_PSW_HI_WAIT =>
            ea_addr := std_logic_vector(unsigned(mem_op_addr) + 1);
            if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) or mem_ready = '1' then
              mem_valid_r <= '0';
              if v25_internal_data_selected(ea_addr, idb_high, internal_data(SFR_PRC_INDEX)) then
                read_internal_data_byte(ea_addr, mem_byte);
                r16 := mem_byte & pop_low;
              else
                r16 := mem_rdata & pop_low;
              end if;
              f := v25_unpack_psw(r16);
              flags <= f;
              regs(active_bank)(V25_REG_SP) <= std_logic_vector(unsigned(regs(active_bank)(V25_REG_SP)) + 6);
              bank_ps(active_bank) <= ps;
              bank_ss(active_bank) <= ss;
              bank_ds0(active_bank) <= ds0;
              bank_ds1(active_bank) <= ds1;
              previous_bank <= active_bank;
              tmp_nat := to_integer(unsigned(f.rb));
              active_bank <= tmp_nat;
              ps <= bank_ps(tmp_nat);
              ss <= bank_ss(tmp_nat);
              ds0 <= bank_ds0(tmp_nat);
              ds1 <= bank_ds1(tmp_nat);
              state <= ST_FETCH_REQ;
            end if;

          when ST_POLL_WAIT =>
            mem_valid_r <= '0';
            mem_write_r <= '0';
            io_valid_r <= '0';
            io_write_r <= '0';
            if poll_wait_count = 0 then
              if poll_pin_waiting(internal_data, port1_in) then
                poll_wait_count <= 4;
              else
                state <= ST_FETCH_REQ;
              end if;
            else
              poll_wait_count <= poll_wait_count - 1;
            end if;

          when ST_HALTED =>
            mem_valid_r <= '0';
            mem_write_r <= '0';
            io_valid_r <= '0';
            io_write_r <= '0';
            timer0_tick_v := '0';
            timer0_md_tick_v := '0';
            timer1_tick_v := '0';
            if timer0_pending_v > 0 then
              timer0_tick_v := '1';
              timer0_pending_v := timer0_pending_v - 1;
            end if;
            if timer0_md_pending_v > 0 then
              timer0_md_tick_v := '1';
              timer0_md_pending_v := timer0_md_pending_v - 1;
            end if;
            if timer1_pending_v > 0 then
              timer1_tick_v := '1';
              timer1_pending_v := timer1_pending_v - 1;
            end if;
            tick_timer_unit(internal_data, timer_tout_r, timer0_tick_v, timer0_md_tick_v, timer1_tick_v);
            select_internal_irq(internal_data, internal_irq_take, internal_irq_vector,
              internal_irq_priority, internal_irq_service_index);

            if nmi_pending = '1' then
              nmi_pending <= '0';
              halt_stop_mode <= '0';
              int_vector_base <= interrupt_vector_base(IRQ_VEC_NMI);
              int_ibrk_after <= '0';
              int_return_ip <= ip;
              mem_valid_r <= '1';
              mem_write_r <= '0';
              mem_addr_r  <= interrupt_vector_base(IRQ_VEC_NMI);
              state <= ST_INT_VEC_IP_LO_REQ;
            elsif halt_stop_mode = '0' and internal_irq_take and
              macro_normal_pending(internal_data, internal_irq_service_index) then
              halt_stop_mode <= '0';
              start_macro_service(internal_irq_service_index);
            elsif halt_stop_mode = '0' and internal_irq_take and flags.iflag = '1' then
              halt_stop_mode <= '0';
              if internal_data(internal_irq_service_index)(5) = '0' and
                internal_data(internal_irq_service_index)(4) = '1' then
                start_interrupt_bank_switch(
                  internal_data,
                  bank_saved_ip,
                  bank_saved_psw,
                  bank_ps,
                  bank_ss,
                  bank_ds0,
                  bank_ds1,
                  flags,
                  previous_bank,
                  active_bank,
                  ip,
                  ps,
                  ss,
                  ds0,
                  ds1,
                  internal_irq_service_index,
                  internal_irq_priority,
                  ip,
                  flags,
                  active_bank,
                  ps,
                  ss,
                  ds0,
                  ds1,
                  bank_vector_ip
                );
                state <= ST_FETCH_REQ;
              else
                irq_ispr_mask := shift_left(to_unsigned(1, 8), internal_irq_priority);
                internal_data(SFR_ISPR_INDEX) <= std_logic_vector(unsigned(internal_data(SFR_ISPR_INDEX)) or irq_ispr_mask);
                internal_data(internal_irq_service_index)(7) <= '0';
                int_vector_base <= interrupt_vector_base(internal_irq_vector);
                int_ibrk_after <= '0';
                int_return_ip <= ip;
                mem_valid_r <= '1';
                mem_write_r <= '0';
                mem_addr_r  <= interrupt_vector_base(internal_irq_vector);
                state <= ST_INT_VEC_IP_LO_REQ;
              end if;
            elsif halt_stop_mode = '0' and irq_request = '1' and flags.iflag = '1' then
              halt_stop_mode <= '0';
              external_irq_vector <= irq_vector;
              int_ibrk_after <= '0';
              int_return_ip <= ip;
              int_ack_valid_r <= '1';
              int_ack_second_r <= '0';
              state <= ST_EXT_INT_ACK1_REQ;
            elsif halt_stop_mode = '0' and dma_transfer_pending(internal_data, 0, dmarq0_edge_v, dmarq0_level_v) then
              dma_base_v := dma_channel_base(0);
              dma_mode_v := internal_data(SFR_DMAM0_INDEX)(7 downto 5);
              dma_src_offset := timer_word(internal_data, dma_base_v);
              dma_dst_offset := timer_word(internal_data, dma_base_v + 2);
              dma_active_channel <= 0;
              dma_src_addr <= dma_phys_addr(internal_data(dma_base_v + 5), dma_src_offset);
              dma_dst_addr <= dma_phys_addr(internal_data(dma_base_v + 4), dma_dst_offset);
              dma_word_mode <= internal_data(SFR_DMAM0_INDEX)(4);
              dma_high_phase <= '0';
              if dma_mode_io_to_mem(dma_mode_v) then
                  dma_source_is_io <= '1';
                else
                  dma_source_is_io <= '0';
                end if;
                if dma_mode_mem_to_io(dma_mode_v) then
                  dma_dest_is_io <= '1';
                else
                  dma_dest_is_io <= '0';
                end if;
              halt_stop_mode <= '0';
              state <= ST_DMA_RD_REQ;
            elsif halt_stop_mode = '0' and dma_transfer_pending(internal_data, 1, dmarq1_edge_v, dmarq1_level_v) then
              dma_base_v := dma_channel_base(1);
              dma_mode_v := internal_data(SFR_DMAM1_INDEX)(7 downto 5);
              dma_src_offset := timer_word(internal_data, dma_base_v);
              dma_dst_offset := timer_word(internal_data, dma_base_v + 2);
              dma_active_channel <= 1;
              dma_src_addr <= dma_phys_addr(internal_data(dma_base_v + 5), dma_src_offset);
              dma_dst_addr <= dma_phys_addr(internal_data(dma_base_v + 4), dma_dst_offset);
              dma_word_mode <= internal_data(SFR_DMAM1_INDEX)(4);
              dma_high_phase <= '0';
              if dma_mode_io_to_mem(dma_mode_v) then
                  dma_source_is_io <= '1';
                else
                  dma_source_is_io <= '0';
                end if;
                if dma_mode_mem_to_io(dma_mode_v) then
                  dma_dest_is_io <= '1';
                else
                  dma_dest_is_io <= '0';
                end if;
              halt_stop_mode <= '0';
              state <= ST_DMA_RD_REQ;
            end if;

          when ST_FAULT =>
            fault_r <= '1';
            mem_valid_r <= '0';
            mem_write_r <= '0';
            io_valid_r <= '0';
            io_write_r <= '0';
        end case;
        timer0_tick_pending <= timer0_pending_v;
        timer0_md_tick_pending <= timer0_md_pending_v;
        timer1_tick_pending <= timer1_pending_v;
      end if;
    end if;
  end process;
end architecture;
