-- SPDX-License-Identifier: BSD-3-Clause
--
-- NEC V25 timing equations from the uPD70320 data sheet tables.
--
-- These helpers intentionally model the documented instruction-clock formulas,
-- not the current implementation FSM latency. They are the common reference for
-- timing assertions, future instruction throttling, and trace comparison.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.v25_pkg.all;

package v25_timing_pkg is
  type v25_bit_timing_op_t is (
    V25_TIM_TEST1,
    V25_TIM_NOT1,
    V25_TIM_CLR1,
    V25_TIM_SET1
  );

  function v25_ea_clocks(modrm : byte_t) return natural;

  function v25_clocks_nop return natural;
  function v25_clocks_segment_prefix return natural;
  function v25_clocks_repeat_prefix return natural;
  function v25_clocks_buslock return natural;
  function v25_clocks_di return natural;
  function v25_clocks_ei return natural;
  function v25_clocks_fint return natural;
  function v25_clocks_retrbi return natural;
  function v25_clocks_brkcs return natural;
  function v25_clocks_movspa return natural;
  function v25_clocks_movspb return natural;
  function v25_clocks_tsksw return natural;
  function v25_clocks_btclr(taken : boolean) return natural;
  function v25_clocks_cond_branch(taken : boolean) return natural;
  function v25_clocks_dbnz(taken : boolean) return natural;
  function v25_clocks_bcwz(taken : boolean) return natural;
  function v25_clocks_branch_near return natural;
  function v25_clocks_branch_short return natural;
  function v25_clocks_branch_regptr16 return natural;
  function v25_clocks_branch_memptr16(
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_branch_far return natural;
  function v25_clocks_branch_memptr32(
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_call_near(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_call_far(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_call_memptr16(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_call_memptr32(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_ret_near(wait_states : natural) return natural;
  function v25_clocks_ret_near_pop(wait_states : natural) return natural;
  function v25_clocks_ret_far(wait_states : natural) return natural;
  function v25_clocks_ret_far_pop(wait_states : natural) return natural;
  function v25_clocks_push_mem16(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_push_reg16(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_push_sreg(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_push_psw(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_push_regs(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_push_imm8(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_push_imm16(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_pop_mem16(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_pop_reg16(wait_states : natural) return natural;
  function v25_clocks_pop_sreg(wait_states : natural) return natural;
  function v25_clocks_pop_psw(wait_states : natural) return natural;
  function v25_clocks_pop_regs(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_prepare(level : natural; wait_states : natural) return natural;
  function v25_clocks_dispose(wait_states : natural) return natural;
  function v25_clocks_brk(
    vector_3 : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_brkv(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_reti(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_chkind(
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_incdec_reg(word_op : boolean) return natural;
  function v25_clocks_incdec_mem(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_notneg_reg return natural;
  function v25_clocks_notneg_mem(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mulu_reg(word_op : boolean) return natural;
  function v25_clocks_mulu_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mul_reg(word_op : boolean; slow_path : boolean) return natural;
  function v25_clocks_mul_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural;
    slow_path : boolean
  ) return natural;
  function v25_clocks_imul_imm_reg(imm8 : boolean; slow_path : boolean) return natural;
  function v25_clocks_imul_imm_mem(
    imm8 : boolean;
    ea_clocks : natural;
    wait_states : natural;
    slow_path : boolean
  ) return natural;
  function v25_clocks_divu_reg(word_op : boolean) return natural;
  function v25_clocks_divu_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_div_reg(word_op : boolean; slow_path : boolean) return natural;
  function v25_clocks_div_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural;
    slow_path : boolean
  ) return natural;
  function v25_clocks_adjba return natural;
  function v25_clocks_adj4a return natural;
  function v25_clocks_adjbs return natural;
  function v25_clocks_adj4s return natural;
  function v25_clocks_cvtbd return natural;
  function v25_clocks_cvtdb return natural;
  function v25_clocks_cvtbw return natural;
  function v25_clocks_cvtwl return natural;
  function v25_clocks_rol4_reg return natural;
  function v25_clocks_rol4_mem(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_ror4_reg return natural;
  function v25_clocks_ror4_mem(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_add4s(
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    bcd_bytes : natural
  ) return natural;
  function v25_clocks_sub4s(
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    bcd_bytes : natural
  ) return natural;
  function v25_clocks_cmp4s(
    wait_states : natural;
    bcd_bytes : natural
  ) return natural;
  function v25_clocks_shift_reg_1 return natural;
  function v25_clocks_shift_mem_1(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_shift_reg_cl(count : natural) return natural;
  function v25_clocks_shift_mem_cl(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural;
    count : natural
  ) return natural;
  function v25_clocks_shift_reg_imm(count : natural) return natural;
  function v25_clocks_shift_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural;
    count : natural
  ) return natural;

  function v25_clocks_trans(wait_states : natural) return natural;
  function v25_clocks_movk_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_movk_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_cmpk_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_cmpk_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_cmpm_single(
    word_op : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_cmpm_repeat(
    word_op : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_ldm_single(
    word_op : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_ldm_repeat(
    word_op : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_stm_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_stm_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_in_acc_imm(word_op : boolean; wait_states : natural) return natural;
  function v25_clocks_in_acc_dw(word_op : boolean; wait_states : natural) return natural;
  function v25_clocks_out_imm_acc(word_op : boolean; wait_states : natural) return natural;
  function v25_clocks_out_dw_acc(word_op : boolean; wait_states : natural) return natural;
  function v25_clocks_inm_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_inm_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_outm_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_outm_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural;
  function v25_clocks_ins_field_reg(slow_path : boolean) return natural;
  function v25_clocks_ins_field_imm(slow_path : boolean) return natural;
  function v25_clocks_ext_field_reg(slow_path : boolean) return natural;
  function v25_clocks_ext_field_imm(slow_path : boolean) return natural;

  function v25_clocks_mov_reg_reg return natural;
  function v25_clocks_mov_reg_imm(word_op : boolean) return natural;
  function v25_clocks_mov_mem_reg(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_reg_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_acc_dmem(
    word_op : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_dmem_acc(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_sreg_reg return natural;
  function v25_clocks_mov_sreg_mem(
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_reg_sreg return natural;
  function v25_clocks_mov_mem_sreg(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_ldea(ea_clocks : natural) return natural;
  function v25_clocks_mov_ds_reg_mem32(
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_mov_ah_psw return natural;
  function v25_clocks_mov_psw_ah return natural;
  function v25_clocks_xchg_reg_reg return natural;
  function v25_clocks_xchg_acc_reg16 return natural;
  function v25_clocks_xchg_mem_reg(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;

  function v25_clocks_alu_reg_reg return natural;
  function v25_clocks_alu_reg_imm(word_op : boolean) return natural;
  function v25_clocks_alu_mem_reg(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_alu_reg_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_addsub_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_logic_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;

  function v25_clocks_cmp_mem_reg(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_cmp_mem_imm(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_test_mem_reg(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_test_reg_reg return natural;
  function v25_clocks_test_reg_imm(word_op : boolean) return natural;
  function v25_clocks_test_acc_imm(word_op : boolean) return natural;
  function v25_clocks_test_mem_imm(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;

  function v25_clocks_bitop_reg_imm(op : v25_bit_timing_op_t; word_op : boolean) return natural;
  function v25_clocks_bitop_reg_cl(op : v25_bit_timing_op_t; word_op : boolean) return natural;
  function v25_clocks_bitop_mem_imm(
    op : v25_bit_timing_op_t;
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
  function v25_clocks_bitop_mem_cl(
    op : v25_bit_timing_op_t;
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural;
end package;

package body v25_timing_pkg is
  function v25_ea_clocks(modrm : byte_t) return natural is
  begin
    if modrm(7 downto 6) = "11" then
      return 0;
    elsif modrm(7 downto 6) = "10" then
      return 4;
    else
      return 3;
    end if;
  end function;

  function v25_clocks_nop return natural is
  begin
    return 4;
  end function;

  function v25_clocks_segment_prefix return natural is
  begin
    return 2;
  end function;

  function v25_clocks_repeat_prefix return natural is
  begin
    return 2;
  end function;

  function v25_clocks_buslock return natural is
  begin
    return 2;
  end function;

  function v25_clocks_di return natural is
  begin
    return 4;
  end function;

  function v25_clocks_ei return natural is
  begin
    return 12;
  end function;

  function v25_clocks_fint return natural is
  begin
    return 2;
  end function;

  function v25_clocks_retrbi return natural is
  begin
    return 12;
  end function;

  function v25_clocks_brkcs return natural is
  begin
    return 15;
  end function;

  function v25_clocks_movspa return natural is
  begin
    return 16;
  end function;

  function v25_clocks_movspb return natural is
  begin
    return 11;
  end function;

  function v25_clocks_tsksw return natural is
  begin
    return 20;
  end function;

  function v25_clocks_btclr(taken : boolean) return natural is
  begin
    if taken then
      return 29;
    end if;
    return 21;
  end function;

  function v25_clocks_cond_branch(taken : boolean) return natural is
  begin
    if taken then
      return 15;
    end if;
    return 8;
  end function;

  function v25_clocks_dbnz(taken : boolean) return natural is
  begin
    if taken then
      return 17;
    end if;
    return 8;
  end function;

  function v25_clocks_bcwz(taken : boolean) return natural is
  begin
    return v25_clocks_cond_branch(taken);
  end function;

  function v25_clocks_branch_near return natural is
  begin
    return 12;
  end function;

  function v25_clocks_branch_short return natural is
  begin
    return 12;
  end function;

  function v25_clocks_branch_regptr16 return natural is
  begin
    return 13;
  end function;

  function v25_clocks_branch_memptr16(
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    return ea_clocks + 17 + (2 * wait_states);
  end function;

  function v25_clocks_branch_far return natural is
  begin
    return 15;
  end function;

  function v25_clocks_branch_memptr32(
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    return ea_clocks + 25 + (4 * wait_states);
  end function;

  function v25_clocks_call_near(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 22 + (2 * wait_states);
    end if;
    return 18 + (2 * wait_states);
  end function;

  function v25_clocks_call_far(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 38 + (4 * wait_states);
    end if;
    return 34 + (4 * wait_states);
  end function;

  function v25_clocks_call_memptr16(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 26 + (4 * wait_states);
    end if;
    return ea_clocks + 24 + (4 * wait_states);
  end function;

  function v25_clocks_call_memptr32(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 36 + (8 * wait_states);
    end if;
    return ea_clocks + 24 + (8 * wait_states);
  end function;

  function v25_clocks_ret_near(wait_states : natural) return natural is
  begin
    return 20 + (2 * wait_states);
  end function;

  function v25_clocks_ret_near_pop(wait_states : natural) return natural is
  begin
    return 20 + (2 * wait_states);
  end function;

  function v25_clocks_ret_far(wait_states : natural) return natural is
  begin
    return 29 + (4 * wait_states);
  end function;

  function v25_clocks_ret_far_pop(wait_states : natural) return natural is
  begin
    return 30 + (4 * wait_states);
  end function;

  function v25_clocks_push_mem16(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 18 + (4 * wait_states);
    end if;
    return ea_clocks + 14 + (4 * wait_states);
  end function;

  function v25_clocks_push_reg16(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 10 + (2 * wait_states);
    end if;
    return 6;
  end function;

  function v25_clocks_push_sreg(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 11 + (2 * wait_states);
    end if;
    return 7;
  end function;

  function v25_clocks_push_psw(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 10 + (2 * wait_states);
    end if;
    return 6;
  end function;

  function v25_clocks_push_regs(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 82 + (16 * wait_states);
    end if;
    return 50;
  end function;

  function v25_clocks_push_imm8(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 13 + (2 * wait_states);
    end if;
    return 9;
  end function;

  function v25_clocks_push_imm16(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 14 + (2 * wait_states);
    end if;
    return 10;
  end function;

  function v25_clocks_pop_mem16(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 16 + (4 * wait_states);
    end if;
    return ea_clocks + 12 + (2 * wait_states);
  end function;

  function v25_clocks_pop_reg16(wait_states : natural) return natural is
  begin
    return 12 + (2 * wait_states);
  end function;

  function v25_clocks_pop_sreg(wait_states : natural) return natural is
  begin
    return 13 + (2 * wait_states);
  end function;

  function v25_clocks_pop_psw(wait_states : natural) return natural is
  begin
    return 14 + (2 * wait_states);
  end function;

  function v25_clocks_pop_regs(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 82 + (16 * wait_states);
    end if;
    return 58;
  end function;

  function v25_clocks_prepare(level : natural; wait_states : natural) return natural is
  begin
    if level = 0 then
      return 27 + (2 * wait_states);
    elsif level = 1 then
      return 39 + (4 * wait_states);
    end if;
    return 46 + (19 * (level - 1)) + (4 * wait_states);
  end function;

  function v25_clocks_dispose(wait_states : natural) return natural is
  begin
    return 12 + (2 * wait_states);
  end function;

  function v25_clocks_brk(
    vector_3 : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
    variable base_clocks : natural;
  begin
    if onchip_ram_access_enabled then
      if vector_3 then
        base_clocks := 55;
      else
        base_clocks := 56;
      end if;
    elsif vector_3 then
      base_clocks := 43;
    else
      base_clocks := 44;
    end if;
    return base_clocks + (10 * wait_states);
  end function;

  function v25_clocks_brkv(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    return v25_clocks_brk(true, onchip_ram_access_enabled, wait_states);
  end function;

  function v25_clocks_reti(
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 45 + (6 * wait_states);
    end if;
    return 37 + (2 * wait_states);
  end function;

  function v25_clocks_chkind(
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    return ea_clocks + 26 + (4 * wait_states);
  end function;

  function v25_clocks_incdec_reg(word_op : boolean) return natural is
  begin
    if word_op then
      return 2;
    end if;
    return 5;
  end function;

  function v25_clocks_incdec_mem(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 15 + (4 * wait_states);
      end if;
      return ea_clocks + 11 + (4 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 11 + (2 * wait_states);
    end if;
    return ea_clocks + 9 + (2 * wait_states);
  end function;

  function v25_clocks_notneg_reg return natural is
  begin
    return 5;
  end function;

  function v25_clocks_notneg_mem(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 15 + (4 * wait_states);
      end if;
      return ea_clocks + 11 + (2 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 11 + (2 * wait_states);
    end if;
    return ea_clocks + 9 + wait_states;
  end function;

  function v25_clocks_mulu_reg(word_op : boolean) return natural is
  begin
    if word_op then
      return 32;
    end if;
    return 24;
  end function;

  function v25_clocks_mulu_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 34 + (2 * wait_states);
    end if;
    return ea_clocks + 26 + wait_states;
  end function;

  function v25_clocks_mul_reg(word_op : boolean; slow_path : boolean) return natural is
  begin
    if word_op then
      if slow_path then
        return 48;
      end if;
      return 39;
    end if;

    if slow_path then
      return 40;
    end if;
    return 31;
  end function;

  function v25_clocks_mul_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural;
    slow_path : boolean
  ) return natural is
  begin
    if word_op then
      if slow_path then
        return ea_clocks + 52 + (2 * wait_states);
      end if;
      return ea_clocks + 43 + (2 * wait_states);
    end if;

    if slow_path then
      return ea_clocks + 42 + wait_states;
    end if;
    return ea_clocks + 33 + wait_states;
  end function;

  function v25_clocks_imul_imm_reg(imm8 : boolean; slow_path : boolean) return natural is
  begin
    if imm8 then
      if slow_path then
        return 49;
      end if;
      return 39;
    end if;

    if slow_path then
      return 50;
    end if;
    return 40;
  end function;

  function v25_clocks_imul_imm_mem(
    imm8 : boolean;
    ea_clocks : natural;
    wait_states : natural;
    slow_path : boolean
  ) return natural is
  begin
    if imm8 then
      if slow_path then
        return ea_clocks + 53 + (2 * wait_states);
      end if;
      return ea_clocks + 43 + (2 * wait_states);
    end if;

    if slow_path then
      return ea_clocks + 54 + (2 * wait_states);
    end if;
    return ea_clocks + 44 + (2 * wait_states);
  end function;

  function v25_clocks_divu_reg(word_op : boolean) return natural is
  begin
    if word_op then
      return 39;
    end if;
    return 31;
  end function;

  function v25_clocks_divu_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 43 + (2 * wait_states);
    end if;
    return ea_clocks + 33 + wait_states;
  end function;

  function v25_clocks_div_reg(word_op : boolean; slow_path : boolean) return natural is
  begin
    if word_op then
      if slow_path then
        return 64;
      end if;
      return 54;
    end if;

    if slow_path then
      return 56;
    end if;
    return 46;
  end function;

  function v25_clocks_div_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural;
    slow_path : boolean
  ) return natural is
  begin
    if word_op then
      if slow_path then
        return ea_clocks + 68 + (2 * wait_states);
      end if;
      return ea_clocks + 58 + (2 * wait_states);
    end if;

    if slow_path then
      return ea_clocks + 58 + wait_states;
    end if;
    return ea_clocks + 48 + wait_states;
  end function;

  function v25_clocks_adjba return natural is
  begin
    return 17;
  end function;

  function v25_clocks_adj4a return natural is
  begin
    return 9;
  end function;

  function v25_clocks_adjbs return natural is
  begin
    return 17;
  end function;

  function v25_clocks_adj4s return natural is
  begin
    return 9;
  end function;

  function v25_clocks_cvtbd return natural is
  begin
    return 19;
  end function;

  function v25_clocks_cvtdb return natural is
  begin
    return 20;
  end function;

  function v25_clocks_cvtbw return natural is
  begin
    return 3;
  end function;

  function v25_clocks_cvtwl return natural is
  begin
    return 8;
  end function;

  function v25_clocks_rol4_reg return natural is
  begin
    return 17;
  end function;

  function v25_clocks_rol4_mem(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 18 + (2 * wait_states);
    end if;
    return ea_clocks + 16 + (2 * wait_states);
  end function;

  function v25_clocks_ror4_reg return natural is
  begin
    return 21;
  end function;

  function v25_clocks_ror4_mem(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 24 + (2 * wait_states);
    end if;
    return ea_clocks + 22 + (2 * wait_states);
  end function;

  function v25_clocks_add4s(
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    bcd_bytes : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return 22 + ((27 + (3 * wait_states)) * bcd_bytes);
    end if;
    return 22 + ((25 + (3 * wait_states)) * bcd_bytes);
  end function;

  function v25_clocks_sub4s(
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    bcd_bytes : natural
  ) return natural is
  begin
    return v25_clocks_add4s(onchip_ram_access_enabled, wait_states, bcd_bytes);
  end function;

  function v25_clocks_cmp4s(
    wait_states : natural;
    bcd_bytes : natural
  ) return natural is
  begin
    return 22 + ((23 + (3 * wait_states)) * bcd_bytes);
  end function;

  function v25_clocks_shift_reg_1 return natural is
  begin
    return 8;
  end function;

  function v25_clocks_shift_mem_1(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 18 + (4 * wait_states);
      end if;
      return ea_clocks + 14 + (2 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 14 + (2 * wait_states);
    end if;
    return ea_clocks + 12 + wait_states;
  end function;

  function v25_clocks_shift_reg_cl(count : natural) return natural is
  begin
    return 11 + (2 * count);
  end function;

  function v25_clocks_shift_mem_cl(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural;
    count : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 21 + (4 * wait_states) + (2 * count);
      end if;
      return ea_clocks + 17 + (2 * wait_states) + (2 * count);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 17 + (2 * wait_states) + (2 * count);
    end if;
    return ea_clocks + 15 + wait_states + (2 * count);
  end function;

  function v25_clocks_shift_reg_imm(count : natural) return natural is
  begin
    return 9 + (2 * count);
  end function;

  function v25_clocks_shift_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural;
    count : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 17 + (4 * wait_states) + (2 * count);
      end if;
      return ea_clocks + 13 + (2 * wait_states) + (2 * count);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 13 + (2 * wait_states) + (2 * count);
    end if;
    return ea_clocks + 11 + wait_states + (2 * count);
  end function;

  function v25_clocks_trans(wait_states : natural) return natural is
  begin
    return 10 + wait_states;
  end function;

  function v25_clocks_movk_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 24 + (4 * wait_states);
      end if;
      return 20 + (2 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return 20 + (2 * wait_states);
    end if;
    return 16 + wait_states;
  end function;

  function v25_clocks_movk_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 16 + ((20 + (4 * wait_states)) * iterations);
      end if;
      return 16 + ((12 + (2 * wait_states)) * iterations);
    end if;

    if onchip_ram_access_enabled then
      return 16 + ((16 + (2 * wait_states)) * iterations);
    end if;
    return 16 + ((12 + wait_states) * iterations);
  end function;

  function v25_clocks_cmpk_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 27 + (4 * wait_states);
      end if;
      return 21 + (4 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return 23 + (2 * wait_states);
    end if;
    return 19 + wait_states;
  end function;

  function v25_clocks_cmpk_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 16 + ((25 + (4 * wait_states)) * iterations);
      end if;
      return 16 + ((25 + (2 * wait_states)) * iterations);
    end if;
    return 16 + ((21 + (2 * wait_states)) * iterations);
  end function;

  function v25_clocks_cmpm_single(
    word_op : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return 19 + (2 * wait_states);
    end if;
    return 17 + wait_states;
  end function;

  function v25_clocks_cmpm_repeat(
    word_op : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    if word_op then
      return 16 + ((17 + (2 * wait_states)) * iterations);
    end if;
    return 16 + ((15 + wait_states) * iterations);
  end function;

  function v25_clocks_ldm_single(
    word_op : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return 14 + (2 * wait_states);
    end if;
    return 12 + wait_states;
  end function;

  function v25_clocks_ldm_repeat(
    word_op : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    if word_op then
      return 16 + ((12 + (2 * wait_states)) * iterations);
    end if;
    return 16 + ((10 + wait_states) * iterations);
  end function;

  function v25_clocks_stm_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 14 + (2 * wait_states);
      end if;
      return 10;
    end if;

    if onchip_ram_access_enabled then
      return 12 + wait_states;
    end if;
    return 10;
  end function;

  function v25_clocks_stm_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 16 + ((10 + (2 * wait_states)) * iterations);
      end if;
      return 16 + ((6 + (2 * wait_states)) * iterations);
    end if;

    if onchip_ram_access_enabled then
      return 16 + ((8 + wait_states) * iterations);
    end if;
    return 16 + ((6 + wait_states) * iterations);
  end function;

  function v25_clocks_in_acc_imm(word_op : boolean; wait_states : natural) return natural is
  begin
    if word_op then
      return 16 + (2 * wait_states);
    end if;
    return 14 + wait_states;
  end function;

  function v25_clocks_in_acc_dw(word_op : boolean; wait_states : natural) return natural is
  begin
    if word_op then
      return 15 + (2 * wait_states);
    end if;
    return 13 + wait_states;
  end function;

  function v25_clocks_out_imm_acc(word_op : boolean; wait_states : natural) return natural is
  begin
    if word_op then
      return 10 + (2 * wait_states);
    end if;
    return 10 + wait_states;
  end function;

  function v25_clocks_out_dw_acc(word_op : boolean; wait_states : natural) return natural is
  begin
    if word_op then
      return 9 + (2 * wait_states);
    end if;
    return 9 + wait_states;
  end function;

  function v25_clocks_inm_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 21 + (4 * wait_states);
      end if;
      return 17 + (4 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return 19 + (2 * wait_states);
    end if;
    return 17 + (2 * wait_states);
  end function;

  function v25_clocks_inm_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return 18 + ((15 + (4 * wait_states)) * iterations);
      end if;
      return 18 + ((11 + (4 * wait_states)) * iterations);
    end if;

    if onchip_ram_access_enabled then
      return 18 + ((13 + (2 * wait_states)) * iterations);
    end if;
    return 18 + ((11 + (2 * wait_states)) * iterations);
  end function;

  function v25_clocks_outm_single(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    return v25_clocks_inm_single(word_op, onchip_ram_access_enabled, wait_states);
  end function;

  function v25_clocks_outm_repeat(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural;
    iterations : natural
  ) return natural is
  begin
    return v25_clocks_inm_repeat(word_op, onchip_ram_access_enabled, wait_states, iterations);
  end function;

  function v25_clocks_ins_field_reg(slow_path : boolean) return natural is
  begin
    if slow_path then
      return 155;
    end if;
    return 63;
  end function;

  function v25_clocks_ins_field_imm(slow_path : boolean) return natural is
  begin
    if slow_path then
      return 156;
    end if;
    return 64;
  end function;

  function v25_clocks_ext_field_reg(slow_path : boolean) return natural is
  begin
    if slow_path then
      return 121;
    end if;
    return 41;
  end function;

  function v25_clocks_ext_field_imm(slow_path : boolean) return natural is
  begin
    if slow_path then
      return 122;
    end if;
    return 42;
  end function;

  function v25_clocks_mov_reg_reg return natural is
  begin
    return 2;
  end function;

  function v25_clocks_mov_reg_imm(word_op : boolean) return natural is
  begin
    if word_op then
      return 6;
    end if;
    return 5;
  end function;

  function v25_clocks_mov_mem_reg(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 6 + (2 * wait_states);
      end if;
      return ea_clocks + 2;
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 4 + wait_states;
    end if;
    return ea_clocks + 2;
  end function;

  function v25_clocks_mov_reg_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 8 + (2 * wait_states);
    end if;
    return ea_clocks + 6 + wait_states;
  end function;

  function v25_clocks_mov_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op and onchip_ram_access_enabled then
      return ea_clocks + 5 + (2 * wait_states);
    end if;
    return ea_clocks + 5 + wait_states;
  end function;

  function v25_clocks_mov_acc_dmem(
    word_op : boolean;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return 11 + (2 * wait_states);
    end if;
    return 9 + wait_states;
  end function;

  function v25_clocks_mov_dmem_acc(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    wait_states : natural
  ) return natural is
  begin
    if not onchip_ram_access_enabled then
      return 5;
    elsif word_op then
      return 9 + (2 * wait_states);
    end if;
    return 7 + wait_states;
  end function;

  function v25_clocks_mov_sreg_reg return natural is
  begin
    return 4;
  end function;

  function v25_clocks_mov_sreg_mem(
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    return ea_clocks + 10 + (2 * wait_states);
  end function;

  function v25_clocks_mov_reg_sreg return natural is
  begin
    return 3;
  end function;

  function v25_clocks_mov_mem_sreg(
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if onchip_ram_access_enabled then
      return ea_clocks + 7 + (2 * wait_states);
    end if;
    return ea_clocks + 3;
  end function;

  function v25_clocks_ldea(ea_clocks : natural) return natural is
  begin
    return ea_clocks + 2;
  end function;

  function v25_clocks_mov_ds_reg_mem32(
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    return ea_clocks + 19 + (4 * wait_states);
  end function;

  function v25_clocks_mov_ah_psw return natural is
  begin
    return 2;
  end function;

  function v25_clocks_mov_psw_ah return natural is
  begin
    return 3;
  end function;

  function v25_clocks_xchg_reg_reg return natural is
  begin
    return 3;
  end function;

  function v25_clocks_xchg_acc_reg16 return natural is
  begin
    return 4;
  end function;

  function v25_clocks_xchg_mem_reg(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 14 + (4 * wait_states);
      end if;
      return ea_clocks + 10 + (4 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 10 + (2 * wait_states);
    end if;
    return ea_clocks + 8 + (2 * wait_states);
  end function;

  function v25_clocks_alu_reg_reg return natural is
  begin
    return 2;
  end function;

  function v25_clocks_alu_reg_imm(word_op : boolean) return natural is
  begin
    if word_op then
      return 6;
    end if;
    return 5;
  end function;

  function v25_clocks_alu_mem_reg(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 12 + (4 * wait_states);
      end if;
      return ea_clocks + 8 + (2 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 8 + (2 * wait_states);
    end if;
    return ea_clocks + 6 + wait_states;
  end function;

  function v25_clocks_alu_reg_mem(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 8 + (2 * wait_states);
    end if;
    return ea_clocks + 6 + wait_states;
  end function;

  function v25_clocks_addsub_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 14 + (4 * wait_states);
      end if;
      return ea_clocks + 10 + (4 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 9 + (2 * wait_states);
    end if;
    return ea_clocks + 7 + (2 * wait_states);
  end function;

  function v25_clocks_logic_mem_imm(
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      if onchip_ram_access_enabled then
        return ea_clocks + 14 + (4 * wait_states);
      end if;
      return ea_clocks + 10 + (4 * wait_states);
    end if;

    if onchip_ram_access_enabled then
      return ea_clocks + 9 + wait_states;
    end if;
    return ea_clocks + 7 + wait_states;
  end function;

  function v25_clocks_cmp_mem_reg(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 8 + (2 * wait_states);
    end if;
    return ea_clocks + 6 + wait_states;
  end function;

  function v25_clocks_cmp_mem_imm(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 10 + (2 * wait_states);
    end if;
    return ea_clocks + 7 + wait_states;
  end function;

  function v25_clocks_test_mem_reg(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 10 + (2 * wait_states);
    end if;
    return ea_clocks + 8 + wait_states;
  end function;

  function v25_clocks_test_reg_reg return natural is
  begin
    return 4;
  end function;

  function v25_clocks_test_reg_imm(word_op : boolean) return natural is
  begin
    if word_op then
      return 8;
    end if;
    return 7;
  end function;

  function v25_clocks_test_acc_imm(word_op : boolean) return natural is
  begin
    if word_op then
      return 6;
    end if;
    return 5;
  end function;

  function v25_clocks_test_mem_imm(
    word_op : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    if word_op then
      return ea_clocks + 11 + (2 * wait_states);
    end if;
    return ea_clocks + 11 + wait_states;
  end function;

  function v25_clocks_bitop_reg_imm(op : v25_bit_timing_op_t; word_op : boolean) return natural is
  begin
    case op is
      when V25_TIM_TEST1 | V25_TIM_NOT1 | V25_TIM_SET1 =>
        return 6;
      when V25_TIM_CLR1 =>
        return 7;
    end case;
  end function;

  function v25_clocks_bitop_reg_cl(op : v25_bit_timing_op_t; word_op : boolean) return natural is
  begin
    case op is
      when V25_TIM_CLR1 =>
        return 8;
      when others =>
        return 7;
    end case;
  end function;

  function v25_clocks_bitop_mem_imm(
    op : v25_bit_timing_op_t;
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    case op is
      when V25_TIM_TEST1 =>
        if word_op then
          return ea_clocks + 10 + (2 * wait_states);
        end if;
        return ea_clocks + 8 + wait_states;

      when V25_TIM_NOT1 | V25_TIM_SET1 =>
        if word_op then
          if onchip_ram_access_enabled then
            return ea_clocks + 14 + (4 * wait_states);
          end if;
          return ea_clocks + 10 + (2 * wait_states);
        end if;
        if onchip_ram_access_enabled then
          return ea_clocks + 10 + (2 * wait_states);
        end if;
        return ea_clocks + 8 + wait_states;

      when V25_TIM_CLR1 =>
        if word_op then
          if onchip_ram_access_enabled then
            return ea_clocks + 15 + (4 * wait_states);
          end if;
          return ea_clocks + 10 + (2 * wait_states);
        end if;
        if onchip_ram_access_enabled then
          return ea_clocks + 11 + (2 * wait_states);
        end if;
        return ea_clocks + 9 + wait_states;
    end case;
  end function;

  function v25_clocks_bitop_mem_cl(
    op : v25_bit_timing_op_t;
    word_op : boolean;
    onchip_ram_access_enabled : boolean;
    ea_clocks : natural;
    wait_states : natural
  ) return natural is
  begin
    case op is
      when V25_TIM_TEST1 =>
        if word_op then
          return ea_clocks + 13 + (2 * wait_states);
        end if;
        return ea_clocks + 11 + wait_states;

      when V25_TIM_NOT1 | V25_TIM_SET1 =>
        if word_op then
          if onchip_ram_access_enabled then
            return ea_clocks + 17 + (4 * wait_states);
          end if;
          return ea_clocks + 13 + (2 * wait_states);
        end if;
        if onchip_ram_access_enabled then
          return ea_clocks + 13 + (2 * wait_states);
        end if;
        return ea_clocks + 11 + wait_states;

      when V25_TIM_CLR1 =>
        if word_op then
          if onchip_ram_access_enabled then
            return ea_clocks + 18 + (4 * wait_states);
          end if;
          return ea_clocks + 14 + (2 * wait_states);
        end if;
        if onchip_ram_access_enabled then
          return ea_clocks + 14 + (2 * wait_states);
        end if;
        return ea_clocks + 12 + wait_states;
    end case;
  end function;
end package body;
