-- SPDX-License-Identifier: BSD-3-Clause
--
-- Shared types and helpers for the experimental NEC V25-compatible core.
-- This is a from-scratch hardware model implemented from NEC documentation;
-- emulator sources are kept for cosimulation/trace comparison only.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package v25_pkg is
  subtype byte_t is std_logic_vector(7 downto 0);
  subtype word_t is std_logic_vector(15 downto 0);
  subtype addr20_t is std_logic_vector(19 downto 0);

  constant V25_REG_AX : natural := 0;
  constant V25_REG_CX : natural := 1;
  constant V25_REG_DX : natural := 2;
  constant V25_REG_BX : natural := 3;
  constant V25_REG_SP : natural := 4;
  constant V25_REG_BP : natural := 5;
  constant V25_REG_SI : natural := 6;
  constant V25_REG_DI : natural := 7;

  type v25_word_regs_t is array (0 to 7) of word_t;
  type v25_bank_regs_t is array (0 to 7) of v25_word_regs_t;

  type v25_flags_t is record
    cf    : std_logic;
    ibrk  : std_logic;
    pf    : std_logic;
    f0    : std_logic;
    af    : std_logic;
    f1    : std_logic;
    zf    : std_logic;
    sf    : std_logic;
    tf    : std_logic;
    iflag : std_logic;
    df    : std_logic;
    oflag : std_logic;
    rb    : std_logic_vector(2 downto 0);
    mf    : std_logic;
  end record;

  constant V25_RESET_FLAGS : v25_flags_t := (
    cf    => '0',
    ibrk  => '1',
    pf    => '0',
    f0    => '0',
    af    => '0',
    f1    => '0',
    zf    => '0',
    sf    => '0',
    tf    => '0',
    iflag => '0',
    df    => '0',
    oflag => '0',
    rb    => "111",
    mf    => '1'
  );

  function v25_phys_addr(seg : word_t; off : word_t) return addr20_t;
  function v25_get_reg8(regs : v25_word_regs_t; idx : natural) return byte_t;
  function v25_set_reg8(regs : v25_word_regs_t; idx : natural; value : byte_t)
    return v25_word_regs_t;

  function v25_pack_psw(flags : v25_flags_t) return word_t;
  function v25_unpack_psw(value : word_t) return v25_flags_t;
  function v25_update_flags_from_ah(old_flags : v25_flags_t; value : byte_t) return v25_flags_t;
  function v25_even_parity8(value : byte_t) return std_logic;
  function v25_sign_extend8(value : byte_t) return word_t;
  function v25_modrm_is_memory(modrm : byte_t) return boolean;
  function v25_modrm_disp_size(modrm : byte_t) return natural;
  function v25_modrm_ea(regs : v25_word_regs_t; modrm : byte_t; disp : word_t) return word_t;
  function v25_modrm_default_seg(ss : word_t; ds0 : word_t; modrm : byte_t) return word_t;

  function v25_logic_flags8(old_flags : v25_flags_t; value : byte_t) return v25_flags_t;
  function v25_logic_flags16(old_flags : v25_flags_t; value : word_t) return v25_flags_t;
  function v25_add_flags8(old_flags : v25_flags_t; a : byte_t; b : byte_t; carry_in : std_logic)
    return v25_flags_t;
  function v25_add_flags16(old_flags : v25_flags_t; a : word_t; b : word_t; carry_in : std_logic)
    return v25_flags_t;
  function v25_sub_flags8(old_flags : v25_flags_t; a : byte_t; b : byte_t; borrow_in : std_logic)
    return v25_flags_t;
  function v25_sub_flags16(old_flags : v25_flags_t; a : word_t; b : word_t; borrow_in : std_logic)
    return v25_flags_t;
  function v25_jcc_taken(cc : std_logic_vector(3 downto 0); flags : v25_flags_t) return boolean;
end package;

package body v25_pkg is
  function v25_phys_addr(seg : word_t; off : word_t) return addr20_t is
    variable base : unsigned(19 downto 0);
    variable ext_off : unsigned(19 downto 0);
  begin
    base := shift_left(resize(unsigned(seg), 20), 4);
    ext_off := resize(unsigned(off), 20);
    return std_logic_vector(base + ext_off);
  end function;

  function v25_get_reg8(regs : v25_word_regs_t; idx : natural) return byte_t is
  begin
    case idx is
      when 0 => return regs(V25_REG_AX)(7 downto 0);
      when 1 => return regs(V25_REG_CX)(7 downto 0);
      when 2 => return regs(V25_REG_DX)(7 downto 0);
      when 3 => return regs(V25_REG_BX)(7 downto 0);
      when 4 => return regs(V25_REG_AX)(15 downto 8);
      when 5 => return regs(V25_REG_CX)(15 downto 8);
      when 6 => return regs(V25_REG_DX)(15 downto 8);
      when others => return regs(V25_REG_BX)(15 downto 8);
    end case;
  end function;

  function v25_set_reg8(regs : v25_word_regs_t; idx : natural; value : byte_t)
    return v25_word_regs_t is
    variable r : v25_word_regs_t := regs;
  begin
    case idx is
      when 0 => r(V25_REG_AX)(7 downto 0) := value;
      when 1 => r(V25_REG_CX)(7 downto 0) := value;
      when 2 => r(V25_REG_DX)(7 downto 0) := value;
      when 3 => r(V25_REG_BX)(7 downto 0) := value;
      when 4 => r(V25_REG_AX)(15 downto 8) := value;
      when 5 => r(V25_REG_CX)(15 downto 8) := value;
      when 6 => r(V25_REG_DX)(15 downto 8) := value;
      when others => r(V25_REG_BX)(15 downto 8) := value;
    end case;
    return r;
  end function;

  function v25_pack_psw(flags : v25_flags_t) return word_t is
    variable psw : word_t := (others => '0');
  begin
    psw(0) := flags.cf;
    psw(1) := flags.ibrk;
    psw(2) := flags.pf;
    psw(3) := flags.f0;
    psw(4) := flags.af;
    psw(5) := flags.f1;
    psw(6) := flags.zf;
    psw(7) := flags.sf;
    psw(8) := flags.tf;
    psw(9) := flags.iflag;
    psw(10) := flags.df;
    psw(11) := flags.oflag;
    psw(14 downto 12) := flags.rb;
    psw(15) := flags.mf;
    return psw;
  end function;

  function v25_unpack_psw(value : word_t) return v25_flags_t is
    variable f : v25_flags_t;
  begin
    f.cf    := value(0);
    f.ibrk  := value(1);
    f.pf    := value(2);
    f.f0    := value(3);
    f.af    := value(4);
    f.f1    := value(5);
    f.zf    := value(6);
    f.sf    := value(7);
    f.tf    := value(8);
    f.iflag := value(9);
    f.df    := value(10);
    f.oflag := value(11);
    f.rb    := value(14 downto 12);
    f.mf    := value(15);
    return f;
  end function;

  function v25_update_flags_from_ah(old_flags : v25_flags_t; value : byte_t) return v25_flags_t is
    variable f : v25_flags_t := old_flags;
  begin
    f.cf := value(0);
    f.ibrk := value(1);
    f.pf := value(2);
    f.f0 := value(3);
    f.af := value(4);
    f.f1 := value(5);
    f.zf := value(6);
    f.sf := value(7);
    return f;
  end function;

  function v25_even_parity8(value : byte_t) return std_logic is
    variable ones : natural := 0;
  begin
    for i in value'range loop
      if value(i) = '1' then
        ones := ones + 1;
      end if;
    end loop;

    if (ones mod 2) = 0 then
      return '1';
    end if;
    return '0';
  end function;

  function v25_sign_extend8(value : byte_t) return word_t is
  begin
    return std_logic_vector(resize(signed(value), 16));
  end function;

  function v25_modrm_is_memory(modrm : byte_t) return boolean is
  begin
    return modrm(7 downto 6) /= "11";
  end function;

  function v25_modrm_disp_size(modrm : byte_t) return natural is
  begin
    if modrm(7 downto 6) = "01" then
      return 1;
    elsif modrm(7 downto 6) = "10" then
      return 2;
    elsif modrm(7 downto 6) = "00" and modrm(2 downto 0) = "110" then
      return 2;
    end if;

    return 0;
  end function;

  function v25_modrm_ea(regs : v25_word_regs_t; modrm : byte_t; disp : word_t) return word_t is
    variable base : unsigned(15 downto 0) := (others => '0');
    variable displacement : unsigned(15 downto 0) := (others => '0');
  begin
    case modrm(2 downto 0) is
      when "000" => base := unsigned(regs(V25_REG_BX)) + unsigned(regs(V25_REG_SI));
      when "001" => base := unsigned(regs(V25_REG_BX)) + unsigned(regs(V25_REG_DI));
      when "010" => base := unsigned(regs(V25_REG_BP)) + unsigned(regs(V25_REG_SI));
      when "011" => base := unsigned(regs(V25_REG_BP)) + unsigned(regs(V25_REG_DI));
      when "100" => base := unsigned(regs(V25_REG_SI));
      when "101" => base := unsigned(regs(V25_REG_DI));
      when "110" =>
        if modrm(7 downto 6) = "00" then
          base := (others => '0');
        else
          base := unsigned(regs(V25_REG_BP));
        end if;
      when others => base := unsigned(regs(V25_REG_BX));
    end case;

    if modrm(7 downto 6) = "01" then
      displacement := unsigned(v25_sign_extend8(disp(7 downto 0)));
    elsif modrm(7 downto 6) = "10" or
          (modrm(7 downto 6) = "00" and modrm(2 downto 0) = "110") then
      displacement := unsigned(disp);
    end if;

    return std_logic_vector(base + displacement);
  end function;

  function v25_modrm_default_seg(ss : word_t; ds0 : word_t; modrm : byte_t) return word_t is
  begin
    if modrm(2 downto 0) = "010" or modrm(2 downto 0) = "011" then
      return ss;
    elsif modrm(2 downto 0) = "110" and modrm(7 downto 6) /= "00" then
      return ss;
    end if;

    return ds0;
  end function;

  function v25_logic_flags8(old_flags : v25_flags_t; value : byte_t) return v25_flags_t is
    variable f : v25_flags_t := old_flags;
  begin
    f.cf := '0';
    f.oflag := '0';
    f.af := '0';
    f.sf := value(7);
    if value = x"00" then
      f.zf := '1';
    else
      f.zf := '0';
    end if;
    f.pf := v25_even_parity8(value);
    return f;
  end function;

  function v25_logic_flags16(old_flags : v25_flags_t; value : word_t) return v25_flags_t is
    variable f : v25_flags_t := old_flags;
  begin
    f.cf := '0';
    f.oflag := '0';
    f.af := '0';
    f.sf := value(15);
    if value = x"0000" then
      f.zf := '1';
    else
      f.zf := '0';
    end if;
    f.pf := v25_even_parity8(value(7 downto 0));
    return f;
  end function;

  function v25_add_flags8(old_flags : v25_flags_t; a : byte_t; b : byte_t; carry_in : std_logic)
    return v25_flags_t is
    variable f : v25_flags_t := old_flags;
    variable ci : unsigned(8 downto 0) := (others => '0');
    variable au : unsigned(8 downto 0);
    variable bu : unsigned(8 downto 0);
    variable sum : unsigned(8 downto 0);
    variable res : byte_t;
  begin
    ci(0) := carry_in;
    au := resize(unsigned(a), 9);
    bu := resize(unsigned(b), 9);
    sum := au + bu + ci;
    res := std_logic_vector(sum(7 downto 0));

    f.cf := sum(8);
    f.oflag := (not (a(7) xor b(7))) and (a(7) xor res(7));
    f.af := a(4) xor b(4) xor res(4);
    f.sf := res(7);
    if res = x"00" then
      f.zf := '1';
    else
      f.zf := '0';
    end if;
    f.pf := v25_even_parity8(res);
    return f;
  end function;

  function v25_add_flags16(old_flags : v25_flags_t; a : word_t; b : word_t; carry_in : std_logic)
    return v25_flags_t is
    variable f : v25_flags_t := old_flags;
    variable ci : unsigned(16 downto 0) := (others => '0');
    variable au : unsigned(16 downto 0);
    variable bu : unsigned(16 downto 0);
    variable sum : unsigned(16 downto 0);
    variable res : word_t;
  begin
    ci(0) := carry_in;
    au := resize(unsigned(a), 17);
    bu := resize(unsigned(b), 17);
    sum := au + bu + ci;
    res := std_logic_vector(sum(15 downto 0));

    f.cf := sum(16);
    f.oflag := (not (a(15) xor b(15))) and (a(15) xor res(15));
    f.af := a(4) xor b(4) xor res(4);
    f.sf := res(15);
    if res = x"0000" then
      f.zf := '1';
    else
      f.zf := '0';
    end if;
    f.pf := v25_even_parity8(res(7 downto 0));
    return f;
  end function;

  function v25_sub_flags8(old_flags : v25_flags_t; a : byte_t; b : byte_t; borrow_in : std_logic)
    return v25_flags_t is
    variable f : v25_flags_t := old_flags;
    variable bi : unsigned(8 downto 0) := (others => '0');
    variable au : unsigned(8 downto 0);
    variable bu : unsigned(8 downto 0);
    variable subtrahend : unsigned(8 downto 0);
    variable diff : unsigned(8 downto 0);
    variable res : byte_t;
  begin
    bi(0) := borrow_in;
    au := resize(unsigned(a), 9);
    bu := resize(unsigned(b), 9);
    subtrahend := bu + bi;
    diff := au - subtrahend;
    res := std_logic_vector(diff(7 downto 0));

    if au < subtrahend then
      f.cf := '1';
    else
      f.cf := '0';
    end if;
    f.oflag := (a(7) xor b(7)) and (a(7) xor res(7));
    f.af := a(4) xor b(4) xor res(4);
    f.sf := res(7);
    if res = x"00" then
      f.zf := '1';
    else
      f.zf := '0';
    end if;
    f.pf := v25_even_parity8(res);
    return f;
  end function;

  function v25_sub_flags16(old_flags : v25_flags_t; a : word_t; b : word_t; borrow_in : std_logic)
    return v25_flags_t is
    variable f : v25_flags_t := old_flags;
    variable bi : unsigned(16 downto 0) := (others => '0');
    variable au : unsigned(16 downto 0);
    variable bu : unsigned(16 downto 0);
    variable subtrahend : unsigned(16 downto 0);
    variable diff : unsigned(16 downto 0);
    variable res : word_t;
  begin
    bi(0) := borrow_in;
    au := resize(unsigned(a), 17);
    bu := resize(unsigned(b), 17);
    subtrahend := bu + bi;
    diff := au - subtrahend;
    res := std_logic_vector(diff(15 downto 0));

    if au < subtrahend then
      f.cf := '1';
    else
      f.cf := '0';
    end if;
    f.oflag := (a(15) xor b(15)) and (a(15) xor res(15));
    f.af := a(4) xor b(4) xor res(4);
    f.sf := res(15);
    if res = x"0000" then
      f.zf := '1';
    else
      f.zf := '0';
    end if;
    f.pf := v25_even_parity8(res(7 downto 0));
    return f;
  end function;

  function v25_jcc_taken(cc : std_logic_vector(3 downto 0); flags : v25_flags_t) return boolean is
    variable sign_ne_over : boolean;
  begin
    sign_ne_over := flags.sf /= flags.oflag;

    case cc is
      when x"0" => return flags.oflag = '1';
      when x"1" => return flags.oflag = '0';
      when x"2" => return flags.cf = '1';
      when x"3" => return flags.cf = '0';
      when x"4" => return flags.zf = '1';
      when x"5" => return flags.zf = '0';
      when x"6" => return flags.cf = '1' or flags.zf = '1';
      when x"7" => return flags.cf = '0' and flags.zf = '0';
      when x"8" => return flags.sf = '1';
      when x"9" => return flags.sf = '0';
      when x"A" => return flags.pf = '1';
      when x"B" => return flags.pf = '0';
      when x"C" => return sign_ne_over;
      when x"D" => return not sign_ne_over;
      when x"E" => return sign_ne_over or flags.zf = '1';
      when others => return (not sign_ne_over) and flags.zf = '0';
    end case;
  end function;
end package body;
