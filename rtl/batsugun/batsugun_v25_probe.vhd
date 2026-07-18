-- SPDX-License-Identifier: BSD-3-Clause
--
-- Clean-room Batsugun V25 integration probe.
-- This is a tiny synthetic uploaded-image harness based on the documented
-- Batsugun V25 memory map. It does not contain game program data.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.v25_pkg.all;

entity batsugun_v25_probe is
  port (
    clk     : in std_logic;
    reset_n : in std_logic;

    diag0 : out std_logic_vector(15 downto 0);
    diag1 : out std_logic_vector(15 downto 0);
    diag2 : out std_logic_vector(15 downto 0);
    diag3 : out std_logic_vector(15 downto 0);
    diag4 : out std_logic_vector(15 downto 0);
    diag5 : out std_logic_vector(15 downto 0);
    diag6 : out std_logic_vector(15 downto 0);
    diag7 : out std_logic_vector(15 downto 0)
  );
end entity;

architecture rtl of batsugun_v25_probe is
  constant RESET_VECTOR_OFFSET : natural := 16#7FF0#;
  constant PROGRAM_OFFSET : natural := 16#0020#;
  constant SHARED_MARK_OFFSET : natural := 16#0010#;

  constant YM2151_ADDR_PHYS : addr20_t := x"00000";
  constant YM2151_DATA_PHYS : addr20_t := x"00001";
  constant OKI6295_PHYS     : addr20_t := x"00004";
  constant SHARED_MARK_PHYS : addr20_t := x"80010";
  constant EXPECTED_RESET_PHYS : addr20_t := x"FFFF0";

  signal wait_states : unsigned(3 downto 0) := "0000";
  signal ready : std_logic := '1';
  signal hold_req : std_logic := '0';
  signal hold_ack : std_logic;

  signal bus_addr    : addr20_t;
  signal bus_dout    : byte_t;
  signal bus_din     : byte_t := x"00";
  signal bus_doe     : std_logic;
  signal bus_r_w     : std_logic;
  signal bus_mreq_n  : std_logic;
  signal bus_mstb_n  : std_logic;
  signal bus_iostb_n : std_logic;

  signal halted : std_logic;
  signal fault  : std_logic;
  signal debug_pc  : addr20_t;
  signal debug_psw : word_t;
  signal debug_ax  : word_t;
  signal debug_cx  : word_t;
  signal debug_dx  : word_t;
  signal debug_bx  : word_t;
  signal debug_sp  : word_t;

  signal reset_fetch_seen : std_logic := '0';
  signal reset_fetch_addr : addr20_t := (others => '0');
  signal reset_fetch_offset : unsigned(14 downto 0) := (others => '0');

  signal ym_addr_write_seen : std_logic := '0';
  signal ym_data_write_seen : std_logic := '0';
  signal oki_write_seen : std_logic := '0';
  signal shared_write_seen : std_logic := '0';
  signal ym_addr_data : byte_t := x"00";
  signal ym_data_data : byte_t := x"00";
  signal oki_data : byte_t := x"00";
  signal shared_data : byte_t := x"00";
  signal unmapped_write_seen : std_logic := '0';
  signal io_cycle_seen : std_logic := '0';

  signal reset_fetch_good : std_logic;
  signal synthetic_pass : std_logic;

  function is_shared_addr(addr : addr20_t) return boolean is
  begin
    return unsigned(addr) >= to_unsigned(16#80000#, addr'length);
  end function;

  function shared_offset(addr : addr20_t) return natural is
  begin
    return to_integer(unsigned(addr(14 downto 0)));
  end function;

  function synthetic_shared_byte(addr : addr20_t) return byte_t is
    variable offset : natural;
  begin
    offset := shared_offset(addr);

    case offset is
      -- Reset fetch at FFFF0h mirrors to shared RAM offset 7FF0h.
      when RESET_VECTOR_OFFSET + 0 => return x"EA"; -- JMP FAR 8000h:0020h.
      when RESET_VECTOR_OFFSET + 1 => return x"20";
      when RESET_VECTOR_OFFSET + 2 => return x"00";
      when RESET_VECTOR_OFFSET + 3 => return x"00";
      when RESET_VECTOR_OFFSET + 4 => return x"80";

      -- Synthetic uploaded program at physical 80020h/shared offset 0020h.
      when PROGRAM_OFFSET + 0  => return x"B8"; -- MOV AX,0000h.
      when PROGRAM_OFFSET + 1  => return x"00";
      when PROGRAM_OFFSET + 2  => return x"00";
      when PROGRAM_OFFSET + 3  => return x"8E"; -- MOV DS0,AX.
      when PROGRAM_OFFSET + 4  => return x"D8";
      when PROGRAM_OFFSET + 5  => return x"B0"; -- MOV AL,34h.
      when PROGRAM_OFFSET + 6  => return x"34";
      when PROGRAM_OFFSET + 7  => return x"A2"; -- MOV [0000h],AL.
      when PROGRAM_OFFSET + 8  => return x"00";
      when PROGRAM_OFFSET + 9  => return x"00";
      when PROGRAM_OFFSET + 10 => return x"B0"; -- MOV AL,56h.
      when PROGRAM_OFFSET + 11 => return x"56";
      when PROGRAM_OFFSET + 12 => return x"A2"; -- MOV [0001h],AL.
      when PROGRAM_OFFSET + 13 => return x"01";
      when PROGRAM_OFFSET + 14 => return x"00";
      when PROGRAM_OFFSET + 15 => return x"B0"; -- MOV AL,78h.
      when PROGRAM_OFFSET + 16 => return x"78";
      when PROGRAM_OFFSET + 17 => return x"A2"; -- MOV [0004h],AL.
      when PROGRAM_OFFSET + 18 => return x"04";
      when PROGRAM_OFFSET + 19 => return x"00";
      when PROGRAM_OFFSET + 20 => return x"B8"; -- MOV AX,8000h.
      when PROGRAM_OFFSET + 21 => return x"00";
      when PROGRAM_OFFSET + 22 => return x"80";
      when PROGRAM_OFFSET + 23 => return x"8E"; -- MOV DS0,AX.
      when PROGRAM_OFFSET + 24 => return x"D8";
      when PROGRAM_OFFSET + 25 => return x"B0"; -- MOV AL,A5h.
      when PROGRAM_OFFSET + 26 => return x"A5";
      when PROGRAM_OFFSET + 27 => return x"A2"; -- MOV [0010h],AL.
      when PROGRAM_OFFSET + 28 => return x"10";
      when PROGRAM_OFFSET + 29 => return x"00";
      when PROGRAM_OFFSET + 30 => return x"F4"; -- HLT.
      when others => return x"00";
    end case;
  end function;

  function batsugun_read_byte(
    addr : addr20_t;
    shared_mark_seen : std_logic;
    shared_mark_value : byte_t
  ) return byte_t is
  begin
    if is_shared_addr(addr) then
      if shared_offset(addr) = SHARED_MARK_OFFSET and shared_mark_seen = '1' then
        return shared_mark_value;
      else
        return synthetic_shared_byte(addr);
      end if;
    elsif addr = YM2151_ADDR_PHYS or addr = YM2151_DATA_PHYS then
      return x"00";
    elsif addr = OKI6295_PHYS then
      return x"00";
    else
      return x"00";
    end if;
  end function;
begin
  dut : entity work.v25_chip
    generic map (
      RESET_PS => x"FFFF",
      RESET_IP => x"0000",
      RESET_BANK => 7,
      ENABLE_TIMING_THROTTLE => true,
      MIN_WAIT_STATES => 1
    )
    port map (
      clk => clk,
      reset_n => reset_n,
      wait_states => wait_states,
      ready => ready,
      hold_req => hold_req,
      hold_ack => hold_ack,
      port0_in => x"FF",
      port1_in => x"FF",
      port2_in => x"FF",
      portt_in => x"FF",
      bus_addr => bus_addr,
      bus_dout => bus_dout,
      bus_din => bus_din,
      bus_doe => bus_doe,
      bus_r_w => bus_r_w,
      bus_mreq_n => bus_mreq_n,
      bus_mstb_n => bus_mstb_n,
      bus_iostb_n => bus_iostb_n,
      halted => halted,
      fault => fault,
      debug_pc => debug_pc,
      debug_psw => debug_psw,
      debug_ax => debug_ax,
      debug_cx => debug_cx,
      debug_dx => debug_dx,
      debug_bx => debug_bx,
      debug_sp => debug_sp
    );

  bus_read_model : process(bus_mreq_n, bus_mstb_n, bus_r_w, bus_iostb_n, bus_addr, shared_write_seen, shared_data)
  begin
    if bus_mreq_n = '0' and bus_mstb_n = '0' and bus_r_w = '1' then
      bus_din <= batsugun_read_byte(bus_addr, shared_write_seen, shared_data);
    elsif bus_iostb_n = '0' and bus_r_w = '1' then
      bus_din <= x"00";
    else
      bus_din <= x"00";
    end if;
  end process;

  bus_activity : process(clk)
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        reset_fetch_seen <= '0';
        reset_fetch_addr <= (others => '0');
        reset_fetch_offset <= (others => '0');
        ym_addr_write_seen <= '0';
        ym_data_write_seen <= '0';
        oki_write_seen <= '0';
        shared_write_seen <= '0';
        ym_addr_data <= x"00";
        ym_data_data <= x"00";
        oki_data <= x"00";
        shared_data <= x"00";
        unmapped_write_seen <= '0';
        io_cycle_seen <= '0';
      else
        if bus_mreq_n = '0' and bus_mstb_n = '0' and bus_r_w = '1' and
          reset_fetch_seen = '0' then
          reset_fetch_seen <= '1';
          reset_fetch_addr <= bus_addr;
          if is_shared_addr(bus_addr) then
            reset_fetch_offset <= unsigned(bus_addr(14 downto 0));
          end if;
        end if;

        if bus_iostb_n = '0' then
          io_cycle_seen <= '1';
        end if;

        if bus_mreq_n = '0' and bus_mstb_n = '0' and bus_r_w = '0' and
          bus_doe = '1' then
          if is_shared_addr(bus_addr) then
            if bus_addr = SHARED_MARK_PHYS then
              shared_write_seen <= '1';
              shared_data <= bus_dout;
            end if;
          elsif bus_addr = YM2151_ADDR_PHYS then
            ym_addr_write_seen <= '1';
            ym_addr_data <= bus_dout;
          elsif bus_addr = YM2151_DATA_PHYS then
            ym_data_write_seen <= '1';
            ym_data_data <= bus_dout;
          elsif bus_addr = OKI6295_PHYS then
            oki_write_seen <= '1';
            oki_data <= bus_dout;
          else
            unmapped_write_seen <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;

  reset_fetch_good <= '1' when reset_fetch_addr = EXPECTED_RESET_PHYS and
    reset_fetch_offset = to_unsigned(RESET_VECTOR_OFFSET, reset_fetch_offset'length) else '0';

  synthetic_pass <= reset_fetch_seen and reset_fetch_good and
    ym_addr_write_seen and ym_data_write_seen and oki_write_seen and
    shared_write_seen and halted and not fault and not unmapped_write_seen and
    not io_cycle_seen;

  diag0 <= reset_n & reset_fetch_seen & reset_fetch_good &
    ym_addr_write_seen & ym_data_write_seen & oki_write_seen &
    shared_write_seen & halted & fault & unmapped_write_seen &
    io_cycle_seen & synthetic_pass & bus_doe & bus_r_w &
    not bus_mreq_n & not bus_iostb_n;
  diag1 <= reset_fetch_addr(19 downto 4);
  diag2 <= reset_fetch_addr(3 downto 0) & std_logic_vector(reset_fetch_offset(11 downto 0));
  diag3 <= debug_pc(19 downto 4);
  diag4 <= debug_pc(3 downto 0) & debug_psw(11 downto 0);
  diag5 <= debug_ax;
  diag6 <= ym_addr_data & ym_data_data;
  diag7 <= oki_data & shared_data;
end architecture;
