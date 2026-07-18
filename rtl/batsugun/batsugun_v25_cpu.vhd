-- SPDX-License-Identifier: BSD-3-Clause
--
-- Small mixed-language boundary around the project V25 implementation.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.v25_pkg.all;

entity batsugun_v25_cpu is
  port (
    clk     : in std_logic;
    reset_n : in std_logic;
    clock_enable : in std_logic := '1';

    port0_in : in std_logic_vector(7 downto 0);
    port1_in : in std_logic_vector(7 downto 0);
    portt_in : in std_logic_vector(7 downto 0);

    bus_addr    : out std_logic_vector(19 downto 0);
    bus_dout    : out std_logic_vector(7 downto 0);
    bus_din     : in  std_logic_vector(7 downto 0);
    bus_doe     : out std_logic;
    bus_r_w     : out std_logic;
    bus_mreq_n  : out std_logic;
    bus_mstb_n  : out std_logic;
    bus_iostb_n : out std_logic;

    halted   : out std_logic;
    fault    : out std_logic;
    debug_pc : out std_logic_vector(19 downto 0)
  );
end entity;

architecture rtl of batsugun_v25_cpu is
begin
  u_cpu : entity work.v25_chip
    generic map (
      RESET_PS => x"FFFF",
      RESET_IP => x"0000",
      RESET_BANK => 7,
      ENABLE_TIMING_THROTTLE => false,
      FIXED_INSTRUCTION_BUDGET => 8,
      ENABLE_TIMER_PRESCALER => true,
      MIN_WAIT_STATES => 1
    )
    port map (
      clk => clk,
      reset_n => reset_n,
      clock_enable => clock_enable,
      wait_states => to_unsigned(0, 4),
      ready => '1',
      hold_req => '0',
      port0_in => port0_in,
      port1_in => port1_in,
      port2_in => x"FF",
      portt_in => portt_in,
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
      debug_psw => open,
      debug_ax => open,
      debug_cx => open,
      debug_dx => open,
      debug_bx => open,
      debug_sp => open
    );
end architecture;
