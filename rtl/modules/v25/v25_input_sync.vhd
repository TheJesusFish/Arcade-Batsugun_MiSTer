-- SPDX-License-Identifier: BSD-3-Clause
--
-- Single-bit input synchronizer with one-cycle edge pulses for top-level pins.

library ieee;
use ieee.std_logic_1164.all;

entity v25_input_sync is
  generic (
    STAGES      : positive range 2 to 4 := 2;
    RESET_VALUE : std_logic := '0'
  );
  port (
    clk           : in  std_logic;
    reset_n       : in  std_logic;
    async_in      : in  std_logic;
    sync_out      : out std_logic;
    rising_pulse  : out std_logic;
    falling_pulse : out std_logic
  );
end entity;

architecture rtl of v25_input_sync is
  signal sync_pipe : std_logic_vector(STAGES - 1 downto 0) := (others => RESET_VALUE);
  signal sync_prev : std_logic := RESET_VALUE;
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if reset_n = '0' then
        sync_pipe <= (others => RESET_VALUE);
        sync_prev <= RESET_VALUE;
      else
        sync_prev <= sync_pipe(STAGES - 1);
        sync_pipe <= sync_pipe(STAGES - 2 downto 0) & async_in;
      end if;
    end if;
  end process;

  sync_out <= sync_pipe(STAGES - 1);
  rising_pulse <= sync_pipe(STAGES - 1) and not sync_prev;
  falling_pulse <= not sync_pipe(STAGES - 1) and sync_prev;
end architecture;
