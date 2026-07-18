-- SPDX-License-Identifier: BSD-3-Clause
--
-- Async-assert, sync-release reset helper for MiSTer/top-level integration.

library ieee;
use ieee.std_logic_1164.all;

entity v25_reset_sync is
  generic (
    STAGES : positive range 2 to 4 := 2
  );
  port (
    clk           : in  std_logic;
    reset_async_n : in  std_logic;
    reset_sync_n  : out std_logic
  );
end entity;

architecture rtl of v25_reset_sync is
  signal sync_pipe : std_logic_vector(STAGES - 1 downto 0) := (others => '0');
begin
  process(clk, reset_async_n)
  begin
    if reset_async_n = '0' then
      sync_pipe <= (others => '0');
    elsif rising_edge(clk) then
      sync_pipe <= sync_pipe(STAGES - 2 downto 0) & '1';
    end if;
  end process;

  reset_sync_n <= sync_pipe(STAGES - 1);
end architecture;
