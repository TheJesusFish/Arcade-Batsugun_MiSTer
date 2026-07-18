# Batsugun MiSTer Core

This is a vibe coded, MAME-based Batsugun arcade core for the MiSTer FPGA.
It is built on top of the excellent work from [Erin Olafsen](https://github.com/va7deo) (Toaplan Scaffolding) and [Pramod Somashakar](https://github.com/psomashekar) (GP9001 implementation).

**This core is vibe coded based on MAME. Its built binaries and MRAs live in the [Slop Core Repo](https://github.com/TheJesusFish/Slop-Core)**.

## Hardware Reference

MAME models Batsugun as:

- Motorola 68000 main CPU at 16 MHz.
- NEC V25 audio/control CPU at 16 MHz.
- Two GP9001 video devices clocked from 27 MHz.
- YM2151 at 27 MHz / 8.
- OKIM6295 at 32 MHz / 8, pin 7 low.
- Raster timing: 27 MHz / 4 pixel clock, 432 total horizontal clocks, 320 visible pixels, 262 total lines, 240 visible lines.


## Source Notes

- MiSTer framework and top-level structure:
  [MiSTer-devel/Main_MiSTer](https://github.com/MiSTer-devel/Main_MiSTer) and
  [Jotego jtcores / JTFrame](https://github.com/jotego/jtcores/tree/master/modules/jtframe)

- MC68000-compatible CPU core:
  [ijor/fx68k](https://github.com/ijor/fx68k)

- YM2151-compatible sound core:
  [Jotego jt51](https://github.com/jotego/jtcores/tree/master/modules/jt51)

- OKIM6295-compatible ADPCM sound core:
  [Jotego jt6295](https://github.com/jotego/jtcores/tree/master/modules/jt6295)

- Behavioral references:
  [MAME Toaplan `batsugun.cpp`](https://github.com/mamedev/mame/blob/master/src/mame/toaplan/batsugun.cpp),
  [MAME Toaplan `gp9001.cpp`](https://github.com/mamedev/mame/blob/master/src/mame/toaplan/gp9001.cpp), and
  [MAME Toaplan `gp9001.h`](https://github.com/mamedev/mame/blob/master/src/mame/toaplan/gp9001.h)

- NEC V25 core: Batsugun-local compact implementation, validated against MAME/FBNeo behavior.
