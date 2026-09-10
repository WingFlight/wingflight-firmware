# Board documentation

This directory used to hold per-board chapters inherited from Cleanflight: F1 and F3
boards such as the Naze32, SPRacingF3 and CC3D, plus a handful of Betaflight-era F4/F7
racing boards. None of them is a Wingflight target, so those chapters were removed.

Wingflight builds for four boards only, all Matek wing controllers:

| Target      | MCU       |
| ----------- | --------- |
| `MATEKF405` | STM32F405 |
| `MATEKF411` | STM32F411 |
| `MATEKF722` | STM32F722 |
| `MATEKH743` | STM32H743 |

The board summary, with identifiers, sensors, UARTs and timer outputs for each, is in
[../Boards.md](../Boards.md). The authoritative per-board description is the target
directory itself: `src/main/target/<TARGET>/target.h`, `target.c` and `target.mk`.

There is deliberately no per-board prose here. The board drawing, silkscreen labels and
pad positions that the Wingflight Configurator shows in the Setup journey live in the
configurator repository, in `src/tabs/journey/board_profiles.json`. To add or correct a
board there, follow the contributor guide `docs/adding-a-board-profile.md` in the
configurator repository.
