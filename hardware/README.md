# Hardware

The KiCad design and manufacturing outputs for the ecoTrace Communication Module
(Rev A, 2026-04-23) are included here so you don't need Drive access.

```
hardware/
  kicad/         KiCad project - open Communication_pcb_main.kicad_pro in KiCad
    Communication_pcb_main.kicad_pro / .kicad_pcb / .kicad_sch
    mcu.kicad_sch  power_mgmt.kicad_sch  simcom.kicad_sch   (schematic sheets)
    jlcpcb/        fab outputs: gerbers, drill, BOM, CPL, GERBER-*.zip
  exports/       viewable without KiCad:
    Communication_pcb_main-schematic.pdf
    board-top.png  board-bottom.png   (3D renders)
```

## No KiCad? Start here

- **Schematic:** `exports/Communication_pcb_main-schematic.pdf`. Sheets: `mcu`,
  `power_mgmt`, `simcom`. The **power_mgmt** sheet (BQ25792) is the one to know: it
  defines the charging path, VSYS, the NTC (TS) network (note the errata), and the
  ACDRV FET gating for USB vs solar inputs.
- **Board:** `exports/board-top.png` / `board-bottom.png`.

## Fabrication

`kicad/jlcpcb/` holds a ready-to-order set (gerbers, drill, BOM, CPL, and a zipped
gerber bundle) targeted at JLCPCB. Re-export from KiCad if you change the board.

Note: there is also a 4-layer variant of this board on the shared drive
(`Communication_pcb_main _4l`); only the 2-layer Rev A design is included here.
