# OB_ROBOT project rules

- Keep `Core/Src/main.c` minimal: hardware initialization, module initialization, and non-blocking module calls only.
- Never call `HAL_Delay()` or use blocking wait loops inside `while (1)` or application state machines.
- Drive 10 ms, 100 ms, timeout, sampling, and control scheduling from hardware timer interrupts and interrupt-produced ticks/flags.
- Keep interrupt handlers bounded: perform time-critical sampling/control or set flags only. Do not refresh the OLED, format strings, print, debounce buttons, or run long business logic in an ISR.
- Implement UI, button handling, motion state machines, and diagnostics as non-blocking module functions outside `main.c`.
