# Power states

The board's intended power/charging states are documented in the diagram
[`ecoTrace_power_states.pdf`](ecoTrace_power_states.pdf) in this folder.

Key ideas (see the PDF for the authoritative version):

- The **BQ25792** is the PMIC: it manages input selection (USB / VBUS via VAC1, solar
  via VAC2), battery charging, and the regulated system rail **VSYS** that powers the
  ESP32-C6 and peripherals.
- Deep-sleep duty cycling (see `src/datalogger/`) is how the logger keeps average
  current low: the ESP32 sleeps most of the time, waking to sample and - less often -
  to power the modem and transmit.
- The modem rail (GPIO23) and external sensor rail (GPIO14) are switched off and held
  off during sleep so they don't dominate the sleep budget.

> TODO: transcribe the state diagram into text/mermaid here so it is searchable and
> reviewable in-repo, not only as a PDF.
