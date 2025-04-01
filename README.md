STM32U031C8T6 sensor module for CottonCandy.

**Debugger Digikey Part #**: 497-STLINK-V3MINIE-ND

Possible Improvements:
1. Use STM32L1 for extended preferred on JLCPCB (alternatives may be possible to reduce assembly cost)
2. Change stlink connector (2x7) to 7 (2x1) 1.27mm pitch headeres so they can be assembled by JLC (2x5 pin interface is also possible)
3. SMD stlink connectors are easier to use (but more expensive)
4. Replace CO2 with a more accurate sensor (SCD40, library very likely available), can remove other sensors too.
5. Change order of CC header pins to match 2.0 interface
6. Add LSE for accurate RTC or LPUART functionality
7. Remove some breakout pins and replace with ready I2C + power for sensor connections
8. Modify interface so module can connect vertically to CottonCandy
9. ADC not usable (with 1/3 bridge) when tested using dry ice (-80C?)
