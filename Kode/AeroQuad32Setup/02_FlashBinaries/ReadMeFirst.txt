Before flashing the AQ32 board with new firmware, verify that reset.bat is configured correctly.  Edit the first line to indicate the COM port assigned to your AQ32 board.

Procedure to flash a binary to your AQ32 board using the Windows Command Prompt:
1) Run reset.bat to put AQ32 in DFUSe mode.
2) (Optional) Run ListDFUSeDevices.bat to verify that the AQ32 is in DFUSe mode.
3) Run FlashDFUSeF4.bat <filename of binary> to upload new firmware to the AQ32 board
4) Hit the reset button on the AeroQuad32 board
5) Run Configurator (finish calibrations, then power cycle board)

For example to upload the default binary included in this installer, run this from the command prompt:
FlashDFUSeF4.bat AeroQuad32.noMag

There is an additional test binary included which was used to verify proper operation of the board before shipping.  It is included for your convenience for any future troubleshooting as required.