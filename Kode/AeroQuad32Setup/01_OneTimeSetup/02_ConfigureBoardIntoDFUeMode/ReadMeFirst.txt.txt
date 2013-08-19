The reset.bat file will put your AQ32 board into DFUSe mode which will allow you to upload new firmware.

Before running this batch file, edit it (right click on file and select edit) and update the first line with the COM port that was assigned to it when you installed the VCP USB drivers.

Please note, that due to batch file limitations, it may not work if the COM port has double digits in it.  If you encounter problems, use the Device Manager (Go to start menu, right click on Computer->Properties->Devicem Manager->Ports) to change the COM port to a value of 9 or less.