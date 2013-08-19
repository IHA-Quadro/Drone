set port=Com5
mode %port%:RTS=off 
mode %port%:DTR=off 
mode %port%:DTR=on
mode %port%:DTR=off 
echo 1EAF >%port%:
pause
