@echo off
set /a x=0

:pyscript
cd C:\Users\LattePanda\Desktop\LB680A_Win64_script
python LB680A_ArduPilot.py

if %x% lss 3 (
	echo %x%
	timeout /t 10 /nobreak
	set /a x+=1
	goto :pyscript
)

echo PyDLLError
pause