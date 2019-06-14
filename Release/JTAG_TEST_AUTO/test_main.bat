@echo on
JtagControl.exe "DG1 A" getdevnum
set num=%ERRORLEVEL%
echo %num% 
for /l %%i in (1,1,%num%) do start CALL test_auto.bat %%i DEVICE%%i