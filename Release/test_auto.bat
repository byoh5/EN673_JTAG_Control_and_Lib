@echo on

echo 첫번째 인수: %1

echo 두번째 인수: %2

echo 모든 인수: %*

set DGNUM=%1

echo DG%DGNUM%

:LOOP
@set YEAR=%date:~0,4%
@set MONTH=%date:~5,2%
@set DAY=%date:~8,2%
@set HOUR=%time:~0,2%
@set MINUTE=%time:~3,2%
@set SECOND=%time:~6,2%
@set POSTFIX=%YEAR%-%MONTH%-%DAY%_%HOUR%-%MINUTE%
echo %DGNUM%
mkdir "DG%DGNUM%" 
mkdir "DG%DGNUM%"/"%POSTFIX%" 
set RDM=%RANDOM%
JtagControl.exe "DG%DGNUM% A" si 100 "DG%DGNUM%"/"%POSTFIX%"/"%HOUR%-%MINUTE%-%SECOND%_%RDM%.txt" 
echo %ERRORLEVEL%



if %ERRORLEVEL% == 0 goto LVL0
if %ERRORLEVEL% == 1 goto LVL1
if %ERRORLEVEL% == 2 goto LVL2
if %ERRORLEVEL% == 3 goto LVL3
if %ERRORLEVEL% == 4 goto LVL4
if %ERRORLEVEL% == 5 goto LVL5

REM goto LOOP

:LVL0
echo LVL0 is activated

@echo off

goto LOOP

:LVL1
echo LVL1 is activated
EXIT

:LVL2
echo LVL2 is activated
EXIT

:LVL3
echo LVL3 is activated
EXIT

:LVL4
echo LVL4 is activated
EXIT

:LVL5
echo LVL5 is activated
EXIT
