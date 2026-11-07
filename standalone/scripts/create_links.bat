@echo off

::
:: Creates links to binaries on Windows desktop
::

cd /d "%~dp0"

pushd ..
if not exist data mkdir data
popd

call .mk_link.bat dmm-Start dmm-start.exe
call .mk_link.bat dmm-plot  dmm-plot.exe
call .mk_link.bat dmm-hist  dmm-hist.exe
call .mk_link.bat dmm-stat  dmm-stat.exe
call .mk_link.bat dmm-data  data

::
:: Preload on startup helps to improve next app start time
::
call .mk_link.bat dmm-preload dmm-start.exe "--preload --quit" "%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup"
