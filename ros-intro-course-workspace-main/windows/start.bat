@echo off

docker container prune --force --filter "label=source=ros-course"

:: Reference https://stackoverflow.com/a/43076919
docker image inspect ros-course >nul 2>nul

if NOT %ERRORLEVEL% == 0 (
    echo Docker image ros-course not found. Will now build it...
    docker build --no-cache -t ros-course ../docker/
) else echo Existing Docker image found!


:: Get network interface name to use for display, thanks to https://stackoverflow.com/a/61936361
Set "IntName="&For /F "Tokens=1,2,4" %%G In (
    '%__APPDIR__%ROUTE.EXE -4 PRINT^|%__APPDIR__%findstr.exe /RC:"0.0.0.0\ *0.0.0.0"'
) Do (Set "_=%%I"&For /F Tokens^=1-2Delims^=^" %%J In (
        '%__APPDIR__%cmd.exe /D/V/C "%__APPDIR__%netsh.exe interface IP show config|%__APPDIR__%findstr.exe "\" !_:.=\.!""'
    ) Do (Call Set "$=%%_%%"&Set "_=%%K"
        Echo(%%J|%__APPDIR__%find.exe "%%I">NUL&&(Call Set "IntName=%%$%%"&GoTo Next)))
:Next
:: Get IP for that interface, thanks to https://stackoverflow.com/questions/5898763/how-do-i-get-the-ip-address-into-a-batch-file-variable
for /f "tokens=3 delims=: " %%i  in ('netsh interface ip show config name^="%IntName%" ^| findstr "IP Address" ^| findstr [0-9]') do set IP=%%i

docker run -it --net=host --env "DISPLAY=%IP%:0.0" --name=ros-course --volume="%cd%/../workspace":"/root/catkin_ws/src/":rw ros-course bash