@ECHO OFF
SET arduino_path=%programfiles(x86)%\Arduino

IF EXIST "%arduino_path%" (
	ECHO "%arduino_path%"
) ELSE (
	EXIT /b 1
)
