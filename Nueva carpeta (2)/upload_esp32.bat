@echo off
echo =========================================
echo    UPLOAD ESP32 - ROBOT SUMO
echo =========================================
echo.
echo Compilando el proyecto...
echo.

REM Compilar el proyecto primero
C:\Users\kalex\.platformio\penv\Scripts\platformio.exe run

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ❌ ERROR: Fallo en la compilacion
    echo Revisa el codigo antes de continuar
    pause
    exit /b 1
)

echo.
echo ✅ Compilacion exitosa!
echo.
echo Subiendo firmware al ESP32...
echo ⚠️  IMPORTANTE: Si falla, presiona BOOT + RESET en el ESP32
echo.

REM Subir usando esptool directo (metodo que funciona)
C:\Users\kalex\.platformio\penv\Scripts\python.exe C:\Users\kalex\.platformio\packages\tool-esptoolpy\esptool.py --chip esp32 --port COM5 --baud 115200 write_flash 0x10000 .pio\build\esp32dev\firmware.bin

echo.
echo ✅ UPLOAD COMPLETADO  
echo 🤖 Robot Sumo cargado en ESP32
echo ⚡ El ESP32 esta listo para funcionar
echo.
echo Si hubo error arriba:
echo 1. Desconecta el ESP32 del USB
echo 2. Reconecta el ESP32 al USB  
echo 3. Manten presionado BOOT
echo 4. Presiona y suelta RESET
echo 5. Suelta BOOT
echo 6. Ejecuta este script inmediatamente

echo.
echo Presiona cualquier tecla para continuar...
pause >nul