@echo off
echo =========================================
echo    UPLOAD RAPIDO ESP32 - ROBOT SUMO
echo =========================================
echo.
echo ⚡ Subiendo firmware al ESP32...
echo ⚠️  Si falla: BOOT + RESET en el ESP32
echo.

C:\Users\kalex\.platformio\penv\Scripts\python.exe C:\Users\kalex\.platformio\packages\tool-esptoolpy\esptool.py --chip esp32 --port COM5 --baud 115200 write_flash 0x10000 .pio\build\esp32dev\firmware.bin

echo.
echo ✅ UPLOAD COMPLETADO
echo 🤖 Robot Sumo actualizado en ESP32
echo.
echo Si hubo error arriba:
echo 1. Desconecta ESP32 del USB
echo 2. Reconecta ESP32 al USB  
echo 3. Manten BOOT presionado
echo 4. Presiona y suelta RESET
echo 5. Suelta BOOT  
echo 6. Ejecuta: upload_rapido.bat

echo.
pause