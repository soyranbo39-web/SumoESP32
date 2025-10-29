# Script para abrir carpeta con archivos de firmware ESP32
# Ejecutar este script para abrir la carpeta con los archivos .bin

$firmwarePath = "C:\Users\kalex\OneDrive\Desktop\Nueva carpeta (2)\.pio\build\esp32dev"

Write-Host "Abriendo carpeta con archivos de firmware ESP32..." -ForegroundColor Green
Write-Host "Ruta: $firmwarePath" -ForegroundColor Yellow

# Verificar que la carpeta existe
if (Test-Path $firmwarePath) {
    # Abrir el explorador de archivos en la carpeta
    Invoke-Item $firmwarePath
    
    Write-Host "`nArchivos disponibles:" -ForegroundColor Cyan
    Get-ChildItem $firmwarePath -Filter "*.bin" | ForEach-Object {
        $size = [math]::Round($_.Length / 1KB, 2)
        Write-Host "  - $($_.Name) ($size KB)" -ForegroundColor White
    }
    
    Write-Host "`nUsa estos archivos en ESP32 Flash Download Tool:" -ForegroundColor Green
    Write-Host "  bootloader.bin  -> 0x1000" -ForegroundColor Yellow
    Write-Host "  partitions.bin  -> 0x8000" -ForegroundColor Yellow  
    Write-Host "  firmware.bin    -> 0x10000" -ForegroundColor Yellow
} else {
    Write-Host "Error: No se encontró la carpeta de firmware" -ForegroundColor Red
    Write-Host "Ruta esperada: $firmwarePath" -ForegroundColor Yellow
}

Write-Host "`nPresiona cualquier tecla para continuar..." -ForegroundColor Gray
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")