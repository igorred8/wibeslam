# ============================================================
#  Экстренный ремонт образа wibeslam:latest БЕЗ git и без
#  пересборки (PowerShell-версия fix_entrypoint.bat).
#  Запуск из папки wibeslam-docker:
#     powershell -ExecutionPolicy Bypass -File .\fix_entrypoint.ps1
#  Если есть Dockerfile/entrypoint.sh от актуального кода -
#  лучше просто пересобрать: .\build.bat
# ============================================================

$ErrorActionPreference = 'Continue'

docker --version > $null 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Docker not found or not running. Start Docker Desktop." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

docker image inspect wibeslam:latest > $null 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: image wibeslam:latest not found. Run build.bat first." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

if (-not (Test-Path "$PSScriptRoot\entrypoint_fallback.sh")) {
    Write-Host "ERROR: entrypoint_fallback.sh not found next to this script." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "Creating temp container from current image..."
docker rm -f wibeslam_fix > $null 2>&1
docker create --name wibeslam_fix --entrypoint sh wibeslam:latest > $null
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: docker create failed." -ForegroundColor Red
    Read-Host "Press Enter to exit"
    exit 1
}

Write-Host "Writing correct /entrypoint.sh into the container..."
docker cp "$PSScriptRoot\entrypoint_fallback.sh" wibeslam_fix:/entrypoint.sh
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: docker cp failed." -ForegroundColor Red
    docker rm -f wibeslam_fix > $null 2>&1
    Read-Host "Press Enter to exit"
    exit 1
}

# нормализация окончаний строк + синтаксическая проверка внутри контейнера
docker exec wibeslam_fix sed -i "s/\r$//" /entrypoint.sh
docker exec wibeslam_fix bash -n /entrypoint.sh
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: entrypoint.sh has syntax problems after copy." -ForegroundColor Red
    docker rm -f wibeslam_fix > $null 2>&1
    Read-Host "Press Enter to exit"
    exit 1
}
docker exec wibeslam_fix chmod +x /entrypoint.sh

Write-Host "Committing fixed image back to wibeslam:latest..."
docker commit --change='ENTRYPOINT ["/entrypoint.sh"]' wibeslam_fix wibeslam:latest
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: docker commit failed." -ForegroundColor Red
    docker rm -f wibeslam_fix > $null 2>&1
    Read-Host "Press Enter to exit"
    exit 1
}

docker rm -f wibeslam_fix > $null 2>&1

Write-Host ""
Write-Host "=== DONE: image repaired. Verify: ==="
docker run --rm --entrypoint ls wibeslam:latest -l /entrypoint.sh
if ($LASTEXITCODE -eq 0) {
    Write-Host "OK - now run run_wifeslam.bat and choose mode 4." -ForegroundColor Green
} else {
    Write-Host "Something is still wrong - rebuild with build.bat instead." -ForegroundColor Yellow
}
Read-Host "Press Enter to exit"
