# Test cibuildwheel locally (Windows only, current Python version)

Write-Host "Testing cibuildwheel locally..." -ForegroundColor Green

# Clean previous builds
Write-Host "Cleaning previous builds..." -ForegroundColor Yellow
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }

# For local testing, use conda's Boost (already installed)
# In CI, vcpkg will be used instead
$condaPrefix = $env:CONDA_PREFIX
if (-not $condaPrefix) {
    # Error:
    Write-Host "CONDA_PREFIX environment variable is not set. Please activate your conda environment." -ForegroundColor Red
    exit 1
}

$scriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
$condaBinPath = Join-Path $condaPrefix "Library\bin"
$normalizedCondaBin = $condaBinPath -replace '\\','/'
$buildReleaseDir = Join-Path (Join-Path $scriptRoot "build") "Release"
$normalizedBuildReleaseDir = $buildReleaseDir -replace '\\','/'
$addPathEntries = @($normalizedCondaBin, $normalizedBuildReleaseDir) | Where-Object { $_ -and $_.Trim() -ne "" }
$addPathArgument = [string]::Join(';', $addPathEntries)

# Run cibuildwheel
Write-Host "`nRunning cibuildwheel..." -ForegroundColor Green
cibuildwheel --platform windows

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild successful!" -ForegroundColor Green
    Write-Host "`nWheels built:" -ForegroundColor Cyan
    Get-ChildItem wheelhouse\*.whl | ForEach-Object {
        Write-Host "  $($_.Name)" -ForegroundColor Cyan
    }
} else {
    Write-Host "`nBuild failed!" -ForegroundColor Red
    exit 1
}
