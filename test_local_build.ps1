# Test local build with VS 2026 (without cibuildwheel)
# Usage:
#   .\test_local_build.ps1              # Incremental build (fast, skips reinstall)
#   .\test_local_build.ps1 -Install     # Build and install wheel
#   .\test_local_build.ps1 -Clean       # Clean build artifacts
#   .\test_local_build.ps1 -CleanAll    # Clean everything including cache

param(
    [switch]$Install,    # Install the wheel after building
    [switch]$Clean,      # Clean build artifacts (build, _skbuild, dist)
    [switch]$CleanAll    # Clean everything including cache
)

Write-Host "Testing local build with VS 2026..." -ForegroundColor Green

# Check conda environment
$condaPrefix = $env:CONDA_PREFIX
if (-not $condaPrefix) {
    Write-Host "CONDA_PREFIX environment variable is not set. Please activate your conda environment." -ForegroundColor Red
    exit 1
}

# Clean if requested
if ($CleanAll) {
    Write-Host "Cleaning all build artifacts and cache..." -ForegroundColor Yellow
    if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
    if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }
    if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }
    if (Test-Path "cache") { Remove-Item -Recurse -Force "cache" }
} elseif ($Clean) {
    Write-Host "Cleaning build artifacts..." -ForegroundColor Yellow
    if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
    if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }
    if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }
} else {
    Write-Host "Using incremental build (use -Clean or -CleanAll to rebuild)" -ForegroundColor Cyan
}

# Initialize VS 2026 environment
Write-Host "Initializing VS 2026 environment..." -ForegroundColor Yellow
$vsPath = "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat"
if (Test-Path $vsPath) {
    cmd /c "`"$vsPath`" x64 && set" | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') {
            [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2])
        }
    }
    Write-Host "VS 2026 environment initialized" -ForegroundColor Green
} else {
    Write-Host "Warning: VS 2026 not found at expected location" -ForegroundColor Yellow
}

# Build wheel using pip (--no-build-isolation to skip reinstalling build deps)
Write-Host "`nBuilding wheel..." -ForegroundColor Green
python -m pip wheel . --no-deps --no-build-isolation -w dist -v

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild successful!" -ForegroundColor Green
    Write-Host "`nWheels built:" -ForegroundColor Cyan
    Get-ChildItem dist\*.whl | ForEach-Object {
        Write-Host "  $($_.Name)" -ForegroundColor Cyan
    }

    if ($Install) {
        # Install wheel (skip dependencies to save time)
        Write-Host "`nInstalling wheel..." -ForegroundColor Green
        $wheel = Get-ChildItem dist\*.whl | Sort-Object LastWriteTime -Descending | Select-Object -First 1
        pip install --force-reinstall --no-deps $wheel.FullName
        python -c "import fastmm; print('fastmm version:', fastmm.__version__)"
    } else {
        Write-Host "`nSkipping installation (use -Install flag to install)" -ForegroundColor Cyan
    }
} else {
    Write-Host "`nBuild failed!" -ForegroundColor Red
    exit 1
}
