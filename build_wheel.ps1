# Build script for FASTMM Windows wheel using scikit-build-core

# Error handling
$ErrorActionPreference = "Stop"

Write-Host "Building FASTMM wheel..." -ForegroundColor Green

# Clean previous builds
Write-Host "Cleaning previous build artifacts..." -ForegroundColor Yellow
if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }
if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }

Write-Host "Building wheel with scikit-build-core..." -ForegroundColor Yellow

# Install build dependencies if not already installed
python -m pip install --quiet --upgrade build scikit-build-core setuptools-scm pybind11

# Build the wheel using python -m build
# scikit-build-core handles CMake configuration and build automatically
python -m build --wheel

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed!" -ForegroundColor Red
    exit 1
}

Write-Host "`nBuilt wheels:" -ForegroundColor Green
Get-ChildItem -Path "dist\*.whl" | ForEach-Object {
    Write-Host "  $($_.Name)" -ForegroundColor Cyan
}

Write-Host "`nBuild complete!" -ForegroundColor Green
Write-Host "Install with: pip install dist\<wheel-name>.whl" -ForegroundColor Yellow
