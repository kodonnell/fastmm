# Build wheel script for Windows
# This script builds the C++ extension and creates a distributable wheel

$ErrorActionPreference = "Stop"

Write-Host "Building FMM wheel..." -ForegroundColor Green

# Clean previous builds
if (Test-Path "build") {
    Write-Host "Cleaning previous build..." -ForegroundColor Yellow
    Remove-Item -Path "build" -Recurse -Force
}
if (Test-Path "dist") {
    Remove-Item -Path "dist" -Recurse -Force
}
# Clean Python build artifacts
Remove-Item -Path "python\fmm\_version.py" -Force -ErrorAction SilentlyContinue
Remove-Item -Path "python\fmm.egg-info" -Recurse -Force -ErrorAction SilentlyContinue

# Create build directory
New-Item -ItemType Directory -Path "build" -Force | Out-Null

# Configure CMake
Write-Host "Configuring CMake..." -ForegroundColor Green
Push-Location build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -ne 0) {
    Pop-Location
    throw "CMake configuration failed"
}

# Build C++ extension
Write-Host "Building C++ extension..." -ForegroundColor Green
cmake --build . --config Release --parallel
if ($LASTEXITCODE -ne 0) {
    Pop-Location
    throw "C++ build failed"
}
Pop-Location

# Copy built binaries to package
Write-Host "Copying binaries to package..." -ForegroundColor Green
New-Item -ItemType Directory -Path "python\fmm" -Force | Out-Null
Copy-Item -Path "build\python\pybind11\Release\fmm.pyd" -Destination "python\fmm\" -Force
Copy-Item -Path "build\Release\FMMLIB.dll" -Destination "python\fmm\" -Force

# Build wheel (setuptools-scm will auto-generate version from pyproject.toml config)
Write-Host "Building wheel..." -ForegroundColor Green
python -m build --wheel --outdir dist/
if ($LASTEXITCODE -ne 0) {
    throw "Wheel build failed"
}

# List built wheels
Write-Host "`nBuilt wheels:" -ForegroundColor Green
Get-ChildItem -Path "dist" -Filter "*.whl" | ForEach-Object {
    Write-Host "  $($_.Name)" -ForegroundColor Cyan
}

Write-Host "`nBuild complete!" -ForegroundColor Green
Write-Host "Install with: pip install dist\<wheel-name>.whl" -ForegroundColor Yellow
