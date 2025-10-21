# Test cibuildwheel locally (Windows only, current Python version)

Write-Host "Testing cibuildwheel locally..." -ForegroundColor Green

# Clean previous builds
Write-Host "Cleaning previous builds..." -ForegroundColor Yellow
if (Test-Path "wheelhouse") { Remove-Item -Recurse -Force "wheelhouse" }
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }

# Set environment variables for cibuildwheel
$env:CIBW_BUILD = "cp312-*"  # Only build for Python 3.12 (current version)

# For local testing, use conda's Boost (already installed)
# In CI, vcpkg will be used instead
$condaPrefix = $env:CONDA_PREFIX
if (-not $condaPrefix) {
    $condaPrefix = "C:/Users/ko/miniforge3/envs/pyfmm"
}

$scriptRoot = if ($PSScriptRoot) { $PSScriptRoot } else { (Get-Location).Path }
$condaBinPath = Join-Path $condaPrefix "Library\bin"
$normalizedCondaBin = $condaBinPath -replace '\\','/'
$buildReleaseDir = Join-Path (Join-Path $scriptRoot "build") "Release"
$normalizedBuildReleaseDir = $buildReleaseDir -replace '\\','/'
$addPathEntries = @($normalizedCondaBin, $normalizedBuildReleaseDir) | Where-Object { $_ -and $_.Trim() -ne "" }
$addPathArgument = [string]::Join(';', $addPathEntries)

# Install build dependencies
$env:CIBW_BEFORE_BUILD_WINDOWS = "pip install scikit-build-core setuptools-scm pybind11 delvewheel"

# Set Boost paths for CMake to use conda's Boost
$env:CIBW_ENVIRONMENT_WINDOWS = "BOOST_ROOT=$condaPrefix/Library"

# Repair Windows wheels to bundle DLL dependencies
$env:CIBW_REPAIR_WHEEL_COMMAND_WINDOWS = "delvewheel repair -w {dest_dir} {wheel} --add-path `"$addPathArgument`""

# Build verbosity
$env:CIBW_BUILD_VERBOSITY = "1"

# Skip PyPy, musllinux, and 32-bit builds (we only have 64-bit Boost)
$env:CIBW_SKIP = "pp* *-musllinux_* *-win32"

# Test the wheel after building
$env:CIBW_TEST_COMMAND = "python -c `"import fmm; print('fmm version:', fmm.__version__); net = fmm.Network(); print('Network created successfully!')`""

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
