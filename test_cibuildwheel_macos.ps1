# Test cibuildwheel for macOS locally using Docker
# Note: This simulates macOS builds on Windows using Docker

Write-Host "=== Testing cibuildwheel for macOS ===" -ForegroundColor Green
Write-Host ""

# Check if Docker is running
try {
    docker info | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "Docker check failed"
    }
} catch {
    Write-Host "Error: Docker is not running. Please start Docker Desktop and try again." -ForegroundColor Red
    exit 1
}

# Check if cibuildwheel is installed
if (-not (Get-Command cibuildwheel -ErrorAction SilentlyContinue)) {
    Write-Host "Installing cibuildwheel..." -ForegroundColor Yellow
    pip install cibuildwheel
}

# Clean previous builds
Write-Host "Cleaning previous builds..." -ForegroundColor Yellow
if (Test-Path "wheelhouse") { Remove-Item -Recurse -Force "wheelhouse" }

# Build only Python 3.11 for faster testing (optional)
# Uncomment to test with just one Python version:
$env:CIBW_BUILD = "cp311-*"

# Enable verbose output for debugging
$env:CIBW_BUILD_VERBOSITY = "3"

Write-Host "Starting cibuildwheel build for macOS (Python 3.11 only)..." -ForegroundColor Green
Write-Host "This will use Docker to simulate macOS environment." -ForegroundColor Yellow
Write-Host ""

# Run cibuildwheel for macOS with error capture
Write-Host "Running: cibuildwheel --platform macos" -ForegroundColor Cyan
cibuildwheel --platform macos 2>&1 | Tee-Object -FilePath "cibuildwheel_macos.log"

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "=== Build complete ===" -ForegroundColor Green
    Write-Host "Wheels are in: ./wheelhouse/" -ForegroundColor Cyan
    Get-ChildItem wheelhouse\*.whl | ForEach-Object {
        Write-Host "  $($_.Name)" -ForegroundColor Cyan
    }
} else {
    Write-Host ""
    Write-Host "=== Build failed ===" -ForegroundColor Red
    Write-Host ""
    Write-Host "Full log saved to: cibuildwheel_macos.log" -ForegroundColor Yellow
    Write-Host "To view the error:" -ForegroundColor Yellow
    Write-Host "  Get-Content cibuildwheel_macos.log -Tail 50" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Last 30 lines of output:" -ForegroundColor Yellow
    Get-Content cibuildwheel_macos.log -Tail 30
    exit 1
}
