# Simple local build test using conda's Boost

Write-Host "Testing local build with conda Boost..." -ForegroundColor Green

# Try to activate conda environment
if (-not $env:CONDA_PREFIX) {
    Write-Host "Attempting to activate pyfmm conda environment..." -ForegroundColor Yellow
    & conda activate pyfmm 2>$null
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Could not activate conda environment. Please run: conda activate pyfmm" -ForegroundColor Red
        exit 1
    }
}

Write-Host "Using conda environment: $env:CONDA_PREFIX" -ForegroundColor Cyan

# Clean previous builds
Write-Host "`nCleaning previous builds..." -ForegroundColor Yellow
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }
if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }

# Set environment to use conda's Boost
$env:BOOST_ROOT = Join-Path $env:CONDA_PREFIX "Library"
Write-Host "BOOST_ROOT set to: $env:BOOST_ROOT" -ForegroundColor Cyan

# Build with pip
Write-Host "`nBuilding wheel..." -ForegroundColor Yellow
pip wheel . --no-deps -w dist -v

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild successful!" -ForegroundColor Green
    
    # Check for wheel
    $wheel = Get-ChildItem -Path "dist" -Filter "*.whl" -ErrorAction SilentlyContinue | Select-Object -First 1
    
    if ($wheel) {
        Write-Host "Wheel created: $($wheel.Name)" -ForegroundColor Green
        
        # Extract and inspect
        Write-Host "`nExtracting wheel..." -ForegroundColor Yellow
        $extractDir = "dist\wheel_contents"
        if (Test-Path $extractDir) { Remove-Item -Recurse -Force $extractDir }
        New-Item -ItemType Directory -Path $extractDir | Out-Null
        
        Expand-Archive -Path $wheel.FullName -DestinationPath $extractDir
        
        Write-Host "`nFiles in fastmm package:" -ForegroundColor Cyan
        $fastmmDir = Join-Path $extractDir "fastmm"
        if (Test-Path $fastmmDir) {
            Get-ChildItem -Path $fastmmDir -File | ForEach-Object {
                $size = "{0:N0}" -f $_.Length
                Write-Host "  $($_.Name) ($size bytes)" -ForegroundColor White
            }
            
            # Check for DLL
            Write-Host "`nChecking for DLLs:" -ForegroundColor Cyan
            $dlls = Get-ChildItem -Path $fastmmDir -Filter "*.dll"
            if ($dlls) {
                foreach ($dll in $dlls) {
                    Write-Host "  ✓ Found: $($dll.Name)" -ForegroundColor Green
                }
            } else {
                Write-Host "  ✗ No DLL files found in fastmm directory!" -ForegroundColor Red
            }
            
            # Check for .pyd
            Write-Host "`nChecking for .pyd files:" -ForegroundColor Cyan
            $pyds = Get-ChildItem -Path $fastmmDir -Filter "*.pyd"
            if ($pyds) {
                foreach ($pyd in $pyds) {
                    Write-Host "  ✓ Found: $($pyd.Name)" -ForegroundColor Green
                }
            } else {
                Write-Host "  ✗ No .pyd files found!" -ForegroundColor Red
            }
        } else {
            Write-Host "  ✗ fastmm directory not found in wheel!" -ForegroundColor Red
        }
    }
} else {
    Write-Host "`nBuild failed!" -ForegroundColor Red
}
