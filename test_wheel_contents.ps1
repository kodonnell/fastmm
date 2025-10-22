# Test script to build wheel and inspect contents

Write-Host "Building wheel locally..." -ForegroundColor Green

# Clean previous builds
if (Test-Path "dist") { Remove-Item -Recurse -Force "dist" }
if (Test-Path "build") { Remove-Item -Recurse -Force "build" }
if (Test-Path "_skbuild") { Remove-Item -Recurse -Force "_skbuild" }

# Build the wheel
Write-Host "`nBuilding wheel with pip..." -ForegroundColor Yellow
pip wheel . --no-deps -w dist

# Check if wheel was created
$wheel = Get-ChildItem -Path "dist" -Filter "*.whl" | Select-Object -First 1

if ($wheel) {
    Write-Host "`nWheel created: $($wheel.Name)" -ForegroundColor Green
    
    # Extract and inspect wheel contents
    Write-Host "`nExtracting wheel contents..." -ForegroundColor Yellow
    $extractDir = "dist\wheel_contents"
    if (Test-Path $extractDir) { Remove-Item -Recurse -Force $extractDir }
    New-Item -ItemType Directory -Path $extractDir | Out-Null
    
    # Unzip the wheel (wheels are just zip files)
    Expand-Archive -Path $wheel.FullName -DestinationPath $extractDir
    
    Write-Host "`nWheel contents:" -ForegroundColor Cyan
    Get-ChildItem -Path $extractDir -Recurse | ForEach-Object {
        $relativePath = $_.FullName.Substring($extractDir.Length + 1)
        if ($_.PSIsContainer) {
            Write-Host "  [DIR]  $relativePath" -ForegroundColor Blue
        } else {
            $size = "{0:N0}" -f $_.Length
            Write-Host "  [FILE] $relativePath ($size bytes)" -ForegroundColor White
        }
    }
    
    # Check specifically for DLL files
    Write-Host "`nDLL files in wheel:" -ForegroundColor Cyan
    $dlls = Get-ChildItem -Path $extractDir -Filter "*.dll" -Recurse
    if ($dlls) {
        foreach ($dll in $dlls) {
            $relativePath = $dll.FullName.Substring($extractDir.Length + 1)
            Write-Host "  ✓ $relativePath" -ForegroundColor Green
        }
    } else {
        Write-Host "  ✗ No DLL files found!" -ForegroundColor Red
    }
    
    # Check for .pyd files
    Write-Host "`n.pyd files in wheel:" -ForegroundColor Cyan
    $pyds = Get-ChildItem -Path $extractDir -Filter "*.pyd" -Recurse
    if ($pyds) {
        foreach ($pyd in $pyds) {
            $relativePath = $pyd.FullName.Substring($extractDir.Length + 1)
            Write-Host "  ✓ $relativePath" -ForegroundColor Green
        }
    } else {
        Write-Host "  ✗ No .pyd files found!" -ForegroundColor Red
    }
    
} else {
    Write-Host "`nFailed to create wheel!" -ForegroundColor Red
}
