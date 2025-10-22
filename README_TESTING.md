# Testing cibuildwheel locally

## Prerequisites

- Docker installed and running
- Python 3.8+ with pip

## Quick start

### Test with a single Python version (faster)

```bash
chmod +x test_cibuildwheel_single.sh
./test_cibuildwheel_single.sh
```

This builds only Python 3.11 wheels for faster iteration during development.

### Test with all Python versions

```bash
chmod +x test_cibuildwheel.sh
./test_cibuildwheel.sh
```

This builds wheels for all configured Python versions (3.8-3.12).

## Testing on different platforms

### Linux (using Docker)

```bash
# Install cibuildwheel
pip install cibuildwheel

# Build Linux wheels
cibuildwheel --platform linux

# Build for specific Python version only
CIBW_BUILD="cp311-*" cibuildwheel --platform linux
```

### macOS

**Note**: Building macOS wheels on Windows via Docker is not reliable. It's recommended to:
- Test macOS builds on GitHub Actions (uses native macOS runners)
- Or test on an actual macOS machine

If you're on macOS:
```bash
# Install cibuildwheel
pip install cibuildwheel

# Build macOS wheels
cibuildwheel --platform macos

# Build for specific Python version only
CIBW_BUILD="cp311-*" cibuildwheel --platform macos
```

### Windows

```powershell
# Install cibuildwheel
pip install cibuildwheel

# Build Windows wheels
cibuildwheel --platform windows

# Build for specific Python version only
$env:CIBW_BUILD="cp311-*"
cibuildwheel --platform windows
```

## Configuration

All cibuildwheel configuration is in `pyproject.toml` under `[tool.cibuildwheel]`.

You can override settings via environment variables:

```bash
# Build only Python 3.11
export CIBW_BUILD="cp311-*"

# Increase verbosity
export CIBW_BUILD_VERBOSITY=3

# Skip tests
export CIBW_TEST_COMMAND=""

# Run cibuildwheel
cibuildwheel --platform linux
```

## Troubleshooting

### Docker not running

```
Error: Docker is not running
```

**Solution**: Start Docker Desktop or the Docker daemon.

### Permission denied

```
bash: ./test_cibuildwheel.sh: Permission denied
```

**Solution**: Make the script executable:
```bash
chmod +x test_cibuildwheel.sh
```

### Build takes too long

**Solution**: Use the single Python version script or set `CIBW_BUILD`:
```bash
CIBW_BUILD="cp311-*" ./test_cibuildwheel.sh
```

### Boost installation fails

Check that:
- On Linux: Docker has internet access
- On macOS: Homebrew is installed
- On Windows: vcpkg is available at `C:/vcpkg`

## Output

Built wheels will be in `./wheelhouse/`:

```bash
ls -lh ./wheelhouse/
```

## Testing the built wheel

```bash
# Create a test environment
python3 -m venv test_env
source test_env/bin/activate  # On Windows: test_env\Scripts\activate

# Install the wheel
pip install ./wheelhouse/fastmm-*.whl

# Test import
python -c "import fastmm; print(fastmm.__version__)"

# Deactivate
deactivate
```
