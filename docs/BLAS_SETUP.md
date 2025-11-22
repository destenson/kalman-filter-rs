# BLAS Backend Setup Guide

This document explains how to configure and use BLAS acceleration with the kalman-filters library.

## Matrix Backend Options

The library supports multiple matrix operation backends:

1. **nalgebra** (default) - Pure Rust, good performance, no external dependencies
2. **naive-matrix** - Simple pure Rust implementation, no dependencies
3. **blas-backend** - Uses ndarray without BLAS (pure Rust)
4. **ndarray-linalg** - Uses ndarray with BLAS acceleration (requires BLAS libraries)

## Using BLAS Acceleration

To use BLAS acceleration, you need to:

1. Choose a BLAS implementation
2. Install the necessary system libraries (if using system BLAS)
3. Enable the appropriate Cargo features

### BLAS Implementation Options

#### OpenBLAS (Recommended for Linux)

**System OpenBLAS** (uses your installed OpenBLAS):
```bash
# Install system libraries (Ubuntu/Debian)
sudo apt-get install libopenblas-dev liblapack-dev

# Build with system OpenBLAS
cargo build --features "ndarray-linalg,openblas-system"
cargo test --features "ndarray-linalg,openblas-system"
```

**Static OpenBLAS** (builds from source):
```bash
# No system libraries required
cargo build --features "ndarray-linalg,openblas-static"
```

#### Intel MKL (Best performance on Intel CPUs)

```bash
# Dynamic linking (requires Intel MKL installed)
cargo build --features "ndarray-linalg,intel-mkl-dynamic-lp64"

# Static linking
cargo build --features "ndarray-linalg,intel-mkl-static-lp64"
```

Available Intel MKL options:
- `intel-mkl-dynamic-lp64` - Dynamic,  parallel threading
- `intel-mkl-dynamic-seq` - Dynamic, sequential (single-threaded)
- `intel-mkl-static-lp64` - Static, parallel threading
- `intel-mkl-static-seq` - Static, sequential

#### Apple Accelerate (macOS only)

```bash
cargo build --features "ndarray-linalg,accelerate"
```

#### Other Options

- `netlib` - Reference BLAS implementation
- `blis` - BLIS BLAS implementation

## Troubleshooting

### Linker Errors with System BLAS

If you get undefined symbol errors like `cblas_dgemm` or `dpotrf_`, the linker cannot find your BLAS libraries. Try:

1. **Verify libraries are installed**:
   ```bash
   pkg-config --libs blas lapack
   # Should output something like: -lblas -llapack
   ```

2. **Set PKG_CONFIG_PATH** (if libraries are in non-standard location):
   ```bash
   export PKG_CONFIG_PATH=/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
   ```

3. **Use static build** instead:
   ```bash
   cargo build --features "ndarray-linalg,openblas-static"
   ```

### Performance Considerations

- **openblas-system/openblas-static**: 9-13x faster than naive for 50x50-100x100 matrices
- **intel-mkl**: Often fastest on Intel CPUs, especially for larger matrices
- **accelerate**: Best on macOS, well-optimized for Apple Silicon
- **nalgebra**: Good balance of performance and ease of use (no system dependencies)

### Feature Combinations

You can combine features:
```bash
# Multiple backends with development features
cargo build --features "nalgebra,blas-backend,serde,parallel"

# All features (not recommended - conflicts possible)
cargo build --all-features
```

**Important**: Only enable ONE BLAS backend at a time (e.g., don't combine `openblas-system` and `intel-mkl-dynamic`).

## Environment-Specific Notes

### Ubuntu/Debian
```bash
sudo apt-get install libopenblas-dev liblapack-dev
cargo build --features "ndarray-linalg,openblas-system"
```

### Fedora/RHEL
```bash
sudo dnf install openblas-devel lapack-devel
cargo build --features "ndarray-linalg,openblas-system"
```

### macOS
```bash
# Use Accelerate (built-in, no installation needed)
cargo build --features "ndarray-linalg,accelerate"

# Or install OpenBLAS via Homebrew
brew install openblas
export PKG_CONFIG_PATH="/opt/homebrew/opt/openblas/lib/pkgconfig"
cargo build --features "ndarray-linalg,openblas-system"
```

### Windows
Use static linking or MKL:
```bash
cargo build --features "ndarray-linalg,openblas-static"
# or
cargo build --features "ndarray-linalg,intel-mkl-static-lp64"
```

## Testing

To test the matrix backends:

```bash
# Test default (nalgebra)
cargo test --lib matrix::

# Test ndarray without BLAS
cargo test --features "blas-backend,std" --no-default-features --lib matrix::

# Test with OpenBLAS
cargo test --features "ndarray-linalg,openblas-system" --lib matrix::

# Test naive implementation
cargo test --features "naive-matrix,std" --no-default-features --lib matrix::
```

## References

- [blas-src documentation](https://docs.rs/blas-src/)
- [ndarray-linalg documentation](https://docs.rs/ndarray-linalg/)
- [OpenBLAS website](https://www.openblas.net/)
- [Intel MKL](https://software.intel.com/content/www/us/en/develop/tools/oneapi/components/onemkl.html)
