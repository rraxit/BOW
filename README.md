# BOW Benchmark Dependencies Setup

This README provides step-by-step instructions for installing all required dependencies for the BOW (Bag of Words) benchmark project.

## Prerequisites

- Ubuntu 20.04+ or similar Debian-based distribution
- CMake 3.16+
- GCC 9+ (recommended: GCC 11+)
- Git

## Required Dependencies

The BOW benchmark project requires the following libraries:

1. **NLopt** - Nonlinear optimization library
2. **FCL** - Flexible Collision Library  
3. **OMPL** - Open Motion Planning Library
4. **Boost** - C++ libraries (specifically program_options component)
5. **YAML-CPP** - YAML parser for C++

## Installation Instructions

### 1. Update System Packages

```bash
sudo apt-get update
sudo apt-get upgrade
```

### 2. Install Build Tools

```bash
sudo apt-get install build-essential cmake git pkg-config
```

### 3. Install NLopt (Nonlinear Optimization Library)

```bash
# Install NLopt with C++ support
sudo apt-get install libnlopt-dev libnlopt-cxx-dev

# Verify installation
find /usr -name "nlopt.hpp" 2>/dev/null
```

**Alternative: Build from Source (if package doesn't include C++ headers)**
```bash
wget https://github.com/stevengj/nlopt/archive/v2.7.1.tar.gz
tar -xzf v2.7.1.tar.gz
cd nlopt-2.7.1
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DNLOPT_CXX=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 4. Install FCL (Flexible Collision Library)

```bash
# Install FCL
sudo apt-get install libfcl-dev

# Verify installation
find /usr -name "*fcl*config.cmake" 2>/dev/null
```

**Alternative: Build from Source**
```bash
# Install dependencies first
sudo apt-get install libeigen3-dev libccd-dev liboctomap-dev

# Build FCL
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 5. Install OMPL (Open Motion Planning Library)

```bash
# Install OMPL
sudo apt-get install libompl-dev

# Verify installation
find /usr -path "*/ompl/base/ProblemDefinition.h" 2>/dev/null
```

**Alternative: Build from Source**
```bash
# Install dependencies
sudo apt-get install libboost-all-dev libeigen3-dev libode-dev

# Build OMPL
git clone https://github.com/ompl/ompl.git
cd ompl
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 6. Install Boost Libraries

```bash
# Install all Boost development libraries
sudo apt-get install libboost-all-dev

# Or install specific components if space is a concern
sudo apt-get install libboost-program-options-dev libboost-system-dev libboost-filesystem-dev
```

### 7. Install YAML-CPP

```bash
# Install YAML-CPP
sudo apt-get install libyaml-cpp-dev

# Verify installation
find /usr -name "*yaml-cpp*" -type f 2>/dev/null | grep -E "\.(so|a)$"
```

### 8. Install Additional Dependencies (Optional)

```bash
# Python dependencies for visualization scripts
pip install fire matplotlib numpy

# Or if using conda
conda install fire matplotlib numpy
```

## Building the Project

After installing all dependencies:

```bash
# IMPORTANT: Set library path first (required if Anaconda is installed)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Clone your project (if not already done)
git clone <your-bow-benchmark-repo>
cd bow_benchmark

# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build the project
make -j$(nproc)

# Run the executables
./BowBenchmark
./BowPlannerRAL
```

## ⚠️ CRITICAL FIX: Anaconda Library Conflict

**If you have Anaconda installed, you MUST run this command before building or running the project:**

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

**Why this is needed:** Anaconda's older `libstdc++` conflicts with system libraries compiled with newer GCC versions, causing `GLIBCXX_3.4.32 not found` errors.

### Make the Fix Permanent

Add this line to your `~/.bashrc`:
```bash
echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Alternative: Per-Session Fix

Run this before each build/run session:
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
./BowBenchmark
./BowPlannerRAL
```

## Common Issues and Solutions

### Issue 1: GLIBCXX Version Mismatch (Anaconda Conflict)

**Problem:** `version 'GLIBCXX_3.4.32' not found` when Anaconda is installed.

**✅ SOLUTION:** Use the critical fix above:
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

### Issue 2: CMake Cannot Find Packages

**Solution:** Clean CMake cache and reconfigure:
```bash
rm -rf build/
mkdir build && cd build
cmake ..
```

### Issue 3: Missing Development Headers

**Problem:** Library found but headers missing.

**Solution:** Ensure `-dev` packages are installed:
```bash
sudo apt-get install libnlopt-dev libfcl-dev libompl-dev libboost-all-dev libyaml-cpp-dev
```

## Verification Commands

Use these commands to verify each dependency is properly installed:

```bash
# Check NLopt
pkg-config --exists nlopt && echo "NLopt found" || echo "NLopt missing"

# Check FCL
find /usr -name "fcl" -type d 2>/dev/null | head -1

# Check OMPL
find /usr -name "ompl" -type d 2>/dev/null | head -1

# Check Boost
dpkg -l | grep libboost-program-options-dev

# Check YAML-CPP
dpkg -l | grep libyaml-cpp-dev

# Check all library files
ldconfig -p | grep -E "(nlopt|fcl|ompl|boost|yaml)"
```

## Alternative Distributions

### Fedora/RHEL/CentOS

```bash
sudo dnf install nlopt-devel fcl-devel ompl-devel boost-devel yaml-cpp-devel
```

### Arch Linux

```bash
sudo pacman -S nlopt fcl ompl boost yaml-cpp
```

### macOS (Homebrew)

```bash
brew install nlopt fcl ompl boost yaml-cpp
```

## Troubleshooting

### If packages are not found in your distribution:

1. **Check available packages:**
   ```bash
   apt search nlopt
   apt search fcl
   apt search ompl
   ```

2. **Add additional repositories (Ubuntu):**
   ```bash
   sudo add-apt-repository universe
   sudo apt-get update
   ```

3. **Build from source** using the alternative instructions above.

### If CMake still cannot find libraries:

```bash
# Set CMAKE_PREFIX_PATH
cmake -DCMAKE_PREFIX_PATH=/usr/local ..

# Or specify individual library paths
cmake -DNLOPT_DIR=/usr/lib/x86_64-linux-gnu/cmake/nlopt \
      -DFCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/fcl \
      ..
```

## Environment Setup Script

Create a complete setup script that includes the critical fix:

```bash
cat > setup_environment.sh << 'EOF'
#!/bin/bash
echo "Setting up BOW Benchmark environment..."

# Update system
sudo apt-get update

# Install all dependencies
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    libnlopt-dev libnlopt-cxx-dev \
    libfcl-dev \
    libompl-dev \
    libboost-all-dev \
    libyaml-cpp-dev

# Update library cache
sudo ldconfig

# Add the critical library path fix to .bashrc (if not already present)
if ! grep -q "LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu" ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
    echo "Added library path fix to ~/.bashrc"
fi

echo "Environment setup complete!"
echo "IMPORTANT: Run 'source ~/.bashrc' or start a new terminal session"
echo "Then you can build the project with:"
echo "  mkdir build && cd build"
echo "  cmake .."
echo "  make -j\$(nproc)"
EOF

chmod +x setup_environment.sh
./setup_environment.sh
```

## Support

If you encounter issues not covered in this README:

1. Check that all dependencies are installed: `./verify_dependencies.sh`
2. Clean and rebuild: `rm -rf build && mkdir build && cd build && cmake ..`
3. Check for conflicting library versions (especially with Anaconda)
4. Verify GCC version compatibility: `gcc --version`

For additional help, please refer to the individual library documentation or create an issue in the project repository.
