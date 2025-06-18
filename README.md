# BOW Benchmark - Motion Planning Algorithms Comparison

This repository provides rigorous benchmarking for **BOW (Bayesian Optimization Approach to Windowing Motion Planning with Constrained Satisfaction)** against state-of-the-art motion planning algorithms.

🌐 **Project Website:** [bow-web.github.io](https://bow-web.github.io)  
📄 **Paper and Experiments:** Visit our website for detailed results and publications

## Overview

The BOW benchmark implements and compares multiple motion planning algorithms across both C++ and Python implementations:

### C++ Implementation (4 Planners)
- **BOW** - Bayesian Optimization Approach to Windowing Motion Planning
- **RRT** - Rapidly-exploring Random Trees
- **HRVO** - Hybrid Reciprocal Velocity Obstacles
- **DWA** - Dynamic Window Approach

### Python Implementation (2 Planners)
- **MPPI** - Model Predictive Path Integral
- **CBF** - Control Barrier Functions

## Quick Start

### For C++ Development
```bash
# Clone the repository
git clone <your-bow-benchmark-repo>
cd bow_benchmark

# Run the setup script
chmod +x setup_environment.sh
./setup_environment.sh

# Build the project
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run benchmarks
./BowBenchmark  # Google Benchmark file
./BowPlannerRAL
```

### For Python Development
```bash
# Create Anaconda environment
conda create -n bow python=3.10
conda activate bow

# Install dependencies
pip install -r requirements.txt

# Run Python benchmarks
./run_py.sh
```

---

## C++ Setup Instructions

### Prerequisites

- Ubuntu 20.04+ or similar Debian-based distribution
- CMake 3.16+
- GCC 9+ (recommended: GCC 11+)
- Git

### Required Dependencies

The C++ implementation requires the following libraries:

1. **NLopt** - Nonlinear optimization library
2. **FCL** - Flexible Collision Library  
3. **OMPL** - Open Motion Planning Library
4. **Boost** - C++ libraries (specifically program_options component)
5. **YAML-CPP** - YAML parser for C++

### Step-by-Step Installation

#### 1. Update System Packages

```bash
sudo apt-get update
sudo apt-get upgrade
```

#### 2. Install Build Tools

```bash
sudo apt-get install build-essential cmake git pkg-config
```

#### 3. Install NLopt (Nonlinear Optimization Library)

```bash
# Install NLopt with C++ support
sudo apt-get install libnlopt-dev libnlopt-cxx-dev

# Verify installation
find /usr -name "nlopt.hpp" 2>/dev/null
```

**Alternative: Build from Source**
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

#### 4. Install FCL (Flexible Collision Library)

```bash
# Install FCL
sudo apt-get install libfcl-dev

# Alternative: Build from Source
# sudo apt-get install libeigen3-dev libccd-dev liboctomap-dev
# git clone https://github.com/flexible-collision-library/fcl.git
```

#### 5. Install OMPL (Open Motion Planning Library)

```bash
# Install OMPL
sudo apt-get install libompl-dev

# Verify installation
find /usr -path "*/ompl/base/ProblemDefinition.h" 2>/dev/null
```

#### 6. Install Boost Libraries

```bash
# Install all Boost development libraries
sudo apt-get install libboost-all-dev
```

#### 7. Install YAML-CPP

```bash
# Install YAML-CPP
sudo apt-get install libyaml-cpp-dev
```

### ⚠️ CRITICAL: Anaconda Library Conflict Fix

**If you have Anaconda installed, you MUST run this before building:**

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

**Make it permanent:**
```bash
echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Building the C++ Project

```bash
# Set library path (if Anaconda is installed)
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run executables
cd /home/airlab/Cppdev/bow_cpp_py/build/
./BowPlannerRAL   # Main benchmark application

# Run Google Benchmark tests
cd benchmark/
./BowBenchmark    # All benchmark tests
./BowBenchmark --benchmark_filter=BOW  # BOW-specific benchmarks
```

---

## Python Setup Instructions

### Environment Setup

We recommend using Anaconda for Python development:

```bash
# Create and activate environment
conda create -n bow python=3.10
conda activate bow
```

### Dependencies Installation

```bash
# Install Python dependencies
pip install -r requirements.txt
```

### Python Planners Integration

The Python implementation integrates the following external libraries:

- **MPPI**: [mppi_playground](https://github.com/kohonda/mppi_playground.git)
- **CBF**: [cbfToolbox](https://github.com/mit-ll-trusted-autonomy/cbfToolbox.git)

### Running Python Benchmarks

```bash
# Activate environment
conda activate bow

# Method 1: Run complete benchmark suite
./run_py.sh

# Method 2: Configure and run custom experiments
# Edit parameters in config/main.yaml to customize:
# - Planning algorithms to test
# - Environment scenarios
# - Performance metrics
# - Simulation parameters
nano config/main.yaml  # or use your preferred editor

# Then run with custom configuration
python main.py

# Method 3: Run individual planner benchmarks
python benchmark_mppi.py
python benchmark_cbf.py
```

---

## Project Structure

```
bow_benchmark/
├── src/                    # C++ source files
│   ├── planners/
│   │   ├── bow/           # BOW planner implementation
│   │   ├── rrt/           # RRT planner implementation
│   │   ├── hrvo/          # HRVO planner implementation
│   │   └── dwa/           # DWA planner implementation
│   └── benchmark/         # Benchmarking utilities
├── python/                # Python implementations
│   ├── planners/
│   │   ├── mppi/          # MPPI planner
│   │   └── cbf/           # CBF planner
│   ├── benchmarks/        # Python benchmark scripts
│   └── config/            # Configuration files
│       └── main.yaml      # Main configuration parameters
├── build/                 # C++ build directory
│   └── benchmark/         # Google Benchmark executables location
├── requirements.txt       # Python dependencies
├── setup_environment.sh   # C++ setup script
├── run_py.sh             # Python benchmark runner
├── main.py               # Python main execution script
└── README.md             # This file
```

## Benchmarking

### C++ Benchmarks

The C++ implementation uses Google Benchmark for performance testing. The benchmark executable is located at:
`/home/airlab/Cppdev/bow_cpp_py/build/benchmark/`

```bash
# Navigate to benchmark directory
cd /home/airlab/Cppdev/bow_cpp_py/build/benchmark/

# Run comprehensive benchmarks
./BowBenchmark

# Run specific BOW planner benchmark
./BowBenchmark --benchmark_filter=BOW

# Run other specific planner benchmarks
./BowPlannerRAL --planner=BOW
./BowPlannerRAL --planner=RRT
./BowPlannerRAL --planner=HRVO
./BowPlannerRAL --planner=DWA
```

### Python Benchmarks

```bash
# Activate environment
conda activate bow

# Run Python benchmark suite
./run_py.sh

# Configure and run specific experiments
# Edit configuration parameters in config/main.yaml
# Then run specific experiments:
python main.py

# Run specific planners directly
python benchmark_mppi.py
python benchmark_cbf.py
```

## Verification Commands

### C++ Dependencies Verification

```bash
# Check all dependencies
pkg-config --exists nlopt && echo "NLopt ✓" || echo "NLopt ✗"
find /usr -name "fcl" -type d 2>/dev/null | head -1 && echo "FCL ✓" || echo "FCL ✗"
find /usr -name "ompl" -type d 2>/dev/null | head -1 && echo "OMPL ✓" || echo "OMPL ✗"
dpkg -l | grep libboost-program-options-dev && echo "Boost ✓" || echo "Boost ✗"
dpkg -l | grep libyaml-cpp-dev && echo "YAML-CPP ✓" || echo "YAML-CPP ✗"
```

### Python Environment Verification

```bash
# Check Python environment
conda activate bow
python -c "import numpy, matplotlib; print('Python environment ✓')"
```

## Automated Setup Script

For convenience, use the provided setup script:

```bash
cat > setup_environment.sh << 'EOF'
#!/bin/bash
echo "Setting up BOW Benchmark environment..."

# Update system
sudo apt-get update

# Install C++ dependencies
sudo apt-get install -y \
    build-essential cmake git pkg-config \
    libnlopt-dev libnlopt-cxx-dev \
    libfcl-dev \
    libompl-dev \
    libboost-all-dev \
    libyaml-cpp-dev

# Update library cache
sudo ldconfig

# Add library path fix for Anaconda compatibility
if ! grep -q "LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu" ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
    echo "Added library path fix to ~/.bashrc"
fi

echo "C++ environment setup complete!"
echo "For Python setup, run:"
echo "  conda create -n bow python=3.10"
echo "  conda activate bow"
echo "  pip install -r requirements.txt"
EOF

chmod +x setup_environment.sh
./setup_environment.sh
```

## Troubleshooting

### Common Issues

1. **GLIBCXX Version Mismatch (Anaconda)**
   ```bash
   export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
   ```

2. **CMake Cannot Find Packages**
   ```bash
   rm -rf build/
   mkdir build && cd build
   cmake ..
   ```

3. **Missing Development Headers**
   ```bash
   sudo apt-get install libnlopt-dev libfcl-dev libompl-dev libboost-all-dev libyaml-cpp-dev
   ```

### Alternative Distributions

**Fedora/RHEL/CentOS:**
```bash
sudo dnf install nlopt-devel fcl-devel ompl-devel boost-devel yaml-cpp-devel
```

**Arch Linux:**
```bash
sudo pacman -S nlopt fcl ompl boost yaml-cpp
```

**macOS (Homebrew):**
```bash
brew install nlopt fcl ompl boost yaml-cpp
```

## Results and Publications

For detailed experimental results, performance comparisons, and research publications, visit our project website:

**🌐 [bow-web.github.io](https://bow-web.github.io)**

## Contributing

We welcome contributions to the BOW benchmark project. Please ensure:

1. All C++ code follows the project's coding standards
2. Python code is compatible with Python 3.10+
3. New planners include appropriate benchmark integration
4. Documentation is updated accordingly

## License

[Add your license information here]

## Citation

If you use this benchmark in your research, please cite our work:

```bibtex
[Add your citation information here]
```

---

## Support

For issues and questions:

1. Check the troubleshooting section above
2. Visit our website: [bow-web.github.io](https://bow-web.github.io)
3. Create an issue in the project repository
4. Ensure all dependencies are properly installed before reporting issues
