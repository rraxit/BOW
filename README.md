# 🎯 BOW Benchmark
> Rigorous benchmarking for Bayesian Optimization Approach to Windowing Motion Planning

[![Website](https://img.shields.io/badge/🌐-Project%20Website-blue)](https://bow-web.github.io)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)](https://python.org/)

## 📋 Overview

BOW Benchmark provides comprehensive performance comparisons between **BOW (Bayesian Optimization Approach to Windowing Motion Planning)** and state-of-the-art motion planning algorithms.

### 🚀 Supported Planners

| Language | Planner | Description |
|----------|---------|-------------|
| **C++** | **BOW** | Bayesian Optimization Approach to Windowing Motion Planning |
| | **RRT** | Rapidly-exploring Random Trees |
| | **[HRVO](https://github.com/steakhouserodriguez/HRVO-python.git)** | Hybrid Reciprocal Velocity Obstacles |
| | **[DWA](https://github.com/onlytailei/CppRobotics/blob/master/src/dynamic_window_approach.cpp)** | Dynamic Window Approach |
| **Python** | **[MPPI](https://github.com/kohonda/mppi_playground.git)** | Model Predictive Path Integral |
| | **[CBF](https://github.com/mit-ll-trusted-autonomy/cbfToolbox.git)** | Control Barrier Functions |

## 🛠️ Quick Start

**First you have to install and setup the environment and then run.**

### Step 1: Install & Setup Environment

#### C++ Environment Setup

```bash
# Install dependencies
sudo apt update
sudo apt install build-essential cmake git pkg-config \
    libnlopt-dev libnlopt-cxx-dev libfcl-dev libompl-dev \
    libboost-all-dev libyaml-cpp-dev

# Clone and build
git clone <repo>
cd BOW
mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### Python Environment Setup

```bash
# Setup conda environment
conda create -n bow python=3.10
conda activate bow
pip install -r requirements.txt
```

#### 🔧 Anaconda Compatibility Fix

If you have Anaconda installed, add this to your `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

### Step 2: Run Benchmarks

#### C++ Benchmarks

```bash
# Run benchmarks
./build/benchmark/BowBenchmark

# Google Benchmark tests with custom options
./build/benchmark/BowBenchmark --benchmark_filter=BOW --benchmark_repetitions=10
```

#### Python Benchmarks

```bash
# Activate environment
conda activate bow

# Complete benchmark suite
./run_py.sh

# Custom experiments (edit config/main.yaml first)
python main.py
```

## 📦 Dependencies

### C++ Requirements

| Library | Purpose | Installation |
|---------|---------|--------------|
| **NLopt** | Nonlinear optimization | `sudo apt install libnlopt-dev libnlopt-cxx-dev` |
| **FCL** | Collision detection | `sudo apt install libfcl-dev` |
| **OMPL** | Motion planning | `sudo apt install libompl-dev` |
| **Boost** | C++ utilities | `sudo apt install libboost-all-dev` |
| **YAML-CPP** | Configuration parsing | `sudo apt install libyaml-cpp-dev` |

### Python Requirements

```bash
pip install -r requirements.txt
```

## 📁 Project Structure

```
BOW/
├── 📂 algos/                 # Python algorithm implementations
├── 📂 benchmark/             # C++ benchmark implementations
│   ├── 📂 include/          # Algorithm headers (dwa, hrvo, ompl)
│   └── 📂 src/              # Benchmark source code
├── 📂 bow/                   # BOW planner implementation
│   └── 📂 bow++/            # C++ BOW implementation
├── 📂 build/                 # C++ build directory
├── 📂 config/                # Configuration files
│   ├── 📂 planner/          # Planner configurations
│   └── 📂 test/             # Test configurations
├── 📂 include/               # C++ headers
│   ├── 📂 bow/              # BOW headers
│   └── 📂 limbo/            # Bayesian optimization library
├── 📂 results/               # Benchmark results
├── 📂 result_analysis_cpp/   # C++ result analysis
├── 📂 result_analysis_py/    # Python result analysis
├── 📂 scripts/               # Utility scripts
├── 📂 src/                   # Additional source implementations
└── 📂 test/                  # Test files
```

## 🔍 Verification

Check your installation:

```bash
# C++ dependencies
pkg-config --exists nlopt && echo "NLopt ✅" || echo "NLopt ❌"
find /usr -name "fcl" -type d 2>/dev/null | head -1 && echo "FCL ✅" || echo "FCL ❌"
find /usr -name "ompl" -type d 2>/dev/null | head -1 && echo "OMPL ✅" || echo "OMPL ❌"

# Python environment
conda activate bow
python -c "import numpy, matplotlib; print('Python environment ✅')"
```

## 🐛 Troubleshooting

### Common Issues

**GLIBCXX Version Mismatch (Anaconda)**
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

**CMake Configuration Issues**
```bash
rm -rf build/ && mkdir build && cd build && cmake ..
```

**Missing Headers**
```bash
sudo apt install libnlopt-dev libfcl-dev libompl-dev libboost-all-dev libyaml-cpp-dev
```

### Other Distributions

**Fedora/RHEL/CentOS:**
```bash
sudo dnf install nlopt-devel fcl-devel ompl-devel boost-devel yaml-cpp-devel
```

**macOS (Homebrew):**
```bash
brew install nlopt fcl ompl boost yaml-cpp
```

## 📊 Results & Publications

Visit our project website for detailed experimental results and publications:

**🌐 [bow-web.github.io](https://bow-web.github.io)**

## 🤝 Contributing

We welcome contributions! Please ensure:
- C++ code follows project standards
- Python compatibility with 3.10+
- Proper benchmark integration
- Updated documentation

## 📄 License

[Add your license information here]

## 📚 Citation

If you use this benchmark in your research, please cite our work:

```bibtex
[Add your citation information here]
```

---

<div align="center">

**🌐 [Project Website](https://bow-web.github.io)** • **📧 [Support](mailto:your-email@domain.com)** • **🐛 [Issues](https://github.com/your-repo/issues)**

</div>
