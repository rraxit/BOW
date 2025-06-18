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
| | **HRVO** | Hybrid Reciprocal Velocity Obstacles |
| | **DWA** | Dynamic Window Approach |
| **Python** | **MPPI** | Model Predictive Path Integral |
| | **CBF** | Control Barrier Functions |

## 🛠️ Quick Start

### C++ Development

```bash
# Clone and build
git clone <your-repo>
cd bow_benchmark
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run benchmarks
./BowPlannerRAL
./benchmark/BowBenchmark
```

### Python Development

```bash
# Setup environment
conda create -n bow python=3.10
conda activate bow
pip install -r requirements.txt

# Run benchmarks
./run_py.sh
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

## ⚙️ Installation

### C++ Setup

```bash
# Install dependencies
sudo apt update
sudo apt install build-essential cmake git pkg-config \
    libnlopt-dev libnlopt-cxx-dev libfcl-dev libompl-dev \
    libboost-all-dev libyaml-cpp-dev

# Build project
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 🔧 Anaconda Compatibility Fix

If you have Anaconda installed, add this to your `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

### Python Setup

```bash
conda create -n bow python=3.10
conda activate bow
pip install -r requirements.txt
```

## 🏃‍♂️ Running Benchmarks

### C++ Benchmarks

```bash
cd build/

# Main benchmark application
./BowPlannerRAL

# Google Benchmark tests
./benchmark/BowBenchmark
./benchmark/BowBenchmark --benchmark_filter=BOW
```

### Python Benchmarks

```bash
conda activate bow

# Complete benchmark suite
./run_py.sh

# Custom experiments (edit config/main.yaml first)
python main.py

# Individual planners
python benchmark_mppi.py
python benchmark_cbf.py
```

## 📁 Project Structure

```
bow_benchmark/
├── 📂 src/                    # C++ implementations
│   ├── 📂 planners/          # Algorithm implementations
│   │   ├── 📂 bow/           # BOW planner
│   │   ├── 📂 rrt/           # RRT planner
│   │   ├── 📂 hrvo/          # HRVO planner
│   │   └── 📂 dwa/           # DWA planner
│   └── 📂 benchmark/         # Benchmarking utilities
├── 📂 python/                # Python implementations
│   ├── 📂 planners/          # MPPI & CBF planners
│   ├── 📂 benchmarks/        # Benchmark scripts
│   └── 📂 config/            # Configuration files
├── 📂 build/                 # C++ build directory
├── 📄 requirements.txt       # Python dependencies
├── 🔧 run_py.sh             # Python benchmark runner
└── 🐍 main.py               # Python main script
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
