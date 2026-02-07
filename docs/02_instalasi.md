# 2. Instalasi ROS 2 Humble

## Persyaratan Sistem

### OS yang Didukung:
- **Ubuntu 24.04 (Noble Numbat)** - Recommended
- Ubuntu 22.04 dengan build dari source
- Windows 10/11
- macOS

Panduan ini fokus pada **Ubuntu 24.04**.

### Spesifikasi Minimum:
- **RAM**: 4GB (8GB recommended)
- **Storage**: 20GB free space
- **CPU**: 64-bit processor
- **Internet**: Untuk download packages

## Metode Instalasi

Ada 3 cara instalasi ROS 2 Humble:
1. **Debian Packages** (Recommended - paling mudah)
2. **Build from Source** (untuk development)
3. **Docker** (untuk isolation)

Kita akan menggunakan metode **Debian Packages**.

## Instalasi ROS 2 Humble di Ubuntu 24.04

### Step 1: Setup Locale

```bash
# Pastikan locale mendukung UTF-8
locale

# Jika belum ada locale UTF-8, install:
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

### Step 2: Setup Sources

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Packages

```bash
# Update package index
sudo apt update

# Upgrade packages
sudo apt upgrade

# Install ROS 2 Humble Desktop (Recommended)
sudo apt install ros-Humble-desktop

# Atau install ROS 2 Base (minimal, tanpa GUI tools)
# sudo apt install ros-Humble-ros-base

# Install development tools
sudo apt install ros-dev-tools
```

**Pilihan Instalasi:**
- `ros-Humble-desktop`: RViz, demos, tutorials (2GB+)
- `ros-Humble-ros-base`: Core ROS 2 tanpa GUI tools (500MB)

### Step 4: Setup Environment

```bash
# Source ROS 2 setup file
source /opt/ros/Humble/setup.bash

# Verify installation
ros2 --help
```

**Output yang diharapkan:**
```
usage: ros2 [-h] [--use-python-default-buffering]
            Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

options:
  -h, --help            show this help message and exit
  ...
```

### Step 5: Auto-source pada Startup (Optional tapi Recommended)

```bash
# Tambahkan ke .bashrc agar otomatis source setiap terminal baru
echo "source /opt/ros/Humble/setup.bash" >> ~/.bashrc

# Reload bashrc
source ~/.bashrc
```

## Instalasi Dependencies Tambahan

### 1. Colcon (Build Tool)

```bash
sudo apt install python3-colcon-common-extensions
```

### 2. rosdep (Dependency Management)

```bash
# Install rosdep
sudo apt install python3-rosdep2

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 3. Useful Tools

```bash
# Install additional tools
sudo apt install \
  python3-pip \
  python3-pytest-cov \
  ros-Humble-rviz2 \
  ros-Humble-rqt* \
  ros-Humble-demo-nodes-cpp \
  ros-Humble-demo-nodes-py
```

## Verifikasi Instalasi

### Test 1: Cek ROS 2 Version

```bash
ros2 doctor
```

### Test 2: Run Demo Talker-Listener

**Terminal 1** (Talker):
```bash
source /opt/ros/Humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2** (Listener):
```bash
source /opt/ros/Humble/setup.bash
ros2 run demo_nodes_py listener
```

**Expected Output (Terminal 1):**
```
[INFO] [1738713600.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1738713601.123456789] [talker]: Publishing: 'Hello World: 2'
```

**Expected Output (Terminal 2):**
```
[INFO] [1738713600.123456789] [listener]: I heard: [Hello World: 1]
[INFO] [1738713601.123456789] [listener]: I heard: [Hello World: 2]
```

### Test 3: Check Available Packages

```bash
ros2 pkg list
```

### Test 4: Check Available Commands

```bash
ros2 --help
```

## Setup Development Environment (Optional)

### Install Visual Studio Code

```bash
# Install VSCode
sudo snap install --classic code

# Install ROS extension
code --install-extension ms-iot.vscode-ros
```

### Install Python Development Tools

```bash
# Install Python packages
pip3 install --user \
  pytest \
  flake8 \
  black \
  mypy \
  setuptools
```

### Install C++ Development Tools

```bash
sudo apt install \
  build-essential \
  cmake \
  git \
  clang-format \
  clang-tidy
```

## Troubleshooting

### Problem 1: "command not found: ros2"

**Solution:**
```bash
source /opt/ros/Humble/setup.bash
echo "source /opt/ros/Humble/setup.bash" >> ~/.bashrc
```

### Problem 2: Repository tidak ditemukan

**Solution:**
```bash
# Pastikan Ubuntu version benar
lsb_release -a

# Update package list
sudo apt update
```

### Problem 3: GPG error

**Solution:**
```bash
# Re-add GPG key
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

### Problem 4: Locale warnings

**Solution:**
```bash
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## Uninstall ROS 2 (jika diperlukan)

```bash
# Remove ROS 2 packages
sudo apt remove ~nros-Humble-* && sudo apt autoremove

# Remove repository
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
```

## Environment Variables Penting

Setelah source setup.bash, beberapa environment variables di-set:

```bash
# Lihat ROS-related environment variables
printenv | grep ROS

# Important variables:
# ROS_DISTRO=Humble
# ROS_VERSION=2
# ROS_PYTHON_VERSION=3
```

## Rangkuman

**Yang sudah kita lakukan:**
1. Setup locale dan sources
2. Install ROS 2 Humble Desktop
3. Install development tools (colcon, rosdep)
4. Auto-source environment
5. Verifikasi instalasi dengan demo

**Yang bisa Anda lakukan sekarang:**
- Jalankan demo nodes
- Explore ROS 2 commands
- Mulai membuat workspace sendiri

## Next Steps

Lanjut ke [Konsep Dasar ROS 2](04_perintah_dasar.md) untuk memahami fundamental ROS 2!

