# LIO-SAM-H
这是hzy的LIO-SAM的代码复现版本

## 环境依赖
**安装gtsam**
```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install -y \
    cmake \
    libboost-all-dev \
    libtbb-dev \
    libeigen3-dev \
    build-essential
```
从源码编译安装GTSAM
```bash
cd
mkdir -p ~/gtsam
cd ~/gtsam

# 克隆 GTSAM 源码（使用 4.2.0 版本，比较稳定）
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2.0  # 使用稳定版本

# 创建构建目录
mkdir build && cd build

# 配置编译选项
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_BUILD_PYTHON=OFF \
      -DCMAKE_INSTALL_PREFIX=/usr/local ..

# 编译（根据你的 CPU 核心数调整，这里使用 4 个核心）
make -j4

# 安装
sudo make install
```