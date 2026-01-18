# biped_rl_controller

## Dependencies

- legged_sdk

- yaml-cpp

## Installation

1. Install [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) under `/opt/unitree_robotics`
    ```bash
    git clone https://github.com/unitreerobotics/unitree_sdk2.git
    cd unitree_sdk2/
    mkdir build
    cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
    sudo make install
    ```

2. Install [LibTorch](https://pytorch.org/) using [pre-built wheel](https://github.com/lupinjia/torch_whl_cpu)
    ```bash
    sudo apt-get install libopenblas-dev
    git clone https://github.com/lupinjia/torch_whl_cpu.git # or download the file
    pip install torch-2.6.0a0+git9f5ebf3-cp310-cp310-linux_aarch64.whl
    ```

3. Install Gainput under `/usr/local`
    ```bash
    git clone https://github.com/lupinjia/gainput.git
    cd gainput
    mkdir build && cd build
    cmake ..
    make
    make install
    ```

4. Clone this repo and compile
    ```bash
    git clone git@gitee.com:jia-yasen/biped_rl_controller.git
    mkdir build && cd build
    cmake ..
    make
    ```

5. Run biped_rl_controller
    ```bash
    ./biped_rl_controller
    ```