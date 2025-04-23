# raspike-ros-firmware

raspike-ros-firmware is a micro-ROS module for SPIKE Prime.

This repository was extracted from the original [raspike-ros](https://github.com/Hiyama1026/raspike-ros) repository for modular reuse.

## Build and Update firmware

```bash
git clone -b dev-for-raspike-ros https://github.com/owhinata/spike-rt.git
git clone https://github.com/exshonda/micro-ROS_ASP3.git
git clone https://github.com/owhinata/raspike-ros-firmware.git
```

**libkernal.a, libpybricks.a**
```bash
(cd spike-rt && \
mkdir -p build/obj-primehub_kernel && \
cd build/obj-primehub_kernel && \
../../asp3/configure.rb -T primehub_gcc -f -m ../../common/kernel.mk && \
make libkernel.a)
```

**libmicroros.a**
```bash
(cd micro-ROS_ASP3/external && \
sed -i -e '/^#.*primehub/s/^#//' -e '/nucleo_f401re/s/^/# /' ../Makefile.config && \
make setup_micro-ros && \
make init_firmware && \
git clone https://github.com/owhinata/raspike_uros_msg primehub/firmware/mcu_ws/raspike_uros_msg && \
make build_firmware)
```

**asp.bin**
```bash
make -C raspike-ros-firmware asp.bin
```

**asp.dfu**
```bash
make -C raspike-ros-firmware asp.bin && \
python3 spike-rt/asp3/target/primehub_gcc/tools/dfu.py -b 0x8008000:micro-ROS_ASP3/spike-rt/uros_raspike_rt/asp.bin asp.dfu
```

**update firmware**
```bash
git clone https://github.com/micropython/micropython.git -b v1.23.0
python micropython/tools/pydfu.py -u asp.dfu --vid 0x0694 --pid 0x0008
```
