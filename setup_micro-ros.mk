#
#  ROS2 Version
#
ROS_DISTRO = humble

LIBUROS_TOP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LIBUROS_WS := $(LIBUROS_TOP_DIR)/libmicroros/uros_ws
LIBUROS_SETUP_MODULE := libmicroros/uros_ws/src/micro_ros_setup
LIBUROS_SETUP_DIR := $(LIBUROS_TOP_DIR)/$(LIBUROS_SETUP_MODULE)
LIBUROS_FIRMWARE_WS := $(LIBUROS_TOP_DIR)/libmicroros

SHELL=/bin/bash

ifeq ("$(wildcard $(LIBUROS_SETUP_DIR)/README.md)","")
$(info GIT cloning micro_ros_setup submodule)
$(info $(shell cd $(LIBUROS_TOP_DIR) && git submodule update --init $(LIBUROS_SETUP_MODULE)))
ifeq ("$(wildcard $(LIBUROS_SETUP_DIR)/README.md)","")
$(error failed)
endif
endif

.PHONY: setup_micro-ros build_micro_ros_setup create_firmware_ws build_firmware

#
# Setup micro-ROS
#
setup_micro-ros:
	sudo apt-get update
	sudo apt-get install -y \
		python3-pip python3-nose clang-format pyflakes3 python3-mypy python3-pytest-mock gperf \
		ros-$(ROS_DISTRO)-osrf-testing-tools-cpp python3-lttng ros-$(ROS_DISTRO)-mimick-vendor \
		python3-babeltrace python3-rosdep2 python3-colcon-common-extensions
	make -f $(LIBUROS_TOP_DIR)/setup_micro-ros.mk build_micro_ros_setup
	make -f $(LIBUROS_TOP_DIR)/setup_micro-ros.mk create_firmware_ws
	make -f $(LIBUROS_TOP_DIR)/setup_micro-ros.mk build_firmware

build_micro_ros_setup:
ifeq ("$(wildcard $(LIBUROS_WS)/install/setup.bash)","")
	source /opt/ros/$(ROS_DISTRO)/setup.bash; \
	cd $(LIBUROS_WS); rosdep update && rosdep install --rosdistro $(ROS_DISTRO) --from-path src --ignore-src -y; \
	colcon build
endif

create_firmware_ws:
ifeq ("$(wildcard $(LIBUROS_FIRMWARE_WS)/firmware/COLCON_IGNORE)","")
	cd $(LIBUROS_FIRMWARE_WS); \
	source $(LIBUROS_WS)/install/setup.bash; \
	ros2 run micro_ros_setup create_firmware_ws.sh generate_lib;
endif
ifeq ("$(wildcard $(LIBUROS_FIRMWARE_WS)/firmware/mcu_ws/raspike_uros_msg/README.md)","")
	cd $(LIBUROS_FIRMWARE_WS); \
	git clone https://github.com/owhinata/raspike_uros_msg firmware/mcu_ws/raspike_uros_msg
endif

build_firmware:
	cd $(LIBUROS_FIRMWARE_WS); \
	source $(LIBUROS_WS)/install/setup.bash; \
	ros2 run micro_ros_setup build_firmware.sh \
		$(LIBUROS_FIRMWARE_WS)/gcc-arm-cm4_toolchain.cmake \
		$(LIBUROS_FIRMWARE_WS)/colcon.meta

