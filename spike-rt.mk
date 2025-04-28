#  SPDX-License-Identifier: MIT
#
#  Makefile for Pybricks.
#
#  Copyright (C) 2022 by Embedded and Real-Time Systems Laboratory,
#                        Graduate School of Information Science, Nagoya Univ., JAPAN
#

SPIKERT_TOP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SPIKERT_DIR := $(SPIKERT_TOP_DIR)/spike-rt

ifeq ("$(wildcard $(SPIKERT_DIR)/README.md)","")
$(info GIT cloning spike-rt)
$(info $(shell cd $(SPIKERT_TOP_DIR) && git submodule update --init spike-rt))
ifeq ("$(wildcard $(SPIKERT_DIR)/README.md)","")
$(error failed)
endif
endif

