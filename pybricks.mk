#  SPDX-License-Identifier: MIT
#
#  Makefile for Pybricks.
#
#  Copyright (C) 2022 by Embedded and Real-Time Systems Laboratory,
#                        Graduate School of Information Science, Nagoya Univ., JAPAN
#

PYBRICKS_TOP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

PYBRICKS_DIR := $(PYBRICKS_TOP_DIR)/libpybricks
PYBRICKS_BRICK_DIR := $(PYBRICKS_DIR)/bricks/primehub_spike-rt
PYBRICKS_OBJDIR := $(PYBRICKS_TOP_DIR)/objs/libpybricks

# PYBRICKS_SPIKE_ID :=
PYBRICKS_SPIKE_ID := spike1_

ifeq ("$(wildcard $(PYBRICKS_DIR)/README.md)","")
$(info GIT cloning libpybricks submodule)
$(info $(shell cd $(PYBRICKS_TOP_DIR) && git submodule update --init libpybricks))
ifeq ("$(wildcard $(PYBRICKS_DIR)/README.md)","")
$(error failed)
endif
endif

PYBRICKS_LIB := $(PYBRICKS_OBJDIR)/libpybricks.a
PYBRICKS_INCLUDES := -I$(PYBRICKS_DIR)/lib/pbio/include \
										 -I$(PYBRICKS_DIR)/bricks/primehub_spike-rt \
										 -I$(PYBRICKS_DIR)/lib/pbio/platform/prime_hub_spike-rt \
										 -I$(PYBRICKS_DIR)/lib/lego \
										 -I$(PYBRICKS_DIR)/lib/contiki-core

PYBRICKS_ACC_CALIB := $(PYBRICKS_TOP_DIR)/calib/$(PYBRICKS_SPIKE_ID)test_imu_acc.calib
PYBRICKS_GYRO_CALIB := $(PYBRICKS_TOP_DIR)/calib/$(PYBRICKS_SPIKE_ID)test_imu_gyro.calib
PYBRICKS_IMU_CALIB_HEADER := $(PYBRICKS_OBJDIR)/imu_calib.inc

.PHONY: $(PYBRICKS_IMU_CALIB_HEADER)
$(PYBRICKS_IMU_CALIB_HEADER): $(PYBRICKS_ACC_CALIB) $(PYBRICKS_GYRO_CALIB)
	@echo "Generating $@ from $^..."
	@mkdir -p $(PYBRICKS_OBJDIR)
	@echo "/* Auto-generated from $(PYBRICKS_SPIKE_ID)test_imu_acc.calib and $(PYBRICKS_SPIKE_ID)test_imu_gyro.calib */" > $@
	@awk -v MATRIX=ax -v BIAS=ay ' \
		{ if (NF) { lines[++n] = $$0 } } \
		END { \
			for (i=0; i<3; i++) { \
				line = lines[i+1]; gsub(/[ \t]+/, " ", line); split(line, t, " "); \
				for (j=0; j<3; j++) MISALIGN[i,j] = t[j+1]; \
			} \
			for (i=0; i<3; i++) { \
				line = lines[i+4]; gsub(/[ \t]+/, " ", line); split(line, k, " "); \
				SCALE[i] = k[i+1]; \
			} \
			printf("static const float %s[3][3] = {\n", MATRIX); \
			for (i=0; i<3; i++) { \
				printf("  { "); \
				for (j=0; j<3; j++) { \
					printf("%s%s * %s", (j>0?", ":""), MISALIGN[i,j], SCALE[j]); \
				} \
				printf(" }%s\n", (i<2?",":"")); \
			} \
			printf("};\n"); \
			printf("static const float %s[3] = { %s, %s, %s };\n", BIAS, lines[7], lines[8], lines[9]); \
		}' $(PYBRICKS_ACC_CALIB) >> $@
	@awk -v MATRIX=wx -v BIAS=wy ' \
		{ if (NF) { lines[++n] = $$0 } } \
		END { \
			for (i=0; i<3; i++) { \
				line = lines[i+1]; gsub(/[ \t]+/, " ", line); split(line, t, " "); \
				for (j=0; j<3; j++) MISALIGN[i,j] = t[j+1]; \
			} \
			for (i=0; i<3; i++) { \
				line = lines[i+4]; gsub(/[ \t]+/, " ", line); split(line, k, " "); \
				SCALE[i] = k[i+1]; \
			} \
			printf("static const float %s[3][3] = {\n", MATRIX); \
			for (i=0; i<3; i++) { \
				printf("  { "); \
				for (j=0; j<3; j++) { \
					printf("%s%s * %s", (j>0?", ":""), MISALIGN[i,j], SCALE[j]); \
				} \
				printf(" }%s\n", (i<2?",":"")); \
			} \
			printf("};\n"); \
			printf("static const float %s[3] = { %s, %s, %s };\n", BIAS, lines[7], lines[8], lines[9]); \
		}' $(PYBRICKS_GYRO_CALIB) >> $@


#
#  Rules to build Pybricks.
#
.PHONY: libpybricks.a
libpybricks.a: $(PYBRICKS_OBJDIR)/libpybricks.a

# recipe for building pybricks
.PHONY: $(PYBRICKS_OBJDIR)/libpybricks.a
$(PYBRICKS_OBJDIR)/libpybricks.a: $(PYBRICKS_IMU_CALIB_HEADER)
	$(MAKE) -C $(PYBRICKS_BRICK_DIR) BUILD=$(PYBRICKS_OBJDIR)
