#
# Copyright (C) 2017 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#

FPSGO_TOP = $(DEVICE_MODULES_PATH)/drivers/misc/mediatek/performance/fpsgo_v3
FBT_TOP = $(FPSGO_TOP)/fbt
COMP_TOP = $(FPSGO_TOP)/composer
MTK_TOP = $(DEVICE_MODULES_PATH)/drivers/misc/mediatek
BASE_TOP = $(FPSGO_TOP)/base
FSTB_TOP = $(FPSGO_TOP)/fstb

obj-y += platform/$(MTK_PLATFORM)/fbt_cpu_platform.o

obj-y += \
	src/fbt_cpu.o \
	src/xgf.o \
	src/mini_top.o

ccflags-y += \
	-I$(DEVICE_MODULES_PATH)/include/ \
	-I$(MTK_TOP)/include/ \
	-I$(FPSGO_TOP)/base/include/ \
	-I$(COMP_TOP)/include/ \
	-I$(FBT_TOP)/include/ \
	-I$(BASE_TOP)/include/ \
	-I$(MTK_TOP)/base/power/include/ \
	-I$(MTK_TOP)/base/power/$(MTK_PLATFORM)/ \
	-I$(FSTB_TOP)/ \
	-I$(MTK_TOP)/cm_mgr/ \
	-I$(DEVICE_MODULES_PATH)/kernel/
