SSC_DBG_MODULE_NAME=mtk-ssc-dbg-v1

SSC_MODULE_FOLDER=$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/ssc/

ccflags-y += -I${SSC_MODULE_FOLDER}/inc/
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/

obj-$(CONFIG_MTK_SSC_MODULE) += ${SSC_DBG_MODULE_NAME}.o

${SSC_DBG_MODULE_NAME}-objs += ssc_dbg_v1.o
