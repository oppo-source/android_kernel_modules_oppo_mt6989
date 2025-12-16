# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat
#/* #ifdef OPLUS_FEATURE_DISPLAY */
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/include/mt-plat/    \
             -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/mtk-interconnect/   \
             -I$(DEVICE_MODULES_PATH)/drivers/misc/mediatek/slbc/               \
             -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/dpc
#/* #endif OPLUS_FEATURE_DISPLAY */

#OPLUS_FEATURE_DISPLAY
#build oplus config into panel
ccflags-y += -DOPLUS_FEATURE_DISPLAY
ccflags-y += -DOPLUS_FEATURE_DISPLAY_TEMP_COMPENSATION
ccflags-y += -DOPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
ccflags-y += -DOPLUS_FEATURE_DISPLAY_ADFR
ccflags-y += -DOPLUS_FEATURE_DISPLAY_HPWM
ccflags-y += -DOPLUS_FEATURE_DISPLAY_APOLLO
#endif

#CONFIG_PXLW_IRIS
ccflags-y += -DCONFIG_PXLW_IRIS
ccflags-y += -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/mediatek_v2 \
             -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/iris/vendor/display-drivers \
             -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/iris/core \
             -I$(DEVICE_MODULES_PATH)/drivers/gpu/drm/mediatek/iris/core/common
#endif

obj-$(CONFIG_DRM_PANEL_NOVATEK_NT35695B_VDO) += panel-nt35695b-auo-vdo.o
obj-$(CONFIG_DRM_PANEL_NOVATEK_NT36672A_VDO) += panel-nt36672a-rt4801-vdo.o
obj-$(CONFIG_DRM_PANEL_SAMSUNG_ANA6705_CMD) += panel-samsung-ana6705-cmd.o
obj-$(CONFIG_DRM_PANEL_SAMSUNG_ANA6705_CMD_FHDP) += panel-samsung-ana6705-cmd-fhdp.o
obj-$(CONFIG_DRM_PANEL_HX_NT37701_DPHY_CMD) += panel-hx-nt37701-dphy-cmd.o
obj-$(CONFIG_DRM_PANEL_HX_NT37701_DPHY_CMD_120HZ) += panel-hx-nt37701-dphy-cmd-120hz.o
obj-$(CONFIG_DRM_PANEL_TRULY_TD4330_VDO) += panel-truly-td4330-vdo.o
obj-$(CONFIG_DRM_PANEL_TRULY_TD4330_CMD) += panel-truly-td4330-cmd.o
obj-$(CONFIG_DRM_PANEL_PANEL_AC222_P_7_A0014_DSI_CMD) += oplus_panel/panel_ac222_p_7_a0014_dsi_cmd.o
obj-$(CONFIG_DRM_PANEL_AC222_P_7_A0014_DSI_CMD_PANEL) += oplus_panel/ac222_p_7_a0014_dsi_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC230_P_7_A0014_DSI_CMD_PANEL_T1) += oplus_panel/ac230_p_7_a0014_dsi_cmd_panel_t1.o
obj-$(CONFIG_DRM_PANEL_AC230_P_3_A0004_DSI_CMD_PANEL_T1) += oplus_panel/ac230_p_3_a0004_dsi_cmd_panel_t1.o
obj-$(CONFIG_DRM_PANEL_AC230_P_7_A0014_DSI_CMD_PANEL_T0) += oplus_panel/ac230_p_7_a0014_dsi_cmd_panel_t0.o
obj-$(CONFIG_DRM_PANEL_AC232_P_7_A0014_DSI_CMD_PANEL_T0) += oplus_panel/ac232_p_7_a0014_dsi_cmd_panel_t0.o
obj-$(CONFIG_DRM_PANEL_PANEL_AE016_P_7_A0014_DSI_CMD) += oplus_panel/panel_ae016_p_7_a0014_dsi_cmd.o
obj-$(CONFIG_DRM_PANEL_AC264_P_7_A0014_DSI_CMD_PANEL) += oplus_panel/ac264_p_7_a0014_dsi_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC264_P_3_A0004_DSI_CMD_PANEL) += oplus_panel/ac264_p_3_a0004_dsi_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC264_P_3_2_A0020_DSI_CMD_PANEL) += oplus_panel/ac264_p_3_2_a0020_dsi_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AA597_P_3_A0025_DSI_VDO_PANEL) += oplus_panel/aa597_p_3_a0025_dsi_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_AA597_P_7_A0025_DSI_VDO_PANEL) += oplus_panel/aa597_p_7_a0025_dsi_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_AC164_P_3_A0001_CMD_PANEL) += ac164_p_3_a0001_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC164_P_7_A0001_CMD_PANEL) += ac164_p_7_a0001_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC164_P_7_A0001_CMD_PANEL) += ac158_p_7_a0001_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC158_P_B_A0012_CMD_PANEL) += ac158_p_b_a0012_cmd_panel.o
obj-$(CONFIG_DRM_S6E8FC3_FHDP_DSI_VDO_SAMSUNG_AMS667FK03) += s6e8fc3_fhdp_dsi_vdo_samsung_ams667fk03.o
obj-$(CONFIG_DRM_NT37703A_FHDP_DSI_VDO_120HZ_DPHY) += nt37703a_fhdp_dsi_vdo_120hz_dphy.o
obj-$(CONFIG_DRM_NT37703A_FHDP_HX_DSI_VDO_120HZ_DPHY) += nt37703a_fhdp_hx_dsi_vdo_120hz_dphy.o
obj-$(CONFIG_DRM_PANEL_AC234_P_B_A0012_CMD_PANEL) += ac234_p_b_a0012_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AC234_P_3_A0004_CMD_PANEL) += ac234_p_3_a0004_cmd_panel.o
obj-$(CONFIG_DRM_PANEL_AA600_P_7_A0025_VDO_PANEL) += oplus_panel/aa600_p_7_a0025_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_AA600_P_3_A0025_VDO_PANEL) += oplus_panel/aa600_p_3_a0025_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_AC304_P_7_A0025_VDO_PANEL) += oplus_panel/ac304_p_7_a0025_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_AC304_P_3_A0027_VDO_PANEL) += oplus_panel/ac304_p_3_a0027_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_PANEL_AB946_P_3_A0020_DSI_CMD_BCP) += oplus_panel/panel_ab946_p_3_a0020_dsi_cmd_bcp.o
obj-$(CONFIG_DRM_PANEL_PANEL_AB948_P_7_A0020_DSI_CMD) += oplus_panel/panel_ab948_p_7_a0020_dsi_cmd.o
obj-$(CONFIG_DRM_PANEL_AE037_P_3_A0026_DSI_VDO) += oplus_panel/panel_ae037_p_3_a0026_dsi_vdo.o
obj-$(CONFIG_DRM_AE174_P_1_A0037_CMD_PANEL) += ae174_p_1_a0037_cmd_panel.o
obj-$(CONFIG_DRM_OPLUS24780_NT37706A_FHDP_DSI_VDO_144HZ_DPHY_BOE) += oplus24780_nt37706a_fhdp_dsi_vdo_144hz_dphy_boe.o
obj-$(CONFIG_DRM_OPLUS24781_NT37703A_FHDP_HX_DSI_VDO_120HZ_DPHY) += oplus24781_nt37703a_fhdp_hx_dsi_vdo_120hz_dphy.o
obj-$(CONFIG_DRM_OPLUS24781_NT37703A_FHDP_DSI_VDO_REPLACE_DPHY_BOE) += oplus24781_nt37703a_fhdp_dsi_vdo_replace_dphy_boe.o
obj-$(CONFIG_DRM_OPLUS25680_TD4376B_FHDP_HX_DSI_VDO) += oplus25680_td4376b_fhdp_hx_dsi_vdo.o
obj-$(CONFIG_DRM_OPLUS25711_NT37706A_FHDP_DSI_VDO_144HZ_DPHY_BOE) += oplus25711_nt37706a_fhdp_dsi_vdo_144hz_dphy_boe.o
obj-$(CONFIG_DRM_PANEL_SAMSUNG_OP_CMD) += panel-samsung-op-cmd.o
obj-$(CONFIG_DRM_PANEL_SAMSUNG_OP_CMD_MSYNC2) += panel-samsung-op-cmd-msync2.o
obj-$(CONFIG_DRM_PANEL_TRULY_FT8756_VDO) += panel-truly-ft8756-vdo.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672C_CPHY_VDO) += panel-alpha-jdi-nt36672c-cphy-vdo.o
obj-$(CONFIG_DRM_PANEL_SC_NT36672C_VDO_120HZ) += panel-sc-nt36672c-vdo-120hz.o
obj-$(CONFIG_DRM_PANEL_TIANMA_R66451_CMD_120HZ) += panel-tianma-r66451-cmd-120hz.o
obj-$(CONFIG_DRM_PANEL_TIANMA_R66451_CMD_120HZ_WA) += panel-tianma-r66451-cmd-120hz-wa.o
obj-$(CONFIG_DRM_PANEL_TIANMA_NT36672E_VDO_120HZ_HFP) += panel-tianma-nt36672e-vdo-120hz-hfp.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_CPHY_VDO) += panel-alpha-jdi-nt36672e-cphy-vdo.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_VDO_120HZ) += panel-alpha-jdi-nt36672e-vdo-120hz.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_VDO_120HZ_THRESHOLD) += panel-alpha-jdi-nt36672e-vdo-120hz-threshold.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_VDO_144HZ) += panel-alpha-jdi-nt36672e-vdo-144hz.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_VDO_60HZ) += panel-alpha-jdi-nt36672e-vdo-60hz.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_VDO_120HZ_HFP) += panel-alpha-jdi-nt36672e-vdo-120hz-hfp.o
obj-$(CONFIG_DRM_PANEL_ALPHA_JDI_NT36672E_VDO_144HZ_HFP) += panel-alpha-jdi-nt36672e-vdo-144hz-hfp.o
obj-$(CONFIG_DRM_PANEL_TRULY_NT35595_CMD) += panel-truly-nt35595-cmd.o
obj-$(CONFIG_DRM_PANEL_NT37801_MSYNC2_RTE) += panel-nt37801-msync2-rte.o
obj-$(CONFIG_DRM_PANEL_SAMSUNG_S68FC01_VDO_AOD) += panel-samsung-s68fc01-vdo-aod.o
obj-$(CONFIG_DRM_PANEL_ALPHA_SAMSUNG_S6E3HAE_CMD_120HZ) += panel-alpha-samsung-s6e3hae-fhd-dphy-cmd-120hz-r4831a.o
obj-$(CONFIG_DRM_PANEL_HIMAX_HX83121A_VDO) += panel-himax-hx83121a-vdo.o
obj-$(CONFIG_DRM_PANEL_NT37801_CMD_LTPO) += panel-nt37801-cmd-ltpo.o
obj-$(CONFIG_DRM_PANEL_NT37801_CMD_120HZ) += panel-nt37801-cmd-120hz.o
obj-$(CONFIG_DRM_PANEL_L12A_42_02_0A_DSC_CMD) += panel-l12a-42-02-0a-dsc-cmd.o
obj-$(CONFIG_DRM_PANEL_NT37701A_CMD) += panel-nt37701a-cmd.o
obj-$(CONFIG_DRM_PANEL_NT37801_CMD_FHD) += panel-nt37801-cmd-fhd.o
obj-$(CONFIG_DRM_PANEL_NT37801_CMD_SPR) += panel-nt37801-cmd-spr.o
obj-$(CONFIG_DRM_PANEL_NT37801_CMD_FHD_PLUS) += panel-nt37801-cmd-fhd-plus.o
obj-$(CONFIG_DRM_PANEL_ALPHA_NT37706_VDO_120HZ) += panel-alpha-nt37706-vdo-120hz.o
obj-$(CONFIG_DRM_PANEL_BOE_TS127QFMLL1DKP0) += panel-boe-ts127qfmll1dkp0.o
obj-$(CONFIG_DRM_PANEL_BOE_TS127QFMLL1DKP0_120HZ) += panel-boe-ts127qfmll1dkp0-120hz.o
obj-$(CONFIG_DRM_PANEL_NT37707_677_FHDP_DPHY_CMD) += panel-nt37707-667-fhdp-dphy-cmd.o
obj-$(CONFIG_DRM_PANEL_NT37705_1056_1068_DPHY_CMD) += panel-nt37705-1056-1068-dphy-cmd.o
obj-$(CONFIG_DRM_PANEL_VDTR6130_CMD_120HZ_CSOT) += panel-vdtr6130-fhdp-dsi-cmd-dphy-120hz-csot.o
ifneq ($(wildcard $(srctree)/$(src)/k6985v1_64),)
obj-$(CONFIG_DRM_TD2204_WQHD_AMB678ZY01_S6E3HC3_CMD) += k6985v1_64/td2204-wqhd-amb678zy01-s6e3hc3-cmd.o
endif
ifneq ($(wildcard $(srctree)/$(src)/k6985v1_64_alpha),)
obj-$(CONFIG_DRM_PANEL_NT37705_ALPHA_CMD) += k6985v1_64_alpha/panel-nt37705-alpha-cmd.o
endif
obj-$(CONFIG_DRM_PANEL_OPLUS21135_TM_NT37705_FHD_DSI_CMD) += oplus_panel/oplus21135_tm_nt37705_fhd_dsi_cmd.o
obj-$(CONFIG_DRM_PANEL_PANEL_AC094_P_3_A0004_DSI_CMD) += oplus_panel/panel_ac094_p_3_a0004_dsi_cmd.o
obj-$(CONFIG_DRM_PANEL_PANEL_AC094_P_B_A0004_DSI_CMD) += oplus_panel/panel_ac094_p_b_a0004_dsi_cmd.o
obj-$(CONFIG_DRM_PANEL_PANEL_AB946_P_3_A0020_DSI_CMD) += oplus_panel/panel_ab946_p_3_a0020_dsi_cmd.o
ifneq ($(wildcard $(srctree)/$(src)/k6989v1_64_alpha),)
obj-$(CONFIG_DRM_PANEL_ILI7838E_ALPHA_CMD) += k6989v1_64_alpha/panel-ili7838e-alpha-cmd.o
obj-$(CONFIG_DRM_PANEL_ILI7838E_DV2_ALPHA_CMD) += k6989v1_64_alpha/panel-ili7838e-dv2-alpha-cmd.o
obj-$(CONFIG_DRM_PANEL_ILI7838E_DV2_SPR_CMD) += k6989v1_64_alpha/panel-ili7838e-dv2-spr-cmd.o
obj-$(CONFIG_DRM_PANEL_RM692H5_ALPHA_CMD) += k6989v1_64_alpha/panel-rm692h5-alpha-cmd.o
endif
ifneq ($(wildcard $(srctree)/$(src)/k6989v1_64_alpha_vdo),)
obj-$(CONFIG_DRM_PANEL_ILI7838E_DV2_ALPHA_VDO) += k6989v1_64_alpha_vdo/panel-ili7838e-dv2-alpha-vdo.o
endif
ifneq ($(wildcard $(srctree)/$(src)/k6989v1_64_flip_alpha),)
obj-$(CONFIG_DRM_PANEL_BOE_BF130_DPHY_CMD_HD) += k6989v1_64_flip_alpha/panel-boe-bf130-dphy-cmd-hd.o
obj-$(CONFIG_DRM_PANEL_SAMSUNG_AMF680CM01_CMD_FHD) += k6989v1_64_flip_alpha/panel-samsung-amf680cm01-cmd-fhd.o
endif
ifneq ($(wildcard $(srctree)/$(src)/k6878v1_64_alpha),)
obj-$(CONFIG_DRM_PANEL_MOT_CSOT_NT37705_DPHY_CMD) += k6878v1_64_alpha/dsi-panel-mot-csot-nt37705-1056x1068-dphy-cmd-cli.o
obj-$(CONFIG_DRM_PANEL_MOT_CSOT_NT37707_FHDP_DPHY_CMD) += k6878v1_64_alpha/dsi-panel-mot-csot-nt37707-667-fhdp-dphy-cmd.o
endif
obj-$(CONFIG_BACKLIGHT_KTZ8866) += ktz8866.o
obj-$(CONFIG_DRM_PANEL_OPLUS24921_NT36532W_2800_2000_DUAL_DSI_VDO_144HZ) += oplus24921_nt36532w_2800_2000_dual_dsi_vdo_144hz_csot.o
obj-$(CONFIG_BACKLIGHT_KTZ8868) += ktz8868.o
obj-$(CONFIG_DRM_PANEL_X25N11_P_D_A0024_VDO_PANEL) += x25n11_p_d_a0024_vdo_panel.o
obj-$(CONFIG_DRM_PANEL_AE150_P_D_A0038_VDO_PVT) += ae150_p_d_a0038_vdo_pvt.o
obj-$(CONFIG_DRM_PANEL_AE150_P_D_A0038_VDO_DVT) += ae150_p_d_a0038_vdo_dvt.o
obj-$(CONFIG_DRM_PANEL_AE150_P_D_A0038_VDO_EVT) += ae150_p_d_a0038_vdo_evt.o

mediatek-drm-gateic-y := mtk_drm_panel_i2c.o \
		  mtk_drm_gateic_rt4801h.o \
		  mtk_drm_gateic_rt4831a.o \
		  mtk_drm_gateic.o

mediatek-drm-panel-drv-y :=  \
		  mtk_round_corner/mtk_drm_rc.o \
		  mtk_drm_panel_helper.o \
		  mtk_drm_panel_helper_dbi.o \
		  mtk_drm_panel_helper_dpi.o \
		  mtk_drm_panel_helper_dsi.o \
		  mtk_drm_panel_drv_dbi.o \
		  mtk_drm_panel_drv_dpi.o \
		  mtk_drm_panel_drv_dsi.o \
		  mtk_drm_panel_drv.o

mediatek-cust-panel-sample-y :=  \
		  cust_panel/mtk_cust_panel_sample/mtk_drm_panel_cust.o \
		  cust_panel/cust_lcm_hal.o


obj-$(CONFIG_DRM_MEDIATEK_PANEL) += mediatek-drm-gateic.o
obj-$(CONFIG_DRM_MEDIATEK_PANEL) += mediatek-drm-panel-drv.o
obj-$(CONFIG_DRM_MEDIATEK_PANEL) += mediatek-cust-panel-sample.o

ccflags-$(CONFIG_DRM_MEDIATEK_PANEL) += -I$(DEVICE_MODULES_PATH)/drivers/leds/
