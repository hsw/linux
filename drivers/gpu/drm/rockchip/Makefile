# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

rockchipdrm-y := rockchip_drm_drv.o rockchip_drm_fb.o \
		rockchip_drm_gem.o

rockchipdrm-$(CONFIG_ROCKCHIP_VOP2) += rockchip_drm_vop2.o rockchip_vop2_reg.o
rockchipdrm-$(CONFIG_ROCKCHIP_VOP) += rockchip_drm_vop.o rockchip_vop_reg.o
rockchipdrm-$(CONFIG_ROCKCHIP_ANALOGIX_DP) += analogix_dp-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_CDN_DP) += cdn-dp-core.o cdn-dp-reg.o
rockchipdrm-$(CONFIG_ROCKCHIP_DW_HDMI) += dw_hdmi-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_DW_HDMI_QP) += dw_hdmi_qp-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_DW_MIPI_DSI) += dw-mipi-dsi-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_DW_MIPI_DSI2) += dw-mipi-dsi2-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_DW_DP) += dw_dp-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_INNO_HDMI) += inno_hdmi-rockchip.o
rockchipdrm-$(CONFIG_ROCKCHIP_LVDS) += rockchip_lvds.o
rockchipdrm-$(CONFIG_ROCKCHIP_RGB) += rockchip_rgb.o
rockchipdrm-$(CONFIG_ROCKCHIP_RK3066_HDMI) += rk3066_hdmi.o

obj-$(CONFIG_DRM_ROCKCHIP) += rockchipdrm.o

rk3026-ebc-y := rk3026_ebc.o rk3026_ebc_blit.o rk3026_ebc_core.o \
		rk3026_ebc_waveform.o
rk3026-ebc-$(CONFIG_DRM_RK3026_EBC_MXCFB) += rk3026_ebc_mxcfb.o
rk3026-ebc-$(CONFIG_DEBUG_FS) += rk3026_ebc_debugfs.o

obj-$(CONFIG_DRM_RK3026_EBC) += rk3026-ebc.o
tests/rk3026_ebc_kunit_test-y := tests/rk3026_ebc_kunit.o \
				 tests/rk3026_ebc_kunit_ref.o
obj-$(CONFIG_DRM_RK3026_EBC_KUNIT_TEST) += tests/rk3026_ebc_kunit_test.o
# rk3026_ebc_trace.h, for <trace/define_trace.h>
CFLAGS_rk3026_ebc.o := -I$(src)
