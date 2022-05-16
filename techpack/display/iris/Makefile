# Pixelworks Iris Driver
ccflags-y += -I$(srctree)/include/drm -I$(srctree)/techpack/display/msm -I$(srctree)/techpack/display/msm/dsi -I$(srctree)/techpack/display/msm/dp
ccflags-y += -I$(srctree)/techpack/display/msm/sde
ccflags-y += -I$(srctree)/techpack/display/rotator
ccflags-y += -I$(srctree)/techpack/display/oplus
ccflags-y += -I$(srctree)/techpack/display/pixelworks

ifeq ($(OPLUS_FEATURE_PXLW_IRIS5), yes)
obj-y += dsi_iris5_ioctl.o \
				dsi_iris5_lightup.o \
				dsi_iris5_lightup_ocp.o \
				dsi_iris5_lp.o \
				dsi_iris5_lut.o \
				dsi_iris5_pq.o \
				dsi_iris5_mode_switch.o \
				dsi_iris5_cmds.o \
				dsi_iris5_i3c.o \
				dsi_iris5_dbg.o \
				dsi_iris5_loop_back.o
endif