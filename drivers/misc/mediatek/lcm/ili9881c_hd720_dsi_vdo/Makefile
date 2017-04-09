#
# Makefile for misc devices that really don't fit anywhere else.
#
obj-y += ili9881c_hd720_dsi_vdo.o

ccflags-$(CONFIG_MTK_LCM) += -I$(srctree)/drivers/misc/mediatek/lcm/inc

#wangsl
ifneq ($(strip $(DRT_LCM_FEATURE)),NULL)
ccflags-y +=  $(foreach n,$(DRT_LCM_FEATURE),-D$(n))
#$(error modules_to_check $(DRT_LCM_FEATURE))
#ccflags-$(LCM_FEATER) += $(addprefix -D, $(DEFINES))
endif

