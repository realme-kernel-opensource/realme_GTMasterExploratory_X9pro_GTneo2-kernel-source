#for AWINIC AW8697 Haptic
#ifdef OPLUS_FEATURE_CHG_BASIC
ifeq ($(strip $(BRAND_SHOW_FLAG)), oneplus)
 	ccflags-y += -DOPLUS_CHG_OP_DEF
else ifeq ($(strip $(CONFIG_OPLUS_CHG_OOS)), y)
 	ccflags-y += -DOPLUS_CHG_OP_DEF
endif
#endif
obj-$(CONFIG_AW8697_HAPTIC) += aw8697.o


