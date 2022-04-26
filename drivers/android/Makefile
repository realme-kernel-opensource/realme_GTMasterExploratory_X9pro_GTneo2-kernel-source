ccflags-y += -I$(src)			# needed for trace events

obj-$(CONFIG_ANDROID_BINDERFS)		+= binderfs.o
obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o binder_alloc.o
obj-$(CONFIG_ANDROID_BINDER_IPC_SELFTEST) += binder_alloc_selftest.o
ifneq ($(strip $(BRAND_SHOW_FLAG)), realme)
	ccflags-y += -DOPLUS_SF_HWC_BINDER_PERF
endif
