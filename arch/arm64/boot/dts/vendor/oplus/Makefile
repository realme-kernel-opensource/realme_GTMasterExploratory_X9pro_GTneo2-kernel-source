ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
    dtbo-$(CONFIG_ARCH_LITO) += kunlun-20111-mtp-overlay.dtbo \
                    kunlun-20111-v2-mtp-overlay.dtbo \
                    kunlun-20111_t0-mtp-overlay.dtbo \
                    kunlun-20111_t0-v2-mtp-overlay.dtbo \
                    kunlun-20251-mtp-overlay.dtbo \
                    kunlun-20251-v2-mtp-overlay.dtbo \
                    kunlun-20251_t0-mtp-overlay.dtbo \
                    kunlun-20251_t0-v2-mtp-overlay.dtbo \
                    cherry-mtp-overlay.dtbo \
                    cherry-v2-mtp-overlay.dtbo \
                    cherry-t0-mtp-overlay.dtbo \
                    cherry-t0-v2-mtp-overlay.dtbo \
                    goku-20267-mtp-overlay.dtbo \
                    goku-20267-v2-mtp-overlay.dtbo

    kunlun-20111-mtp-overlay.dtbo-base := lito.dtb
    kunlun-20111-v2-mtp-overlay.dtbo-base := lito-v2.dtb
    kunlun-20111_t0-mtp-overlay.dtbo-base := lito.dtb
    kunlun-20111_t0-v2-mtp-overlay.dtbo-base := lito-v2.dtb
    kunlun-20251-mtp-overlay.dtbo-base := lito.dtb
    kunlun-20251-v2-mtp-overlay.dtbo-base := lito-v2.dtb
    kunlun-20251_t0-mtp-overlay.dtbo-base := lito.dtb
    kunlun-20251_t0-v2-mtp-overlay.dtbo-base := lito-v2.dtb
    cherry-mtp-overlay.dtbo-base := lito.dtb
    cherry-v2-mtp-overlay.dtbo-base := lito-v2.dtb
    cherry-t0-mtp-overlay.dtbo-base := lito.dtb
    cherry-t0-v2-mtp-overlay.dtbo-base := lito-v2.dtb
    goku-20267-mtp-overlay.dtbo-base := lito.dtb
    goku-20267-v2-mtp-overlay.dtbo-base := lito-v2.dtb
else
    dtb-$(CONFIG_ARCH_LITO) += lito-mtp.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo

