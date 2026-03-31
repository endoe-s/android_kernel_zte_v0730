subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/spi/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
	include $(srctree)/drivers/misc/mediatek/gud/Makefile.include
endif

obj-$(CONFIG_MTK_SPI) += spi.o
obj-$(CONFIG_FINGERPRINT_SUPPORT) += fingerprint_core.o
obj-$(CONFIG_FINGERPRINT_GF3208_REE)	+= goodix/gf-milanf.o  goodix/gf-common.o 
obj-$(CONFIG_MICROARRAY_FINGERPRINT) += afs121n/madev.o afs121n/fingerprint.o
obj-$(CONFIG_LEADCORE_FINGERPRINT) += lc1550/
obj-$(CONFIG_CHIPSAILING_FINGERPRINT) += chipsailing/
obj-$(CONFIG_SUNWAVE_FINGERPRINT) += sunwavecorp/
