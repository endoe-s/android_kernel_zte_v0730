
#ccflags-y +=-I$(srctree)/drivers/spi/mediatek/mt6797/

#MODFLAGS       =$(MODFLAGS)  -DMODULE -fno-pic  
ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/


#sunwave-objs := sunwavecorp.o platform.o misce.o mt6797_dts.o mt6797_dts_plat.o

obj-y += sunwavecorp.o
obj-y += platform.o
obj-y += misce.o
obj-y += mt6797_dts.o
#obj-y += mt6797_dts_plat.o
#ifdef CONFIG_SUNWAVECORP_MT6797
#sunwave-objs +=  mt6797_dts.o
#obj-$(CONFIG_SUNWAVE_FINGERPRINT)   = sunwave.o mt6797_dts_plat.o
#endif

