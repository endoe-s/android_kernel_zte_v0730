subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/

obj-y   += cs15xx-spi.o
ldflags-y += --whole-archive $(src)/cs_navigation.a  --no-whole-archive
ldflags-y += --whole-archive $(src)/autogain.a --no-whole-archive
