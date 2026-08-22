subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
$(warning SRCTREE=$(srctree))
$(warning SRC=$(src))
$(warning OBJ=$(obj))
$(warning CURDIR=$(CURDIR))
obj-y   += cs15xx-spi.o
ldflags-y += --whole-archive $(srctree)/$(src)/cs_navigation.a  --no-whole-archive
ldflags-y += --whole-archive $(srctree)/$(src)/autogain.a --no-whole-archive
