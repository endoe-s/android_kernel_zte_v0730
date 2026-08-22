subdir-ccflags-y += -Werror -I$(srctree)/drivers/misc/mediatek/include/mt-plat/


$(warning ===== CHIPSAILING PATH DEBUG =====)
$(warning srctree=$(srctree))
$(warning src=$(src))
$(warning obj=$(obj))
$(warning objtree=$(objtree))
$(warning CURDIR=$(CURDIR))
$(warning KBUILD_SRC=$(KBUILD_SRC))
$(warning KBUILD_OUTPUT=$(KBUILD_OUTPUT))

$(warning --- cs_navigation.a ---)
$(warning 1=$(src)/cs_navigation.a)
$(warning 2=$(srctree)/$(src)/cs_navigation.a)
$(warning 3=$(abspath $(src)/cs_navigation.a))
$(warning 4=$(abspath $(srctree)/$(src)/cs_navigation.a))
$(warning 5=$(CURDIR)/$(src)/cs_navigation.a)
$(warning 6=$(CURDIR)/../$(src)/cs_navigation.a)
$(warning 7=$(objtree)/$(src)/cs_navigation.a)
$(warning 8=$(srctree)/drivers/spi/mediatek/mt6755/chipsailing/cs_navigation.a)
$(warning 9=$(abspath $(srctree)/drivers/spi/mediatek/mt6755/chipsailing/cs_navigation.a))

$(warning --- autogain.a ---)
$(warning 1=$(src)/autogain.a)
$(warning 2=$(srctree)/$(src)/autogain.a)
$(warning 3=$(abspath $(src)/autogain.a))
$(warning 4=$(abspath $(srctree)/$(src)/autogain.a))
$(warning 5=$(CURDIR)/$(src)/autogain.a)
$(warning 6=$(CURDIR)/../$(src)/autogain.a)
$(warning 7=$(objtree)/$(src)/autogain.a)
$(warning 8=$(srctree)/drivers/spi/mediatek/mt6755/chipsailing/autogain.a)
$(warning 9=$(abspath $(srctree)/drivers/spi/mediatek/mt6755/chipsailing/autogain.a))
$(warning ===== END PATH DEBUG =====)

obj-y   += cs15xx-spi.o
ldflags-y += --whole-archive $(abspath $(srctree)/$(src)/cs_navigation.a))  --no-whole-archive
ldflags-y += --whole-archive $(abspath $(srctree)/$(src)/autogain.a)) --no-whole-archive
