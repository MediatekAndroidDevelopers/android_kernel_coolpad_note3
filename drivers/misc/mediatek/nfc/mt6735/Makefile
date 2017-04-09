#include $(srctree)/drivers/misc/mediatek/Makefile.custom

#wangsl nfc gpio conflict to others
obj-$(CONFIG_NFC_MT6605)   += mt6605.o
#obj-y   += mt6605.o
ccflags-$(CONFIG_NFC_MT6605) +=  -DDEBUG
