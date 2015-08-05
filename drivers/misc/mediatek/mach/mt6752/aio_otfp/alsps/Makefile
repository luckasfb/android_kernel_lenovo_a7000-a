
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  cm36652-new/
endif

# lenovo-sw youwc1 add for ALS/PS start
ifeq ($(CONFIG_MTK_EPL21822),y)
obj-y	+=  EPL21822/
endif
# lenovo-sw youwc1 add for ALS/PS end
# lenovo-sw molg1 add 20141230 begin
ifeq ($(CONFIG_MTK_EPL252X),y)
obj-y	+=  epl252x/
endif
# lenovo-sw molg1 add 20141230 end
