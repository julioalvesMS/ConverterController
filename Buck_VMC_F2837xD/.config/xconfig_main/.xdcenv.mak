#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/controlSUITE/powerSUITE/v_1_01_00_00/packages;C:/Users/julio/Codes/ConverterController/Buck_VMC_F2837xD/.config
override XDCROOT = C:/ti/ccs910/xdctools_3_55_02_22_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/controlSUITE/powerSUITE/v_1_01_00_00/packages;C:/Users/julio/Codes/ConverterController/Buck_VMC_F2837xD/.config;C:/ti/ccs910/xdctools_3_55_02_22_core/packages;..
HOSTOS = Windows
endif
