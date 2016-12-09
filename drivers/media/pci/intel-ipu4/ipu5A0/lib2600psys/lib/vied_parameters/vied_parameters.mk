# #
# Support for Intel Camera Imaging ISP subsystem.
# Copyright (c) 2010 - 2016, Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details
#
#
# MODULE is VIED_PARAMETERS

VIED_PARAMETERS_DIR=$${MODULES_DIR}/vied_parameters

VIED_PARAMETERS_INTERFACE=$(VIED_PARAMETERS_DIR)/interface
VIED_PARAMETERS_SOURCES=$(VIED_PARAMETERS_DIR)/src
VIED_PARAMETERS_EXTINCLUDE = $${MODULES_DIR}/support

VIED_PARAMETERS_DYNAMIC_HOST_FILES += $(VIED_PARAMETERS_SOURCES)/ia_css_terminal.c
VIED_PARAMETERS_STATIC_HOST_FILES += $(VIED_PARAMETERS_SOURCES)/ia_css_terminal_manifest.c

VIED_PARAMETERS_HOST_FILES  = $(VIED_PARAMETERS_DYNAMIC_HOST_FILES)
VIED_PARAMETERS_HOST_FILES += $(VIED_PARAMETERS_STATIC_HOST_FILES)

VIED_PARAMETERS_DYNAMIC_FW_FILES += $(VIED_PARAMETERS_SOURCES)/ia_css_terminal.c
VIED_PARAMETERS_STATIC_FW_FILES += $(VIED_PARAMETERS_SOURCES)/ia_css_terminal_manifest.c

VIED_PARAMETERS_FW_FILES  = $(VIED_PARAMETERS_DYNAMIC_HOST_FILES)
VIED_PARAMETERS_FW_FILES += $(VIED_PARAMETERS_STATIC_HOST_FILES)
VIED_PARAMETERS_SUPPORT_CPPFLAGS = -I$(VIED_PARAMETERS_DIR)/support
VIED_PARAMETERS_SUPPORT_CPPFLAGS += -I$(VIED_PARAMETERS_DIR)/support/$(IPU_SYSVER)
VIED_PARAMETERS_PSA_UTILS_HOST_FILES = $(MODULES_DIR)/vied_parameters/support/ia_css_psys_parameter_utils.c
VIED_PARAMETERS_PSA_UTILS_HOST_FILES += $(MODULES_DIR)/vied_parameters/support/$(IPU_SYSVER)/ia_css_psys_parameter_utils_dep.c

VIED_PARAMETERS_UTILS_HOST_CPPFLAGS = $(VIED_PARAMETERS_SUPPORT_CPPFLAGS)

VIED_PARAMETERS_ISA_UTILS_HOST_FILES = $(MODULES_DIR)/vied_parameters/support/ia_css_isys_parameter_utils.c
VIED_PARAMETERS_ISA_UTILS_HOST_FILES += $(MODULES_DIR)/vied_parameters/support/$(IPU_SYSVER)/ia_css_isys_parameter_utils_dep.c

VIED_PARAMETERS_PRINT_CPPFLAGS += -I$(VIED_PARAMETERS_DIR)/print/interface
VIED_PARAMETERS_PRINT_FILES    += $(VIED_PARAMETERS_DIR)/print/src/ia_css_terminal_print.c

# VIED_PARAMETERS Trace Log Level = VIED_PARAMETERS_TRACE_LOG_LEVEL_NORMAL
# Other options are [VIED_PARAMETERS_TRACE_LOG_LEVEL_OFF, VIED_PARAMETERS_TRACE_LOG_LEVEL_DEBUG]
ifndef VIED_PARAMETERS_TRACE_CONFIG_HOST
	VIED_PARAMETERS_TRACE_CONFIG_HOST=VIED_PARAMETERS_TRACE_LOG_LEVEL_NORMAL
endif
ifndef VIED_PARAMETERS_TRACE_CONFIG_FW
	VIED_PARAMETERS_TRACE_CONFIG_FW=VIED_PARAMETERS_TRACE_LOG_LEVEL_NORMAL
endif

VIED_PARAMETERS_HOST_CPPFLAGS += -DVIED_PARAMETERS_TRACE_CONFIG=$(VIED_PARAMETERS_TRACE_CONFIG_HOST)
VIED_PARAMETERS_FW_CPPFLAGS += -DVIED_PARAMETERS_TRACE_CONFIG=$(VIED_PARAMETERS_TRACE_CONFIG_FW)

VIED_PARAMETERS_HOST_CPPFLAGS += -I$(VIED_PARAMETERS_INTERFACE)
VIED_PARAMETERS_HOST_CPPFLAGS += -I$(VIED_PARAMETERS_EXTINCLUDE)
VIED_PARAMETERS_HOST_CPPFLAGS += $(VIED_PARAMETERS_SUPPORT_CPPFLAGS)
VIED_PARAMETERS_FW_CPPFLAGS += -I$(VIED_PARAMETERS_INTERFACE)
VIED_PARAMETERS_FW_CPPFLAGS += -I$(VIED_PARAMETERS_EXTINCLUDE)
VIED_PARAMETERS_FW_CPPFLAGS +=  $(VIED_PARAMETERS_SUPPORT_CPPFLAGS)

#For IPU interface
include $(MODULES_DIR)/fw_abi_common_types/cpu/fw_abi_cpu_types.mk
VIED_PARAMETERS_HOST_CPPFLAGS += $(FW_ABI_COMMON_TYPES_HOST_CPPFLAGS)
VIED_PARAMETERS_FW_CPPFLAGS   += $(FW_ABI_COMMON_TYPES_FW_CPPFLAGS)