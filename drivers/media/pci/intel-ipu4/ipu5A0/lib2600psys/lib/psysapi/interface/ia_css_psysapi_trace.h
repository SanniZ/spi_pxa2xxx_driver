/*
* Support for Intel Camera Imaging ISP subsystem.
 * Copyright (c) 2010 - 2016, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
*/

#ifndef __IA_CSS_PSYSAPI_TRACE_H_INCLUDED__
#define __IA_CSS_PSYSAPI_TRACE_H_INCLUDED__

#include "ia_css_trace.h"

#define PSYSAPI_TRACE_LOG_LEVEL_OFF 0
#define PSYSAPI_TRACE_LOG_LEVEL_NORMAL 1
#define PSYSAPI_TRACE_LOG_LEVEL_DEBUG 2

/* PSYSAPI and all the submodules in PSYSAPI will have the default tracing
 * level set to the PSYSAPI_TRACE_CONFIG level. If not defined in the
 * psysapi.mk fill it will be set by default to no trace
 * (PSYSAPI_TRACE_LOG_LEVEL_OFF)
 */
#define PSYSAPI_TRACE_CONFIG_DEFAULT PSYSAPI_TRACE_LOG_LEVEL_OFF

#if !defined(PSYSAPI_TRACE_CONFIG)
	#define PSYSAPI_TRACE_CONFIG PSYSAPI_TRACE_CONFIG_DEFAULT
#endif

/* Module specific trace setting will be used if
 * the trace level is not specified from the module or
  PSYSAPI_TRACING_OVERRIDE is defined
 */
#if (defined(PSYSAPI_TRACE_CONFIG))
	/* Module specific trace setting */
	#if PSYSAPI_TRACE_CONFIG == PSYSAPI_TRACE_LOG_LEVEL_OFF
		/* PSYSAPI_TRACE_LOG_LEVEL_OFF */
		#define PSYSAPI_TRACE_METHOD IA_CSS_TRACE_METHOD_NATIVE
		#define PSYSAPI_TRACE_LEVEL_ASSERT IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_ERROR IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_WARNING IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_INFO IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_DEBUG IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_VERBOSE IA_CSS_TRACE_LEVEL_DISABLED
	#elif PSYSAPI_TRACE_CONFIG == PSYSAPI_TRACE_LOG_LEVEL_NORMAL
		/* PSYSAPI_TRACE_LOG_LEVEL_NORMAL */
		#define PSYSAPI_TRACE_METHOD IA_CSS_TRACE_METHOD_NATIVE
		#define PSYSAPI_TRACE_LEVEL_ASSERT IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_ERROR IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_WARNING IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_INFO IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_DEBUG IA_CSS_TRACE_LEVEL_DISABLED
		#define PSYSAPI_TRACE_LEVEL_VERBOSE IA_CSS_TRACE_LEVEL_DISABLED
	#elif PSYSAPI_TRACE_CONFIG == PSYSAPI_TRACE_LOG_LEVEL_DEBUG
		/* PSYSAPI_TRACE_LOG_LEVEL_DEBUG */
		#define PSYSAPI_TRACE_METHOD IA_CSS_TRACE_METHOD_NATIVE
		#define PSYSAPI_TRACE_LEVEL_ASSERT IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_ERROR IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_WARNING IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_INFO IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_DEBUG IA_CSS_TRACE_LEVEL_ENABLED
		#define PSYSAPI_TRACE_LEVEL_VERBOSE IA_CSS_TRACE_LEVEL_ENABLED
	#else
		#error "No PSYSAPI_TRACE_CONFIG Tracing level defined"
	#endif
#else
	#error "PSYSAPI_TRACE_CONFIG not defined"
#endif

/* Overriding submodules in PSYSAPI with a specific tracing level */
/* #define PSYSAPI_DYNAMIC_TRACING_OVERRIDE TRACE_LOG_LEVEL_VERBOSE */

#endif /* __IA_CSS_PSYSAPI_TRACE_H_INCLUDED__  */
