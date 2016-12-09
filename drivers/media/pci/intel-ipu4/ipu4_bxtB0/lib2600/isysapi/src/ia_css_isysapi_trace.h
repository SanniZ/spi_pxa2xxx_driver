/**
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

#ifndef __IA_CSS_ISYSAPI_TRACE_H
#define __IA_CSS_ISYSAPI_TRACE_H

#include "ia_css_trace.h"

#define ISYSAPI_TRACE_LOG_LEVEL_OFF 0
#define ISYSAPI_TRACE_LOG_LEVEL_NORMAL 1
#define ISYSAPI_TRACE_LOG_LEVEL_DEBUG 2

/* ISYSAPI and all the submodules in ISYSAPI will have
 * the default tracing level set to this level
 */
#define ISYSAPI_TRACE_CONFIG_DEFAULT ISYSAPI_TRACE_LOG_LEVEL_NORMAL

/* In case ISYSAPI_TRACE_CONFIG is not defined, set it to default level */
#if !defined(ISYSAPI_TRACE_CONFIG)
	#define ISYSAPI_TRACE_CONFIG ISYSAPI_TRACE_CONFIG_DEFAULT
#endif

/* ISYSAPI Module tracing backend is mapped to
 * TUNIT tracing for target platforms
 */
#ifdef IA_CSS_TRACE_PLATFORM_CELL
	#ifndef HRT_CSIM
		#define ISYSAPI_TRACE_METHOD IA_CSS_TRACE_METHOD_TRACE
	#else
		#define ISYSAPI_TRACE_METHOD IA_CSS_TRACE_METHOD_NATIVE
	#endif
#else
	#define ISYSAPI_TRACE_METHOD IA_CSS_TRACE_METHOD_NATIVE
#endif

#if (defined(ISYSAPI_TRACE_CONFIG))
	/* TRACE_OFF */
	#if ISYSAPI_TRACE_CONFIG == ISYSAPI_TRACE_LOG_LEVEL_OFF
		#define ISYSAPI_TRACE_LEVEL_ASSERT IA_CSS_TRACE_LEVEL_DISABLED
		#define ISYSAPI_TRACE_LEVEL_ERROR IA_CSS_TRACE_LEVEL_DISABLED
		#define ISYSAPI_TRACE_LEVEL_WARNING IA_CSS_TRACE_LEVEL_DISABLED
		#define ISYSAPI_TRACE_LEVEL_INFO IA_CSS_TRACE_LEVEL_DISABLED
		#define ISYSAPI_TRACE_LEVEL_DEBUG IA_CSS_TRACE_LEVEL_DISABLED
		#define ISYSAPI_TRACE_LEVEL_VERBOSE IA_CSS_TRACE_LEVEL_DISABLED
	/* TRACE_NORMAL */
	#elif ISYSAPI_TRACE_CONFIG == ISYSAPI_TRACE_LOG_LEVEL_NORMAL
		#define ISYSAPI_TRACE_LEVEL_ASSERT IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_ERROR IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_WARNING IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_INFO IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_DEBUG IA_CSS_TRACE_LEVEL_DISABLED
		#define ISYSAPI_TRACE_LEVEL_VERBOSE IA_CSS_TRACE_LEVEL_DISABLED
	/* TRACE_DEBUG */
	#elif ISYSAPI_TRACE_CONFIG == ISYSAPI_TRACE_LOG_LEVEL_DEBUG
		#define ISYSAPI_TRACE_LEVEL_ASSERT IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_ERROR IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_WARNING IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_INFO IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_DEBUG IA_CSS_TRACE_LEVEL_ENABLED
		#define ISYSAPI_TRACE_LEVEL_VERBOSE IA_CSS_TRACE_LEVEL_ENABLED
	#else
		#error "No ISYSAPI_TRACE_CONFIG Tracing level defined"
	#endif
#else
	#error "ISYSAPI_TRACE_CONFIG not defined"
#endif

#endif /* __IA_CSS_ISYSAPI_TRACE_H */
