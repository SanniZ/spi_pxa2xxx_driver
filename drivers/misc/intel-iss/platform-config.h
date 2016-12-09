/*
 * ISS platform-specific definitions
 *
 * Copyright (c) 2012-2015, Intel Corporation.
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

#ifndef PLATFORM_CONFIG__H
#define PLATFORM_CONFIG__H

/* Build ID string */
#define	BUILD_ID	"kernel 4.4"

#define	ISH_DEBUG	0
#if ISH_DEBUG
#define	ISH_DBG_PRINT	printk
#else
#define	ISH_DBG_PRINT	no_printk
#endif

#define	ISH_INFO	1
#if ISH_INFO
#define	ISH_INFO_PRINT	printk
#else
#define	ISH_INFO_PRINT	no_printk
#endif

#define ISH_LOG		0

/*
 * Define if running on VirtualBox -
 * may solve imprecise timer emulation problems
 */
#if 0
#define	HOST_VIRTUALBOX	1
#endif

/* Timer-polling workaround for DUTs with non-functional interrupts reporting */
#if 0
#define	TIMER_POLLING	1
#endif

#define CHV_DEVICE_ID 0x22D8
#define BXT_M_AX_DEVICE_ID 0x0AA2
#define BXT_M_BX_DEVICE_ID 0x1AA2
#define BXT_P_AX_DEVICE_ID 0x5AA2

#define	REVISION_ID_CHT_A0	0x6
#define	REVISION_ID_CHT_A0_SI	0x0
#define	REVISION_ID_CHT_Bx_SI	0x10
#define	REVISION_ID_CHT_Kx_SI	0x20
#define	REVISION_ID_CHT_B0	0xB0

#define	REVISION_ID_SI_MASK	0x70

/*
 * For buggy (pre-)silicon, select model rather than retrieve it
 */
/* If defined, will support A0 only, will not check revision ID */
#if 0
#define	SUPPORT_Ax_ONLY	1
#endif


/* If defined, will support B0 only, will not check revision ID */
#if  0
#define	SUPPORT_Bx_ONLY	1
#endif

/* If defined, will support BXT only, will not check revision ID */
#if 0
#define	SUPPORT_BXT_ONLY	1
#endif

#if defined(SUPPORT_Ax_ONLY) + defined(SUPPORT_Bx_ONLY) + \
	defined(SUPPORT_BXT_ONLY) > 1
#error Only one of SUPPORT_Ax_ONLY, SUPPORT_Bx_ONLY or SUPPORT_BXT_ONLY \
	may be defined
#endif

/* D3 RCR */
#define	D3_RCR	1

/* Define in order to force FW-initated reset */
#define	FORCE_FW_INIT_RESET	1

/* Include ISH register debugger */
#define	ISH_DEBUGGER	1

/* Debug mutex locking/unlocking */
#define	DEBUG_LOCK	0

#if DEBUG_LOCK

static void	do_mutex_lock(void *m)
{
	mutex_lock(m);
}

static void	do_mutex_unlock(void *m)
{
	mutex_unlock(m);
}

#ifdef mutex_lock
#undef mutex_lock
#endif
#ifdef mutex_unlock
#undef mutex_unlock
#endif

#define mutex_lock(a) \
	do {\
		dev_warn(NULL, "%s:%d[%s] -- mutex_lock(%p)\n",	\
			__FILE__, __LINE__, __func__, a);	\
		do_mutex_lock(a);	\
	} while (0)

#define mutex_unlock(a) \
	do {\
		dev_warn(NULL, "%s:%d[%s] -- mutex_unlock(%p)\n",	\
			__FILE__, __LINE__, __func__, a);	\
		do_mutex_unlock(a);	\
	} while (0)
#endif /* DEBUG_LOCK */
/*************************************/

#endif /* PLATFORM_CONFIG__H*/

