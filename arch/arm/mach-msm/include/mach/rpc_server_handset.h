/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ASM_ARCH_MSM_RPC_SERVER_HANDSET_H
#define __ASM_ARCH_MSM_RPC_SERVER_HANDSET_H

typedef signed char shextdet_form_position_result_t;
enum
{
SHEXTDET_FORM_POSITION_OPEN = 0,
SHEXTDET_FORM_POSITION_CLOSE,
SHEXTDET_FORM_POSITION_SWIVEL
};

struct msm_handset_platform_data {
	const char *hs_name;
	uint32_t pwr_key_delay_ms; /* default 500ms */
};

shextdet_form_position_result_t shextdet_get_form_state (void);

#define SHEXTDET_HEADSET_WAKELOCK
#ifdef SHEXTDET_HEADSET_WAKELOCK	/* Wake_Lock -> */
#define SHEXTDET_IOCTL_MAGIC 'x'
#define SHEXTDET_WAKE_UNLOCK _IO(SHEXTDET_IOCTL_MAGIC, 0x01)   /* Wake UnLock */
#define SHEXTDET_WAKE_LOCK_START _IO(SHEXTDET_IOCTL_MAGIC, 0x02)
#endif	/* Wake_Lock <- */

#endif
