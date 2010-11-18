/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __ARCH_ARM_MACH_MSM_DEBUG_MM_H_
#define __ARCH_ARM_MACH_MSM_DEBUG_MM_H_

/* *********************DEBUG LEVELS************************
 * 0 - Disables all the messages including error messages
 * 1 - Prints error messages only
 * 2 - Prints both error and info messages (Default)
 * 3 - Prints all the messages including debug messages
 * ********************************************************/
/* Change the debug level according to the above description in
 * individual module level header files i.e. debug_audio_mm.h
 * and debug_adsp_mm.h. More module level header files can be
 * added as per requirement.
 */

#ifndef MSM_MM_DEBUG_LEVEL
#undef MSM_MM_DEBUG
#undef MSM_MM_INFO
#undef MSM_MM_ERROR
#elif (MSM_MM_DEBUG_LEVEL == 0)
#undef MSM_MM_DEBUG
#undef MSM_MM_INFO
#undef MSM_MM_ERROR
#elif (MSM_MM_DEBUG_LEVEL == 1)
#undef MSM__DEBUG
#undef MSM_MM_INFO
#define MSM_MM_ERROR
#elif (MSM_MM_DEBUG_LEVEL == 2)
#undef MSM_MM_DEBUG
#define MSM_MM_INFO
#define MSM_MM_ERROR
#elif (MSM_MM_DEBUG_LEVEL == 3)
#define MSM_MM_DEBUG
#define MSM_MM_INFO
#define MSM_MM_ERROR
#endif

/* Defining the DEBUG macro before including "kernel.h" will
 * enable the pr_debug messages in corresponding modules
 */
#ifdef MSM_MM_DEBUG
#define DEBUG
#endif
#include <linux/kernel.h>

/* The below macro removes the directory path name and retains only the
 * file name to avoid long path names in log messages that comes as
 * part of __FILE__ to compiler.
 */
#define __MM_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/')+1) : \
	__FILE__

#ifndef MSM_MM_MODULE
#define MSM_MM_MODULE ""
#endif

#ifdef MSM_MM_DEBUG
#define MM_DBG(fmt, args...) pr_debug("[%s] " fmt,\
	__func__, ##args)
#else
#define MM_DBG(fmt, args...) do {} while (0)
#endif

#ifdef MSM_MM_INFO
#define MM_INFO(fmt, args...) pr_info("[%s] " fmt,\
	__func__, ##args)
#else
#define MM_INFO(fmt, args...) do {} while (0)
#endif

#ifdef MSM_MM_ERROR
#define MM_ERR(fmt, args...) pr_err("[%s] " fmt,\
	       __func__, ##args)
#else
#define MM_ERR(fmt, args...) do {} while (0)
#endif

#endif /* __ARCH_ARM_MACH_MSM_DEBUG_MM_H_ */
