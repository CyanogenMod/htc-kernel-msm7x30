/** include/asm-arm/arch-msm/msm_rpc_version.h
 *
 * Copyright (C) 2008 HTC Corporation
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM__ARCH_MSM_RPC_VERSION_H
#define __ASM__ARCH_MSM_RPC_VERSION_H

#if (CONFIG_MSM_AMSS_VERSION == 6210)
#define INT_ADSP                               INT_ADSP_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:20f17fd3"
#define RPC_ADSP_RTOS_ATOM_VERS                0x20f17fd3 /* 552697811 */
#define RPC_ADSP_RTOS_MTOA_VERS                0x75babbd6 /* 1975172054 */
#define AUDMGR_VERS                            0x46255756 /* 1176852310 */
#define AUDMGR_CB_VERS                         0x5fa922a9 /* 1604919977 */
#define DOG_KEEPALIVE_VERS                     0
#define RPC_DOG_KEEPALIVE_BEACON               1
#define TIME_REMOTE_MTOA_VERS                  0
#define RPC_SND_VERS                           0x94756085 /* 2490720389 */
#define RPC_SND_CB_VERS                        0xfb9dd580 /* 4221425024 */
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  24
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 25
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00000000"
#define PM_LIBVERS                             0xfb837d0b
#define HTC_PROCEDURE_SET_VIB_ON_OFF           21

#elif (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:71d1094b"
#define RPC_ADSP_RTOS_ATOM_VERS                0x71d1094b /* 1909524811 */
#define RPC_ADSP_RTOS_MTOA_VERS                0xee3a9966 /* 3996817766 */
#define AUDMGR_VERS                            0xe94e8f0c /* 3914239756 */
#define AUDMGR_CB_VERS                         0x21570ba7 /* 559352743 */
#define DOG_KEEPALIVE_VERS                     0x731fa727 /* 1931454247 */
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x9202a8e4 /* 2449647844 */
#define RPC_SND_VERS                           0xaa2b1a44 /* 2854951492 */
#define RPC_SND_CB_VERS                        0x71e691ca /* 1910936010 */
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  24
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 25
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:0da5b528"
#define PM_LIBVERS                             0xfb837d0b
#define HTC_PROCEDURE_SET_VIB_ON_OFF           21

#elif (CONFIG_MSM_AMSS_VERSION == 6355)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:00010001"
#define RPC_ADSP_RTOS_ATOM_VERS                0x10001 /* 65537 */
#define RPC_ADSP_RTOS_MTOA_VERS                0x20001 /* 131073 */
#define AUDMGR_VERS                            0x10002 /* 65538 */
#define AUDMGR_CB_VERS                         0x10002 /* 65538 */
#define DOG_KEEPALIVE_VERS                     0x10001 /* 65537 */
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x10001 /* 65537 */
#define RPC_SND_VERS                           0x10001
#define RPC_SND_CB_VERS                        0x10001
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  23
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 24
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00010001"
#define PM_LIBVERS                             0x10001
#define HTC_PROCEDURE_SET_VIB_ON_OFF           21

#elif (CONFIG_MSM_AMSS_VERSION == 1355)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:00010001"
#define RPC_ADSP_RTOS_ATOM_VERS                0x10001
#define RPC_ADSP_RTOS_MTOA_VERS                0x10001
#define AUDMGR_VERS                            0x10002
#define AUDMGR_CB_VERS                         0x10002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x10001
#define RPC_SND_VERS                           0x10001
#define RPC_SND_CB_VERS                        0x10001
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  23
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 24
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:915823fc"
#define PM_LIBVERS                             0x10001
#define HTC_PROCEDURE_SET_VIB_ON_OFF           21

#elif (CONFIG_MSM_AMSS_VERSION == 4320)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:00010001"
#define RPC_ADSP_RTOS_ATOM_VERS                0x10001
#define RPC_ADSP_RTOS_MTOA_VERS                0x30002
#define AUDMGR_VERS                            0x20002
#define AUDMGR_CB_VERS                         0x20002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x10002
#define RPC_SND_VERS                           0x20002
#define RPC_SND_CB_VERS                        0x20002
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  24
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 25
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00010003"
#define PM_LIBVERS                             0x10001
#define HTC_PROCEDURE_SET_VIB_ON_OFF           21

#elif (CONFIG_MSM_AMSS_VERSION == 4725)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:00010001"
#define RPC_ADSP_RTOS_ATOM_VERS                0x10001
#define RPC_ADSP_RTOS_MTOA_VERS                0x30002
#define AUDMGR_VERS                            0x20002
#define AUDMGR_CB_VERS                         0x20002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x10002
#define RPC_SND_VERS                           0x20002
#define RPC_SND_CB_VERS                        0x20002
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  24
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 25
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00010003"
#define PM_LIBVERS                             0x10001
#define HTC_PROCEDURE_SET_VIB_ON_OFF           22

#elif (CONFIG_MSM_AMSS_VERSION == 4735)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:00010001"
#define RPC_ADSP_RTOS_ATOM_VERS                0x10001
#define RPC_ADSP_RTOS_MTOA_VERS                0x30002
#define AUDMGR_VERS                            0x20002
#define AUDMGR_CB_VERS                         0x20002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x10002
#define RPC_SND_VERS                           0x20002
#define RPC_SND_CB_VERS                        0x20002
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  24
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 25
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00010004"
#define PM_LIBVERS                             0x10001
#define HTC_PROCEDURE_SET_VIB_ON_OFF           22

#elif (CONFIG_MSM_AMSS_VERSION == 4410)
#define INT_ADSP                               INT_ADSP_A9_A11
#define ADSP_DRIVER_NAME                       "rs3000000a:00010001"
#define RPC_ADSP_RTOS_ATOM_VERS                0x10001
#define RPC_ADSP_RTOS_MTOA_VERS                0x20001
#define AUDMGR_VERS                            0x10002
#define AUDMGR_CB_VERS                         0x10002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define TIME_REMOTE_MTOA_VERS                  0x10001
#define RPC_SND_VERS                           0x10001
#define RPC_SND_CB_VERS                        0x10001
#define VOCPCM_REGISTER_PCM_INPUT_CLIENT_PROC  23
#define VOCPCM_REGISTER_PCM_OUTPUT_CLIENT_PROC 24
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00010001"
#define PM_LIBVERS                             0x10001
#define HTC_PROCEDURE_SET_VIB_ON_OFF           21

#elif(CONFIG_MSM_AMSS_VERSION == 3200)
#define TIME_REMOTE_MTOA_VERS                  0x9202a8e4 /* 2449647844 */
#define DOG_KEEPALIVE_VERS                     0x731fa727 /* 1931454247 */
#define RPC_DOG_KEEPALIVE_BEACON               2
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00010000"
#define HTC_PROCEDURE_SET_VIB_ON_OFF           22
#define PM_LIBVERS                             0x10001

#elif (CONFIG_MSM_AMSS_VERSION == 1170)
#define TIME_REMOTE_MTOA_VERS                  0x10002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00040000"
#define HTC_PROCEDURE_SET_VIB_ON_OFF           22
#define PM_LIBVERS                             0x20001

#elif (CONFIG_MSM_AMSS_VERSION == 1200)
#define TIME_REMOTE_MTOA_VERS                  0x10002
#define DOG_KEEPALIVE_VERS                     0x10001
#define RPC_DOG_KEEPALIVE_BEACON               2
#define APP_TIMEREMOTE_PDEV_NAME               "rs30000048:00040000"
#define HTC_PROCEDURE_SET_VIB_ON_OFF           22
#define PM_LIBVERS                             0x30001
#endif

#endif /*__ASM__ARCH_MSM_RPC_VERSION_H */
