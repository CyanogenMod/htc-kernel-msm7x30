/*
 *	Abstract:
 *		Debug Flags for SDIO_AL
 *
 *	Working environment:
 *		Android/LTE project
 *
 *	Referenced documents:
 *		N/A
 *
 *	Revision history:
 *		Trial18NOV20100 --Bert Lin--
 */

#if 1   /* HTC */
/* 1 */
#define DBG_SDIO_AL_L1_ALLOG        (1<<(0*4+0)) /*** al_log - reserved ***/
#define DBG_SDIO_AL_L2_INFO         (1<<(0*4+1)) /*** al_info - reserved ***/
#define DBG_SDIO_AL_L3_ALDBG        (1<<(0*4+2)) /* al_debug */
#define DBG_SDIO_AL_L4_ALDBG        (1<<(0*4+3)) /* al_verbose */

/* 2 */
/* DBG_SDIO_AL_XXX_LOG */
#define DBG_SDIO_AL_RPC_LOG         (1<<(1*4+0)) /* al_rpc_dbg */
#define DBG_SDIO_AL_RMNET_LOG       (1<<(1*4+1)) /* al_rmnet_dbg */
#define DBG_SDIO_AL_QMI_LOG         (1<<(1*4+2)) /* al_qmi_dbg */
#define DBG_SDIO_AL_DIAG_LOG        (1<<(1*4+3)) /* al_diag_dbg */

/* 3 */
#define DBG_SDIO_AL_RPC_RAW_DATA	(1<<(2*4+0))
#define DBG_SDIO_AL_RMNET_RAW_DATA	(1<<(2*4+1))
#define DBG_SDIO_AL_QMI_RAW_DATA	(1<<(2*4+2))
#define DBG_SDIO_AL_DIAG_RAW_DATA	(1<<(2*4+3))

/* 4 */
#define DBG_SDIO_AL_ALL_RAW_DATA    (1<<(3*4+0))
#define DBG_SDIO_AL_TXRX_SIZE_LOG   (1<<(3*4+1)) /* depend on DBG_SDIO_AL_XXX_LOG series flags */
#define DBG_SDIO_QMI_DBG            (1<<(3*4+2)) /* arch/arm/mach-msm/lte/sdio_ctl.c D D_DUMP_BUFFER */
#define DBG_SDIO_RMNET_RAW_DATA     (1<<(3*4+3)) /* drivers/net/lte/msm_rmnet_sdio.c dbg_dump_buf */

/* 5 */
#define DBG_SDIO_RPC_DBG            (1<<(4*4+0)) /* SDIO_XPRT_DBG */
#define DBG_SDIO_RPC_INFO           (1<<(4*4+1)) /* SDIO_XPRT_INFO */
#define DBG_SDIO_RPC_RAW_DATA       (1<<(4*4+2)) /* arch/arm/mach-msm/lte/rpcrouter_sdio_xprt.c dbg_dump_buf */
#define DBG_SDIO_RMNET_DBG			(1<<(4*4+3)) /* drivers/net/lte/msm_rmnet_sdio.c DBG, DBG_INC_READ_CNT, ... */

/* 6 */
#define DBG_SDIO_QMI_RAW_DATA		(1<<(5*4+0)) /* arch/arm/mach-msm/lte/sdio_ctl.c dbg_dump_buf*/

#endif /* HTC_END */

/* note:
 *		if you want use DBG_SDIO_AL_TXRX_SIZE_LOG,
 *		you have to open DBG_SDIO_AL_RPC_LOG flag to print RPC [TX] size & [RX] size,
 *		and so forth.
 *
 *		rpc   	- original 30000
 *		rmnet 	- original 80000
 *		qmi   	- original 4000
 *		all	  	- original b4000
 *		sdioal	- original 4
 *		sdioal+rpc+rmnet+qmi - b4004
 *		sdioal_all+rpc+rmnet+qmi - b400f
 *		rpc_raw	- 40100
 *		rmnet_raw - 8400
 *		sdioal_raw - f00
 *		sdioal_raw + sdio_size - 2ff0
 *		sdioal_raw + sdio_size + rpc_raw + rmnet_raw - 148ff0
 *		rpc_raw + rpc_dbg + sdioal_rpc_raw - 70100
 *		qmi_raw + qmi_dbg + sdioal_qmi_raw - 104400
 *		rmnet_raw + rmnet_dbg + sdioal_rmnet_raw - 88200
 *
 */

