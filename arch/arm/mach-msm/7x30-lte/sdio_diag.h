#ifndef SDIODIAG_H
#define SDIODIAG_H


extern int sdio_diag_initialized;
void  msm_sdio_diag_write(void *data, int len);
int __init sdio_diag_init(void);

#endif
