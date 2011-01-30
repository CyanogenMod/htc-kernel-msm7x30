#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <asm/uaccess.h>

extern int mmc_wimax_set_netlog_status(int on);
extern int mmc_wimax_set_cliam_host_status(int on);
// extern int sqn_sdio_get_sdc_clocks(void);
// extern void sqn_sdio_set_sdc_clocks(int on);
extern int mmc_wimax_set_busclk_pwrsave(int on);
extern int mmc_wimax_set_CMD53_timeout_trigger_counter(int counter);
extern int mmc_wimax_get_CMD53_timeout_trigger_counter(void);

extern int sqn_sdio_notify_host_wakeup(void);
extern int mmc_wimax_set_thp_log(int on);
extern int mmc_wimax_set_sdio_hw_reset(int on);
extern int mmc_wimax_set_packet_filter(int on);

extern int mmc_wimax_set_netlog_withraw_status(int on);
extern int mmc_wimax_set_sdio_interrupt_log(int on);

#define BUF_LEN 100

// # insmod wimaxdbg

static char *wimaxdbg_name=NULL;
module_param(wimaxdbg_name,charp,0);

static int dbg_para = 0;
static struct proc_dir_entry *wimaxdbg_proc_file;

ssize_t wimaxdbg_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char buf[16];
    unsigned long len = count;
    int n;
    
    printk(KERN_INFO "%d (%s)\n", (int)len, __func__);
    
    if (len >= sizeof(buf)) 
        len = sizeof(buf) - 1;
    
    if (copy_from_user(buf, buffer, len)) 
        return -EFAULT;
        
    buf[len] = '\0';
    
    n = simple_strtol(buf, NULL, 10);
    dbg_para = n;
    
    printk(KERN_INFO "%s: dbg_parameter:%d\n", __func__, dbg_para);
    if (dbg_para < 2) { // 0: netlog off, 1: netlog on
        printk(KERN_INFO "%s: mmc_wimax_set_netlog_status:%d\n", __func__, dbg_para);
        mmc_wimax_set_netlog_status(dbg_para);
    } 
    else if (dbg_para == 3) { // 3: sdc_clock off
        printk(KERN_INFO "%s: sqn_sdio_set_sdc_clocks:0\n", __func__);
        // sqn_sdio_set_sdc_clocks(0); // Need to insert sequans_sdio.ko first
    }
    else if (dbg_para == 4) { // 4: sdc_clock on
        printk(KERN_INFO "%s: sqn_sdio_set_sdc_clocks:1\n", __func__);
        // sqn_sdio_set_sdc_clocks(1); // Need to insert sequans_sdio.ko first
    }
    else if (dbg_para == 5) { // 5: claim_host debug on
        printk(KERN_INFO "%s: mmc_wimax_set_cliam_host_status:1\n", __func__);
        mmc_wimax_set_cliam_host_status(1);
    }
    else if (dbg_para == 6) { // 6: claim host debug off
        printk(KERN_INFO "%s: mmc_wimax_set_cliam_host_status:0\n", __func__);
        mmc_wimax_set_cliam_host_status(0);
    }
    else if (dbg_para == 7) { // 7: Turn off dynamic SDC CLK OFF
        printk(KERN_INFO "%s: mmc_wimax_set_busclk_pwrsave:0\n", __func__);
        mmc_wimax_set_busclk_pwrsave(0);
    }
    else if (dbg_para == 8) { // 8: Turn on dynamic SDC CLK OFF
        printk(KERN_INFO "%s: mmc_wimax_set_busclk_pwrsave:1\n", __func__);
        mmc_wimax_set_busclk_pwrsave(1);
    }
    else if (dbg_para == 9) { // 9: Disable force CMD53 timeout testing
        printk(KERN_INFO "%s: mmc_wimax_set_CMD53_timeout_trigger_counter:0\n", __func__);
        mmc_wimax_set_CMD53_timeout_trigger_counter(0);
    }
    else if (dbg_para == 10) { // 10: Force CMD53 timeout testing
        printk(KERN_INFO "%s: mmc_wimax_set_CMD53_timeout_trigger_counter:1\n", __func__);
        mmc_wimax_set_CMD53_timeout_trigger_counter(5);
    }
    else if (dbg_para == 11) { // 11: Manually re-send host wakeup
        // printk(KERN_INFO "%s: sqn_sdio_notify_host_wakeup\n", __func__);
        // sqn_sdio_notify_host_wakeup(); // Need to insert sequans_sdio.ko first
    }
    else if (dbg_para == 12) { // 12: Disable THP logging
        printk(KERN_INFO "%s: mmc_wimax_set_thp_log_status:0\n", __func__);
        mmc_wimax_set_thp_log(0);
    }
    else if (dbg_para == 13) { // 13: Enable THP logging
        printk(KERN_INFO "%s: mmc_wimax_set_thp_log_status:1\n", __func__);
        mmc_wimax_set_thp_log(1);
    }
    else if (dbg_para == 14) { // 14: Disable SDIO HW RESET, default is disabled it.
        printk(KERN_INFO "%s: mmc_wimax_set_sdio_hw_reset:0\n", __func__);
        mmc_wimax_set_sdio_hw_reset(0);
    }
    else if (dbg_para == 15) { // 15: Enable SDIO HW RESET
        printk(KERN_INFO "%s: mmc_wimax_set_sdio_hw_reset:1\n", __func__);
        mmc_wimax_set_sdio_hw_reset(1);
    }
    else if (dbg_para == 16) { // 16: Disable SDIO Packet filter
        printk(KERN_INFO "%s: mmc_wimax_set_packet_filter:0\n", __func__);
        mmc_wimax_set_packet_filter(0);
    }
    else if (dbg_para == 17) { // 17: Enable SDIO Packet filter
        printk(KERN_INFO "%s: mmc_wimax_set_packet_filter:1\n", __func__);
        mmc_wimax_set_packet_filter(1);
    }
    else if (dbg_para == 18) { // 18: Disable SDIO GPIO interrupt logging
        printk(KERN_INFO "%s: mmc_wimax_set_sdio_interrupt_log:0\n", __func__);
        mmc_wimax_set_sdio_interrupt_log(0);
    }
    else if (dbg_para == 19) { // 19: Enable SDIO GPIO interrupt logging
        printk(KERN_INFO "%s: mmc_wimax_set_sdio_interrupt_log:1\n", __func__);
        mmc_wimax_set_sdio_interrupt_log(1);
    }
    else if (dbg_para == 20) { // 20: Disable dumping raw data for network packets
        printk(KERN_INFO "%s: mmc_wimax_set_netlog_withraw_status:0\n", __func__);
        mmc_wimax_set_netlog_withraw_status(0);
    }
    else if (dbg_para == 21) { // 21: Enable dumping raw data for network packets
        printk(KERN_INFO "%s: mmc_wimax_set_netlog_withraw_status:1\n", __func__);
        mmc_wimax_set_netlog_withraw_status(1);
    }
    else {
        printk(KERN_INFO "%s: None function:%d\n", __func__, dbg_para);
    }

    return (len);    
}

ssize_t wimaxdbg_read(char *buf,char **start,off_t offset,int count,int *eof,void *data)
{
    int len=0;
    
    if(offset>0)
        return 0;
    
    /*
    sprintf(buf,"wimxdbg: %d\nsdcclk:%d\n", dbg_para, sqn_sdio_get_sdc_clocks());
    
    for(len=0;len<BUF_LEN;len++)
    {
        if(buf[len]=='\0') break;
    }
    
    printk(KERN_INFO "%s: wimaxdbg:%s\n",__func__, buf);
    printk(KERN_INFO "%s: sdcclk:%d\n", __func__, sqn_sdio_get_sdc_clocks());
    */

    return len;
}

int wimaxdbg_init(void)
{
    wimaxdbg_proc_file=create_proc_entry("wimaxdbg",S_IRUGO|S_IWUGO,NULL);

    if(!wimaxdbg_proc_file)
    {
        printk(KERN_DEBUG "%s: No memory\n", __func__);
        return -ENOMEM;
    }

    wimaxdbg_proc_file->read_proc = wimaxdbg_read;
    wimaxdbg_proc_file->write_proc = wimaxdbg_write;

    dbg_para = 0;

    return 0;
}

void wimaxdbg_cleanup(void)
{
    printk(KERN_INFO "%s: mimaxdbg_module_claen called. Module is now clean\n", __func__);
    remove_proc_entry("wimaxdbg",NULL);
}

module_init(wimaxdbg_init);
module_exit(wimaxdbg_cleanup);

MODULE_DESCRIPTION("HTC wimaxdbg for SDIO devices");
MODULE_AUTHOR("HTC");
MODULE_LICENSE("GPL");
