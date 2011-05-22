#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <asm/uaccess.h>

#define BUF_LEN 100

// # insmod wimaxuart

static char *wimaxuart_name=NULL;
module_param(wimaxuart_name,charp,0);

extern int mmc_wimax_uart_switch(int uart);

static struct proc_dir_entry *wimaxuart_proc_file;
int uart_switch = 0;

ssize_t wimaxuart_write(struct file *file, const char *buffer, unsigned long count, void *data)
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
    uart_switch = n;

    printk("%s: uart_switch:%d\n", __func__, uart_switch);
    mmc_wimax_uart_switch(uart_switch);

    return (len);    
}

ssize_t wimaxuart_read(char *buf,char **start,off_t offset,int count,int *eof,void *data)
{
    int len=0;
    
    if(offset>0)
        return 0;
    
    sprintf(buf,"wimaxuart: %d\n", uart_switch);
    
    for(len=0;len<BUF_LEN;len++)
    {
        if(buf[len]=='\0') break;
    }
    
    printk(KERN_INFO "%s: the wimaxuart parameter is %s\n",__func__, buf);

    return len;
}

int wimaxuart_init(void)
{
    wimaxuart_proc_file=create_proc_entry("wimaxuart",S_IRUGO|S_IWUGO,NULL);

    if(!wimaxuart_proc_file)
    {
        printk(KERN_DEBUG "%s: No memory\n", __func__);
        return -ENOMEM;
    }

    wimaxuart_proc_file->read_proc = wimaxuart_read;
    wimaxuart_proc_file->write_proc = wimaxuart_write;

    uart_switch = 0;

    return 0;
}

void wimaxuart_cleanup(void)
{
    printk(KERN_INFO "%s: wimaxuart_module_claen called. Module is now clean\n", __func__);
    remove_proc_entry("wimaxuart",NULL);
}

module_init(wimaxuart_init);
module_exit(wimaxuart_cleanup);

MODULE_DESCRIPTION("HTC wimaxuart for SDIO devices");
MODULE_AUTHOR("HTC");
MODULE_LICENSE("GPL");
