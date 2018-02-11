// Author: Ahmet SOYYIGIT, ahmedius2@gmail.com

// A simple PL011 driver (Raspberry Pi UART) using RTDM (Real Time Driver Model)
// Interrupt driven

// I took bcm2835 and ringbuffer libraries from internet


#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include <rtdm/rtdm.h>
#include <rtdm/rtdm_driver.h>

#include "bcm2835.h"
#include "pl011-uart.h"
#include "ringbuf.h"

#define DEVICE_NAME    "pl011_rtdm01"

#define SOME_SUB_CLASS 4711

extern uint32_t *bcm2835_peripherals;
uint32_t *bcm2835_peripherals_base_mapped;

rtdm_irq_t pl011_irq_handle;
ringBufS circBuf;
rtdm_lock_t receiveBufferLock;

/**
 * The context of a device instance
 *
 * A context is created each time a device is opened and passed to
 * other device handlers when they are called.
 *
 */


 int pl011_irq_handler(rtdm_irq_t *irq_handle){
 	char ch;
 	int i;
 	rtdm_lockctx_t intrStatus;

 	bcm2835_peri_write(bcm2835_peripherals + UART0_ICR,0x7FF); // clear int flag
 	// by writing 1, it clears the flag and make it 0

 	// write mis ris icr imsc
 	/*sprintf(uartBuf, "MIS:0x%08lx RIS:0x%08lx ICR:0x%08lx IMSC:0x%08lx\r\n",
 		bcm2835_peri_read(bcm2835_peripherals + UART0_MIS),
 		bcm2835_peri_read(bcm2835_peripherals + UART0_RIS),
 		bcm2835_peri_read(bcm2835_peripherals + UART0_ICR),
 		bcm2835_peri_read(bcm2835_peripherals + UART0_IMSC));
 	for(i=0 ; i<strlen(uartBuf) ; ++i){
	    while(bcm2835_peri_read(bcm2835_peripherals + UART0_FR) & 0x20);
	    bcm2835_peri_write(bcm2835_peripherals + UART0_DR,uartBuf[i]);
	}*/

	rtdm_lock_get_irqsave(&receiveBufferLock, intrStatus);
	// flush fifo
	for(i=0; i<8 ; ++i){
	    while(bcm2835_peri_read(bcm2835_peripherals + UART0_FR) & 0x10);
	    ch = bcm2835_peri_read(bcm2835_peripherals + UART0_DR);
	    // put ch to circular buffer, if full, overwrite data
	    if(ringBufS_full(&circBuf))
	    	ringBufS_get(&circBuf);
	    ringBufS_put(&circBuf,ch);
	}
	rtdm_lock_put_irqrestore(&receiveBufferLock, intrStatus);

 	return 0;
 }

/**
 * Open the device
 *
 * This function is called when the device shall be opened.
 *
 */
static int pl011_open_nrt(struct rtdm_dev_context *context,
				rtdm_user_info_t * user_info, int oflags)
{
	return 0;
}

/**
 * Close the device
 *
 * This function is called when the device shall be closed.
 *
 */
static int pl011_close_nrt(struct rtdm_dev_context *context,
				 rtdm_user_info_t * user_info)
{
	return 0;
}

/**
* ioctl to device 
*
* non real time
*
*/
static int pl011_ioctl_rt(struct rtdm_dev_context *context,
				     rtdm_user_info_t * user_info,
				     unsigned int request, void __user *arg)
{

	return 0;
}

/**
 * Read from the device
 *
 * This function is called when the device is read in non-realtime context.
 *
 */
static ssize_t pl011_read_rt(struct rtdm_dev_context *context,
				    rtdm_user_info_t * user_info, void *buf,
				    size_t nbyte)
{
	int i;
	char *uartBuf;
	rtdm_lockctx_t intrStatus;

	uartBuf = rtdm_malloc(nbyte);

	rtdm_lock_get_irqsave(&receiveBufferLock, intrStatus);
	for(i=0 ; i<nbyte ; ++i){
		// BUSY WAITING 
	    while(bcm2835_peri_read(bcm2835_peripherals + UART0_FR) & 0x10);
	    uartBuf[i] = bcm2835_peri_read(bcm2835_peripherals + UART0_DR);
	}
	rtdm_lock_put_irqrestore(&receiveBufferLock, intrStatus);

	if (rtdm_safe_copy_to_user(user_info, buf, uartBuf, nbyte))
		rtdm_printk("ERROR : can't copy data to driver pl011_read_rt\n");

    rtdm_free(uartBuf);

	return nbyte;
}

/**
 * Write in the device
 *
 * This function is called when the device is written in non-realtime context.
 *
 */
static ssize_t pl011_write_rt(struct rtdm_dev_context *context,
				     rtdm_user_info_t * user_info,
				     const void *buf, size_t nbyte)
{
	int i;
	char *uartBuf;
	rtdm_lockctx_t intrStatus;

	uartBuf = rtdm_malloc(nbyte);
	if(uartBuf == NULL)
		rtdm_printk("ERROR : pl011 write malloc\n");

	if (rtdm_safe_copy_from_user(user_info, uartBuf, buf, nbyte))
		rtdm_printk("ERROR : can't copy data to driver pl011_write_rt\n");

	rtdm_lock_get_irqsave(&receiveBufferLock, intrStatus);
	for(i=0 ; i<nbyte ; ++i){
	    while(bcm2835_peri_read(bcm2835_peripherals + UART0_FR) & 0x20);
	    bcm2835_peri_write(bcm2835_peripherals + UART0_DR,uartBuf[i]);
	}
	rtdm_lock_put_irqrestore(&receiveBufferLock, intrStatus);
	
    rtdm_free(uartBuf);

	return nbyte;
}

/**
 * This structure describe the simple RTDM device
 *
 */
static struct rtdm_device device = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,

	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = 0,
	.device_name = DEVICE_NAME,

	.open_nrt = pl011_open_nrt,

	.ops = {
		.close_nrt = pl011_close_nrt,
		.ioctl_rt = pl011_ioctl_rt,
		.read_rt = pl011_read_rt,
		.write_rt = pl011_write_rt,
	},

	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = SOME_SUB_CLASS,
	.profile_version = 1,
	.driver_name = "pl011_rtdm",
	.driver_version = RTDM_DRIVER_VER(0, 1, 2),
	.peripheral_name = "bcm2835_pl011",
	.provider_name = "ahmet",
	.proc_name = device.device_name,
};

/**
 * This function is called when the module is loaded
 *
 * It simply registers the RTDM device.
 *
 */
int __init pl011_init(void)
{
	int res;

	rtdm_lock_init(&receiveBufferLock);

	ringBufS_init(&circBuf);

	/* Initialize the bcm2835 library , use ioremap*/
	res = bcm2835_init();
	if (res != 1) {
		rtdm_printk( "%s: Error in bcm2835_init (%d).\r\n", __FUNCTION__, res);
		return -1;
	}
	// do this so spi driver can access this address
	bcm2835_peripherals_base_mapped = bcm2835_peripherals;

    //Set pins for pl011 UART and configure
    bcm2835_peri_write(bcm2835_peripherals + UART0_CR, 0); // disable uart before configuration
    bcm2835_gpio_fsel(RPI_GPIO_P1_08, BCM2835_GPIO_FSEL_ALT0); // configure pin as uart tx
    bcm2835_gpio_fsel(RPI_GPIO_P1_10, BCM2835_GPIO_FSEL_ALT0); // configure pin as uart rx

    // Disable pull up / pull down
    bcm2835_peri_write(bcm2835_peripherals + GPPUD, 0);
    // I need to wait 150 cycles here
    rtdm_task_busy_sleep(150000);
	bcm2835_peri_write(bcm2835_peripherals + GPPUDCLK0, (1<<14)|(1<<15));
    // I need to wait 150 cycles here again
    rtdm_task_busy_sleep(150000);
    bcm2835_peri_write(bcm2835_peripherals + GPPUDCLK0, 0);

    // Wait for uart to finish and flush transmit fifo
    while(bcm2835_peri_read(bcm2835_peripherals + UART0_FR) & 0x8);
    bcm2835_peri_write(bcm2835_peripherals + UART0_LCRH,0);

    bcm2835_peri_write(bcm2835_peripherals + UART0_IBRD,19);   // Set baut rate to 9600
    bcm2835_peri_write(bcm2835_peripherals + UART0_FBRD,53);   // Set baut rate to 9600
    bcm2835_peri_write(bcm2835_peripherals + UART0_LCRH,0x70); // FIFO is enabled, word length is 8 bit
    
    // set receive interrupt when fifo is 1/2 full
    bcm2835_peri_write(bcm2835_peripherals + UART0_IFLS, 0x10);
    bcm2835_peri_write(bcm2835_peripherals + UART0_IMSC, 0x10); // set RXIM bit, unmask it
    bcm2835_peri_write(bcm2835_peripherals + UART0_ICR,0x7FF);  // Clear all interrupts

    bcm2835_peri_write(bcm2835_peripherals + UART0_CR,0x301);  // Enable uart, enable receive and transmit

    res = rtdm_dev_register(&device);

    rtdm_irq_request (&pl011_irq_handle, 83, &pl011_irq_handler, 
    	RTDM_IRQTYPE_SHARED, DEVICE_NAME, NULL);

    if(res != 0){
    	rtdm_printk("COULD NOT REGISTER RTDM pl011 DEVICE\n");
    }

    rtdm_printk("pl011 rtdm driver initialized successfully.\n");

	return res;
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
void __exit pl011_exit(void)
{
	rtdm_printk("Exiting pl011 rtdm driver\n");
	rtdm_irq_free(&pl011_irq_handle);
	rtdm_dev_unregister(&device, 1000);
	bcm2835_close();
	ringBufS_free(&circBuf);
}

// Functions for driver stacking
/**
 * Read data from 
 */
static ssize_t pl011_get_buffer_count_kernel(void)
{
	return circBuf.count;
}

/**
 * Read from the device
 *
 * This function is called when the device is read in non-realtime context.
 *
 */
static ssize_t pl011_read_rt_kernel_nonblocking(void *buf, size_t nbyte)
{
	int i;
	char ch;
	char *cpbuf = (char*)buf;
	rtdm_lockctx_t intrStatus;

	rtdm_lock_get_irqsave(&receiveBufferLock, intrStatus);
	// flush fifo
	for(;;){
	    if(bcm2835_peri_read(bcm2835_peripherals + UART0_FR) & 0x10)
	    	break;
	    ch = bcm2835_peri_read(bcm2835_peripherals + UART0_DR);
	    // put ch to circular buffer, if full, overwrite data
	    if(ringBufS_full(&circBuf))
	    	ringBufS_get(&circBuf);
	    ringBufS_put(&circBuf,ch);
	    // this time is enough, i think... ? maybe I can delay
	}

	for(i=0 ; i<nbyte ; ++i){
		if(!ringBufS_empty(&circBuf))
	    	cpbuf[i] = ringBufS_get(&circBuf);
	    else
	    	break;
	}
	rtdm_lock_put_irqrestore(&receiveBufferLock, intrStatus);

	return i;
}

EXPORT_SYMBOL(bcm2835_peripherals_base_mapped);
EXPORT_SYMBOL(pl011_get_buffer_count_kernel);
EXPORT_SYMBOL(pl011_read_rt_kernel_nonblocking);

module_init(pl011_init);
module_exit(pl011_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ahmet");
