// divide by 4 because adding with pointer

#define GPFSEL1 0x00200004/4
#define GPSET0  0x0020001C/4// /4 ?
#define GPCLR0  0x00200028/4
#define GPPUD       0x00200094/4
#define GPPUDCLK0   0x00200098/4

//main registers
#define UART0_BASE_PL011   0x00201000
#define UART0_DR     (UART0_BASE_PL011+0x00)/4
#define UART0_RSRECR (UART0_BASE_PL011+0x04)/4
#define UART0_FR     (UART0_BASE_PL011+0x18)/4
#define UART0_ILPR   (UART0_BASE_PL011+0x20)/4
#define UART0_IBRD   (UART0_BASE_PL011+0x24)/4
#define UART0_FBRD   (UART0_BASE_PL011+0x28)/4
#define UART0_LCRH   (UART0_BASE_PL011+0x2C)/4
#define UART0_CR     (UART0_BASE_PL011+0x30)/4

// interrupt registers
#define UART0_IFLS   (UART0_BASE_PL011+0x34)/4 // fifo
#define UART0_IMSC   (UART0_BASE_PL011+0x38)/4 // write 1 to unmask(enable) interrupts
#define UART0_RIS    (UART0_BASE_PL011+0x3C)/4 // raw interrupt status
#define UART0_MIS    (UART0_BASE_PL011+0x40)/4 // masked interrupt status
#define UART0_ICR    (UART0_BASE_PL011+0x44)/4 // Interrupt clear register, write 1 to clear

//dma blablabla
#define UART0_DMACR  (UART0_BASE_PL011+0x48)/4
#define UART0_ITCR   (UART0_BASE_PL011+0x80)/4
#define UART0_ITIP   (UART0_BASE_PL011+0x84)/4
#define UART0_ITOP   (UART0_BASE_PL011+0x88)/4
#define UART0_TDR    (UART0_BASE_PL011+0x8C)/4
