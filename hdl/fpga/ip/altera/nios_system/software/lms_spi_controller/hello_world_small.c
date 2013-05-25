/*
 * "Small Hello World" example.
 *
 * This example prints 'Hello from Nios II' to the STDOUT stream. It runs on
 * the Nios II 'standard', 'full_featured', 'fast', and 'low_cost' example
 * designs. It requires a STDOUT  device in your system's hardware.
 *
 * The purpose of this example is to demonstrate the smallest possible Hello
 * World application, using the Nios II HAL library.  The memory footprint
 * of this hosted application is ~332 bytes by default using the standard
 * reference design.  For a more fully featured Hello World application
 * example, see the example titled "Hello World".
 *
 * The memory footprint of this example has been reduced by making the
 * following changes to the normal "Hello World" example.
 * Check in the Nios II Software Developers Manual for a more complete
 * description.
 *
 * In the SW Application project (small_hello_world):
 *
 *  - In the C/C++ Build page
 *
 *    - Set the Optimization Level to -Os
 *
 * In System Library project (small_hello_world_syslib):
 *  - In the C/C++ Build page
 *
 *    - Set the Optimization Level to -Os
 *
 *    - Define the preprocessor option ALT_NO_INSTRUCTION_EMULATION
 *      This removes software exception handling, which means that you cannot
 *      run code compiled for Nios II cpu with a hardware multiplier on a core
 *      without a the multiply unit. Check the Nios II Software Developers
 *      Manual for more details.
 *
 *  - In the System Library page:
 *    - Set Periodic system timer and Timestamp timer to none
 *      This prevents the automatic inclusion of the timer driver.
 *
 *    - Set Max file descriptors to 4
 *      This reduces the size of the file handle pool.
 *
 *    - Check Main function does not exit
 *    - Uncheck Clean exit (flush buffers)
 *      This removes the unneeded call to exit when main returns, since it
 *      won't.
 *
 *    - Check Don't use C++
 *      This builds without the C++ support code.
 *
 *    - Check Small C library
 *      This uses a reduced functionality C library, which lacks
 *      support for buffering, file IO, floating point and getch(), etc.
 *      Check the Nios II Software Developers Manual for a complete list.
 *
 *    - Check Reduced device drivers
 *      This uses reduced functionality drivers if they're available. For the
 *      standard design this means you get polled UART and JTAG UART drivers,
 *      no support for the LCD driver and you lose the ability to program
 *      CFI compliant flash devices.
 *
 *    - Check Access device drivers directly
 *      This bypasses the device file system to access device drivers directly.
 *      This eliminates the space required for the device file system services.
 *      It also provides a HAL version of libc services that access the drivers
 *      directly, further reducing space. Only a limited number of libc
 *      functions are available in this configuration.
 *
 *    - Use ALT versions of stdio routines:
 *
 *           Function                  Description
 *        ===============  =====================================
 *        alt_printf       Only supports %s, %x, and %c ( < 1 Kbyte)
 *        alt_putstr       Smaller overhead than puts with direct drivers
 *                         Note this function doesn't add a newline.
 *        alt_putchar      Smaller overhead than putchar with direct drivers
 *        alt_getchar      Smaller overhead than getchar with direct drivers
 *
 */

#include "sys/alt_stdio.h"
#include "system.h"
#include "io.h"
#include "altera_avalon_spi.h"
#include "altera_avalon_uart_regs.h"
#include "altera_avalon_jtag_uart_regs.h"
#include "altera_avalon_pio_regs.h"
#include "priv/alt_busy_sleep.h"
#include "priv/alt_file.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "sys/alt_dev.h"

#include "lms_spi_config.h"

xcvr_config_t xcvr_config;


//
//volatile uint16_t * const OC_I2C_PRESCALER = (uint16_t *)OC_I2C_MASTER_0_BASE ;
//volatile uint8_t * const OC_I2C_DATA = (uint8_t *)OC_I2C_MASTER_0_BASE + 3 ;
//volatile uint8_t * const OC_I2C_CTRL = (uint8_t *)OC_I2C_MASTER_0_BASE + 2 ;
//volatile uint8_t * const OC_I2C_CMD_STATUS = (uint8_t *)OC_I2C_MASTER_0_BASE + 4 ;

// Register offsets from the base
#define I2C 				OC_I2C_MASTER_0_BASE
#define OC_I2C_PRESCALER 	0
#define OC_I2C_CTRL 		2
#define OC_I2C_DATA 		3
#define OC_I2C_CMD_STATUS 	4

#define SI5338_I2C 			(0xE0)
#define OC_I2C_ENABLE 		(1<<7)
#define OC_I2C_STA 			(1<<7)
#define OC_I2C_STO 			(1<<6)
#define OC_I2C_WR  			(1<<4)
#define OC_I2C_RD 			(1<<5)
#define OC_I2C_TIP 			(1<<1)
#define OC_I2C_RXACK 		(1<<7)
#define OC_I2C_NACK 		(1<<3)

void si5338_complete_transfer( uint8_t check_rxack ) {
	if( (IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_TIP) == 0 ) {
		while( (IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_TIP) == 0 ) { } ;
	}
	while( IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_TIP ) { } ;
	while( check_rxack && (IORD_8DIRECT(I2C, OC_I2C_CMD_STATUS)&OC_I2C_RXACK) ) { } ;
}

void si5338_read( uint8_t addr, uint8_t *data ) {

	// Set the address to the Si5338
	IOWR_8DIRECT(I2C, OC_I2C_DATA, SI5338_I2C ) ;
	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_STA | OC_I2C_WR ) ;
	si5338_complete_transfer( 1 ) ;

	IOWR_8DIRECT(I2C, OC_I2C_DATA, addr ) ;
	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_WR | OC_I2C_STO ) ;
	si5338_complete_transfer( 1 ) ;

	// Next transfer is a read operation, so '1' in the read/write bit
	IOWR_8DIRECT(I2C, OC_I2C_DATA, SI5338_I2C | 1 ) ;
	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_STA | OC_I2C_WR ) ;
	si5338_complete_transfer( 1 ) ;

	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_RD | OC_I2C_NACK | OC_I2C_STO ) ;
	si5338_complete_transfer( 0 ) ;

	*data = IORD_8DIRECT(I2C, OC_I2C_DATA) ;
	return ;
}

void si5338_write( uint8_t addr, uint8_t data ) {

	// Set the address to the Si5338
	IOWR_8DIRECT(I2C, OC_I2C_DATA, SI5338_I2C) ;
	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_STA | OC_I2C_WR ) ;
	si5338_complete_transfer( 1 ) ;

	IOWR_8DIRECT(I2C, OC_I2C_DATA, addr) ;
	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_CMD_STATUS | OC_I2C_WR ) ;
	si5338_complete_transfer( 1 ) ;

	IOWR_8DIRECT(I2C, OC_I2C_DATA, data ) ;
	IOWR_8DIRECT(I2C, OC_I2C_CMD_STATUS, OC_I2C_WR | OC_I2C_STO ) ;
	si5338_complete_transfer( 0 ) ;

	return ;
}

void bladerf_lms_reset() {
	uint32_t gpio = IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE) ;
	gpio &= ~1 ;
	IOWR_ALTERA_AVALON_PIO_DATA(PIO_0_BASE, gpio) ;
	gpio |= 1 ;
	IOWR_ALTERA_AVALON_PIO_DATA(PIO_0_BASE, gpio) ;
	return ;
}

#define BAND_MASK   3
#define BAND_LOW 	2
#define BAND_HIGH 	1

void bladerf_tx_enable() {
	IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_0_BASE,4) ;
	lms_tx_enable() ;
	return ;
}

void bladerf_tx_disable() {
	IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_0_BASE,4) ;
	lms_tx_disable() ;
	return ;
}

void bladerf_rx_enable() {
	IOWR_ALTERA_AVALON_PIO_SET_BITS(PIO_0_BASE,2) ;
	lms_rx_enable() ;
	return ;
}

void bladerf_rx_disable() {
	IOWR_ALTERA_AVALON_PIO_CLEAR_BITS(PIO_0_BASE,2) ;
	lms_rx_disable() ;
	return ;
}

void bladerf_set_frequency( lms_module_t mod, uint32_t freq ) {
	uint32_t band = (freq > 1500000000) ? BAND_HIGH : BAND_LOW ;
	uint32_t gpio = IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE) ;
	gpio &= (mod == TX) ? ~(BAND_MASK << 3) : ~(BAND_MASK << 5) ;
	gpio |= (mod == TX) ? band << 3 : band << 5 ;
	IOWR_ALTERA_AVALON_PIO_DATA(PIO_0_BASE, gpio) ;
	lms_set_frequency( mod, freq ) ;
	if( mod == TX ) {
		lms_pa_enable( freq > 1500000000 ? PA_2 : PA_1 ) ;
	}
	return ;
}

// Entry point
int main()
{
  uint8_t data ;
  alt_putstr("bladeRF LMS6002D SPI Register Readback!\n");
  alt_putstr("---------------------------------------\n");
  bladerf_lms_reset() ;
  bladerf_tx_enable() ;
  bladerf_rx_disable() ;

//  lms_dump_registers() ;

//  xcvr_config.tx_freq_hz = 576000000u;
//  xcvr_config.rx_freq_hz = 1576000000u;

//  xcvr_config.loopback_mode = LB_NONE;
//  xcvr_config.lna = LNA_NONE;
//  xcvr_config.pa = PA_2;
//  xcvr_config.bw = BW_28MHz ;

//  lms_config_init(&xcvr_config);
//  lms_lpf_enable( TX, BW_28MHz ) ;
//  lms_set_txvga2_gain( 25 ) ;
//  lms_spi_write( 0x57, 0x84 ) ;
//  lms_dump_registers() ;

  // 500MHz out of PA1
//  lms_spi_write( 0X82, 0x1F ) ;
//  lms_spi_write( 0X83, 0x08 ) ;
//  lms_spi_write( 0X85, 0x3E ) ;
//  lms_spi_write( 0X86, 0x0D ) ;
//  lms_spi_write( 0X87, 0x00 ) ;
//  lms_spi_write( 0X88, 0x00 ) ;
//  lms_spi_write( 0X89, 0x45 ) ;
//  lms_spi_write( 0X8A, 0x00 ) ;
//  lms_spi_write( 0X8B, 0x08 ) ;
//  lms_spi_write( 0X90, 0x34 ) ;
//  lms_spi_write( 0X91, 0x15 ) ;
//  lms_spi_write( 0X92, 0x55 ) ;
//  lms_spi_write( 0X93, 0x55 ) ;
//  lms_spi_write( 0X94, 0x88 ) ;
//  lms_spi_write( 0X95, 0x99 ) ;
//  lms_spi_write( 0X96, 0x8C ) ;
//  lms_spi_write( 0X97, 0xE3 ) ;
//  lms_spi_write( 0X98, 0x40 ) ;
//  lms_spi_write( 0X99, 0x99 ) ;
//  lms_spi_write( 0X9A, 0x03 ) ;
//  lms_spi_write( 0X9B, 0x76 ) ;
//  lms_spi_write( 0X9C, 0x38 ) ;
//  lms_spi_write( 0XA0, 0x34 ) ;
//  lms_spi_write( 0XA1, 0x15 ) ;
//  lms_spi_write( 0XA2, 0x55 ) ;
//  lms_spi_write( 0XA3, 0x55 ) ;
//  lms_spi_write( 0XA4, 0x88 ) ;
//  lms_spi_write( 0XA5, 0x99 ) ;
//  lms_spi_write( 0XA6, 0x8C ) ;
//  lms_spi_write( 0XA7, 0xE3 ) ;
//  lms_spi_write( 0XA8, 0x40 ) ;
//  lms_spi_write( 0XA9, 0x9F ) ;
//  lms_spi_write( 0XAA, 0x03 ) ;
//  lms_spi_write( 0XAB, 0x76 ) ;
//  lms_spi_write( 0XAC, 0x38 ) ;
//  lms_spi_write( 0XB2, 0x1F ) ;
//  lms_spi_write( 0XB3, 0x08 ) ;
//  lms_spi_write( 0XB4, 0x02 ) ;
//  lms_spi_write( 0XB5, 0x0C ) ; // 0x0C for normal LPF
//  lms_spi_write( 0XB6, 0x30 ) ;
//  lms_spi_write( 0XC0, 0x02 ) ;
//  lms_spi_write( 0XC1, 0x1F ) ;
//  lms_spi_write( 0XC2, 0x80 ) ;
//  lms_spi_write( 0XC3, 0x80 ) ;
//  lms_spi_write( 0XC4, 0x0B ) ;
//  lms_spi_write( 0XC5, 0xC8 ) ;
//  lms_spi_write( 0XC6, 0x00 ) ;
//  lms_spi_write( 0XC7, 0x40 ) ;
//  lms_spi_write( 0XC8, 0x0C ) ;
//  lms_spi_write( 0XC9, 0x0C ) ;
//  lms_spi_write( 0XCA, 0x18 ) ;
//  lms_spi_write( 0XCB, 0x50 ) ;
//  lms_spi_write( 0XCC, 0x00 ) ;
//  lms_spi_write( 0XCD, 0x00 ) ;
//  lms_spi_write( 0XD2, 0x1F ) ;
//  lms_spi_write( 0XD3, 0x08 ) ;
//  lms_spi_write( 0XD4, 0x02 ) ;
//  lms_spi_write( 0XD5, 0x0C ) ;
//  lms_spi_write( 0XD6, 0x30 ) ;
//  lms_spi_write( 0XD7, 0x94 ) ;
//  lms_spi_write( 0XD8, 0x00 ) ;
//  lms_spi_write( 0XD9, 0x09 ) ;
//  lms_spi_write( 0XDA, 0x20 ) ;
//  lms_spi_write( 0XDB, 0x00 ) ;
//  lms_spi_write( 0XDC, 0x00 ) ;
//  lms_spi_write( 0XDD, 0x00 ) ;
//  lms_spi_write( 0XDE, 0x00 ) ;
//  lms_spi_write( 0XDF, 0x1F ) ;
//  lms_spi_write( 0XE2, 0x1F ) ;
//  lms_spi_write( 0XE3, 0x08 ) ;
//  lms_spi_write( 0XE4, 0x32 ) ;
//  lms_spi_write( 0XE5, 0x01 ) ;
//  lms_spi_write( 0XE6, 0x00 ) ;
//  lms_spi_write( 0XE7, 0x00 ) ;
//  lms_spi_write( 0XE8, 0x01 ) ;
//  lms_spi_write( 0XF0, 0x01 ) ;
//  lms_spi_write( 0XF1, 0x80 ) ;
//  lms_spi_write( 0XF2, 0x80 ) ;
//  lms_spi_write( 0XF3, 0x00 ) ;
//  lms_spi_write( 0XF4, 0x00 ) ;
//  lms_spi_write( 0XF5, 0x50 ) ;
//  lms_spi_write( 0XF6, 0x02 ) ; // 0x78
//  lms_spi_write( 0XF7, 0x00 ) ;
//  lms_spi_write( 0XF8, 0x1C ) ;
//  lms_spi_write( 0XF9, 0x37 ) ;
//  lms_spi_write( 0XFA, 0x77 ) ;
//  lms_spi_write( 0XFB, 0x77 ) ;
//  lms_spi_write( 0XFC, 0x18 ) ;
//  lms_spi_write( 0XFD, 0x00 ) ;

  lms_spi_write( 0X82, 0x1F ) ;
  lms_spi_write( 0X83, 0x08 ) ;
  lms_spi_write( 0X85, 0x3E ) ;
  lms_spi_write( 0X86, 0x0D ) ;
  lms_spi_write( 0X87, 0x00 ) ;
  lms_spi_write( 0X88, 0x00 ) ;
  lms_spi_write( 0X89, 0x45 ) ;
  lms_spi_write( 0X8A, 0x00 ) ;
  lms_spi_write( 0X8B, 0x08 ) ;
  lms_spi_write( 0X90, 0x34 ) ;
  lms_spi_write( 0X91, 0x15 ) ;
  lms_spi_write( 0X92, 0x55 ) ;
  lms_spi_write( 0X93, 0x55 ) ;
  lms_spi_write( 0X94, 0x88 ) ;
  lms_spi_write( 0X95, 0x99 ) ;
  lms_spi_write( 0X96, 0x8C ) ;
  lms_spi_write( 0X97, 0xE3 ) ;
  lms_spi_write( 0X98, 0x40 ) ;
  lms_spi_write( 0X99, 0x92 ) ;
  lms_spi_write( 0X9A, 0x03 ) ;
  lms_spi_write( 0X9B, 0x76 ) ;
  lms_spi_write( 0X9C, 0x38 ) ;
  lms_spi_write( 0XA0, 0x34 ) ;
  lms_spi_write( 0XA1, 0x15 ) ;
  lms_spi_write( 0XA2, 0x55 ) ;
  lms_spi_write( 0XA3, 0x55 ) ;
  lms_spi_write( 0XA4, 0x88 ) ;
  lms_spi_write( 0XA5, 0x99 ) ;
  lms_spi_write( 0XA6, 0x8C ) ;
  lms_spi_write( 0XA7, 0xE3 ) ;
  lms_spi_write( 0XA8, 0x40 ) ;
  lms_spi_write( 0XA9, 0x92 ) ;
  lms_spi_write( 0XAA, 0x03 ) ;
  lms_spi_write( 0XAB, 0x76 ) ;
  lms_spi_write( 0XAC, 0x38 ) ;
  lms_spi_write( 0XB2, 0x1F ) ;
  lms_spi_write( 0XB3, 0x08 ) ;
  lms_spi_write( 0XB4, 0x02 ) ;
  lms_spi_write( 0XB5, 0x4C ) ;
  lms_spi_write( 0XB6, 0x30 ) ;
  lms_spi_write( 0XC0, 0x02 ) ;
  lms_spi_write( 0XC1, 0x1F ) ;
  lms_spi_write( 0XC2, 0x80 ) ;
  lms_spi_write( 0XC3, 0x80 ) ;
  lms_spi_write( 0XC4, 0x0B ) ;
  lms_spi_write( 0XC5, 0xC8 ) ;
  lms_spi_write( 0XC6, 0x00 ) ;
  lms_spi_write( 0XC7, 0x40 ) ;
  lms_spi_write( 0XC8, 0x0C ) ;
  lms_spi_write( 0XC9, 0x0C ) ;
  lms_spi_write( 0XCA, 0x18 ) ;
  lms_spi_write( 0XCB, 0x50 ) ;
  lms_spi_write( 0XCC, 0x00 ) ;
  lms_spi_write( 0XCD, 0x00 ) ;
  lms_spi_write( 0XD2, 0x1F ) ;
  lms_spi_write( 0XD3, 0x08 ) ;
  lms_spi_write( 0XD4, 0x02 ) ;
  lms_spi_write( 0XD5, 0x0C ) ;
  lms_spi_write( 0XD6, 0x30 ) ;
  lms_spi_write( 0XD7, 0x94 ) ;
  lms_spi_write( 0XD8, 0x00 ) ;
  lms_spi_write( 0XD9, 0x09 ) ;
  lms_spi_write( 0XDA, 0x20 ) ;
  lms_spi_write( 0XDB, 0x00 ) ;
  lms_spi_write( 0XDC, 0x00 ) ;
  lms_spi_write( 0XDD, 0x00 ) ;
  lms_spi_write( 0XDE, 0x00 ) ;
  lms_spi_write( 0XDF, 0x1F ) ;
  lms_spi_write( 0XE2, 0x1F ) ;
  lms_spi_write( 0XE3, 0x08 ) ;
  lms_spi_write( 0XE4, 0x32 ) ;
  lms_spi_write( 0XE5, 0x01 ) ;
  lms_spi_write( 0XE6, 0x00 ) ;
  lms_spi_write( 0XE7, 0x00 ) ;
  lms_spi_write( 0XE8, 0x01 ) ;
  lms_spi_write( 0XF0, 0x01 ) ;
  lms_spi_write( 0XF1, 0x80 ) ;
  lms_spi_write( 0XF2, 0x80 ) ;
  lms_spi_write( 0XF3, 0x00 ) ;
  lms_spi_write( 0XF4, 0x00 ) ;
  lms_spi_write( 0XF5, 0xD0 ) ;
  lms_spi_write( 0XF6, 0x78 ) ;
  lms_spi_write( 0XF7, 0x00 ) ;
  lms_spi_write( 0XF8, 0x1C ) ;
  lms_spi_write( 0XF9, 0x37 ) ;
  lms_spi_write( 0XFA, 0x77 ) ;
  lms_spi_write( 0XFB, 0x77 ) ;
  lms_spi_write( 0XFC, 0x18 ) ;
  lms_spi_write( 0XFD, 0x00 ) ;

//  lms_spi_write( 0x45, 25<<3 ) ;

//  lms_soft_reset() ;
//  lms_tx_enable() ;
//  lms_pa_enable( PA_1 ) ;
//  lms_set_frequency( TX, 576000000 ) ;
  bladerf_set_frequency( TX, 2420000000u ) ;
  lms_set_txvga2_gain( 20 ) ;

  bladerf_set_frequency( RX, 320000000 ) ;
  lms_lpf_enable( TX, BW_20MHz ) ;

  lms_spi_write( 0x47, 0x40 ) ;
  lms_spi_write( 0x48, 12 ) ;
  lms_spi_write( 0x49, 12 ) ;

  lms_dump_registers() ;

  // Set the prescaler for 384kHz with a 38.4MHz clock
  IOWR_16DIRECT(I2C, OC_I2C_PRESCALER, 0x20 ) ;
  IOWR_8DIRECT(I2C, OC_I2C_CTRL, OC_I2C_ENABLE ) ;

//  {
//	  printf( "Si5338 Register Table\n" ) ;
//	  printf( "---------------------\n" ) ;
//	  uint8_t i ;
//	  for( i = 0 ; ; i++ ) {
//		  si5338_read( i, &data ) ;
//		  printf( "addr: %d  data: %x\n", i, data ) ;
//		  if( i == 255 ) break ;
//	  }
//  }

  /* Turn on Si5338 TX/RX clocks */
  {
	  si5338_write(32,0xA2);
	  si5338_write(37,0x03);
	  si5338_write(65,0x1F);
	  si5338_write(70,0x01);
	  si5338_write(75,0x80);
	  si5338_write(76,0xFE);
	  si5338_write(77,0x03);
	  si5338_write(81,0x01);
	  si5338_write(86,0x80);
	  si5338_write(87,0xFE);
	  si5338_write(88,0x03);
	  si5338_write(92,0x01);
	  si5338_write(110,0x00);
	  si5338_write(144,0x80);

	  si5338_write(33, 0xA2);
	  si5338_write(38, 0x03);
  }

  /* Event loop never exits. */
  {
	  while(1)
	  {
		  // Check if anything is in the JTAG UART
		  uint32_t reg = IORD_ALTERA_AVALON_JTAG_UART_DATA(JTAG_UART_0_BASE) ;
		  if( reg & ALTERA_AVALON_JTAG_UART_DATA_RVALID_MSK )
		  {
			  // Get value from JTAG UART
			  uint8_t letter = (uint8_t)(reg & ALTERA_AVALON_JTAG_UART_DATA_DATA_MSK) ;

			  // Write it out to the FSK UART
			  while( (IORD_ALTERA_AVALON_UART_STATUS(UART_0_BASE) & ALTERA_AVALON_UART_STATUS_TRDY_MSK) == 0 ) { ; }
			  IOWR_ALTERA_AVALON_UART_TXDATA(UART_0_BASE, letter) ;
		  }

		  // Check if anything is in the FSK UART
		  if( IORD_ALTERA_AVALON_UART_STATUS(UART_0_BASE) & ALTERA_AVALON_UART_STATUS_RRDY_MSK )
		  {
			  uint8_t val ;
			  val = IORD_ALTERA_AVALON_UART_RXDATA(UART_0_BASE) ;

			  // Write it out the JTAG UART
			  alt_printf( "%c", val ) ;
		  }

//		  {
//			  while( (IORD_ALTERA_AVALON_UART_STATUS(UART_0_BASE) & ALTERA_AVALON_UART_STATUS_TRDY_MSK) == 0 ) {
//				  uint8_t a ;
//				  a = 1 ;
//			  };
//			  IOWR_ALTERA_AVALON_UART_TXDATA(UART_0_BASE, letter) ;
//			  alt_printf( "Writing: '%c'\n", letter ) ;
//			  letter = letter + 1 ;
//			  if( letter > 'z' ) {
//				  letter = 'a' ;
//			  }
//		  }
	  }
  }

  alt_putstr( "-- End of main loop --\n" ) ;

  // It's a trap!!
  //dac_write( 0xffff ) ;
  //while (1) {
//	  dac_write( 0xffff ) ;
//  }

  return 0;
}
