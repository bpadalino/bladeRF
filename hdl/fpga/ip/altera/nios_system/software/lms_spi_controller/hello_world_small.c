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
#include "altera_avalon_spi.h"
#include "altera_avalon_uart_regs.h"
#include "altera_avalon_jtag_uart_regs.h"
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

uint16_t * const OC_I2C_PRESCALER = (uint16_t *)OC_I2C_MASTER_0_BASE ;
uint8_t * const OC_I2C_DATA = (uint8_t *)OC_I2C_MASTER_0_BASE + 3 ;
uint8_t * const OC_I2C_CTRL = (uint8_t *)OC_I2C_MASTER_0_BASE + 2 ;
uint8_t * const OC_I2C_CMD_STATUS = (uint8_t *)OC_I2C_MASTER_0_BASE + 4 ;

#define SI5338_I2C (0xE0)
#define OC_I2C_ENABLE (1<<7)
#define OC_I2C_STA (1<<7)
#define OC_I2C_STO (1<<6)
#define OC_I2C_WR  (1<<4)
#define OC_I2C_RD (1<<5)
#define OC_I2C_TIP (1<<1)
#define OC_I2C_RXACK (1<<7)
#define OC_I2C_NACK (1<<3)

// Entry point
int main()
{
  uint8_t data ;
  alt_putstr("bladeRF LMS6002D SPI Register Readback!\n");
  alt_putstr("---------------------------------------\n");

  xcvr_config.tx_freq_hz = 500000000;
  xcvr_config.rx_freq_hz = 500000000;

  xcvr_config.loopback_mode = LB_RF_LNA_START;
  xcvr_config.lna = LNA_1;
  xcvr_config.pa = PA_2;

  lms_config_init(&xcvr_config);

  // Set the prescaler for 384kHz with a 38.4MHz clock
  *OC_I2C_PRESCALER = 0x20 ;
  *OC_I2C_CTRL |= OC_I2C_ENABLE ;

  // Set the address to the Si5338 + register offset
  *OC_I2C_DATA = SI5338_I2C ;

  *OC_I2C_CMD_STATUS |= (OC_I2C_STA  | OC_I2C_WR) ;

  //while( (*OC_I2C_CMD_STATUS&OC_I2C_TIP) == 0 ) { } ;
  printf( "Transfer started...\n" ) ;
  while( (*OC_I2C_CMD_STATUS&OC_I2C_RXACK) == 1 ) { } ;
  printf( "RX ACK!\n" ) ;

  *OC_I2C_DATA = 10 ;

  *OC_I2C_CMD_STATUS |= (OC_I2C_WR | OC_I2C_STO) ;

  //while( (*OC_I2C_CMD_STATUS&OC_I2C_TIP) == 0 ) { } ;
  //printf( "Transfer startd...\n" ) ;
  while( (*OC_I2C_CMD_STATUS&OC_I2C_RXACK) == 1 ) { } ;
  printf( "RX_ACK!\n" ) ;
  while( (*OC_I2C_CMD_STATUS&OC_I2C_TIP) == 1 ) { } ;
  printf( "Transfer finished!\n" ) ;
  // Next transfer is a read operation, so '1' in the read

  *OC_I2C_DATA = SI5338_I2C + 1 ;

  *OC_I2C_CMD_STATUS |= (OC_I2C_STA | OC_I2C_WR) ;

  //while( (*OC_I2C_CMD_STATUS&OC_I2C_TIP) == 0 ) { } ;
  //printf( "Transfer started!\n" ) ;
  while( (*OC_I2C_CMD_STATUS&OC_I2C_RXACK) == 1 ) { } ;
  printf( "RX ACK'd!\n" ) ;

  *OC_I2C_CMD_STATUS |= (OC_I2C_RD | OC_I2C_NACK | OC_I2C_STO) ;

  while( (*OC_I2C_CMD_STATUS&OC_I2C_TIP) == 1 ) { } ;
  printf( "Transfer should be done!!\n" ) ;

  data = *OC_I2C_DATA ;

  alt_printf( "I2C data: %x\n", data ) ;

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
