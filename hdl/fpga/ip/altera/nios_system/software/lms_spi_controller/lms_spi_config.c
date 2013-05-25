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
#include <inttypes.h>

#include "sys/alt_dev.h"

#include "lms_spi_config.h"

const freq_range_t bands[] = {
	{ .low =  232500000, .high =  285625000, .value = 0x27 },
	{ .low =  285625000, .high =  336875000, .value = 0x2f },
	{ .low =  336875000, .high =  405000000, .value = 0x37 },
	{ .low =  405000000, .high =  465000000, .value = 0x3f },
	{ .low =  465000000, .high =  571250000, .value = 0x26 },
	{ .low =  571250000, .high =  673750000, .value = 0x2e },
	{ .low =  673750000, .high =  810000000, .value = 0x36 },
	{ .low =  810000000, .high =  930000000, .value = 0x3e },
	{ .low =  930000000, .high = 1142500000, .value = 0x25 },
	{ .low = 1142500000, .high = 1347500000, .value = 0x2d },
	{ .low = 1347500000, .high = 1620000000, .value = 0x35 },
	{ .low = 1620000000, .high = 1860000000, .value = 0x3d },
	{ .low = 1860000000u, .high = 2285000000u, .value = 0x24 },
	{ .low = 2285000000u, .high = 2695000000u, .value = 0x2c },
	{ .low = 2695000000u, .high = 3240000000u, .value = 0x34 },
	{ .low = 3240000000u, .high = 3720000000u, .value = 0x3c }
};


const uint8_t lms_reg_dumpset[] = {
	 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0E, 0x0F,

     0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F,
     0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F,

     0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,

     0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F,

     0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F,

     0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,

     0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C
} ;

// Trim DAC write
void dac_write( uint16_t val ) {
	alt_printf( "DAC Writing: %x\n", val ) ;
	uint8_t data[3] ;
	data[0] = 0x28, data[1] = 0, data[2] = 0 ;
	alt_avalon_spi_command( SPI_1_BASE, 0, 3, data, 0, 0, 0 ) ;
	data[0] = 0x08, data[1] = (val>>8)&0xff, data[2] = val&0xff  ;
	alt_avalon_spi_command( SPI_1_BASE, 0, 3, data, 0, 0, 0) ;
	return ;
}

// SPI Read
void lms_spi_read( uint8_t address, uint8_t *val )
{
	uint8_t rv ;
	if( address > 0x7f )
	{
		alt_printf( "Invalid read address: %x\n", address ) ;
	} else {
		alt_avalon_spi_command( SPI_0_BASE, 0, 1, &address, 0, 0, ALT_AVALON_SPI_COMMAND_MERGE ) ;
		rv = alt_avalon_spi_command( SPI_0_BASE, 0, 0, 0, 1, val, 0 ) ;
		if( rv != 1 )
		{
			alt_putstr( "SPI data read did not work :(\n") ;
		}
	}
	if( LMS_VERBOSE )
	{
		alt_printf( "r-addr: %x data: %x\n", address, *val ) ;
	}
	return ;
}

// SPI Write
void lms_spi_write( uint8_t address, uint8_t val )
{
	if( LMS_VERBOSE )
	{
		alt_printf( "w-addr: %x data: %x\n", address, val ) ;
	}
	/*if( address > 0x7f )
	{
		alt_printf( "Invalid write address: %x\n", address ) ;
	} else*/ 
    {
		uint8_t data[2] = { address |= LMS_WRITE, val } ;
		alt_avalon_spi_command( SPI_0_BASE, 0, 2, data, 0, 0, 0 ) ;
	}
	return ;
}

// When enabling an LPF, we must select both the module and the filter bandwidth
void lms_lpf_enable( lms_module_t mod, lms_bw_t bw )
{
	uint8_t reg = (mod == RX) ? 0x54 : 0x34 ;
	uint8_t data ;
	// Check to see which bandwidth we have selected
	lms_spi_read( reg, &data ) ;
	if( (lms_bw_t)(data&0x3c>>2) != bw )
	{
		data &= ~0x3c ;
		data |= (bw<<2) ;
		data |= (1<<1) ;
		lms_spi_write( reg, data ) ;
	}
	// Check to see if we are bypassed
	lms_spi_read( reg+1, &data ) ;
	if( data&(1<<6) )
	{
		data &= ~(1<<6) ;
		lms_spi_write( reg+1, data ) ;
	}
	return ;
}

void lms_lpf_bypass( lms_module_t mod )
{
	uint8_t reg = (mod == RX) ? 0x55 : 0x35 ;
	uint8_t data ;
	lms_spi_read( reg, &data ) ;
	data |= (1<<6) ;
	lms_spi_write( reg, data ) ;
	return ;
}

// Disable the LPF for a specific module
void lms_lpf_disable( lms_module_t mod )
{
	uint8_t reg = (mod == RX) ? 0x54 : 0x34 ;
	lms_spi_write( reg, 0x00 ) ;
	return ;
}

// Get the bandwidth for the selected module
lms_bw_t lms_get_bandwidth( lms_module_t mod )
{
	uint8_t data ;
	uint8_t reg = (mod == RX) ? 0x54 : 0x34 ;
	lms_spi_read( reg, &data ) ;
	data &= 0x3c ;
	data >>= 2 ;
	return (lms_bw_t)data ;
}

// Enable dithering on the module PLL
void lms_dither_enable( lms_module_t mod, uint8_t nbits )
{
	// Select the base address based on which PLL we are configuring
	uint8_t reg = (mod == RX) ? 0x24 : 0x14 ;
	uint8_t data ;

	// Read what we currently have in there
	lms_spi_read( reg, &data ) ;

	// Enable dithering
	data |= (1<<7) ;

	// Clear out the number of bits from before
	data &= ~(7<<4) ;

	// Put in the number of bits to dither
	data |= ((nbits-1)&7) ;

	// Write it out
	lms_spi_write( reg, data ) ;
	return ;
}

// Disable dithering on the module PLL
void lms_dither_disable( lms_module_t mod )
{
	uint8_t reg = (mod == RX) ? 0x24 : 0x14 ;
	uint8_t data ;
	lms_spi_read( reg, &data ) ;
	data &= ~(1<<7) ;
	lms_spi_write( reg, data ) ;
	return ;
}

// Soft reset of the LMS
void lms_soft_reset( )
{
	lms_spi_write( 0x05, 0x12 ) ;
	lms_spi_write( 0x05, 0x32 ) ;
	return ;
}

// Set the gain on the LNA
void lms_lna_set_gain( lms_lna_gain_t gain )
{
	uint8_t data ;
	lms_spi_read( 0x75, &data ) ;
	data &= ~(3<<6) ;
	data |= ((gain&3)<<6) ;
	lms_spi_write( 0x75, data ) ;
	return ;
}

// Select which LNA to enable
void lms_lna_select( lms_lna_t lna )
{
	uint8_t data ;
	lms_spi_read( 0x75, &data ) ; 
	data &= ~(3<<4) ;
	data |= ((lna&3)<<4) ;
	lms_spi_write( 0x75, data ) ;
	return ;
}

// Disable RXVGA1
void lms_rxvga1_disable()
{
	// Set bias current to 0
	lms_spi_write( 0x7b, 0x03 ) ;
	return ;
}

// Enable RXVGA1
void lms_rxvga1_enable()
{
	// Set bias current to nominal
	lms_spi_write( 0x7b, 0x33 ) ;
	return ;
}

// Disable RXVGA2
void lms_rxvga2_disable()
{
	uint8_t data ;
	lms_spi_read( 0x64, &data ) ;
	data &= ~(1<<1) ;
	lms_spi_write( 0x64, data ) ;
	return ;
}

// Set the gain on RXVGA2
void lms_rxvga2_set_gain( uint8_t gain )
{
	// NOTE: Gain is calculated as gain*3dB and shouldn't really
	// go above 30dB
	if( (gain&0x1f) > 10 )
	{
		alt_putstr( "Setting gain above 30dB? You crazy!!\n" ) ;
	}
	lms_spi_write( 0x65, (0x1f)&gain ) ;
	return ;
}

// Enable RXVGA2
void lms_rxvga2_enable( uint8_t gain )
{
	uint8_t data ;
	lms_spi_read( 0x64, &data ) ;
	data |= (1<<1) ;
	lms_spi_write( 0x64, data ) ;
	lms_rxvga2_set_gain( gain ) ;
	return ;
}

// Enable PA (PA_ALL is NOT valid for enabling)
void lms_pa_enable( lms_pa_t pa )
{
	uint8_t data ;
	lms_spi_read( 0x44, &data ) ;
	if( pa == PA_AUX )
	{
		data &= ~(1<<1) ;
	} else if( pa == PA_1 )
	{
		data &= ~(3<<3) ;
		data |= (1<<3) ;
	} else if( pa == PA_2 )
	{
		data &= ~(3<<3) ;
		data |= (2<<3) ;
	}
	lms_spi_write( 0x44, data ) ;
	return ;
}

// Disable PA
void lms_pa_disable( lms_pa_t pa )
{
	uint8_t data ;
	lms_spi_read( 0x44, &data ) ;
	if( pa == PA_ALL )
	{
		data |= (1<<1) ;
		data &= ~(4<<2) ;
		data &= ~(2<<2) ;
	} else if( pa == PA_AUX )
	{
		data |= (1<<1) ;
	} else if( pa == PA_1 )
	{
		data &= ~(4<<2) ;
	} else { // pa == PA_2
		data &= ~(2<<2) ;
	}
	lms_spi_write( 0x44, data ) ;
	return ;
}

void lms_peakdetect_enable( )
{
	uint8_t data ;
	lms_spi_read( 0x44, &data ) ;
	data &= ~(1<<0) ;
	lms_spi_write( 0x44, data ) ;
	return ;
}

void lms_peakdetect_disable( )
{
	uint8_t data ;
	lms_spi_read( 0x44, &data ) ;
	data |= (1<<0) ;
	lms_spi_write( 0x44, data ) ;
	return ;
}

// Enable TX loopback
void lms_tx_loopback_enable( lms_txlb_t mode )
{
	uint8_t data ;
	switch(mode)
	{
		case TXLB_BB:
			lms_spi_read( 0x46, &data ) ;
			data |= (3<<2) ;
			lms_spi_write( 0x46, data ) ;
			break ;
		case TXLB_RF:
			// Disable all the PA's first
			lms_pa_disable( PA_ALL ) ;
			// Connect up the switch
			lms_spi_read( 0x0b, &data ) ;
			data |= (1<<0) ;
			lms_spi_write( 0x0b, data ) ;
			// Enable the AUX PA only
			lms_pa_enable( PA_AUX ) ;
			lms_peakdetect_enable( );
			// Make sure we're muxed over to the AUX mux
			lms_spi_read( 0x45, &data ) ;
			data &= ~(7<<0) ;
			lms_spi_write( 0x45, data ) ;
			break ;
	}
	return ;
}

void lms_set_txvga2_gain( uint8_t gain ) {
	if( gain > 25 ) {
		gain = 25 ;
	}
	lms_spi_write( 0x45, gain << 3 ) ;
	return ;
}

// Disable TX loopback
void lms_tx_loopback_disable( lms_txlb_t mode )
{
	uint8_t data ;
	switch(mode)
	{
		case TXLB_BB:
			lms_spi_read( 0x46, &data ) ;
			data &= ~(3<<2) ;
			lms_spi_write( 0x46, data ) ;
			break ;
		case TXLB_RF:
			// Disable the AUX PA
			lms_pa_disable( PA_AUX ) ;
			// Disconnect the switch
			lms_spi_read( 0x0b, &data ) ;
			data &= ~(1<<0) ;
			lms_spi_write( 0x0b, data ) ;
            // Power up the LNA's
            lms_spi_write( 0x70, 0 ) ;
			break ;
	}
	return ;
}

// Loopback enable
void lms_loopback_enable( lms_loopback_mode_t mode )
{
	uint8_t data ;
	switch(mode)
	{
		case LB_BB_LPF:
			// Disable RXVGA1 first
			lms_rxvga1_disable() ;

			// Enable BB TX and RX loopback
			lms_tx_loopback_enable( TXLB_BB ) ;
			lms_spi_write( 0x08, 1<<6 ) ;
			break ;

		case LB_BB_VGA2:
			// Disable RXLPF first
			lms_lpf_disable( RX ) ;

			// Enable TX and RX loopback
			lms_tx_loopback_enable( TXLB_BB ) ;
			lms_spi_write( 0x08, 1<<5 ) ;
			break ;

		case LB_BB_OP:
			// Disable RXLPF, RXVGA2, and RXVGA1
			lms_rxvga1_disable() ;
			lms_rxvga2_disable() ;
			lms_lpf_disable( RX ) ;

			// Enable TX and RX loopback
			lms_tx_loopback_enable( TXLB_BB ) ;
			lms_spi_write( 0x08, 1<<4 ) ;
			break ;

		case LB_RF_LNA1:
		case LB_RF_LNA2:
		case LB_RF_LNA3:
			// Disable all LNAs
			lms_lna_select( LNA_NONE ) ;

			// Enable AUX PA, PD[0], and loopback
			lms_tx_loopback_enable( TXLB_RF ) ;
			lms_spi_read( 0x7d, &data ) ;
			data |= 1 ;
			lms_spi_write( 0x7d, data ) ;

            // Choose the LNA (1 = LNA1, 2 = LNA2, 3 = LNA3)
            lms_spi_write( 0x08, (mode - LB_RF_LNA_START) ) ;

            // Set magical decode test registers bit
            lms_spi_write( 0x70, (1<<1) ) ;
			break ;

		case LB_NONE:
			// Weird
			break ;
	}
	return ;
}

// Figure out what loopback mode we're in (if any at all!)
lms_loopback_mode_t lms_get_loopback_mode( )
{
	uint8_t data ;
	lms_loopback_mode_t mode = LB_NONE ;
	lms_spi_read( 0x08, &data ) ;
	if( data == 0 )
	{
		mode = LB_NONE ;
	} else if( data&(1<<6) )
	{
		mode = LB_BB_LPF ;
	} else if( data&(1<<5) )
	{
		mode = LB_BB_VGA2 ;
	} else if( data&(1<<4) )
	{
		mode = LB_BB_OP ;
	} else if( (data&0xf) == 1 )
	{
		mode = LB_RF_LNA1 ;
	} else if( (data&0xf) == 2 )
	{
		mode = LB_RF_LNA2 ;
	} else if( (data&0xf) == 3 )
	{
		mode = LB_RF_LNA3 ;
	}
	return mode ;
}

// Disable loopback mode - must choose which LNA to hook up and what bandwidth you want
void lms_loopback_disable( lms_lna_t lna, lms_bw_t bw )
{
	// Read which type of loopback mode we were in
	lms_loopback_mode_t mode = lms_get_loopback_mode() ;

	// Disable all RX loopback modes
	lms_spi_write( 0x08, 0 ) ;

	switch(mode)
	{
		case LB_BB_LPF:
			// Disable TX baseband loopback
			lms_tx_loopback_disable( TXLB_BB ) ;
			// Enable RXVGA1
			lms_rxvga1_enable() ;
			break ;
		case LB_BB_VGA2:
			// Disable TX baseband loopback
			lms_tx_loopback_disable( TXLB_BB ) ;
			// Enable RXLPF
			lms_lpf_enable( RX, bw ) ;
			break ;
		case LB_BB_OP:
			// Disable TX baseband loopback
			lms_tx_loopback_disable( TXLB_BB ) ;
			// Enable RXLPF, RXVGA1 and RXVGA2
			lms_lpf_enable( RX, bw ) ;
			lms_rxvga2_enable( 30/3 ) ;
			lms_rxvga1_enable() ;
			break ;
		case LB_RF_LNA1:
		case LB_RF_LNA2:
		case LB_RF_LNA3:
			// Disable TX RF loopback
			lms_tx_loopback_disable( TXLB_RF ) ;
			// Enable selected LNA
			lms_lna_select( lna ) ;
			break ;
		case LB_NONE:
			// Weird
			break ;
	}
	return ;
}

// Top level power down of the LMS
void lms_power_down( )
{
	uint8_t data ;
	lms_spi_read( 0x05, &data ) ;
	data &= ~(1<<4) ;
	lms_spi_write( 0x05, data ) ;
	return ;
}

// Enable the PLL of a module
void lms_pll_enable( lms_module_t mod )
{
	uint8_t reg = (mod == RX) ? 0x24 : 0x14 ;
	uint8_t data ;
	lms_spi_read( reg, &data ) ;
	data |= (1<<3) ;
	lms_spi_write( reg, data ) ;
	return ;
}

// Disable the PLL of a module
void lms_pll_disable( lms_module_t mod )
{
	uint8_t reg = (mod == RX) ? 0x24 : 0x14 ;
	uint8_t data ;
	lms_spi_read( reg, &data ) ;
	data &= ~(1<<3) ;
	lms_spi_write( reg, data ) ;
	return ;
}

// Enable the RX subsystem
void lms_rx_enable( )
{
	uint8_t data ;
	lms_spi_read( 0x05, &data ) ;
	data |= (1<<2) ;
	lms_spi_write( 0x05, data ) ;
	return ;
}

// Disable the RX subsystem
void lms_rx_disable( )
{
	uint8_t data ;
	lms_spi_read( 0x05, &data ) ;
	data &= ~(1<<2) ;
	lms_spi_write( 0x05, data ) ;
	return ;
}

// Enable the TX subsystem
void lms_tx_enable( )
{
	uint8_t data ;
	lms_spi_read( 0x05, &data ) ;
	data |= (1<<3) ;
	lms_spi_write( 0x05, data ) ;
	return ;
}

// Disable the TX subsystem
void lms_tx_disable( )
{
	uint8_t data ;
	lms_spi_read( 0x05, &data ) ;
	data &= ~(1<<3) ;
	lms_spi_write( 0x05, data ) ;
	return ;
}

// Print a frequency structure
void lms_print_frequency( lms_freq_t *f )
{
	printf( "  x        : %d\n", f->x ) ;
	printf( "  nint     : %d\n", f->nint ) ;
	printf( "  nfrac    : %"PRIu32"\n", f->nfrac ) ;
	printf( "  freqsel  : %x\n", f->freqsel ) ;
	printf( "  reference: %"PRIu32"\n", f->reference ) ;
	printf( "  freq     : %"PRIu32"\n", (uint32_t) ( ((uint64_t)((f->nint<<23) + f->nfrac)) * (f->reference/f->x) >>23) )  ;
}

// Get the frequency structure
void lms_get_frequency( lms_module_t mod, lms_freq_t *f ) {
	uint8_t base = (mod == RX) ? 0x20 : 0x10 ;
	uint8_t data ;
	lms_spi_read( base+0, &data ) ;
	f->nint = ((uint16_t)data) << 1 ;
	lms_spi_read( base+1, &data ) ;
	f->nint |= (data&0x80)>>7 ;
	f->nfrac = ((uint32_t)data&0x7f)<<16 ;
	lms_spi_read( base+2, &data ) ;
	f->nfrac |= ((uint32_t)data)<<8 ;
	lms_spi_read( base+3, &data) ;
	f->nfrac |= data ;
	lms_spi_read( base+5, &data ) ;
	f->freqsel = (data>>2) ;
	f->x = 1 << ((f->freqsel&7)-3);
	f->reference = 38400000 ;
	return ;
}

// Set the frequency of a module
void lms_set_frequency( lms_module_t mod, uint32_t freq )
{
	// Select the base address based on which PLL we are configuring
	uint8_t base = (mod == RX) ? 0x20 : 0x10 ;
	uint32_t lfreq = freq ;
	uint8_t freqsel = bands[0].value ;
	uint16_t nint ;
	uint32_t nfrac ;
	lms_freq_t f ;
	uint8_t data ;
    uint32_t x;
	uint32_t reference = 38400000 ;
	uint64_t vcofreq ;
	uint64_t temp ;
	uint32_t left ;


	// Turn on the DSMs
	lms_spi_read( 0x09, &data ) ;
	data |= 0x05 ;
	lms_spi_write( 0x09, data ) ;

	// Figure out freqsel
	if( lfreq < bands[0].low )
	{
		// Too low
	} else if( lfreq > bands[15].high )
	{
		// Too high!
	} else
	{
		uint8_t i = 0 ;
		while( i < 16 )
		{
			if( (lfreq > bands[i].low) && (lfreq <= bands[i].high) )
			{
				freqsel = bands[i].value ;
				break ;
			}
			i++ ;
		}
	}

    x = 1 << ((freqsel&7)-3);
    //nint = floor( 2^(freqsel(2:0)-3) * f_lo / f_ref)
    //nfrac = floor(2^23 * (((x*f_lo)/f_ref) -nint))
    {
         temp ;
         vcofreq = (uint64_t)freq*x ;

        nint = vcofreq/reference ;
        left = vcofreq - nint*reference ;
        nfrac = 0 ;
        {
        	// Long division ...
        	int i ;
        	for( i = 0 ; i < 24 ; i++ ) {
        		if( left >= reference ) {
        			left = left - reference ;
        			nfrac = (nfrac << 1) + 1 ;
        		} else {
        			nfrac <<= 1 ;
        		}
        		left <<= 1 ;
        	}
        }

//        temp = (uint64_t)((uint64_t)x*(uint64_t)freq) ;
//        nint = ((uint64_t)x*(uint64_t)freq)/(uint64_t)reference ;
//        {
//        	uint32_t left =
//        }
//        nfrac = (temp - (nint*reference))<<23 ;

    }
	//nfrac = (lfreq>>2) - (lfreq>>5) - (lfreq>>12) ;
	//nfrac <<= ((freqsel&7)-3) ;
    f.x = x ;
	f.nint = nint ;
	f.nfrac = nfrac ;
	f.freqsel = freqsel ;
	f.reference = reference ;
	lms_print_frequency( &f ) ;

	// Program freqsel, selout (rx only), nint and nfrac
	if( mod == RX )
	{
		lms_spi_write( base+5, freqsel<<2 | (freq < 1500000000 ? 1 : 2 ) ) ;
	} else {
//		lms_spi_write( base+5, freqsel<<2 ) ;
		lms_spi_write( base+5, freqsel<<2 | (freq < 1500000000 ? 1 : 2 ) ) ;
	}
	data = nint>>1 ;// alt_printf( "%x\n", data ) ;
	lms_spi_write( base+0, data ) ;
	data = ((nint&1)<<7) | ((nfrac>>16)&0x7f) ;//  alt_printf( "%x\n", data ) ;
	lms_spi_write( base+1, data ) ;
	data = ((nfrac>>8)&0xff) ;//  alt_printf( "%x\n", data ) ;
	lms_spi_write( base+2, data ) ;
	data = (nfrac&0xff) ;//  alt_printf( "%x\n", data ) ;
	lms_spi_write( base+3, data ) ;

	// Set the PLL Ichp, Iup and Idn currents
	lms_spi_read( base+6, &data ) ;
	data &= ~(0x1f) ;
	data |= 0x0c ;
	lms_spi_write( base+6, data ) ;
	lms_spi_read( base+7, &data ) ;
	data &= ~(0x1f) ;
	data |= 3 ;
    data = 0xe3;
	lms_spi_write( base+7, data ) ;
	lms_spi_read( base+8, &data ) ;
	data &= ~(0x1f) ;
	lms_spi_write( base+8, data ) ;

	// Loop through the VCOCAP to figure out optimal values
	lms_spi_read( base+9, &data ) ;
	data &= ~(0x3f) ;
	{
		uint8_t i, vtune, low = 64, high = 0;
		for( i = 0 ; i < 64 ; i++ )
		{
			data &= ~(0x3f) ;
			data |= i ;
			lms_spi_write( base+9, data ) ;
			lms_spi_read( base+10, &vtune ) ;
			if( (vtune&0xc0) == 0xc0 )
			{
				alt_putstr( "MESSED UP!!!!!\n" ) ;
			}
			if( vtune&0x80 )
			{
				//alt_putstr( "Setting HIGH\n" ) ;
				high = i ;
			}
			if( (vtune&0x40) && low == 64 )
			{
				low = i ;
				break ;
			}
		}
		//alt_printf( "LOW: %x HIGH: %x VCOCAP: %x\n", low, high, (low+high)>>1 ) ;
		data &= ~(0x3f) ;
		data |= ((low+high)>>1) ;
		lms_spi_write( base+9, data ) ;
		lms_spi_write( base+9, data ) ;
		lms_spi_read( base+10, &vtune ) ;
		//alt_printf( "VTUNE: %x\n", vtune&0xc0 ) ;
	}

	// Turn off the DSMs
	lms_spi_read( 0x09, &data ) ;
	data &= ~(0x05) ;
	lms_spi_write( 0x09, data ) ;

	return ;
}

void lms_dump_registers(void)
{
	uint8_t data,i;
    uint16_t num_reg = sizeof(lms_reg_dumpset);
    for (i = 0; i < num_reg; i++)
    {   
        lms_spi_read( lms_reg_dumpset[i], &data ) ;
        alt_printf( "addr: %x data: %x\n", lms_reg_dumpset[i], data ) ;
    }
}

void lms_calibrate_dc(void)
{
	// RX path
	lms_spi_write( 0x09, 0x8c ) ; // CLK_EN[3]
	lms_spi_write( 0x43, 0x08 ) ; // I filter
	lms_spi_write( 0x43, 0x28 ) ; // Start Calibration
	lms_spi_write( 0x43, 0x08 ) ; // Stop calibration

	lms_spi_write( 0x43, 0x09 ) ; // Q Filter
	lms_spi_write( 0x43, 0x29 ) ;
	lms_spi_write( 0x43, 0x09 ) ;

	lms_spi_write( 0x09, 0x84 ) ;

	lms_spi_write( 0x09, 0x94 ) ; // CLK_EN[4]
	lms_spi_write( 0x66, 0x00 ) ; // Enable comparators

	lms_spi_write( 0x63, 0x08 ) ; // DC reference module
	lms_spi_write( 0x63, 0x28 ) ;
	lms_spi_write( 0x63, 0x08 ) ;

	lms_spi_write( 0x63, 0x09 ) ;
	lms_spi_write( 0x63, 0x29 ) ;
	lms_spi_write( 0x63, 0x09 ) ;

	lms_spi_write( 0x63, 0x0a ) ;
	lms_spi_write( 0x63, 0x2a ) ;
	lms_spi_write( 0x63, 0x0a ) ;

	lms_spi_write( 0x63, 0x0b ) ;
	lms_spi_write( 0x63, 0x2b ) ;
	lms_spi_write( 0x63, 0x0b ) ;

	lms_spi_write( 0x63, 0x0c ) ;
	lms_spi_write( 0x63, 0x2c ) ;
	lms_spi_write( 0x63, 0x0c ) ;

	lms_spi_write( 0x66, 0x0a ) ;
	lms_spi_write( 0x09, 0x84 ) ;

	// TX path
	lms_spi_write( 0x57, 0x04 ) ;
	lms_spi_write( 0x09, 0x42 ) ;

	lms_spi_write( 0x33, 0x08 ) ;
	lms_spi_write( 0x33, 0x28 ) ;
	lms_spi_write( 0x33, 0x08 ) ;

	lms_spi_write( 0x33, 0x09 ) ;
	lms_spi_write( 0x33, 0x29 ) ;
	lms_spi_write( 0x33, 0x09 ) ;

	lms_spi_write( 0x57, 0x84 ) ;
	lms_spi_write( 0x09, 0x81 ) ;

	lms_spi_write( 0x42, 0x77 ) ;
	lms_spi_write( 0x43, 0x7f ) ;

	return ;
}

void lms_lpf_init(void)
{
	lms_spi_write( 0x06, 0x0d ) ;
	lms_spi_write( 0x17, 0x43 ) ;
	lms_spi_write( 0x27, 0x43 ) ;
	lms_spi_write( 0x41, 0x1f ) ;
	lms_spi_write( 0x44, 1<<3 ) ;
	lms_spi_write( 0x45, 0x1f<<3 ) ;
	lms_spi_write( 0x48, 0xc  ) ;
	lms_spi_write( 0x49, 0xc ) ;
	lms_spi_write( 0x57, 0x84 ) ;
	return ;
}


int lms_config_init(xcvr_config_t *config)
{

  lms_soft_reset() ;
  lms_lpf_init() ;
  lms_tx_enable() ;
  lms_rx_enable() ;

  lms_spi_write( 0x48, 20 ) ;
  lms_spi_write( 0x49, 20 ) ;

  lms_set_frequency( RX,  config->rx_freq_hz ) ;
  lms_set_frequency( TX,  config->tx_freq_hz ) ;

  lms_lna_select( config->lna  ) ;
  lms_pa_enable( config->pa ) ;

  if( config->loopback_mode == LB_NONE ) {
	  lms_loopback_disable( config->lna, config->bw ) ;
  } else {
	  lms_loopback_enable(config->loopback_mode);
  }

  return 0;
}

