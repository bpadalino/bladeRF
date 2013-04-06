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

#define LMS_READ 	0
#define LMS_WRITE 	(1<<7)
#define LMS_VERBOSE 1
#define kHz(x) (x*1000)
#define MHz(x) (x*1000000)
#define GHz(x) (x*1000000000)

// Frequency selection structure
typedef struct {
	uint16_t nint ;
	uint32_t nfrac ;
	uint8_t freqsel ;
	uint32_t reference ;
} lms_freq_t ;

// Bandwidth selection
typedef enum {
	BW_28MHz,
	BW_20MHz,
	BW_12MHz,
	BW_14MHz,
	BW_10MHz,
	BW_8p75MHz,
	BW_7MHz,
	BW_6MHz,
	BW_5p5MHz,
	BW_5MHz,
	BW_3p84MHz,
	BW_3MHz,
	BW_2p75MHz,
	BW_2p5MHz,
	BW_1p75MHz,
	BW_1p5MHz,
} lms_bw_t ;

// Module selection for those which have both RX and TX constituents
typedef enum
{
	RX,
	TX
} lms_module_t ;

// Loopback options
typedef enum {
	LB_BB_LPF = 0,
	LB_BB_VGA2,
	LB_BB_OP,
    LB_RF_LNA_START,
	LB_RF_LNA1,
	LB_RF_LNA2,
	LB_RF_LNA3,
	LB_NONE
} lms_loopback_mode_t ;

// LNA options
typedef enum {
	LNA_NONE,
	LNA_1,
	LNA_2,
	LNA_3
} lms_lna_t ;


// LNA gain options
typedef enum {
	LNA_UNKNOWN,
	LNA_BYPASS,
	LNA_MID,
	LNA_MAX
} lms_lna_gain_t ;

typedef enum {
	TXLB_BB,
	TXLB_RF
} lms_txlb_t ;

typedef enum {
	PA_AUX,
	PA_1,
	PA_2,
	PA_ALL
} lms_pa_t ;

// Frequency Range table
typedef struct {
	uint32_t low ;
	uint32_t high ;
	uint8_t value ;
} freq_range_t ;

typedef struct {
    uint32_t tx_freq_hz;
    uint32_t rx_freq_hz;
    lms_loopback_mode_t loopback_mode;
    lms_lna_t lna;
    lms_pa_t  pa;
} xcvr_config_t;

// When enabling an LPF, we must select both the module and the filter bandwidth
void lms_lpf_enable( lms_module_t mod, lms_bw_t bw );

void lms_lpf_bypass( lms_module_t mod );

// Disable the LPF for a specific module
void lms_lpf_disable( lms_module_t mod );

// Get the bandwidth for the selected module
lms_bw_t lms_get_bandwidth( lms_module_t mod );

// Enable dithering on the module PLL
void lms_dither_enable( lms_module_t mod, uint8_t nbits );

// Disable dithering on the module PLL
void lms_dither_disable( lms_module_t mod );
// Soft reset of the LMS
void lms_soft_reset( );

// Set the gain on the LNA
void lms_lna_set_gain( lms_lna_gain_t gain );

// Select which LNA to enable
void lms_lna_select( lms_lna_t lna );

// Disable RXVGA1
void lms_rxvga1_disable();

// Enable RXVGA1
void lms_rxvga1_enable();

// Disable RXVGA2
void lms_rxvga2_disable();

// Set the gain on RXVGA2
void lms_rxvga2_set_gain( uint8_t gain );
// Enable RXVGA2
void lms_rxvga2_enable( uint8_t gain );

// Enable PA (PA_ALL is NOT valid for enabling)
void lms_pa_enable( lms_pa_t pa );

// Disable PA
void lms_pa_disable( lms_pa_t pa );

void lms_peakdetect_enable( );

void lms_peakdetect_disable( );
// Enable TX loopback
void lms_tx_loopback_enable( lms_txlb_t mode );
// Disable TX loopback
void lms_tx_loopback_disable( lms_txlb_t mode );
// Loopback enable
void lms_loopback_enable( lms_loopback_mode_t mode );
// Figure out what loopback mode we're in (if any at all!)
lms_loopback_mode_t lms_get_loopback_mode( );

// Disable loopback mode - must choose which LNA to hook up and what bandwidth you want
void lms_loopback_disable( lms_lna_t lna, lms_bw_t bw );

// Top level power down of the LMS
void lms_power_down( );

// Enable the PLL of a module
void lms_pll_enable( lms_module_t mod );
// Disable the PLL of a module
void lms_pll_disable( lms_module_t mod );
// Enable the RX subsystem
void lms_rx_enable( );
// Disable the RX subsystem
void lms_rx_disable( );

// Enable the TX subsystem
void lms_tx_enable( );

// Disable the TX subsystem
void lms_tx_disable( );
// Print a frequency structure
void lms_print_frequency( lms_freq_t *f );
// Get the frequency structure
void lms_get_frequency( lms_module_t mod, lms_freq_t *f );
// Set the frequency of a module
void lms_set_frequency( lms_module_t mod, uint32_t freq );

void lms_dump_registers(void);

void lms_calibrate_dc(void);

void lms_lpf_init(void);

int lms_config_init(xcvr_config_t *config);
