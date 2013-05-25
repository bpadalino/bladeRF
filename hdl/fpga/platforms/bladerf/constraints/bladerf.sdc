# Clock inputs
create_clock -period "100.0 MHz" [get_ports fx3_pclk]
create_clock -period "38.4 MHz"  [get_ports c4_clock]
create_clock -period "38.4 MHz"  [get_ports lms_pll_out]
create_clock -period "19.2 MHz"  [get_ports lms_sclk]
create_clock -period "76.8 MHz"  [get_ports lms_rx_clock_out]
create_clock -period "38.4 MHz"  [get_ports c4_tx_clock]

# Generate the appropriate PLL clocks
derive_pll_clocks
derive_clock_uncertainty

# TODO: Constrain the FX3 interface

# TODO: Constrain the LMS interface
set_input_delay -clock [get_clocks lms_rx_clock_out] -max 10.0 [get_ports lms_rx_data*]
set_input_delay -clock [get_clocks lms_rx_clock_out] -min 1.0 [get_ports lms_rx_data*] -add_delay



#set_input_delay -clock [get_clocks *clk*0*] -reference_pin [get_ports lms_rx_clock] -max 6 [get_ports lms_rx_data*] -add_delay

set_input_delay -clock [get_clocks lms_sclk] -reference_pin [get_ports lms_sclk] -min 0.2 [get_ports lms_sdo]
set_input_delay -clock [get_clocks lms_sclk] -reference_pin [get_ports lms_sclk] -max 10.0 [get_ports lms_sdo] -add_delay
set_output_delay -clock [get_clocks lms_sclk] -reference_pin [get_ports lms_sclk] -min 0.2 [get_ports {lms_sen lms_sdio}]
set_output_delay -clock [get_clocks lms_sclk] -reference_pin [get_ports lms_sclk] -max 9 [get_ports {lms_sen lms_sdio}] -add_delay

# System synchronous transmit samples
set tx_clks_max 0.01
set tx_clks_min 0.01
set tx_clkd_max 0.02
set tx_clkd_min 0.01

set tx_setup 1
set tx_hold 0.2

set tx_board_max 0.180
set tx_board_min 0.120

create_clock -period "38.4 MHz" -name virtual_tx_clock

set_output_delay -clock virtual_tx_clock -max [expr $tx_clks_max + $tx_board_max + $tx_setup - $tx_clkd_min] [get_ports {lms_tx_data[*]}]
set_output_delay -clock virtual_tx_clock -min [expr $tx_clks_min + $tx_board_min - $tx_hold - $tx_clkd_max] [get_ports {lms_tx_data[*]}] -add_delay


#set_output_delay -clock [get_clocks c4_tx_clock] -min 1.0 [get_ports *lms_tx_data*]
#set_output_delay -clock [get_clocks c4_tx_clock] -max 10 [get_ports *lms_tx_data*] -add_delay
#set_output_delay -clock [get_clocks c4_tx_clock] -min 1.0 [get_ports lms_tx_iq_select]
#set_output_delay -clock [get_clocks c4_tx_clock] -max 10 [get_ports lms_tx_iq_select] -add_delay