if { [file exists opencores] == 0 } {
	vlib opencores
}
vlog -work opencores +incdir+../rtl/verilog ../rtl/verilog/i2c_master_bit_ctrl.v
vlog -work opencores +incdir+../rtl/verilog ../rtl/verilog/i2c_master_byte_ctrl.v
vlog -work opencores +incdir+../rtl/verilog ../rtl/verilog/i2c_master_defines.v
vlog -work opencores +incdir+../rtl/verilog ../rtl/verilog/i2c_master_top.v

vlog -work opencores +incdir+../rtl/verilog ../bench/verilog/i2c_slave_model.v
vlog -work opencores +incdir+../rtl/verilog ../bench/verilog/tst_bench_top.v
vlog -work opencores +incdir+../rtl/verilog ../bench/verilog/wb_master_model.v
