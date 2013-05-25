load_package flow

project_open bladerf -force

set ip_generate [file normalize [file join $::quartus(binpath) ../sopc_builder/bin/ip-generate]]

set nios_system [file normalize [file join pwd ../../../hdl/fpga/ip/altera/nios_system]]

puts "Building the Qsys..."
if { [ catch {exec $ip_generate \
    --project-directory=$nios_system \
    --output-directory=$nios_system \
    --report-file=bsf:[file join $nios_system nios_system.bsf] \
    --system-info=DEVICE_FAMILY=Cyclone\ IV\ E \
    --system-info=DEVICE=EP4CE40F23C8 \
    --system-info=DEVICE_SPEEDGRADE=8 \
    --component-file=[file join $nios_system nios_system.qsys]} fp ] } {
    puts stderr "HOLY CRAP: $fp"
} else {
    puts "ALL GOOD!"
}

puts "Generating the Qsys synthesis files..."
if { [ catch {exec $ip_generate \
    --project-directory=$nios_system \
    --output-directory=[file join $nios_system synthesis] \
    --file-set=QUARTUS_SYNTH \
    --report-file=sopcinfo:[file join $nios_system nios_system.sopcinfo] \
    --report-file=html:[file join $nios_system nios_system.html] \
    --report-file=qip:[file join $nios_system synthesis/nios_system.qip] \
    --report-file=cmp:[file join $nios_system nios_system.cmp] \
    --system-info=DEVICE_FAMILY=Cyclone\ IV\ E \
    --system-info=DEVICE=EP4CE40F23C8 \
    --system-info=DEVICE_SPEEDGRADE=8 \
    --component-file=[file join $nios_system nios_system.qsys]} fp ] } {
    puts stderr "HOLY CRAP: $fp"
} else {
    puts "ALL GOOD!"
}

set failed 0
if { [catch {execute_module -tool map} result] } {
    puts "Result: $result"
    puts stderr "ERROR: Analysis & Synthesis Failed"
    set failed 1
} else {
    puts "INFO: Analysis & Synthesis OK!"
}

if { $failed == 0 && [catch {execute_module -tool fit} result ] } {
    puts "Result: $result"
    puts stderr "ERROR: Fitter failed"
    set failed 1
} else {
    puts "INFO: Fitter OK!"
}

if { $failed == 0 && [catch {execute_module -tool sta} result] } {
    puts "Result: $result"
    puts stderr "ERROR: Timing Analysis Failed!"
    set failed 1
} else {
    puts "INFO: Timing Analysis OK!"
}

if { $failed == 0 && [catch {execute_module -tool asm} result] } {
    puts "Result: $result"
    puts stderr "ERROR: Assembler Failed!"
    set failed 1
} else {
    puts "INFO: Assembler OK!"
}

if { $failed == 0 && [catch {execute_module -tool eda} result] } {
    puts "Result: $result"
    puts stderr "ERROR: EDA failed!"
    set failed 1
} else {
    puts "INFO: EDA OK!"
}

project_close
