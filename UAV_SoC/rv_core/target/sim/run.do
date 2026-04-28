vlib work
vmap work work

vlog -sv ../../hw/src/pkg/riscv_pkg.svh
vlog -sv ../../hw/src/pkg/core_pkg.svh
vlog -sv ../../hw/src/alu.sv
vlog -sv ../../hw/src/registers.sv
vlog -sv ../../hw/src/decoder.sv
vlog -sv ../../hw/src/program_counter.sv
vlog -sv ../../hw/src/instr_mem.sv
vlog -sv ../../hw/src/ram.sv
vlog -sv ../../hw/src/core.sv
vlog -sv ../../hw/src/data_controller.sv
vlog -sv ../../../utils/uart/uart.sv

vlog -sv ../../hw/tb/tb_sys.sv

vsim -voptargs="+acc" work.tb_sys

add wave -position insertpoint sim:/tb_sys/*
add wave -divider {CORE}
add wave -position insertpoint sim:/tb_sys/i_core/*
add wave -divider {UART}
add wave -position insertpoint sim:/tb_sys/i_uart/*
add wave -divider {REGISTERS}
add wave -position insertpoint sim:/tb_sys/i_core/i_registers/registers
add wave -divider {RAM}
add wave -position insertpoint sim:/tb_sys/i_ram/mem

run -all