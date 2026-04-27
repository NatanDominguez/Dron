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

vlog -sv ../../hw/tb/tb_core.sv

vsim -voptargs="+acc" work.tb_core

add wave -position insertpoint sim:/tb_core/*
add wave -divider {INSTR_MEM}
add wave -position insertpoint sim:/tb_core/i_instr_mem/memory
add wave -position insertpoint sim:/tb_core/i_instr_mem/*
add wave -divider {DECODER}
add wave -position insertpoint sim:/tb_core/i_decoder/*
add wave -divider {ALU}
add wave -position insertpoint sim:/tb_core/i_alu/*
add wave -divider {REGISTERS}
add wave -position insertpoint sim:/tb_core/i_registers/*
add wave -position insertpoint sim:/tb_core/i_registers/registers
add wave -divider {RAM}
add wave -position insertpoint sim:/tb_core/i_ram/mem
add wave -position insertpoint sim:/tb_core/i_ram/*

run -all