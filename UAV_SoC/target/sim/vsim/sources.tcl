

set ROOT ../../../


# COMMON CELLS

vlog -sv $ROOT/common_cells/src/delta_counter.sv
#vlog -sv $ROOT/common_cells/src/prescaler.sv
vlog -sv $ROOT/common_cells/src/timer.sv
vlog -sv $ROOT/common_cells/src/pwm_gen.sv

# FLIGHT CONTROL

vlog -sv $ROOT/flight_control/src/signal_gen.sv
vlog -sv $ROOT/flight_control/src/bldc_ol_fsm.sv
vlog -sv $ROOT/flight_control/src/bldc_cl_fsm.sv
vlog -sv $ROOT/flight_control/src/bldc_controller.sv


# TESTBENCH

vlog -sv ../src/uav_soc_tb.sv