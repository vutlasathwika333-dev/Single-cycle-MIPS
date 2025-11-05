# ===================================================
# Timing and Design Constraints for FIFO with Error Detection
# Compatible with Cadence Genus
# ===================================================

# Create main clock (100 MHz → 10 ns period)
create_clock -name clk -period 10 [get_ports clk]

# Set input delays (1 ns) relative to clock
set_input_delay 1 -clock [get_clocks clk] [get_ports {rst_n wr_en wr_data[*] rd_en}]

# Set output delays (2 ns) relative to clock
set_output_delay 2 -clock [get_clocks clk] [get_ports {rd_data[*] full empty overflow underflow parity_error}]

# Mark reset as false path to exclude from timing
set_false_path -from [get_ports rst_n]

# Prevent optimization of FIFO memory (optional; ignore if no such cells)
if {[llength [get_cells fifo_mem* -quiet]] > 0} {
    set_dont_touch [get_cells fifo_mem*]
}

# Set operating condition for on-chip variation analysis
if {[llength [get_operating_conditions -quiet]] > 0} {
    set_operating_conditions -analysis_type on_chip_variation
}

# ===================================================
# END OF FILE
# ===================================================
