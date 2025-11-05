# ==============================================================
# Genus Synthesis Script for counter with Error Detection (nirjala)
# ==============================================================
set_db init_lib_search_path /home/iiitdmk/Desktop/vlsiproject_mips/lib/
# === Library Search Paths ===
set_db init_lib_search_path /home/install/FOUNDRY/digital/90nm/dig/lib/

# Specify your standard cell library
# Replace "slow.lib" if your library file has a different name
set_db library slow.lib

# === Read RTL Source ===
read_hdl {/home/iiitdmk/Desktop/vlsiproject_mips/mips_cpu.v}

# === Elaborate the Top Design ===
elaborate 

# === Apply Timing Constraints ===
read_sdc /home/iiitdmk/Desktop/vlsiproject_mips/constraints_input.sdc

# === Set Synthesis Effort Levels ===
set_db syn_generic_effort medium
set_db syn_map_effort     medium
set_db syn_opt_effort     medium

# === Perform Synthesis ===
syn_generic
syn_map
syn_opt

# === Write Outputs ===
write_hdl > /home/iiitdmk/Desktop/vlsiproject_mips/mips_netlist.v
write_sdc > /home/iiitdmk/Desktop/vlsiproject_mips/mips_output.sdc

# === Reports ===
report timing > /home/iiitdmk/Desktop/vlsiproject_mips/mips_timing.rpt
report power  > /home/iiitdmk/Desktop/vlsiproject_mips/mips_power.rpt
report area   > /home/iiitdmk/Desktop/vlsiproject_mips/mips_area.rpt
report gates  > /home/iiitdmk/Desktop/vlsiproject_mips/mips_gates.rpt

# === Open GUI for Schematic View ===
gui_show

# === End of Script ===
