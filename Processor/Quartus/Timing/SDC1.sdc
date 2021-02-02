create_clock -period 20.000 -name clk [get_ports clk]
derive_clock_uncertainty
set_input_delay -clock { clk } -min 2 [get_ports inp]
set_input_delay -clock { clk } -max 5 [get_ports inp]
set_output_delay -clock clk -max 7 [get_ports outp]
set_output_delay -clock clk -min -3 [get_ports outp]
