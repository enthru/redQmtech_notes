
# set_property CFGBVS VCCO [current_design]
# set_property CONFIG_VOLTAGE 3.3 [current_design]

### ADC

# data

set_property IOSTANDARD LVCMOS33 [get_ports {adc_dat_a_i[*]}]
set_property IOB TRUE [get_ports {adc_dat_a_i[*]}]

set_property PACKAGE_PIN T22 [get_ports {adc_dat_a_i[0]}]
set_property PACKAGE_PIN U22 [get_ports {adc_dat_a_i[1]}]
set_property PACKAGE_PIN U20 [get_ports {adc_dat_a_i[2]}]
set_property PACKAGE_PIN V20 [get_ports {adc_dat_a_i[3]}]
set_property PACKAGE_PIN T21 [get_ports {adc_dat_a_i[4]}]
set_property PACKAGE_PIN U21 [get_ports {adc_dat_a_i[5]}]
set_property PACKAGE_PIN V22 [get_ports {adc_dat_a_i[6]}]
set_property PACKAGE_PIN W22 [get_ports {adc_dat_a_i[7]}]
set_property PACKAGE_PIN W20 [get_ports {adc_dat_a_i[8]}]
set_property PACKAGE_PIN W21 [get_ports {adc_dat_a_i[9]}]
set_property PACKAGE_PIN V18 [get_ports {adc_dat_a_i[10]}]
set_property PACKAGE_PIN V19 [get_ports {adc_dat_a_i[11]}]
set_property PACKAGE_PIN AA22 [get_ports {adc_dat_a_i[12]}]
set_property PACKAGE_PIN AB22 [get_ports {adc_dat_a_i[13]}]
set_property PACKAGE_PIN AA21 [get_ports {adc_dat_a_i[14]}]
set_property PACKAGE_PIN AB21 [get_ports {adc_dat_a_i[15]}]

set_property IOSTANDARD LVCMOS33 [get_ports {adc_dat_b_i[*]}]
set_property IOB TRUE [get_ports {adc_dat_b_i[*]}]

#As 2d ADC not used for now, temporary set to the Y19 just to fill ports. See comment below.
set_property PACKAGE_PIN Y20 [get_ports {adc_dat_b_i[0]}]
set_property PACKAGE_PIN Y21 [get_ports {adc_dat_b_i[1]}]
set_property PACKAGE_PIN AB19 [get_ports {adc_dat_b_i[2]}]
set_property PACKAGE_PIN AB20 [get_ports {adc_dat_b_i[3]}]
set_property PACKAGE_PIN AA17 [get_ports {adc_dat_b_i[4]}]
set_property PACKAGE_PIN AB17 [get_ports {adc_dat_b_i[5]}]
set_property PACKAGE_PIN AA16 [get_ports {adc_dat_b_i[6]}]
set_property PACKAGE_PIN AB16 [get_ports {adc_dat_b_i[7]}]
set_property PACKAGE_PIN AB14 [get_ports {adc_dat_b_i[8]}]
set_property PACKAGE_PIN AB15 [get_ports {adc_dat_b_i[9]}]
set_property PACKAGE_PIN Y18 [get_ports {adc_dat_b_i[10]}]
set_property PACKAGE_PIN AA18 [get_ports {adc_dat_b_i[11]}]
set_property PACKAGE_PIN W17 [get_ports {adc_dat_b_i[12]}]
set_property PACKAGE_PIN W18 [get_ports {adc_dat_b_i[13]}]
set_property PACKAGE_PIN W16 [get_ports {adc_dat_b_i[14]}]
set_property PACKAGE_PIN Y16 [get_ports {adc_dat_b_i[15]}]

# clock input
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets {system_i/pll_0/inst/clk_in1_system_pll_0_0}]
set_property IOSTANDARD LVCMOS33 [get_ports adc_clk_i]
set_property PACKAGE_PIN Y19 [get_ports adc_clk_i]

# clock duty cycle stabilizer (CSn)

set_property IOSTANDARD LVCMOS33 [get_ports adc_csn_o]
set_property PACKAGE_PIN E21    [get_ports adc_csn_o]
set_property SLEW FAST [get_ports adc_csn_o]
set_property DRIVE 8 [get_ports adc_csn_o]

### DAC
set_property IOSTANDARD LVCMOS33 [get_ports {dac_dat_o[*]}]
set_property SLEW SLOW [get_ports {dac_dat_o[*]}]
set_property DRIVE 4 [get_ports {dac_dat_o[*]}]
# set_property IOB TRUE [get_ports {dac_dat_o[*]}]

set_property PACKAGE_PIN C19 [get_ports {dac_dat_o[0]}]
set_property PACKAGE_PIN D18 [get_ports {dac_dat_o[1]}]
set_property PACKAGE_PIN B20 [get_ports {dac_dat_o[2]}]
set_property PACKAGE_PIN B19 [get_ports {dac_dat_o[3]}]
set_property PACKAGE_PIN A22 [get_ports {dac_dat_o[4]}]
set_property PACKAGE_PIN A21 [get_ports {dac_dat_o[5]}]
set_property PACKAGE_PIN E18 [get_ports {dac_dat_o[6]}]
set_property PACKAGE_PIN F18 [get_ports {dac_dat_o[7]}]
set_property PACKAGE_PIN C18 [get_ports {dac_dat_o[8]}]
set_property PACKAGE_PIN C17 [get_ports {dac_dat_o[9]}]
set_property PACKAGE_PIN F17 [get_ports {dac_dat_o[10]}]
set_property PACKAGE_PIN G17 [get_ports {dac_dat_o[11]}]
set_property PACKAGE_PIN A19 [get_ports {dac_dat_o[12]}]
set_property PACKAGE_PIN A18 [get_ports {dac_dat_o[13]}]

#set_property PACKAGE_PIN A18 [get_ports {dac_dat_o[0]}]
#set_property PACKAGE_PIN A19 [get_ports {dac_dat_o[1]}]
#set_property PACKAGE_PIN G17 [get_ports {dac_dat_o[2]}]
#set_property PACKAGE_PIN F17 [get_ports {dac_dat_o[3]}]
#set_property PACKAGE_PIN C17 [get_ports {dac_dat_o[4]}]
#set_property PACKAGE_PIN C18 [get_ports {dac_dat_o[5]}]
#set_property PACKAGE_PIN F18 [get_ports {dac_dat_o[6]}]
#set_property PACKAGE_PIN E18 [get_ports {dac_dat_o[7]}]
#set_property PACKAGE_PIN A21 [get_ports {dac_dat_o[8]}]
#set_property PACKAGE_PIN A22 [get_ports {dac_dat_o[9]}]
#set_property PACKAGE_PIN B19 [get_ports {dac_dat_o[10]}]
#set_property PACKAGE_PIN B20 [get_ports {dac_dat_o[11]}]
#set_property PACKAGE_PIN D18 [get_ports {dac_dat_o[12]}]
#set_property PACKAGE_PIN C19 [get_ports {dac_dat_o[13]}]

# DAC stuff
set_property IOSTANDARD LVCMOS33 [get_ports dac_*_o]
set_property SLEW FAST [get_ports dac_*_o]
set_property DRIVE 8 [get_ports dac_*_o]

set_property PACKAGE_PIN J21 [get_ports dac_wrt_o]
set_property PACKAGE_PIN J22 [get_ports dac_sel_o]
#Clock
set_property PACKAGE_PIN A17 [get_ports dac_clk_o]
set_property PACKAGE_PIN K19 [get_ports dac_rst_o]

### Expansion connector
set_property IOSTANDARD LVCMOS33 [get_ports {exp_p_tri_io[*]}]
set_property IOSTANDARD LVCMOS33 [get_ports {exp_n_tri_io[*]}]
set_property SLEW FAST [get_ports {exp_p_tri_io[*]}]
set_property SLEW FAST [get_ports {exp_n_tri_io[*]}]
set_property DRIVE 8 [get_ports {exp_p_tri_io[*]}]
set_property DRIVE 8 [get_ports {exp_n_tri_io[*]}]

set_property PACKAGE_PIN D22 [get_ports {exp_p_tri_io[0]}]
set_property PACKAGE_PIN C22 [get_ports {exp_n_tri_io[0]}]
#E21
set_property PACKAGE_PIN AA19 [get_ports {exp_p_tri_io[1]}]
set_property PACKAGE_PIN D21 [get_ports {exp_n_tri_io[1]}]
set_property PACKAGE_PIN E19 [get_ports {exp_p_tri_io[2]}]
set_property PACKAGE_PIN E20 [get_ports {exp_n_tri_io[2]}]
set_property PACKAGE_PIN J16 [get_ports {exp_p_tri_io[3]}]
set_property PACKAGE_PIN J17 [get_ports {exp_n_tri_io[3]}]
set_property PACKAGE_PIN J18 [get_ports {exp_p_tri_io[4]}]
set_property PACKAGE_PIN K18 [get_ports {exp_n_tri_io[4]}]
set_property PACKAGE_PIN F21 [get_ports {exp_p_tri_io[5]}]
set_property PACKAGE_PIN F22 [get_ports {exp_n_tri_io[5]}]
set_property PACKAGE_PIN H19 [get_ports {exp_p_tri_io[6]}]
set_property PACKAGE_PIN H20 [get_ports {exp_n_tri_io[6]}]
set_property PACKAGE_PIN H22 [get_ports {exp_p_tri_io[7]}]
set_property PACKAGE_PIN G22 [get_ports {exp_n_tri_io[7]}]

#BCD Codes
set_property PULLTYPE PULLUP [get_ports {exp_n_tri_io[0]}]
set_property PULLTYPE PULLUP [get_ports {exp_n_tri_io[1]}]
set_property PULLTYPE PULLUP [get_ports {exp_n_tri_io[2]}]
set_property PULLTYPE PULLUP [get_ports {exp_n_tri_io[3]}]

set_property IOSTANDARD LVCMOS33 [get_ports {exp_n_alex[*]}]
set_property SLEW FAST [get_ports {exp_n_alex[*]}]
set_property DRIVE 8 [get_ports {exp_n_alex[*]}]

set_property PACKAGE_PIN B21 [get_ports {exp_n_alex[0]}]
set_property PACKAGE_PIN B22 [get_ports {exp_n_alex[1]}]
set_property PACKAGE_PIN D20 [get_ports {exp_n_alex[2]}]
set_property PACKAGE_PIN C20 [get_ports {exp_n_alex[3]}]

