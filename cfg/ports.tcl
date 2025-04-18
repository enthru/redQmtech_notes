
### ADC

create_bd_port -dir I -from 15 -to 0 adc_dat_a_i
create_bd_port -dir I -from 15 -to 0 adc_dat_b_i

create_bd_port -dir I adc_clk_i
create_bd_port -dir O adc_csn_o

### DAC

create_bd_port -dir O -from 13 -to 0 dac_dat_o

create_bd_port -dir O dac_clk_o
create_bd_port -dir O dac_rst_o
create_bd_port -dir O dac_sel_o
create_bd_port -dir O dac_wrt_o

## Expansion connector

create_bd_port -dir IO -from 7 -to 0 exp_p_tri_io
create_bd_port -dir IO -from 7 -to 0 exp_n_tri_io
