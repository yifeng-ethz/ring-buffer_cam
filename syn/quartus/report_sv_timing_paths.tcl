project_open ring_buffer_cam_syn -revision ring_buffer_cam_syn_sv_p4
create_timing_netlist
read_sdc
update_timing_netlist

report_timing -setup -npaths 20 -detail full_path \
  -file output_files/ring_buffer_cam_syn_sv_p4/top_setup_paths.rpt
report_timing -hold -npaths 5 -detail summary \
  -file output_files/ring_buffer_cam_syn_sv_p4/top_hold_paths.rpt

delete_timing_netlist
project_close
