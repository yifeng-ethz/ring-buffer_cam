package require -exact qsys 16.1

set SCRIPT_DIR [file dirname [info script]]
if {[string length $SCRIPT_DIR] == 0} {
    set SCRIPT_DIR [pwd]
}
set IP_DIR $SCRIPT_DIR
set IP_DIR_TAIL [file tail $IP_DIR]
set IP_DIR_IS_SCRIPT [string equal $IP_DIR_TAIL "script"]
if {$IP_DIR_IS_SCRIPT} {
    set IP_DIR [file dirname $IP_DIR]
}

set DEFAULT_SEARCH_KEY_WIDTH_CONST 8
set DEFAULT_RING_DEPTH_CONST       512
set DEFAULT_SIDE_DATA_BITS_CONST   31
set DEFAULT_INTERLEAVING_FACTOR_CONST 4
set DEFAULT_INTERLEAVING_INDEX_CONST  0
set DEFAULT_N_PARTITIONS_CONST     4
set DEFAULT_ENCODER_LEAF_WIDTH_CONST 16
set DEFAULT_PIPE_STAGES_CONST      4
set DEFAULT_DEBUG_CONST            1
set IP_UID_DEFAULT_CONST           1380074317
set BUILD_DEFAULT_CONST            507
set VERSION_MAJOR_DEFAULT_CONST    26
set VERSION_MINOR_DEFAULT_CONST    2
set VERSION_PATCH_DEFAULT_CONST    10
set VERSION_DATE_DEFAULT_CONST     20260507
set VERSION_GIT_DEFAULT_CONST      0
set VERSION_GIT_SHORT_DEFAULT_CONST "unknown"
set VERSION_GIT_DESCRIBE_DEFAULT_CONST "unknown"
if {![catch {set VERSION_GIT_SHORT_DEFAULT_CONST [string trim [exec git -C $IP_DIR rev-parse --short HEAD]]}]} {
    if {[regexp {^[0-9a-fA-F]+$} $VERSION_GIT_SHORT_DEFAULT_CONST]} {
        scan $VERSION_GIT_SHORT_DEFAULT_CONST %x VERSION_GIT_DEFAULT_CONST
        set VERSION_GIT_DEFAULT_CONST [expr {$VERSION_GIT_DEFAULT_CONST & 0x7fffffff}]
    }
}
catch {
    set VERSION_GIT_DESCRIBE_DEFAULT_CONST [string trim [exec git -C $IP_DIR describe --always --dirty --tags]]
}
set VERSION_GIT_HEX_DEFAULT_CONST [format "0x%08X" $VERSION_GIT_DEFAULT_CONST]
set VERSION_STRING_DEFAULT_CONST [format "%d.%d.%d.%04d" \
    $VERSION_MAJOR_DEFAULT_CONST \
    $VERSION_MINOR_DEFAULT_CONST \
    $VERSION_PATCH_DEFAULT_CONST \
    $BUILD_DEFAULT_CONST]
set INSTANCE_ID_DEFAULT_CONST      0
set SIGNOFF_ALMS_CONST             2191
set SIGNOFF_REGS_CONST             2861
set SIGNOFF_RAM_BLOCKS_CONST       19
set SIGNOFF_MEM_BITS_CONST         153600
set SIGNOFF_WNS_SLOW_85C_CONST     0.515
set SIGNOFF_WNS_SLOW_0C_CONST      0.575
set SIGNOFF_HOLD_WNS_CONST         0.171
set SIGNOFF_FMAX_CONST             147.97

set_module_property NAME ring_buffer_cam
set_module_property DISPLAY_NAME "Ring-buffer CAM"
set_module_property VERSION $VERSION_STRING_DEFAULT_CONST
set_module_property DESCRIPTION "Ring-buffer CAM Mu3e IP Core"
set_module_property GROUP "Mu3e Data Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK elaborate
set_module_property VALIDATION_CALLBACK validate

proc add_html_text {group_name item_name html_text} {
    add_display_item $group_name $item_name TEXT ""
    set_display_item_property $item_name DISPLAY_HINT html
    set_display_item_property $item_name TEXT $html_text
}

proc is_power_of_two {value} {
    if {$value < 1} {
        return 0
    }
    return [expr {($value & ($value - 1)) == 0}]
}

set CSR_ADDR_W_CONST            5
set CSR_LAST_WORD_CONST         20
set HIT_TYPE1_WIDTH_CONST       39
set HIT_TYPE2_WIDTH_CONST       36
set RUN_CONTROL_WIDTH_CONST     9
set FILLLEVEL_WIDTH_CONST       16
set SIDECAR_METADATA_WIDTH_CONST 64
set DEBUG_WORD_WIDTH_CONST      32

set CSR_TABLE_HTML {<html><table border="1" width="100%">
<tr><th>Word</th><th>Byte</th><th>Name</th><th>Access</th><th>Description</th></tr>
<tr><td>0x00</td><td>0x000</td><td>UID</td><td>RO</td><td>Software-visible IP identifier. Default is ASCII <b>RBCM</b> but it is integration-time overridable.</td></tr>
<tr><td>0x01</td><td>0x004</td><td>META</td><td>RW/RO</td><td>Read-multiplexed metadata word. Write <b>0</b>=VERSION, <b>1</b>=DATE, <b>2</b>=GIT, <b>3</b>=INSTANCE_ID. VERSION is packed as MAJOR[31:24], MINOR[23:16], PATCH[15:12], BUILD[11:0].</td></tr>
<tr><td>0x02</td><td>0x008</td><td>CTRL</td><td>RW</td><td>Bit 0 <b>go</b>, bit 1 <b>soft_reset</b>, bit 4 <b>filter_inerr</b>, bit 5 <b>counter_freeze</b>. Writing counter_freeze=1 snapshots all 64-bit diagnostic counters for coherent low/high-word readout.</td></tr>
<tr><td>0x03</td><td>0x00C</td><td>EXPECTED_LATENCY</td><td>RW</td><td>Read-pointer delay target in cycles. Reset default is 2000.</td></tr>
<tr><td>0x04</td><td>0x010</td><td>FILL_LEVEL</td><td>RO</td><td>Live fill-level estimate derived from push, pop, and overwrite counters.</td></tr>
<tr><td>0x05</td><td>0x014</td><td>INERR_COUNT_LO</td><td>RO</td><td>Low word of filtered ingress timestamp-error hit count.</td></tr>
<tr><td>0x06</td><td>0x018</td><td>PUSH_COUNT_LO</td><td>RO</td><td>Low word of accepted push operations.</td></tr>
<tr><td>0x07</td><td>0x01C</td><td>POP_COUNT_LO</td><td>RO</td><td>Low word of drained hits.</td></tr>
<tr><td>0x08</td><td>0x020</td><td>OVERWRITE_COUNT_LO</td><td>RO</td><td>Low word of overwrite events.</td></tr>
<tr><td>0x09</td><td>0x024</td><td>CACHE_MISS_COUNT_LO</td><td>RO</td><td>Low word of cache-miss / empty-search events.</td></tr>
<tr><td>0x0A</td><td>0x028</td><td>INERR_COUNT_HI</td><td>RO</td><td>High word of filtered ingress timestamp-error hit count.</td></tr>
<tr><td>0x0B</td><td>0x02C</td><td>PUSH_COUNT_HI</td><td>RO</td><td>High word of accepted push operations.</td></tr>
<tr><td>0x0C</td><td>0x030</td><td>POP_COUNT_HI</td><td>RO</td><td>High word of drained hits.</td></tr>
<tr><td>0x0D</td><td>0x034</td><td>OVERWRITE_COUNT_HI</td><td>RO</td><td>High word of overwrite events.</td></tr>
<tr><td>0x0E</td><td>0x038</td><td>CACHE_MISS_COUNT_HI</td><td>RO</td><td>High word of cache-miss / empty-search events.</td></tr>
<tr><td>0x0F</td><td>0x03C</td><td>DEASM_FULL_DROP_LO</td><td>RO</td><td>Low word of ingress/deassembly FIFO full-induced drop observations.</td></tr>
<tr><td>0x10</td><td>0x040</td><td>DEASM_FULL_DROP_HI</td><td>RO</td><td>High word of ingress/deassembly FIFO full-induced drop observations.</td></tr>
<tr><td>0x11</td><td>0x044</td><td>POP_CMD_FULL_DROP_LO</td><td>RO</td><td>Low word of pop-command FIFO full observations.</td></tr>
<tr><td>0x12</td><td>0x048</td><td>POP_CMD_FULL_DROP_HI</td><td>RO</td><td>High word of pop-command FIFO full observations.</td></tr>
<tr><td>0x13</td><td>0x04C</td><td>EGRESS_NOT_READY_DROP_LO</td><td>RO</td><td>Low word of egress hit words generated while the downstream sink was not ready.</td></tr>
<tr><td>0x14</td><td>0x050</td><td>EGRESS_NOT_READY_DROP_HI</td><td>RO</td><td>High word of egress hit words generated while the downstream sink was not ready.</td></tr>
</table></html>}

proc compute_derived_values {} {
    set search_key_width    [get_parameter_value SEARCH_KEY_WIDTH]
    set ring_depth          [get_parameter_value RING_BUFFER_N_ENTRY]
    set side_data_bits      [get_parameter_value SIDE_DATA_BITS]
    set interleaving_factor [get_parameter_value INTERLEAVING_FACTOR]
    set n_partitions        [get_parameter_value N_PARTITIONS]
    set encoder_leaf_width  [get_parameter_value ENCODER_LEAF_WIDTH]
    set encoder_pipe_stages [get_parameter_value ENCODER_PIPE_STAGES]

    set partition_depth     [expr {$ring_depth / $n_partitions}]
    set interleaving_bits   0
    set entry_addr_w        0
    set partition_addr_w    0
    set cam_store_bits      [expr {((($search_key_width + 7) / 8) * 8) * $ring_depth}]
    set side_store_bits     [expr {(1 + $search_key_width + $side_data_bits) * $ring_depth}]
    set nominal_hits_per_cycle [expr {double($n_partitions) / double($encoder_pipe_stages)}]
    set nominal_limit_pct      100
    set nominal_cycles_per_hit 1.0
    set worst_hits_per_cycle   [expr {1.0 / double($encoder_pipe_stages)}]
    set worst_limit_pct        [expr {int(round($worst_hits_per_cycle * 100.0))}]
    set hides_refill_latency   0

    if {$interleaving_factor > 1} {
        set interleaving_bits [expr {int(round(log($interleaving_factor) / log(2.0)))}]
    }
    if {$ring_depth > 1} {
        set entry_addr_w [expr {int(ceil(log($ring_depth) / log(2.0)))}]
    }
    if {$partition_depth > 1} {
        set partition_addr_w [expr {int(ceil(log($partition_depth) / log(2.0)))}]
    }
    if {$nominal_hits_per_cycle >= 1.0} {
        set nominal_hits_per_cycle 1.0
        set nominal_limit_pct      100
        set nominal_cycles_per_hit 1.0
        set hides_refill_latency   1
    } else {
        set nominal_limit_pct      [expr {int(round($nominal_hits_per_cycle * 100.0))}]
        set nominal_cycles_per_hit [expr {double($encoder_pipe_stages) / double($n_partitions)}]
    }

    set_parameter_value PARTITION_DEPTH_DERIVED      $partition_depth
    set_parameter_value ENTRY_ADDR_W_DERIVED         $entry_addr_w
    set_parameter_value PARTITION_ADDR_W_DERIVED     $partition_addr_w
    set_parameter_value INTERLEAVING_BITS_DERIVED    $interleaving_bits
    set_parameter_value ENTRY_PAYLOAD_BITS_DERIVED   [expr {$search_key_width + $side_data_bits}]
    set_parameter_value CAM_STORE_BITS_DERIVED       $cam_store_bits
    set_parameter_value SIDE_STORE_BITS_DERIVED      $side_store_bits

    catch {
        set_display_item_property sizing_html TEXT "<html><b>Derived sizing</b><br/>Global depth: <b>${ring_depth}</b> entries<br/>Partition depth: <b>${partition_depth}</b> entries per encoder slice<br/>Entry address width: <b>${entry_addr_w}</b> bits<br/>Partition address width: <b>${partition_addr_w}</b> bits<br/>Interleaving bits consumed from timestamp: <b>${interleaving_bits}</b><br/>Byte-aligned CAM storage: <b>${cam_store_bits}</b> bits<br/>Side-RAM storage: <b>${side_store_bits}</b> bits</html>"
    }
    catch {
        set_display_item_property encoder_html TEXT "<html><b>Encoder operating point</b><br/>Leaf width: <b>${encoder_leaf_width}</b><br/>Pipeline stages: <b>${encoder_pipe_stages}</b><br/>The delivered default is <b>N_PARTITIONS=4</b> with <b>ENCODER_PIPE_STAGES=4</b> so the physical partition count matches the staged encoder depth. Presets keep <b>P2</b> and <b>P3</b> available for timing/resource trade studies.</html>"
    }
    catch {
        set refill_note "Full 1 hit/cycle refill hiding is not expected for this setting."
        if {$hides_refill_latency} {
            set refill_note "This setting has enough partitions to hide the encoder refill latency in an interleaved round-robin schedule."
        }
        set_display_item_property throughput_html TEXT [format "<html><b>Expected pop throughput</b><br/>Nominal distributed-case ceiling: <b>%.2f</b> hit/cycle (about <b>%d%%</b>, <b>%.2f</b> cycles/hit)<br/>Worst case, all matches stay in one partition: <b>%.2f</b> hit/cycle (about <b>%d%%</b>, <b>%d</b> cycles/hit)<br/>Interpretation: %s</html>" $nominal_hits_per_cycle $nominal_limit_pct $nominal_cycles_per_hit $worst_hits_per_cycle $worst_limit_pct $encoder_pipe_stages $refill_note]
    }
}

proc validate {} {
    compute_derived_values

    set search_key_width    [get_parameter_value SEARCH_KEY_WIDTH]
    set ring_depth          [get_parameter_value RING_BUFFER_N_ENTRY]
    set side_data_bits      [get_parameter_value SIDE_DATA_BITS]
    set interleaving_factor [get_parameter_value INTERLEAVING_FACTOR]
    set interleaving_index  [get_parameter_value INTERLEAVING_INDEX]
    set n_partitions        [get_parameter_value N_PARTITIONS]
    set encoder_leaf_width  [get_parameter_value ENCODER_LEAF_WIDTH]
    set encoder_pipe_stages [get_parameter_value ENCODER_PIPE_STAGES]
    set partition_depth     [get_parameter_value PARTITION_DEPTH_DERIVED]
    set ip_uid_value        [get_parameter_value IP_UID]
    set build_value         [get_parameter_value BUILD]
    set version_major       [get_parameter_value VERSION_MAJOR]
    set version_minor       [get_parameter_value VERSION_MINOR]
    set version_patch       [get_parameter_value VERSION_PATCH]
    set version_date        [get_parameter_value VERSION_DATE]
    set version_git         [get_parameter_value VERSION_GIT]
    set instance_id         [get_parameter_value INSTANCE_ID]
    set debug_level         [get_parameter_value DEBUG]

    if {![is_power_of_two $ring_depth]} {
        send_message error "RING_BUFFER_N_ENTRY must be a power of two."
    }
    if {$ring_depth < 32} {
        send_message error "RING_BUFFER_N_ENTRY must be at least 32."
    }
    if {$search_key_width < 1 || $search_key_width > 512} {
        send_message error "SEARCH_KEY_WIDTH must stay in the range 1..512."
    }
    if {$side_data_bits < 0 || $side_data_bits > 512} {
        send_message error "SIDE_DATA_BITS must stay in the range 0..512."
    }
    if {![is_power_of_two $interleaving_factor]} {
        send_message error "INTERLEAVING_FACTOR must be a power of two."
    }
    if {$interleaving_factor < 1 || $interleaving_factor > 32} {
        send_message error "INTERLEAVING_FACTOR must stay in the range 1,2,4,8,16,32."
    }
    if {$interleaving_index < 0 || $interleaving_index >= $interleaving_factor} {
        send_message error "INTERLEAVING_INDEX must stay in the range 0..INTERLEAVING_FACTOR-1."
    }
    if {![is_power_of_two $n_partitions]} {
        send_message error "N_PARTITIONS must be a power of two."
    }
    if {$n_partitions < 1 || $n_partitions > 8} {
        send_message error "N_PARTITIONS must stay in the range 1,2,4,8."
    }
    if {$ring_depth % $n_partitions != 0} {
        send_message error "RING_BUFFER_N_ENTRY must be divisible by N_PARTITIONS."
    }
    if {$n_partitions > $ring_depth} {
        send_message error "N_PARTITIONS cannot exceed RING_BUFFER_N_ENTRY."
    }
    if {![is_power_of_two $encoder_leaf_width]} {
        send_message error "ENCODER_LEAF_WIDTH must be a power of two."
    }
    if {$encoder_leaf_width < 4 || $encoder_leaf_width > 64} {
        send_message error "ENCODER_LEAF_WIDTH must stay in the range 4,8,16,32,64."
    }
    if {$encoder_leaf_width > $partition_depth} {
        send_message error "ENCODER_LEAF_WIDTH cannot exceed the derived partition depth."
    }
    if {$encoder_pipe_stages < 1 || $encoder_pipe_stages > 4} {
        send_message error "ENCODER_PIPE_STAGES must stay in the range 1..4."
    }
    if {$ip_uid_value < 0 || $ip_uid_value > 2147483647} {
        send_message error "IP_UID must stay in the signed 31-bit Platform Designer integer range."
    }
    if {$build_value < 0 || $build_value > 4095} {
        send_message error "BUILD must stay in the range 0..4095."
    }
    if {$version_major < 0 || $version_major > 255} {
        send_message error "VERSION_MAJOR must stay in the range 0..255."
    }
    if {$version_minor < 0 || $version_minor > 255} {
        send_message error "VERSION_MINOR must stay in the range 0..255."
    }
    if {$version_patch < 0 || $version_patch > 15} {
        send_message error "VERSION_PATCH must stay in the range 0..15."
    }
    if {$version_date < 0 || $version_date > 2147483647} {
        send_message error "VERSION_DATE must stay in the signed 31-bit Platform Designer integer range."
    }
    if {$version_git < 0 || $version_git > 2147483647} {
        send_message error "VERSION_GIT must stay in the signed 31-bit Platform Designer integer range."
    }
    if {$instance_id < 0 || $instance_id > 2147483647} {
        send_message error "INSTANCE_ID must stay in the signed 31-bit Platform Designer integer range."
    }
    if {$debug_level < 0 || $debug_level > 2} {
        send_message error "DEBUG must stay in the range 0..2."
    }
}

proc elaborate {} {
    compute_derived_values

    set interleaving_factor [get_parameter_value INTERLEAVING_FACTOR]
    set interleaving_index_max [expr {$interleaving_factor - 1}]
    set debug_level [get_parameter_value DEBUG]
    set debug_conduit_enabled false
    set sidecar_conduit_enabled false
    if {$debug_level >= 1} {
        set debug_conduit_enabled true
    }
    if {$debug_level >= 2} {
        set sidecar_conduit_enabled true
    }

    set_parameter_property VERSION_MAJOR ENABLED false
    set_parameter_property VERSION_MINOR ENABLED false
    set_parameter_property VERSION_PATCH ENABLED false
    set_parameter_property BUILD ENABLED false
    set_parameter_property VERSION_DATE ENABLED false
    set_parameter_property VERSION_GIT ENABLED false

    set_parameter_property RING_BUFFER_N_ENTRY ALLOWED_RANGES {32 64 128 256 512 1024 2048}
    set_parameter_property INTERLEAVING_FACTOR ALLOWED_RANGES {1 2 4 8 16 32}
    set_parameter_property INTERLEAVING_INDEX ALLOWED_RANGES 0:$interleaving_index_max
    set_parameter_property N_PARTITIONS ALLOWED_RANGES {1 2 4 8}
    set_parameter_property ENCODER_LEAF_WIDTH ALLOWED_RANGES {4 8 16 32 64}

    set_interface_property debug_observability ENABLED $debug_conduit_enabled
    set_interface_property hit_type1_metadata ENABLED $sidecar_conduit_enabled
    set_interface_property hit_type2_metadata ENABLED $sidecar_conduit_enabled
}

add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL ring_buffer_cam
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file ring_buffer_cam.vhd VHDL PATH ../rtl/vhd_ver/ring_buffer_cam.vhd TOP_LEVEL_FILE
add_fileset_file ring_buffer_cam_v2_core.vhd VHDL PATH ../rtl/vhd_ver/ring_buffer_cam_v2_core.vhd
add_fileset_file alt_simple_dpram.vhd VHDL PATH ../rtl/common/alt_simple_dpram.vhd
add_fileset_file cam_helper_pkg.vhd VHDL PATH ../rtl/vhd_ver/cam_helper_pkg.vhd
add_fileset_file cam_mem_a5.vhd VHDL PATH ../rtl/vhd_ver/cam_mem_a5.vhd
add_fileset_file cam_mem_blk_a5.vhd VHDL PATH ../rtl/vhd_ver/cam_mem_blk_a5.vhd
add_fileset_file scfifo_w40d256.vhd VHDL PATH ../rtl/common/alt_fifo/scfifo_w40d256.vhd
add_fileset_file cmd_fifo.vhd VHDL PATH ../rtl/common/alt_fifo/cmd_fifo/cmd_fifo.vhd
add_fileset_file addr_enc_logic_partitioned.vhd VHDL PATH ../rtl/vhd_ver/addr_enc_logic_partitioned.vhd

add_parameter SEARCH_KEY_WIDTH NATURAL $DEFAULT_SEARCH_KEY_WIDTH_CONST
set_parameter_property SEARCH_KEY_WIDTH DISPLAY_NAME "Search Key Width"
set_parameter_property SEARCH_KEY_WIDTH UNITS Bits
set_parameter_property SEARCH_KEY_WIDTH HDL_PARAMETER true
set_parameter_property SEARCH_KEY_WIDTH DESCRIPTION {Bit width of the CAM search key. The default 8-bit key corresponds to timestamp[11:4].}

add_parameter RING_BUFFER_N_ENTRY NATURAL $DEFAULT_RING_DEPTH_CONST
set_parameter_property RING_BUFFER_N_ENTRY DISPLAY_NAME "Ring Depth"
set_parameter_property RING_BUFFER_N_ENTRY UNITS None
set_parameter_property RING_BUFFER_N_ENTRY HDL_PARAMETER true
set_parameter_property RING_BUFFER_N_ENTRY DESCRIPTION "Total ring-buffer / CAM depth. Use powers of two. The delivered default is 512 for simulation and standalone signoff."

add_parameter SIDE_DATA_BITS NATURAL $DEFAULT_SIDE_DATA_BITS_CONST
set_parameter_property SIDE_DATA_BITS DISPLAY_NAME "Side Data Width"
set_parameter_property SIDE_DATA_BITS UNITS Bits
set_parameter_property SIDE_DATA_BITS HDL_PARAMETER true
set_parameter_property SIDE_DATA_BITS DESCRIPTION "Side-band payload width stored alongside the search key."

add_parameter INTERLEAVING_FACTOR NATURAL $DEFAULT_INTERLEAVING_FACTOR_CONST
set_parameter_property INTERLEAVING_FACTOR DISPLAY_NAME "Interleaving Factor"
set_parameter_property INTERLEAVING_FACTOR UNITS None
set_parameter_property INTERLEAVING_FACTOR HDL_PARAMETER true
set_parameter_property INTERLEAVING_FACTOR DESCRIPTION "Power-of-two time-interleaving fanout shared across the surrounding sorter complex."

add_parameter INTERLEAVING_INDEX NATURAL $DEFAULT_INTERLEAVING_INDEX_CONST
set_parameter_property INTERLEAVING_INDEX DISPLAY_NAME "Interleaving Index"
set_parameter_property INTERLEAVING_INDEX UNITS None
set_parameter_property INTERLEAVING_INDEX HDL_PARAMETER true
set_parameter_property INTERLEAVING_INDEX DESCRIPTION "Modulo index accepted by this IP instance within the larger interleaving complex."

add_parameter N_PARTITIONS NATURAL $DEFAULT_N_PARTITIONS_CONST
set_parameter_property N_PARTITIONS DISPLAY_NAME "Physical Match Partitions"
set_parameter_property N_PARTITIONS UNITS None
set_parameter_property N_PARTITIONS HDL_PARAMETER true
set_parameter_property N_PARTITIONS DESCRIPTION "Physical one-hot match partitions. The delivered default uses four partitions to match the four-stage encoder pipeline. Lower-partition presets remain available for timing/resource comparisons and regression."

add_parameter ENCODER_LEAF_WIDTH NATURAL $DEFAULT_ENCODER_LEAF_WIDTH_CONST
set_parameter_property ENCODER_LEAF_WIDTH DISPLAY_NAME "Encoder Leaf Width"
set_parameter_property ENCODER_LEAF_WIDTH UNITS None
set_parameter_property ENCODER_LEAF_WIDTH HDL_PARAMETER true
set_parameter_property ENCODER_LEAF_WIDTH DESCRIPTION "Leaf slice width used by the staged one-hot encoder."

add_parameter ENCODER_PIPE_STAGES NATURAL $DEFAULT_PIPE_STAGES_CONST
set_parameter_property ENCODER_PIPE_STAGES DISPLAY_NAME "Encoder Pipe Stages"
set_parameter_property ENCODER_PIPE_STAGES UNITS None
set_parameter_property ENCODER_PIPE_STAGES HDL_PARAMETER true
set_parameter_property ENCODER_PIPE_STAGES DESCRIPTION "Internal pipeline depth of the staged encoder. The delivered default is P4. Presets keep P2 and P3 available."

add_parameter DEBUG NATURAL $DEFAULT_DEBUG_CONST
set_parameter_property DEBUG DISPLAY_NAME "Debug Level"
set_parameter_property DEBUG UNITS None
set_parameter_property DEBUG ALLOWED_RANGES 0:2
set_parameter_property DEBUG HDL_PARAMETER true
set_parameter_property DEBUG AFFECTS_ELABORATION true
set_parameter_property DEBUG DESCRIPTION "Debug level exported by the RTL. 0 keeps the nominal datapath with optional outputs tied low, 1 enables synthesizable FIFO/fill/queue observability conduits, 2 also enables the 64-bit per-hit metadata sidecar."

add_parameter IP_UID NATURAL $IP_UID_DEFAULT_CONST
set_parameter_property IP_UID DISPLAY_NAME "UID"
set_parameter_property IP_UID UNITS None
set_parameter_property IP_UID ALLOWED_RANGES 0:2147483647
set_parameter_property IP_UID HDL_PARAMETER true
set_parameter_property IP_UID DISPLAY_HINT hexadecimal
set_parameter_property IP_UID DESCRIPTION {Software-visible IP identifier. Default corresponds to ASCII "RBCM".}

add_parameter VERSION_MAJOR NATURAL $VERSION_MAJOR_DEFAULT_CONST
set_parameter_property VERSION_MAJOR DISPLAY_NAME "Version Major"
set_parameter_property VERSION_MAJOR UNITS None
set_parameter_property VERSION_MAJOR ALLOWED_RANGES 0:255
set_parameter_property VERSION_MAJOR HDL_PARAMETER true

add_parameter VERSION_MINOR NATURAL $VERSION_MINOR_DEFAULT_CONST
set_parameter_property VERSION_MINOR DISPLAY_NAME "Version Minor"
set_parameter_property VERSION_MINOR UNITS None
set_parameter_property VERSION_MINOR ALLOWED_RANGES 0:255
set_parameter_property VERSION_MINOR HDL_PARAMETER true

add_parameter VERSION_PATCH NATURAL $VERSION_PATCH_DEFAULT_CONST
set_parameter_property VERSION_PATCH DISPLAY_NAME "Version Patch"
set_parameter_property VERSION_PATCH UNITS None
set_parameter_property VERSION_PATCH ALLOWED_RANGES 0:15
set_parameter_property VERSION_PATCH HDL_PARAMETER true

add_parameter BUILD NATURAL $BUILD_DEFAULT_CONST
set_parameter_property BUILD DISPLAY_NAME "Build Stamp"
set_parameter_property BUILD UNITS None
set_parameter_property BUILD ALLOWED_RANGES 0:4095
set_parameter_property BUILD HDL_PARAMETER true
set_parameter_property BUILD DESCRIPTION {12-bit build stamp packed into VERSION[11:0].}

add_parameter VERSION_DATE NATURAL $VERSION_DATE_DEFAULT_CONST
set_parameter_property VERSION_DATE DISPLAY_NAME "Version Date"
set_parameter_property VERSION_DATE UNITS None
set_parameter_property VERSION_DATE ALLOWED_RANGES 0:2147483647
set_parameter_property VERSION_DATE HDL_PARAMETER true
set_parameter_property VERSION_DATE DESCRIPTION {YYYYMMDD provenance word exposed through META when software writes selector 1.}

add_parameter VERSION_GIT NATURAL $VERSION_GIT_DEFAULT_CONST
set_parameter_property VERSION_GIT DISPLAY_NAME "Git Stamp"
set_parameter_property VERSION_GIT UNITS None
set_parameter_property VERSION_GIT ALLOWED_RANGES 0:2147483647
set_parameter_property VERSION_GIT HDL_PARAMETER true
set_parameter_property VERSION_GIT DISPLAY_HINT hexadecimal
set_parameter_property VERSION_GIT DESCRIPTION {Truncated build git hash exposed through META when software writes selector 2.}

add_parameter INSTANCE_ID NATURAL $INSTANCE_ID_DEFAULT_CONST
set_parameter_property INSTANCE_ID DISPLAY_NAME "Instance ID"
set_parameter_property INSTANCE_ID UNITS None
set_parameter_property INSTANCE_ID ALLOWED_RANGES 0:2147483647
set_parameter_property INSTANCE_ID HDL_PARAMETER true
set_parameter_property INSTANCE_ID DESCRIPTION {Integration-time instance identifier exposed through META when software writes selector 3.}

foreach derived_name {PARTITION_DEPTH_DERIVED ENTRY_ADDR_W_DERIVED PARTITION_ADDR_W_DERIVED INTERLEAVING_BITS_DERIVED ENTRY_PAYLOAD_BITS_DERIVED CAM_STORE_BITS_DERIVED SIDE_STORE_BITS_DERIVED} {
    add_parameter $derived_name NATURAL 0
    set_parameter_property $derived_name HDL_PARAMETER false
    set_parameter_property $derived_name DERIVED true
    set_parameter_property $derived_name VISIBLE false
}

set TAB_CONFIGURATION "Configuration"
set TAB_IDENTITY      "Identity"
set TAB_INTERFACES    "Interfaces"
set TAB_REGMAP        "Register Map"

add_display_item "" $TAB_CONFIGURATION GROUP tab
add_display_item $TAB_CONFIGURATION "Overview" GROUP
add_display_item $TAB_CONFIGURATION "Sizing" GROUP
add_display_item $TAB_CONFIGURATION "Delivered Footprint" GROUP
add_display_item $TAB_CONFIGURATION "Match Engine" GROUP
add_display_item $TAB_CONFIGURATION "Throughput" GROUP
add_display_item $TAB_CONFIGURATION "Advanced" GROUP

add_html_text "Overview" overview_html "<html><b>Function</b><br/>This IP stores timestamp-tagged hits in a ring-buffer shaped CAM and drains them in timestamp order. The delivered release keeps the established Avalon-MM and Avalon-ST system contract, uses the active partitioned V2 core, and adds the common CSR identity header at words <b>0</b> and <b>1</b>.<br/><br/><b>Clocking</b><br/>All interfaces run inside a single synchronous domain shared by CSR, ingress, pop control, ordered egress, and fill-level monitoring.</html>"
add_display_item "Sizing" SEARCH_KEY_WIDTH parameter
add_display_item "Sizing" RING_BUFFER_N_ENTRY parameter
add_display_item "Sizing" SIDE_DATA_BITS parameter
add_html_text "Sizing" sizing_html "<html><b>Derived sizing</b><br/>Updated by the validation callback.</html>"
add_html_text "Delivered Footprint" resources_html [format {<html><b>Standalone signoff footprint</b><br/>Numbers below are for the delivered default signoff shape <b>RING_BUFFER_N_ENTRY=%d</b>, <b>N_PARTITIONS=%d</b>, <b>ENCODER_PIPE_STAGES=%d</b> on Arria V <b>5AGXBA7D4F31C5</b>.<br/><br/><b>Fitter resources</b><br/>ALMs: <b>%d</b><br/>Registers: <b>%d</b><br/>RAM blocks: <b>%d</b><br/>Block memory bits: <b>%d</b><br/><br/><b>Timing at 137.5 MHz standalone signoff</b><br/>Slow 85C WNS: <b>%.3f ns</b><br/>Slow 0C WNS: <b>%.3f ns</b><br/>Worst reported hold slack: <b>%.3f ns</b><br/>Slow-corner Fmax: <b>%.2f MHz</b><br/><br/><b>Interpretation</b><br/>These values are packaging-time signoff evidence for the delivered profile. Changing visible sizing or encoder parameters alters the implementation footprint and timing.</html>} \
    $DEFAULT_RING_DEPTH_CONST \
    $DEFAULT_N_PARTITIONS_CONST \
    $DEFAULT_PIPE_STAGES_CONST \
    $SIGNOFF_ALMS_CONST \
    $SIGNOFF_REGS_CONST \
    $SIGNOFF_RAM_BLOCKS_CONST \
    $SIGNOFF_MEM_BITS_CONST \
    $SIGNOFF_WNS_SLOW_85C_CONST \
    $SIGNOFF_WNS_SLOW_0C_CONST \
    $SIGNOFF_HOLD_WNS_CONST \
    $SIGNOFF_FMAX_CONST]
add_display_item "Match Engine" INTERLEAVING_FACTOR parameter
add_display_item "Match Engine" INTERLEAVING_INDEX parameter
add_display_item "Match Engine" N_PARTITIONS parameter
add_display_item "Match Engine" ENCODER_LEAF_WIDTH parameter
add_display_item "Match Engine" ENCODER_PIPE_STAGES parameter
add_html_text "Match Engine" encoder_html "<html><b>Compatibility note</b><br/>The visible partition and encoder parameters are retained for catalog compatibility with existing systems. This delivered release uses the partitioned V2 core, so the visible encoder settings now match the live datapath again.</html>"
add_html_text "Throughput" throughput_html "<html><b>Expected pop throughput</b><br/>Updated by the validation callback.</html>"
add_html_text "Advanced" advanced_html "<html><b>Integration notes</b><br/>1. The CSR aperture is fixed at <b>10</b> words, with the common identity header in words <b>0</b> and <b>1</b>.<br/>2. Interleaving filters the accepted timestamp lane before hits enter the local ring store.<br/>3. The partitioned encoder parameters remain explicit so existing Qsys systems can regenerate in place while the live datapath stays auditable.</html>"

add_display_item "" $TAB_IDENTITY GROUP tab
add_display_item $TAB_IDENTITY "Delivered Profile" GROUP
add_display_item $TAB_IDENTITY "Versioning" GROUP
add_display_item $TAB_IDENTITY "Debug" GROUP

add_html_text "Delivered Profile" profile_html [format {<html><b>Catalog revision</b><br/>This release is packaged as <b>%s</b>.<br/><br/><b>Packaging provenance</b><br/>Default git stamp <b>%s</b> (%s). Git describe: <b>%s</b>.<br/><br/><b>Delivered interface contract</b><br/>The packaged image keeps the established <b>hit_type1</b>, <b>hit_type2</b>, <b>run_control</b>, and <b>filllevel</b> interface names so existing Platform Designer systems can be upgraded in place while picking up the common CSR identity header, the locked <b>BUILD=%d</b> / <b>VERSION_DATE=%d</b> metadata, the earlier low-stage partitioned-encoder and non-power-of-two ring-depth repairs, the carried overwrite erase-slot timing closure, the settled SEARCH-tail conservative overwrite-slot guard, and DEBUG-gated observability / metadata sidecar conduits. With <b>DEBUG=0</b>, optional debug outputs are deterministically zero and the nominal datapath is unchanged.<br/><br/><b>Delivered signoff profile</b><br/>The packaged default shape signs off at <b>%.2f MHz</b> slow-corner Fmax with <b>%d ALMs</b>, <b>%d</b> registers, and <b>%d</b> RAM blocks. The Configuration tab shows the full standalone footprint summary used for this delivered image.<br/><br/><b>Runtime visibility</b><br/>Software can blind-scan the CSR window through <b>UID</b> at word <b>0</b> and the <b>META</b> mux at word <b>1</b>.</html>} \
    $VERSION_STRING_DEFAULT_CONST \
    $VERSION_GIT_HEX_DEFAULT_CONST \
    $VERSION_GIT_SHORT_DEFAULT_CONST \
    $VERSION_GIT_DESCRIBE_DEFAULT_CONST \
    $BUILD_DEFAULT_CONST \
    $VERSION_DATE_DEFAULT_CONST \
    $SIGNOFF_FMAX_CONST \
    $SIGNOFF_ALMS_CONST \
    $SIGNOFF_REGS_CONST \
    $SIGNOFF_RAM_BLOCKS_CONST]
add_html_text "Versioning" versioning_html [format {<html><b>Common identity header</b><br/>Word <b>0</b> is <b>UID</b>.<br/>Word <b>1</b> is <b>META</b>: write 0=VERSION, 1=DATE, 2=GIT, 3=INSTANCE_ID.<br/><br/><b>VERSION encoding</b><br/>VERSION[31:24] = MAJOR, VERSION[23:16] = MINOR, VERSION[15:12] = PATCH, VERSION[11:0] = BUILD.<br/><br/><b>Packaged defaults</b><br/>Default <b>VERSION_GIT</b> = <b>%s</b> (%s). Git describe = <b>%s</b>.<br/><br/><b>Editability</b><br/><b>IP_UID</b> and <b>INSTANCE_ID</b> remain integration-editable. Version, build, date, and git provenance fields are locked to the packaged image.</html>} \
    $VERSION_GIT_HEX_DEFAULT_CONST \
    $VERSION_GIT_SHORT_DEFAULT_CONST \
    $VERSION_GIT_DESCRIBE_DEFAULT_CONST]
add_display_item "Versioning" IP_UID parameter
add_display_item "Versioning" VERSION_MAJOR parameter
add_display_item "Versioning" VERSION_MINOR parameter
add_display_item "Versioning" VERSION_PATCH parameter
add_display_item "Versioning" BUILD parameter
add_display_item "Versioning" VERSION_DATE parameter
add_display_item "Versioning" VERSION_GIT parameter
add_display_item "Versioning" INSTANCE_ID parameter
add_html_text "Debug" debug_html "<html><b>Debug control</b><br/><b>DEBUG=0</b> keeps optional observability and metadata outputs tied to zero. <b>DEBUG=1</b> enables the synthesizable <b>debug_observability</b> conduit with live fill, FIFO levels, and queue state. <b>DEBUG=2</b> also enables a 64-bit per-hit metadata sidecar from <b>hit_type1_metadata</b> to <b>hit_type2_metadata</b>.</html>"
add_display_item "Debug" DEBUG parameter

add_display_item "" $TAB_INTERFACES GROUP tab
add_display_item $TAB_INTERFACES "Clock / Reset" GROUP
add_display_item $TAB_INTERFACES "Data Path" GROUP
add_display_item $TAB_INTERFACES "Control Path" GROUP
add_display_item $TAB_INTERFACES "Monitoring" GROUP

add_html_text "Clock / Reset" clock_html "<html><b>clock_interface</b> and <b>reset_interface</b><br/>Single synchronous clock/reset domain for the full CAM datapath and CSR logic.</html>"
add_html_text "Data Path" datapath_html "<html><b>hit_type1</b><br/>39-bit Avalon-ST sink carrying the Type-1 hit payload plus a timestamp-error sideband. When <b>DEBUG=2</b>, the optional <b>hit_type1_metadata</b> conduit accepts one 64-bit metadata word on each accepted non-empty hit beat.<br/><br/><b>hit_type2</b><br/>36-bit Avalon-ST source that emits ordered Type-2 hits after CAM lookup and pop arbitration. When <b>DEBUG=2</b>, <b>hit_type2_metadata</b> carries the stored metadata aligned with output hit beats; subheaders and invalid cycles drive zero.</html>"
add_html_text "Control Path" control_html "<html><b>csr</b> and <b>run_control</b><br/>Word-addressed Avalon-MM CSR window plus a 9-bit Avalon-ST run-control sink. The run-control path does not alter the external latency contract when encoder pipeline presets are changed.</html>"
add_html_text "Monitoring" monitor_html "<html><b>filllevel</b><br/>16-bit Avalon-ST source for live fill-level reporting and integration-time occupancy monitoring.<br/><br/><b>debug_observability</b><br/>Optional DEBUG>=1 conduit exposing 32-bit fill level, FIFO levels/status, and compact queue-state snapshots for hardware bring-up.</html>"

add_display_item "" $TAB_REGMAP GROUP tab
add_display_item $TAB_REGMAP "CSR Window" GROUP
add_html_text "CSR Window" csr_table_html $CSR_TABLE_HTML

add_interface csr avalon end
set_interface_property csr addressUnits WORDS
set_interface_property csr associatedClock clock_interface
set_interface_property csr associatedReset reset_interface
set_interface_property csr bitsPerSymbol 8
set_interface_property csr burstOnBurstBoundariesOnly false
set_interface_property csr burstcountUnits WORDS
set_interface_property csr explicitAddressSpan 0
set_interface_property csr holdTime 0
set_interface_property csr linewrapBursts false
set_interface_property csr maximumPendingReadTransactions 0
set_interface_property csr maximumPendingWriteTransactions 0
set_interface_property csr readLatency 1
set_interface_property csr readWaitTime 1
set_interface_property csr setupTime 0
set_interface_property csr timingUnits Cycles
set_interface_property csr writeWaitTime 0
set_interface_property csr ENABLED true
add_interface_port csr avs_csr_readdata readdata Output 32
add_interface_port csr avs_csr_read read Input 1
add_interface_port csr avs_csr_address address Input $CSR_ADDR_W_CONST
add_interface_port csr avs_csr_waitrequest waitrequest Output 1
add_interface_port csr avs_csr_write write Input 1
add_interface_port csr avs_csr_writedata writedata Input 32
set_interface_assignment csr embeddedsw.configuration.isFlash 0
set_interface_assignment csr embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment csr embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment csr embeddedsw.configuration.isPrintableDevice 0

add_interface hit_type1 avalon_streaming end
set_interface_property hit_type1 associatedClock clock_interface
set_interface_property hit_type1 associatedReset reset_interface
set_interface_property hit_type1 dataBitsPerSymbol $HIT_TYPE1_WIDTH_CONST
set_interface_property hit_type1 errorDescriptor {"tserr"}
set_interface_property hit_type1 firstSymbolInHighOrderBits true
set_interface_property hit_type1 maxChannel 15
set_interface_property hit_type1 readyLatency 0
set_interface_property hit_type1 ENABLED true
add_interface_port hit_type1 asi_hit_type1_channel channel Input 4
add_interface_port hit_type1 asi_hit_type1_startofpacket startofpacket Input 1
add_interface_port hit_type1 asi_hit_type1_endofpacket endofpacket Input 1
add_interface_port hit_type1 asi_hit_type1_empty empty Input 1
add_interface_port hit_type1 asi_hit_type1_data data Input $HIT_TYPE1_WIDTH_CONST
add_interface_port hit_type1 asi_hit_type1_valid valid Input 1
add_interface_port hit_type1 asi_hit_type1_ready ready Output 1
add_interface_port hit_type1 asi_hit_type1_error error Input 1

add_interface hit_type2 avalon_streaming start
set_interface_property hit_type2 associatedClock clock_interface
set_interface_property hit_type2 associatedReset reset_interface
set_interface_property hit_type2 dataBitsPerSymbol $HIT_TYPE2_WIDTH_CONST
set_interface_property hit_type2 errorDescriptor {"tsglitcherr"}
set_interface_property hit_type2 firstSymbolInHighOrderBits true
set_interface_property hit_type2 maxChannel 15
set_interface_property hit_type2 readyLatency 0
set_interface_property hit_type2 ENABLED true
add_interface_port hit_type2 aso_hit_type2_channel channel Output 4
add_interface_port hit_type2 aso_hit_type2_startofpacket startofpacket Output 1
add_interface_port hit_type2 aso_hit_type2_endofpacket endofpacket Output 1
add_interface_port hit_type2 aso_hit_type2_data data Output $HIT_TYPE2_WIDTH_CONST
add_interface_port hit_type2 aso_hit_type2_valid valid Output 1
add_interface_port hit_type2 aso_hit_type2_ready ready Input 1
add_interface_port hit_type2 aso_hit_type2_error error Output 1

add_interface clock_interface clock end
set_interface_property clock_interface clockRate 0
set_interface_property clock_interface ENABLED true
add_interface_port clock_interface i_clk clk Input 1

add_interface reset_interface reset end
set_interface_property reset_interface associatedClock clock_interface
set_interface_property reset_interface synchronousEdges DEASSERT
set_interface_property reset_interface ENABLED true
add_interface_port reset_interface i_rst reset Input 1

add_interface run_control avalon_streaming end
set_interface_property run_control associatedClock clock_interface
set_interface_property run_control associatedReset reset_interface
set_interface_property run_control dataBitsPerSymbol $RUN_CONTROL_WIDTH_CONST
set_interface_property run_control errorDescriptor ""
set_interface_property run_control firstSymbolInHighOrderBits true
set_interface_property run_control maxChannel 0
set_interface_property run_control readyLatency 0
set_interface_property run_control ENABLED true
add_interface_port run_control asi_ctrl_data data Input $RUN_CONTROL_WIDTH_CONST
add_interface_port run_control asi_ctrl_valid valid Input 1
add_interface_port run_control asi_ctrl_ready ready Output 1

add_interface filllevel avalon_streaming start
set_interface_property filllevel associatedClock clock_interface
set_interface_property filllevel associatedReset reset_interface
set_interface_property filllevel dataBitsPerSymbol $FILLLEVEL_WIDTH_CONST
set_interface_property filllevel errorDescriptor ""
set_interface_property filllevel firstSymbolInHighOrderBits true
set_interface_property filllevel maxChannel 0
set_interface_property filllevel readyLatency 0
set_interface_property filllevel ENABLED true
add_interface_port filllevel aso_filllevel_data data Output $FILLLEVEL_WIDTH_CONST
add_interface_port filllevel aso_filllevel_valid valid Output 1

add_interface debug_observability conduit start
set_interface_property debug_observability associatedClock clock_interface
set_interface_property debug_observability associatedReset reset_interface
set_interface_property debug_observability ENABLED true
add_interface_port debug_observability coe_debug_fill_level fill_level Output $DEBUG_WORD_WIDTH_CONST
add_interface_port debug_observability coe_debug_fifo_level fifo_level Output $DEBUG_WORD_WIDTH_CONST
add_interface_port debug_observability coe_debug_queue_state queue_state Output $DEBUG_WORD_WIDTH_CONST

add_interface hit_type1_metadata conduit end
set_interface_property hit_type1_metadata associatedClock clock_interface
set_interface_property hit_type1_metadata associatedReset reset_interface
set_interface_property hit_type1_metadata ENABLED false
add_interface_port hit_type1_metadata asi_hit_type1_metadata metadata Input $SIDECAR_METADATA_WIDTH_CONST
add_interface_port hit_type1_metadata asi_hit_type1_metadata_valid valid    Input 1

add_interface hit_type2_metadata conduit start
set_interface_property hit_type2_metadata associatedClock clock_interface
set_interface_property hit_type2_metadata associatedReset reset_interface
set_interface_property hit_type2_metadata ENABLED false
add_interface_port hit_type2_metadata aso_hit_type2_metadata metadata Output $SIDECAR_METADATA_WIDTH_CONST
add_interface_port hit_type2_metadata aso_hit_type2_metadata_valid valid    Output 1

# Presets are provided via the sibling .qprs file:
#   ring_buffer_cam_presets.qprs
