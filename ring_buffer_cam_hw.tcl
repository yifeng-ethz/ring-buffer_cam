package require -exact qsys 16.1

set_module_property NAME ring_buffer_cam
set_module_property DISPLAY_NAME "Ring-buffer CAM"
set_module_property VERSION 26.1.4.0402
set_module_property DESCRIPTION "Ring-buffer shaped content-addressable memory for timestamp ordering. This delivered release keeps the current Qsys interface contract, adds the common CSR identity header, and uses the active partitioned V2 core."
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
set CSR_LAST_WORD_CONST         9
set HIT_TYPE1_WIDTH_CONST       39
set HIT_TYPE2_WIDTH_CONST       36
set RUN_CONTROL_WIDTH_CONST     9
set FILLLEVEL_WIDTH_CONST       16
set DEFAULT_RING_DEPTH_CONST    512
set DEFAULT_PIPE_STAGES_CONST   4
set IP_UID_DEFAULT_CONST        1380074317
set BUILD_DEFAULT_CONST         402
set VERSION_MAJOR_DEFAULT_CONST 26
set VERSION_MINOR_DEFAULT_CONST 1
set VERSION_PATCH_DEFAULT_CONST 4
set VERSION_DATE_DEFAULT_CONST  20260402
set VERSION_GIT_DEFAULT_CONST   0
set INSTANCE_ID_DEFAULT_CONST   0

set CSR_TABLE_HTML {<html><table border="1" width="100%">
<tr><th>Word</th><th>Byte</th><th>Name</th><th>Access</th><th>Description</th></tr>
<tr><td>0x00</td><td>0x000</td><td>UID</td><td>RO</td><td>Software-visible IP identifier. Default is ASCII <b>RBCM</b> but it is integration-time overridable.</td></tr>
<tr><td>0x01</td><td>0x004</td><td>META</td><td>RW/RO</td><td>Read-multiplexed metadata word. Write <b>0</b>=VERSION, <b>1</b>=DATE, <b>2</b>=GIT, <b>3</b>=INSTANCE_ID. VERSION is packed as MAJOR[31:24], MINOR[23:16], PATCH[15:12], BUILD[11:0].</td></tr>
<tr><td>0x02</td><td>0x008</td><td>CTRL</td><td>RW</td><td>Bit 0 <b>go</b>, bit 1 <b>soft_reset</b>, bit 4 <b>filter_inerr</b>.</td></tr>
<tr><td>0x03</td><td>0x00C</td><td>EXPECTED_LATENCY</td><td>RW</td><td>Read-pointer delay target in cycles. Reset default is 2000.</td></tr>
<tr><td>0x04</td><td>0x010</td><td>FILL_LEVEL</td><td>RO</td><td>Live fill-level estimate derived from push, pop, and overwrite counters.</td></tr>
<tr><td>0x05</td><td>0x014</td><td>INERR_COUNT</td><td>RO</td><td>Count of filtered ingress timestamp-error hits.</td></tr>
<tr><td>0x06</td><td>0x018</td><td>PUSH_COUNT</td><td>RO</td><td>Total accepted push operations.</td></tr>
<tr><td>0x07</td><td>0x01C</td><td>POP_COUNT</td><td>RO</td><td>Total drained hits.</td></tr>
<tr><td>0x08</td><td>0x020</td><td>OVERWRITE_COUNT</td><td>RO</td><td>Total overwrite events.</td></tr>
<tr><td>0x09</td><td>0x024</td><td>CACHE_MISS_COUNT</td><td>RO</td><td>Total cache-miss / empty-search events.</td></tr>
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
}

proc elaborate {} {
    compute_derived_values

    set interleaving_factor [get_parameter_value INTERLEAVING_FACTOR]
    set interleaving_index_max [expr {$interleaving_factor - 1}]

    set_parameter_property RING_BUFFER_N_ENTRY ALLOWED_RANGES {32 64 128 256 512 1024 2048}
    set_parameter_property INTERLEAVING_FACTOR ALLOWED_RANGES {1 2 4 8 16 32}
    set_parameter_property INTERLEAVING_INDEX ALLOWED_RANGES 0:$interleaving_index_max
    set_parameter_property N_PARTITIONS ALLOWED_RANGES {1 2 4 8}
    set_parameter_property ENCODER_LEAF_WIDTH ALLOWED_RANGES {4 8 16 32 64}

}

add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL ring_buffer_cam
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file ring_buffer_cam.vhd VHDL PATH ring_buffer_cam.vhd TOP_LEVEL_FILE
add_fileset_file ring_buffer_cam_v2_core.vhd VHDL PATH ring_buffer_cam_v2_core.vhd
add_fileset_file cam_helper_pkg.vhd VHDL PATH cam_helper_pkg.vhd
add_fileset_file alt_simple_dpram.vhd VHDL PATH alt_simple_dpram.vhd
add_fileset_file cam_mem_a5.vhd VHDL PATH cam_mem_a5.vhd
add_fileset_file cam_mem_blk_a5.vhd VHDL PATH cam_mem_blk_a5.vhd
add_fileset_file alt_fifo/scfifo_w40d256.vhd VHDL PATH alt_fifo/scfifo_w40d256.vhd
add_fileset_file alt_fifo/cmd_fifo/cmd_fifo.vhd VHDL PATH alt_fifo/cmd_fifo/cmd_fifo.vhd
add_fileset_file b2o_encoder.v VERILOG PATH b2o_encoder.v
add_fileset_file addr_enc_logic_small.vhd VHDL PATH addr_enc_logic_small.vhd
add_fileset_file addr_enc_logic_partitioned.vhd VHDL PATH addr_enc_logic_partitioned.vhd

add_parameter SEARCH_KEY_WIDTH NATURAL 8
set_parameter_property SEARCH_KEY_WIDTH DISPLAY_NAME "Search Key Width"
set_parameter_property SEARCH_KEY_WIDTH UNITS Bits
set_parameter_property SEARCH_KEY_WIDTH HDL_PARAMETER true
set_parameter_property SEARCH_KEY_WIDTH DESCRIPTION {Bit width of the CAM search key. The default 8-bit key corresponds to timestamp[11:4].}

add_parameter RING_BUFFER_N_ENTRY NATURAL $DEFAULT_RING_DEPTH_CONST
set_parameter_property RING_BUFFER_N_ENTRY DISPLAY_NAME "Ring Depth"
set_parameter_property RING_BUFFER_N_ENTRY UNITS None
set_parameter_property RING_BUFFER_N_ENTRY HDL_PARAMETER true
set_parameter_property RING_BUFFER_N_ENTRY DESCRIPTION "Total ring-buffer / CAM depth. Use powers of two. The delivered default is 512 for simulation and standalone signoff."

add_parameter SIDE_DATA_BITS NATURAL 31
set_parameter_property SIDE_DATA_BITS DISPLAY_NAME "Side Data Width"
set_parameter_property SIDE_DATA_BITS UNITS Bits
set_parameter_property SIDE_DATA_BITS HDL_PARAMETER true
set_parameter_property SIDE_DATA_BITS DESCRIPTION "Side-band payload width stored alongside the search key."

add_parameter INTERLEAVING_FACTOR NATURAL 4
set_parameter_property INTERLEAVING_FACTOR DISPLAY_NAME "Interleaving Factor"
set_parameter_property INTERLEAVING_FACTOR UNITS None
set_parameter_property INTERLEAVING_FACTOR HDL_PARAMETER true
set_parameter_property INTERLEAVING_FACTOR DESCRIPTION "Power-of-two time-interleaving fanout shared across the surrounding sorter complex."

add_parameter INTERLEAVING_INDEX NATURAL 0
set_parameter_property INTERLEAVING_INDEX DISPLAY_NAME "Interleaving Index"
set_parameter_property INTERLEAVING_INDEX UNITS None
set_parameter_property INTERLEAVING_INDEX HDL_PARAMETER true
set_parameter_property INTERLEAVING_INDEX DESCRIPTION "Modulo index accepted by this IP instance within the larger interleaving complex."

add_parameter N_PARTITIONS NATURAL 4
set_parameter_property N_PARTITIONS DISPLAY_NAME "Physical Match Partitions"
set_parameter_property N_PARTITIONS UNITS None
set_parameter_property N_PARTITIONS HDL_PARAMETER true
set_parameter_property N_PARTITIONS DESCRIPTION "Physical one-hot match partitions. The delivered default uses four partitions to match the four-stage encoder pipeline. Lower-partition presets remain available for timing/resource comparisons and regression."

add_parameter ENCODER_LEAF_WIDTH NATURAL 16
set_parameter_property ENCODER_LEAF_WIDTH DISPLAY_NAME "Encoder Leaf Width"
set_parameter_property ENCODER_LEAF_WIDTH UNITS None
set_parameter_property ENCODER_LEAF_WIDTH HDL_PARAMETER true
set_parameter_property ENCODER_LEAF_WIDTH DESCRIPTION "Leaf slice width used by the staged one-hot encoder."

add_parameter ENCODER_PIPE_STAGES NATURAL $DEFAULT_PIPE_STAGES_CONST
set_parameter_property ENCODER_PIPE_STAGES DISPLAY_NAME "Encoder Pipe Stages"
set_parameter_property ENCODER_PIPE_STAGES UNITS None
set_parameter_property ENCODER_PIPE_STAGES HDL_PARAMETER true
set_parameter_property ENCODER_PIPE_STAGES DESCRIPTION "Internal pipeline depth of the staged encoder. The delivered default is P4. Presets keep P2 and P3 available."

add_parameter DEBUG NATURAL 1
set_parameter_property DEBUG DISPLAY_NAME "Debug Level"
set_parameter_property DEBUG UNITS None
set_parameter_property DEBUG ALLOWED_RANGES 0:2
set_parameter_property DEBUG HDL_PARAMETER true
set_parameter_property DEBUG DESCRIPTION "Debug level exported by the RTL. 0 disables optional debug behavior, 1 keeps synthesizable debug, 2 enables simulation-only debug."

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
add_display_item $TAB_CONFIGURATION "Match Engine" GROUP
add_display_item $TAB_CONFIGURATION "Throughput" GROUP
add_display_item $TAB_CONFIGURATION "Advanced" GROUP

add_html_text "Overview" overview_html "<html><b>Function</b><br/>This IP stores timestamp-tagged hits in a ring-buffer shaped CAM and drains them in timestamp order. The delivered release keeps the established Avalon-MM and Avalon-ST system contract, uses the active partitioned V2 core, and adds the common CSR identity header at words <b>0</b> and <b>1</b>.<br/><br/><b>Clocking</b><br/>All interfaces run inside a single synchronous domain shared by CSR, ingress, pop control, ordered egress, and fill-level monitoring.</html>"
add_display_item "Sizing" SEARCH_KEY_WIDTH parameter
add_display_item "Sizing" RING_BUFFER_N_ENTRY parameter
add_display_item "Sizing" SIDE_DATA_BITS parameter
add_html_text "Sizing" sizing_html "<html><b>Derived sizing</b><br/>Updated by the validation callback.</html>"
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

add_html_text "Delivered Profile" profile_html "<html><b>Catalog revision</b><br/>This release is packaged as <b>26.1.4.0402</b>. It keeps the established <b>hit_type1</b>, <b>hit_type2</b>, <b>run_control</b>, and <b>filllevel</b> interface names so existing Platform Designer systems can be upgraded in place while picking up the common CSR identity header and the active partitioned V2 CAM core.<br/><br/><b>Runtime visibility</b><br/>Software can blind-scan the CSR window through <b>UID</b> at word <b>0</b> and the <b>META</b> mux at word <b>1</b>.</html>"
add_html_text "Versioning" versioning_html {<html><b>Common identity header</b><br/>Word <b>0</b> is <b>UID</b>.<br/>Word <b>1</b> is <b>META</b>: write 0=VERSION, 1=DATE, 2=GIT, 3=INSTANCE_ID.<br/><br/><b>VERSION encoding</b><br/>VERSION[31:24] = MAJOR, VERSION[23:16] = MINOR, VERSION[15:12] = PATCH, VERSION[11:0] = BUILD.</html>}
add_display_item "Versioning" IP_UID parameter
add_display_item "Versioning" VERSION_MAJOR parameter
add_display_item "Versioning" VERSION_MINOR parameter
add_display_item "Versioning" VERSION_PATCH parameter
add_display_item "Versioning" BUILD parameter
add_display_item "Versioning" VERSION_DATE parameter
add_display_item "Versioning" VERSION_GIT parameter
add_display_item "Versioning" INSTANCE_ID parameter
add_html_text "Debug" debug_html "<html><b>Debug control</b><br/>The RTL exports the existing synthesizable and simulation-oriented debug behavior through the single debug-level parameter below.</html>"
add_display_item "Debug" DEBUG parameter

add_display_item "" $TAB_INTERFACES GROUP tab
add_display_item $TAB_INTERFACES "Clock / Reset" GROUP
add_display_item $TAB_INTERFACES "Data Path" GROUP
add_display_item $TAB_INTERFACES "Control Path" GROUP
add_display_item $TAB_INTERFACES "Monitoring" GROUP

add_html_text "Clock / Reset" clock_html "<html><b>clock_interface</b> and <b>reset_interface</b><br/>Single synchronous clock/reset domain for the full CAM datapath and CSR logic.</html>"
add_html_text "Data Path" datapath_html "<html><b>hit_type1</b><br/>39-bit Avalon-ST sink carrying the Type-1 hit payload plus a timestamp-error sideband.<br/><br/><b>hit_type2</b><br/>36-bit Avalon-ST source that emits ordered Type-2 hits after CAM lookup and pop arbitration.</html>"
add_html_text "Control Path" control_html "<html><b>csr</b> and <b>run_control</b><br/>Word-addressed Avalon-MM CSR window plus a 9-bit Avalon-ST run-control sink. The run-control path does not alter the external latency contract when encoder pipeline presets are changed.</html>"
add_html_text "Monitoring" monitor_html "<html><b>filllevel</b><br/>16-bit Avalon-ST source for live fill-level reporting and integration-time occupancy monitoring.</html>"

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

# Presets are provided via the sibling .qprs file:
#   ring_buffer_cam_presets.qprs
