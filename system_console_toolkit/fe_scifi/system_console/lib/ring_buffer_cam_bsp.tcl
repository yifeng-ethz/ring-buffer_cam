###########################################################################################################
# @Name 	    ring_buffer_cam_bsp.tcl
#
# @Brief		Board-Support-Package (BSP) of the Ring-Buffer CAM IP core. 
#				Contains the address map info.
#
# @Functions	.
#
# @Author		Yifeng Wang (yifenwan@phys.ethz.ch)
# @Date			Mar 24, 2025
# @Version		1.0 (file created)
#				
#
###########################################################################################################
package require Tcl 			8.5
############## match the version of the IP #######################
package provide ring_buffer_cam::bsp 	24.0

namespace eval ::ring_buffer_cam::bsp:: {
	namespace export \
    get_address_map \
    configure
}

proc ::ring_buffer_cam::bsp::get_address_map {} {
    array set csr_map {}
    ################################################ csr ################################################
    set csr_map(0)  \
    "
    <register>
        <name>control_and_status</name>
        <description>Control and status registers of Ring-Buffer CAM IP</description>
        <addressOffset>0x0</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>go</name>
                <description>Allow to accept hits</description>
                <bitRange>\[0:0\]</bitRange>
                <access>read-write</access>
            </field>
            <field>
              <name>soft_reset</name>
              <description>TODO: implement this as flushing the hits and reset the register values, without touching the read timer, good for a running reset</description>
              <bitRange>\[1:1\]</bitRange>
              <access>read-write</access>
            </field>
            <field>
              <name>filter_inerr</name>
              <description>Filter the ingress error hits with 'tserr' </description>
              <bitRange>\[4:4\]</bitRange>
              <access>read-write</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ latency #######################################
    set csr_map(1)  \
    "
    <register>
        <name>Latency</name>
        <description>Read clock latency for pop command generator respect to the Mu3e global clock. It has to be larger than the latest hits. You are advised to measure the delay PDF with histogram statistics IP. </description>
        <addressOffset>0x4</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>expected_latency</name>
                <description>Expected latency value to set (can be tune at run time) in unit of cycles (8 ns). ex: read clock = gts - latency</description>
                <bitRange>\[15:0\]</bitRange>
                <access>read-write</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ fill level #######################################
    set csr_map(2)  \
    "
    <register>
        <name>Fill_level_meter</name>
        <description>Fill-level of the ring-buffer CAM. fill-level = push - pop - overwrite + cache-miss. Note: if cache-miss is non-zero, it means memory consistency is broken.</description>
        <addressOffset>0x8</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>fill_level</name>
                <description>Fill-level counter of current hits inside.  </description>
                <bitRange>\[31:0\]</bitRange>
                <access>read-only</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ in-error count #######################################
    set csr_map(3)  \
    "
    <register>
        <name>In-error count</name>
        <description>Number of hits with 'tserr' has been discarded at the ingress. Does not count toward other counters. 'tserr' is the hits out of valid delay regime. You can test this functionality by turning on 'disable leap correction' in MuTRiG Timestamp Processor IP.</description>
        <addressOffset>0xc</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>inerr_count</name>
                <description>Counter of error='tserr' hits</description>
                <bitRange>\[31:0\]</bitRange>
                <access>read-only</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ push count #######################################
    set csr_map(4)  \
    "
    <register>
        <name>push count</name>
        <description>(DEBUG) Number of hits that is pushed into ring-buffer CAM, excluding rejected hits by error and timestamp interleaving.</description>
        <addressOffset>0x10</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>push_count</name>
                <description>Counter of push-in hits</description>
                <bitRange>\[31:0\]</bitRange>
                <access>read-only</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ pop count #######################################
    set csr_map(5)  \
    "
    <register>
        <name>pop count</name>
        <description>(DEBUG) Number of hits that is popped from ring-buffer CAM. This should reflect the real output hits, including 'tsglitcherr' hits (should be rejected later by downstream).</description>
        <addressOffset>0x14</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>pop_count</name>
                <description>Counter of pop-out hits</description>
                <bitRange>\[31:0\]</bitRange>
                <access>read-only</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ overwrite count #######################################
    set csr_map(6)  \
    "
    <register>
        <name>overwrite count</name>
        <description>(DEBUG) Number of hits that is overwritten by push-in engine. Presumably collected by the push-in engine. </description>
        <addressOffset>0x18</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>overwrite_count</name>
                <description>Counter of overwritten hits</description>
                <bitRange>\[31:0\]</bitRange>
                <access>read-only</access>
            </field>
        </fields>
    </register>
    "
    
    ################################################ cache miss count #######################################
    set csr_map(7)  \
    "
    <register>
        <name>cache-miss count</name>
        <description>(DEBUG) Number of hits that is missed by push-out engine. When lookup found such hit, but in RAM it does not exist. So, it will create underflow and we need to add it to total counter. </description>
        <addressOffset>0x1c</addressOffset>
        <size>32</size>
        <fields>
            <field>
                <name>cache_miss_count</name>
                <description>Counter of cache-missed hits</description>
                <bitRange>\[31:0\]</bitRange>
                <access>read-only</access>
            </field>
        </fields>
    </register>
    "
    
    return [array get csr_map]
}
