package require Tcl 8.5

set script_dir [file dirname [info script]]
set helper_file [file normalize [file join $script_dir .. dashboard_infra system_console lib board_bring_up_meta.tcl]]
if {![llength [info commands ::board_bring_up::meta::field]]} {
        source $helper_file
}

namespace eval ::board_bring_up::meta::ring_buffer_cam {
}

proc ::board_bring_up::meta::ring_buffer_cam::get_contract {} {
        set registers [list \
                [::board_bring_up::meta::register \
                        "uid" \
                        "Software-visible IP identifier. Default payload is ASCII RBCM." \
                        "0x0" \
                        [list [::board_bring_up::meta::field "uid" "Compile-time / integration-time overridable IP identifier." {[31:0]} "read-only"]]] \
                [::board_bring_up::meta::register \
                        "meta" \
                        "Read-multiplexed metadata word. Write 0=VERSION, 1=DATE, 2=GIT, 3=INSTANCE_ID before reading back." \
                        "0x4" \
                        [list \
                                [::board_bring_up::meta::field "version" {MAJOR[31:24], MINOR[23:16], PATCH[15:12], BUILD[11:0] when selector=0.} {[31:0]} "read-write"] \
                                [::board_bring_up::meta::field "date" {YYYYMMDD provenance word when selector=1.} {[31:0]} "read-write"] \
                                [::board_bring_up::meta::field "git" {Truncated build git hash when selector=2.} {[31:0]} "read-write"] \
                                [::board_bring_up::meta::field "instance_id" {Integrator-defined instance identifier when selector=3.} {[31:0]} "read-write"]]] \
                [::board_bring_up::meta::register \
                        "control_and_status" \
                        "Control and status registers of Ring-Buffer CAM IP" \
                        "0x8" \
                        [list \
                                [::board_bring_up::meta::field "go" "Allow to accept hits" {[0:0]} "read-write"] \
                                [::board_bring_up::meta::field "soft_reset" "running reset request for the ring-buffer CAM" {[1:1]} "read-write"] \
                                [::board_bring_up::meta::field "filter_inerr" "Filter the ingress error hits with 'tserr'" {[4:4]} "read-write"]]] \
                [::board_bring_up::meta::register \
                        "Latency" \
                        "Read clock latency for pop command generator respect to the Mu3e global clock." \
                        "0xc" \
                        [list [::board_bring_up::meta::field "expected_latency" "Expected latency value to set in unit of cycles (8 ns)." {[15:0]} "read-write"]]] \
                [::board_bring_up::meta::register \
                        "Fill_level_meter" \
                        "Fill-level of the ring-buffer CAM." \
                        "0x10" \
                        [list [::board_bring_up::meta::field "fill_level" "Fill-level counter of current hits inside." {[31:0]} "read-only"]]] \
                [::board_bring_up::meta::register \
                        "In-error count" \
                        "Number of hits with 'tserr' discarded at the ingress." \
                        "0x14" \
                        [list [::board_bring_up::meta::field "inerr_count" "Counter of error='tserr' hits" {[31:0]} "read-only"]]] \
                [::board_bring_up::meta::register \
                        "push count" \
                        "Number of hits pushed into ring-buffer CAM." \
                        "0x18" \
                        [list [::board_bring_up::meta::field "push_count" "Counter of push-in hits" {[31:0]} "read-only"]]] \
                [::board_bring_up::meta::register \
                        "pop count" \
                        "Number of hits popped from ring-buffer CAM." \
                        "0x1c" \
                        [list [::board_bring_up::meta::field "pop_count" "Counter of pop-out hits" {[31:0]} "read-only"]]] \
                [::board_bring_up::meta::register \
                        "overwrite count" \
                        "Number of hits overwritten by the push-in engine." \
                        "0x20" \
                        [list [::board_bring_up::meta::field "overwrite_count" "Counter of overwritten hits" {[31:0]} "read-only"]]] \
                [::board_bring_up::meta::register \
                        "cache-miss count" \
                        "Number of hits missed by push-out engine." \
                        "0x24" \
                        [list [::board_bring_up::meta::field "cache_miss_count" "Counter of cache-missed hits" {[31:0]} "read-only"]]]]

        return [::board_bring_up::meta::contract $registers]
}
