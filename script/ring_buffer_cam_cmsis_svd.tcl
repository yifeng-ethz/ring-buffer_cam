package require Tcl 8.5

set script_dir [file dirname [info script]]
set helper_file [file normalize [file join $script_dir .. dashboard_infra cmsis_svd lib mu3e_cmsis_svd.tcl]]
source $helper_file

namespace eval ::mu3e::cmsis::spec {}

proc ::mu3e::cmsis::spec::build_device {} {
    set registers [list \
        [::mu3e::cmsis::svd::register UID 0x00 \
            -description {Software-visible IP identifier. Default ASCII "RBCM".} \
            -access read-only \
            -resetValue 0x5242434D \
            -fields [list \
                [::mu3e::cmsis::svd::field value 0 32 \
                    -description {Compile-time or integration-time UID word. Default ASCII "RBCM".} \
                    -access read-only]]] \
        [::mu3e::cmsis::svd::register META 0x04 \
            -description {Read-multiplexed metadata word. Write 0=VERSION, 1=DATE, 2=GIT, 3=INSTANCE_ID and read back the selected page.} \
            -access read-write \
            -fields [list \
                [::mu3e::cmsis::svd::field page_sel 0 2 -description "Selects the metadata page." -access read-write] \
                [::mu3e::cmsis::svd::field reserved 2 30 -description "Reserved, read as zero." -access read-only]]] \
        [::mu3e::cmsis::svd::register CTRL 0x08 \
            -description "Go, soft-reset, and filter control word." \
            -access read-write \
            -fields [list \
                [::mu3e::cmsis::svd::field go 0 1 -description "Enable timestamp ordering and pop logic." -access read-write] \
                [::mu3e::cmsis::svd::field soft_reset 1 1 -description "Requests a local soft reset pulse." -access w1s] \
                [::mu3e::cmsis::svd::field reserved0 2 2 -description "Reserved, read as zero." -access read-only] \
                [::mu3e::cmsis::svd::field filter_inerr 4 1 -description "Filters ingress hits that carry timestamp error." -access read-write] \
                [::mu3e::cmsis::svd::field reserved1 5 27 -description "Reserved, read as zero." -access read-only]]] \
        [::mu3e::cmsis::svd::register EXPECTED_LATENCY 0x0C \
            -description "Read-pointer delay target in cycles." \
            -access read-write \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Expected latency target." -access read-write]]] \
        [::mu3e::cmsis::svd::register FILL_LEVEL 0x10 \
            -description "Live fill-level estimate derived from push, pop, and overwrite counters." \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Ring-buffer fill-level estimate." -access read-only]]] \
        [::mu3e::cmsis::svd::register INERR_COUNT 0x14 \
            -description "Count of filtered ingress timestamp-error hits." \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Filtered ingress-error hit count." -access read-only]]] \
        [::mu3e::cmsis::svd::register PUSH_COUNT 0x18 \
            -description "Total accepted push operations." \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Accepted push count." -access read-only]]] \
        [::mu3e::cmsis::svd::register POP_COUNT 0x1C \
            -description "Total drained hits." \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Drained hit count." -access read-only]]] \
        [::mu3e::cmsis::svd::register OVERWRITE_COUNT 0x20 \
            -description "Total overwrite events." \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Overwrite-event count." -access read-only]]] \
        [::mu3e::cmsis::svd::register CACHE_MISS_COUNT 0x24 \
            -description "Total cache-miss or empty-search events." \
            -access read-only \
            -fields [list [::mu3e::cmsis::svd::field value 0 32 -description "Cache-miss or empty-search count." -access read-only]]]]

    return [::mu3e::cmsis::svd::device MU3E_RING_BUFFER_CAM \
        -version 26.1.12.0419 \
        -description "CMSIS-SVD description of the ring_buffer_cam CSR window." \
        -peripherals [list \
            [::mu3e::cmsis::svd::peripheral RING_BUFFER_CAM_CSR 0x0 \
                -description "Relative CSR aperture for the ring-buffer CAM." \
                -groupName MU3E_DATA_PATH \
                -addressBlockSize 0x80 \
                -registers $registers]]]
}

if {[info exists ::argv0] &&
    [file normalize $::argv0] eq [file normalize [info script]]} {
    set out_path [file join $script_dir ring_buffer_cam.svd]
    if {[llength $::argv] >= 1} {
        set out_path [lindex $::argv 0]
    }
    ::mu3e::cmsis::svd::write_device_file \
        [::mu3e::cmsis::spec::build_device] $out_path
}
