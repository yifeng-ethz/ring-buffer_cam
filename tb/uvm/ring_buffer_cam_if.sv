// ring_buffer_cam_if.sv — SystemVerilog interfaces for the DUT ports

`ifndef RING_BUFFER_CAM_IF_SV
`define RING_BUFFER_CAM_IF_SV

// ── Avalon-ST hit ingress ────────────────────────────────────────
interface avst_hit_if (input logic clk, input logic rst);
  logic [3:0]  channel;
  logic        startofpacket;
  logic        endofpacket;
  logic [38:0] data;
  logic        valid;
  logic        ready;
  logic [0:0]  error;

  modport drv (
    output data, valid, channel, startofpacket, endofpacket, error,
    input  ready, clk, rst
  );
  modport mon (
    input data, valid, ready, channel, startofpacket, endofpacket, error, clk, rst
  );
endinterface

// ── Avalon-ST hit egress ─────────────────────────────────────────
interface avst_out_if (input logic clk, input logic rst);
  logic [3:0]  channel;
  logic        startofpacket;
  logic        endofpacket;
  logic [35:0] data;
  logic        valid;
  logic        ready;
  logic [0:0]  error;

  logic        filllevel_valid;
  logic [15:0] filllevel_data;

  modport drv (
    output ready,
    input  data, valid, channel, startofpacket, endofpacket, error,
           filllevel_valid, filllevel_data, clk, rst
  );
  modport mon (
    input data, valid, ready, channel, startofpacket, endofpacket, error,
          filllevel_valid, filllevel_data, clk, rst
  );
endinterface

// ── Avalon-ST run control ────────────────────────────────────────
interface avst_ctrl_if (input logic clk, input logic rst);
  logic [8:0] data;
  logic       valid;
  logic       ready;

  modport drv (
    output data, valid,
    input  ready, clk, rst
  );
  modport mon (
    input data, valid, ready, clk, rst
  );
endinterface

// ── Avalon-MM CSR ────────────────────────────────────────────────
interface avmm_csr_if (input logic clk, input logic rst);
  logic [4:0]  address;
  logic        read;
  logic [31:0] readdata;
  logic        write;
  logic [31:0] writedata;
  logic        waitrequest;

  modport drv (
    output address, read, write, writedata,
    input  readdata, waitrequest, clk, rst
  );
  modport mon (
    input address, read, readdata, write, writedata, waitrequest, clk, rst
  );
endinterface

`endif
