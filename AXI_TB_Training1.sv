import uvm_pkg::*;
`include "uvm_macros.svh"

typedef enum logic [1:0] {OKAY=2'b00, EXOKAY=1, SLVERR='d2, DECERR='h3} resp_e;

interface axi_interface(input logic aclk, logic aresetn); //Active Low Reset
  //Read Address/Request Channel Signals
  logic [7:0]  ARADDR;
  logic [2:0]  ARID;
  logic        ARVALID;
  logic        ARREADY;
  
  //Read Data/Response Channel Signals
  logic        RVALID;
  logic        RREADY;
  logic [2:0]  RID;
  logic [1:0]  RRESP;
  logic [31:0] RDATA;
  
  //Write Address/Request Channel Signals
  logic [7:0]  AWADDR;
  logic [2:0]  AWID;
  logic        AWVALID;
  logic        AWREADY;
  
  //Write Data Channel Signals
  logic        WREADY;
  logic        WVALID;
  logic [2:0]  WID;
  logic [31:0] WDATA;
  logic [3:0]  WSTRB; //Strobe Signals - Byte Enable Signals
  
  //Write Response Channel Signals
  logic        BVALID;
  logic        BREADY;
  logic [2:0]  BID;
  logic [1:0]  BRESP;
  
  clocking mst_rd_req_cb @(posedge aclk);
    input  ARREADY;
    output ARADDR;
    output ARID;
    output ARVALID;
  endclocking
  
  clocking mst_rd_rsp_cb @(posedge aclk);
    input  RVALID;
    input  RID;
    input  RRESP;
    input  RDATA;
    output RREADY;
  endclocking
  
  clocking mst_wr_req_cb @(posedge aclk);
    input  AWREADY;
    output AWVALID;
    output AWADDR;
    output AWID;
  endclocking
  
  clocking mst_wr_dat_cb @(posedge aclk);
    input  WREADY;
    output WVALID;
    output WID;
    output WDATA;
    output WSTRB;
  endclocking
  
  clocking mst_wr_rsp_cb @(posedge aclk);
    input  BVALID;
    input  BID;
    input  BRESP;
    output BREADY;
  endclocking
  
  clocking mst_mon_cb @(posedge aclk);
    input ARADDR, ARID, ARVALID, ARREADY;
    input RVALID, RREADY, RID, RRESP, RDATA;
    input AWADDR, AWID, AWVALID, AWREADY;
    input WDATA, WID, WSTRB, WVALID, WREADY;
    input BID, BRESP, BVALID, BREADY;
  endclocking
  
  clocking slv_rd_req_cb @(posedge aclk);
    output ARREADY;
    input  ARADDR;
    input  ARID;
    input  ARVALID;
  endclocking
  
  clocking slv_rd_rsp_cb @(posedge aclk);
    output RVALID;
    output RID;
    output RRESP;
    output RDATA;
    input  RREADY;
  endclocking
  
  clocking slv_wr_req_cb @(posedge aclk);
    output AWREADY;
    input  AWVALID;
    input  AWADDR;
    input  AWID;
  endclocking
  
  clocking slv_wr_dat_cb @(posedge aclk);
    output WREADY;
    input  WVALID;
    input  WID;
    input  WDATA;
    input  WSTRB;
  endclocking
  
  clocking slv_wr_rsp_cb @(posedge aclk);
    output BVALID;
    output BID;
    output BRESP;
    input  BREADY;
  endclocking
  
  clocking slv_mon_cb @(posedge aclk);
    input ARADDR, ARID, ARVALID, ARREADY;
    input RVALID, RREADY, RID, RRESP, RDATA;
    input AWADDR, AWID, AWVALID, AWREADY;
    input WDATA, WID, WSTRB, WVALID, WREADY;
    input BID, BRESP, BVALID, BREADY;
  endclocking
endinterface

class axi_transaction extends uvm_sequence_item;
  rand logic [7:0]  ADDR;
  rand logic [2:0]  ID;
       resp_e       RESP; //Not Random because Master Accepts it
  rand logic [31:0] DATA;
  rand logic [3:0]  WSTRB;
  
  `uvm_object_utils_begin(axi_transaction)
  `uvm_field_int(ADDR, UVM_DEFAULT)
  `uvm_field_int(ID, UVM_DEFAULT)
  `uvm_field_enum(resp_e, RESP, UVM_DEFAULT)
  `uvm_field_int(DATA, UVM_DEFAULT)
  `uvm_field_int(WSTRB, UVM_DEFAULT)
  `uvm_object_utils_end
  
  function new(string name = "axi_transaction");
    super.new(name);
  endfunction : new
endclass : axi_transaction
