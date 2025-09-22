import uvm_pkg::*;
`include "uvm_macros.svh"

typedef enum logic [1:0] {OKAY=2'b00, EXOKAY=1, SLVERR='d2, DECERR='h3} resp_e;
typedef enum logic       {READ, WRITE} cmd_e;

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
  rand cmd_e        CMD_TYPE;
  rand logic [7:0]  ADDR;
  rand logic [2:0]  ID;
       resp_e       RESP; //Not Random because Master Accepts it
  rand logic [31:0] DATA;
  rand logic [3:0]  WSTRB;
  
  `uvm_object_utils_begin(axi_transaction)
  `uvm_field_enum(cmd_e, CMD_TYPE, UVM_DEFAULT)
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

class axi_sequencer extends uvm_sequencer #(axi_transaction);
  `uvm_component_utils(axi_sequencer)
  
  function new(string name = "axi_sequencer", uvm_component parent);
    super.new(name, parent);
  endfunction : new
endclass : axi_sequencer

class axi_driver extends uvm_driver #(axi_transaction);
  axi_transaction       axi_rd_req_q[$];
  axi_transaction       axi_wr_req_q[$];
  virtual axi_interface axi_intf;
  
  `uvm_component_utils(axi_driver)
  
  function new(string name = "axi_driver", uvm_component parent);
    super.new(name,parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    uvm_config_db #(virtual axi_interface)::get(this, "", "AxiInterface", axi_intf);
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    fork
      get_trans_from_seqr();
      drive_read_req_channel();
      respond_read_rsp_channel();
      drive_write_req_channel();
      respond_write_rsp_channel();
    join
  endtask : run_phase
  
  task get_trans_from_seqr();
    forever begin
      axi_transaction axi_trans;
      
      seq_item_port.get_next_item(axi_trans); //Step1 : Get the Item from Sequencer
      if(axi_trans.CMD_TYPE == READ) axi_rd_req_q.push_back(axi_trans);
      else                           axi_wr_req_q.push_back(axi_trans);
      seq_item_port.item_done(axi_trans); //Step3 : Completed sending to the interface
    end
  endtask : get_trans_from_seqr
  
  task drive_read_req_channel();
    forever begin
      axi_transaction rd_req;
      
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        axi_intf.mst_rd_req_cb.ARVALID <= 1'b0;
        @(axi_intf.mst_rd_req_cb);
      end
      
      if(axi_rd_req_q.size() == 0) begin //Queue is Non Empty
        @(axi_intf.mst_rd_req_cb);
        continue;
      end
      
      rd_req = axi_rd_req_q.pop_front();
      
      axi_intf.mst_rd_req_cb.ARVALID <= 1'b1;
      axi_intf.mst_rd_req_cb.ARADDR  <= rd_req.ADDR;
      axi_intf.mst_rd_req_cb.ARID    <= rd_req.ID;
      
      if(axi_intf.ARREADY === 1'b1) begin
        @(axi_intf.mst_rd_req_cb);
      end
      else begin
        while(axi_intf.ARREADY !== 1'b1) begin
          @(axi_intf.mst_rd_req_cb);
        end
      end
      
      axi_intf.mst_rd_req_cb.ARVALID <= 1'b0;
      @(axi_intf.mst_rd_req_cb);
    end
  endtask : drive_read_req_channel
  
  task respond_read_rsp_channel();
    forever begin
      logic rand_ready;
      
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        axi_intf.mst_rd_rsp_cb.RREADY <= 1'b0;
        @(axi_intf.mst_rd_rsp_cb);
      end
      
      std::randomize(rand_ready);
      axi_intf.mst_rd_rsp_cb.RREADY <= rand_ready;
      @(axi_intf.mst_rd_rsp_cb);
    end
  endtask : respond_read_rsp_channel
  
  task drive_write_req_channel();
    forever begin
      axi_transaction wr_req;
      logic           addr_before_data;
      
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        axi_intf.mst_wr_req_cb.AWVALID <= 1'b0;
        axi_intf.mst_wr_dat_cb.WVALID  <= 1'b0;
        @(axi_intf.mst_wr_req_cb);
      end
      
      if(axi_wr_req_q.size() == 0) begin //Queue is Non Empty
        @(axi_intf.mst_wr_req_cb);
        continue;
      end
      
      wr_req = axi_wr_req_q.pop_front();
      std::randomize(addr_before_data);
      
      if(addr_before_data == 1'b1) begin
        axi_intf.mst_wr_req_cb.AWVALID <= 1'b1;
        axi_intf.mst_wr_req_cb.AWADDR  <= wr_req.ADDR;
        axi_intf.mst_wr_req_cb.AWID    <= wr_req.ID;
      
        fork
          begin
            while(axi_intf.mst_wr_req_cb.AWREADY !== 1'b1) begin
              @(axi_intf.mst_wr_req_cb);
            end
            
            axi_intf.mst_wr_req_cb.AWVALID <= 1'b0;
            @(axi_intf.mst_wr_req_cb);
          end
        join_none
        
        @(axi_intf.mst_wr_req_cb);
        axi_intf.mst_wr_dat_cb.WVALID <= 1'b1;
        axi_intf.mst_wr_dat_cb.WDATA  <= wr_req.DATA;
        axi_intf.mst_wr_dat_cb.WID    <= wr_req.ID;
        axi_intf.mst_wr_dat_cb.WSTRB  <= wr_req.WSTRB;
      
        while(axi_intf.mst_wr_dat_cb.WREADY !== 1'b1) begin
          @(axi_intf.mst_wr_dat_cb);
        end
      
        axi_intf.mst_wr_dat_cb.WVALID <= 1'b0;
        @(axi_intf.mst_wr_dat_cb);       
      end
      else begin
        axi_intf.mst_wr_dat_cb.WVALID <= 1'b1;
        axi_intf.mst_wr_dat_cb.WDATA  <= wr_req.DATA;
        axi_intf.mst_wr_dat_cb.WID    <= wr_req.ID;
        axi_intf.mst_wr_dat_cb.WSTRB  <= wr_req.WSTRB;
      
        fork
          begin
            while(axi_intf.mst_wr_dat_cb.WREADY !== 1'b1) begin
              @(axi_intf.mst_wr_dat_cb);
            end
            
            axi_intf.mst_wr_dat_cb.WVALID <= 1'b0;
            @(axi_intf.mst_wr_dat_cb);
          end
        join_none

        @(axi_intf.mst_wr_dat_cb);
        axi_intf.mst_wr_req_cb.AWVALID <= 1'b1;
        axi_intf.mst_wr_req_cb.AWADDR  <= wr_req.ADDR;
        axi_intf.mst_wr_req_cb.AWID    <= wr_req.ID;
      
        while(axi_intf.mst_wr_req_cb.AWREADY !== 1'b1) begin
          @(axi_intf.mst_wr_req_cb);
        end
      
        axi_intf.mst_wr_req_cb.AWVALID <= 1'b0;
        @(axi_intf.mst_wr_req_cb);
      end
    end    
  endtask : drive_write_req_channel

  task respond_write_rsp_channel();
    forever begin
      logic rand_ready;
      
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        axi_intf.mst_wr_rsp_cb.BREADY <= 1'b0;
        @(axi_intf.mst_wr_rsp_cb);
      end
      
      std::randomize(rand_ready);
      axi_intf.mst_wr_rsp_cb.BREADY <= rand_ready;
      @(axi_intf.mst_wr_rsp_cb);
    end
  endtask : respond_write_rsp_channel
endclass : axi_driver

class axi_agent extends uvm_agent;
  axi_sequencer seqr;
  axi_driver    drvr;
  
  `uvm_component_utils(axi_agent)
  
  function new(string name = "axi_agent", uvm_component parent);
    super.new(name,parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    seqr = axi_sequencer::type_id::create("seqr", this);
    drvr = axi_driver::type_id::create("drvr", this);
  endfunction : build_phase
  
  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    drvr.seq_item_port.connect(seqr.seq_item_export);
  endfunction : connect_phase
endclass : axi_agent

class axi_bfm extends uvm_component; //BusFunctionalModel/Responder/Reactive Component
  virtual axi_interface axi_intf;
  axi_transaction       rd_txn_q[$];
  axi_transaction       wr_txn_q[$];
  logic [31:0]          axi_mem[256];
  
  `uvm_component_utils(axi_bfm)
  
  function new(string name = "SlaveBFM", uvm_component parent);
    super.new(name,parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    uvm_config_db #(virtual axi_interface)::get(this, "", "AxiInterface", axi_intf);
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    fork
      capture_read_request_interface();
      send_read_response();
      capture_write_request_interface();
      send_write_response();
    join
  endtask : run_phase
  
  task capture_read_request_interface();
    forever begin
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        bit rand_ready;
        std::randomize(rand_ready);
        axi_intf.slv_rd_req_cb.ARREADY <= rand_ready;
        @(axi_intf.slv_rd_req_cb);
      end
      
      if(axi_intf.slv_rd_req_cb.ARVALID === 1'b1) begin
        axi_transaction axi_txn;
        int unsigned    rand_dly;
        
        axi_txn = axi_transaction::type_id::create("axi_txn");
        axi_txn.CMD_TYPE = READ;
        axi_txn.ADDR     = axi_intf.slv_rd_req_cb.ARADDR;
        axi_txn.ID       = axi_intf.slv_rd_req_cb.ARID;
        rd_txn_q.push_back(axi_txn);
        
        std::randomize(rand_dly) with {rand_dly inside {[0:5]};};
        repeat(rand_dly) @(axi_intf.slv_rd_req_cb);
        
        axi_intf.slv_rd_req_cb.ARREADY <= 1'b1;
        @(axi_intf.slv_rd_req_cb);
        axi_intf.slv_rd_req_cb.ARREADY <= 1'b0;
        @(axi_intf.slv_rd_req_cb);
      end
      else begin
        axi_intf.slv_rd_req_cb.ARREADY <= 1'b0;
        @(axi_intf.slv_rd_req_cb);
      end
    end
  endtask : capture_read_request_interface
  
  task send_read_response();
    forever begin
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        axi_intf.slv_rd_rsp_cb.RVALID <= 1'b0;
        @(axi_intf.slv_rd_rsp_cb);
      end
      
      if(rd_txn_q.size() > 0) begin
        axi_transaction axi_txn;
        int unsigned    rand_dly;
        
        axi_txn = rd_txn_q.pop_front();
        std::randomize(rand_dly) with {rand_dly inside {[0:5]};};
        repeat(rand_dly) @(axi_intf.slv_rd_rsp_cb);
        
        axi_intf.slv_rd_rsp_cb.RVALID <= 1'b1;
        axi_intf.slv_rd_rsp_cb.RDATA  <= axi_mem[axi_txn.ADDR];
        axi_intf.slv_rd_rsp_cb.RID    <= axi_txn.ID;
        axi_intf.slv_rd_rsp_cb.RRESP  <= OKAY;
        
        while(axi_intf.slv_rd_rsp_cb.RREADY !== 1'b1) begin
          @(axi_intf.slv_rd_rsp_cb);
        end
        axi_intf.slv_rd_rsp_cb.RVALID <= 1'b0;
        @(axi_intf.slv_rd_rsp_cb);
      end
      else begin
        axi_intf.slv_rd_rsp_cb.RVALID <= 1'b0;
        @(axi_intf.slv_rd_rsp_cb);
      end
    end
  endtask : send_read_response
  
  task capture_write_request_interface();
    forever begin
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        bit rand_ready;
        std::randomize(rand_ready);
        axi_intf.slv_wr_req_cb.AWREADY <= rand_ready;
        std::randomize(rand_ready);
        axi_intf.slv_wr_dat_cb.WREADY <= rand_ready;
        @(axi_intf.slv_wr_req_cb);
      end
      
      if((axi_intf.slv_wr_req_cb.AWVALID === 1'b1) && (axi_intf.slv_wr_dat_cb.WVALID === 1'b1)) begin
        axi_transaction axi_txn;
        axi_txn = axi_transaction::type_id::create("axi_txn");
      
        fork
          begin
            int unsigned rand_dly;
        
            axi_txn.CMD_TYPE = WRITE;
            axi_txn.ADDR     = axi_intf.slv_wr_req_cb.AWADDR;
            axi_txn.ID       = axi_intf.slv_wr_req_cb.AWID;

            std::randomize(rand_dly) with {rand_dly inside {[0:5]};};
            repeat(rand_dly) @(axi_intf.slv_wr_req_cb);
        
            axi_intf.slv_wr_req_cb.AWREADY <= 1'b1;
            @(axi_intf.slv_wr_req_cb);
            axi_intf.slv_wr_req_cb.AWREADY <= 1'b0;
            @(axi_intf.slv_wr_req_cb);
          end
          begin
            int unsigned rand_dly;
        
            axi_txn.DATA  = axi_intf.slv_wr_dat_cb.WDATA;
            axi_txn.WSTRB = axi_intf.slv_wr_dat_cb.WSTRB;

            std::randomize(rand_dly) with {rand_dly inside {[0:5]};};
            repeat(rand_dly) @(axi_intf.slv_wr_dat_cb);
        
            axi_intf.slv_wr_dat_cb.WREADY <= 1'b1;
            @(axi_intf.slv_wr_dat_cb);
            axi_intf.slv_wr_dat_cb.WREADY <= 1'b0;
            @(axi_intf.slv_wr_dat_cb);
          end
        join
        wr_txn_q.push_back(axi_txn);
      end
      else begin
        axi_intf.slv_wr_req_cb.AWREADY <= 1'b0;
        axi_intf.slv_wr_dat_cb.WREADY <= 1'b0;
        @(axi_intf.slv_wr_dat_cb);
      end
    end
  endtask : capture_write_request_interface
  
  task send_write_response();
    forever begin
      while(axi_intf.aresetn === 1'b0) begin //DUT not in Reset
        axi_intf.slv_wr_rsp_cb.BVALID <= 1'b0;
        @(axi_intf.slv_wr_rsp_cb);
      end
      
      if(wr_txn_q.size() > 0) begin
        axi_transaction axi_txn;
        int unsigned    rand_dly;
        
        axi_txn = wr_txn_q.pop_front();
        for(int idx = 0; idx < 4; idx++) begin
          axi_mem[axi_txn.ADDR][idx*8 +: 8] = axi_txn.WSTRB[idx] ? axi_txn.DATA[idx*8 +: 8] : axi_mem[axi_txn.ADDR][idx*8 +: 8]; //idx*8 +: 8 --> Take From idx*8 upto 8 bits
        end
        
        std::randomize(rand_dly) with {rand_dly inside {[0:5]};};
        repeat(rand_dly) @(axi_intf.slv_wr_rsp_cb);
        
        axi_intf.slv_wr_rsp_cb.BVALID <= 1'b1;
        axi_intf.slv_wr_rsp_cb.BID    <= axi_txn.ID;
        axi_intf.slv_wr_rsp_cb.BRESP  <= OKAY;
        
        while(axi_intf.slv_wr_rsp_cb.BREADY !== 1'b1) begin
          @(axi_intf.slv_wr_rsp_cb);
        end
        axi_intf.slv_wr_rsp_cb.BVALID <= 1'b0;
        @(axi_intf.slv_wr_rsp_cb);
      end
      else begin
        axi_intf.slv_wr_rsp_cb.BVALID <= 1'b0;
        @(axi_intf.slv_wr_rsp_cb);
      end
    end
  endtask : send_write_response
endclass : axi_bfm

class axi_env extends uvm_env;
  axi_agent   agt;
  axi_bfm     bfm;
  
  `uvm_component_utils(axi_env)
  
  function new(string name = "MyEnv", uvm_component parent);
    super.new(name,parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agt = axi_agent::type_id::create("agt", this);
    bfm = axi_bfm::type_id::create("bfm", this);
  endfunction : build_phase
  
  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
  endfunction : connect_phase
endclass : axi_env

class test_sequence extends uvm_sequence #(axi_transaction);
  `uvm_object_utils(test_sequence)
  
  function new(string name = "test_sequence");
    super.new(name);
  endfunction : new
  
  task body();
    for(int idx = 0; idx < 256; idx++) begin
      axi_transaction axi_txn;
      
      `uvm_do_with(axi_txn, {ADDR == idx; CMD_TYPE == WRITE; WSTRB == 'hF;})
      #10ns;
      `uvm_do_with(axi_txn, {ADDR == idx; CMD_TYPE == READ;})
      #10ns;
    end
  endtask : body
endclass : test_sequence

class my_test extends uvm_test;
  axi_env my_env;
  
  `uvm_component_utils(my_test)
  
  function new(string name = "TestComponent", uvm_component parent);
    super.new(name, parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    my_env = axi_env::type_id::create("my_env", this);
  endfunction : build_phase
  
  function void end_of_elaboration_phase(uvm_phase phase);
    super.end_of_elaboration_phase(phase);
    uvm_top.print_topology();
  endfunction : end_of_elaboration_phase
  
  task run_phase(uvm_phase phase);
    test_sequence tst_seq;
    tst_seq = test_sequence::type_id::create("tst_seq");
    phase.phase_done.set_drain_time(this, 2us);
    phase.raise_objection(this);
    tst_seq.start(my_env.agt.seqr);
    phase.drop_objection(this);
  endtask : run_phase
endclass : my_test

module axi_tb();
  logic aclk;
  logic arstn;
  
  axi_interface axi_intf(aclk, arstn);

  always #5 aclk = ~aclk;
  
  initial begin
    uvm_config_db #(virtual axi_interface)::set(null, "*", "AxiInterface", axi_intf);
    run_test();
  end

  initial begin
    $dumpfile("dump.vcd"); $dumpvars;
      
    aclk = 1'b1;
    arstn = 1'b0;

    #20ns;
    arstn = 1'b1;
  end
endmodule
