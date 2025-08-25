import uvm_pkg::*;
`include "uvm_macros.svh"

parameter logic [7:0] PORT0 = 'hAB;
parameter logic [7:0] PORT1 = 'hBC;
parameter logic [7:0] PORT2 = 'hDE;
parameter logic [7:0] PORT3 = 'hEF;

interface input_interface(input logic clk, logic rst);
    logic data_status;
    logic data_stall;
    logic [7:0] input_data;

    clocking driver_cb @(posedge clk); //Timing + Direction
        input data_stall;
        output data_status;
        output input_data;
    endclocking

    clocking monitor_cb @(posedge clk);
        input data_stall;
        input data_status;
        input input_data;
    endclocking
endinterface

interface memory_interface(input logic clk, logic rst);
    logic mem_wr_en;
    logic [1:0] mem_addr;
    logic [7:0] mem_data;

    clocking driver_cb @(posedge clk); //Timing + Direction
        output mem_wr_en;
        output mem_addr;
        output mem_data;
    endclocking

    clocking monitor_cb @(posedge clk);
        input mem_wr_en;
        input mem_addr;
        input mem_data;
    endclocking
endinterface

interface output_interface(input logic clk, logic rst);
    logic read;
    logic ready;
    logic [7:0] data;

    clocking driver_cb @(posedge clk); //Timing + Direction
        output read;
        input ready;
        input data;
    endclocking

    clocking monitor_cb @(posedge clk);
        input read;
        input ready;
        input data;
    endclocking
endinterface

//Transaction Class - UVM_OBJECT --> UVM_SEQUENCE_ITEM
class transaction extends uvm_sequence_item;
  //Destination Address, Source Address, Length, DATA, FCS
  
  rand logic [7:0] dest_addr; //Byte0
  rand logic [7:0] src_addr;  //Byte1
  rand logic [7:0] length;    //Byte2
  rand logic [7:0] data[]; //Fixed_aray or Dynamic_array?
       logic [7:0] FCS;    //Can this be random?
       logic [7:0] pack_data[]; //pack the packet in order as DA, SA, Length, DATA, FCS

  //`uvm_object_utils(transaction) //Replicate the same template as in uvm_sequence_item
  //Does Template replication only for Transaction class and if i need the replication
  //for all the members of the transaction class do it as below
  `uvm_object_utils_begin(transaction)
  `uvm_field_int(dest_addr, UVM_DEFAULT)
  `uvm_field_int(src_addr, UVM_DEFAULT)
  `uvm_field_int(length, UVM_DEFAULT)
  `uvm_field_array_int(data, UVM_DEFAULT)
  `uvm_field_int(FCS, UVM_DEFAULT)
  `uvm_field_array_int(pack_data, UVM_DEFAULT)
  `uvm_object_utils_end
  
  //Template ??? - Copy(Shallow Copy), Print, Compare, Clone(Deep Copy)
  
  constraint dest_addr_c {
    dest_addr inside {PORT0, PORT1, PORT2, PORT3};
  }
  
  //No constraint on Source Addr and Length
  
  constraint data_c {
    solve length before data;
    data.size() == length;
  }
  
  function new(string name = "Switch_Transaction");
    super.new(name); //Call the constructor of the parent
  endfunction : new
  
  function logic [7:0] calc_fcs();
    logic [7:0] live_FCS = 'h0;
    
    live_FCS = live_FCS ^ dest_addr; //XOR GATE --> Behavior??? (1&0/0&1 --> 1) [Odd Parity of Ones]
    live_FCS = live_FCS ^ src_addr;
    live_FCS = live_FCS ^ length;
    for(int idx = 0; idx < length; idx++) begin
      live_FCS = live_FCS ^ data[idx];
    end
    return live_FCS;
  endfunction : calc_fcs
  
  function void post_randomize();
    super.post_randomize();
    FCS = calc_fcs();
    pack_packet();
  endfunction : post_randomize
  
  function void pack_packet();
    pack_data = new[4+length];
    pack_data[0] = dest_addr;
    pack_data[1] = src_addr;
    pack_data[2] = length;
    for(int idx = 0; idx < length; idx++) begin
      pack_data[3+idx] = data[idx];
    end
    //pack_data[4+length-1] = FCS; //4-1
    pack_data[3+length] = FCS;
  endfunction : pack_packet
  
  function void display_packet();
    //$display(); - UVM_INFO
    //`uvm_info(NAME, WHAT YOU NEED TO PRINT, VERBOSITY)
    `uvm_info(get_name(), $sformatf("DESTINATION_ADDR = %0d", dest_addr), UVM_NONE)
    `uvm_info(get_name(), $sformatf("SOURCE_ADDR = %0d", src_addr), UVM_LOW)
    `uvm_info(get_name(), $sformatf("LENGTH = %0d", length), UVM_MEDIUM)
    `uvm_info(get_name(), $sformatf("DATA_PACKET - %0d = %p", data.size(), data), UVM_HIGH)
    `uvm_info(get_name(), $sformatf("FCS = %0d", FCS), UVM_HIGH)
    `uvm_info(get_name(), $sformatf("PACKED_PACKET - %0d = %p", pack_data.size(), pack_data), UVM_DEBUG)
  endfunction : display_packet
  
  //Verbosity --> Defaulted to UVM_LOW --> +UVM_VERBOSITY=UVM_LOW
  //UVM_NONE   - Always Printed
  //UVM_LOW    - Printed only if my verbosity != UVM_NONE
  //UVM_MEDIUM - Printed only if my verbosity != UVM_LOW
  //UVM_HIGH   - Printed only if my verbosity != UVM_MEDIUM
  //UVM_DEBUG  - Printed only if my verbosity != UVM_HIGH
endclass : transaction

class sequencer extends uvm_sequencer #(transaction); //# --> Type Definition
  `uvm_component_utils(sequencer)
  
  function new(string name = "sequencer", uvm_component parent);
    super.new(name, parent);
  endfunction : new
endclass : sequencer

class driver extends uvm_driver #(transaction);
  virtual input_interface  drv_inp_intf;
  virtual memory_interface drv_mem_intf;
  bit                      config_cmpltd;
  
  `uvm_component_utils(driver)
  
  function new(string name = "my_driver", uvm_component parent);
    super.new(name, parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    uvm_config_db #(virtual input_interface)::get(this, "", "InputInterface", drv_inp_intf);
    uvm_config_db #(virtual memory_interface)::get(this, "", "ConfigInterface", drv_mem_intf);
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    //reset_sequence
    //config_mem_intf
    //drive_transaction
    fork
      reset_intf();  //task or function
      config_intf();
      drive_intf();
    join
  endtask : run_phase
  
  task reset_intf();
    forever begin
      @(posedge drv_mem_intf.rst);
      config_cmpltd = 1'b0;
      
      while(drv_mem_intf.rst == 1'b1) begin //Why Needed? - Dont drive anything until reset is deasserted
        drv_inp_intf.driver_cb.data_status <= 1'b0;
        drv_inp_intf.driver_cb.input_data  <= 8'd0;
      
        drv_mem_intf.driver_cb.mem_wr_en <= 1'b0;
        drv_mem_intf.driver_cb.mem_addr  <= 2'd0;
        drv_mem_intf.driver_cb.mem_data  <= 8'h0;
        
        @(drv_inp_intf.driver_cb);
      end
    end
  endtask : reset_intf
  
  task config_intf();
    forever begin
      @(negedge drv_mem_intf.rst); //Wait for Reset to be deasserted
      
      @(drv_mem_intf.driver_cb); //Wait for Postive Edge of the clk
      
      for(int idx = 0; idx < 4; idx++) begin
        drv_mem_intf.driver_cb.mem_wr_en <= 1'b1;
        drv_mem_intf.driver_cb.mem_addr <= idx;
        if(idx == 0) drv_mem_intf.driver_cb.mem_data <= PORT0;
        if(idx == 1) drv_mem_intf.driver_cb.mem_data <= PORT1;
        if(idx == 2) drv_mem_intf.driver_cb.mem_data <= PORT2;
        if(idx == 3) drv_mem_intf.driver_cb.mem_data <= PORT3;
        @(drv_mem_intf.driver_cb); //Waiting for another posedge to move ahead
      end
      
      drv_mem_intf.driver_cb.mem_wr_en <= 1'b0;
      drv_mem_intf.driver_cb.mem_addr  <= 2'd0;
      drv_mem_intf.driver_cb.mem_data  <= 8'h0;
      @(drv_mem_intf.driver_cb);
      config_cmpltd = 1'b1; //At this point, configuration is completed, driver can now start sending transactions to DUT
    end
  endtask : config_intf
  
  task drive_intf(); //Driver - uvm_sequence_item_pull_port --> Sequencer - uvm_sequence_item_pull_export
    forever begin
      transaction  txn;
      int unsigned pkt_size;
      
      wait(config_cmpltd == 1'b1);
      txn = new();
      seq_item_port.get_next_item(txn); //Wait for Sequencer to send the item to be driven on the interface
      txn.pack_packet();
      pkt_size = $size(txn.pack_data); //for Fixed and Dynamic Array
      
        //if(drv_inp_intf.data_stall == 1'b0) begin
        //  drv_inp_intf.driver_cb.data_status <= 1'b1;
        //  drv_inp_intf.driver_cb.input_data  <= txn.pack_data[idx];
        //  @(drv_inp_intf.driver_cb);
        //end
        //else begin
        //  while(drv_inp_intf.data_stall == 1'b1) begin
        //    drv_inp_intf.driver_cb.data_status <= 1'b0;
        //    drv_inp_intf.driver_cb.input_data  <= 8'h0;
        //    @(drv_inp_intf.driver_cb);
        //  end       
        //  drv_inp_intf.driver_cb.data_status <= 1'b1;
        //  drv_inp_intf.driver_cb.input_data  <= txn.pack_data[idx];
        //  @(drv_inp_intf.driver_cb);
        //end --> To Simplify
      for(int idx = 0; idx < pkt_size; idx++) begin
        while(drv_inp_intf.data_stall == 1'b1) begin
          drv_inp_intf.driver_cb.data_status <= 1'b0;
          drv_inp_intf.driver_cb.input_data  <= 8'h0;
          @(drv_inp_intf.driver_cb);
        end
        drv_inp_intf.driver_cb.data_status <= 1'b1;
        drv_inp_intf.driver_cb.input_data  <= txn.pack_data[idx];
        @(drv_inp_intf.driver_cb);
      end
      
      drv_inp_intf.driver_cb.data_status <= 1'b0;
      drv_inp_intf.driver_cb.input_data  <= 8'h0;
      @(drv_inp_intf.driver_cb);
      seq_item_port.item_done(); //Notifying sequencer that i have sent this transaction to the interface
    end
  endtask : drive_intf
endclass : driver

class test_sequence extends uvm_sequence #(transaction);
  `uvm_object_utils(test_sequence)
  
  function new(string name = "Test_Sequence");
    super.new(name);
  endfunction : new
  
  task body();
    uvm_test_done.raise_objection(); //phase handle is not availble with sequence because it is a uvm_object not component so using global handle
    //super.body();
    for(int idx = 0; idx < 1; idx++) begin
      transaction my_txn;
      
      //my_txn = new(); //new --> create
      //my_txn.randomize();
      //start_item(my_txn);
      //finish_item(my_txn);
      `uvm_do(my_txn) //this macro takes care of all 4 steps mentioned above
      `uvm_info(get_name(), $sformatf("SENT_PACKET = %p", my_txn.pack_data), UVM_NONE)
    end
    uvm_test_done.drop_objection();
  endtask
endclass : test_sequence

class input_monitor extends uvm_monitor;
  virtual input_interface mon_inp_intf;
  uvm_analysis_port #(transaction) inp_mon_port;
  
  `uvm_component_utils(input_monitor)
  
  function new(string name = "input_monitor", uvm_component parent);
    super.new(name, parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    uvm_config_db #(virtual input_interface)::get(this, "", "InputInterface", mon_inp_intf);
    inp_mon_port = new("inp_mon_port", this);
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    logic [7:0]  data_q[$];
    int unsigned inc_cnt = 0;
    int unsigned exp_cnt = 0;
    
    forever begin
      @(mon_inp_intf.monitor_cb);
      if((mon_inp_intf.monitor_cb.data_status == 1'b1) && (mon_inp_intf.monitor_cb.data_stall == 1'b0)) begin
        data_q.push_back(mon_inp_intf.monitor_cb.input_data);
        inc_cnt += 1; //Received DA,SA,LEN
        //exp_cnt += 1; //Issue here -- every cycle exp_cnt can grow -- Needs to be saturated/decided at the LEN phase(3rd)
        if(inc_cnt == 3) begin //LEN
          exp_cnt = data_q[$] + 4; //DATA_LEN + DA,SA,LEN,FCS
        end
        if(exp_cnt == inc_cnt) begin
          transaction mon_txn;
          mon_txn = transaction::type_id::create("mon_txn");       
          mon_txn.pack_data = new[exp_cnt];
          for(int idx = 0; idx < exp_cnt ; idx++) begin
            mon_txn.pack_data[idx] = data_q.pop_front();
          end
          `uvm_info(get_name(), $sformatf("RECEIVED_INPUT_MONITOR_PACKET = %p", mon_txn.pack_data), UVM_NONE)
          inc_cnt = 0;
          exp_cnt = 0;
          inp_mon_port.write(mon_txn);
        end
      end
    end
  endtask : run_phase
endclass : input_monitor

class output_monitor extends uvm_monitor;
  int unsigned             mon_port_num;
  virtual output_interface mon_out_intf;
  uvm_analysis_port #(transaction) out_mon_port;
  
  `uvm_component_utils(output_monitor)
  
  function new(string name = "output_monitor", uvm_component parent);
    super.new(name,parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    uvm_config_db #(virtual output_interface)::get(this, "", $sformatf("OutputInterface%0d", mon_port_num), mon_out_intf);
    out_mon_port = new("out_mon_port", this);
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    logic [7:0]  data_q[$];
    int unsigned inc_cnt = 0;
    int unsigned exp_cnt = 0;
    
    forever begin
      @(mon_out_intf.monitor_cb);
      if((mon_out_intf.monitor_cb.read == 1'b1) && (mon_out_intf.monitor_cb.ready == 1'b1)) begin
        data_q.push_back(mon_out_intf.monitor_cb.data);
        inc_cnt += 1; //Received DA,SA,LEN
        //exp_cnt += 1; //Issue here -- every cycle exp_cnt can grow -- Needs to be saturated/decided at the LEN phase(3rd)
        if(inc_cnt == 4) begin //LEN
          exp_cnt = data_q[$] + 5; //DATA_LEN + DUMMY,DA,SA,LEN,FCS
        end
        if(exp_cnt == inc_cnt) begin
          transaction mon_txn;
          mon_txn = transaction::type_id::create("mon_txn");       
          mon_txn.pack_data = new[exp_cnt-1]; //Discard the Dummy Packet
          data_q.pop_front();
          for(int idx = 0; idx < exp_cnt ; idx++) begin
            mon_txn.pack_data[idx] = data_q.pop_front();
          end
          `uvm_info(get_name(), $sformatf("RECEIVED_OUTPUT_MONITOR_%0d_PACKET = %p", mon_port_num-1, mon_txn.pack_data), UVM_NONE)
          inc_cnt = 0;
          exp_cnt = 0;
          out_mon_port.write(mon_txn);
        end
      end
    end
  endtask : run_phase
endclass : output_monitor

class agent extends uvm_agent;
  sequencer seqr; //Export
  driver    drvr; //Port
  input_monitor  inp_mon;
  output_monitor out_mon[4];
  
  `uvm_component_utils(agent)
  
  function new(string name = "ActiveAgent", uvm_component parent);
    super.new(name, parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    //seqr = new(); //Not possible in UVM instead do Create
    seqr = sequencer::type_id::create("seqr", this); //new and also copy the templates
    drvr = driver::type_id::create("drvr", this);
    inp_mon = input_monitor::type_id::create("inp_mon", this);
    for(int idx = 0; idx < 4; idx++) begin
      out_mon[idx] = output_monitor::type_id::create($sformatf("out_mon_%0d", idx), this);
      out_mon[idx].mon_port_num = idx+1;
    end
  endfunction : build_phase
  
  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    drvr.seq_item_port.connect(seqr.seq_item_export); //TLM Connection
  endfunction : connect_phase
endclass : agent

class scoreboard extends uvm_scoreboard;
  uvm_tlm_analysis_fifo #(transaction) inp_mon_fifo;
  uvm_tlm_analysis_fifo #(transaction) out_mon_fifo[4];
  
  `uvm_component_utils(scoreboard)
  
  function new(string name = "scoreboard", uvm_component parent);
    super.new(name, parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    inp_mon_fifo = new("inp_mon_fifo", this);
    for(int idx = 0; idx < 4; idx++) begin
      out_mon_fifo[idx] = new($sformatf("out_mon_fifo_%0d", idx), this);
    end
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    //TODO
  endtask : run_phase
endclass : scoreboard

class env extends uvm_env;
  agent      agt;
  scoreboard scbd;
  
  `uvm_component_utils(env)
  
  function new(string name = "MyEnv", uvm_component parent);
    super.new(name,parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agt = agent::type_id::create("agt", this);
    scbd = scoreboard::type_id::create("scbd", this);
  endfunction : build_phase
  
  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    agt.inp_mon.inp_mon_port.connect(scbd.inp_mon_fifo.analysis_export);
    for(int idx = 0; idx < 4; idx++) begin
      agt.out_mon[idx].out_mon_port.connect(scbd.out_mon_fifo[idx].analysis_export);
    end
  endfunction : connect_phase
endclass : env

class my_test extends uvm_test;
  env my_env;
  
  `uvm_component_utils(my_test)
  
  function new(string name = "TestComponent", uvm_component parent);
    super.new(name, parent);
  endfunction : new
  
  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    my_env = env::type_id::create("my_env", this);
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
    tst_seq.start(my_env.agt.seqr); //Connect the sequencer hierarchically starting from env --> agt --> seqr
    phase.drop_objection(this);
  endtask : run_phase
endclass : my_test

module switch_tb();
    logic clk;
    logic rst;

    input_interface  inp_intf(clk, rst);
    memory_interface mem_intf(clk, rst);
    output_interface out1_intf(clk, rst);
    output_interface out2_intf(clk, rst);
    output_interface out3_intf(clk, rst);
    output_interface out4_intf(clk, rst);

    switch dut(
        .data_0      (out1_intf.data),
        .data_1      (out2_intf.data),
        .data_2      (out3_intf.data),
        .data_3      (out4_intf.data),
        .ready_0     (out1_intf.ready),
        .ready_1     (out2_intf.ready),
        .ready_2     (out3_intf.ready),
        .ready_3     (out4_intf.ready),
        .read_0      (out1_intf.read),
        .read_1      (out2_intf.read),
        .read_2      (out3_intf.read),
        .read_3      (out4_intf.read),
        .mem_wr_en   (mem_intf.mem_wr_en),
        .mem_addr    (mem_intf.mem_addr),
        .mem_data    (mem_intf.mem_data),
        .input_data  (inp_intf.input_data),
        .data_status (inp_intf.data_status),
        .data_stall  (inp_intf.data_stall),
        .clk         (clk),
        .reset       (rst)
    ); 

    always #5 clk = ~clk;
  
    assign out1_intf.read = dut.ready_0;
    assign out2_intf.read = dut.ready_1;
    assign out3_intf.read = dut.ready_2;
    assign out4_intf.read = dut.ready_3;
  
    initial begin
      //uvm_config_db #(Type of Transaction/Interface)::set(<Hier>, <Path>, <Name>, <Actual Handle>)
      //null  - for Visibility through the testbench, this - visibility is limited only to the child below this class
      uvm_config_db #(virtual input_interface)::set(null, "*", "InputInterface", inp_intf);
      uvm_config_db #(virtual memory_interface)::set(null, "*", "ConfigInterface", mem_intf);
      uvm_config_db #(virtual output_interface)::set(null, "*", "OutputInterface1", out1_intf);
      uvm_config_db #(virtual output_interface)::set(null, "*", "OutputInterface2", out2_intf);
      uvm_config_db #(virtual output_interface)::set(null, "*", "OutputInterface3", out3_intf);
      uvm_config_db #(virtual output_interface)::set(null, "*", "OutputInterface4", out4_intf);
      run_test(); //Invoke the uvm_test
    end

    initial begin
        $dumpfile("dump.vcd"); $dumpvars;
      
        clk = 1'b1;
        rst = 1'b0;

        #10ns;
        rst = 1'b1;
        #20ns;
        rst = 1'b0;
    end
endmodule
