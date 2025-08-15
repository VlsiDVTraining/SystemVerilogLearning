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

    initial begin
        repeat(1000) begin
            #10ns;
            data_status = $urandom();
            input_data = $urandom();
        end
    end
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

    initial begin
        mem_addr = 0;
        #60ns;
        repeat(4) begin
            mem_wr_en = 1'b1;
            mem_data = $urandom();
            #10ns;
            mem_addr++;
        end
        mem_wr_en = 1'b0;
    end
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

module switch_tb();
    logic clk;
    logic rst;

    input_interface inp_intf(clk, rst);
    memory_interface mem_intf(clk, rst);
    output_interface out1_intf(clk, rst);
    output_interface out2_intf(clk, rst);
    output_interface out3_intf(clk, rst);
    output_interface out4_intf(clk, rst);

    switch dut(
        .data_0   (out1_intf.data),
        .data_1   (out2_intf.data),
        .data_2   (out3_intf.data),
        .data_3   (out4_intf.data),
        .ready_0  (out1_intf.ready),
        .ready_1  (out2_intf.ready),
        .ready_2  (out3_intf.ready),
        .ready_3  (out4_intf.ready),
        .read_0   (out1_intf.read),
        .read_1   (out2_intf.read),
        .read_2   (out3_intf.read),
        .read_3   (out4_intf.read),
        .mem_wr_en(mem_intf.mem_wr_en),
        .mem_addr (mem_intf.mem_addr),
        .mem_data (mem_intf.mem_data),
        .input_data(inp_intf.input_data),
        .data_status(inp_intf.data_status),
        .data_stall(inp_intf.data_stall),
        .clk      (clk),
        .reset    (rst)
    );

    always #5 clk = ~clk;

    initial begin
        transaction txn;
      
        //$dumpfile("dump.vcd"); $dumpvars;
      
        txn = new();
        txn.randomize();
        txn.display_packet();

        clk = 1'b1;
        rst = 1'b0;

        #10ns;
        rst = 1'b1;
        #20ns;
        rst = 1'b0;

        #1500ns;
        $finish();
    end
endmodule
