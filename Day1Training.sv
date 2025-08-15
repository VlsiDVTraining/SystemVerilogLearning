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
        $dumpfile("dump.vcd"); $dumpvars;

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
