`timescale 1ns/1ps
module espacc_tb ();
`include "assert.v" // task needs to be declared in module

   reg clk;
   reg rst;

   reg [31:0]  conf_info_reg0;
   reg [31:0]  conf_info_reg1;
   reg         conf_done;

   reg         dma_read_ctrl_ready;
   wire        dma_read_ctrl_valid;
   wire [31:0] dma_read_ctrl_data_index;
   wire [31:0] dma_read_ctrl_data_length;
   wire [2:0]  dma_read_ctrl_data_size;

   wire        dma_read_chnl_ready;
   reg         dma_read_chnl_valid;
   reg [63:0]  dma_read_chnl_data;

   reg         dma_write_ctrl_ready;
   wire        dma_write_ctrl_valid;
   wire [31:0] dma_write_ctrl_data_index;
   wire [31:0] dma_write_ctrl_data_length;
   wire [2:0]  dma_write_ctrl_data_size;

   reg         dma_write_chnl_ready;
   wire        dma_write_chnl_valid;
   wire [63:0] dma_write_chnl_data;

   wire        acc_done;
   wire [31:0] debug;

   // local variables

   // memory to store program and data
   reg [31:0]  MEM [0:31];

   parameter DMA_BUS_WIDTH  = 32'd64;
   parameter HOST_MAX_INSTR = 32'd16;
   parameter HOST_MAX_DATA  = 32'd16;

   espacc_rtl_basic_dma
     #(.DMA_BUS_WIDTH(DMA_BUS_WIDTH),
       .HOST_MAX_INSTR(HOST_MAX_INSTR),
       .HOST_MAX_DATA(HOST_MAX_DATA)
       )
   // could also do
   //espacc_rtl_basic_dma64
   //  #(.HOST_MAX_INSTR(HOST_MAX_INSTR),
   //    .HOST_MAX_DATA(HOST_MAX_DATA)
   //    )
   dut
     (
      .clk (clk),
      .rst (rst),
      .dma_read_chnl_valid (dma_read_chnl_valid),
      .dma_read_chnl_data (dma_read_chnl_data),
      .dma_read_chnl_ready (dma_read_chnl_ready),
      .conf_info_reg0 (conf_info_reg0),
      .conf_info_reg1 (conf_info_reg1),
      .conf_done (conf_done),
      .acc_done (acc_done),
      .debug (debug),
      .dma_read_ctrl_valid (dma_read_ctrl_valid),
      .dma_read_ctrl_data_index (dma_read_ctrl_data_index),
      .dma_read_ctrl_data_length (dma_read_ctrl_data_length),
      .dma_read_ctrl_data_size (dma_read_ctrl_data_size),
      .dma_read_ctrl_ready (dma_read_ctrl_ready),
      .dma_write_ctrl_valid (dma_write_ctrl_valid),
      .dma_write_ctrl_data_index (dma_write_ctrl_data_index),
      .dma_write_ctrl_data_length (dma_write_ctrl_data_length),
      .dma_write_ctrl_data_size (dma_write_ctrl_data_size),
      .dma_write_ctrl_ready (dma_write_ctrl_ready),
      .dma_write_chnl_valid (dma_write_chnl_valid),
      .dma_write_chnl_data (dma_write_chnl_data),
      .dma_write_chnl_ready (dma_write_chnl_ready)
      );

   integer cyc = 2; // one clock cycle
   integer i = 0;
   integer beats_per_pass = 8; // hardwired here and in accelerator

   initial begin
      // set up waveform writing
      $dumpfile("espacc.vcd");
      $dumpvars(0,espacc_tb);
   end

   // generate clock
   initial begin
      clk <= 1'b0;
      forever begin
         #1 clk <= ~clk;
      end
   end

   // load program and data
   initial begin
      // program
      // Config Inst
      // Radix 4,dir:0,twid_first:0,twid,transpose,flush all 1,done 0,vlen 4
      MEM[0] = 32'b00000000000000001000111000110000;
      // Mem Inst
      // Load input data at a stride of 1 and vector stride of 4
      MEM[1] = 32'b11111111000000100000000010011111;
      // Spad base address for loading inputs
      MEM[2] = 32'b00000000000000000000000000000000;
      // Mem Inst
      // Load input data at a stride of 1 and vector stride of 4
      MEM[3] = 32'b11111111000000100000000010101111;
      // Twid mem base address
      MEM[4] = 32'b00000000000000000000000000000000;
      // Mem Inst
      // Store output data at a stride of 1 and vector stride of 4
      MEM[5] = 32'b11111111000000100000000011001111;
      // Spad base address for storing outputs
      MEM[6] = 32'b00000000000000000000000000000000;
      // Config Inst
      // Radix 4,dir:0,twid_first:0,twid:0,transpose:1,flush:1,done:1,vlen:4
      MEM[7] = 32'b00000000000000001001110000110000;
      // Mem Inst
      // Load input data at a stride of 1 and vector stride of 4
      MEM[8] = 32'b11111111000000100000000010011111;
      // Spad base address for loading inputs
      MEM[9] = 32'b00000000000000000000000000000000;
      // Mem Inst
      // Load twiddles at a stride of 1 and vector stride of 4
      MEM[10] = 32'b11111111000000100000000010101111;
      // Twid mem base address
      MEM[11] = 32'b00000000000000000000000000000000;
      // Mem Inst
      // This says to store data at a stride of 1 and vector stride of 4
      MEM[12] = 32'b11111111000000100000000011001111;
      // Spad base address for storing outputs
      MEM[13] = 32'b00000000000000000000000000000000;

      // pad out the remaining instructions
      MEM[14] = 32'b00000000000000000000000000000000;
      MEM[15] = 32'b00000000000000000000000000000000;

      // data
      MEM[16] = 32'd1;
      MEM[17] = 32'd2;
      MEM[18] = 32'd3;
      MEM[19] = 32'd4;
      MEM[20] = 32'd5;
      MEM[21] = 32'd6;
      MEM[22] = 32'd7;
      MEM[23] = 32'd8;
      MEM[24] = 32'd9;
      MEM[25] = 32'd10;
      MEM[26] = 32'd11;
      MEM[27] = 32'd12;
      MEM[28] = 32'd13;
      MEM[29] = 32'd14;
      MEM[30] = 32'd15;
      MEM[31] = 32'd16;
   end

   initial begin
      // initialize values
      rst <= 1'b1;
      conf_info_reg0 <= 32'b0;
      conf_info_reg1 <= 32'b0;
      conf_done <= 1'b0;
      dma_read_ctrl_ready <= 1'b0;
      dma_read_chnl_valid <= 1'b0;
      dma_write_ctrl_ready <= 1'b0;
      dma_write_chnl_ready <= 1'b0;

      // start running
      #cyc; // first clock cycle
      rst <= 1'b0; // trigger reset active low
      #cyc;

      assert(dma_read_ctrl_valid == 1'b0); // initial state of valid
      rst <= 1'b1; // complete, move to IDLE state
      #cyc;

      // set conf_info_reg0 to a value, and set conf_done to 1
      conf_info_reg0 <= 32'h00000000; // relative addr of MEM holding program
      conf_info_reg1 <= 32'd16; // number of instructions in the program
      conf_done <= 1'b1; // go to CONFIG state
      #cyc;

      conf_done <= 1'b0; // lower conf_done after one cycle
      #cyc;

      // check read state
      assert(dma_read_ctrl_data_index == 32'h00000000); // 0 MEM addr
      assert(dma_read_ctrl_data_length == 32'd8); // 16 instr, 8 beats
      assert(dma_read_ctrl_data_size == 3'b010);  // WORD, 32 bits
      assert(dma_read_ctrl_valid == 1'b1); // read valid

      // set dma_read_ctrl_ready, wait a cycle then deactivate it.
      dma_read_ctrl_ready <= 1'b1;
      #cyc;
      dma_read_ctrl_ready <= 1'b0;
      #cyc;

      // put program in read channel, set chnl_valid.
      // test bench construct to replicate DMA, not synthesizable
      for (i=0; i<dma_read_ctrl_data_length;i++) begin
         dma_read_chnl_data <= MEM[i];
         dma_read_chnl_valid <= 1'b1;
         #cyc;
      end

      // deassert read_chnl_valid
      dma_read_chnl_valid <= 1'b0;
      #cyc;

      #(5*cyc); // for gtkwave readability

      // PASS 0

      // accelerator in read_ctrl state, read_ctrl_valid == 1
      // host read ctrl ready
      dma_read_ctrl_ready <= 1'b1;
      #cyc;
      dma_read_ctrl_ready <= 1'b0;
      #cyc;

      // put the data on the channel
      for (i=HOST_MAX_INSTR;i<HOST_MAX_INSTR+beats_per_pass;i++) begin
         dma_read_chnl_data <= MEM[i];
         dma_read_chnl_valid <= 1'b1;
         #cyc;
      end
      // deassert read_chnl_valid
      dma_read_chnl_valid <= 1'b0;

      #(8*cyc); // takes 8 cycles to finish first pass

      // do an accelerator write to host
      dma_write_ctrl_ready <= 1'b1;
      #cyc;

      // deassert ctrl_ready
      dma_write_ctrl_ready <= 1'b0;
      #cyc;

      // assert chnl_ready
      dma_write_chnl_ready <= 1'b1;

      // read the accelerator data
      for (i=HOST_MAX_INSTR;i<HOST_MAX_INSTR+beats_per_pass;i++) begin
         MEM[i] <= dma_write_chnl_data;
         #cyc;
      end

      // deassert chnl_ready
      dma_write_chnl_ready <= 1'b0;
      #cyc;

      // PASS 1

      // accelerator in read_ctrl state, read_ctrl_valid == 1
      // host read ctrl ready
      dma_read_ctrl_ready <= 1'b1;
      #cyc;
      dma_read_ctrl_ready <= 1'b0;
      #cyc;

      // put the data on the channel
      for (i=HOST_MAX_INSTR;i<HOST_MAX_INSTR+beats_per_pass;i++) begin
         dma_read_chnl_data <= MEM[i + beats_per_pass];
         dma_read_chnl_valid <= 1'b1;
         #cyc;
      end
      // deassert read_chnl_valid
      dma_read_chnl_valid <= 1'b0;

      #(8*cyc); // takes 8 cycles to finish first pass

      // do an accelerator write to host
      dma_write_ctrl_ready <= 1'b1;
      #cyc;

      // deassert ctrl_ready
      dma_write_ctrl_ready <= 1'b0;
      #cyc;

      // assert chnl_ready
      dma_write_chnl_ready <= 1'b1;

      // read the accelerator data
      for (i=HOST_MAX_INSTR;i<HOST_MAX_INSTR+beats_per_pass;i++) begin
         MEM[i + beats_per_pass] <= dma_write_chnl_data;
         #cyc;
      end

      // deassert chnl_ready
      dma_write_chnl_ready <= 1'b0;
      #cyc;

   end

   always @(posedge clk) begin
      $display("time=%3d", $time);
   end

   initial begin
      #200 $finish;
   end

endmodule
