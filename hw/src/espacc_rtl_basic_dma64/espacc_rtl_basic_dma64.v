module espacc_rtl_basic_dma64
  #(parameter HOST_MAX_INSTR=16,
    parameter HOST_MAX_DATA=16)
  (
   clk,
   rst,
   dma_read_chnl_valid,
   dma_read_chnl_data,
   dma_read_chnl_ready,
   conf_info_reg0,
   conf_info_reg1,
   conf_done,
   acc_done,
   debug,
   dma_read_ctrl_valid,
   dma_read_ctrl_data_index,
   dma_read_ctrl_data_length,
   dma_read_ctrl_data_size,
   dma_read_ctrl_ready,
   dma_write_ctrl_valid,
   dma_write_ctrl_data_index,
   dma_write_ctrl_data_length,
   dma_write_ctrl_data_size,
   dma_write_ctrl_ready,
   dma_write_chnl_valid,
   dma_write_chnl_data,
   dma_write_chnl_ready
   );

   input clk;
   input rst;

   input [31:0]  conf_info_reg0;
   input [31:0]  conf_info_reg1;
   input 	 conf_done;

   input 	     dma_read_ctrl_ready;
   output 	     dma_read_ctrl_valid;
   output [31:0]     dma_read_ctrl_data_index;
   output [31:0]     dma_read_ctrl_data_length;
   output [2:0]      dma_read_ctrl_data_size;

   output            dma_read_chnl_ready;
   input             dma_read_chnl_valid;
   input [63:0]      dma_read_chnl_data; // 64 bit width

   input             dma_write_ctrl_ready;
   output            dma_write_ctrl_valid;
   output [31:0]     dma_write_ctrl_data_index;
   output [31:0]     dma_write_ctrl_data_length;
   output [2:0]      dma_write_ctrl_data_size;

   input             dma_write_chnl_ready;
   output            dma_write_chnl_valid;
   output [63:0]     dma_write_chnl_data; // 64 bit width

   output            acc_done;
   output [31:0]     debug;

   espacc_rtl_basic_dma
     #(
       .DMA_BUS_WIDTH(64), // 64 bit width
       .HOST_MAX_INSTR(HOST_MAX_INSTR),
       .HOST_MAX_DATA(HOST_MAX_DATA)
       )
   basic_dma
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

endmodule
