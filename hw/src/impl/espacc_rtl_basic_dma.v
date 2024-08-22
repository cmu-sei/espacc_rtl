// For interface description see
// https://www.esp.cs.columbia.edu/docs/specs/esp_accelerator_specification.pdf

// Conventions

// - bitwidth: number of bits. This is typically associated to a signal, or to
//   a unit of data.
// - token: the unit of input or output data transferred between the
//   accelerator and the ESP socket. The bitwidth of a token depends on the
//   particular accelerator and may vary across different transactions over
//   a bus or data channel.
// - beat: the unit of data transferred on a bus, or a data channel. The
//   bitwidth of one beat depends on the particular implementation of the
//   accelerator (e.g. dma32 or dma64) and not on the data type of the input
//   or output token in a transaction. Therefore, for any given implementation
//   of an ESP accelerator, the bitwidth of a beat is constant.
// - flit: the unit of data transferred over a network-on-chip (NoC). For ESP
//   accelerators, the bitwidth of a flit is equal to the bitwidth of a beat
//   plus two bits. These additional bits indicate if the flit is the head,
//   part of the body, or the tail of a packet.
// - packet: a set of flits transferred in an ordered sequence across the NoC.
//   Packets must have one header flit, one tail flit and as many body flits
//   as necessary. Single-flit packets have just one flit with both head and
//   tail bits set. A packet that is granted a link of the NoC will traverse
//   such link from head to tail not interleaved with another packet.
// - initiator or master: a component that can initiate a transaction over a
//   bus, or a NoC.
// - target or slave: a component that servers a transaction initiated by a
//   master.
// - latency-insensitive channel (LIC): a bundle of data wires and two control
//   wires named ready and valid. During read transactions, the master drives
//   the ready control signal, while the slave drives the data and the paired
//   valid control signal. Roles are inverted for write transactions. A beat
//   is transferred over a LIC when both ready and valid are set. Both master
//   and slave have the ability to delay the transfer of a beat for as many
//   cycles as necessary. For more on latency-insensitive channels please
//   refer to [Carloni, 2015].
// - CSR: configuration and/or status register.
// - DMA: the acronym for direct-memory access. When referring to an ESP
//   accelerator, the term DMA refers to the mechanism used by the accelerator
//   to access data in the system memory hierarchy. A DMA transaction
//   initiated by an accelerator in ESP may be accessing external memory
//   directly or by mediation of the ESP cache hierarchy. The selection is
//   managed by software at run time and is transparent to the accelerator.
// - PLM: the accelerator's private local memory, composed of a set of SRAM
//   bank groups customized for the accelerator's datapath.

module espacc_rtl_basic_dma
  #(parameter DMA_BUS_WIDTH = 64,
    parameter HOST_MAX_INSTR = 16,
    parameter HOST_MAX_DATA  = 16)
  (                            // driver: description, valid/active (duration)
   clk,                        // sckt: clock
   rst,                        // sckt: reset, low
   dma_read_chnl_valid,        // sckt: when high with _ready, data can be read
   dma_read_chnl_data,         // sckt: data to be read
   dma_read_chnl_ready,        // acc: ready to read data, active high
   /* <<--params-list-->> */
   conf_info_reg0,             // sckt: config input, valid if conf_done high
   conf_info_reg1,             // sckt: config input, valid if conf_done high
   conf_done,                  // sckt: config regs valid, high one clk cyc
   acc_done,                   // acc: accelerator done, high one clk cyc
   debug,                      // acc: 32 bit error code/debug register
   dma_read_ctrl_valid,        // acc: new request, data fields must be valid
   dma_read_ctrl_data_index,   // acc: offset into dma in num beats, start addr
   dma_read_ctrl_data_length,  // acc: length of dma in num beats
   dma_read_ctrl_data_size,    // acc: bitwidth of data token
   dma_read_ctrl_ready,        // sckt: socket ready to accept, indep. of vld
   dma_write_ctrl_valid,       // acc: new request, data fields must be valid
   dma_write_ctrl_data_index,  // acc: offset into dma in num beats, start addr
   dma_write_ctrl_data_length, // acc: length of dma in num beats
   dma_write_ctrl_data_size,   // acc: bitwidth of data token
   dma_write_ctrl_ready,       // sckt: socket ready to accept, indep. of vld
   dma_write_chnl_valid,       // acc: write channel ready
   dma_write_chnl_data,        // acc: data to write
   dma_write_chnl_ready        // sckt: sckt ready for data, xfer with _valid
   );

   // Encoding of DMA size in (read/write)_ctrl_data_size
   //| Encoding | Name  | Bitwidth |
   //-------------------------------
   //|   000    | Byte  |     8    |
   //|   001    | Hword |    16    |
   //|   010    | Word  |    32    |
   //|   011    | Dword |    64    |

   input clk;
   input rst;

   /* <<--params-def-->> */
   input [31:0]  conf_info_reg0;
   input [31:0]  conf_info_reg1;
   input 	 conf_done;

   input 	     dma_read_ctrl_ready;
   output reg	     dma_read_ctrl_valid;
   output reg [31:0] dma_read_ctrl_data_index;
   output reg [31:0] dma_read_ctrl_data_length;
   output reg [2:0]  dma_read_ctrl_data_size;

   output reg        dma_read_chnl_ready;
   input             dma_read_chnl_valid;
   input [DMA_BUS_WIDTH-1:0]      dma_read_chnl_data;

   input             dma_write_ctrl_ready;
   output reg        dma_write_ctrl_valid;
   output reg [31:0] dma_write_ctrl_data_index;
   output reg [31:0] dma_write_ctrl_data_length;
   output reg [2:0]  dma_write_ctrl_data_size;

   input             dma_write_chnl_ready;
   output reg        dma_write_chnl_valid;
   output reg [DMA_BUS_WIDTH-1:0] dma_write_chnl_data;

   output reg        acc_done;
   output reg [31:0] debug;

   // start by asserting a read and wait until the data is valid, then write
   reg [2:0]     state;
   reg [2:0]     IDLE       = 0;
   reg [2:0]     READ_CTRL  = 1;
   reg [2:0]     READ_CHNL  = 2;
   reg [2:0]     COMPUTE    = 3;
   reg [2:0]     WRITE_CTRL = 4;
   reg [2:0]     WRITE_CHNL = 5;

   // local variables
   reg [31:0]    beat_ctr;            // beat counter
   reg           configured;
   reg [31:0]    beats_per_pass;      // beats per pass
   reg [31:0]    host_offset;         // offset in memory to data on host
   integer       i;
   reg [31:0]    num_passes;          // number of passes rd/compute/write
   reg [31:0]    pass_ctr;            // pass counter
   reg [31:0]    prog_host_base_addr; // host base address
   reg [31:0]    prog_num_instr_beats;// number of instruction beats in program
   reg [31:0] 	 vals_per_beat;       // number of values per beat for data

   // program and data memory
   reg [31:0]    PGM  [0:HOST_MAX_INSTR-1]; // program memory
   reg [31:0]    DATA [0:HOST_MAX_DATA-1];  // data memory
   reg           pgm_read; // whether the program has been read

   reg [31:0] 	 debug0;
   reg [31:0] 	 debug1;

   // PLM (private local memory) should be implemented as ping pong in cases
   // when accelerator invocation results in several load/compute/store phases.

   initial begin
      debug <= 32'b0;
      host_offset <= HOST_MAX_INSTR/2; // address offset in beats to data
      pgm_read <= 1'b0;
      state <= IDLE;
   end

   always @(posedge clk or negedge rst) begin
      if (!rst) begin 
	 acc_done <= 1'b0;
         // move to IDLE state, keep coming back here while !rst
         // control signals
         dma_read_ctrl_valid <= 1'b0;
         dma_read_chnl_ready <= 1'b0;
         dma_write_ctrl_valid <= 1'b0;
         dma_write_chnl_valid <= 1'b0;
	 dma_write_chnl_data <= DMA_BUS_WIDTH;

         // local variables
         beat_ctr <= 32'b0;
         configured <= 1'b0;
         pgm_read <= 1'b0;
         for (i=0; i<HOST_MAX_INSTR-1; i++) PGM[i]  <= 32'b0;
         for (i=0; i<HOST_MAX_DATA-1; i++)  DATA[i] <= 32'b0;

         // some artificial program stuff that would be calculated
         num_passes <= 32'd2; // number of times through read/compute/write
	 // reading 8 32-bit values per pass, a beat has two values.
         beats_per_pass <= 32'd4; // beats to read each pass.
	 vals_per_beat <= 32'd2;  // values per beat
         pass_ctr <= 32'b0; // initialize pass_ctr

         state <= IDLE;
      end else begin
         case (state)
           IDLE: begin // 000

              if (acc_done) begin
		 acc_done <= 1'b0; // lower acc_done if was set high in state 5
	      end

              state <= IDLE; // default, non-blocking change below.
              if (conf_done && !configured) begin
                 // program read
                 if (!pgm_read) begin
                    prog_host_base_addr <= conf_info_reg0;
                    prog_num_instr_beats <= conf_info_reg1/2; // in beats
                    dma_read_ctrl_data_index <= conf_info_reg0;
                    dma_read_ctrl_data_length <= conf_info_reg1/2; // in beats
                    dma_read_ctrl_data_size <= 3'b010; // 32-bit WORD
                    configured <= 1'b1;

                    // state transition
                    dma_read_ctrl_valid <= 1'b1;
                    state <= READ_CTRL;
                 end
              end else if (pgm_read) begin // if (conf_done && !configured)
                 // tell the host ready to read
                 dma_read_ctrl_data_index // in beats
                   <= prog_host_base_addr
                      + host_offset
                      + pass_ctr*beats_per_pass;
                 dma_read_ctrl_data_length <= beats_per_pass;
                 dma_read_ctrl_data_size <= 3'b010;  // 32-bit words

                 // declare read ctrl valid for host and move to READ_CTRL
                 dma_read_ctrl_valid <= 1'b1;
                 state <= READ_CTRL;
              end
           end // case: IDLE
           READ_CTRL: begin // 001
              // state waits for host read_ctrl_ready and deasserts _valid
              if (dma_read_ctrl_ready) begin // host is ready
                 dma_read_ctrl_valid <= 1'b0;
                 beat_ctr <= 32'b0;
                 state <= READ_CHNL;
              end
           end // case: READ_CTRL
           READ_CHNL: begin // 010
              dma_read_chnl_ready <= 1'b1; // buffers known ahead of time
              if (dma_read_chnl_valid) begin 

                 if (!pgm_read) begin // reading the program
                    PGM[vals_per_beat*beat_ctr] <= dma_read_chnl_data[31:0];
                    debug0 <= dma_read_chnl_data[31:0];
                    PGM[vals_per_beat*beat_ctr + 1]
		      <= dma_read_chnl_data[63:32];
		    debug1 <= dma_read_chnl_data[63:32];
                    beat_ctr <= beat_ctr + 1;
                 end else begin // reading data
                    DATA[pass_ctr*beats_per_pass*vals_per_beat
                	 + vals_per_beat*beat_ctr]
    		            <= dma_read_chnl_data[31:0];
                    debug0 <= dma_read_chnl_data[31:0];
		    DATA[pass_ctr*beats_per_pass*vals_per_beat
                    	 + vals_per_beat*beat_ctr + 1]
		      <= dma_read_chnl_data[63:32];
                    debug1 <= dma_read_chnl_data[63:32];
                    beat_ctr <= beat_ctr + 1;
                 end
                 // mark prgm_read, transition to next state
                 if (!pgm_read && beat_ctr == prog_num_instr_beats - 1) begin
                    pgm_read <= 1'b1;
                    dma_read_chnl_ready <= 1'b0;
                    state <= IDLE;
                 end else if (pgm_read && beat_ctr == beats_per_pass - 1) begin
                    // all data read for this pass, move to compute
                    dma_read_chnl_ready <= 1'b0;
                    beat_ctr <= 32'b0;
                    state <= COMPUTE;
                 end
              end
           end // case: READ_CHNL
           COMPUTE: begin // 011
	      // using beat_ctr, but counting values not beats
              DATA[pass_ctr*beats_per_pass*vals_per_beat+beat_ctr]
                 <= 2*DATA[pass_ctr*beats_per_pass*vals_per_beat + beat_ctr];
	      debug0
                <= DATA[pass_ctr*beats_per_pass*vals_per_beat + beat_ctr];
	      debug1
                <= 2*DATA[pass_ctr*beats_per_pass*vals_per_beat + beat_ctr];
              beat_ctr <= beat_ctr + 1; 

              if (beat_ctr == beats_per_pass*vals_per_beat - 1) begin
                 // set write ctrl values
                 dma_write_ctrl_data_index // in beats
                   <= prog_host_base_addr
		      + host_offset
		      + pass_ctr*beats_per_pass;
                 dma_write_ctrl_data_length <= beats_per_pass;
                 dma_write_ctrl_data_size <= 3'b010;  // 32-bit word

		 // declare write_ctrl_valid and move to WRITE_CTRL
                 dma_write_ctrl_valid <= 1'b1;
                 state <= WRITE_CTRL;
              end
           end // case: COMPUTE
           WRITE_CTRL: begin // 100
              // state waits for host write_ctrl_ready and deasserts _valid
              if (dma_write_ctrl_ready) begin
                 beat_ctr <= 32'b0;
                 dma_write_ctrl_valid <= 1'b0;
                 state <= WRITE_CHNL;
		 dma_write_chnl_data[31:0] // chnl data for beat_ctr == 0
                   <= DATA[pass_ctr*beats_per_pass*vals_per_beat];
		 dma_write_chnl_data[63:32]
                   <= DATA[pass_ctr*beats_per_pass*vals_per_beat + 1];
              end
           end // case: WRITE_CTRL
           WRITE_CHNL: begin // 101
	      dma_write_chnl_valid <= 1'b1;
	      
              if (dma_write_chnl_ready) begin
		 // is it possible that this reads past the end of DATA?
		 dma_write_chnl_data[31:0] // data for next beat
			 <= DATA[pass_ctr*beats_per_pass*vals_per_beat
				 + vals_per_beat*(beat_ctr+1)];
		 dma_write_chnl_data[63:32]
                   <= DATA[pass_ctr*beats_per_pass*vals_per_beat
			   + vals_per_beat*(beat_ctr+1) + 1];
                 beat_ctr <= beat_ctr + 1;
              end

              if (beat_ctr == beats_per_pass - 1) begin
                 pass_ctr <= pass_ctr + 1; // all beats done, increment pass
                 if (pass_ctr == num_passes - 1) begin // passes not done
                    acc_done <= 1'b1;
                 end
		 dma_write_chnl_valid <= 1'b0;
                 state <= IDLE;
	      end
           end
         endcase
      end
   end
endmodule
