/*
 * maestro_hci_hwpe_interconnect.sv
 * Francesco Conti <f.conti@unibo.it>
 * Tobias Riedener <tobiasri@student.ethz.ch>
 *
 * Copyright (C) 2019-2020 ETH Zurich, University of Bologna
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * The accelerator port expander has the purpose to "expand" the range of
 * the input accelerator ports (NB_IN_CHAN) to a set of output memory ports
 * (NB_OUT_CHAN >= NB_IN_CHAN).
 * It makes several assumptions:
 *  1. All input ports are synchronous. Therefore req, add, wen are taken
 *     only from in[0]. be, wdata are taken from all in ports.
 *     Similarly, all gnt and r_valid signals are propagated from the
 *     first "internal virtual port" (see 3).
 *  2. Required ports on the output are decoded by a proper rotation of the
 *     input ports (virtually expanded with nil ports if NB_OUT_CHAN is
 *     strictly greater than NB_IN_CHAN). This is performed using a
 *     hwpe_stream_tcdm_reorder block.
 */

module maestro_hci_hwpe_interconnect
#(
  parameter int unsigned FIFO_DEPTH  = 0,
  parameter int unsigned NB_OUT_CHAN = 8,
  parameter int unsigned DWH = hci_package::DEFAULT_DW,
  parameter int unsigned DWH_SUB = hci_package::DEFAULT_DW,
  parameter int unsigned AWH = hci_package::DEFAULT_AW,
  parameter int unsigned BWH = hci_package::DEFAULT_BW,
  parameter int unsigned WWH = hci_package::DEFAULT_WW,
  parameter int unsigned OWH = AWH,
  parameter int unsigned UWH = hci_package::DEFAULT_UW, // User Width not yet implemented
  parameter int unsigned AWM = 12
)
(
  input  logic clk_i,
  input  logic rst_ni,
  input  logic clear_i,
  input  logic sel_i,

  hci_core_intf.slave in,
  hci_mem_intf.master out [NB_OUT_CHAN-1:0]
);

  //There is only one input port, but with variable data width.
  //NB_IN_CHAN states, to how many standard (32-bit) ports the input port is equivalent
  localparam NB_IN_CHAN  = DWH / WWH;
  localparam NB_IN_SUBCHAN  = DWH_SUB / WWH;

  //Word-interleaved scheme:
  // - First bits of requested address are shared
  // - Lowest 2 bits are byte offset within a DWORD -> ignored
  // - The bits inbetween designate the selected bank
  localparam LSB_COMMON_ADDR = $clog2(NB_OUT_CHAN) + $clog2(WWH/BWH);
  localparam AWC = AWM+$clog2(NB_OUT_CHAN);
  localparam OutAW = AWC - LSB_COMMON_ADDR + 1;

`ifndef SYNTHESIS
  initial assert (NB_IN_CHAN <= NB_OUT_CHAN)    else  $fatal("NB_IN_CHAN > NB_OUT_CHAN!");
  initial assert (NB_IN_SUBCHAN <= NB_IN_CHAN)  else  $fatal("NB_IN_SUBCHAN > NB_IN_CHAN!");
  initial assert (AWC+2 <= 32)                  else  $fatal("AWM+$clog2(NB_OUT_CHAN)+2 > 32!");
`endif


  logic [$clog2(NB_OUT_CHAN)-1:0] bank_offset_s;
  logic [NB_IN_CHAN-1:0] virt_in_gnt;
  logic [NB_IN_CHAN-1:0] virt_in_rvalid;
  logic                  virt_in_gnt_0_q;

  hci_core_intf #(
    .DW ( DWH ),
    .AW ( AWH ),
    .BW ( BWH ),
    .WW ( WWH ),
    .OW ( OWH ),
    .UW ( UWH )
  ) postfifo (
    .clk ( clk_i )
  );

  // using the interface from hwpe-stream here
  hwpe_stream_intf_tcdm #( .DW (WWH),
                           .AW (OutAW) ) virt_in  [NB_IN_CHAN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_tcdm #( .DW (WWH),
                           .AW (OutAW) ) virt_in_chan  [NB_IN_CHAN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_tcdm #( .DW (WWH),
                           .AW (OutAW) ) virt_in_subchan  [NB_IN_SUBCHAN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_tcdm #( .DW (WWH),
                        .AW (OutAW) ) virt_out [NB_OUT_CHAN-1:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_tcdm #( .DW (WWH),
                        .AW (OutAW) ) virt_out_sub [NB_OUT_CHAN-1:0] (
  .clk ( clk_i )
  );



  // aux signal for r_valid generation
  logic [NB_OUT_CHAN-1:0] out_r_valid;

  // propagate handshake + address only from port 0
  generate

    // FIFOs for HWPE ports
    if(FIFO_DEPTH == 0) begin: no_fifo_gen
      hci_core_assign i_no_fifo (
        .tcdm_slave  ( in            ),
        .tcdm_master ( postfifo      )
      );
    end // no_fifo_gen
    else begin: fifo_gen
      hci_core_fifo #(
        .FIFO_DEPTH ( FIFO_DEPTH ),
        .DW         ( DWH        ),
        .BW         ( BWH        ),
        .AW         ( AWH        ),
        .WW         ( WWH        ),
        .OW         ( OWH        ),
        .UW         ( UWH        )
      ) i_fifo (
        .clk_i       ( clk_i         ),
        .rst_ni      ( rst_ni        ),
        .clear_i     ( clear_i       ),
        .flags_o     (               ),
        .tcdm_slave  ( in            ),
        .tcdm_master ( postfifo      )
      );
    end // fifo_gen

    // unimplemented user bits = 0
    assign postfifo.r_user = '0;

    assign bank_offset_s = postfifo.add[LSB_COMMON_ADDR-1:$clog2(WWH/BWH)];


    for(genvar ii=0; ii<NB_IN_SUBCHAN; ii++) begin : virt_in_sub_bind
      assign virt_in[ii].req   = postfifo.req;
      assign virt_in[ii].wen   = postfifo.wen;
      assign virt_in[ii].be    = postfifo.be[ii*WWH/BWH+WWH/BWH-1:ii*WWH/BWH];
      assign virt_in[ii].data  = postfifo.data[ii*WWH+WWH-1:ii*WWH];
      assign postfifo.r_data[ii*WWH+WWH-1:ii*WWH]  = virt_in[ii].r_data;
      // in a word-interleaved scheme, the internal word-address is given
      // by the highest set of bits in postfifo[0].add, plus the bank-level offset
      always_comb
      begin : address_generation
        if(bank_offset_s + ii >= NB_OUT_CHAN)
          virt_in[ii].add = postfifo.add[AWC:LSB_COMMON_ADDR] + 1;
        else
          virt_in[ii].add = postfifo.add[AWC:LSB_COMMON_ADDR];
      end // address_generation

      assign virt_in_gnt[ii] = virt_in[ii].gnt;
      assign virt_in_rvalid[ii] = virt_in[ii].r_valid;
    end

    for(genvar ii=NB_IN_SUBCHAN; ii<NB_IN_CHAN; ii++) begin : virt_in_bind
      always_comb begin
        if(sel_i) begin
          virt_in[ii].req   = postfifo.req;
          virt_in[ii].wen   = postfifo.wen;
          virt_in[ii].be    = postfifo.be[ii*WWH/BWH+WWH/BWH-1:ii*WWH/BWH];
          virt_in[ii].data  = postfifo.data[ii*WWH+WWH-1:ii*WWH];
          postfifo.r_data[ii*WWH+WWH-1:ii*WWH]  = virt_in[ii].r_data;
          // in a word-interleaved scheme, the internal word-address is given
          // by the highest set of bits in postfifo[0].add, plus the bank-level offset
          if(bank_offset_s + ii >= NB_OUT_CHAN)
            virt_in[ii].add = postfifo.add[AWC:LSB_COMMON_ADDR] + 1;
          else
            virt_in[ii].add = postfifo.add[AWC:LSB_COMMON_ADDR];

      end else begin
          virt_in[ii].req   = '0;
          virt_in[ii].wen   = '0;
          virt_in[ii].be    = '0;
          virt_in[ii].data  = '0;
          postfifo.r_data[ii*WWH+WWH-1:ii*WWH]  = '0;
          virt_in[ii].add = '0;
        end

        virt_in_gnt[ii] = virt_in[ii].gnt;
        virt_in_rvalid[ii] = virt_in[ii].r_valid;
      end
  end

    // register REQ&GNT --> TCDM protocol assumes that (post FIFO)
    // the GNT and R_VALID are exactly asserted in consecutive
    // cycles
    always_ff @(posedge clk_i or negedge rst_ni)
    begin
      if(~rst_ni) begin
        virt_in_gnt_0_q <= '0;
      end
      else if(clear_i) begin
        virt_in_gnt_0_q <= '0;
      end
      else begin
        virt_in_gnt_0_q <= virt_in_gnt[0] & virt_in[0].req;
      end
    end

    // only propagate GNT for those initiators that have asserted REQ
    assign postfifo.gnt     = virt_in_gnt[0] & virt_in[0].req;
    // filter R_VALID with registered GNT
    assign postfifo.r_valid = virt_in_rvalid[0] & virt_in_gnt_0_q;

    for(genvar ii=0; ii<NB_OUT_CHAN; ii++) begin
      always_comb begin
        if (sel_i) begin
          out[ii].req  = virt_out[ii].req;
          out[ii].wen  = virt_out[ii].wen;
          out[ii].be   = virt_out[ii].be;
          out[ii].data = virt_out[ii].data;
          out[ii].add  = virt_out[ii].add;
          virt_out[ii].gnt     = out[ii].gnt;
          virt_out[ii].r_valid = out_r_valid[ii];
          virt_out[ii].r_data  = out[ii].r_data;

          // unimplemented user bits = 0
          out[ii].user = '0;
        end else begin
          out[ii].req  = virt_out_sub[ii].req;
          out[ii].wen  = virt_out_sub[ii].wen;
          out[ii].be   = virt_out_sub[ii].be;
          out[ii].data = virt_out_sub[ii].data;
          out[ii].add  = virt_out_sub[ii].add;
          virt_out_sub[ii].gnt     = out[ii].gnt;
          virt_out_sub[ii].r_valid = out_r_valid[ii];
          virt_out_sub[ii].r_data  = out[ii].r_data;

          // unimplemented user bits = 0
          out[ii].user = '0;
        end
      end
    end

  endgenerate

  for(genvar ii=0; ii<NB_IN_SUBCHAN; ii++) begin
    always_comb begin
      if (sel_i) begin
        virt_in_chan[ii].add  = virt_in[ii].add;
        virt_in_chan[ii].data = virt_in[ii].data;
        virt_in_chan[ii].be   = virt_in[ii].be;
        virt_in_chan[ii].wen  = virt_in[ii].wen;
        virt_in_chan[ii].req  = virt_in[ii].req;
        virt_in[ii].r_data    = virt_in_chan[ii].r_data;
        virt_in[ii].gnt       = virt_in_chan[ii].gnt;
        virt_in[ii].r_valid   = virt_in_chan[ii].r_valid;
        virt_in_subchan[ii].add  = '0;
        virt_in_subchan[ii].data = '0;
        virt_in_subchan[ii].be   = '0;
        virt_in_subchan[ii].wen  = '0;
        virt_in_subchan[ii].req  = '0;
      end else begin
        virt_in_subchan[ii].add  = virt_in[ii].add;
        virt_in_subchan[ii].data = virt_in[ii].data;
        virt_in_subchan[ii].be   = virt_in[ii].be;
        virt_in_subchan[ii].wen  = virt_in[ii].wen;
        virt_in_subchan[ii].req  = virt_in[ii].req;
        virt_in[ii].r_data       = virt_in_subchan[ii].r_data;
        virt_in[ii].gnt          = virt_in_subchan[ii].gnt;
        virt_in[ii].r_valid      = virt_in_subchan[ii].r_valid;
        virt_in_chan[ii].add  = '0;
        virt_in_chan[ii].data = '0;
        virt_in_chan[ii].be   = '0;
        virt_in_chan[ii].wen  = '0;
        virt_in_chan[ii].req  = '0;
      end
    end
  end

  for(genvar ii=NB_IN_SUBCHAN; ii<NB_IN_CHAN; ii++) begin
    always_comb begin
      virt_in_chan[ii].add  = virt_in[ii].add;
      virt_in_chan[ii].data = virt_in[ii].data;
      virt_in_chan[ii].be   = virt_in[ii].be;
      virt_in_chan[ii].wen  = virt_in[ii].wen;
      virt_in_chan[ii].req  = virt_in[ii].req;
      if (sel_i) begin
        virt_in[ii].r_data    = virt_in_chan[ii].r_data;
        virt_in[ii].gnt     = virt_in_chan[ii].gnt;
        virt_in[ii].r_valid = virt_in_chan[ii].r_valid;
      end else begin
        virt_in[ii].r_data    = 1'b0;
        virt_in[ii].gnt       = 1'b0;
        virt_in[ii].r_valid   = 1'b0;
      end
    end
  end


  //Re-order the interfaces such that the port requesting the lowest bits of data
  //are located at the correct bank offset
  hci_hwpe_reorder #(
    .NB_IN_CHAN  ( NB_IN_CHAN  ),
    .NB_OUT_CHAN ( NB_OUT_CHAN ),
    .DW          ( WWH         )
  ) i_reorder (
    .clk_i   ( clk_i           ),
    .rst_ni  ( rst_ni          ),
    .clear_i ( clear_i         ),
    .order_i ( bank_offset_s   ),
    .in      ( virt_in_chan         ),
    .out     ( virt_out        )
    );

  hci_hwpe_reorder #(
  .NB_IN_CHAN  ( NB_IN_SUBCHAN ),
  .NB_OUT_CHAN ( NB_OUT_CHAN   ),
  .DW          ( WWH           )
  ) i_reorder_sub (
  .clk_i   ( clk_i                      ),
  .rst_ni  ( rst_ni                     ),
  .clear_i ( clear_i                    ),
  .order_i ( bank_offset_s              ),
  .in      ( virt_in_subchan         ),
  .out     ( virt_out_sub               )
  );

endmodule // maestro_hci_hwpe_interconnect
