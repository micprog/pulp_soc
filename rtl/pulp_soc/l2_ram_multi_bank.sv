// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "soc_mem_map.svh"

module l2_ram_multi_bank #(
   parameter NB_BANKS                   = 4,
   parameter NB_L2_BANKS_PRI            = 2,
   // Don't forget to adjust the SRAM macros and the FPGA settings if you change the banksizes
   parameter int unsigned BANK_SIZE_INTL_SRAM = 32768, //Number of 32-bit words
   parameter int unsigned BANK_SIZE_PRI       = 8192 //Number of 32-bit words
) (
   input logic             clk_i,
   input logic             rst_ni,
   input logic             init_ni,
   input logic             test_mode_i,
   XBAR_TCDM_BUS.Slave     mem_slave[NB_BANKS],
   XBAR_TCDM_BUS.Slave     mem_pri_slave[NB_L2_BANKS_PRI]
);
    //Derived parameters
    localparam int unsigned INTL_MEM_ADDR_WIDTH = $clog2(BANK_SIZE_INTL_SRAM);
    localparam int unsigned PRI_MEM_ADDR_WIDTH  = $clog2(BANK_SIZE_PRI);

    //Used in testbenches



    //INTERLEAVED Memory
    logic [31:0]           interleaved_addresses[NB_BANKS];
    for(genvar i=0; i<NB_BANKS; i++) begin : CUTS
        //Perform TCDM handshaking for constant 1 cycle latency
        assign mem_slave[i].gnt = mem_slave[i].req;
        assign mem_slave[i].r_opc = 1'b0;
        always_ff @(posedge clk_i, negedge rst_ni) begin
            if (!rst_ni) begin
                mem_slave[i].r_valid <= 1'b0;
            end else begin
                mem_slave[i].r_valid <= mem_slave[i].req;
            end
        end
       //Remove Address offset
       assign interleaved_addresses[i] = mem_slave[i].add - `SOC_MEM_MAP_TCDM_START_ADDR;

      `ifndef PULP_FPGA_EMUL
          /*
           This model the hybrid SRAM and SCM configuration
           that has been tape-out.
           */
          generic_memory #(
                           .ADDR_WIDTH ( INTL_MEM_ADDR_WIDTH ),
                           .DATA_WIDTH ( 32                  )
                           ) bank_i (
                                               .CLK   ( clk_i                                             ),
                                               .INITN ( 1'b1                                              ),
                                               .CEN   ( ~mem_slave[i].req                                 ),
                                               .BEN   ( ~mem_slave[i].be                                  ),
                                               .WEN   ( mem_slave[i].wen                                  ),
                                               .A     ( interleaved_addresses[i][INTL_MEM_ADDR_WIDTH+1:2] ), //Convert from
                                                                                                             //byte to word addressing
                                               .D     ( mem_slave[i].wdata                                ),
                                               .Q     ( mem_slave[i].r_rdata                              )
                                               );

      `else // !`ifndef PULP_FPGA_EMUL
          fpga_interleaved_ram #(.ADDR_WIDTH(INTL_MEM_ADDR_WIDTH)) bank_i
              (
               .clk_i,
               .rst_ni,
               .csn_i   (~mem_slave[i].req                                 ),
               .wen_i   (mem_slave[i].wen                                  ),
               .be_i    (mem_slave[i].be                                   ),
               .addr_i  (interleaved_addresses[i][INTL_MEM_ADDR_WIDTH+1:2] ),
               .wdata_i (mem_slave[i].wdata                                ),
               .rdata_o (mem_slave[i].r_rdata                              )
               );
      `endif
   end

   //PRIVATE BANKS
   logic [NB_L2_BANKS_PRI-1:0][31:0] pri_address;
   //Remove Address offset
   assign pri_address[0] = mem_pri_slave[i].add - `SOC_MEM_MAP_PRIVATE_BANK0_START_ADDR;
   assign pri_address[1] = mem_pri_slave[i].add - `SOC_MEM_MAP_PRIVATE_BANK1_START_ADDR;
   assign pri_address[2] = mem_pri_slave[i].add - `SOC_MEM_MAP_PRIVATE_BANK2_START_ADDR;
   for (genvar i = 0; i < NB_L2_BANKS_PRI; i++) begin : PRI_BANKS_GEN
      //Perform TCDM handshaking for constant 1 cycle latency
      assign mem_pri_slave[i].gnt = mem_pri_slave[i].req;
      assign mem_pri_slave[i].r_opc = 1'b0;
      always_ff @(posedge clk_i, negedge rst_ni) begin
          if (!rst_ni) begin
              mem_pri_slave[i].r_valid <= 1'b0;
          end else begin
              mem_pri_slave[i].r_valid <= mem_pri_slave[i].req;
          end
      end
`ifndef PULP_FPGA_EMUL
      generic_memory #(
        .ADDR_WIDTH(PRI_MEM_ADDR_WIDTH),
        .DATA_WIDTH(32)
      ) bank_sram_pri_i (
        .CLK  (clk_i),
        .INITN(1'b1),
        .CEN  (~mem_pri_slave[i].req),
        .BEN  (~mem_pri_slave[i].be),
        .WEN  (mem_pri_slave[i].wen),
        .A    (pri_address[i][PRI_MEM_ADDR_WIDTH+1:2]), //Convert from byte to word addressing
        .D    (mem_pri_slave[i].wdata),
        .Q    (mem_pri_slave[i].r_rdata)
      );
`else
      fpga_private_ram #(
        .ADDR_WIDTH(PRI_MEM_ADDR_WIDTH)
      ) bank_sram_pri0_i (
        .clk_i,
        .rst_ni,
        .csn_i   ( ~mem_pri_slave[i].req                 ),
        .wen_i   ( mem_pri_slave[i].wen                  ),
        .be_i    ( mem_pri_slave[i].be                   ),
        .addr_i  ( pri0_address[PRI_MEM_ADDR_WIDTH+1:2] ), //Convert from byte to word addressing
        .wdata_i ( mem_pri_slave[i].wdata                ),
        .rdata_o ( mem_pri_slave[i].r_rdata              )
        );
`endif

   end

endmodule // l2_ram_multi_bank
