// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba    <zarubaf@iis.ee.ethz.ch>, ETH Zurich
//         Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 15.08.2018
// Description: SRAM wrapper for FPGA (requires the fpga-support submodule)
//
// Note: the wrapped module contains two different implementations for
// ALTERA and XILINX tools, since these follow different coding styles for
// inferrable RAMS with byte enable. define `FPGA_TARGET_XILINX or
// `FPGA_TARGET_ALTERA in your build environment (default is ALTERA)

module sram_foo #(
    parameter DATA_WIDTH = 64,
    parameter NUM_WORDS  = 1024,
    parameter OUT_REGS   = 0    // enables output registers in FPGA macro (read lat = 2)
)(
   input  logic                          clk_i,
   input  logic                          rst_ni,
   input  logic                          req_i,
   input  logic                          we_i,
   input  logic [$clog2(NUM_WORDS)-1:0]  addr_i,
   input  logic [DATA_WIDTH-1:0]         wdata_i,
   input  logic [(DATA_WIDTH+7)/8-1:0]   be_i,
   output logic [DATA_WIDTH-1:0]         rdata_o
);

localparam DATA_WIDTH_ALIGNED = ((DATA_WIDTH+63)/64)*64;
localparam BE_WIDTH_ALIGNED   = (((DATA_WIDTH+7)/8+7)/8)*8;

logic [DATA_WIDTH_ALIGNED-1:0]  wdata_aligned;
logic [BE_WIDTH_ALIGNED-1:0]    be_aligned;
logic [DATA_WIDTH_ALIGNED-1:0]  rdata_aligned;


logic [ (((DATA_WIDTH+7)/8)*8) -1:0] sram [0:NUM_WORDS-1];

genvar i;

localparam NUM_BYTES = (DATA_WIDTH+7)/8;

logic [DATA_WIDTH-1:0] bit_enable;

for (i=0; i<DATA_WIDTH;i=i+1) begin
    assign bit_enable[i] = be_i[i/8];
end

always @(posedge clk_i) begin
    if ( (rst_ni == 1) && (we_i) && (req_i) ) begin
        for (integer j=0; j<DATA_WIDTH;j=j+1) begin
            if (bit_enable[j]) begin
                 sram[addr_i][j] <= wdata_i[j];
            end
        end
    end

    if ( (rst_ni == 1) && (!we_i) && (req_i) ) begin
        rdata_o <= sram[addr_i];
    end

end


endmodule : sram_foo
