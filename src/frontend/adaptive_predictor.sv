// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Description: Two-Level Adaptive predictor
// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>

module adaptive_predictor (
    input  logic                        clk_i,
    input  logic                        rst_ni,
    input  logic                        flush_i,
    input  logic                        debug_mode_i,
    input  logic [63:0]                 vpc_i,
    input  ariane_pkg::bht_update_t     bht_update_i,
    output ariane_pkg::bht_prediction_t bht_prediction_o
);

    localparam CORR_REG_ENTRIES = 512;
    localparam CORR_REG_WIDTH   = 10;
    // global pattern table entries
    localparam GPT_ENTRIES      = 2 ** CORR_REG_WIDTH;

    ariane_pkg::bht_update_t bht_update;

    logic [63:0] predict_address;
    logic [63:0] update_address;

    logic [CORR_REG_ENTRIES-1:0][CORR_REG_WIDTH-1:0] mem_d, mem_q;

    always_comb begin
        mem_d = mem_q;
        update_address = '0;
        predict_address = '0;

        // do the lookup in the correlation register
        predict_address = mem_q[vpc_i[$clog2(CORR_REG_ENTRIES):1]];

        // update correlation register
        if (bht_update_i.valid && !debug_mode_i) begin
            update_address = bht_update_i.pc[$clog2(CORR_REG_ENTRIES):1];
            mem_d[update_address] = {mem_q[update_address][CORR_REG_WIDTH-2:0], bht_update_i.taken};
        end
    end

    assign bht_update.valid = bht_update_i.valid;
    assign bht_update.pc = mem_q[update_address];
    assign bht_update.mispredict = bht_update_i.mispredict;
    assign bht_update.taken = bht_update_i.taken;

    // global pattern table
    bht #(
        .NR_ENTRIES ( GPT_ENTRIES )
    ) i_gpt (
        .clk_i            ( clk_i            ),
        .rst_ni           ( rst_ni           ),
        .flush_i          ( flush_i          ),
        .debug_mode_i     ( debug_mode_i     ),
        .vpc_i            ( predict_address  ),
        .bht_update_i     ( bht_update       ),
        .bht_prediction_o ( bht_prediction_o )
    );


    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (~rst_ni) begin
            mem_q <= '0;
        end else begin
            mem_q <= mem_d;
        end
    end
endmodule