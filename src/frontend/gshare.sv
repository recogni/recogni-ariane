// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Description: GShare branch predictor
// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>

module gshare (
    input  logic                        clk_i,
    input  logic                        rst_ni,
    input  logic                        flush_i,
    input  logic                        debug_mode_i,
    input  logic [63:0]                 vpc_i,
    input  ariane_pkg::bht_update_t     bht_update_i,
    output ariane_pkg::bht_prediction_t bht_prediction_o
);

    localparam HIST_LEN = 52;
    logic [HIST_LEN-1:0] global_history_d, global_history_q;
    logic [63:0] hashed_address;

    // keep a global prediction state
    always_comb begin
        global_history_d = global_history_q;

        // we've got a new valid resolve request
        if (bht_update_i.valid && !debug_mode_i) begin
            global_history_d = {global_history_q[HIST_LEN-2:0], bht_update_i.taken};
        end
    end

    assign hashed_address = vpc_i ^ global_history_q;

    bht #(
        .NR_ENTRIES ( ariane_pkg::BHT_ENTRIES )
    ) i_bht (
        .clk_i            ( clk_i            ),
        .rst_ni           ( rst_ni           ),
        .flush_i          ( flush_i          ),
        .debug_mode_i     ( debug_mode_i     ),
        .vpc_i            ( hashed_address   ),
        .bht_update_i     ( bht_update_i     ),
        .bht_prediction_o ( bht_prediction_o )
    );

    always_ff @(posedge clk_i or negedge rst_ni) begin : proc_
        if (~rst_ni) begin
            global_history_q <= '0;
        end else begin
            global_history_q <= global_history_d;
        end
    end

endmodule