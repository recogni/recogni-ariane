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
// Author: Florian Zaruba, ETH Zurich
// Date: 19.03.2017
// Description: Test-harness for Ariane
//              Instantiates an AXI-Bus and memories

module ariane_testharness #(
    parameter int unsigned AXI_ID_WIDTH      = 4,
    parameter int unsigned AXI_USER_WIDTH    = 1,
    parameter int unsigned AXI_ADDRESS_WIDTH = 64,
    parameter int unsigned AXI_DATA_WIDTH    = 64,
    parameter bit          InclSimDTM        = 1'b1,
    parameter int unsigned NUM_WORDS         = 2**25,         // memory size
    parameter bit          StallRandomOutput = 1'b0,
    parameter bit          StallRandomInput  = 1'b0
) (
    input  logic                           clk_i,
    input  logic                           rtc_i,
    input  logic                           rst_ni,
    output logic [31:0]                    exit_o
);

    // disable test-enable
    logic        test_en;
    logic        ndmreset;
    logic        ndmreset_n;
    logic        debug_req_core;

    int          jtag_enable;
    logic        init_done;
    logic [31:0] jtag_exit, dmi_exit;

    logic        jtag_TCK;
    logic        jtag_TMS;
    logic        jtag_TDI;
    logic        jtag_TRSTn;
    logic        jtag_TDO_data;
    logic        jtag_TDO_driven;

    logic        debug_req_valid;
    logic        debug_req_ready;
    logic        debug_resp_valid;
    logic        debug_resp_ready;

    logic        jtag_req_valid;
    logic [6:0]  jtag_req_bits_addr;
    logic [1:0]  jtag_req_bits_op;
    logic [31:0] jtag_req_bits_data;
    logic        jtag_resp_ready;
    logic        jtag_resp_valid;

    logic        dmi_req_valid;
    logic        dmi_resp_ready;
    logic        dmi_resp_valid;

    dm::dmi_req_t  jtag_dmi_req;
    dm::dmi_req_t  dmi_req;

    dm::dmi_req_t  debug_req;
    dm::dmi_resp_t debug_resp;

    assign test_en = 1'b0;

    localparam NB_SLAVE = 2;

    localparam AXI_ID_WIDTH_SLAVES = AXI_ID_WIDTH + $clog2(NB_SLAVE);

    localparam AXI_BUFF_WIDTH_ASYNC = 8;

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH    ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH      ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH    )
    ) slave[NB_SLAVE-1:0]();

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH_SLAVES ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH      )
    ) master[ariane_soc::NB_PERIPHERALS-1:0]();

    AXI_BUS_ASYNC #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH    ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH       ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH_SLAVES  ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH       ),
        .BUFFER_WIDTH   ( AXI_BUFF_WIDTH_ASYNC )
    ) master_pulp();

    rstgen i_rstgen_main (
        .clk_i        ( clk_i                ),
        .rst_ni       ( rst_ni & (~ndmreset) ),
        .test_mode_i  ( test_en              ),
        .rst_no       ( ndmreset_n           ),
        .init_no      (                      ) // keep open
    );

    // ---------------
    // Debug
    // ---------------
    assign init_done = rst_ni;

    initial begin
        if (!$value$plusargs("jtag_rbb_enable=%b", jtag_enable)) jtag_enable = 'h0;
    end

    // debug if MUX
    assign debug_req_valid     = (jtag_enable[0]) ? jtag_req_valid     : dmi_req_valid;
    assign debug_resp_ready    = (jtag_enable[0]) ? jtag_resp_ready    : dmi_resp_ready;
    assign debug_req           = (jtag_enable[0]) ? jtag_dmi_req       : dmi_req;
    assign exit_o              = (jtag_enable[0]) ? jtag_exit          : dmi_exit;
    assign jtag_resp_valid     = (jtag_enable[0]) ? debug_resp_valid   : 1'b0;
    assign dmi_resp_valid      = (jtag_enable[0]) ? 1'b0               : debug_resp_valid;

    // SiFive's SimJTAG Module
    // Converts to DPI calls
    SimJTAG i_SimJTAG (
        .clock                ( clk_i                ),
        .reset                ( ~rst_ni              ),
        .enable               ( jtag_enable[0]       ),
        .init_done            ( init_done            ),
        .jtag_TCK             ( jtag_TCK             ),
        .jtag_TMS             ( jtag_TMS             ),
        .jtag_TDI             ( jtag_TDI             ),
        .jtag_TRSTn           ( jtag_TRSTn           ),
        .jtag_TDO_data        ( jtag_TDO_data        ),
        .jtag_TDO_driven      ( jtag_TDO_driven      ),
        .exit                 ( jtag_exit            )
    );

    dmi_jtag i_dmi_jtag (
        .clk_i            ( clk_i           ),
        .rst_ni           ( rst_ni          ),
        .testmode_i       ( test_en         ),
        .dmi_req_o        ( jtag_dmi_req    ),
        .dmi_req_valid_o  ( jtag_req_valid  ),
        .dmi_req_ready_i  ( debug_req_ready ),
        .dmi_resp_i       ( debug_resp      ),
        .dmi_resp_ready_o ( jtag_resp_ready ),
        .dmi_resp_valid_i ( jtag_resp_valid ),
        .dmi_rst_no       (                 ), // not connected
        .tck_i            ( jtag_TCK        ),
        .tms_i            ( jtag_TMS        ),
        .trst_ni          ( jtag_TRSTn      ),
        .td_i             ( jtag_TDI        ),
        .td_o             ( jtag_TDO_data   ),
        .tdo_oe_o         ( jtag_TDO_driven )
    );

    // SiFive's SimDTM Module
    // Converts to DPI calls
    logic [1:0] debug_req_bits_op;
    assign dmi_req.op = dm::dtm_op_t'(debug_req_bits_op);

    if (InclSimDTM) begin
        SimDTM i_SimDTM (
            .clk                  ( clk_i                ),
            .reset                ( ~rst_ni              ),
            .debug_req_valid      ( dmi_req_valid        ),
            .debug_req_ready      ( debug_req_ready      ),
            .debug_req_bits_addr  ( dmi_req.addr         ),
            .debug_req_bits_op    ( debug_req_bits_op    ),
            .debug_req_bits_data  ( dmi_req.data         ),
            .debug_resp_valid     ( dmi_resp_valid       ),
            .debug_resp_ready     ( dmi_resp_ready       ),
            .debug_resp_bits_resp ( debug_resp.resp      ),
            .debug_resp_bits_data ( debug_resp.data      ),
            .exit                 ( dmi_exit             )
        );
    end else begin
        assign dmi_req_valid = '0;
        assign debug_req_bits_op = '0;
        assign dmi_exit = 1'b0;
    end

    ariane_axi::req_t    dm_axi_m_req,  dm_axi_s_req;
    ariane_axi::resp_t   dm_axi_m_resp, dm_axi_s_resp;

    // debug module
    dm_top #(
        // current implementation only supports 1 hart
        .NrHarts              ( 1                         ),
        .AxiIdWidth           ( AXI_ID_WIDTH_SLAVES       ),
        .AxiAddrWidth         ( AXI_ADDRESS_WIDTH         ),
        .AxiDataWidth         ( AXI_DATA_WIDTH            ),
        .AxiUserWidth         ( AXI_USER_WIDTH            )
    ) i_dm_top (

        .clk_i                ( clk_i                ),
        .rst_ni               ( rst_ni               ), // PoR
        .testmode_i           ( test_en              ),
        .ndmreset_o           ( ndmreset             ),
        .dmactive_o           (                      ), // active debug session
        .debug_req_o          ( debug_req_core       ),
        .unavailable_i        ( '0                   ),
        .axi_s_req_i          ( dm_axi_s_req         ),
        .axi_s_resp_o         ( dm_axi_s_resp        ),
        .axi_m_req_o          ( dm_axi_m_req         ),
        .axi_m_resp_i         ( dm_axi_m_resp        ),
        .dmi_rst_ni           ( rst_ni               ),
        .dmi_req_valid_i      ( debug_req_valid      ),
        .dmi_req_ready_o      ( debug_req_ready      ),
        .dmi_req_i            ( debug_req            ),
        .dmi_resp_valid_o     ( debug_resp_valid     ),
        .dmi_resp_ready_i     ( debug_resp_ready     ),
        .dmi_resp_o           ( debug_resp           )
    );

    axi_master_connect i_axi_master_dm (.axi_req_i(dm_axi_m_req), .axi_resp_o(dm_axi_m_resp), .master(slave[1]));
    axi_slave_connect  i_axi_slave_dm  (.axi_req_o(dm_axi_s_req), .axi_resp_i(dm_axi_s_resp), .slave(master[ariane_soc::Debug]));


    // ---------------
    // ROM
    // ---------------
    logic                         rom_req;
    logic [AXI_ADDRESS_WIDTH-1:0] rom_addr;
    logic [AXI_DATA_WIDTH-1:0]    rom_rdata;

    axi2mem #(
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH_SLAVES ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH      )
    ) i_axi2rom (
        .clk_i  ( clk_i                   ),
        .rst_ni ( ndmreset_n              ),
        .slave  ( master[ariane_soc::ROM] ),
        .req_o  ( rom_req                 ),
        .we_o   (                         ),
        .addr_o ( rom_addr                ),
        .be_o   (                         ),
        .data_o (                         ),
        .data_i ( rom_rdata               )
    );

    bootrom i_bootrom (
        .clk_i      ( clk_i     ),
        .req_i      ( rom_req   ),
        .addr_i     ( rom_addr  ),
        .rdata_o    ( rom_rdata )
    );

    // ---------------
    // Memory
    // ---------------

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH_SLAVES ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH      )
    ) dram();

    logic                         req;
    logic                         we;
    logic [AXI_ADDRESS_WIDTH-1:0] addr;
    logic [AXI_DATA_WIDTH/8-1:0]  be;
    logic [AXI_DATA_WIDTH-1:0]    wdata;
    logic [AXI_DATA_WIDTH-1:0]    rdata;

    axi_pkg::aw_chan_t aw_chan_i;
    axi_pkg::w_chan_t  w_chan_i;
    axi_pkg::b_chan_t  b_chan_o;
    axi_pkg::ar_chan_t ar_chan_i;
    axi_pkg::r_chan_t  r_chan_o;
    axi_pkg::aw_chan_t aw_chan_o;
    axi_pkg::w_chan_t  w_chan_o;
    axi_pkg::b_chan_t  b_chan_i;
    axi_pkg::ar_chan_t ar_chan_o;
    axi_pkg::r_chan_t  r_chan_i;

    axi_delayer #(
        .aw_t              ( axi_pkg::aw_chan_t ),
        .w_t               ( axi_pkg::w_chan_t  ),
        .b_t               ( axi_pkg::b_chan_t  ),
        .ar_t              ( axi_pkg::ar_chan_t ),
        .r_t               ( axi_pkg::r_chan_t  ),
        .StallRandomOutput ( StallRandomOutput  ),
        .StallRandomInput  ( StallRandomInput   ),
        .FixedDelayInput   ( 0                  ),
        .FixedDelayOutput  ( 0                  )
    ) i_axi_delayer (
        .clk_i      ( clk_i                             ),
        .rst_ni     ( ndmreset_n                        ),
        .aw_valid_i ( master[ariane_soc::DRAM].aw_valid ),
        .aw_chan_i  ( aw_chan_i                         ),
        .aw_ready_o ( master[ariane_soc::DRAM].aw_ready ),
        .w_valid_i  ( master[ariane_soc::DRAM].w_valid  ),
        .w_chan_i   ( w_chan_i                          ),
        .w_ready_o  ( master[ariane_soc::DRAM].w_ready  ),
        .b_valid_o  ( master[ariane_soc::DRAM].b_valid  ),
        .b_chan_o   ( b_chan_o                          ),
        .b_ready_i  ( master[ariane_soc::DRAM].b_ready  ),
        .ar_valid_i ( master[ariane_soc::DRAM].ar_valid ),
        .ar_chan_i  ( ar_chan_i                         ),
        .ar_ready_o ( master[ariane_soc::DRAM].ar_ready ),
        .r_valid_o  ( master[ariane_soc::DRAM].r_valid  ),
        .r_chan_o   ( r_chan_o                          ),
        .r_ready_i  ( master[ariane_soc::DRAM].r_ready  ),
        .aw_valid_o ( dram.aw_valid                     ),
        .aw_chan_o  ( aw_chan_o                         ),
        .aw_ready_i ( dram.aw_ready                     ),
        .w_valid_o  ( dram.w_valid                      ),
        .w_chan_o   ( w_chan_o                          ),
        .w_ready_i  ( dram.w_ready                      ),
        .b_valid_i  ( dram.b_valid                      ),
        .b_chan_i   ( b_chan_i                          ),
        .b_ready_o  ( dram.b_ready                      ),
        .ar_valid_o ( dram.ar_valid                     ),
        .ar_chan_o  ( ar_chan_o                         ),
        .ar_ready_i ( dram.ar_ready                     ),
        .r_valid_i  ( dram.r_valid                      ),
        .r_chan_i   ( r_chan_i                          ),
        .r_ready_o  ( dram.r_ready                      )
    );

    assign aw_chan_i.atop = '0;
    assign aw_chan_i.id = master[ariane_soc::DRAM].aw_id;
    assign aw_chan_i.addr = master[ariane_soc::DRAM].aw_addr;
    assign aw_chan_i.len = master[ariane_soc::DRAM].aw_len;
    assign aw_chan_i.size = master[ariane_soc::DRAM].aw_size;
    assign aw_chan_i.burst = master[ariane_soc::DRAM].aw_burst;
    assign aw_chan_i.lock = master[ariane_soc::DRAM].aw_lock;
    assign aw_chan_i.cache = master[ariane_soc::DRAM].aw_cache;
    assign aw_chan_i.prot = master[ariane_soc::DRAM].aw_prot;
    assign aw_chan_i.qos = master[ariane_soc::DRAM].aw_qos;
    assign aw_chan_i.region = master[ariane_soc::DRAM].aw_region;

    assign ar_chan_i.id = master[ariane_soc::DRAM].ar_id;
    assign ar_chan_i.addr = master[ariane_soc::DRAM].ar_addr;
    assign ar_chan_i.len = master[ariane_soc::DRAM].ar_len;
    assign ar_chan_i.size = master[ariane_soc::DRAM].ar_size;
    assign ar_chan_i.burst = master[ariane_soc::DRAM].ar_burst;
    assign ar_chan_i.lock = master[ariane_soc::DRAM].ar_lock;
    assign ar_chan_i.cache = master[ariane_soc::DRAM].ar_cache;
    assign ar_chan_i.prot = master[ariane_soc::DRAM].ar_prot;
    assign ar_chan_i.qos = master[ariane_soc::DRAM].ar_qos;
    assign ar_chan_i.region = master[ariane_soc::DRAM].ar_region;

    assign w_chan_i.data = master[ariane_soc::DRAM].w_data;
    assign w_chan_i.strb = master[ariane_soc::DRAM].w_strb;
    assign w_chan_i.last = master[ariane_soc::DRAM].w_last;

    assign master[ariane_soc::DRAM].r_id = r_chan_o.id;
    assign master[ariane_soc::DRAM].r_data = r_chan_o.data;
    assign master[ariane_soc::DRAM].r_resp = r_chan_o.resp;
    assign master[ariane_soc::DRAM].r_last = r_chan_o.last;

    assign master[ariane_soc::DRAM].b_id = b_chan_o.id;
    assign master[ariane_soc::DRAM].b_resp = b_chan_o.resp;


    assign dram.aw_id = aw_chan_o.id;
    assign dram.aw_addr = aw_chan_o.addr;
    assign dram.aw_len = aw_chan_o.len;
    assign dram.aw_size = aw_chan_o.size;
    assign dram.aw_burst = aw_chan_o.burst;
    assign dram.aw_lock = aw_chan_o.lock;
    assign dram.aw_cache = aw_chan_o.cache;
    assign dram.aw_prot = aw_chan_o.prot;
    assign dram.aw_qos = aw_chan_o.qos;
    assign dram.aw_region = aw_chan_o.region;
    assign dram.aw_user = master[ariane_soc::DRAM].aw_user;

    assign dram.ar_id = ar_chan_o.id;
    assign dram.ar_addr = ar_chan_o.addr;
    assign dram.ar_len = ar_chan_o.len;
    assign dram.ar_size = ar_chan_o.size;
    assign dram.ar_burst = ar_chan_o.burst;
    assign dram.ar_lock = ar_chan_o.lock;
    assign dram.ar_cache = ar_chan_o.cache;
    assign dram.ar_prot = ar_chan_o.prot;
    assign dram.ar_qos = ar_chan_o.qos;
    assign dram.ar_region = ar_chan_o.region;
    assign dram.ar_user = master[ariane_soc::DRAM].ar_user;

    assign dram.w_data = w_chan_o.data;
    assign dram.w_strb = w_chan_o.strb;
    assign dram.w_last = w_chan_o.last;
    assign dram.w_user = master[ariane_soc::DRAM].w_user;

    assign r_chan_i.id = dram.r_id;
    assign r_chan_i.data = dram.r_data;
    assign r_chan_i.resp = dram.r_resp;
    assign r_chan_i.last = dram.r_last;
    assign master[ariane_soc::DRAM].r_user = dram.r_user;

    assign b_chan_i.id = dram.b_id;
    assign b_chan_i.resp = dram.b_resp;
    assign master[ariane_soc::DRAM].b_user = dram.b_user;


    axi2mem #(
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH_SLAVES ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH      )
    ) i_axi2mem (
        .clk_i  ( clk_i      ),
        .rst_ni ( ndmreset_n ),
        .slave  ( dram       ),
        .req_o  ( req        ),
        .we_o   ( we         ),
        .addr_o ( addr       ),
        .be_o   ( be         ),
        .data_o ( wdata      ),
        .data_i ( rdata      )
    );

    sram #(
        .DATA_WIDTH ( AXI_DATA_WIDTH ),
        .NUM_WORDS  ( NUM_WORDS      )
    ) i_sram (
        .clk_i      ( clk_i                                                                       ),
        .rst_ni     ( rst_ni                                                                      ),
        .req_i      ( req                                                                         ),
        .we_i       ( we                                                                          ),
        .addr_i     ( addr[$clog2(NUM_WORDS)-1+$clog2(AXI_DATA_WIDTH/8):$clog2(AXI_DATA_WIDTH/8)] ),
        .wdata_i    ( wdata                                                                       ),
        .be_i       ( be                                                                          ),
        .rdata_o    ( rdata                                                                       )
    );

    // ---------------
    // AXI Xbar
    // ---------------
    axi_node_intf_wrap #(
        .NB_SLAVE           ( NB_SLAVE                   ),
        .NB_MASTER          ( ariane_soc::NB_PERIPHERALS ),
        .AXI_ADDR_WIDTH     ( AXI_ADDRESS_WIDTH          ),
        .AXI_DATA_WIDTH     ( AXI_DATA_WIDTH             ),
        .AXI_USER_WIDTH     ( AXI_USER_WIDTH             ),
        .AXI_ID_WIDTH       ( AXI_ID_WIDTH               )
        // .MASTER_SLICE_DEPTH ( 0                          ),
        // .SLAVE_SLICE_DEPTH  ( 0                          )
    ) i_axi_xbar (
        .clk          ( clk_i      ),
        .rst_n        ( ndmreset_n ),
        .test_en_i    ( test_en    ),
        .slave        ( slave      ),
        .master       ( master     ),
        .start_addr_i ({
            ariane_soc::DebugBase,
            ariane_soc::ROMBase,
            ariane_soc::CLINTBase,
            ariane_soc::PLICBase,
            ariane_soc::GPIOBase,
            ariane_soc::PULPBase,
            ariane_soc::EthernetBase,
            ariane_soc::GPIOBase,
            ariane_soc::DRAMBase,
            ariane_soc::UARTBase
        }),
        .end_addr_i   ({
            ariane_soc::DebugBase    + ariane_soc::DebugLength - 1,
            ariane_soc::ROMBase      + ariane_soc::ROMLength - 1,
            ariane_soc::CLINTBase    + ariane_soc::CLINTLength - 1,
            ariane_soc::PLICBase     + ariane_soc::PLICLength - 1,
            ariane_soc::PULPBase     + ariane_soc::PULPLenght - 1,
            ariane_soc::SPIBase      + ariane_soc::SPILength - 1,
            ariane_soc::EthernetBase + ariane_soc::EthernetLength -1,
            ariane_soc::GPIOBase     + ariane_soc::GPIOLength - 1,
            ariane_soc::DRAMBase     + ariane_soc::DRAMLength - 1,
            ariane_soc::UARTBase     + ariane_soc::UARTLength - 1
        })
    );

    // ---------------
    // CLINT
    // ---------------
    logic ipi;
    logic timer_irq;

    ariane_axi::req_t    axi_clint_req;
    ariane_axi::resp_t   axi_clint_resp;

    clint #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
        .AXI_ID_WIDTH   ( AXI_ID_WIDTH_SLAVES ),
        .NR_CORES       ( 1                   )
    ) i_clint (
        .clk_i       ( clk_i          ),
        .rst_ni      ( ndmreset_n     ),
        .testmode_i  ( test_en        ),
        .axi_req_i   ( axi_clint_req  ),
        .axi_resp_o  ( axi_clint_resp ),
        .rtc_i       ( rtc_i          ),
        .timer_irq_o ( timer_irq      ),
        .ipi_o       ( ipi            )
    );

    axi_slave_connect i_axi_slave_connect_clint (.axi_req_o(axi_clint_req), .axi_resp_i(axi_clint_resp), .slave(master[ariane_soc::CLINT]));

    // ---------------
    // Peripherals
    // ---------------
    logic tx, rx;
    logic [1:0] irqs;

    ariane_peripherals #(
      .AxiAddrWidth ( AXI_ADDRESS_WIDTH   ),
      .AxiDataWidth ( AXI_DATA_WIDTH      ),
      .AxiIdWidth   ( AXI_ID_WIDTH_SLAVES ),
      .InclUART     ( 1'b0                ),
      .InclSPI      ( 1'b0                ),
      .InclEthernet ( 1'b0                )
    ) i_ariane_peripherals (
      .clk_i     ( clk_i                        ),
      .rst_ni    ( ndmreset_n                   ),
      .plic      ( master[ariane_soc::PLIC]     ),
      .uart      ( master[ariane_soc::UART]     ),
      .spi       ( master[ariane_soc::SPI]      ),
      .ethernet  ( master[ariane_soc::Ethernet] ),
      .irq_o     ( irqs                         ),
      .rx_i      ( rx                           ),
      .tx_o      ( tx                           ),
      .eth_txck  ( ),
      .eth_rxck  ( ),
      .eth_rxctl ( ),
      .eth_rxd   ( ),
      .eth_rst_n ( ),
      .eth_tx_en ( ),
      .eth_txd   ( ),
      .phy_mdio  ( ),
      .eth_mdc   ( ),
      .mdio      ( ),
      .mdc       ( ),
      .spi_clk_o ( ),
      .spi_mosi  ( ),
      .spi_miso  ( ),
      .spi_ss    ( )
    );

    uart_bus #(.BAUD_RATE(115200), .PARITY_EN(0)) i_uart_bus (.rx(tx), .tx(rx), .rx_en(1'b1));

    // ---------------
    // Core
    // ---------------
    ariane_axi::req_t    axi_ariane_req;
    ariane_axi::resp_t   axi_ariane_resp;

    ariane #(
`ifdef PITON_ARIANE
        .SwapEndianess ( 0                                               ),
        .CachedAddrEnd ( (ariane_soc::DRAMBase + ariane_soc::DRAMLength) ),
`endif
        .CachedAddrBeg ( ariane_soc::DRAMBase )
    ) i_ariane (
        .clk_i                ( clk_i               ),
        .rst_ni               ( ndmreset_n          ),
        .boot_addr_i          ( ariane_soc::ROMBase ), // start fetching from ROM
        .hart_id_i            ( '0                  ),
        .irq_i                ( irqs                ),
        .ipi_i                ( ipi                 ),
        .time_irq_i           ( timer_irq           ),
        .debug_req_i          ( debug_req_core      ),
        .axi_req_o            ( axi_ariane_req      ),
        .axi_resp_i           ( axi_ariane_resp     )
    );

    axi_master_connect i_axi_master_connect_ariane (.axi_req_i(axi_ariane_req), .axi_resp_o(axi_ariane_resp), .master(slave[0]));




/*
    axi_slice_dc_slave
    #(
        .AXI_ADDR_WIDTH              ( AXI_ADDRESS_WIDTH                 ),
        .AXI_DATA_WIDTH              ( AXI_DATA_WIDTH                    ),
        .AXI_USER_WIDTH              ( AXI_USER_WIDTH                    ),
        .AXI_ID_WIDTH                ( AXI_ID_WIDTH_SLAVES               ),
        .BUFFER_WIDTH                ( AXI_BUFF_WIDTH_ASYNC              )
    ) i_axi_slice_dc_slave
    (
        .clk_i                       ( clk_i                             ),
        .rst_ni                      ( ndmreset_n                        ),

        .test_cgbypass_i             ( 1'b0                              ),

        .axi_slave_aw_valid          ( master[ariane_soc::PULP].aw_valid ),
        .axi_slave_aw_addr           ( master[ariane_soc::PULP].aw_addr  ),
        .axi_slave_aw_prot           ( master[ariane_soc::PULP].aw_prot  ),
        .axi_slave_aw_region         ( master[ariane_soc::PULP].aw_region),
        .axi_slave_aw_len            ( master[ariane_soc::PULP].aw_len   ),
        .axi_slave_aw_size           ( master[ariane_soc::PULP].aw_size  ),
        .axi_slave_aw_burst          ( master[ariane_soc::PULP].aw_burst ),
        .axi_slave_aw_lock           ( master[ariane_soc::PULP].aw_lock  ),
        .axi_slave_aw_cache          ( master[ariane_soc::PULP].aw_cache ),
        .axi_slave_aw_qos            ( master[ariane_soc::PULP].aw_qos   ),
        .axi_slave_aw_id             ( master[ariane_soc::PULP].aw_id    ),
        .axi_slave_aw_user           ( master[ariane_soc::PULP].aw_user  ),
        .axi_slave_aw_ready          ( master[ariane_soc::PULP].aw_ready ),

        // READ ADDRESS CHANNEL
        .axi_slave_ar_valid          ( master[ariane_soc::PULP].ar_valid ),
        .axi_slave_ar_addr           ( master[ariane_soc::PULP].ar_addr  ),
        .axi_slave_ar_prot           ( master[ariane_soc::PULP].ar_prot  ),
        .axi_slave_ar_region         ( master[ariane_soc::PULP].ar_region),
        .axi_slave_ar_len            ( master[ariane_soc::PULP].ar_len   ),
        .axi_slave_ar_size           ( master[ariane_soc::PULP].ar_size  ),
        .axi_slave_ar_burst          ( master[ariane_soc::PULP].ar_burst ),
        .axi_slave_ar_lock           ( master[ariane_soc::PULP].ar_lock  ),
        .axi_slave_ar_cache          ( master[ariane_soc::PULP].ar_cache ),
        .axi_slave_ar_qos            ( master[ariane_soc::PULP].ar_qos   ),
        .axi_slave_ar_id             ( master[ariane_soc::PULP].ar_id    ),
        .axi_slave_ar_user           ( master[ariane_soc::PULP].ar_user  ),
        .axi_slave_ar_ready          ( master[ariane_soc::PULP].ar_ready ),

        // WRITE DATA CHANNEL
        .axi_slave_w_valid           ( master[ariane_soc::PULP].w_valid  ),
        .axi_slave_w_data            ( master[ariane_soc::PULP].w_data   ),
        .axi_slave_w_strb            ( master[ariane_soc::PULP].w_strb   ),
        .axi_slave_w_user            ( master[ariane_soc::PULP].w_user   ),
        .axi_slave_w_last            ( master[ariane_soc::PULP].w_last   ),
        .axi_slave_w_ready           ( master[ariane_soc::PULP].w_ready  ),

        // READ DATA CHANNEL
        .axi_slave_r_valid           ( master[ariane_soc::PULP].r_valid  ) ,
        .axi_slave_r_data            ( master[ariane_soc::PULP].r_data   ) ,
        .axi_slave_r_resp            ( master[ariane_soc::PULP].r_resp   ) ,
        .axi_slave_r_last            ( master[ariane_soc::PULP].r_last   ) ,
        .axi_slave_r_id              ( master[ariane_soc::PULP].r_id     ) ,
        .axi_slave_r_user            ( master[ariane_soc::PULP].r_user   ) ,
        .axi_slave_r_ready           ( master[ariane_soc::PULP].r_ready  ) ,

        // WRITE RESPONSE CHANNEL
        .axi_slave_b_valid           ( master[ariane_soc::PULP].b_valid   ),
        .axi_slave_b_resp            ( master[ariane_soc::PULP].b_resp    ),
        .axi_slave_b_id              ( master[ariane_soc::PULP].b_id      ),
        .axi_slave_b_user            ( master[ariane_soc::PULP].b_user    ),
        .axi_slave_b_ready           ( master[ariane_soc::PULP].b_ready   ),

        // AXI4 MASTER
        //***************************************
        // WRITE ADDRESS CHANNEL
        .axi_master_aw_addr          ( master_pulp.aw_addr                ),
        .axi_master_aw_prot          ( master_pulp.aw_prot                ),
        .axi_master_aw_region        ( master_pulp.aw_region              ),
        .axi_master_aw_len           ( master_pulp.aw_len                 ),
        .axi_master_aw_size          ( master_pulp.aw_size                ),
        .axi_master_aw_burst         ( master_pulp.aw_burst               ),
        .axi_master_aw_lock          ( master_pulp.aw_lock                ),
        .axi_master_aw_cache         ( master_pulp.aw_cache               ),
        .axi_master_aw_qos           ( master_pulp.aw_qos                 ),
        .axi_master_aw_id            ( master_pulp.aw_id                  ),
        .axi_master_aw_user          ( master_pulp.aw_user                ),
        .axi_master_aw_writetoken    ( master_pulp.aw_writetoken          ),
        .axi_master_aw_readpointer   ( master_pulp.aw_readpointer         ),
        // READ ADDRESS CHANNEL
        .axi_master_ar_addr          ( master_pulp.ar_addr                ),
        .axi_master_ar_prot          ( master_pulp.ar_prot                ),
        .axi_master_ar_region        ( master_pulp.ar_region              ),
        .axi_master_ar_len           ( master_pulp.ar_len                 ),
        .axi_master_ar_size          ( master_pulp.ar_size                ),
        .axi_master_ar_burst         ( master_pulp.ar_burst               ),
        .axi_master_ar_lock          ( master_pulp.ar_lock                ),
        .axi_master_ar_cache         ( master_pulp.ar_cache               ),
        .axi_master_ar_qos           ( master_pulp.ar_qos                 ),
        .axi_master_ar_id            ( master_pulp.ar_id                  ),
        .axi_master_ar_user          ( master_pulp.ar_user                ),
        .axi_master_ar_writetoken    ( master_pulp.ar_writetoken          ),
        .axi_master_ar_readpointer   ( master_pulp.ar_readpointer         ),

        // WRITE DATA CHANNEL
        .axi_master_w_data           ( master_pulp.w_data                 ),
        .axi_master_w_strb           ( master_pulp.w_strb                 ),
        .axi_master_w_user           ( master_pulp.w_user                 ),
        .axi_master_w_last           ( master_pulp.w_last                 ),
        .axi_master_w_writetoken     ( master_pulp.w_writetoken           ),
        .axi_master_w_readpointer    ( master_pulp.w_readpointer          ),

        // READ DATA CHANNEL
        .axi_master_r_data           ( master_pulp.r_data                 ),
        .axi_master_r_resp           ( master_pulp.r_resp                 ),
        .axi_master_r_last           ( master_pulp.r_last                 ),
        .axi_master_r_id             ( master_pulp.r_id                   ),
        .axi_master_r_user           ( master_pulp.r_user                 ),
        .axi_master_r_writetoken     ( master_pulp.r_writetoken           ),
        .axi_master_r_readpointer    ( master_pulp.r_readpointer          ),

        // WRITE RESPONSE CHANNEL
        .axi_master_b_resp           ( master_pulp.b_resp                 ),
        .axi_master_b_id             ( master_pulp.b_id                   ),
        .axi_master_b_user           ( master_pulp.b_user                 ),
        .axi_master_b_writetoken     ( master_pulp.b_writetoken           ),
        .axi_master_b_readpointer    ( master_pulp.b_readpointer          )
    );
*/


    // -------------
    // Request Side
    // -------------
    axi_slice_dc_slave_wrap
    #(
        .AXI_ADDR_WIDTH              ( AXI_ADDRESS_WIDTH                 ),
        .AXI_DATA_WIDTH              ( AXI_DATA_WIDTH                    ),
        .AXI_USER_WIDTH              ( AXI_USER_WIDTH                    ),
        .AXI_ID_WIDTH                ( AXI_ID_WIDTH_SLAVES               ),
        .BUFFER_WIDTH                ( AXI_BUFF_WIDTH_ASYNC              )
    ) i_axi_slice_dc_slave
    (

      .clk_i                 ( clk_i                    ),
      .rst_ni                ( ndmreset_n               ),
      .test_cgbypass_i       ( 1'b0                     ),
      .isolate_i             ( 1'b0                     ),
      .axi_slave             ( master[ariane_soc::PULP] ),
      .axi_master_async      ( master_pulp              )
    );

    pulp_soc #(
        .CORE_TYPE          ( 0                      ),
        .USE_FPU            ( 0                      ),
        .AXI_ADDR_WIDTH     ( 32                     ),
        .AXI_DATA_IN_WIDTH  ( 64                     ),
        .AXI_DATA_OUT_WIDTH ( 32                     ),
        .AXI_ID_IN_WIDTH    ( AXI_ID_WIDTH_SLAVES    ),
        .AXI_ID_OUT_WIDTH   ( 4                      ), //6???
        .AXI_USER_WIDTH     ( 1                      ),
        .AXI_STRB_WIDTH_IN  ( 64/8                   ),
        .AXI_STRB_WIDTH_OUT ( 32/8                   )
    ) i_pulp_soc
    (
      .ref_clk_i                     ( clk_i ),
      .slow_clk_i                    ( 1'b0  ),
      .test_clk_i                    ( 1'b0  ),
      .rstn_glob_i                   ( rst_ni ),

      .sel_fll_clk_i                 ( 1'b0 ),

      .dft_test_mode_i               ( 1'b0 ),
      .dft_cg_enable_i               ( 1'b0 ),
      .mode_select_i                 ( 1'b0 ),
      .soc_jtag_reg_i                ( '0   ),
      .soc_jtag_reg_o                (      ),
      .boot_l2_i                     ( 1'b0 ),

      .cluster_rtc_o                 ( ), //KEEP OPEN
      .cluster_fetch_enable_o        ( ), //KEEP OPEN
      .cluster_boot_addr_o           ( ), //KEEP OPEN
      .cluster_test_en_o             ( ), //KEEP OPEN
      .cluster_pow_o                 ( ), //KEEP OPEN
      .cluster_byp_o                 ( ), //KEEP OPEN
      .cluster_rstn_o                ( ), //KEEP OPEN
      .cluster_irq_o                 ( ), //KEEP OPEN

      // AXI4 SLAVE
      .data_slave_aw_writetoken_i    ( master_pulp.aw_writetoken   ),
      .data_slave_aw_addr_i          ( master_pulp.aw_addr[31:0]   ),
      .data_slave_aw_prot_i          ( master_pulp.aw_prot         ),
      .data_slave_aw_region_i        ( master_pulp.aw_region       ),
      .data_slave_aw_len_i           ( master_pulp.aw_len          ),
      .data_slave_aw_size_i          ( master_pulp.aw_size         ),
      .data_slave_aw_burst_i         ( master_pulp.aw_burst        ),
      .data_slave_aw_lock_i          ( master_pulp.aw_lock         ),
      .data_slave_aw_cache_i         ( master_pulp.aw_cache        ),
      .data_slave_aw_qos_i           ( master_pulp.aw_qos          ),
      .data_slave_aw_id_i            ( master_pulp.aw_id           ),
      .data_slave_aw_user_i          ( master_pulp.aw_user         ),
      .data_slave_aw_readpointer_o   ( master_pulp.aw_readpointer  ),
      .data_slave_ar_writetoken_i    ( master_pulp.ar_writetoken   ),
      .data_slave_ar_addr_i          ( master_pulp.ar_addr[31:0]   ),
      .data_slave_ar_prot_i          ( master_pulp.ar_prot         ),
      .data_slave_ar_region_i        ( master_pulp.ar_region       ),
      .data_slave_ar_len_i           ( master_pulp.ar_len          ),
      .data_slave_ar_size_i          ( master_pulp.ar_size         ),
      .data_slave_ar_burst_i         ( master_pulp.ar_burst        ),
      .data_slave_ar_lock_i          ( master_pulp.ar_lock         ),
      .data_slave_ar_cache_i         ( master_pulp.ar_cache        ),
      .data_slave_ar_qos_i           ( master_pulp.ar_qos          ),
      .data_slave_ar_id_i            ( master_pulp.ar_id           ),
      .data_slave_ar_user_i          ( master_pulp.ar_user         ),
      .data_slave_ar_readpointer_o   ( master_pulp.ar_readpointer  ),
      .data_slave_w_writetoken_i     ( master_pulp.w_writetoken    ),
      .data_slave_w_data_i           ( master_pulp.w_data          ),
      .data_slave_w_strb_i           ( master_pulp.w_strb          ),
      .data_slave_w_user_i           ( master_pulp.w_user          ),
      .data_slave_w_last_i           ( master_pulp.w_last          ),
      .data_slave_w_readpointer_o    ( master_pulp.w_readpointer   ),
      .data_slave_r_writetoken_o     ( master_pulp.r_writetoken    ),
      .data_slave_r_data_o           ( master_pulp.r_data          ),
      .data_slave_r_resp_o           ( master_pulp.r_resp          ),
      .data_slave_r_last_o           ( master_pulp.r_last          ),
      .data_slave_r_id_o             ( master_pulp.r_id            ),
      .data_slave_r_user_o           ( master_pulp.r_user          ),
      .data_slave_r_readpointer_i    ( master_pulp.r_readpointer   ),
      .data_slave_b_writetoken_o     ( master_pulp.b_writetoken    ),
      .data_slave_b_resp_o           ( master_pulp.b_resp          ),
      .data_slave_b_id_o             ( master_pulp.b_id            ),
      .data_slave_b_user_o           ( master_pulp.b_user          ),
      .data_slave_b_readpointer_i    ( master_pulp.b_readpointer   ),

      // AXI4 MASTER -- PULPissimo no MASTER NOW
      .data_master_aw_writetoken_o   (    ),
      .data_master_aw_addr_o         (    ),
      .data_master_aw_prot_o         (    ),
      .data_master_aw_region_o       (    ),
      .data_master_aw_len_o          (    ),
      .data_master_aw_size_o         (    ),
      .data_master_aw_burst_o        (    ),
      .data_master_aw_lock_o         (    ),
      .data_master_aw_cache_o        (    ),
      .data_master_aw_qos_o          (    ),
      .data_master_aw_id_o           (    ),
      .data_master_aw_user_o         (    ),
      .data_master_aw_readpointer_i  ( '0 ),
      .data_master_ar_writetoken_o   (    ),
      .data_master_ar_addr_o         (    ),
      .data_master_ar_prot_o         (    ),
      .data_master_ar_region_o       (    ),
      .data_master_ar_len_o          (    ),
      .data_master_ar_size_o         (    ),
      .data_master_ar_burst_o        (    ),
      .data_master_ar_lock_o         (    ),
      .data_master_ar_cache_o        (    ),
      .data_master_ar_qos_o          (    ),
      .data_master_ar_id_o           (    ),
      .data_master_ar_user_o         (    ),
      .data_master_ar_readpointer_i  ( '0 ),
      .data_master_w_writetoken_o    (    ),
      .data_master_w_data_o          (    ),
      .data_master_w_strb_o          (    ),
      .data_master_w_user_o          (    ),
      .data_master_w_last_o          (    ),
      .data_master_w_readpointer_i   ( '0 ),
      .data_master_r_writetoken_i    ( '0 ),
      .data_master_r_data_i          ( '0 ),
      .data_master_r_resp_i          ( '0 ),
      .data_master_r_last_i          ( '0 ),
      .data_master_r_id_i            ( '0 ),
      .data_master_r_user_i          ( '0 ),
      .data_master_r_readpointer_o   (    ),
      .data_master_b_writetoken_i    ( '0 ),
      .data_master_b_resp_i          ( '0 ),
      .data_master_b_id_i            ( '0 ),
      .data_master_b_user_i          ( '0 ),
      .data_master_b_readpointer_o   (    ),

      .cluster_events_wt_o           (    ),
      .cluster_events_rp_i           ( '0 ),
      .cluster_events_da_o           (    ),
      .cluster_clk_o                 (    ),
      .cluster_busy_i                ( '0 ),
      .dma_pe_evt_ack_o              (    ),
      .dma_pe_evt_valid_i            ( '0 ),
      .dma_pe_irq_ack_o              (    ),
      .dma_pe_irq_valid_i            ( '0 ),
      .pf_evt_ack_o                  (    ),
      .pf_evt_valid_i                ( '0 ),

      ///////////////////////////////////////////////////
      //      To I/O Controller and padframe           //
      ///////////////////////////////////////////////////
      .pad_mux_o                     (    ),
      .pad_cfg_o                     (    ),
      .gpio_in_i                     ( '0 ),
      .gpio_out_o                    (    ),
      .gpio_dir_o                    (    ),
      .gpio_cfg_o                    (    ),
      .uart_tx_o                     (    ),
      .uart_rx_i                     ( '0 ),
      .cam_clk_i                     ( '0 ),
      .cam_data_i                    ( '0 ),
      .cam_hsync_i                   ( '0 ),
      .cam_vsync_i                   ( '0 ),
      .timer_ch0_o                   (    ),
      .timer_ch1_o                   (    ),
      .timer_ch2_o                   (    ),
      .timer_ch3_o                   (    ),
      .i2c0_scl_i                    ( '0 ),
      .i2c0_scl_o                    (    ),
      .i2c0_scl_oe_o                 (    ),
      .i2c0_sda_i                    ( '0 ),
      .i2c0_sda_o                    (    ),
      .i2c0_sda_oe_o                 (    ),
      .i2c1_scl_i                    ( '0 ),
      .i2c1_scl_o                    (    ),
      .i2c1_scl_oe_o                 (    ),
      .i2c1_sda_i                    ( '0 ),
      .i2c1_sda_o                    (    ),
      .i2c1_sda_oe_o                 (    ),
      .i2s_sd0_i                     ( '0 ),
      .i2s_sd1_i                     ( '0 ),
      .i2s_sck_i                     ( '0 ),
      .i2s_ws_i                      ( '0 ),
      .i2s_sck0_o                    (    ),
      .i2s_ws0_o                     (    ),
      .i2s_mode0_o                   (    ),
      .i2s_sck1_o                    (    ),
      .i2s_ws1_o                     (    ),
      .i2s_mode1_o                   (    ),
      .spi_master0_clk_o             (    ),
      .spi_master0_csn0_o            (    ),
      .spi_master0_csn1_o            (    ),
      .spi_master0_mode_o            (    ),
      .spi_master0_sdo0_o            (    ),
      .spi_master0_sdo1_o            (    ),
      .spi_master0_sdo2_o            (    ),
      .spi_master0_sdo3_o            (    ),
      .spi_master0_sdi0_i            ( '0 ),
      .spi_master0_sdi1_i            ( '0 ),
      .spi_master0_sdi2_i            ( '0 ),
      .spi_master0_sdi3_i            ( '0 ),
      .sdio_clk_o                    (    ),
      .sdio_cmd_o                    (    ),
      .sdio_cmd_i                    ( '0 ),
      .sdio_cmd_oen_o                (    ),
      .sdio_data_o                   (    ),
      .sdio_data_i                   ( '0 ),
      .sdio_data_oen_o               (    ),

    ///////////////////////////////////////////////////
    ///////////////////////////////////////////////////
    // From JTAG Tap Controller to axi_dcb module    //
    ///////////////////////////////////////////////////
      .jtag_tck_i                    ( '0 ),
      .jtag_trst_ni                  ( '0 ),
      .jtag_axireg_tdi_i             ( '0 ),
      .jtag_axireg_tdo_o             (    ),
      .jtag_axireg_sel_i             ( '0 ),
      .jtag_shift_dr_i               ( '0 ),
      .jtag_update_dr_i              ( '0 ),
      .jtag_capture_dr_i             ( '0 )
    );

endmodule
