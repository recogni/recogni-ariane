// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "axi/assign.svh"

// Xilinx Peripehrals
module ariane_peripherals #(
  parameter int AxiAddrWidth = -1,
  parameter int AxiDataWidth = -1,
  parameter int AxiIdWidth   = -1,
  parameter int AxiUserWidth = 1,
  parameter bit InclUART     = 1,
  parameter bit InclSPI      = 0,
  parameter bit InclEthernet = 0,
  parameter bit InclGPIO     = 0,
  parameter bit InclTimer    = 1
) (
  input  logic       clk_i           , // Clock
  input  logic       rst_ni          , // Asynchronous reset active low
  AXI_BUS.Slave      spi             ,
  AXI_BUS.Slave      ethernet        ,
  AXI_BUS.Slave      apb_peripherals ,
  output logic [1:0] irq_o           ,
  // UART
  input  logic       rx_i            ,
  output logic       tx_o            ,
  // Ethernet
  input  wire        eth_txck        ,
  input  wire        eth_rxck        ,
  input  wire        eth_rxctl       ,
  input  wire [3:0]  eth_rxd         ,
  output wire        eth_rst_n       ,
  output wire        eth_tx_en       ,
  output wire [3:0]  eth_txd         ,
  inout  wire        phy_mdio        ,
  output logic       eth_mdc         ,
  // MDIO Interface
  inout              mdio            ,
  output             mdc             ,
  // SPI
  output logic       spi_clk_o       ,
  output logic       spi_mosi        ,
  input  logic       spi_miso        ,
  output logic       spi_ss
);
  // ---------------
  // 0. APB Peripherals
  // ---------------
  typedef struct packed {
      ariane_axi::addr_t    paddr;   // same as AXI4-Lite
      axi_pkg::prot_t       pprot;   // same as AXI4-Lite, specification is the same
      logic                 psel;    // onehot, one psel line per connected APB4 slave
      logic                 penable; // enable signal shows second APB4 cycle
      logic                 pwrite;  // write enable
      ariane_axi::data_32_t pwdata;  // write data, comes from W channel
      ariane_axi::strb_32_t pstrb;   // write strb, comes from W channel
  } apb_req_t;

  typedef struct packed {
      logic                 pready;   // slave signals that it is ready
      ariane_axi::data_32_t prdata;   // read data, connects to R channel
      logic                 pslverr;  // gets translated into either `axi_pkg::RESP_OK` or `axi_pkg::RESP_SLVERR`
  } apb_resp_t;

  ariane_axi::req_slv_t                    periph_axi_req;
  ariane_axi::resp_slv_t                   periph_axi_resp;
  ariane_axi::req_32_t                     periph_axi_32_req;
  ariane_axi::resp_32_t                    periph_axi_32_resp;
  ariane_axi::req_lite_t                   periph_axi_lite_req;
  ariane_axi::resp_lite_t                  periph_axi_lite_resp;
  apb_req_t  [ariane_soc::NoApbSlaves-1:0] periph_apb_req;
  apb_resp_t [ariane_soc::NoApbSlaves-1:0] periph_apb_resp;

  `AXI_ASSIGN_TO_REQ(periph_axi_req, apb_peripherals)
  `AXI_ASSIGN_FROM_RESP(apb_peripherals, periph_axi_resp)

  axi_dw_converter #(
      .AxiMaxReads     ( 32'd8                     ), // Number of outstanding reads
      .AxiMstDataWidth ( ariane_axi::DataWidth32   ), // Master data width
      .AxiSlvDataWidth ( AxiDataWidth              ), // Slave data width
      .AxiAddrWidth    ( AxiAddrWidth              ), // Address width
      .AxiIdWidth      ( AxiIdWidth                ), // ID width
      .aw_chan_t       ( ariane_axi::aw_chan_slv_t ), // AW Channel Type
      .mst_w_chan_t    ( ariane_axi::w_chan_32_t   ), //  W Channel Type for the mst port
      .slv_w_chan_t    ( ariane_axi::w_chan_t      ), //  W Channel Type for the slv port
      .b_chan_t        ( ariane_axi::b_chan_slv_t  ), //  B Channel Type
      .ar_chan_t       ( ariane_axi::ar_chan_slv_t ), // AR Channel Type
      .mst_r_chan_t    ( ariane_axi::r_chan_32_t   ), //  R Channel Type for the mst port
      .slv_r_chan_t    ( ariane_axi::r_chan_slv_t  ), //  R Channel Type for the slv port
      .axi_mst_req_t   ( ariane_axi::req_32_t      ), // AXI Request Type for mst ports
      .axi_mst_resp_t  ( ariane_axi::resp_32_t     ), // AXI Response Type for mst ports
      .axi_slv_req_t   ( ariane_axi::req_slv_t     ), // AXI Request Type for slv ports
      .axi_slv_resp_t  ( ariane_axi::resp_slv_t    )  // AXI Response Type for slv ports
  ) i_axi_dw_converter_apb_periph (
      .clk_i,
      .rst_ni,
      .slv_req_i  ( periph_axi_req     ),
      .slv_resp_o ( periph_axi_resp    ),
      .mst_req_o  ( periph_axi_32_req  ),
      .mst_resp_i ( periph_axi_32_resp )
  );

  axi_to_axi_lite #(
      .AxiIdWidth      ( AxiIdWidth              ),
      .AxiAddrWidth    ( AxiAddrWidth            ),
      .AxiDataWidth    ( ariane_axi::DataWidth32 ),
      .AxiUserWidth    ( AxiUserWidth            ),
      .AxiMaxWriteTxns ( 32'd8                   ),
      .AxiMaxReadTxns  ( 32'd8                   ),
      .FallThrough     ( 1'b0                    ),
      .full_req_t      ( ariane_axi::req_32_t    ),
      .full_resp_t     ( ariane_axi::resp_32_t   ),
      .lite_req_t      ( ariane_axi::req_lite_t  ),
      .lite_resp_t     ( ariane_axi::resp_lite_t )
  ) i_axi_to_axi_lite_apb_periph (
      .clk_i,
      .rst_ni,
      .test_i     ( 1'b0                 ),
      .slv_req_i  ( periph_axi_32_req    ),
      .slv_resp_o ( periph_axi_32_resp   ),
      .mst_req_o  ( periph_axi_lite_req  ),
      .mst_resp_i ( periph_axi_lite_resp )
  );

  axi_lite_to_apb #(
      .NoApbSlaves     ( ariane_soc::NoApbSlaves     ), // Number of connected APB slaves
      .NoRules         ( ariane_soc::NoApbSlaves     ), // Number of APB address rules
      .AddrWidth       ( ariane_axi::AddrWidth       ), // Address width
      .DataWidth       ( ariane_axi::DataWidth32     ), // Data width
      .axi_lite_req_t  ( ariane_axi::req_lite_t      ), // AXI4-Lite request struct
      .axi_lite_resp_t ( ariane_axi::resp_lite_t     ), // AXI4-Lite response sruct
      .apb_req_t       ( apb_req_t                   ), // APB4 request struct
      .apb_resp_t      ( apb_resp_t                  ), // APB4 response struct
      .rule_t          ( axi_pkg::xbar_rule_64_t     )  // Address Decoder rule from `common_cells`
  ) i_axi_lite_to_apb_periph (
      .clk_i,
      .rst_ni,
      .axi_lite_req_i  ( periph_axi_lite_req    ),
      .axi_lite_resp_o ( periph_axi_lite_resp   ),
      .apb_req_o       ( periph_apb_req         ),
      .apb_resp_i      ( periph_apb_resp        ),
      .addr_map_i      ( ariane_soc::ApbAddrMap )
  );

  // Apb Plic
  logic [ariane_soc::NumSources-1:0] irq_sources;

  REG_BUS #(
      .ADDR_WIDTH ( 32 ),
      .DATA_WIDTH ( 32 )
  ) reg_bus_plic (clk_i);
  apb_to_reg i_apb_to_reg (
      .clk_i,
      .rst_ni,
      .penable_i ( periph_apb_req [ariane_soc::ApbPlic].penable     ),
      .pwrite_i  ( periph_apb_req [ariane_soc::ApbPlic].pwrite      ),
      .paddr_i   ( periph_apb_req [ariane_soc::ApbPlic].paddr[31:0] ),
      .psel_i    ( periph_apb_req [ariane_soc::ApbPlic].psel        ),
      .pwdata_i  ( periph_apb_req [ariane_soc::ApbPlic].pwdata      ),
      .prdata_o  ( periph_apb_resp[ariane_soc::ApbPlic].prdata      ),
      .pready_o  ( periph_apb_resp[ariane_soc::ApbPlic].pready      ),
      .pslverr_o ( periph_apb_resp[ariane_soc::ApbPlic].pslverr     ),
      .reg_o     ( reg_bus_plic      )
  );

  reg_intf::reg_intf_resp_d32 plic_resp;
  reg_intf::reg_intf_req_a32_d32 plic_req;

  assign plic_req.addr  = reg_bus_plic.addr;
  assign plic_req.write = reg_bus_plic.write;
  assign plic_req.wdata = reg_bus_plic.wdata;
  assign plic_req.wstrb = reg_bus_plic.wstrb;
  assign plic_req.valid = reg_bus_plic.valid;

  assign reg_bus_plic.rdata = plic_resp.rdata;
  assign reg_bus_plic.error = plic_resp.error;
  assign reg_bus_plic.ready = plic_resp.ready;

  plic_top #(
    .N_SOURCE    ( ariane_soc::NumSources  ),
    .N_TARGET    ( ariane_soc::NumTargets  ),
    .MAX_PRIO    ( ariane_soc::MaxPriority )
  ) i_plic (
    .clk_i,
    .rst_ni,
    .req_i         ( plic_req    ),
    .resp_o        ( plic_resp   ),
    .le_i          ( '0          ), // 0:level 1:edge
    .irq_sources_i ( irq_sources ),
    .eip_targets_o ( irq_o       )
  );

  // APB Uart
  if (InclUART) begin : gen_uart
    apb_uart i_apb_uart (
      .CLK     ( clk_i                                          ),
      .RSTN    ( rst_ni                                         ),
      .PSEL    ( periph_apb_req[ariane_soc::ApbUart].psel       ),
      .PENABLE ( periph_apb_req[ariane_soc::ApbUart].penable    ),
      .PWRITE  ( periph_apb_req[ariane_soc::ApbUart].pwrite     ),
      .PADDR   ( periph_apb_req[ariane_soc::ApbUart].paddr[4:2] ),
      .PWDATA  ( periph_apb_req[ariane_soc::ApbUart].pwdata     ),
      .PRDATA  ( periph_apb_resp[ariane_soc::ApbUart].prdata    ),
      .PREADY  ( periph_apb_resp[ariane_soc::ApbUart].pready    ),
      .PSLVERR ( periph_apb_resp[ariane_soc::ApbUart].pslverr   ),
      .INT     ( irq_sources[0]                                 ),
      .OUT1N   (                                                ), // keep open
      .OUT2N   (                                                ), // keep open
      .RTSN    (                                                ), // no flow control
      .DTRN    (                                                ), // no flow control
      .CTSN    ( 1'b0                                           ),
      .DSRN    ( 1'b0                                           ),
      .DCDN    ( 1'b0                                           ),
      .RIN     ( 1'b0                                           ),
      .SIN     ( rx_i                                           ),
      .SOUT    ( tx_o                                           )
    );
  end else begin
    /* pragma translate_off */
    `ifndef VERILATOR
    mock_uart i_mock_uart (
      .clk_i,
      .rst_ni,
      .penable_i ( periph_apb_req[ariane_soc::ApbUart].penable     ),
      .pwrite_i  ( periph_apb_req[ariane_soc::ApbUart].pwrite      ),
      .paddr_i   ( periph_apb_req[ariane_soc::ApbUart].paddr[31:0] ),
      .psel_i    ( periph_apb_req[ariane_soc::ApbUart].psel        ),
      .pwdata_i  ( periph_apb_req[ariane_soc::ApbUart].pwdata      ),
      .prdata_o  ( periph_apb_resp[ariane_soc::ApbUart].prdata     ),
      .pready_o  ( periph_apb_resp[ariane_soc::ApbUart].pready     ),
      .pslverr_o ( periph_apb_resp[ariane_soc::ApbUart].pslverr    )
    );
    `endif
    /* pragma translate_on */
  end

  // Apb Timer
  if (InclTimer) begin : gen_timer
    apb_timer #(
      .APB_ADDR_WIDTH ( 64 ),
      .TIMER_CNT      ( 2  )
    ) i_timer (
      .HCLK    ( clk_i                                         ),
      .HRESETn ( rst_ni                                        ),
      .PSEL    ( periph_apb_req[ariane_soc::ApbTimer].psel     ),
      .PENABLE ( periph_apb_req[ariane_soc::ApbTimer].penable  ),
      .PWRITE  ( periph_apb_req[ariane_soc::ApbTimer].pwrite   ),
      .PADDR   ( periph_apb_req[ariane_soc::ApbTimer].paddr    ),
      .PWDATA  ( periph_apb_req[ariane_soc::ApbTimer].pwdata   ),
      .PRDATA  ( periph_apb_resp[ariane_soc::ApbTimer].prdata  ),
      .PREADY  ( periph_apb_resp[ariane_soc::ApbTimer].pready  ),
      .PSLVERR ( periph_apb_resp[ariane_soc::ApbTimer].pslverr ),
      .irq_o   ( irq_sources[6:3]                              )
    );
  end else begin
    assign periph_apb_resp[ariane_soc::ApbTimer].prdata  = 32'hdeadbeef;
    assign periph_apb_resp[ariane_soc::ApbTimer].pready  = 1'b1;
    assign periph_apb_resp[ariane_soc::ApbTimer].pslverr = 1'b1;
  end


    // ---------------
    // 3. SPI
    // ---------------
    if (InclSPI) begin : gen_spi
        logic [31:0] s_axi_spi_awaddr;
        logic [7:0]  s_axi_spi_awlen;
        logic [2:0]  s_axi_spi_awsize;
        logic [1:0]  s_axi_spi_awburst;
        logic [0:0]  s_axi_spi_awlock;
        logic [3:0]  s_axi_spi_awcache;
        logic [2:0]  s_axi_spi_awprot;
        logic [3:0]  s_axi_spi_awregion;
        logic [3:0]  s_axi_spi_awqos;
        logic        s_axi_spi_awvalid;
        logic        s_axi_spi_awready;
        logic [31:0] s_axi_spi_wdata;
        logic [3:0]  s_axi_spi_wstrb;
        logic        s_axi_spi_wlast;
        logic        s_axi_spi_wvalid;
        logic        s_axi_spi_wready;
        logic [1:0]  s_axi_spi_bresp;
        logic        s_axi_spi_bvalid;
        logic        s_axi_spi_bready;
        logic [31:0] s_axi_spi_araddr;
        logic [7:0]  s_axi_spi_arlen;
        logic [2:0]  s_axi_spi_arsize;
        logic [1:0]  s_axi_spi_arburst;
        logic [0:0]  s_axi_spi_arlock;
        logic [3:0]  s_axi_spi_arcache;
        logic [2:0]  s_axi_spi_arprot;
        logic [3:0]  s_axi_spi_arregion;
        logic [3:0]  s_axi_spi_arqos;
        logic        s_axi_spi_arvalid;
        logic        s_axi_spi_arready;
        logic [31:0] s_axi_spi_rdata;
        logic [1:0]  s_axi_spi_rresp;
        logic        s_axi_spi_rlast;
        logic        s_axi_spi_rvalid;
        logic        s_axi_spi_rready;

        xlnx_axi_clock_converter i_xlnx_axi_clock_converter_spi (
            .s_axi_aclk     ( clk_i              ),
            .s_axi_aresetn  ( rst_ni             ),

            .s_axi_awid     ( spi.aw_id          ),
            .s_axi_awaddr   ( spi.aw_addr[31:0]  ),
            .s_axi_awlen    ( spi.aw_len         ),
            .s_axi_awsize   ( spi.aw_size        ),
            .s_axi_awburst  ( spi.aw_burst       ),
            .s_axi_awlock   ( spi.aw_lock        ),
            .s_axi_awcache  ( spi.aw_cache       ),
            .s_axi_awprot   ( spi.aw_prot        ),
            .s_axi_awregion ( spi.aw_region      ),
            .s_axi_awqos    ( spi.aw_qos         ),
            .s_axi_awvalid  ( spi.aw_valid       ),
            .s_axi_awready  ( spi.aw_ready       ),
            .s_axi_wdata    ( spi.w_data         ),
            .s_axi_wstrb    ( spi.w_strb         ),
            .s_axi_wlast    ( spi.w_last         ),
            .s_axi_wvalid   ( spi.w_valid        ),
            .s_axi_wready   ( spi.w_ready        ),
            .s_axi_bid      ( spi.b_id           ),
            .s_axi_bresp    ( spi.b_resp         ),
            .s_axi_bvalid   ( spi.b_valid        ),
            .s_axi_bready   ( spi.b_ready        ),
            .s_axi_arid     ( spi.ar_id          ),
            .s_axi_araddr   ( spi.ar_addr[31:0]  ),
            .s_axi_arlen    ( spi.ar_len         ),
            .s_axi_arsize   ( spi.ar_size        ),
            .s_axi_arburst  ( spi.ar_burst       ),
            .s_axi_arlock   ( spi.ar_lock        ),
            .s_axi_arcache  ( spi.ar_cache       ),
            .s_axi_arprot   ( spi.ar_prot        ),
            .s_axi_arregion ( spi.ar_region      ),
            .s_axi_arqos    ( spi.ar_qos         ),
            .s_axi_arvalid  ( spi.ar_valid       ),
            .s_axi_arready  ( spi.ar_ready       ),
            .s_axi_rid      ( spi.r_id           ),
            .s_axi_rdata    ( spi.r_data         ),
            .s_axi_rresp    ( spi.r_resp         ),
            .s_axi_rlast    ( spi.r_last         ),
            .s_axi_rvalid   ( spi.r_valid        ),
            .s_axi_rready   ( spi.r_ready        ),

            .m_axi_awaddr   ( s_axi_spi_awaddr   ),
            .m_axi_awlen    ( s_axi_spi_awlen    ),
            .m_axi_awsize   ( s_axi_spi_awsize   ),
            .m_axi_awburst  ( s_axi_spi_awburst  ),
            .m_axi_awlock   ( s_axi_spi_awlock   ),
            .m_axi_awcache  ( s_axi_spi_awcache  ),
            .m_axi_awprot   ( s_axi_spi_awprot   ),
            .m_axi_awregion ( s_axi_spi_awregion ),
            .m_axi_awqos    ( s_axi_spi_awqos    ),
            .m_axi_awvalid  ( s_axi_spi_awvalid  ),
            .m_axi_awready  ( s_axi_spi_awready  ),
            .m_axi_wdata    ( s_axi_spi_wdata    ),
            .m_axi_wstrb    ( s_axi_spi_wstrb    ),
            .m_axi_wlast    ( s_axi_spi_wlast    ),
            .m_axi_wvalid   ( s_axi_spi_wvalid   ),
            .m_axi_wready   ( s_axi_spi_wready   ),
            .m_axi_bresp    ( s_axi_spi_bresp    ),
            .m_axi_bvalid   ( s_axi_spi_bvalid   ),
            .m_axi_bready   ( s_axi_spi_bready   ),
            .m_axi_araddr   ( s_axi_spi_araddr   ),
            .m_axi_arlen    ( s_axi_spi_arlen    ),
            .m_axi_arsize   ( s_axi_spi_arsize   ),
            .m_axi_arburst  ( s_axi_spi_arburst  ),
            .m_axi_arlock   ( s_axi_spi_arlock   ),
            .m_axi_arcache  ( s_axi_spi_arcache  ),
            .m_axi_arprot   ( s_axi_spi_arprot   ),
            .m_axi_arregion ( s_axi_spi_arregion ),
            .m_axi_arqos    ( s_axi_spi_arqos    ),
            .m_axi_arvalid  ( s_axi_spi_arvalid  ),
            .m_axi_arready  ( s_axi_spi_arready  ),
            .m_axi_rdata    ( s_axi_spi_rdata    ),
            .m_axi_rresp    ( s_axi_spi_rresp    ),
            .m_axi_rlast    ( s_axi_spi_rlast    ),
            .m_axi_rvalid   ( s_axi_spi_rvalid   ),
            .m_axi_rready   ( s_axi_spi_rready   )
        );

        xlnx_axi_quad_spi i_xlnx_axi_quad_spi (
            .ext_spi_clk    ( clk_i                  ),
            .s_axi4_aclk    ( clk_i                  ),
            .s_axi4_aresetn ( rst_ni                 ),
            .s_axi4_awaddr  ( s_axi_spi_awaddr[23:0] ),
            .s_axi4_awlen   ( s_axi_spi_awlen        ),
            .s_axi4_awsize  ( s_axi_spi_awsize       ),
            .s_axi4_awburst ( s_axi_spi_awburst      ),
            .s_axi4_awlock  ( s_axi_spi_awlock       ),
            .s_axi4_awcache ( s_axi_spi_awcache      ),
            .s_axi4_awprot  ( s_axi_spi_awprot       ),
            .s_axi4_awvalid ( s_axi_spi_awvalid      ),
            .s_axi4_awready ( s_axi_spi_awready      ),
            .s_axi4_wdata   ( s_axi_spi_wdata        ),
            .s_axi4_wstrb   ( s_axi_spi_wstrb        ),
            .s_axi4_wlast   ( s_axi_spi_wlast        ),
            .s_axi4_wvalid  ( s_axi_spi_wvalid       ),
            .s_axi4_wready  ( s_axi_spi_wready       ),
            .s_axi4_bresp   ( s_axi_spi_bresp        ),
            .s_axi4_bvalid  ( s_axi_spi_bvalid       ),
            .s_axi4_bready  ( s_axi_spi_bready       ),
            .s_axi4_araddr  ( s_axi_spi_araddr[23:0] ),
            .s_axi4_arlen   ( s_axi_spi_arlen        ),
            .s_axi4_arsize  ( s_axi_spi_arsize       ),
            .s_axi4_arburst ( s_axi_spi_arburst      ),
            .s_axi4_arlock  ( s_axi_spi_arlock       ),
            .s_axi4_arcache ( s_axi_spi_arcache      ),
            .s_axi4_arprot  ( s_axi_spi_arprot       ),
            .s_axi4_arvalid ( s_axi_spi_arvalid      ),
            .s_axi4_arready ( s_axi_spi_arready      ),
            .s_axi4_rdata   ( s_axi_spi_rdata        ),
            .s_axi4_rresp   ( s_axi_spi_rresp        ),
            .s_axi4_rlast   ( s_axi_spi_rlast        ),
            .s_axi4_rvalid  ( s_axi_spi_rvalid       ),
            .s_axi4_rready  ( s_axi_spi_rready       ),

            .io0_i          ( '0                     ),
            .io0_o          ( spi_mosi               ),
            .io0_t          ( '0                     ),
            .io1_i          ( spi_miso               ),
            .io1_o          (                        ),
            .io1_t          ( '0                     ),
            .ss_i           ( '0                     ),
            .ss_o           ( spi_ss                 ),
            .ss_t           ( '0                     ),
            .sck_o          ( spi_clk_o              ),
            .sck_i          ( '0                     ),
            .sck_t          (                        ),
            .ip2intc_irpt   ( irq_sources[1]         )
        );
    end else begin
        assign spi_clk_o      = 1'b0;
        assign spi_mosi       = 1'b0;
        assign spi_ss         = 1'b0;
        assign irq_sources[1] = 1'b0;

        ariane_axi::req_slv_t  spi_axi_req;
        ariane_axi::resp_slv_t spi_axi_resp;

        `AXI_ASSIGN_TO_REQ(spi_axi_req, spi)
        `AXI_ASSIGN_FROM_RESP(spi, spi_axi_resp)

        axi_err_slv #(
          .AxiIdWidth ( AxiIdWidth             ),
          .req_t      ( ariane_axi::req_slv_t  ),
          .resp_t     ( ariane_axi::resp_slv_t ),
          .Resp       ( axi_pkg::RESP_SLVERR   ),
          .ATOPs      ( 1'b1                   ),
          .MaxTrans   ( 32'd1                  )
        ) i_axi_err_slv_spi (
          .clk_i,
          .rst_ni,
          .test_i     ( 1'b0         ),
          .slv_req_i  ( spi_axi_req  ),
          .slv_resp_o ( spi_axi_resp )
        );
    end


    // ---------------
    // 4. Ethernet
    // ---------------
    if (0) begin
    end else begin
      assign irq_sources [2] = 1'b0;

      ariane_axi::req_slv_t  ethernet_axi_req;
      ariane_axi::resp_slv_t ethernet_axi_resp;

      `AXI_ASSIGN_TO_REQ(ethernet_axi_req, ethernet)
      `AXI_ASSIGN_FROM_RESP(ethernet, ethernet_axi_resp)

      axi_err_slv #(
        .AxiIdWidth ( AxiIdWidth             ),
        .req_t      ( ariane_axi::req_slv_t  ),
        .resp_t     ( ariane_axi::resp_slv_t ),
        .Resp       ( axi_pkg::RESP_SLVERR   ),
        .ATOPs      ( 1'b1                   ),
        .MaxTrans   ( 32'd1                  )
      ) i_axi_err_slv_spi (
        .clk_i,
        .rst_ni,
        .test_i     ( 1'b0              ),
        .slv_req_i  ( ethernet_axi_req  ),
        .slv_resp_o ( ethernet_axi_resp )
      );
    end
endmodule
