
module ph1p35_324_ddr_wrapper (
    input wire         I_sys_clk,
    input wire         I_sys_rst_n,

    output wire        O_ddr_clk,
    output  wire       O_init_calib_complete,
    input  wire[24:0]  I_mc_app_addr,
    input  wire[2:0]   I_mc_app_cmd,
    input  wire        I_mc_app_en,
    input  wire[127:0] I_mc_app_wdf_data,
    input  wire        I_mc_app_wdf_end,
    input  wire[15:0]  I_mc_app_wdf_mask,
    input  wire        I_mc_app_wdf_wren,
    output wire[127:0] O_mc_app_rd_data,
    output wire        O_mc_app_rd_data_end,
    output wire        O_mc_app_rd_data_valid,
    output wire        O_mc_app_rdy,
    output wire        O_mc_app_wdf_rdy,

// DDR3 signals
    output wire[12:0]  ddr_addr,
    output wire[1:0]   ddr_ba,
    output wire        ddr_cke,
    output wire        ddr_odt,
    output wire        ddr_cs_n,
    output wire        ddr_ras_n,
    output wire        ddr_cas_n,
    output wire        ddr_we_n,
    output wire        ddr_ck_p,
    output wire        ddr_ck_n,
    inout wire[1:0]    ddr_dm,
    inout wire[15:0]   ddr_dq,
    inout wire[1:0]    ddr_dqs_p,
    inout wire[1:0]    ddr_dqs_n   

);

    wire         ddrphy_rdy;
    wire         dhi_clk;

    wire [24:0]  paxi_awaddr;
    wire         paxi_awvalid;
    wire         paxi_awready;
       
    wire [127:0] paxi_wdata;
    wire [15:0]  paxi_wstrb;
    wire         paxi_wvalid;
    wire         paxi_wlast;
    wire         paxi_wready;
                                 
    wire [24:0]  paxi_araddr;
    wire         paxi_arvalid;
    wire         paxi_arready;

    wire [127:0] paxi_rdata;
    wire         paxi_rlast;
    wire         paxi_rvalid;
    wire         paxi_rready;                                    

    assign O_init_calib_complete = ddrphy_rdy;
    assign O_ddr_clk = dhi_clk;

    // The three PAXI request channels have INDEPENDENT ready signals. The
    // old (arready | awready), wdf_ready=awready mapping popped commands when
    // the required channel had not accepted them. Under mixed video/UDP load
    // this loses reads and separates write addresses from their pixel data.
    // Retain each command and retire AW and W separately. VALID never depends
    // on READY, and an already accepted half of a write is never replayed.
    reg command_valid;
    reg command_read;
    reg [24:0] command_addr;
    reg [127:0] command_data;
    reg [15:0] command_mask;
    reg address_pending;
    reg data_pending;
    wire command_done = command_read ? paxi_arready :
                        ((!address_pending || paxi_awready) &&
                         (!data_pending || paxi_wready));
    wire command_ready = I_sys_rst_n && ddrphy_rdy &&
                         (!command_valid || command_done);

    always @(posedge dhi_clk or negedge I_sys_rst_n) begin
        if (!I_sys_rst_n) begin
            command_valid <= 1'b0;
            command_read <= 1'b0;
            command_addr <= 25'd0;
            command_data <= 128'd0;
            command_mask <= 16'd0;
            address_pending <= 1'b0;
            data_pending <= 1'b0;
        end else if (!ddrphy_rdy) begin
            command_valid <= 1'b0;
            address_pending <= 1'b0;
            data_pending <= 1'b0;
        end else if (I_mc_app_en && command_ready) begin
            command_valid <= 1'b1;
            command_read <= I_mc_app_cmd[0];
            command_addr <= I_mc_app_addr;
            command_data <= I_mc_app_wdf_data;
            command_mask <= I_mc_app_wdf_mask;
            address_pending <= !I_mc_app_cmd[0];
            data_pending <= !I_mc_app_cmd[0];
        end else if (command_valid) begin
            if (paxi_awvalid && paxi_awready) address_pending <= 1'b0;
            if (paxi_wvalid && paxi_wready) data_pending <= 1'b0;
            if (command_done) command_valid <= 1'b0;
        end
    end

    assign paxi_awaddr              = command_addr;
    assign paxi_awvalid             = command_valid && !command_read && address_pending && ddrphy_rdy;
    assign paxi_wdata               = command_data;
    assign paxi_wstrb               = command_mask;
    assign paxi_wvalid              = command_valid && !command_read && data_pending && ddrphy_rdy;
    assign paxi_wlast               = 1'b1;
    assign paxi_araddr              = command_addr;
    assign paxi_arvalid             = command_valid && command_read && ddrphy_rdy;
    assign paxi_rready              = 1'b1                                      ;
    assign O_mc_app_rd_data        = paxi_rdata                                 ; 
    assign O_mc_app_rd_data_valid  = paxi_rvalid                                ;
    assign O_mc_app_rd_data_end    = paxi_rlast;
    assign O_mc_app_rdy            = command_ready;
    assign O_mc_app_wdf_rdy        = command_ready;


    ddr2 u_ddr2 (
        .ref_clk      ( I_sys_clk    ),
        .sys_rst_n    ( I_sys_rst_n  ),
        .apb_pclk     ( I_sys_clk    ),
        .apb_prst_n   ( I_sys_rst_n  ),                 
        .dhi_clk      ( dhi_clk      ),
        .cfg_clk      (  ),
        .ddrphy_rst_n (  ),
        .ddrphy_rdy   ( ddrphy_rdy   ),

        .paxi_awaddr  ( paxi_awaddr  ),
        .paxi_awvalid ( paxi_awvalid ),
        .paxi_awready ( paxi_awready ),
        .paxi_wdata   ( paxi_wdata   ),
        .paxi_wstrb   ( paxi_wstrb   ),
        .paxi_wvalid  ( paxi_wvalid  ),
        .paxi_wlast   ( paxi_wlast   ),
        .paxi_wready  ( paxi_wready  ),
        .paxi_araddr  ( paxi_araddr  ),
        .paxi_arvalid ( paxi_arvalid ),
        .paxi_arready ( paxi_arready ),
        .paxi_rdata   ( paxi_rdata   ),
        .paxi_rlast   ( paxi_rlast   ),
        .paxi_rvalid  ( paxi_rvalid  ),
        .paxi_rready  ( paxi_rready  ),

        .ddr_ck_p     ( ddr_ck_p     ),
        .ddr_ck_n     ( ddr_ck_n     ),
        .ddr_cke      ( ddr_cke      ),
        .ddr_odt      ( ddr_odt      ),
        .ddr_cs_n     ( ddr_cs_n     ),
        .ddr_ras_n    ( ddr_ras_n    ),
        .ddr_cas_n    ( ddr_cas_n    ),
        .ddr_we_n     ( ddr_we_n     ),
        .ddr_ba       ( ddr_ba       ),
        .ddr_addr     ( ddr_addr     ),
        .ddr_dqs_p    ( ddr_dqs_p    ),
        .ddr_dqs_n    ( ddr_dqs_n    ),
        .ddr_dq       ( ddr_dq       ),
        .ddr_dm       ( ddr_dm       )
    );





endmodule
