`timescale 1ns / 1ps

`define READ 1'b0
`define WRITE 1'b1
`define NoBusy 1'b0
`define Busy 1'b1
`define DONE 1'b1
`define UNDONE 1'b0
module sealedDDR (
	input clk100mhz,    // Clock
	input rst,
	input [31:0]addr_to_DDR,
	input [31:0]data_to_DDR,
	input read_write,
	output  [31:0]data_from_DDR,
	output reg busy,
	output reg done,
	output  ddr_start_ready,
	/************************/
	inout [15:0]            ddr2_dq,
	inout [1:0]             ddr2_dqs_n,
	inout [1:0]             ddr2_dqs_p,
	output [12:0]           ddr2_addr,
	output [2:0]            ddr2_ba,
	output                  ddr2_ras_n,
	output                  ddr2_cas_n,
	output                  ddr2_we_n,
	output [0:0]            ddr2_ck_p,
	output [0:0]            ddr2_ck_n,
	output [0:0]            ddr2_cke,
	output [0:0]            ddr2_cs_n,
	output [1:0]            ddr2_dm,
	output [0:0]            ddr2_odt
/************tempLook********/

);

	wire [31:0] reg_addr_in;
	wire [127:0] reg_data_to_DDR;
	wire reg_read_write;

 	



	wire clk200mhz;
	wire [1:0] rdqs_n;
	//reg [31:0]data_in;
	wire ack;

	wire [127:0] app_rd_data;
	wire app_rd_data_valid;
	wire app_rdy;
	
	clk_wiz_0 clk_divider(.clk_in1(clk100mhz),.clk_out1(clk200mhz));

	ddr2_wr ddr2_wr_ins(
		.clk_in(clk100mhz),
		.rst(rst),
		//ddr2 parameter
		.ddr2_ck_p(ddr2_ck_p),
		.ddr2_ck_n(ddr2_ck_n),
		.ddr2_cke(ddr2_cke),
		.ddr2_cs_n(ddr2_cs_n),
		.ddr2_ras_n(ddr2_ras_n),
		.ddr2_cas_n(ddr2_cas_n),
		.ddr2_we_n(ddr2_we_n),
		.ddr2_dm(ddr2_dm),
		.ddr2_ba(ddr2_ba),
		.ddr2_addr(ddr2_addr),
		.ddr2_dq(ddr2_dq),
		.ddr2_dqs_p(ddr2_dqs_p),
		.ddr2_dqs_n(ddr2_dqs_n),
		.rdqs_n(rdqs_n),
		.ddr2_odt(ddr2_odt),
		.clk_ref_i(clk200mhz),  //鐢ㄤ簬ddr
		.addr_i_32(reg_addr_in),
		.data_i(reg_data_to_DDR),
		.stb_i(reg_read_write),
		.ack_o(ack),
		.app_rd_data(app_rd_data),
		.app_rd_data_valid(app_rd_data_valid),
		.app_rdy(app_rdy)
	);
	parameter readInsistLoop=256;
	parameter writeInsistLoop=256;
	parameter ddr_startLoop=256;
	reg [63:0]write_insist_count;
	reg [63:0]read_insist_count;
	reg [64:0]wake_count;
	reg [63:0]before_rdy_count;
	reg ddr_is_ready;
	assign ddr_start_ready=ddr_is_ready;
	always @(posedge clk100mhz or posedge rst) begin
		if(rst) begin
		    wake_count<=0;
			//reg_read_write<= `READ;
			//reg_addr_in<=32'b0;
			//reg_data_to_DDR<=128'h0;
			busy<=`Busy;
			done<=`UNDONE;
			read_insist_count<=0;
			write_insist_count<=0;
			before_rdy_count<=0;
			ddr_is_ready<=1'b0;
		end
		else if(~ddr_is_ready)begin
			if(before_rdy_count<ddr_startLoop&app_rdy)begin
				before_rdy_count<=before_rdy_count+1;
			end
			else if(app_rdy)begin
				ddr_is_ready<=1'b1;
				busy=`NoBusy;
			end
		end
		else if(busy==`NoBusy&app_rdy&ddr_is_ready|done) begin
			//Only allow accept req when notbusy
			//reg_read_write<=read_write;
			//reg_addr_in<=addr_to_DDR;
			//reg_data_to_DDR<={96'b0,data_to_DDR};
			busy<=`Busy;
			done<=`UNDONE;
			read_insist_count<=0;
			write_insist_count<=0;
		end
		else if(busy==`Busy&ddr_is_ready)begin
			if(reg_read_write==`WRITE)begin
//				if(ack)begin
				if(write_insist_count>=writeInsistLoop)begin
					done<=`DONE;
					busy<=`NoBusy;
//					reg_read_write<=`READ;
				end
				else begin
				    write_insist_count<=write_insist_count+1;
				end
			end
			else if(reg_read_write==`READ)begin
				//if(app_rd_data_valid)begin
				if(read_insist_count>=readInsistLoop&app_rd_data_valid)begin
				//data_from_DDR<=app_rd_data[31:0];
				//data_from_DDR<=app_rd_data;
					done<=`DONE;
					busy<=`NoBusy;
				end
				else begin
				    read_insist_count<=read_insist_count+1;
				end
			end
		end
		else begin 
			done<=`UNDONE;
		end
	end

assign data_from_DDR=app_rd_data[31:0];
assign reg_addr_in=addr_to_DDR;
assign reg_read_write=read_write;
assign reg_data_to_DDR={96'b0,data_to_DDR};

endmodule



//-----------------------------------------------------------------------------------------------------------------
//ddr2_read_control------------------------------------------------------------------------------------------------
module ddr2_read_control(
    input clk_in,
    input rst_n,
    input enable,

    input [26:0]addr_read,


    //ddr2 mig signals
    output reg app_en,
    output reg [2:0] app_cmd,
    output reg [26:0] app_addr,
    input [127:0] app_rd_data,
    input app_rdy,
    input app_rd_data_end,
    input app_rd_data_valid
    );

   

    reg [26:0] app_addr_tmp;

    //读取FSM
    reg [4:0] cstate;
    
    localparam IDLE = 5'b0_0001;
    localparam READ = 5'b0_0010;
    localparam WAIT = 5'b0_0100;
    localparam ADDR_ACCUMULATE = 5'b0_1000;
    localparam WAIT_FOR_CONFIG = 5'b1_0000;

    always @(posedge clk_in)
    begin
        if(rst_n) begin
            app_en <= 0;
            app_addr<=0;
            cstate <= IDLE;
        end
        else if(enable) begin
            case(cstate)
            IDLE:begin
                app_en <= 1;
                app_addr<=addr_read;
                app_cmd <= 3'b001;
                cstate <= READ;
            end
            READ:begin
                if(app_rdy) begin
                    app_en <= 1'b0;
                    app_addr<=addr_read;
                    cstate <= IDLE;
                end
            end
            default:cstate <= IDLE;
            endcase
        end 
        else begin
            app_en <= 1'b0;
            cstate <= IDLE;
        end 
    end

endmodule

//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------





//-----------------------------------------------------------------------------------------------------------------
//ddr2_write_control-----------------------------------------------------------------------------------------------
module ddr2_write_control(
    clk_in,
    rst_n,
    ADDR_I,
    DATA_I,
    STB_I,
    ACK_O,
    read_en,
    //ddr_ signals
    app_en,
    app_wdf_wren,
    app_wdf_end,
    app_cmd,
    app_addr,
    app_wdf_data,
    app_rdy,
    app_wdf_rdy
    );

    parameter DQ_WIDTH          = 16;
    parameter ECC_TEST          = "OFF";
    parameter ADDR_WIDTH        = 27;
    parameter nCK_PER_CLK       = 4;

    localparam DATA_WIDTH       = 16;
    localparam PAYLOAD_WIDTH    = (ECC_TEST == "OFF") ? DATA_WIDTH : DQ_WIDTH;
    localparam APP_DATA_WIDTH   = 2 * nCK_PER_CLK * PAYLOAD_WIDTH;  //突发长度为8
    localparam APP_MASK_WIDTH   = APP_DATA_WIDTH / 8;

    input clk_in;
    input rst_n;  
    input [26:0] ADDR_I;    //读取地址、偏移
    input [127:0] DATA_I;  //需要写入的数据
    input STB_I;    //选通信号
    output reg ACK_O;    //可以接收数据标志位，高有效
    output reg read_en;

    // Wire declarations
    output reg app_en, app_wdf_wren, app_wdf_end;
    output reg [2:0] app_cmd;
    output reg [ADDR_WIDTH-1:0] app_addr;
    output reg [APP_DATA_WIDTH-1:0] app_wdf_data;
    input app_rdy, app_wdf_rdy;

    //生成写入数据的信号值
    //----------FSM--------
    reg [2:0] cstate;

    parameter IDLE = 3'b001;
    parameter WRITE = 3'b010;

    reg [3:0] write_count;


    always @(posedge clk_in)
    begin
        if(rst_n) begin
            app_cmd <= 3'b1;
            app_en <= 1'b0;
            app_wdf_data <= 128'h0;
            app_addr <= 27'h0;
            app_wdf_end <= 1'b0;
            app_wdf_wren <= 1'b0;
            write_count <= 0;
            read_en <= 0;
            ACK_O <= 0;
            cstate <= IDLE;
        end
        else if(STB_I) begin
            case(cstate)
                IDLE:begin
                    if(app_rdy & app_wdf_rdy) begin
                        app_wdf_data <= DATA_I;
                        app_cmd <= 3'b0;
                        app_addr <= ADDR_I;
                        app_wdf_wren <= 1'b1;
                        app_wdf_end <= 1'b1;
                        app_en <= 1'b1;
                        ACK_O <= 0;  //可以接收数据
                        write_count <= write_count + 1;
                        cstate <= WRITE;
                    end
                    else cstate <= IDLE; 
                end
                WRITE:begin
                    app_en <= 1'b0;
                    app_cmd <= 3'b1;
                    ACK_O <= 1;
                    app_wdf_wren <= 1'b0;
                    app_wdf_end <= 0;
                    if(write_count == 3)
                        read_en <= 1;
                    cstate <= IDLE;
                end
                default:cstate <= IDLE;
            endcase
        end
        else begin
            app_en <= 0;
            app_wdf_wren <= 0;
            app_wdf_end <= 0;
            ACK_O <= 0;
            cstate <= IDLE;
        end
    end


endmodule


//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------







//-----------------------------------------------------------------------------------------------------------------
//ddr2_wr----------------------------------------------------------------------------------------------------------
module ddr2_wr(
    input clk_in,
    input rst,
    input clk_ref_i,
    output ddr2_ck_p,
    output ddr2_ck_n,
    output ddr2_cke,
    output ddr2_cs_n,
    output ddr2_ras_n,
    output ddr2_cas_n,
    output ddr2_we_n,
    output [1:0] ddr2_dm,
    output [2:0] ddr2_ba,
    output [12:0] ddr2_addr,
    inout [15:0] ddr2_dq,
    inout [1:0] ddr2_dqs_p,
    inout [1:0] ddr2_dqs_n,
    output [1:0] rdqs_n,
    output ddr2_odt,

    input [31:0] addr_i_32,
    input [127:0] data_i,
    input stb_i,
    output ack_o,
    output [127:0] app_rd_data,
    output app_rd_data_valid,
    output app_rdy
    );

    wire [26:0]addr_i;
    assign addr_i[26:3]=addr_i_32[23:0];
    assign addr_i[2:0]=3'b0;

    //---MIG IP core parameter
    //---user interface signals
    wire [26:0] app_addr;
    reg [2:0] app_cmd;
    wire [2:0] app_cmd_wr;
    wire [26:0] app_addr_wr;
    wire app_en_wr;
    wire app_en;
    wire [127:0] app_wdf_data;
    wire app_wdf_end;
    wire app_wdf_wren;
   // wire [127:0] app_rd_data;
    wire app_rd_data_end;
   // wire app_rd_data_valid;
    //wire app_rdy;
    wire app_wdf_rdy;
    wire app_sr_active;
    wire app_ref_ack;
    wire app_zq_ack;
    wire ui_clk;
    wire ui_clk_sync_rst;

    wire [2:0] app_cmd_pe;
    wire [26:0] app_addr_pe;
    wire app_en_pe;
    assign app_en = app_en_wr | app_en_pe;


    always @(*)
    begin
        if(stb_i&app_en_wr)begin
            app_cmd <= app_cmd_wr;
            //app_addr <= app_addr_wr;
        end
        else if(~stb_i&app_en_pe) begin
             app_cmd <= app_cmd_pe;
             //app_addr <= app_addr_pe;
        end
    end
    assign app_addr=addr_i;

//    always @(*)
//    begin
//        if(app_en_pe) begin
//            app_cmd = app_cmd_pe;
//            app_addr = app_addr_pe;
//        end
//        else if(app_en_wr) begin
//            app_cmd = app_cmd_wr;
//            app_addr = app_addr_wr;
//        end
//        else begin
//            app_cmd = 3'b1;
//            app_addr = 27'bx;
//        end
//    end

    wire read_enable;

    /* DDR2 ????…???§??? */
    ddr2_write_control ddr2_write_ctr_imp(
        .clk_in(ui_clk),
        .rst_n(ui_clk_sync_rst),
        .ADDR_I(addr_i),
        .DATA_I(data_i),
        .STB_I(stb_i),
        .ACK_O(ack_o),
        .read_en(read_enable),
        //ddr_signals
        .app_en(app_en_wr),
        .app_wdf_wren(app_wdf_wren),
        .app_wdf_end(app_wdf_end),
        .app_cmd(app_cmd_wr),
        .app_addr(app_addr_wr),
        .app_wdf_data(app_wdf_data),
        .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy)
    );

    /* DDR2 è???…???§??? */
    ddr2_read_control ddr2_weight_load_ins(
        .clk_in(ui_clk),
        .rst_n(ui_clk_sync_rst),
        .enable(~stb_i),
        .addr_read(addr_i),
        //ddr2_signal
        .app_en(app_en_pe),
        .app_cmd(app_cmd_pe),
        .app_addr(app_addr_pe),
        .app_rd_data(app_rd_data),
        .app_rdy(app_rdy),
        .app_rd_data_end(app_rd_data_end),
        .app_rd_data_valid(app_rd_data_valid)
    );



//MIG IP core
ddr2_ram weight_ddr2_ram (
    // Memory interface ports
    .ddr2_addr                      (ddr2_addr),  // output [12:0]                       ddr2_addr
    .ddr2_ba                        (ddr2_ba),  // output [2:0]                      ddr2_ba
    .ddr2_cas_n                     (ddr2_cas_n),  // output                                       ddr2_cas_n
    .ddr2_ck_n                      (ddr2_ck_n),  // output [0:0]                        ddr2_ck_n
    .ddr2_ck_p                      (ddr2_ck_p),  // output [0:0]                        ddr2_ck_p
    .ddr2_cke                       (ddr2_cke),  // output [0:0]                       ddr2_cke
    .ddr2_ras_n                     (ddr2_ras_n),  // output                                       ddr2_ras_n
    .ddr2_we_n                      (ddr2_we_n),  // output                                       ddr2_we_n
    .ddr2_dq                        (ddr2_dq),  // inout [15:0]                         ddr2_dq
    .ddr2_dqs_n                     (ddr2_dqs_n),  // inout [1:0]                        ddr2_dqs_n
    .ddr2_dqs_p                     (ddr2_dqs_p),  // inout [1:0]                        ddr2_dqs_p
    .init_calib_complete            (init_calib_complete),  // output                                       init_calib_complete

    .ddr2_cs_n                      (ddr2_cs_n),  // output [0:0]           ddr2_cs_n
    .ddr2_dm                        (ddr2_dm),  // output [1:0]                        ddr2_dm
    .ddr2_odt                       (ddr2_odt),  // output [0:0]                       ddr2_odt
    // Application interface ports
    .app_addr                       (app_addr),  // input [26:0]                       app_addr
    .app_cmd                        (app_cmd),  // input [2:0]                                  app_cmd
    .app_en                         (app_en),  // input                                        app_en
    .app_wdf_data                   (app_wdf_data),  // input [127:0]    app_wdf_data
    .app_wdf_end                    (app_wdf_end),  // input                                        app_wdf_end
    .app_wdf_wren                   (app_wdf_wren),  // input                                        app_wdf_wren
    .app_rd_data                    (app_rd_data),  // output [127:0]   app_rd_data
    .app_rd_data_end                (app_rd_data_end),  // output                                       app_rd_data_end
    .app_rd_data_valid              (app_rd_data_valid),  // output                                       app_rd_data_valid
    .app_rdy                        (app_rdy),  // output                                       app_rdy
    .app_wdf_rdy                    (app_wdf_rdy),  // output                                       app_wdf_rdy
    .app_sr_req                     (1'b0),  // input                                        app_sr_req
    .app_ref_req                    (1'b0),  // input                                        app_ref_req
    .app_zq_req                     (1'b0),  // input                                        app_zq_req
    .app_sr_active                  (app_sr_active),  // output                                       app_sr_active
    .app_ref_ack                    (app_ref_ack),  // output                                       app_ref_ack
    .app_zq_ack                     (app_zq_ack),  // output                                       app_zq_ack
    .ui_clk                         (ui_clk),  // output                                       ui_clk
    .ui_clk_sync_rst                (ui_clk_sync_rst),  // output                                       ui_clk_sync_rst

    .app_wdf_mask                   (16'h0000),  // input [15:0]  app_wdf_mask

    // System Clock Ports
    .sys_clk_i                       (clk_in),
    // Reference Clock Ports
    .clk_ref_i                      (clk_ref_i),
    .sys_rst                        (~rst) // input  sys_rst  é??è?¤?????‰???
);
endmodule


//-----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------
