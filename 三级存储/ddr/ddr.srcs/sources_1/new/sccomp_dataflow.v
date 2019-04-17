`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/04/21 20:52:06
// Design Name: 
// Module Name: Dataflow
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module sccomp_dataflow(
input clk_in,
input reset,
input [15:0] sw,
output [7:0] o_seg,
output [7:0] o_sel,


input SD_CD, output SD_RESET, output SD_SCK, output SD_CMD, 
    inout [3:0] SD_DAT, output [15:0] LED,

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
    );


wire [15:0]led;

wire locked;
wire exc;
wire [31:0]status;
   
wire [31:0]rdata;
wire [31:0]wdata;
wire IM_R,DM_CS,DM_R,DM_W;
wire [31:0]inst,pc,addr;
wire inta,intr;

wire [31:0]data_fmem;

wire [31:0]ip_in;
wire seg7_cs,switch_cs;

assign ip_in = pc-32'h00400000;

assign intr = 0;


// wire clk = clk_in;
// wire rst=reset;

wire clk;
wire rst=reset|~locked;

////--------------------debug-------------------------------
clk_wiz_1 clk_inst
  (
   // Clock out ports
   .clk_out1(clk),     // output clk_out1
   // Status and control signals
   .reset(reset), // input reset
   .locked(locked),       // output locked
  // Clock in ports
   .clk_in1(clk_in)); 


wire imem_stuck;
wire imem_ready;
assign LED={led[15:7], ip_in[6:2], imem_stuck, imem_ready};
static_cpu sccpu(clk,reset,inst,rdata,pc,addr,wdata,IM_R,DM_CS,DM_R,DM_W,intr,inta, imem_stuck, imem_ready);
//rdata 从dmem中读取来的数据


/*指令存储器*/
//imem imem(ip_in[12:2],inst);
//imemory im(pc,inst);
//dist_iram_ip IMEM (
//  .a(ip_in[12:2]),      // input wire [10 : 0] a
//  .spo(inst)  // output wire [31 : 0] spo
//);
// sdmem_controller sc(.CLK100MHZ(clk_in), .SD_CD(SD_CD), .SD_RESET(SD_RESET), .SD_SCK(SD_SCK), .SD_CMD(SD_CMD), 
//     .SD_DAT(SD_DAT), .LED(led), .BTNC(rst), .sdmem_adr(ip_in), .data_out(inst), .sd_init(imem_stuck), .sd_ready(imem_ready));
    
    
ddrmem_controller dcl(
    .rst(rst),
    .clk100mhz_in(clk_in),
    .led(led),
    .SD_CD(SD_CD), .SD_RESET(SD_RESET), .SD_SCK(SD_SCK), .SD_CMD(SD_CMD), 
    .SD_DAT(SD_DAT),

    .ddr_init(imem_stuck),
    .ddr_ready(imem_ready),
    .ddrmem_addr_ctl(ip_in),
    .ddrmem_data(inst),
    /*************************/
        .ddr2_dq(ddr2_dq),
        .ddr2_dqs_n(ddr2_dqs_n),
        .ddr2_dqs_p(ddr2_dqs_p),
        .ddr2_addr(ddr2_addr),
        .ddr2_ba(ddr2_ba),
        .ddr2_ras_n(ddr2_ras_n),
        .ddr2_cas_n(ddr2_cas_n),
        .ddr2_we_n(ddr2_we_n),
        .ddr2_ck_p(ddr2_ck_p),
        .ddr2_ck_n(ddr2_ck_n),
        .ddr2_cke(ddr2_cke),
        .ddr2_cs_n(ddr2_cs_n),
        .ddr2_dm(ddr2_dm),
        .ddr2_odt(ddr2_odt)
);

    
wire [31:0]addr_in=addr-32'h10010000;

/*数据存储器*/
dist_dmem_ip DMEM (
  .a(addr_in[12:2]),      // input wire [10 : 0] a
  .d(wdata),      // input wire [31 : 0] d
  .clk(clk),  // input wire clk
  .we(DM_W),    // input wire we
  .spo(data_fmem)  // output wire [31 : 0] spo
);


//dmem scdmem(~clk,reset,DM_CS,DM_W,DM_R,addr-32'h10010000,wdata,data_fmem);


/*地址译码*/
io_sel io_mem(addr, DM_CS, DM_W, DM_R, seg7_cs, switch_cs);

seg7x16 seg7(clk, reset, seg7_cs, wdata, o_seg, o_sel);

sw_mem_sel sw_mem(switch_cs, sw, data_fmem, rdata);
 
    
   
endmodule

module io_sel(
    input [31:0] addr,
   input cs,
   input sig_w,
   input sig_r,
   output seg7_cs,
   output switch_cs
    );

assign seg7_cs = (addr == 32'h10010000 && cs == 1 && sig_w == 1) ? 1 : 0;
assign switch_cs = (addr == 32'h10010010 && cs == 1 && sig_r == 1) ? 1 : 0;
endmodule

module seg7x16(
    input clk,
   input reset,
   input cs,
   input [31:0] i_data,
   output [7:0] o_seg,
   output [7:0] o_sel
    );

    reg [14:0] cnt;
   always @ (posedge clk, posedge reset)
      if (reset)
        cnt <= 0;
      else
        cnt <= cnt + 1'b1;
 
    wire seg7_clk = cnt[14]; 
   
   reg [2:0] seg7_addr;
   
   always @ (posedge seg7_clk, posedge reset)
     if(reset)
      seg7_addr <= 0;
    else
      seg7_addr <= seg7_addr + 1'b1;
      
   reg [7:0] o_sel_r;
   
   always @ (*)
     case(seg7_addr)
      7 : o_sel_r = 8'b01111111;
      6 : o_sel_r = 8'b10111111;
      5 : o_sel_r = 8'b11011111;
      4 : o_sel_r = 8'b11101111;
      3 : o_sel_r = 8'b11110111;
      2 : o_sel_r = 8'b11111011;
      1 : o_sel_r = 8'b11111101;
      0 : o_sel_r = 8'b11111110;
    endcase
  
   reg [31:0] i_data_store;
   always @ (posedge clk, posedge reset)
     if(reset)
      i_data_store <= 0;
    else if(cs)
      i_data_store <= i_data;
      
   reg [7:0] seg_data_r;
   always @ (*)
     case(seg7_addr)
      0 : seg_data_r = i_data_store[3:0];
      1 : seg_data_r = i_data_store[7:4];
      2 : seg_data_r = i_data_store[11:8];
      3 : seg_data_r = i_data_store[15:12];
      4 : seg_data_r = i_data_store[19:16];
      5 : seg_data_r = i_data_store[23:20];
      6 : seg_data_r = i_data_store[27:24];
      7 : seg_data_r = i_data_store[31:28];
    endcase
   
   reg [7:0] o_seg_r;
   always @ (posedge clk, posedge reset)
     if(reset)
      o_seg_r <= 8'hff;
    else
      case(seg_data_r)
        4'h0 : o_seg_r <= 8'hC0;
          4'h1 : o_seg_r <= 8'hF9;
          4'h2 : o_seg_r <= 8'hA4;
          4'h3 : o_seg_r <= 8'hB0;
          4'h4 : o_seg_r <= 8'h99;
          4'h5 : o_seg_r <= 8'h92;
          4'h6 : o_seg_r <= 8'h82;
          4'h7 : o_seg_r <= 8'hF8;
          4'h8 : o_seg_r <= 8'h80;
          4'h9 : o_seg_r <= 8'h90;
          4'hA : o_seg_r <= 8'h88;
          4'hB : o_seg_r <= 8'h83;
          4'hC : o_seg_r <= 8'hC6;
          4'hD : o_seg_r <= 8'hA1;
          4'hE : o_seg_r <= 8'h86;
          4'hF : o_seg_r <= 8'h8E;
      endcase
      
   assign o_sel = o_sel_r;
   assign o_seg = o_seg_r;

endmodule


module sw_mem_sel(
    input switch_cs,
   input [15:0] sw,
   input [31:0] data,
   output [31:0] data_sel
    );

    assign data_sel = (switch_cs) ? {16'b0, sw[15:0]} : data;
endmodule
