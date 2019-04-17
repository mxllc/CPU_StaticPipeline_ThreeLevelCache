`timescale 1ns / 1ps
`define READ 1'b0
`define WRITE 1'b1
`define NoBusy 1'b0
`define Busy 1'b1
`define DONE 1'b1
`define UNDONE 1'b0

//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2018/12/18 00:17:54
// Design Name:
// Module Name: ddrmem_controller
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


module ddrmem_controller (
    input rst,
    input clk100mhz_in,
    output [15:0]led,
    input SD_CD, output SD_RESET, output SD_SCK, output SD_CMD, 
    inout [3:0] SD_DAT,

    input ddr_init,
    output ddr_ready,
    input [31:0]ddrmem_addr_ctl,
    output [31:0]ddrmem_data,
    /*************************/
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



    reg seg7_cs;
    reg [31:0]seg_data_i;
    wire clk200mhz;

    wire [1:0] rdqs_n;

    wire [127:0]data_in;

    wire [31:0] ddrmem_data_out;


    // seg7x16 seg7(clk100mhz_in, rst, seg7_cs,seg_data_i, o_seg, o_sel);
    // seg7x16 seg7(clk100mhz_in, rst, 1'b1, ddrmem_data_out, o_seg, o_sel);

    assign ddrmem_data = ddrmem_data_out;

    reg read_write;

// module labkit(input CLK100MHZ, input SD_CD, output SD_RESET, output SD_SCK, output SD_CMD, 
//     inout [3:0] SD_DAT, output [15:0] LED, input .BTNC(rst));

reg sd_init;//发送信号使得sd初始化
wire sd_ready;//sd初始化完成信号
wire [31:0]sd_data;//sd读进来的数据
reg [31:0]sdmem_adr;//读取sdmem数据所用的地址

wire [15:0]LED;

sdmem_controller sdc(.CLK100MHZ(clk100mhz_in), .SD_CD(SD_CD), .SD_RESET(SD_RESET), .SD_SCK(SD_SCK), .SD_CMD(SD_CMD), 
    .SD_DAT(SD_DAT), .LED(LED), .BTNC(rst), .sdmem_adr(sdmem_adr), .data_out(sd_data), .sd_init(sd_init), .sd_ready(sd_ready));



assign data_in = {96'h0, sd_data};
    reg [31:0]addr_in_ddr;
    reg [31:0]addr_in_ddr_mem;
    wire [31:0]adr_in_ddr;
    wire [127:0]data_from_DDR;
    wire ddr_busy;
    wire ddr_done;
    // wire temp_read_write,temp_ack;
    sealedDDR sealedDDR_0(
        .clk100mhz(clk100mhz_in),
        .rst(rst),
        .addr_to_DDR(adr_in_ddr),
        .data_to_DDR(data_in),
        .read_write(read_write),
        .data_from_DDR(data_from_DDR),
        .busy(ddr_busy),
        .done(ddr_done),
        /************************/
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

//debug
// wire [1:0]pos_in_128=sw[15:14];


reg [3:0] state;

parameter STATE_IDLE = 0;
parameter STATE_WRITE_WAIT = 1;//等待sd初始化
parameter STATE_WRITE_PRE = 2;
parameter STATE_WRITE = 3;
parameter STATE_WAIT = 4;

reg first_write;
reg [10:0]bytes_cnt;//2048次

assign adr_in_ddr = (state == STATE_WAIT)?addr_in_ddr_mem:addr_in_ddr;

always @(posedge clk100mhz_in or posedge rst) begin
    if (rst) begin
        state <= STATE_IDLE;
        first_write <= 1;
        sdmem_adr <= 0;
        addr_in_ddr <= 0;
        bytes_cnt <= 0;
    end
    else begin
        case(state)
            STATE_IDLE:begin
                if(ddr_init)begin
                    sd_init <= 1;
                    state <= STATE_WRITE_WAIT;
                end
            end
            STATE_WRITE_WAIT:begin
                if (sd_ready) begin
                    state <= STATE_WRITE;
                    sd_init <= 0;
                end
            end
            // STATE_WRITE_PRE:begin

            //     if (first_write) begin
            //         first_write <= 0;

            //     end
            //     else begin
                    
            //     end

            // end
            STATE_WRITE:begin
                if (ddr_done) begin
                    if (first_write) begin//第一次写ddr，
                        first_write <= 0;
                        sdmem_adr <= 0;
                        read_write <= 1;
                        addr_in_ddr <= 0;
                        state <= STATE_WRITE;
                        bytes_cnt <= bytes_cnt + 1;
                    end
                    else begin
                        if (bytes_cnt == 11'd2047) begin
                            bytes_cnt <= 0;
                            state <= STATE_WAIT;
                            read_write <= 0;
                            sdmem_adr <= 0;
                            addr_in_ddr <= 0;
                        end
                        else begin
                            bytes_cnt <= bytes_cnt + 1;
                            state <= STATE_WRITE;
                            read_write <= 1;
                            sdmem_adr <= sdmem_adr + 4;
                            addr_in_ddr <= addr_in_ddr + 8;
                        end
                    end
                end
            end
            STATE_WAIT:begin
                state <= STATE_WAIT;
                addr_in_ddr<=0;
            end
        endcase
    end
end

reg [3:0]ddrmem_state;

reg ddrmem_we;
reg [31:0]ddrmem_data_in;
reg [31:0]ddrmem_addr;
wire [31:0]ddrmem_adr;
assign  ddrmem_adr = (ddrmem_state == STATE_WAIT)?ddrmem_addr_ctl:ddrmem_addr;

assign ddr_ready = (ddrmem_state == STATE_WAIT)?1'b1:1'b0;

reg ddrmem_first_write;

reg [10:0]ddrmem_byte_cnt;//2048

    /*数据存储器*/
dist_ddrmem_ip DDRMEM (
      .a(ddrmem_adr[12:2]),      // input wire [10 : 0] a
      .d(ddrmem_data_in),      // input wire [31 : 0] d
      .clk(clk100mhz_in),  // input wire clk
      .we(ddrmem_we),    // input wire we
      .spo(ddrmem_data_out)  // output wire [31 : 0] spo
    );

always @(posedge clk100mhz_in or posedge rst) begin
    if (rst) begin
        ddrmem_state <= STATE_IDLE;
        ddrmem_first_write <= 1;
        ddrmem_we <= 0;
        ddrmem_data_in <= 0;
        ddrmem_byte_cnt <= 0;
        addr_in_ddr_mem <= 0;
        ddrmem_addr <= 0;
    end
    else begin
         case(ddrmem_state)
            STATE_IDLE:begin
                if (state == STATE_WAIT) begin
                    ddrmem_state <= STATE_WRITE;
                    addr_in_ddr_mem <= 0;
                    ddrmem_addr <= 0 - 4;
                    ddrmem_byte_cnt <= 0;
                end
            end
            STATE_WRITE:begin
                if (ddr_done) begin
                    ddrmem_data_in <= data_from_DDR[31:0];
                    if (ddrmem_byte_cnt == 11'd2047) begin
                        ddrmem_state <= STATE_WAIT;
                        ddrmem_we <= 0;
                        ddrmem_addr <= 0;
                        addr_in_ddr_mem <= 0;
                        ddrmem_byte_cnt <= 0;
                    end
                    else begin
                        ddrmem_state <= STATE_WRITE;
                        ddrmem_we <= 1;
                        ddrmem_addr <= ddrmem_addr + 4;
                        addr_in_ddr_mem <= addr_in_ddr_mem + 8;
                        ddrmem_byte_cnt <= ddrmem_byte_cnt + 1;
                    end
                end
                else begin
                    ddrmem_we <= 0;
                end
            end
            STATE_WAIT:begin
                ddrmem_state <= STATE_WAIT;
                ddrmem_we <= 0;
            end

        endcase

    end
end



assign led = {LED[15:4], ddrmem_state};

endmodule

//---------------------------------------------------------------------------------------------------------
//sdmem_controller-----------------------------------------------------------------------------------------
module sdmem_controller(input CLK100MHZ, input SD_CD, output SD_RESET, output SD_SCK, output SD_CMD, 
    inout [3:0] SD_DAT, output [15:0] LED, input BTNC, input [31:0] sdmem_adr, output [31:0] data_out, input sd_init, output sd_ready);

//input [31:0] sdmem_adr, input read_write, output [31:0] data_out,
// wire [31:0]sdmem_adr= {16'b0, sw};
// wire [31:0]data_out;
// wire sd_ready;

labkit sdlab(.CLK100MHZ(CLK100MHZ), .SD_CD(SD_CD), .SD_RESET(SD_RESET), .SD_SCK(SD_SCK), .SD_CMD(SD_CMD), .SD_DAT(SD_DAT), .LED(LED), .BTNC(BTNC), 
    .sdmem_adr(sdmem_adr), .data_out(data_out), .sd_init(sd_init), .sd_ready(sd_ready));


// seg7x16 seg7(CLK100MHZ, BTNC, 1'b1, data_out, o_seg, o_sel);
endmodule


// Be sure to enable SD_CD, SD_RESET, SD_SCK, SD_CMD, and SD_DAT in the
// constraints file.
module labkit(input CLK100MHZ, input SD_CD, output SD_RESET, output SD_SCK, output SD_CMD, 
    inout [3:0] SD_DAT, output [15:0] LED, input BTNC, input [31:0] sdmem_adr, output [31:0] data_out, input sd_init, output sd_ready);

    

    // Clock the SD card at 25 MHz.
    wire clk_100mhz = CLK100MHZ;
    wire clk_50mhz;
    wire clk_25mhz;
    clock_divider div1(clk_100mhz, clk_50mhz);
    clock_divider div2(clk_50mhz, clk_25mhz);

    wire rst = BTNC;
    wire spiClk;
    wire spiMiso;
    wire spiMosi;
    wire spiCS;

    // MicroSD SPI/SD Mode/Nexys 4
    // 1: Unused / DAT2 / SD_DAT[2]
    // 2: CS / DAT3 / SD_DAT[3]
    // 3: MOSI / CMD / SD_CMD
    // 4: VDD / VDD / ~SD_RESET
    // 5: SCLK / SCLK / SD_SCK
    // 6: GND / GND / - 
    // 7: MISO / DAT0 / SD_DAT[0]
    // 8: UNUSED / DAT1 / SD_DAT[1]
    assign SD_DAT[2] = 1;
    assign SD_DAT[3] = spiCS;
    assign SD_CMD = spiMosi;
    assign SD_RESET = 0;
    assign SD_SCK = spiClk;
    assign spiMiso = SD_DAT[0];
    assign SD_DAT[1] = 1;
    
    reg rd = 0;
    reg wr = 0;
    reg [7:0] din = 0;
    wire [7:0] dout;
    wire byte_available;
    wire ready;
    wire ready_for_next_byte;
    reg [31:0] adr; //02_00是第二个扇区
    
    reg [31:0] bytes = 0;
    reg [1:0] bytes_read = 0;
    
    wire [4:0] state;
    
    parameter STATE_INIT = 0;
    parameter STATE_START = 1;
    parameter STATE_WRITE = 2;
    parameter STATE_READ = 3;
    parameter STATE_START_READ = 4;
    parameter STATE_WAIT = 5;

    reg [2:0] test_state = STATE_INIT; 
    assign LED = {state, ready, test_state, bytes[15:9]};
    
    sd_controller sdcont(.cs(spiCS), .mosi(spiMosi), .miso(spiMiso),
            .sclk(spiClk), .rd(rd), .wr(wr), .reset(rst),
            .din(din), .dout(dout), .byte_available(byte_available),
            .ready(ready), .address(adr), 
            .ready_for_next_byte(ready_for_next_byte), .clk(clk_25mhz), 
            .status(state));
    


    reg next_byte;
    reg read_bytes_ready;//读出4字节
    reg write_bytes_next;//需要下一个4字节
    wire [31:0]write_bytes;//外界送进来的4字节
    reg [1:0]byte_cnt;

    reg [4:0]sector_cnt;//16个
    reg [4:0]sector_cnt_rd_sd;//16个

//-------------------------------------------------------------------
    reg [31:0]addr_sdmem;
    reg we;
    reg we_first;
    reg sd_write_nxt_first;

    always @(posedge clk_100mhz or posedge rst) begin
        if (rst) begin
            we <= 0;
            we_first <= 1;
        end
        else if (read_bytes_ready && we_first) begin
            we <= 1;
            we_first <= 0;
        end
        else if (read_bytes_ready == 0)begin
            we_first <= 1;
            we <= 0;
        end
        else begin
            we <= 0;
        end
    end

    always @(posedge clk_100mhz or posedge rst) begin
        if (rst) begin
            sd_write_nxt_first <= 1;            
        end
        else if (write_bytes_next && sd_write_nxt_first) begin
            sd_write_nxt_first <= 0;
        end
        else if (write_bytes_next == 0) begin
            sd_write_nxt_first <= 1;
        end
    end

    always @(posedge clk_100mhz or posedge rst) begin
        if (rst) begin
            addr_sdmem <= 4;
        end
        else if (rd && sector_cnt_rd_sd == 0) begin//sd取出，存入mem
            addr_sdmem <= 0 - 4;
        end
        else if (read_bytes_ready && we_first) begin
            addr_sdmem <= addr_sdmem + 4;
        end
        else if (wr && sector_cnt == 0) begin//mem取出，存入sd
            // addr_sdmem <= 32'h02_04;
            addr_sdmem <= 32'h00_00;
        end
        else if (write_bytes_next && sd_write_nxt_first) begin
            addr_sdmem <= addr_sdmem + 4;
        end
        else if (test_state == STATE_WAIT)begin//处于读sdmem阶段
            addr_sdmem <= sdmem_adr;
        end
    end

    wire seg7_cs = (test_state == STATE_INIT||test_state == STATE_READ)?1:0;
    wire [31:0]seg_data = (test_state == STATE_INIT)?write_bytes:bytes;

   
wire [31:0]addr_sdmem_in;
assign addr_sdmem_in = (test_state == STATE_WAIT)?sdmem_adr:addr_sdmem;

    /*数据存储器*/
    dist_sdmem_ip SDMEM (
      .a(addr_sdmem_in[12:2]),      // input wire [10 : 0] a
      .d(bytes),      // input wire [31 : 0] d
      .clk(clk_100mhz),  // input wire clk
      .we(we),    // input wire we
      .spo(write_bytes)  // output wire [31 : 0] spo
    );
//---------------------------------------------------------------
assign data_out = write_bytes;

assign sd_ready = (test_state == STATE_WAIT)?1:0;


reg only_one;
reg flag_wr_adr_ready;
reg flag_rd_adr_ready;
reg flag_srd_adr_ready;


    always @(posedge clk_25mhz or posedge rst) begin
        if(rst) begin
            bytes <= 32'h12_34_56_78;
            bytes_read <= 0;
            din <= 0;
            wr <= 0;
            rd <= 0;
            next_byte <= 0;
            read_bytes_ready <= 0;
            write_bytes_next <= 0;
            byte_cnt <=0;
            test_state <= STATE_INIT; 
            only_one <= 1;
            sector_cnt <= 0;
            // adr = 32'h00_01_FA_00; //02_00是第二个扇区
            adr = 32'h00_00_00_00; //02_00是第二个扇区
            flag_wr_adr_ready <= 0;
            flag_rd_adr_ready <= 0;
            flag_srd_adr_ready <= 0;
            sector_cnt_rd_sd <= 0;
        end
        else begin
            case (test_state)
                STATE_INIT: begin
                    // if(ready) begin
                    //     test_state <= STATE_START;
                    //     wr <= 1;
                    //     din <= 8'h34;
                    //     // din <= 8'hAA;
                    // end
                    adr <= 32'h00_00_00_00;
                    // if(ready && sd_init) begin
                    if(ready && sd_init ) begin//-----------------------------------------------------------------------------------------
                        //------------写SD开始
                        // test_state <= STATE_START;
                        // // rd <= 1;
                        // // adr <= 32'h00_00_00_00;
                        // wr <= 1;
                        // only_one <= 0;
                        //-------------------------

                        test_state <= STATE_START_READ;
                        rd <= 1;
                        only_one <= 0;

                    end
                end
                STATE_START_READ: begin
                    if(ready == 0) begin
                        test_state <= STATE_READ;
                    end  
                end

                STATE_READ: begin
                    if(ready) begin                    
                        // if (adr == 0) begin
                        //     adr <= 32'h00_00_04_00;
                        // end
                        // else begin
                        //     // test_state <= STATE_START;
                        //     test_state <= STATE_INIT;
                        //     // wr <= 1;  
                        // end

                        if (sector_cnt_rd_sd == 15) begin
                            test_state <= STATE_WAIT;
                            sector_cnt_rd_sd <= 0;
                        end
                        else begin
                            if (flag_srd_adr_ready == 0) begin
                                adr <= adr + 32'h00_00_02_00;
                                flag_srd_adr_ready <= 1;
                            end
                            begin
                                rd <= 1;
                                test_state <= STATE_START_READ;
                                sector_cnt_rd_sd <= sector_cnt_rd_sd + 1;
                                flag_srd_adr_ready <= 0;
                            end

                        end

                    end
                    if(byte_available) begin
                        rd <= 0;
                        if(bytes_read == 0) begin
                            bytes_read <= 1;
                            bytes[31:24] <= dout;
                        end
                        else if(bytes_read == 1) begin
                            bytes_read <= 2;
                            bytes[23:16] <= dout;                  
                        end
                        else if(bytes_read == 2) begin
                            bytes_read <= 3;
                            bytes[15:8] <= dout;
                        end
                        else if(bytes_read == 3) begin
                            bytes_read <= 4;
                            bytes[7:0] <= dout;
                            read_bytes_ready <= 1; //集齐4字节，向ddr发出信号，告之可读
                        end
                        else begin
                            read_bytes_ready <= 0;
                        end
                    end
                    else begin
                        read_bytes_ready <= 0;
                    end                    
                end

                STATE_WAIT:begin
                    test_state <= STATE_WAIT;
                end
            endcase
        end
    end
endmodule

module clock_divider(input clk_in, output reg clk_out = 0);
    always @(posedge clk_in) begin
        clk_out <= ~clk_out;
    end
endmodule

//-----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------
/* SD Card controller module. Allows reading from and writing to a microSD card
through SPI mode. */
module sd_controller(
    output reg cs, // Connect to SD_DAT[3].
    output mosi, // Connect to SD_CMD.
    input miso, // Connect to SD_DAT[0].
    output sclk, // Connect to SD_SCK.
                // For SPI mode, SD_DAT[2] and SD_DAT[1] should be held HIGH. 
                // SD_RESET should be held LOW.

    input rd,   // Read-enable. When [ready] is HIGH, asseting [rd] will 
                // begin a 512-byte READ operation at [address]. 
                // [byte_available] will transition HIGH as a new byte has been
                // read from the SD card. The byte is presented on [dout].
    output reg [7:0] dout, // Data output for READ operation.
    output reg byte_available, // A new byte has been presented on [dout].

    input wr,   // Write-enable. When [ready] is HIGH, asserting [wr] will
                // begin a 512-byte WRITE operation at [address].
                // [ready_for_next_byte] will transition HIGH to request that
                // the next byte to be written should be presentaed on [din].
    input [7:0] din, // Data input for WRITE operation.
    output reg ready_for_next_byte, // A new byte should be presented on [din].

    input reset, // Resets controller on assertion.
    output ready, // HIGH if the SD card is ready for a read or write operation.
    input [31:0] address,   // Memory address for read/write operation. This MUST 
                            // be a multiple of 512 bytes, due to SD sectoring.
    input clk,  // 25 MHz clock.
    output [4:0] status // For debug purposes: Current state of controller.
);

    parameter RST = 0;
    parameter INIT = 1;
    parameter CMD0 = 2;
    parameter CMD55 = 3;
    parameter CMD41 = 4;
    parameter POLL_CMD = 5;
    
    parameter IDLE = 6;
    parameter READ_BLOCK = 7;
    parameter READ_BLOCK_WAIT = 8;
    parameter READ_BLOCK_DATA = 9;
    parameter READ_BLOCK_CRC = 10;
    parameter SEND_CMD = 11;
    parameter RECEIVE_BYTE_WAIT = 12;
    parameter RECEIVE_BYTE = 13;
    parameter WRITE_BLOCK_CMD = 14;
    parameter WRITE_BLOCK_INIT = 15;
    parameter WRITE_BLOCK_DATA = 16;
    parameter WRITE_BLOCK_BYTE = 17;
    parameter WRITE_BLOCK_WAIT = 18;
    
    parameter WRITE_DATA_SIZE = 515;
    
    reg [4:0] state = RST;
    assign status = state;
    reg [4:0] return_state;
    reg sclk_sig = 0;
    reg [55:0] cmd_out;
    reg [7:0] recv_data;
    reg cmd_mode = 1;
    reg [7:0] data_sig = 8'hFF;
    
    reg [9:0] byte_counter;
    reg [9:0] bit_counter;
    
    reg [26:0] boot_counter = 27'd100_000_000;
    always @(posedge clk) begin
        if(reset == 1) begin
            state <= RST;
            sclk_sig <= 0;
            boot_counter <= 27'd100_000_000;
        end
        else begin
            case(state)
                RST: begin
                    if(boot_counter == 0) begin
                        sclk_sig <= 0;
                        cmd_out <= {56{1'b1}};
                        byte_counter <= 0;
                        byte_available <= 0;
                        ready_for_next_byte <= 0;
                        cmd_mode <= 1;
                        bit_counter <= 160;
                        cs <= 1;
                        state <= INIT;
                    end
                    else begin
                        boot_counter <= boot_counter - 1;
                    end
                end
                INIT: begin
                    if(bit_counter == 0) begin
                        cs <= 0;
                        state <= CMD0;
                    end
                    else begin
                        bit_counter <= bit_counter - 1;
                        sclk_sig <= ~sclk_sig;
                    end
                end
                CMD0: begin
                    cmd_out <= 56'hFF_40_00_00_00_00_95;
                    bit_counter <= 55;
                    return_state <= CMD55;
                    state <= SEND_CMD;
                end
                CMD55: begin
                    cmd_out <= 56'hFF_77_00_00_00_00_01;
                    bit_counter <= 55;
                    return_state <= CMD41;
                    state <= SEND_CMD;
                end
                CMD41: begin
                    cmd_out <= 56'hFF_69_00_00_00_00_01;
                    bit_counter <= 55;
                    return_state <= POLL_CMD;
                    state <= SEND_CMD;
                end
                POLL_CMD: begin
                    if(recv_data[0] == 0) begin
                        state <= IDLE;
                    end
                    else begin
                        state <= CMD55;
                    end
                end
                IDLE: begin
                    if(rd == 1) begin
                        state <= READ_BLOCK;
                    end
                    else if(wr == 1) begin
                        state <= WRITE_BLOCK_CMD;
                    end
                    else begin
                        state <= IDLE;
                    end
                end
                READ_BLOCK: begin
                    cmd_out <= {16'hFF_51, address, 8'hFF};
                    bit_counter <= 55;
                    return_state <= READ_BLOCK_WAIT;
                    state <= SEND_CMD;
                end
                READ_BLOCK_WAIT: begin
                    if(sclk_sig == 1 && miso == 0) begin
                        byte_counter <= 511;
                        bit_counter <= 7;
                        return_state <= READ_BLOCK_DATA;
                        state <= RECEIVE_BYTE;
                    end
                    sclk_sig <= ~sclk_sig;
                end
                READ_BLOCK_DATA: begin
                    dout <= recv_data;
                    byte_available <= 1;
                    if (byte_counter == 0) begin
                        bit_counter <= 7;
                        return_state <= READ_BLOCK_CRC;
                        state <= RECEIVE_BYTE;
                    end
                    else begin
                        byte_counter <= byte_counter - 1;
                        return_state <= READ_BLOCK_DATA;
                        bit_counter <= 7;
                        state <= RECEIVE_BYTE;
                    end
                end
                READ_BLOCK_CRC: begin
                    bit_counter <= 7;
                    return_state <= IDLE;
                    state <= RECEIVE_BYTE;
                end
                SEND_CMD: begin
                    if (sclk_sig == 1) begin
                        if (bit_counter == 0) begin
                            state <= RECEIVE_BYTE_WAIT;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                            cmd_out <= {cmd_out[54:0], 1'b1};
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_BYTE_WAIT: begin
                    if (sclk_sig == 1) begin
                        if (miso == 0) begin
                            recv_data <= 0;
                            bit_counter <= 6;
                            state <= RECEIVE_BYTE;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                RECEIVE_BYTE: begin
                    byte_available <= 0;
                    if (sclk_sig == 1) begin
                        recv_data <= {recv_data[6:0], miso};
                        if (bit_counter == 0) begin
                            state <= return_state;
                        end
                        else begin
                            bit_counter <= bit_counter - 1;
                        end
                    end
                    sclk_sig <= ~sclk_sig;
                end
                WRITE_BLOCK_CMD: begin
                    cmd_out <= {16'hFF_58, address, 8'hFF};
                    bit_counter <= 55;
                    return_state <= WRITE_BLOCK_INIT;
                    state <= SEND_CMD;
            ready_for_next_byte <= 1;
                end
                WRITE_BLOCK_INIT: begin
                    cmd_mode <= 0;
                    byte_counter <= WRITE_DATA_SIZE; 
                    state <= WRITE_BLOCK_DATA;
                    ready_for_next_byte <= 0;
                end
                WRITE_BLOCK_DATA: begin
                    if (byte_counter == 0) begin
                        state <= RECEIVE_BYTE_WAIT;
                        return_state <= WRITE_BLOCK_WAIT;
                    end
                    else begin
                        if ((byte_counter == 2) || (byte_counter == 1)) begin
                            data_sig <= 8'hFF;
                        end
                        else if (byte_counter == WRITE_DATA_SIZE) begin
                            data_sig <= 8'hFE;
                        end
                        else begin
                            data_sig <= din;
                            ready_for_next_byte <= 1;
                        end
                        bit_counter <= 7;
                        state <= WRITE_BLOCK_BYTE;
                        byte_counter <= byte_counter - 1;
                    end
                end
                WRITE_BLOCK_BYTE: begin
                    if (sclk_sig == 1) begin
                        if (bit_counter == 0) begin
                            state <= WRITE_BLOCK_DATA;
                            ready_for_next_byte <= 0;
                        end
                        else begin
                            data_sig <= {data_sig[6:0], 1'b1};
                            bit_counter <= bit_counter - 1;
                        end;
                    end;
                    sclk_sig <= ~sclk_sig;
                end
                WRITE_BLOCK_WAIT: begin
                    if (sclk_sig == 1) begin
                        if (miso == 1) begin
                            state <= IDLE;
                            cmd_mode <= 1;
                        end
                    end
                    sclk_sig = ~sclk_sig;
                end
            endcase
        end
    end

    assign sclk = sclk_sig;
    assign mosi = cmd_mode ? cmd_out[55] : data_sig[7];
    assign ready = (state == IDLE);
endmodule
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------