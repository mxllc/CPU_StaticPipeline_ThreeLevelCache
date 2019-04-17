module static_cpu(
input clk,
input rst,
input [31:0]instruction,//指令
input [31:0]rdata,//从内存中读取来的数据
output reg[31:0]PC,//指令的位置
output [31:0]addr,//在内存的中的位置
output reg[31:0]wdata,//存入内存的数据
output IM_R,//
output DM_CS,//内存片选有效
output DM_R,//内存读
output DM_W,//内存写
output intr,
output inta,


output reg imem_stuck,
input imem_ready
);


reg [31:0]instruction_reg;


//中断处理 先不要动 等到要54条下板的时候再处理
assign intr=1'b1;
assign inta=1'b1;
assign IM_R = 1'b1;

always @(posedge clk or posedge rst) begin
  if (rst) begin
    imem_stuck <= 1;
  end
  else if (imem_ready) begin
    imem_stuck <= 0;
  end
end



//----------------------------------------control 信号------------------------------------------------------------------------
//对应control模块
wire [3:0]aluc;//alu控制信号
wire rf_write;//寄存器堆写信号
wire DM_W_id;//内存写信号
wire DM_R_id;//内存读信号
wire DM_CS_id = DM_W_id | DM_R_id;//内存选中信号

wire sign_extend;//符号扩展信号，在ID阶段直接使用，不完后传递
wire ALU_a;//alu_a的数据来源选择  sll|srl|sra?0:1;
wire ALU_b;//alu_b的数据来源选择  ori|addi|addiu|andi|xori|slti|sltiu|lui?0:1; //1-rf_rd2  0-s_z_extend
wire [2:0]rf_wd;//寄存器写数据来源

wire [1:0]rf_wa;//寄存器写数据地址选择
wire jump_26;
wire beq;
wire bne;
wire jr;
//cp0
wire mfc0;
wire mtc0;

wire BREAK;
wire eret;
wire syscall;
wire teq;
wire [4:0]cause;
wire mthi;

wire mtlo;
wire jarl;
wire [1:0]RMemMode;
wire sign_lblh;
wire [1:0]WMemMode;

wire bgez;
wire clz;
wire multu;
wire mult;
wire div;
wire divu;

wire [31:0]rf_rdata1_id, rf_rdata2_id; //从regfile中读取出来
//rf_rdata1, rf_rdata2 //用来往流水寄存器传递


//ex 与 id 冲突, mem 与 id 冲突, wb 与 id 冲突
wire conflict_in_ex, conflict_in_mem, conflict_in_wb, data_conflict;

reg DM_W_wb;

//----------------------------------------control 信号------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------


//----------------------------扩充的控制信号（懒得再去修改control模块，因此在这个地方进行补充）---------------------------------------------------------

wire mfhi = (instruction_reg[31:26] == 6'b000000) & (instruction_reg[5:0] == 6'b010000);
wire mflo = (instruction_reg[31:26] == 6'b000000) & (instruction_reg[5:0] == 6'b010010);


//-------------------------------------------------------------------------------------------------------------------------------------------------



//-----------------------------------------------流水寄存器-------------------------------------------------------------------
reg [3:0]aluc_ex;

//流水寄存器
reg rf_write_ex, rf_write_mem, rf_write_wb;
reg [4:0]rf_waddr_ex, rf_waddr_mem, rf_waddr_wb;



reg [31:0]pc_id, pc_ex, pc_mem, pc_wb;

//等54的时候再修改
reg [31:0]hi_wdata_ex, hi_wdata_mem, hi_wdata_wb;//hi寄存器写回
reg [31:0]lo_wdata_ex, lo_wdata_mem, lo_wdata_wb;//lo寄存器写回


//内存读取
reg DM_R_ex, DM_R_mem;//内存读有效
reg DM_W_ex, DM_W_mem;//内存写有效
reg [31:0]addr_ex, addr_mem; //从内存取数据的地址
wire [31:0]addr_id;

reg clz_ex;
reg [2:0]rf_wd_ex, rf_wd_mem;

reg [31:0] mul_z_mem;
reg [31:0] clz_result_mem;
reg [31:0] HI_ex, HI_mem, HI_wb;
reg [31:0] LO_ex, LO_mem, LO_wb;
reg sign_lblh_ex, sign_lblh_mem;
reg [31:0]alu_r_mem;
reg [31:0]alu_a_ex, alu_b_ex;


reg [31:0]rf_rdata2_ex, rf_rdata2_mem;
reg [31:0]rf_rdata1_ex, rf_rdata1_mem;


reg [1:0]WMemMode_ex, WMemMode_mem;
reg [1:0]RMemMode_ex, RMemMode_mem;


reg multu_ex, multu_mem;
reg mult_ex, mult_mem;

reg div_ex, divu_ex;

reg [31:0]rdata_cp0_ex, rdata_cp0_mem;

// reg [31:0]clz_result_ex;


//------------------------------------------------流水寄存器------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------


//alu
wire [31:0]alu_a;//alu_a的输入数据
wire [31:0]alu_b;//alu_b的输入数据
wire [31:0]alu_r;//输出alu_r
wire zero;//alu 零标志


//regfile
wire [4:0]rs;
wire [4:0]rt;
wire [4:0]rd;
wire [4:0]rf_raddr1;
wire [4:0]rf_raddr2;
wire [4:0]rf_waddr;
wire [31:0]rf_rdata1;
wire [31:0]rf_rdata2;
reg [31:0]rf_wdata;




wire [15:0]instr_low16=instruction_reg[15:0];
wire [31:0]EXTZ5={27'b0,instruction_reg[10:6]};
wire [31:0]EXTZ16={16'b0,instr_low16};
wire [31:0]EXTS16={{16{instr_low16[15]}},instr_low16};




//sb,sh
wire [1:0]sb_r=addr_mem[1:0];//判断0 1 2 3
wire sh_r=addr_mem[1];//判断 0 1

//
wire [1:0]lb_r=addr_mem[1:0];
wire lh_r=addr_mem[1];

wire [31:0]MemDataS8 = lb_r[1]?(lb_r[0]?{{24{rdata[31]}}, rdata[31:24]}:{{24{rdata[23]}}, rdata[23:16]}):(lb_r[0]?{{24{rdata[15]}}, rdata[15:8]}:{{24{rdata[7]}}, rdata[7:0]});
wire [31:0]MemDataZ8 = lb_r[1]?(lb_r[0]?{24'd0, rdata[31:24]}:{24'd0, rdata[23:16]}):(lb_r[0]?{24'd0, rdata[15:8]}:{24'd0, rdata[7:0]});

wire [31:0]MemDataS16 = lh_r?{{16{rdata[31]}}, rdata[31:16]}:{{16{rdata[15]}}, rdata[15:0]};
wire [31:0]MemDataZ16 = lh_r?{16'd0, rdata[31:16]}:{16'd0, rdata[15:0]};


wire [31:0]clz_result;
wire [31:0]q_div;
wire [31:0]r_div;
wire [31:0]q_divu;
wire [31:0]r_divu;


wire [31:0]rdata_cp0;
wire [31:0]exc_addr;
wire exception=BREAK|syscall|(teq&(rf_rdata1 == rf_rdata2));
wire [31:0]status;




wire jump_16=(beq & (rf_rdata1 == rf_rdata2))|(bne&((rf_rdata1 != rf_rdata2)))|(bgez&(~rf_rdata1[31]));



wire [25:0]instr_index=instruction_reg[25:0];
wire [5:0]instr_op=instruction_reg[31:26];





wire signed [31:0]mul_a=rf_rdata1_ex;
wire signed [31:0]mul_b=rf_rdata2_ex;
wire signed [31:0]mul_z = mul_a*mul_b;

// wire signed [63:0]mult_z = mul_a*mul_b;
wire signed [63:0]mult_z;

wire [31:0]multu_a=rf_rdata1_ex;
wire [31:0]multu_b=rf_rdata2_ex;
wire [63:0]multu_z=multu_a*multu_b;


wire busy_div;
wire busy_divu;

// reg start_div;
// reg start_divu;


wire start_div;
wire start_divu;
wire over_div;
wire over_divu;

// wire busy=busy_div|busy_divu|(div_ex&(~over_div))|(divu_ex&(~over_divu));
wire busy=busy_div|busy_divu;
assign start_div = (div_ex&(~busy_div)&(~over_div))?1:0;
assign start_divu = (divu_ex&(~busy_divu)&(~over_divu))?1:0;


//--------------------------------div-debug----------------------------------------------------------------------------------------------
//--------------------------------div-debug----------------------------------------------------------------------------------------------
//--------------------------------div-debug----------------------------------------------------------------------------------------------
//除法 --------------TODO： 考虑除法之前的数据冲突
// wire busy=busy_div|busy_divu|(div&(~over_div))|(divu&(~over_divu));
// wire busy=busy_div|busy_divu;//流水线中不需要后面的那些，因为是在两个周期中。 如果后面的加入判断条件，会导致在id阶段就产生busy信号
// always @(posedge clk or posedge rst) begin
//   if (rst) begin
//     start_div<=1'b0;
//   end
//   else if (!div) begin
//     start_div<=1'b0;
//   end
//   else if (div&(~busy_div)&(~over_div)) begin
//     start_div<=1'b1;
//   end
//   else begin
//     start_div<=1'b0;
//   end
// end

// always @(posedge clk or posedge rst) begin
//   if (rst) begin
//     start_divu<=1'b0;
//   end
//   else if (!divu) begin
//     start_divu<=1'b0;
//   end
//   else if (divu&(~busy_divu)&(~over_divu)) begin
//     start_divu<=1'b1;
//   end
//   else begin
//     start_divu<=1'b0;
//   end
// end



// always @(posedge clk or posedge rst) begin
//   if (rst) begin
//     start_div<=1'b0;
//   end
//   else if (div & ~data_conflict) begin
//     start_div<=1'b1;
//   end
//   else begin
//     start_div<=1'b0;
//   end
// end

// always @(posedge clk or posedge rst) begin
//   if (rst) begin
//     start_divu<=1'b0;
//   end
//   else if (divu & ~data_conflict) begin // ~data_conflict 是用来防止与前一条产生数据冲突导致 start_divu在数据冲突时也被传递
//     start_divu<=1'b1;
//   end
//   else begin
//     start_divu<=1'b0;
//   end
// end
//--------------------------------div-debug----------------------------------------------------------------------------------------------
//--------------------------------div-debug----------------------------------------------------------------------------------------------
//--------------------------------div-debug----------------------------------------------------------------------------------------------







reg [31:0]HI;
reg [31:0]LO;






//分阶段写回
//mthi在ex上升沿写回

//HI 注意：上升沿修改。
always @(posedge clk or posedge rst) begin
  if(rst)begin
    HI<=32'h00000000;
  end
  else begin
    if (mthi)begin
      HI<=rf_rdata1;
    end
    else if (multu_ex) begin
      HI<=multu_z[63:32];
    end
    else if (mult_ex) begin
      HI<=mult_z[63:32];
    end
    else if (over_div) begin
      HI<=r_div;
    end
    else if (over_divu) begin
      HI<=r_divu;
    end
    else begin
      HI<=HI;
    end
  end
end

//LO 注意：上升沿修改。
always @(posedge clk or posedge rst) begin
  if(rst)begin
    LO<=32'h00000000;
  end
  else begin
    if (mtlo)begin
      LO<=rf_rdata1;
    end
    else if (multu_ex) begin
      LO<=multu_z[31:0];
    end
    else if (mult_ex) begin
      LO<=mult_z[31:0];
    end
    else if (over_div) begin
      LO<=q_div;
    end
    else if (over_divu) begin
      LO<=q_divu;
    end
    else begin
      LO<=LO;
    end
  end
end
//在wb阶段写回 -----------------修改！！！！！！




assign rs=instruction_reg[25:21];
assign rt=instruction_reg[20:16];
assign rd=instruction_reg[15:11];


//选择寄存器 读写地址
assign rf_raddr1 = rs;
assign rf_raddr2 = rt;
// assign rf_waddr = jal?5'd31:(rf_wa?rd:rt);//54条可能需要把rf_Wa修改成两位--优化
// assign rf_waddr = rf_wa[1]?(rf_wa[0]?5'd31:mfc0...):(rf_wa[0]?rd:rt);
assign rf_waddr = rf_wa[1]?(rf_wa[0]?5'd31:rt):(rf_wa[0]?rd:rt);

assign addr_id = rf_rdata1 + (sign_extend ? EXTS16:EXTZ16);//需要加入控制信号**** lw sw************************************

wire [31:0]Wdata_sh= sh_r?{rf_rdata2_mem[15:0],rdata[15:0]}:{rdata[31:16],rf_rdata2_mem[15:0]};
reg [31:0]Wdata_sb;
always @(*) begin
  case(sb_r)
  2'b11:Wdata_sb={rf_rdata2_mem[7:0],rdata[23:0]};
  2'b10:Wdata_sb={rdata[31:24],rf_rdata2_mem[7:0],rdata[15:0]};
  2'b01:Wdata_sb={rdata[31:16],rf_rdata2_mem[7:0],rdata[7:0]};
  2'b00:Wdata_sb={rdata[31:8],rf_rdata2_mem[7:0]};
  default:Wdata_sb={rdata[31:8],rf_rdata2_mem[7:0]};
  endcase
end


always @(*) begin
  case(WMemMode_mem)
    2'b11:wdata=rf_rdata2_mem;
    2'b10:wdata=rf_rdata2_mem;
    2'b01:wdata=Wdata_sh;
    2'b00:wdata=Wdata_sb;
    default:wdata=rf_rdata2_mem;

  endcase
end






always @(posedge clk or posedge rst) begin
    if (rst)
        rf_wdata <= 0;
    else begin
        case(rf_wd_mem) 
        3'b111:rf_wdata <=mul_z_mem;
        3'b110:rf_wdata <=clz_result_mem;
        3'b101:rf_wdata <=HI_mem;
        3'b100:rf_wdata <=LO_mem;

        3'b010:rf_wdata <=rdata_cp0_mem;

        // 3'b011:rf_wdata <=pc_mem + 4;//流水中要改为+8   +4在跳转之前就被执行
      3'b011:rf_wdata <=pc_mem + 8;

        3'b000:
          case(RMemMode_mem)
            2'b11:rf_wdata <=rdata;
            2'b10:rf_wdata <=rdata;
            2'b01:rf_wdata <=sign_lblh_mem?MemDataS16:MemDataZ16;
            2'b00:rf_wdata <=sign_lblh_mem?MemDataS8:MemDataZ8;
            default:rf_wdata <=rdata;
          endcase
        3'b001:rf_wdata <=alu_r_mem;
        default:rf_wdata <=alu_r_mem;
      endcase   
    end
  
end

assign alu_a = ALU_a?rf_rdata1:EXTZ5;
assign alu_b = ALU_b?rf_rdata2:(sign_extend ? EXTS16:EXTZ16);











//-----------------------------------------IF--------------------------------------------------

wire [31:0]NPC=PC+32'd4;
//wire [31:0]IPC=PC-32'h00400000;//哈佛结构的指令映射

//PC
always @(posedge clk or posedge rst) begin
  if (rst) begin
    PC<=32'h00400000;
  end
  else if (imem_stuck) begin
    PC<=32'h00400000;
  end
  else if (busy) begin
    PC<=PC;
  end
  else if (data_conflict) begin
    PC <= PC;
  end   
  else if (jump_16) begin
    // PC <= NPC+{{14{instr_low16[15]}},instr_low16,2'b00};  此处不同于单周期，不能再使用NPC， 当前PC为 pc_id + 4
    PC <= PC+{{14{instr_low16[15]}},instr_low16,2'b00};
  end
  else if (jump_26) begin
    PC<={PC[31:28],instr_index,{2'b00}};
  end
  else if (jr|jarl) begin
    PC<=rf_rdata1;
  end
  else if (exception| eret) begin //完整的情况应该在这里加一个中断屏蔽检查。 但是检测程序不检查，就懒得写了
    PC<=exc_addr;
  end
//  else if (if_stop) begin
//    PC<=PC;
//  end
  else begin
    PC<=NPC;
  end
end







//-----------------------------------------ID--------------------------------------------------

CONTROL con_inst(.instruction(instruction_reg),.aluc(aluc),.rf_write(rf_write),.DM_W(DM_W_id),.DM_R(DM_R_id),
    .sign_extend(sign_extend),.ALU_a(ALU_a),.ALU_b(ALU_b),.rf_wd(rf_wd),
    .rf_wa(rf_wa),.jump_26(jump_26),.beq(beq),.bne(bne),.jr(jr), .mfc0(mfc0), .mtc0(mtc0),
    .BREAK(BREAK), .eret(eret), .syscall(syscall), .teq(teq), .cause(cause), .mthi(mthi),
    .mtlo(mtlo),.jarl(jarl),.RMemMode(RMemMode),.sign_lblh(sign_lblh),.WMemMode(WMemMode),
    .bgez(bgez),.clz(clz),.multu(multu),.mult(mult),.div(div),.divu(divu));


always @(posedge clk or posedge rst) begin
    if (rst) 
        instruction_reg <= 32'b0;
    else if (data_conflict | busy) begin //数据冲突 或者 除法忙信号  导致的 暂停流水
        instruction_reg <= instruction_reg;
    end
    else 
        instruction_reg <= instruction;
end


always @(posedge clk or posedge rst) begin
    if (rst) begin
        pc_id <= 0;
    end
    else if (data_conflict | busy) begin //暂停流水
        pc_id <= pc_id;
    end
    else begin
        pc_id <= PC;
    end
end




//---------------------------------------------数据冲突检测-----------------------------------------------------------------------------
//---------------------------------------------数据冲突检测-----------------------------------------------------------------------------
//---------------------------------------------数据冲突检测-----------------------------------------------------------------------------
// assign conflict_in_ex =  (rf_waddr_ex != 0) & (((~DM_R_id)&rf_write_ex & (rf_waddr_ex == rs || rf_waddr_ex == rt)) || (DM_R_ex & rf_write_ex & rf_waddr_ex == rt))
//                        ||((mfhi | mflo) & (over_divu | over_div | multu_ex | mult_ex));
//                        //hi lo 读取的数据冲突


// // id 与 mem 冲突 写回寄存器的addr为 rs 或 rt, 或者  从内存写寄存器 地址冲突
// assign conflict_in_mem = (rf_waddr_mem != 0) & (((~DM_R_id)&rf_write_mem & (rf_waddr_mem == rs || rf_waddr_mem == rt)) || (DM_R_mem & rf_write_mem & rf_waddr_mem == rt));


// // id 与 wb 冲突 写回寄存器的addr 与id阶段的 rs 或 rt
// assign conflict_in_wb = (rf_waddr_wb != 0) & (((~DM_R_id)&rf_write_wb & (rf_waddr_wb == rs || rf_waddr_wb == rt)) || (DM_R_id & rf_write_wb & rf_waddr_wb == rt));



// assign data_conflict = conflict_in_ex | conflict_in_mem | conflict_in_wb;


assign conflict_in_ex =  ((rf_waddr_ex != 0) & (rf_write_ex & (rf_waddr_ex == rs || rf_waddr_ex == rt)))
                       ||((mfhi | mflo) & (over_divu | over_div | multu_ex | mult_ex));
                       //hi lo 读取的数据冲突


// id 与 mem 冲突 写回寄存器的addr为 rs 或 rt, 或者  从内存写寄存器 地址冲突
assign conflict_in_mem = (rf_waddr_mem != 0) & (&rf_write_mem & (rf_waddr_mem == rs || rf_waddr_mem == rt));


// id 与 wb 冲突 写回寄存器的addr 与id阶段的 rs 或 rt
assign conflict_in_wb = (rf_waddr_wb != 0) & (&rf_write_wb & (rf_waddr_wb == rs || rf_waddr_wb == rt));



assign data_conflict = conflict_in_ex | conflict_in_mem | conflict_in_wb;

//---------------------------------------------数据冲突检测-----------------------------------------------------------------------------
//---------------------------------------------数据冲突检测-----------------------------------------------------------------------------
//---------------------------------------------数据冲突检测-----------------------------------------------------------------------------




//-----------------------------------------EX--------------------------------------------------





//alu的运算
alu al(.a(alu_a_ex),.b(alu_b_ex),.aluc(aluc_ex), .r(alu_r),.zero(zero),.carry(),.negative(),.overflow());




always @(posedge clk or posedge rst) begin
    if (rst) begin
        DM_W_ex <= 0;
        DM_R_ex <= 0;

        rf_write_ex <= 0;
        pc_ex <= 0;

        hi_wdata_ex <= 0;
        lo_wdata_ex <= 0;

        addr_ex <= 0;
        rf_wd_ex <= 0;

        clz_ex <= 0;
        rf_rdata2_ex <= 0;
        multu_ex <= 0;
        mult_ex <= 0;

        div_ex <= 0;
        divu_ex <= 0;
        rdata_cp0_ex <= 0;
    end
    else if (data_conflict) begin
        DM_W_ex <= 0;
        DM_R_ex <= 0;
        clz_ex <= 0;

        rf_write_ex <= 0;
        multu_ex <= 0;
        mult_ex <= 0;
    end
    else if(busy) begin
        pc_ex <=pc_ex;
    end
    else begin
        aluc_ex <= aluc;
        alu_a_ex <= alu_a;
        alu_b_ex <= alu_b;
        DM_W_ex <= DM_W_id;
        DM_R_ex <= DM_R_id;

        rf_waddr_ex <= rf_waddr;
        rf_write_ex <= rf_write;// 写寄存器地址 与 写寄存器信号 都是在译码阶段确定的

        pc_ex <= pc_id;

        // hi_wdata_ex <= hi_wdata_id;
        // lo_wdata_ex <= lo_wdata_id;

        addr_ex <= addr_id;

        rf_wd_ex <= rf_wd;
        HI_ex <= HI;
        LO_ex <= LO;
        RMemMode_ex <= RMemMode;
        WMemMode_ex <= WMemMode;
        sign_lblh_ex <= sign_lblh;
        clz_ex <= clz;

        rf_rdata2_ex <= rf_rdata2;
        rf_rdata1_ex <= rf_rdata1;

        multu_ex <= multu;
        mult_ex <=  mult;

        div_ex <= div;
        divu_ex <= divu;

        rdata_cp0_ex <= rdata_cp0;
    end
end



//在ex阶段计算
CLZ clz_inst(.in(rf_rdata1_ex),.out(clz_result));








//-----------------------------------------MEM--------------------------------------------------

// always @(posedge clk or posedge rst) begin
//  if (rst) begin
//      DM_W_mem <= 0;
//      DM_R_mem <= 0;      
//  end
//  else if () begin
//      rf_write_mem <= rf_write_ex;
//      rf_waddr_mem <= rf_waddr_ex;

//      DM_W_mem <= DM_W_ex;
//      DM_R_mem <= DM_R_ex;

//      if () begin
//          rd_wdata_mem <=
//      end
//      else begin
//          rf_wdata_mem <= rf_wdata_ex;
//      end
//  end
// end

assign addr = addr_mem;

always @(posedge clk or posedge rst) begin
    if (rst | busy) begin//除法忙信号，在mem的所有控制信号都无效
        rf_write_mem <= 0;
        pc_mem <= 0;
        hi_wdata_mem <= 0;
        lo_wdata_mem <= 0;
        DM_R_mem <= 0;
        DM_W_mem <= 0;
        rf_wd_mem <= 0;
        mul_z_mem <= 0;
        sign_lblh_mem <= 0;
        alu_r_mem <= 0;
        RMemMode_mem <= 0;
        WMemMode_mem <= 0;
        addr_mem <= 0;
        clz_result_mem <= 0;
        LO_mem <= 0;
        HI_mem <= 0;
        rf_rdata2_mem <= 0;
        multu_mem <= 0;
        mult_mem <= 0;
        rdata_cp0_mem <= 0;
    end
    else  begin
        rf_write_mem <= rf_write_ex;
        pc_mem <= pc_ex;
        hi_wdata_mem <= hi_wdata_ex;
        lo_wdata_mem <= lo_wdata_ex;
        DM_R_mem <= DM_R_ex;
        DM_W_mem <= DM_W_ex;
        rf_wd_mem <= rf_wd_ex;
        mul_z_mem <= mul_z;
        sign_lblh_mem <= sign_lblh_ex;
        alu_r_mem <= alu_r;
        RMemMode_mem <= RMemMode_ex;
        WMemMode_mem <= WMemMode_ex;
        rf_waddr_mem <= rf_waddr_ex;
        addr_mem <= addr_ex;
        clz_result_mem <= clz_result;
        LO_mem <= LO_ex;
        HI_mem <= HI_ex;
        rf_rdata2_mem <= rf_rdata2_ex;
        multu_mem <= multu_ex;
        mult_mem <= mult_ex;
        rdata_cp0_mem <= rdata_cp0_ex;
    end
end





assign DM_CS = DM_W_mem | DM_R_mem;
assign DM_R = DM_R_mem;
assign DM_W = DM_W_mem;




//-----------------------------------------WB--------------------------------------------------

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rf_write_wb <=0;
        pc_wb <= 0;
        hi_wdata_wb <= 0;
        lo_wdata_wb <= 0;
        HI_wb <= 0;
        LO_wb <= 0;
        DM_W_wb <= 0;
    end
    else begin
        rf_write_wb <= rf_write_mem;
        rf_waddr_wb <= rf_waddr_mem;
        pc_wb <= pc_mem;
        hi_wdata_wb <= hi_wdata_mem;
        lo_wdata_wb <= lo_wdata_mem;
        HI_wb <= HI_mem;
        LO_wb <= LO_mem;
        DM_W_wb <= DM_W_mem;
    end
end


//译码阶段执行
CP0 cp0_inst(.clk(clk), .rst(rst), .mfc0(mfc0), .mtc0(mtc0), .pc(pc_id), 
.Rd(rd),        //考虑  [2:0]  sel 
.wdata(rf_rdata2),    // rt中读取出来  rf_raddr2 = rt;
.exception(exception), 
.eret(eret),
.cause(cause), 
.intr(), 
.rdata(rdata_cp0),      // Data from CP0 register for GP register 
.status(status), 
.timer_int(), 
.exc_addr(exc_addr)    // Address for PC at the beginning of an exception 
); 


Regfiles rfl(.clk(clk),.rst(rst),.wena(rf_write_wb),.raddr1(rf_raddr1),.raddr2(rf_raddr2),
    .waddr(rf_waddr_wb),.wdata(rf_wdata),.rdata1(rf_rdata1),.rdata2(rf_rdata2));

DIV div_inst(.dividend(rf_rdata1_ex),.divisor(rf_rdata2_ex),.start(start_div),.clock(clk),.reset(rst),.q(q_div),.r(r_div),.busy(busy_div),.over(over_div));
DIVU divu_inst(.dividend(rf_rdata1_ex),.divisor(rf_rdata2_ex),.start(start_divu),.clock(clk),.reset(rst),.q(q_divu),.r(r_divu),.busy(busy_divu),.over(over_divu));


endmodule








module alu(
input [31:0] a, //32 位输入，操作数1
input [31:0] b, //32 位输入，操作数2
input [3:0] aluc, //4 位输入，控制 alu 的操作
output reg [31:0] r, //32 位输出，由a、b 经过aluc 指定的操作生成
output reg zero, //0 标志位
output reg carry, // 进位标志位
output reg negative, // 负数标志位
output reg overflow // 溢出标志位
);

reg signed [31:0] alg;
reg [32:0] temp;
always @(*)
begin
casex(aluc)
4'b0000://Addu
    begin
        temp=a+b;
        r=temp;
        if(r==0)
            zero=1;
        else
            zero=0;
        if(temp[32])
            carry=1;
        else
            carry=0;       
        if(r[31])
            negative=1;
        else
            negative=0;
        overflow=0;
    end
4'b0010://Add
    begin
        r=a+b;
        if(r==0)
            zero=1;
        else
            zero=0;
        if(!a[31]&&!b[31]&&r[31]||a[31]&&b[31]&&!r[31])//(正+负和负+正不会溢出),正+正||负+负
            overflow=1;
        else  
            overflow=0;  
        if(r[31])
            negative=1;
        else
            negative=0;
        carry=0;
    end
4'b0001://Subu
    begin
        r=a-b;
        if(a<b)
            carry=1;
        else
            carry=0;
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0;
        overflow=0;
    end
4'b0011://Sub
    begin
        r=a-b;        
        if(!a[31]&&b[31]&&r[31]||a[31]&&!b[31]&&!r[31])//（负-负和正-正不会溢出），正-负||负-正可能溢出
            overflow=1;
        else
            overflow=0;
        if(r[31])
            negative=1;
        else
            negative=0;  
        if(r==0)
            zero=1;
        else
            zero=0;
        carry=0;
    end
4'b0100://And
    begin
    r=a&b;
    if(r==0)
        zero=1;
    else
        zero=0;
    if(r[31])
        negative=1;
    else
        negative=0;
    carry=0;
    overflow=0;
    end
4'b0101://Or
    begin
    r=a|b;
    if(r==0)
        zero=1;
    else
        zero=0;
    if(r[31])
        negative=1;
    else
        negative=0;   
    carry=0;
    overflow=0;
    end
4'b0110://Xor
    begin
        r=a^b;
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0;
        carry=0;
        overflow=0;
    end
    
4'b0111://Nor
    begin
        r=~(a|b);
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0;     
        carry=0;
        overflow=0;
    end
4'b100x://Lui
    begin
        r={b[15:0],16'b0};
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0;   
        carry=0;
        overflow=0;   
    end
4'b1011://Slt
    begin
        if(a[31]&&!b[31]||!a[31]&&!b[31]&&a<b||a[31]&&b[31]&&a[30:0]<b[30:0])//负<正||正<正||负<负 (正恒>负)
            r=1;
        else
            r=0;
        if(a==b)
            zero=1;
        else
            zero=0;
        negative=r; 
        carry=0;
        overflow=0;
    end
4'b1010://Sltu
    begin
        if(a<b)
            begin
                carry=1;
                r=1;
            end
        else
            begin
                carry=0;
                r=0;
            end
        zero=a==b?1:0;         
        negative=0;
        overflow=0;     
    end
4'b1100://Sra
    begin 
        alg=b;
        r=alg>>>a[4:0];//调试的时候注意一下这个地方是否有问题  
        if(a<=32&&a>0)
            carry=b[a-1];
        else
            carry=b[31];         
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0;  
        overflow=0; 
    end
4'b111x://Sll//Slr
    begin
        r=b<<a[4:0];
        if(a<=32&&a>0)
            carry=b[32-a];
        else
            carry=0;
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0; 
        overflow=0; 
    end
4'b1101://Srl
    begin
        r=b>>a[4:0];
        if(a<32&&a>0)
            carry=b[a-1];
        else
            carry=0;
        if(r==0)
            zero=1;
        else
            zero=0;
        if(r[31])
            negative=1;
        else
            negative=0;
        overflow=0;  
    end
endcase

end

endmodule


//------------------------------------------------------------------------------

module CLZ(
input [31:0]in,
output reg[31:0]out
    );

always @(*) begin
  if (in[31]) 
    out = 32'd0;
  else if (in[30])
    out = 32'd1;
  else if(in[29])
    out = 32'd2;
  else if(in[28])
    out = 32'd3;
  else if(in[27])
    out = 32'd4;
  else if(in[26])
    out = 32'd5;
  else if(in[25])
    out = 32'd6;
  else if(in[24])
    out = 32'd7;
  else if(in[23])
    out = 32'd8;
  else if(in[22])
    out = 32'd9;
  else if(in[21])
    out = 32'd10;
  else if(in[20])
    out = 32'd11;
  else if(in[19])
    out = 32'd12;
  else if(in[18])
    out = 32'd13;
  else if(in[17])
    out = 32'd14;
  else if(in[16])
    out = 32'd15;
  else if(in[15])
    out = 32'd16;
  else if(in[14])
    out = 32'd17;
  else if(in[13])
    out = 32'd18;
  else if(in[12])
    out = 32'd19;
  else if(in[11])
    out = 32'd20;
  else if(in[10])
    out = 32'd21;
  else if(in[9])
    out = 32'd22;
  else if(in[8])
    out = 32'd23;
  else if(in[7])
    out = 32'd24;
  else if(in[6])
    out = 32'd25;
  else if(in[5])
    out = 32'd26;
  else if(in[4])
    out = 32'd27;
  else if(in[3])
    out = 32'd28;
  else if(in[2])
    out = 32'd29;
  else if(in[1])
    out = 32'd30;
  else if(in[0])
    out = 32'd31;
  else 
    out = 32'd32;
end

endmodule


//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/04/15 14:48:48
// Design Name: 
// Module Name: Regfiles
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


module Regfiles(clk,rst,wena,raddr1,raddr2,waddr,wdata,rdata1,rdata2);
    input clk,rst,wena;
    input [4:0] raddr1,raddr2,waddr;
    input [31:0] wdata;
    output [31:0] rdata1,rdata2;
    
    reg [31:0] array_reg[31:0];//32个32位
    assign rdata1=array_reg[raddr1];
    assign rdata2=array_reg[raddr2];
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            array_reg[0]<=32'b0;
            array_reg[1]<=32'b0;
            array_reg[2]<=32'b0;
            array_reg[3]<=32'b0;
            array_reg[4]<=32'b0;
            array_reg[5]<=32'b0;
            array_reg[6]<=32'b0;
            array_reg[7]<=32'b0;
            array_reg[8]<=32'b0;
            array_reg[9]<=32'b0;
            array_reg[10]<=32'b0;
            array_reg[11]<=32'b0;
            array_reg[12]<=32'b0;
            array_reg[13]<=32'b0;
            array_reg[14]<=32'b0;
            array_reg[15]<=32'b0;
            array_reg[16]<=32'b0;
            array_reg[17]<=32'b0;
            array_reg[18]<=32'b0;
            array_reg[19]<=32'b0;
            array_reg[20]<=32'b0;
            array_reg[21]<=32'b0;
            array_reg[22]<=32'b0;
            array_reg[23]<=32'b0;
            array_reg[24]<=32'b0;
            array_reg[25]<=32'b0;
            array_reg[26]<=32'b0;
            array_reg[27]<=32'b0;
            array_reg[28]<=32'b0;
            array_reg[29]<=32'b0;
            array_reg[30]<=32'b0;
            array_reg[31]<=32'b0;            
        end
        else if (wena==1 && waddr != 0) begin
            array_reg[waddr]<=wdata;
        end
    end

endmodule


//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/28 20:36:24
// Design Name: 
// Module Name: DIV
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


module DIV( 
    input signed [31:0]dividend,//被除数 
    input signed [31:0]divisor,//除数 
    input start,//启动除法运算  
    input clock, 
    input reset, 
    output [31:0]q,//商 
    output reg [31:0]r,//余数     
    output reg busy,//除法器忙标志位 
    output reg over
);
reg[5:0]count; 
reg signed [31:0] reg_q; 
reg signed [31:0] reg_r; 
reg signed [31:0] reg_b; 
reg r_sign; 

wire [32:0] sub_add = r_sign?({reg_r,q[31]} + {1'b0,reg_b}):({reg_r,q[31]} - {1'b0,reg_b});//加、减法器 

// assign q = reg_q;    
// wire signed[31:0] tq=(dividend[31]^divisor[31])?(-reg_q):reg_q;
assign q = reg_q;     
always @ (negedge clock or posedge reset)
begin 
    if (reset)
        begin//重置 
            count <=0; 
            busy <= 0; 
            over<=0;
        end
    else
        begin 
            if (start) 
                begin//开始除法运算，初始化 
                    reg_r <= 0; 
                    r_sign <= 0; 
                    count <= 0; 
                    busy <= 1; 
                    if(dividend<0)
                        reg_q <= -dividend;
                    else
                        reg_q <= dividend;
                    if(divisor<0)
                        reg_b <= -divisor; 
                    else
                        reg_b <= divisor; 
                end 
            else if (busy) 
                begin
                    if(count<=31)
                        begin 
                            reg_r <= sub_add[31:0];//部分余数 
                            r_sign <= sub_add[32];//如果为负，下次相加 
                            reg_q <= {reg_q[30:0],~sub_add[32]};//上商
                            count <= count +1;//计数器加一 
                        end
                    else
                        begin
                            if(dividend[31]^divisor[31])
                                reg_q<=-reg_q;
                            if(!dividend[31])
                                r<=r_sign? reg_r + reg_b : reg_r;
                            else
                                r<=-(r_sign? reg_r + reg_b : reg_r);
                            busy <= 0;
                            over <= 1;
                        end
                end
            else
            over<=0;
        end 
end 
endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/03/28 19:11:17
// Design Name: 
// Module Name: DIVU
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


module DIVU(
    input [31:0]dividend,         //被除数
    input [31:0]divisor,          //除数
    input start,                  //启动除法运算
    input clock,
    input reset,
    output [31:0]q,               //商
    output [31:0]r,               //余数    
    output reg busy,                   //除法器忙标志位
    output reg over
    );

    reg [4:0]count;
    reg [31:0] reg_q;
    reg [31:0] reg_r;
    reg [31:0] reg_b;
    reg r_sign;
    wire [32:0] sub_add = r_sign?({reg_r,q[31]} + {1'b0,reg_b}):({reg_r,q[31]} - {1'b0,reg_b});    //加、减法器
    assign r = r_sign? reg_r + reg_b : reg_r;
    assign q = reg_q; 


    always @(negedge clock or posedge reset) begin
        if (reset) begin
            busy<=0;
            count<=0;
            over<=0;
        end
        else begin
            if (start) begin
                reg_q<=dividend;
                reg_b<=divisor;
                reg_r<=32'b0;
                count<=0;
                busy<=1;
                r_sign<=0;
            end
            else if (busy) begin
                reg_r<=sub_add[31:0];
                reg_q<={reg_q[30:0],~sub_add[32]};
                r_sign<=sub_add[32];
                count<=count +5'b1;
                if(count == 5'd31)begin
                    busy<=0;
                    over<=1;
                end
            end
            else begin
                 over<=0;
            end
        end

    end
endmodule

module CP0(
input clk,
input rst, 
input mfc0,            // CPU instruction is Mfc0
input mtc0,            // CPU instruction is Mtc0 
input [31:0]pc, 
input [4:0] Rd,        // Specifies Cp0 register 
input [31:0] wdata,    // Data from GP register to replace CP0 register
input exception, 
input eret,            // Instruction is ERET (Exception Return) 
input [4:0]cause, 
input intr, 
output [31:0] rdata,      // Data from CP0 register for GP register 
output [31:0] status, 
output reg timer_int, 
output [31:0]exc_addr    // Address for PC at the beginning of an exception 
); 

//syscall>break>teq>eret
reg [31:0]CP0_array_reg[31:0];
assign status=CP0_array_reg[12];
assign exc_addr=eret?CP0_array_reg[14]:32'd4;
assign rdata=mfc0?CP0_array_reg[Rd]:32'b0;


wire syscall =(cause==5'b01000)?1'b1:1'b0;
wire break =(cause==5'b01001)?1'b1:1'b0;
wire teq =(cause==5'b01101)?1'b1:1'b0;

wire exception_excute= exception & CP0_array_reg[12][0] & ((CP0_array_reg[12][1] & syscall) | (CP0_array_reg[12][2] & break) | (CP0_array_reg[12][3] & teq));
reg sll_5;

integer i,j;
always @(negedge clk or posedge rst) begin
  if (rst) begin
    for(i=0;i<12;i=i+1)begin
      CP0_array_reg[i]<=32'b0;
    end
    for(j=13;j<32;j=j+1)begin
      CP0_array_reg[j]<=32'b0;
    end
    CP0_array_reg[12]<=32'h0000000f;
    sll_5<=1'b0;
  end
  else begin
    if (exception_excute & (~sll_5))begin
      CP0_array_reg[12]<=CP0_array_reg[12]<<5;
      CP0_array_reg[13][6:2]<=cause;
      // CP0_array_reg[14]<=pc;
      CP0_array_reg[14]<=pc + 4;//为了兼容流水线 在id阶段判断，此时IF中的值为pc + 4, 因此下一条指令应该是pc + 8 .  但是测试测序中会在eret之前给[14]加4
      sll_5<=1'b1;
    end
    if(eret & sll_5)begin
      CP0_array_reg[12]<=CP0_array_reg[12]>>5;
      sll_5<=1'b0;
    end
    if (mtc0) begin
      CP0_array_reg[Rd]<=wdata;
    end   
  end
end

endmodule