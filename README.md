# Computer-Architecture
Project: Designing a virtual CPU

MODULES FOR INDIVIDUAL UNITS:

PC:
module MIPS_PC(rst, clk, PCin, PCout);
input clk, rst;
input [31:0] PCin;
output reg [31:0] PCout;
always @(posedge clk) begin
if (rst) PCout <= 0;
else PCout <= PCin; //change it to pc=pc+32'd1 if your memory is word address
end
endmodule

Instruction Memory:
module MIPS_IMem(addr, clk, inst);
input [31:0] addr;
input clk;
output reg [31:0] inst;
reg [31:0] mem [255:0];
initial begin
$readmemh("C:\\Users\\lprahara\\Final Project\\MIPS_CPU\\Instructions.txt", mem);
end
always @( negedge clk) begin
inst<=mem[addr[31:0]];
end
endmodule

Register File
module MIPS_RM(read_reg1, read_reg2, write_reg, write_data, clk, read_data1, read_data2,RegWrite);
input [4:0] read_reg1, read_reg2, write_reg;
input [31:0] write_data;
input clk;
input RegWrite;
output [31:0] read_data1, read_data2;
reg [31:0] reg_file [0:31];
assign read_data1 = reg_file[read_reg1];
assign read_data2 = reg_file[read_reg2];
always @(negedge clk) begin
if (RegWrite)
reg_file[write_reg] = write_data;
end
endmodule

ALU Control:
module MIPS_ALUControl(ALUOp, Instruction, ControlOut);
input [1:0] ALUOp;
input [5:0] Instruction;
output reg [3:0] ControlOut;
parameter ALU_Addition = 4'b0010; // Addition
parameter ALU_Subtraction = 4'b0110; // Subtraction
parameter ALU_AND = 4'b0000; // AND
parameter ALU_OR = 4'b0001; // OR
parameter ALU_Set_on_less_than = 4'b0111; // Set on less than

always @(ALUOp) begin
 case(ALUOp)
 2'b00 : ControlOut <= ALU_Addition; // LW, SW instructions
 2'b01 : ControlOut <= ALU_Subtraction; // BEQ insruction
 2'b10 : case (Instruction) // R-type instructions
 6'b100000 : ControlOut <= ALU_Addition;
 6'b100010 : ControlOut <= ALU_Subtraction;
 6'b100100 : ControlOut <= ALU_AND;
 6'b100101 : ControlOut <= ALU_OR;
 6'b101010 : ControlOut <= ALU_Set_on_less_than;
 endcase
 endcase
end
endmodule

ALU:
module MIPS_ALU(
    input [31:0] ALU_A,
    input [31:0] ALU_B,
    input [3:0] ALUop,
    output reg [31:0] ALUout,
    output ALUZero
    );
 
	 assign ALUZero = (ALUout==0); //Zero is true if ALUOut is 0
	 always @(ALUop, ALU_A, ALU_B) begin //reevaluate if these change
	  case (ALUop)
		0: ALUout <= ALU_A & ALU_B;
		1: ALUout <= ALU_A | ALU_B;
		2: ALUout <= ALU_A + ALU_B;
		6: ALUout <= ALU_A - ALU_B;
		7: ALUout <= ALU_A < ALU_B ? ALU_B : 0;
		12: ALUout <= ~(ALU_A | ALU_B); // result is nor
		default: ALUout <= 0;
	  endcase
    end
endmodule

Control Unit:
module MIPS_CU(op, reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch, alu_op);
input [5:0] op;
output reg reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch;
output reg [1:0] alu_op;
always @(op) begin
case (op)
6'b000000: begin //R-type
{reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch} <=7'b1001000;
alu_op<=2'b10;
end
6'b100011: begin //lw
{reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch} <=7'b0111100;
alu_op<=2'b00;
end
6'b101011: begin //sw
{reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch} <=7'bx1x0010;
alu_op<=2'b00;
end
6'b000100: begin //beq
{reg_dst, alu_src, mem_to_reg, reg_write, mem_read, mem_write, branch} <=7'bx0x0001;
alu_op<=2'b01;
end
endcase
end
endmodule

Multiplexor[4:0]

module MIPS_RegDstMUX(RegDst, RegDstMUXin0, RegDstMUXin1, RegDstMUXout);
input RegDst;
input [4:0] RegDstMUXin0, RegDstMUXin1;
output reg [4:0] RegDstMUXout;
always @(RegDst, RegDstMUXin0, RegDstMUXin1) begin //reevaluate if these change
   case (RegDst)
   0: RegDstMUXout <= RegDstMUXin0 ;
   1: RegDstMUXout <= RegDstMUXin1;
   endcase
   end
endmodule

Multiplexor[31:0]

module MIPS_MUX (mux_sel, mux_in0, mux_in1, mux_out );
    input mux_sel;
    input [31:0] mux_in0,mux_in1;
    output reg [31:0] mux_out;
    always @(mux_sel, mux_in0, mux_in1) begin //reevaluate if these change
    case (mux_sel)
    0: mux_out <= mux_in0 ;
    1: mux_out <= mux_in1;
    endcase
    end
 endmodule

Adder:
module MIPSADDER(
    input [31:0] Adder_A,
    input [31:0] Adder_B,
    output [31:0] AdderResult
    );
assign AdderResult=Adder_A+Adder_B;   
endmodule

Data Memory:
module MIPS_DM(
    input [31:0] addr,
    input [31:0] writedata,
    output reg [31:0] dataread,
    input clk,
    input MemWrite,
    input MemRead
    );
    reg [31:0] data_mem [0:256];
 
    always @(negedge clk) begin
    if (MemWrite)
    data_mem[addr] = writedata;
    end
    always @(posedge clk) begin
    if (MemRead)
    dataread=data_mem[addr];
    end
    endmodule

AND :
module MIPS_AND(Branch, ALUZero, BranchAndZeroOut);
   input [0:0] Branch;
   input [0:0] ALUZero;
   output [0:0] BranchAndZeroOut;
   assign BranchAndZeroOut = ALUZero & Branch;
endmodule
	
Module for the CPU:
module MIPS_CPU(clk, rst);
//Inputs
input clk;
input rst;
//Interconnections
//For PC:
wire [31:0] output_pc;
//For Inst
wire [31:0] instr;
// Instruction is split in to different fields
wire [5:0] operation_code, funct_field;
wire [4:0] source_reg_1, source_reg_2, dest_reg, shift_amount;
wire [16:0] immediate;
wire [31:0] extended_immediate, branch_offset;
wire [31:0] pcsrc_mux_out, memtoreg_mux_out, alu_src_mux_out;
wire [4:0] regdst_mux_out;
// Control Signals
wire CU_regwrite, CU_ALU_source, CU_alu_zero, CU_regdst, CU_branch, CU_memread, CU_memtoreg, CU_memwrite, CU_PCSource;
wire [1:0] CU_aluop;
wire [3:0] alu_operation; // From ALU control
wire [31:0] reg_file_data_1, reg_file_data_2, alu_output, data_mem_out, pc_adder_out, 
branch_pc_calc_out;
assign operation_code = instr[31:26];
assign source_reg_1 = instr[25:21];
assign source_reg_2 = instr[20:16];
assign dest_reg = instr[15:11];
assign shift_amount = instr[10:6]; //Unused right now
assign funct_field = instr[5:0];
assign immediate = instr[15:0];
assign extended_immediate = { {16{immediate[15]}}, immediate};
//assign branch_offset = extended_immediate << 2;
assign branch_offset = extended_immediate;

MIPS_PC pc_0 (
     // inputs
     .rst ( rst ),
     .clk ( clk ),
     .PCin (pcsrc_mux_out ),
     // outputs
     .PCout (output_pc) );	

MIPS_IMem instructionmemory_0 (
      // inputs
      .addr ( output_pc),
      .clk ( clk ),
      // outputs
      .inst ( instr )
      ); 
MIPS_RM registermemory_0 (
       // inputs
       .read_reg1 ( source_reg_1 ),
       .read_reg2 ( source_reg_2 ),
       .write_reg ( regdst_mux_out ),
       .write_data ( memtoreg_mux_out ),
       .clk ( clk ),
       .RegWrite (CU_regwrite ),
       // outputs
       .read_data1 (reg_file_data_1 ),
       .read_data2 ( reg_file_data_2)
         );
MIPS_ALUControl alucontrol_0 (
       // inputs
      .ALUOp ( CU_aluop ),
      .Instruction ( funct_field ),
      // outputs
      .ControlOut ( alu_operation )
       ); 
MIPS_ALU alu_0 (
       // inputs
       .ALU_A ( reg_file_data_1),
       .ALU_B ( alu_src_mux_out),
       .ALUop ( alu_operation),
       // outputs
      .ALUZero ( CU_alu_zero),
      .ALUout ( alu_output )
      );

MIPS_CU control_0(
        //inputs
    .op(operation_code),
	//outputs
	.reg_dst(CU_regdst),
	.branch(CU_branch),
	.mem_read(CU_memread),
	.mem_write(CU_memwrite),
	.mem_to_reg(CU_memtoreg),
	.alu_src(CU_ALU_source),
	.reg_write(CU_regwrite),
	.alu_op(CU_aluop)         
	);
 MIPS_RegDstMUX regdstmux_0 (
       //inputs
       .RegDst ( CU_regdst ),
       .RegDstMUXin0 ( source_reg_2 ),
       .RegDstMUXin1 ( dest_reg ),
       //outputs
       .RegDstMUXout ( regdst_mux_out )
       );
MIPS_MUX alusrcmux_0 (
         // inputs
         .mux_sel ( CU_ALU_source),
         .mux_in0 ( reg_file_data_2 ),
         .mux_in1 ( extended_immediate ),
         // outputs
         .mux_out ( alu_src_mux_out )
         ); 
MIPS_MUX memtoregmux_0 (
          // inputs
          .mux_sel ( CU_memtoreg ),
          .mux_in0 ( alu_output ),
          .mux_in1 ( data_mem_out ),
          // outputs
          .mux_out ( memtoreg_mux_out )
          ); 
MIPS_MUX branchmux_0 (
          // inputs
          .mux_sel ( CU_PCSource ),
          .mux_in0 ( pc_adder_out ),
          .mux_in1 ( branch_pc_calc_out ),
          // outputs
          .mux_out ( pcsrc_mux_out )
          );

MIPSADDER PC_ADDER(
//inputs
 .Adder_A (output_pc),
 .Adder_B(1),
//outputs
 .AdderResult(pc_adder_out));

MIPSADDER BRANCH_PC_CALCULATOR(
//inputs
 .Adder_A (pc_adder_out),
.Adder_B(branch_offset),
//outputs
.AdderResult(branch_pc_calc_out)
);

MIPS_DM datamemory_0 (
         // inputs
        .addr ( alu_output ),
        .writedata ( reg_file_data_2 ),
        .clk ( clk ),
        .MemRead ( CU_memread ),
        .MemWrite ( CU_memwrite ),
        // outputs
        .dataread ( data_mem_out )
        );  
MIPS_AND andgate_0 (
        // inputs
        .Branch ( CU_branch),
        .ALUZero ( CU_alu_zero),
        //outputs
         .BranchAndZeroOut ( CU_PCSource )
         );
Endmodule
Test Bench for the Module CPU:
module MIPS_CPU_TB ;
//Inputs
reg clk;
reg rst;
// Instantiate the Unit Under Test (UUT)
MIPS_CPU uut (
.clk(clk),
.rst(rst)
);
  initial begin
     clk = 0;
     forever #50 clk = !clk;
      end
  initial begin         
      uut.registermemory_0.reg_file[0] = 0;
      uut.registermemory_0.reg_file[1] = 5;
      uut.registermemory_0.reg_file[2] = 10;
      uut.datamemory_0.data_mem[5] = 20;
      rst = 1;
      #100;
      rst = 0;
      end
endmodule
