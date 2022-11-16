module top( input logic clk, reset,
            output logic [31:0] writedata, dataadr,
            output logic memwrite);

    logic [31:0] pc, instr, readdata;
    // Instantiate processor and memory.
    mips mipsCore(clk, reset, pc, instr, memwrite, dataadr, writedata, readdata);
    instructionMemory insMemory(pc[7:2], instr);
    dataMemory dataMem(clk, memwrite, dataadr, writedata, readdata);

endmodule



module mips(
            input logic clk, reset,
            output logic [31:0] pc,
            input logic [31:0] instr,
            output logic memwrite,
            output logic [31:0] aluout, writedata,
            input logic [31:0] readdata
);

    logic memtoreg, alusrc, regdst, regwrite, jump, pcsrc, zero;
    logic [2:0] alucontrol;



// The control unit receives the current instruction from the datapath and tells the
// datapath how to execute that instruction.
    controlUnit c(instr[31:26], instr[5:0], zero, memtoreg, memwrite, pcsrc,
    alusrc, regdst, regwrite, jump, alucontrol);

// The datapath operates on words of data. It
// contains structures such as memories, registers, ALUs, and multiplexers.
// MIPS is a 32-bit architecture, so we will use a 32-bit datapath. 
    datapath dp(clk, reset, memtoreg, pcsrc, alusrc, regdst, regwrite, jump,
    alucontrol, zero, pc, instr, aluout, writedata, readdata);

endmodule


module controlUnit(
                    input logic [5:0] op, funct,
                    input logic zero,
                    output logic memtoreg, memwrite,
                    output logic pcsrc, alusrc,
                    output logic regdst, regwrite,
                    output logic jump,
                    output logic [2:0] alucontrol
);

    logic [1:0] aluop;
    logic branch;

    mainDecoder md(op, memtoreg, memwrite, branch, alusrc, regdst, regwrite, jump, aluop);
    ALUdecoder ad(funct, aluop, alucontrol);

    assign pcsrc = branch & zero;

endmodule


module mainDecoder( input logic [5:0] op,
                    output logic memtoreg, memwrite,
                    output logic branch, alusrc,
                    output logic regdst, regwrite,
                    output logic jump,
                    output logic [1:0] aluop);

    logic [8:0] controls;
    assign {regwrite, regdst, alusrc, branch, memwrite, memtoreg, jump, aluop} = controls;


    // This comes from the truth table that was created for decoder?
    always_comb
        case(op)
            6'b000000: controls <= 9'b110000010; // RTYPE
            6'b100011: controls <= 9'b101001000; // LW
            6'b101011: controls <= 9'b001010000; // SW
            6'b000100: controls <= 9'b000100001; // BEQ
            6'b001000: controls <= 9'b101000000; // ADDI
            6'b000010: controls <= 9'b000000100; // J
            default: controls <= 9'bxxxxxxxxx;
        endcase


    // always@(*)
    //     case(op)
    //         6'b000000: controls <= 9'b110000010; // RTYPE
    //         6'b100011: controls <= 9'b101001000; // LW
    //         6'b101011: controls <= 9'b001010000; // SW
    //         6'b000100: controls <= 9'b000100001; // BEQ
    //         6'b001000: controls <= 9'b101000000; // ADDI
    //         6'b000010: controls <= 9'b000000100; // J
    //         default: controls <= 9'bxxxxxxxxx;
    //     endcase
endmodule


module ALUdecoder(  input logic [5:0] funct,
                    input logic [1:0] aluop,
                    output logic [2:0] alucontrol);
        

    always_comb
        case(aluop)
            2'b00: alucontrol <= 3'b010; // add (for lw/sw/addi)
            2'b01: alucontrol <= 3'b110; // sub (for beq)
            default: case(funct) // RTYPE instructions
                6'b100000: alucontrol <= 3'b010; // add
                6'b100010: alucontrol <= 3'b110; // sub
                6'b100100: alucontrol <= 3'b000; // and
                6'b100101: alucontrol <= 3'b001; // or
                6'b101010: alucontrol <= 3'b111; // slt
                default: alucontrol <= 3'bxxx; // ???
                endcase
        endcase

        // always@(*)
    //     case(aluop)
    //         2'b00: alucontrol <= 3'b010; // add (for lw/sw/addi)
    //         2'b01: alucontrol <= 3'b110; // sub (for beq)
    //         default: case(funct) // RTYPE instructions
    //             6'b100000: alucontrol <= 3'b010; // add
    //             6'b100010: alucontrol <= 3'b110; // sub
    //             6'b100100: alucontrol <= 3'b000; // and
    //             6'b100101: alucontrol <= 3'b001; // or
    //             6'b101010: alucontrol <= 3'b111; // slt
    //             default: alucontrol <= 3'bxxx; // ???
    //             endcase
    //     endcase

endmodule




module ALU(	input [31:0] A, B, 
            input [2:0] F, 
				output reg [31:0] Y, output Zero);
				
	always@( * )
		case (F[2:0])
			3'b000: Y <= A & B;
			3'b001: Y <= A | B;
			3'b010: Y <= A + B;
			//3'b011: Y <= 0;  // not used
			3'b011: Y <= A & ~B;
			3'b101: Y <= A + ~B;
			3'b110: Y <= A - B;
			3'b111: Y <= A < B ? 1:0;
			default: Y <= 0; //default to 0, should not happen
		endcase
	
	assign Zero = (Y == 32'b0);
endmodule



module datapath(input logic clk, reset,
                input logic memtoreg, pcsrc,
                input logic alusrc, regdst,
                input logic regwrite, jump,
                input logic [2:0] alucontrol,
                output logic zero,
                output logic [31:0] pc,
                input logic [31:0] instr,
                output logic [31:0] aluout, writedata,
                input logic [31:0] readdata
);

    logic [4:0] writereg;
    logic [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
    logic [31:0] signimm, signimmsh;
    logic [31:0] srca, srcb;
    logic [31:0] result;

    // Update PC logic
    flopr#(32) pcreg(clk, reset, pcnext, pc);
    adder pcadd1(pc, 32'b100, pcplus4);
    s12 immsh(signimm, signimmsh);
    adder pcadd2(pcplus4, signimmsh, pcbranch);
    mux2 #(32) pcbrmux(pcplus4, pcbranch, pcsrc, pcnextbr);
    mux2 #(32) pcmux(pcnextbr,{pcplus4[31:28], instr[25:0], 2'b00}, jump, pcnext);

    // Register file logic
    regfile rf(clk, regwrite, instr[25:21], instr[20:16], writereg, result, srca, writedata);
    mux2 #(5) wrmux(instr[20:16], instr[15:11], regdst, writereg);
    mux2 #(32) resmux(aluout, readdata, memtoreg, result);
    signext se(instr[15:0], signimm);

    // ALU logic
    mux2 #(32) scrbmux(writedata, signimm, alusrc, srcb);
    ALU alu(srca, srcb, alucontrol, aluout, zero);


endmodule


// Memory: Register files, data memory, instruction memory

module regfile( input logic clk,
                input logic we3,
                input logic [4:0] ra1, ra2, wa3,
                input logic [31:0] wd3,
                output logic [31:0] rd1, rd2);

    // 32 registers, each 32-bit.
    logic [31:0] rf[31:0];


    //three ported register file
    //read two ports combinationally
    //write third port on rising edge of clk
    //register 0 hardwired to 0
    // note: for pipelined processor, write third port
    // on falling edge of clk


    always_ff@(posedge clk)
        if(we3) rf[wa3] <= wd3;


    // always @(posedge clk)
    //     if (we3) rf[wa3] <= wd3;	
    assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
    assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
    


endmodule



module instructionMemory(   input logic [5:0] a,
                            output logic [31:0] rd);

    logic [31:0] RAM[63:0];

    initial begin
        $readmemh("memfile.dat", RAM);
    end

    assign rd = RAM[a]; // word aligned

endmodule



module dataMemory(  input logic clk, we,
                    input logic [31:0] a, wd,
                    output logic [31:0] rd);

    logic [31:0] RAM [63:0];
    assign rd = RAM[a[31:2]]; // word aligned


    always_ff@(posedge clk)
        if(we) RAM[a[31:2]] <= wd;

    // always@(posedge clk)
    //     if(we) RAM[a[31:2]] <= wd;


endmodule


// Building blocks
module adder(input logic[31:0] a, b,
            output logic [31:0] y);


    assign y = a + b;

endmodule

module s12(input logic [31:0] a,
            output logic [31:0] y);

    // Shift left by 2 (equivalent of multiplication by 4.)
    assign y = {a[29:0], 2'b00};
endmodule


module signext( input logic [15:0] a,
                output logic [31:0] y);
    assign y = {{16{a[15]}}, a};

endmodule


module flopr #(parameter WIDTH = 8)
            (input logic clk, reset,
            input logic [WIDTH-1:0] d,
            output logic [WIDTH-1:0] q);

 
    always_ff@(posedge clk, posedge reset)
        if(reset) q <= 0;
        else q <=d;

    // always@(posedge clk, posedge reset)
    //     if(reset) q <= 0;
    //     else q <=d;

endmodule


module mux2 #(parameter WIDTH = 8)
            (input logic [WIDTH-1:0] d0, d1,
            input logic s,
            output logic[WIDTH-1:0] y);
    
    assign y = s ? d1 : d0;
endmodule