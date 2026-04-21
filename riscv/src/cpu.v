module cpu(
    input  wire                 clk_in,
    input  wire                 rst_in,
    input  wire                 rdy_in,
    input  wire [ 7:0]          mem_din,
    output reg  [ 7:0]          mem_dout,
    output reg  [31:0]          mem_a,
    output reg                  mem_wr,
    input  wire                 io_buffer_full,
    output wire [31:0]          dbgreg_dout
);

    // Memory Controller State
    reg [2:0] mc_state;
    localparam MC_IDLE = 0, MC_READ = 1, MC_WRITE = 2;
    reg [2:0] mc_cnt, mc_target;
    reg [31:0] mc_addr, mc_data_reg;
    reg mc_is_if;
    reg mc_if_done, mc_lsb_done;
    reg [31:0] mc_if_data, mc_lsb_rdata;

    // IF State
    reg [31:0] pc, fetch_pc;
    reg fetching;
    reg [15:0] if_buf[0:7];
    reg [3:0] if_head, if_tail, if_count;
    reg inst_valid;
    reg [31:0] inst, inst_pc;

    // ROB State
    localparam ROB_SIZE = 16;
    reg rob_busy[0:ROB_SIZE-1], rob_ready[0:ROB_SIZE-1], rob_mispredict[0:ROB_SIZE-1];
    reg [31:0] rob_val[0:ROB_SIZE-1], rob_pc[0:ROB_SIZE-1], rob_target_pc[0:ROB_SIZE-1];
    reg [4:0] rob_rd[0:ROB_SIZE-1];
    reg [3:0] rob_head, rob_tail, rob_count;

    // RF State
    reg [31:0] rf_val[0:31];
    reg [3:0] rf_rob_id[0:31];
    reg rf_busy[0:31];

    // RS State
    localparam RS_SIZE = 16;
    reg rs_busy[0:RS_SIZE-1], rs_has_q1[0:RS_SIZE-1], rs_has_q2[0:RS_SIZE-1];
    reg [5:0] rs_op[0:RS_SIZE-1];
    reg [31:0] rs_v1[0:RS_SIZE-1], rs_v2[0:RS_SIZE-1], rs_imm[0:RS_SIZE-1], rs_pc[0:RS_SIZE-1];
    reg [3:0] rs_q1[0:RS_SIZE-1], rs_q2[0:RS_SIZE-1], rs_rob_id[0:RS_SIZE-1];
    reg rs_is_rv32c[0:RS_SIZE-1];

    // LSB State
    localparam LSB_SIZE = 16;
    reg lsb_busy[0:LSB_SIZE-1], lsb_has_q1[0:LSB_SIZE-1], lsb_has_q2[0:LSB_SIZE-1], lsb_addr_ready[0:LSB_SIZE-1];
    reg [5:0] lsb_op[0:LSB_SIZE-1];
    reg [31:0] lsb_v1[0:LSB_SIZE-1], lsb_v2[0:LSB_SIZE-1], lsb_imm[0:LSB_SIZE-1], lsb_addr_val[0:LSB_SIZE-1];
    reg [3:0] lsb_q1[0:LSB_SIZE-1], lsb_q2[0:LSB_SIZE-1], lsb_rob_id[0:LSB_SIZE-1];
    reg [3:0] lsb_head, lsb_tail, lsb_count;

    // CDB
    reg cdb_valid, cdb_mispredict;
    reg [3:0] cdb_rob_id;
    reg [31:0] cdb_val, cdb_target_pc;

    // RV32C Decoder Function
    function [31:0] decode_c;
        input [15:0] c;
        reg [31:0] i;
        begin
            i = 32'h00000013;
            case (c[1:0])
                2'b00: case (c[15:13])
                    3'b000: i = {2'b00, c[10:7], c[12:11], c[5], c[6], 2'b00, 5'd2, 3'b000, 2'b01, c[4:2], 7'b0010011};
                    3'b010: i = {5'b0, c[5], c[12:10], c[6], 2'b00, 2'b01, c[9:7], 3'b010, 2'b01, c[4:2], 7'b0000011};
                    3'b110: i = {5'b0, c[5], c[12], 2'b01, c[4:2], 2'b01, c[9:7], 3'b010, c[11:10], c[6], 2'b00, 7'b0100011};
                endcase
                2'b01: case (c[15:13])
                    3'b000: i = {{7{c[12]}}, c[12], c[6:2], c[11:7], 3'b000, c[11:7], 7'b0010011};
                    3'b001: i = {c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], c[12], {8{c[12]}}, 5'd1, 7'b1101111};
                    3'b010: i = {{7{c[12]}}, c[12], c[6:2], 5'd0, 3'b000, c[11:7], 7'b0010011};
                    3'b011: if (c[11:7] == 5'd2) i = {{3{c[12]}}, c[12], c[4:3], c[5], c[2], c[6], 4'b0, 5'd2, 3'b000, 5'd2, 7'b0010011};
                            else i = {{15{c[12]}}, c[12], c[6:2], c[11:7], 7'b0110111};
                    3'b100: case (c[11:10])
                        2'b00: i = {7'b0, c[6:2], 2'b01, c[9:7], 3'b101, 2'b01, c[9:7], 7'b0010011};
                        2'b01: i = {7'b0100000, c[6:2], 2'b01, c[9:7], 3'b101, 2'b01, c[9:7], 7'b0010011};
                        2'b10: i = {{7{c[12]}}, c[12], c[6:2], 2'b01, c[9:7], 3'b111, 2'b01, c[9:7], 7'b0010011};
                        2'b11: case ({c[12], c[6:5]})
                            3'b000: i = {7'b0100000, 2'b01, c[4:2], 2'b01, c[9:7], 3'b000, 2'b01, c[9:7], 7'b0110011};
                            3'b001: i = {7'b0, 2'b01, c[4:2], 2'b01, c[9:7], 3'b100, 2'b01, c[9:7], 7'b0110011};
                            3'b010: i = {7'b0, 2'b01, c[4:2], 2'b01, c[9:7], 3'b110, 2'b01, c[9:7], 7'b0110011};
                            3'b011: i = {7'b0, 2'b01, c[4:2], 2'b01, c[9:7], 3'b111, 2'b01, c[9:7], 7'b0110011};
                        endcase
                    endcase
                    3'b101: i = {c[12], c[8], c[10:9], c[6], c[7], c[2], c[11], c[5:3], c[12], {8{c[12]}}, 5'd0, 7'b1101111};
                    3'b110: i = {{4{c[12]}}, c[12], c[6:5], 5'd0, 2'b01, c[9:7], 3'b000, c[11:10], c[4:3], c[12], 7'b1100011};
                    3'b111: i = {{4{c[12]}}, c[12], c[6:5], 5'd0, 2'b01, c[9:7], 3'b001, c[11:10], c[4:3], c[12], 7'b1100011};
                endcase
                2'b10: case (c[15:13])
                    3'b000: i = {7'b0, c[6:2], c[11:7], 3'b001, c[11:7], 7'b0010011};
                    3'b010: i = {4'b0, c[3:2], c[12], c[6:4], 2'b00, 5'd2, 3'b010, c[11:7], 7'b0000011};
                    3'b100: if (c[12] == 0) if (c[6:2] == 0) i = {12'b0, c[11:7], 3'b000, 5'd0, 7'b1100111};
                                           else i = {7'b0, c[6:2], 5'd0, 3'b000, c[11:7], 7'b0110011};
                            else if (c[6:2] == 0) i = {12'b0, c[11:7], 3'b000, 5'd1, 7'b1100111};
                                 else i = {7'b0, c[6:2], c[11:7], 3'b000, c[11:7], 7'b0110011};
                    3'b110: i = {4'b0, c[8:7], c[12], c[6:2], 5'd2, 3'b010, c[11:9], 2'b00, 7'b0100011};
                endcase
            endcase
            decode_c = i;
        end
    endfunction

    integer i, j;
    reg [4:0] rs1, rs2, rd; reg [31:0] imm; reg [5:0] op; reg [31:0] d_inst;
    reg [3:0] free_rs; reg rs_found;

    assign dbgreg_dout = rf_val[10];

    always @(posedge clk_in) begin
        if (rst_in) begin
            mc_state <= MC_IDLE; mem_a <= 0; mem_wr <= 0; mc_if_done <= 0; mc_lsb_done <= 0;
            pc <= 0; fetch_pc <= 0; fetching <= 0; if_head <= 0; if_tail <= 0; if_count <= 0; inst_valid <= 0;
            rob_head <= 0; rob_tail <= 0; rob_count <= 0;
            for (i=0; i<ROB_SIZE; i=i+1) begin rob_busy[i] <= 0; rob_ready[i] <= 0; end
            for (i=0; i<32; i=i+1) begin rf_val[i] <= 0; rf_busy[i] <= 0; end
            for (i=0; i<RS_SIZE; i=i+1) rs_busy[i] <= 0;
            lsb_head <= 0; lsb_tail <= 0; lsb_count <= 0;
            for (i=0; i<LSB_SIZE; i=i+1) lsb_busy[i] <= 0;
            cdb_valid <= 0;
        end else if (rdy_in) begin
            // Misprediction Recovery
            if (rob_busy[rob_head] && rob_ready[rob_head] && rob_mispredict[rob_head]) begin
                pc <= rob_target_pc[rob_head]; fetch_pc <= rob_target_pc[rob_head];
                fetching <= 0; if_head <= 0; if_tail <= 0; if_count <= 0; inst_valid <= 0;
                rob_head <= 0; rob_tail <= 0; rob_count <= 0;
                for (i=0; i<ROB_SIZE; i=i+1) begin rob_busy[i] <= 0; rob_ready[i] <= 0; end
                for (i=0; i<32; i=i+1) rf_busy[i] <= 0;
                for (i=0; i<RS_SIZE; i=i+1) rs_busy[i] <= 0;
                lsb_head <= 0; lsb_tail <= 0; lsb_count <= 0;
                for (i=0; i<LSB_SIZE; i=i+1) lsb_busy[i] <= 0;
                cdb_valid <= 0;
                mc_state <= MC_IDLE; mem_wr <= 0; mc_if_done <= 0; mc_lsb_done <= 0;
            end else begin
                // 1. Memory Controller
                case (mc_state)
                    MC_IDLE: begin
                        mc_if_done <= 0; mc_lsb_done <= 0;
                        if (lsb_count > 0 && !lsb_addr_ready[lsb_head] && !lsb_has_q1[lsb_head]) begin
                            lsb_addr_val[lsb_head] <= lsb_v1[lsb_head] + lsb_imm[lsb_head]; lsb_addr_ready[lsb_head] <= 1;
                        end else if (lsb_count > 0 && lsb_addr_ready[lsb_head] && (lsb_op[lsb_head] < 16 || lsb_rob_id[lsb_head] == rob_head)) begin
                            mc_addr <= lsb_addr_val[lsb_head];
                            mc_target <= (lsb_op[lsb_head] == 6'd13 || lsb_op[lsb_head] == 6'd18) ? 4 : (lsb_op[lsb_head] == 6'd12 || lsb_op[lsb_head] == 6'd15 || lsb_op[lsb_head] == 6'd17) ? 2 : 1;
                            if (lsb_op[lsb_head] >= 16) begin
                                if (!io_buffer_full || lsb_addr_val[lsb_head][17:16] != 2'b11) begin
                                    mc_state <= MC_WRITE; mem_a <= lsb_addr_val[lsb_head]; mem_dout <= lsb_v2[lsb_head][7:0]; mem_wr <= 1; mc_cnt <= 1;
                                end
                            end else begin
                                mc_state <= MC_READ; mem_a <= lsb_addr_val[lsb_head]; mem_wr <= 0; mc_cnt <= 0;
                            end
                        end else if (!fetching && if_count <= 4) begin
                            mc_addr <= fetch_pc & ~32'h3; mc_target <= 4; mc_state <= MC_READ; mem_a <= fetch_pc & ~32'h3; mem_wr <= 0; mc_cnt <= 0; mc_is_if <= 1; fetching <= 1;
                        end
                    end
                    MC_READ: begin
                        if (mc_cnt > 0) mc_data_reg[((mc_cnt-1)*8)+:8] <= mem_din;
                        if (mc_cnt == mc_target) begin
                            mc_state <= MC_IDLE; mem_a <= 0;
                            if (mc_is_if) begin mc_if_done <= 1; mc_if_data <= {mc_data_reg[23:0], mem_din}; mc_is_if <= 0; end
                            else begin mc_lsb_done <= 1; mc_lsb_rdata <= {mem_din, mc_data_reg[23:0]}; end
                        end else begin mem_a <= mc_addr + mc_cnt + 1; mc_cnt <= mc_cnt + 1; end
                    end
                    MC_WRITE: begin
                        if (mc_cnt == mc_target) begin mc_state <= MC_IDLE; mem_a <= 0; mem_wr <= 0; mc_lsb_done <= 1; end
                        else begin mem_a <= mc_addr + mc_cnt; mem_dout <= lsb_v2[lsb_head][(mc_cnt*8)+:8]; mc_cnt <= mc_cnt + 1; end
                    end
                endcase

                // 2. IF
                if (mc_if_done) begin
                    fetching <= 0;
                    if ((fetch_pc & 32'h2) == 0) begin if_buf[if_tail] <= mc_if_data[15:0]; if_buf[(if_tail+1)%8] <= mc_if_data[31:16]; if_tail <= (if_tail + 2) % 8; if_count <= if_count + 2; fetch_pc <= fetch_pc + 4; end
                    else begin if_buf[if_tail] <= mc_if_data[31:16]; if_tail <= (if_tail + 1) % 8; if_count <= if_count + 1; fetch_pc <= fetch_pc + 2; end
                end
                if (!inst_valid && if_count > 0) begin
                    if (if_buf[if_head][1:0] == 2'b11) begin if (if_count >= 2) begin inst <= {if_buf[(if_head+1)%8], if_buf[if_head]}; inst_pc <= pc; inst_valid <= 1; end end
                    else begin inst <= {16'b0, if_buf[if_head]}; inst_pc <= pc; inst_valid <= 1; end
                end

                // 3. Commit
                if (rob_count > 0 && rob_ready[rob_head]) begin
                    if (rob_rd[rob_head] != 0) begin rf_val[rob_rd[rob_head]] <= rob_val[rob_head]; if (rf_busy[rob_rd[rob_head]] && rf_rob_id[rob_rd[rob_head]] == rob_head) rf_busy[rob_rd[rob_head]] <= 0; end
                    rob_busy[rob_head] <= 0; rob_head <= (rob_head + 1) % ROB_SIZE; rob_count <= rob_count - 1;
                end

                // 4. CDB Update
                if (cdb_valid) begin
                    for (i=0; i<RS_SIZE; i=i+1) if (rs_busy[i]) begin if (rs_has_q1[i] && rs_q1[i] == cdb_rob_id) begin rs_v1[i] <= cdb_val; rs_has_q1[i] <= 0; end if (rs_has_q2[i] && rs_q2[i] == cdb_rob_id) begin rs_v2[i] <= cdb_val; rs_has_q2[i] <= 0; end end
                    for (i=0; i<LSB_SIZE; i=i+1) if (lsb_busy[i]) begin if (lsb_has_q1[i] && lsb_q1[i] == cdb_rob_id) begin lsb_v1[i] <= cdb_val; lsb_has_q1[i] <= 0; end if (lsb_has_q2[i] && lsb_q2[i] == cdb_rob_id) begin lsb_v2[i] <= cdb_val; lsb_has_q2[i] <= 0; end end
                    if (rob_busy[cdb_rob_id]) begin rob_val[cdb_rob_id] <= cdb_val; rob_ready[cdb_rob_id] <= 1; rob_target_pc[cdb_rob_id] <= cdb_target_pc; rob_mispredict[cdb_rob_id] <= cdb_mispredict; end
                end

                // 5. Execute RS
                cdb_valid <= 0; rs_found = 0;
                for (i=0; i<RS_SIZE; i=i+1) if (rs_busy[i] && !rs_has_q1[i] && !rs_has_q2[i] && !rs_found) begin
                    rs_busy[i] <= 0; cdb_valid <= 1; cdb_rob_id <= rs_rob_id[i]; cdb_mispredict <= 0; rs_found = 1;
                    case (rs_op[i])
                        6'd1: cdb_val <= rs_imm[i]; // LUI
                        6'd2: cdb_val <= rs_pc[i] + rs_imm[i]; // AUIPC
                        6'd3: begin cdb_val <= rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_target_pc <= rs_pc[i] + rs_imm[i]; cdb_mispredict <= 1; end // JAL
                        6'd4: begin cdb_val <= rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_target_pc <= (rs_v1[i] + rs_imm[i]) & ~32'h1; cdb_mispredict <= 1; end // JALR
                        6'd5: begin cdb_val <= 0; cdb_target_pc <= (rs_v1[i] == rs_v2[i]) ? rs_pc[i] + rs_imm[i] : rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_mispredict <= (rs_v1[i] == rs_v2[i]); end // BEQ
                        6'd6: begin cdb_val <= 0; cdb_target_pc <= (rs_v1[i] != rs_v2[i]) ? rs_pc[i] + rs_imm[i] : rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_mispredict <= (rs_v1[i] != rs_v2[i]); end // BNE
                        6'd7: begin cdb_val <= 0; cdb_target_pc <= ($signed(rs_v1[i]) < $signed(rs_v2[i])) ? rs_pc[i] + rs_imm[i] : rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_mispredict <= ($signed(rs_v1[i]) < $signed(rs_v2[i])); end // BLT
                        6'd8: begin cdb_val <= 0; cdb_target_pc <= ($signed(rs_v1[i]) >= $signed(rs_v2[i])) ? rs_pc[i] + rs_imm[i] : rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_mispredict <= ($signed(rs_v1[i]) >= $signed(rs_v2[i])); end // BGE
                        6'd9: begin cdb_val <= 0; cdb_target_pc <= (rs_v1[i] < rs_v2[i]) ? rs_pc[i] + rs_imm[i] : rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_mispredict <= (rs_v1[i] < rs_v2[i]); end // BLTU
                        6'd10: begin cdb_val <= 0; cdb_target_pc <= (rs_v1[i] >= rs_v2[i]) ? rs_pc[i] + rs_imm[i] : rs_pc[i] + (rs_is_rv32c[i] ? 2 : 4); cdb_mispredict <= (rs_v1[i] >= rs_v2[i]); end // BGEU
                        6'd19: cdb_val <= rs_v1[i] + rs_imm[i]; // ADDI
                        6'd20: cdb_val <= ($signed(rs_v1[i]) < $signed(rs_imm[i])) ? 1 : 0; // SLTI
                        6'd21: cdb_val <= (rs_v1[i] < rs_imm[i]) ? 1 : 0; // SLTIU
                        6'd22: cdb_val <= rs_v1[i] ^ rs_imm[i]; // XORI
                        6'd23: cdb_val <= rs_v1[i] | rs_imm[i]; // ORI
                        6'd24: cdb_val <= rs_v1[i] & rs_imm[i]; // ANDI
                        6'd25: cdb_val <= rs_v1[i] << rs_imm[i][4:0]; // SLLI
                        6'd26: cdb_val <= rs_v1[i] >> rs_imm[i][4:0]; // SRLI
                        6'd27: cdb_val <= $signed(rs_v1[i]) >>> rs_imm[i][4:0]; // SRAI
                        6'd28: cdb_val <= rs_v1[i] + rs_v2[i]; // ADD
                        6'd29: cdb_val <= rs_v1[i] - rs_v2[i]; // SUB
                        6'd30: cdb_val <= rs_v1[i] << rs_v2[i][4:0]; // SLL
                        6'd31: cdb_val <= ($signed(rs_v1[i]) < $signed(rs_v2[i])) ? 1 : 0; // SLT
                        6'd32: cdb_val <= (rs_v1[i] < rs_v2[i]) ? 1 : 0; // SLTU
                        6'd33: cdb_val <= rs_v1[i] ^ rs_v2[i]; // XOR
                        6'd34: cdb_val <= rs_v1[i] >> rs_v2[i][4:0]; // SRL
                        6'd35: cdb_val <= $signed(rs_v1[i]) >>> rs_v2[i][4:0]; // SRA
                        6'd36: cdb_val <= rs_v1[i] | rs_v2[i]; // OR
                        6'd37: cdb_val <= rs_v1[i] & rs_v2[i]; // AND
                        default: cdb_val <= 0;
                    endcase
                end

                // 6. Execute LSB
                if (mc_lsb_done) begin
                    lsb_busy[lsb_head] <= 0; lsb_head <= (lsb_head + 1) % LSB_SIZE; lsb_count <= lsb_count - 1;
                    if (lsb_op[lsb_head] < 16) begin
                        cdb_valid <= 1; cdb_rob_id <= lsb_rob_id[lsb_head]; cdb_mispredict <= 0;
                        case (lsb_op[lsb_head])
                            6'd11: cdb_val <= {{24{mc_lsb_rdata[7]}}, mc_lsb_rdata[7:0]};
                            6'd12: cdb_val <= {{16{mc_lsb_rdata[15]}}, mc_lsb_rdata[15:0]};
                            6'd13: cdb_val <= mc_lsb_rdata;
                            6'd14: cdb_val <= {24'b0, mc_lsb_rdata[7:0]};
                            6'd15: cdb_val <= {16'b0, mc_lsb_rdata[15:0]};
                        endcase
                    end else begin
                        rob_ready[lsb_rob_id[lsb_head]] <= 1;
                    end
                end

                // 7. Issue
                if (inst_valid && rob_count < ROB_SIZE) begin
                    d_inst = (inst[1:0] == 2'b11) ? inst : decode_c(inst[15:0]);
                    rs1 = d_inst[19:15]; rs2 = d_inst[24:20]; rd = d_inst[11:7]; op = 0; imm = 0;
                    case (d_inst[6:0])
                        7'b0110111: begin op = 6'd1; imm = {d_inst[31:12], 12'b0}; end
                        7'b0010111: begin op = 6'd2; imm = {d_inst[31:12], 12'b0}; end
                        7'b1101111: begin op = 6'd3; imm = {{12{d_inst[31]}}, d_inst[19:12], d_inst[20], d_inst[30:21], 1'b0}; end
                        7'b1100111: begin op = 6'd4; imm = {{20{d_inst[31]}}, d_inst[31:20]}; end
                        7'b1100011: begin imm = {{20{d_inst[31]}}, d_inst[7], d_inst[30:25], d_inst[11:8], 1'b0}; case (d_inst[14:12]) 3'b000: op = 6'd5; 3'b001: op = 6'd6; 3'b100: op = 6'd7; 3'b101: op = 6'd8; 3'b110: op = 6'd9; 3'b111: op = 6'd10; endcase end
                        7'b0000011: begin imm = {{20{d_inst[31]}}, d_inst[31:20]}; case (d_inst[14:12]) 3'b000: op = 6'd11; 3'b001: op = 6'd12; 3'b010: op = 6'd13; 3'b100: op = 6'd14; 3'b101: op = 6'd15; endcase end
                        7'b0100011: begin imm = {{20{d_inst[31]}}, d_inst[31:25], d_inst[11:7]}; case (d_inst[14:12]) 3'b000: op = 6'd16; 3'b001: op = 6'd17; 3'b010: op = 6'd18; endcase end
                        7'b0010011: begin imm = {{20{d_inst[31]}}, d_inst[31:20]}; case (d_inst[14:12]) 3'b000: op = 6'd19; 3'b010: op = 6'd20; 3'b011: op = 6'd21; 3'b100: op = 6'd22; 3'b110: op = 6'd23; 3'b111: op = 6'd24; 3'b001: op = 6'd25; 3'b101: op = (d_inst[31:25] == 7'b0) ? 6'd26 : 6'd27; endcase end
                        7'b0110011: begin case (d_inst[14:12]) 3'b000: op = (d_inst[31:25] == 7'b0) ? 6'd28 : 6'd29; 3'b001: op = 6'd30; 3'b010: op = 6'd31; 3'b011: op = 6'd32; 3'b100: op = 6'd33; 3'b101: op = (d_inst[31:25] == 7'b0) ? 6'd34 : 6'd35; 3'b110: op = 6'd36; 3'b111: op = 6'd37; endcase end
                    endcase

                    if (op != 0) begin
                        rs_found = 0;
                        for (j=0; j<RS_SIZE; j=j+1) if (!rs_busy[j] && !rs_found) begin free_rs = j; rs_found = 1; end
                        if ((op >= 11 && op <= 18 && lsb_count < LSB_SIZE) || (op < 11 || op > 18) && rs_found) begin
                            rob_busy[rob_tail] <= 1; rob_ready[rob_tail] <= 0; rob_rd[rob_tail] <= rd; rob_pc[rob_tail] <= inst_pc;
                            if (rd != 0) begin rf_busy[rd] <= 1; rf_rob_id[rd] <= rob_tail; end
                            if (op >= 11 && op <= 18) begin
                                lsb_busy[lsb_tail] <= 1; lsb_op[lsb_tail] <= op; lsb_imm[lsb_tail] <= imm; lsb_rob_id[lsb_tail] <= rob_tail; lsb_addr_ready[lsb_tail] <= 0;
                                if (rf_busy[rs1]) begin if (rob_ready[rf_rob_id[rs1]]) begin lsb_v1[lsb_tail] <= rob_val[rf_rob_id[rs1]]; lsb_has_q1[lsb_tail] <= 0; end else begin lsb_q1[lsb_tail] <= rf_rob_id[rs1]; lsb_has_q1[lsb_tail] <= 1; end end
                                else begin lsb_v1[lsb_tail] <= rf_val[rs1]; lsb_has_q1[lsb_tail] <= 0; end
                                if (rf_busy[rs2]) begin if (rob_ready[rf_rob_id[rs2]]) begin lsb_v2[lsb_tail] <= rob_val[rf_rob_id[rs2]]; lsb_has_q2[lsb_tail] <= 0; end else begin lsb_q2[lsb_tail] <= rf_rob_id[rs2]; lsb_has_q2[lsb_tail] <= 1; end end
                                else begin lsb_v2[lsb_tail] <= rf_val[rs2]; lsb_has_q2[lsb_tail] <= 0; end
                                lsb_tail <= (lsb_tail + 1) % LSB_SIZE; lsb_count <= lsb_count + 1;
                            end else begin
                                rs_busy[free_rs] <= 1; rs_op[free_rs] <= op; rs_imm[free_rs] <= imm; rs_rob_id[free_rs] <= rob_tail; rs_pc[free_rs] <= inst_pc; rs_is_rv32c[free_rs] <= (inst[1:0] != 2'b11);
                                if (rf_busy[rs1]) begin if (rob_ready[rf_rob_id[rs1]]) begin rs_v1[free_rs] <= rob_val[rf_rob_id[rs1]]; rs_has_q1[free_rs] <= 0; end else begin rs_q1[free_rs] <= rf_rob_id[rs1]; rs_has_q1[free_rs] <= 1; end end
                                else begin rs_v1[free_rs] <= rf_val[rs1]; rs_has_q1[free_rs] <= 0; end
                                if (rf_busy[rs2]) begin if (rob_ready[rf_rob_id[rs2]]) begin rs_v2[free_rs] <= rob_val[rf_rob_id[rs2]]; rs_has_q2[free_rs] <= 0; end else begin rs_q2[free_rs] <= rf_rob_id[rs2]; rs_has_q2[free_rs] <= 1; end end
                                else begin rs_v2[free_rs] <= rf_val[rs2]; rs_has_q2[free_rs] <= 0; end
                            end
                            rob_tail <= (rob_tail + 1) % ROB_SIZE; rob_count <= rob_count + 1; inst_valid <= 0;
                            if (inst[1:0] == 2'b11) begin pc <= pc + 4; if_head <= (if_head + 2) % 8; if_count <= if_count - 2; end
                            else begin pc <= pc + 2; if_head <= (if_head + 1) % 8; if_count <= if_count - 1; end
                        end
                    end
                end
            end
        end
    end
endmodule