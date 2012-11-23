`include "lm32_include.v"

`ifdef CFG_MMU_ENABLED
`define LM32_ITLB_CTRL_FLUSH                5'h1
`define LM32_ITLB_CTRL_UPDATE               5'h2
`define LM32_TLB_CTRL_SWITCH_TO_KERNEL_MODE	5'h4
`define LM32_TLB_CTRL_SWITCH_TO_USER_MODE   5'h8
`define LM32_TLB_CTRL_INVALIDATE_ENTRY      5'h10

`define LM32_TLB_STATE_CHECK             2'b01
`define LM32_TLB_STATE_FLUSH             2'b10

`define LM32_KERNEL_MODE                 1
`define LM32_USER_MODE                   0

/////////////////////////////////////////////////////
// Module interface
/////////////////////////////////////////////////////
module lm32_itlb (
    // ----- Inputs -------
    clk_i,
    rst_i,
    stall_a,
    stall_f,
    stall_x,
    stall_m,
    pc_a,
    pc_f,
    pc_x,
    pc_m,
    pc_w,
    read_enable_f,
    csr,
    csr_write_data,
    csr_write_enable,
    itlbe,
    eitlbe,
    exception_x,
    eret_q_x,
    exception_m,
    q_x,
    // ----- Outputs -------
    physical_pc_f,
    itlb_miss_int,
    kernel_mode,
    csr_read_data
    );

/////////////////////////////////////////////////////
// Parameters
/////////////////////////////////////////////////////

parameter itlb_sets = 1024;				// Number of lines of ITLB
parameter page_size = 4096;				// System page size

`define LM32_ITLB_IDX_RNG		addr_itlb_index_msb:addr_itlb_index_lsb
`define LM32_ITLB_ADDRESS_PFN_RNG	addr_pfn_msb:addr_pfn_lsb
`define LM32_PAGE_OFFSET_RNG		addr_page_offset_msb:addr_page_offset_lsb
`define LM32_ITLB_INVALID_ADDRESS	{ vpfn_width{1'b1} }

localparam addr_page_offset_lsb = 0;
localparam addr_page_offset_msb = addr_page_offset_lsb + `CLOG2(page_size) - 1;
localparam addr_itlb_index_width = `CLOG2(itlb_sets);
localparam addr_itlb_index_lsb = addr_page_offset_msb + 1;
localparam addr_itlb_index_msb = addr_itlb_index_lsb + addr_itlb_index_width - 1;
localparam addr_pfn_lsb = addr_page_offset_msb + 1;
localparam addr_pfn_msb = `LM32_WORD_WIDTH - 1;
localparam vpfn_width = `LM32_WORD_WIDTH - `CLOG2(page_size);
localparam addr_itlb_tag_width = vpfn_width - addr_itlb_index_width;
localparam addr_itlb_tag_lsb = addr_itlb_index_msb + 1;
localparam addr_itlb_tag_msb = addr_itlb_tag_lsb + addr_itlb_tag_width - 1;

`define LM32_ITLB_TAG_INVALID		{ addr_itlb_tag_width{ 1'b0 } }
`define LM32_ITLB_LOOKUP_RANGE		vpfn_width-1:0

/* The following define is the range containing the TAG inside the itlb_read_data wire which contains the ITLB value from BlockRAM
 * Indeed itlb_read_data contains { VALID_BIT, TAG_VALUE, LOOKUP_VALUE }
 * LM32_ITLB_TAG_RANGE is the range to extract the TAG_VALUE */
`define LM32_ITLB_TAG_RANGE		vpfn_width+addr_itlb_tag_width-1:vpfn_width

/* The following define is the range containing the TAG inside a memory address like itlb_update_vaddr_csr_reg for instance. */
`define LM32_ITLB_ADDR_TAG_RNG		addr_itlb_tag_msb:addr_itlb_tag_lsb
`define LM32_ITLB_VALID_BIT		vpfn_width+addr_itlb_tag_width

/////////////////////////////////////////////////////
// Inputs
/////////////////////////////////////////////////////

input clk_i;                                        // Clock
input rst_i;                                        // Reset

input stall_a;					    // Stall instruction in A stage
input stall_f;					    // Stall instruction in F stage
input stall_x;					    // Stall instruction in X stage
input stall_m;					    // Stall instruction in X stage

input [`LM32_PC_RNG] pc_a;			    // Address of instruction in A stage
input [`LM32_PC_RNG] pc_f;			    // Address of instruction in F stage
input [`LM32_PC_RNG] pc_x;			    // Address of instruction in X stage
input [`LM32_PC_RNG] pc_m;			    // Address of instruction in M stage
input [`LM32_PC_RNG] pc_w;			    // Address of instruction in W stage

input read_enable_f;                                // Indicates if cache access is valid

input [`LM32_CSR_RNG] csr;				// CSR read/write index
input [`LM32_WORD_RNG] csr_write_data;			// Data to write to specified CSR
input csr_write_enable;					// CSR write enable
input itlbe;
input eitlbe;
input exception_x;					// An exception occured in the X stage
input exception_m;
input eret_q_x;
input q_x;

/////////////////////////////////////////////////////
// Outputs
/////////////////////////////////////////////////////
output [`LM32_WORD_RNG] csr_read_data;
wire   [`LM32_WORD_RNG] csr_read_data;

output kernel_mode;
wire   kernel_mode;
output itlb_miss_int;
wire   itlb_miss_int;
output [`LM32_PC_RNG] physical_pc_f;
wire   [`LM32_PC_RNG] physical_pc_f;

/////////////////////////////////////////////////////
// Internal nets and registers
/////////////////////////////////////////////////////

wire [addr_itlb_index_width-1:0] itlb_data_read_address;
wire [addr_itlb_index_width-1:0] itlb_data_write_address;
wire itlb_data_read_port_enable;
wire itlb_write_port_enable;
wire [vpfn_width + addr_itlb_tag_width + 1 - 1:0] itlb_write_data; // +1 is for valid_bit
wire [vpfn_width + addr_itlb_tag_width + 1 - 1:0] itlb_read_data; // +1 is for valid_bit
reg kernel_mode_reg = `LM32_KERNEL_MODE;
wire switch_to_kernel_mode;
wire switch_to_user_mode;
reg [`LM32_WORD_RNG] itlb_update_vaddr_csr_reg = `LM32_WORD_WIDTH'd0;
reg [`LM32_WORD_RNG] itlb_update_paddr_csr_reg = `LM32_WORD_WIDTH'd0;
reg [1:0] itlb_state;
reg itlb_updating;
reg [addr_itlb_index_width-1:0] itlb_update_set;
reg itlb_flushing;
reg [addr_itlb_index_width-1:0] itlb_flush_set;
wire itlb_miss;
reg itlb_miss_q = `FALSE;
reg [`LM32_PC_RNG] itlb_miss_addr;
wire itlb_data_valid;
wire [`LM32_ITLB_LOOKUP_RANGE] itlb_lookup;
reg go_to_user_mode;
reg go_to_user_mode_2;
reg itlb_enabled;

/////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Instantiations
/////////////////////////////////////////////////////

lm32_ram
  #(
    // ----- Parameters -------
    .data_width (vpfn_width + addr_itlb_tag_width + 1),
    .address_width (addr_itlb_index_width)
// Modified for Milkymist: removed non-portable RAM parameters
    ) itlb_data_ram
    (
     // ----- Inputs -------
     .read_clk (clk_i),
     .write_clk (clk_i),
     .reset (rst_i),
     .read_address (itlb_data_read_address),
     .enable_read (itlb_data_read_port_enable),
     .write_address (itlb_data_write_address),
     .enable_write (`TRUE),
     .write_enable (itlb_write_port_enable),
     .write_data (itlb_write_data),
     // ----- Outputs -------
     .read_data (itlb_read_data)
     );

/////////////////////////////////////////////////////
// Combinational logic
/////////////////////////////////////////////////////

// Compute address to use to index into the ITLB data memory
assign itlb_data_read_address = pc_a[`LM32_ITLB_IDX_RNG];

// tlb_update_address will receive data from a CSR register
assign itlb_data_write_address = itlb_update_vaddr_csr_reg[`LM32_ITLB_IDX_RNG];

assign itlb_data_read_port_enable = (stall_a == `FALSE) || !stall_f;
assign itlb_write_port_enable = itlb_updating || itlb_flushing;

assign physical_pc_f = (itlb_enabled == `FALSE)
			    ? {pc_f}
			    : {itlb_lookup, pc_f[`LM32_PAGE_OFFSET_RNG+2]};

assign itlb_write_data = (itlb_flushing == `TRUE)
			 ? {`FALSE, {addr_itlb_tag_width{1'b0}}, {vpfn_width{1'b0}}}
			 : {`TRUE, {itlb_update_vaddr_csr_reg[`LM32_ITLB_ADDR_TAG_RNG]}, itlb_update_paddr_csr_reg[`LM32_ITLB_ADDRESS_PFN_RNG]};

assign kernel_mode = kernel_mode_reg;

assign csr_read_data = {itlb_miss_addr, 2'b0};
assign itlb_miss = (itlb_enabled == `TRUE) && (read_enable_f) && ~(itlb_data_valid) && (~itlb_miss_q);
assign itlb_miss_int = (itlb_miss || itlb_miss_q);
assign itlb_read_tag = itlb_read_data[`LM32_ITLB_TAG_RANGE];
assign itlb_data_valid = itlb_read_data[`LM32_ITLB_VALID_BIT];
assign itlb_lookup = itlb_read_data[`LM32_ITLB_LOOKUP_RANGE];

/////////////////////////////////////////////////////
// Sequential logic
/////////////////////////////////////////////////////

`ifdef CFG_VERBOSE_DISPLAY_ENABLED
always @(posedge clk_i)
begin
	if (itlb_write_port_enable)
	begin
		$display("[ITLB data : %d] Writing 0x%08X to 0x%08X", $time, itlb_write_data, itlb_data_write_address);
	end
end
`endif

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
		go_to_user_mode <= `FALSE;
	else
		go_to_user_mode <= (eret_q_x || switch_to_user_mode);
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
		go_to_user_mode_2 <= `FALSE;
	else
		go_to_user_mode_2 <= go_to_user_mode;
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
		kernel_mode_reg <= `LM32_KERNEL_MODE;
	else
	begin
		if (exception_x || switch_to_kernel_mode)
			kernel_mode_reg <= `LM32_KERNEL_MODE;
		else if (go_to_user_mode_2)
			kernel_mode_reg <= `LM32_USER_MODE;
	end
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
		itlb_miss_q <= `FALSE;
	else
	begin
		if (itlb_miss && ~itlb_miss_q && ~(exception_x == `TRUE && stall_m == `FALSE && stall_x == `FALSE && q_x == `TRUE))
			itlb_miss_q <= `TRUE;
		else if (itlb_miss_q && exception_x == `TRUE && stall_m == `FALSE && stall_x == `FALSE && q_x == `TRUE)
			itlb_miss_q <= `FALSE;
	end
end

// CSR Write
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
	begin
		itlb_update_vaddr_csr_reg <= `LM32_WORD_WIDTH'd0;
		itlb_update_paddr_csr_reg <= `LM32_WORD_WIDTH'd0;
	end
	else
	begin
		if (csr_write_enable)
		begin
			case (csr)
			`LM32_CSR_TLBVADDR: if (~csr_write_data[0]) itlb_update_vaddr_csr_reg[31:1] <= csr_write_data[31:1];
			`LM32_CSR_TLBPADDR: if (~csr_write_data[0]) itlb_update_paddr_csr_reg[31:1] <= csr_write_data[31:1];
			endcase
		end
		itlb_update_vaddr_csr_reg[0] <= 0;
		itlb_update_paddr_csr_reg[0] <= 0;
	end
end

reg [`LM32_PC_RNG] pc_exception;
reg in_exception;

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
	begin
		itlb_enabled <= `FALSE;
		in_exception <= `FALSE;
		pc_exception <= {`LM32_PC_WIDTH{1'b0}};
	end
	else
	begin
		if (~stall_x)
		begin
			if (eret_q_x)
			begin
//				$display("[%t] itlb_enabled <= 0x%08X upon eret", $time, eitlbe);
				itlb_enabled <= eitlbe;
			end
			else if (exception_x || in_exception)
			begin
				if (~in_exception)
				begin
					if (~exception_m)
						in_exception <= 1;
					else
						in_exception <= 0;
				end
				else
				begin
					if (exception_m)
					begin
						//$display("[%t] pc_exception <= 0x%08X", $time, pc_m);
						pc_exception <= pc_m;
					end
					if (pc_exception == pc_w)
					begin
						in_exception <= 0;
					end
				end
				//$display("[%t] itlb_enabled <= 0x%08X upon exception", $time, 0);
				itlb_enabled <= 0;
			end
			else
			begin
				//if (itlb_enabled != itlbe)
					//$display("[%t] itlb_enabled <= 0x%08X", $time, itlbe);

				itlb_enabled <= itlbe;
			end
		end
	end
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
	begin
		//$display("ITLB STATE MACHINE RESET");
		itlb_flushing <= 1;
		itlb_flush_set <= {addr_itlb_index_width{1'b1}};
		itlb_state <= `LM32_TLB_STATE_FLUSH;
		itlb_updating <= 0;
		itlb_miss_addr <= {`LM32_PC_WIDTH{1'b0}};
	end
	else
	begin
		case (itlb_state)

		`LM32_TLB_STATE_CHECK:
		begin
			itlb_updating <= 0;
			itlb_flushing <= 0;
			if (itlb_miss == `TRUE)
			begin
				itlb_miss_addr <= pc_f;
				//$display("WARNING : ITLB MISS on addr 0x%08X at time %t", pc_f * 4, $time);
			end
			if (csr_write_enable && ~csr_write_data[0])
			begin
				if (csr == `LM32_CSR_TLBPADDR /*&& (kernel_mode_reg == `LM32_KERNEL_MODE)*/)
				begin
					//$display("[%t] ITLB WCSR to PADDR with csr_write_data == 0x%08X", $time, csr_write_data);
`ifdef CFG_VERBOSE_DISPLAY_ENABLED
					$display("it's an UPDATE at %t", $time);
`endif
					itlb_updating <= 1;
				end
				// FIXME : test for kernel mode is removed for testing purposes ONLY
				else if (csr == `LM32_CSR_TLBVADDR /*&& (kernel_mode_reg == `LM32_KERNEL_MODE)*/)
				begin
`ifdef CFG_VERBOSE_DISPLAY_ENABLED
					$display("ITLB WCSR at %t with csr_write_data == 0x%08X", $time, csr_write_data);
`endif
					case (csr_write_data[5:1])
					`LM32_ITLB_CTRL_FLUSH:
					begin
`ifdef CFG_VERBOSE_DISPLAY_ENABLED
						$display("it's a FLUSH at %t", $time);
`endif
						itlb_flushing <= 1;
						itlb_flush_set <= {addr_itlb_index_width{1'b1}};
						itlb_state <= `LM32_TLB_STATE_FLUSH;
						itlb_updating <= 0;
					end

					`LM32_TLB_CTRL_INVALIDATE_ENTRY:
					begin
`ifdef CFG_VERBOSE_DISPLAY_ENABLED
						$display("it's an INVALIDATE ENTRY at %t", $time);
`endif
						itlb_flushing <= 1;
//						itlb_flush_set <= itlb_update_vaddr_csr_reg[`LM32_ITLB_IDX_RNG];
						itlb_flush_set <= csr_write_data[`LM32_ITLB_IDX_RNG];
						itlb_updating <= 0;
						itlb_state <= `LM32_TLB_STATE_CHECK;
					end

					endcase
				end
			end
		end

		`LM32_TLB_STATE_FLUSH:
		begin
			itlb_updating <= 0;
			if (itlb_flush_set == {addr_itlb_index_width{1'b0}})
				itlb_state <= `LM32_TLB_STATE_CHECK;
			itlb_flush_set <= itlb_flush_set - 1'b1;
		end

		endcase
	end
end

endmodule

`endif

