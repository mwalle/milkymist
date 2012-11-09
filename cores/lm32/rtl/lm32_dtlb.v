`include "lm32_include.v"

`ifdef CFG_MMU_ENABLED

`define LM32_DTLB_CTRL_FLUSH                5'h1
`define LM32_DTLB_CTRL_UPDATE               5'h2
`define LM32_TLB_CTRL_INVALIDATE_ENTRY      5'h10

`define LM32_TLB_STATE_CHECK     2'b01
`define LM32_TLB_STATE_FLUSH     2'b10

/////////////////////////////////////////////////////
// Module interface
/////////////////////////////////////////////////////

module lm32_dtlb (
    // ----- Inputs -------
    clk_i,
    rst_i,
    enable,
    stall_x,
    stall_m,
    address_x,
    address_m,
    load_q_m,
    store_q_m,
    csr,
    csr_write_data,
    csr_write_enable,
    exception_x,
    eret_q_x,
    exception_m,
    // ----- Outputs -----
    physical_load_store_address_m,
	stall_request,
    dtlb_miss_int,
    csr_read_data
    );

/////////////////////////////////////////////////////
// Parameters
/////////////////////////////////////////////////////

parameter dtlb_sets = 1024;				// Number of lines of DTLB
parameter page_size = 4096;				// System page size

`define LM32_DTLB_IDX_RNG		addr_dtlb_index_msb:addr_dtlb_index_lsb
`define LM32_DTLB_ADDRESS_PFN_RNG	addr_pfn_msb:addr_pfn_lsb
`define LM32_PAGE_OFFSET_RNG		addr_page_offset_msb:addr_page_offset_lsb
`define LM32_DTLB_INVALID_ADDRESS	{ vpfn_width{1'b1} }

localparam addr_page_offset_lsb = 0;
localparam addr_page_offset_msb = addr_page_offset_lsb + `CLOG2(page_size) - 1;
localparam addr_dtlb_index_width = `CLOG2(dtlb_sets);
localparam addr_dtlb_index_lsb = addr_page_offset_msb + 1;
localparam addr_dtlb_index_msb = addr_dtlb_index_lsb + addr_dtlb_index_width - 1;
localparam addr_pfn_lsb = addr_page_offset_msb + 1;
localparam addr_pfn_msb = `LM32_WORD_WIDTH - 1;
localparam vpfn_width = `LM32_WORD_WIDTH - `CLOG2(page_size);
localparam addr_dtlb_tag_width = vpfn_width - addr_dtlb_index_width;
localparam addr_dtlb_tag_lsb = addr_dtlb_index_msb + 1;
localparam addr_dtlb_tag_msb = addr_dtlb_tag_lsb + addr_dtlb_tag_width - 1;

`define LM32_DTLB_TAG_INVALID		{ addr_dtlb_tag_width{ 1'b0 } }
`define LM32_DTLB_LOOKUP_RANGE		vpfn_width-1:0

/* The following define is the range containing the TAG inside the dtlb_read_data wire which contains the DTLB value from BlockRAM
 * Indeed dtlb_read_data contains { VALID_BIT, TAG_VALUE, LOOKUP_VALUE }
 * LM32_DTLB_TAG_RANGE is the range to extract the TAG_VALUE */
`define LM32_DTLB_TAG_RANGE		vpfn_width+addr_dtlb_tag_width-1:vpfn_width

/* The following define is the range containing the TAG inside a memory address like dtlb_update_vaddr_csr_reg for instance. */
`define LM32_DTLB_ADDR_TAG_RNG		addr_dtlb_tag_msb:addr_dtlb_tag_lsb
`define LM32_DTLB_VALID_BIT		vpfn_width+addr_dtlb_tag_width

/////////////////////////////////////////////////////
// Inputs
/////////////////////////////////////////////////////

input clk_i;                                            // Clock
input rst_i;                                            // Reset

input enable;                                           // Data TLB enable

input stall_x;                                          // Stall X stage
input stall_m;                                          // Stall M stage

input [`LM32_WORD_RNG] address_x;                       // X stage load/store address
input [`LM32_WORD_RNG] address_m;                       // M stage load/store address
input load_q_m;                                         // Load instruction in M stage
input store_q_m;                                        // Store instruction in M stage

input [`LM32_CSR_RNG] csr;				// CSR read/write index
input [`LM32_WORD_RNG] csr_write_data;			// Data to write to specified CSR
input csr_write_enable;					// CSR write enable
input exception_x;					// An exception occured in the X stage
input exception_m;
input eret_q_x;

/////////////////////////////////////////////////////
// Outputs
/////////////////////////////////////////////////////

output [`LM32_WORD_RNG] physical_load_store_address_m;
wire   [`LM32_WORD_RNG] physical_load_store_address_m;
output [`LM32_WORD_RNG] csr_read_data;
wire   [`LM32_WORD_RNG] csr_read_data;
output dtlb_miss_int;
wire   dtlb_miss_int;
output stall_request;
wire   stall_request;

/////////////////////////////////////////////////////
// Internal nets and registers
/////////////////////////////////////////////////////

wire [addr_dtlb_index_width-1:0] dtlb_data_read_address;
wire [addr_dtlb_index_width-1:0] dtlb_data_write_address;
wire dtlb_data_read_port_enable;
wire dtlb_write_port_enable;
wire [vpfn_width + addr_dtlb_tag_width + 1 - 1:0] dtlb_write_data; // +1 is for valid_bit
wire [vpfn_width + addr_dtlb_tag_width + 1 - 1:0] dtlb_read_data; // +1 is for valid_bit

reg [`LM32_WORD_RNG] dtlb_update_vaddr_csr_reg = `LM32_WORD_WIDTH'd0;
reg [`LM32_WORD_RNG] dtlb_update_paddr_csr_reg = `LM32_WORD_WIDTH'd0;
reg [1:0] dtlb_state;
reg dtlb_updating;
reg [addr_dtlb_index_width-1:0] dtlb_update_set;
reg dtlb_flushing;
reg [addr_dtlb_index_width-1:0] dtlb_flush_set;
wire dtlb_miss;
reg dtlb_miss_q = `FALSE;
reg [`LM32_WORD_RNG] dtlb_miss_addr;
wire dtlb_data_valid;
wire [`LM32_DTLB_LOOKUP_RANGE] dtlb_lookup;

assign stall_request = (dtlb_state == `LM32_TLB_STATE_FLUSH) && (enable == `TRUE);

/////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Instantiations
/////////////////////////////////////////////////////

lm32_ram
  #(
    // ----- Parameters -------
    .data_width (vpfn_width + addr_dtlb_tag_width + 1),
    .address_width (addr_dtlb_index_width)
// Modified for Milkymist: removed non-portable RAM parameters
    ) dtlb_data_ram
    (
     // ----- Inputs -------
     .read_clk (clk_i),
     .write_clk (clk_i),
     .reset (rst_i),
     .read_address (dtlb_data_read_address),
     .enable_read (dtlb_data_read_port_enable),
     .write_address (dtlb_data_write_address),
     .enable_write (`TRUE),
     .write_enable (dtlb_write_port_enable),
     .write_data (dtlb_write_data),
     // ----- Outputs -------
     .read_data (dtlb_read_data)
     );

/////////////////////////////////////////////////////
// Combinational logic
/////////////////////////////////////////////////////

// Compute address to use to index into the DTLB data memory

assign dtlb_data_read_address = address_x[`LM32_DTLB_IDX_RNG];
assign dtlb_tag_read_address = address_x[`LM32_DTLB_IDX_RNG];

// tlb_update_address will receive data from a CSR register
assign dtlb_data_write_address = dtlb_update_vaddr_csr_reg[`LM32_DTLB_IDX_RNG];

assign dtlb_data_read_port_enable = (stall_x == `FALSE) || !stall_m;
assign dtlb_write_port_enable = dtlb_updating || dtlb_flushing;

assign physical_load_store_address_m = (enable == `FALSE)
			    ? address_m
			    : {dtlb_lookup, address_m[`LM32_PAGE_OFFSET_RNG]};

assign dtlb_write_data = (dtlb_flushing == `TRUE)
			 ? {`FALSE, {addr_dtlb_tag_width{1'b0}}, {vpfn_width{1'b0}}}
			 : {`TRUE, {dtlb_update_vaddr_csr_reg[`LM32_DTLB_ADDR_TAG_RNG]}, dtlb_update_paddr_csr_reg[`LM32_DTLB_ADDRESS_PFN_RNG]};

assign dtlb_read_tag = dtlb_read_data[`LM32_DTLB_TAG_RANGE];
assign dtlb_data_valid = dtlb_read_data[`LM32_DTLB_VALID_BIT];
assign dtlb_lookup = dtlb_read_data[`LM32_DTLB_LOOKUP_RANGE];
assign csr_read_data = dtlb_miss_addr;
assign dtlb_miss = (enable == `TRUE) && (load_q_m || store_q_m) && ~(dtlb_data_valid);
assign dtlb_miss_int = (dtlb_miss || dtlb_miss_q);

/////////////////////////////////////////////////////
// Sequential logic
/////////////////////////////////////////////////////

// CSR Write
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
	begin
		dtlb_update_vaddr_csr_reg <= `LM32_WORD_WIDTH'd0;
		dtlb_update_paddr_csr_reg <= `LM32_WORD_WIDTH'd0;
	end
	else
	begin
		if (csr_write_enable)
		begin
			case (csr)
			`LM32_CSR_TLB_VADDRESS: if (csr_write_data[0]) dtlb_update_vaddr_csr_reg[31:1] <= csr_write_data[31:1];
			`LM32_CSR_TLB_PADDRESS: if (csr_write_data[0]) dtlb_update_paddr_csr_reg[31:1] <= csr_write_data[31:1];
			endcase
		end
		dtlb_update_vaddr_csr_reg[0] <= 0;
		dtlb_update_paddr_csr_reg[0] <= 0;
	end
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
		dtlb_miss_q <= `FALSE;
	else
	begin
		if (dtlb_miss && ~dtlb_miss_q)
			dtlb_miss_q <= `TRUE;
		else if (dtlb_miss_q && exception_m)
			dtlb_miss_q <= `FALSE;
	end
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
	if (rst_i == `TRUE)
	begin
		$display("DTLB STATE MACHINE RESET");
		dtlb_flushing <= 1;
		dtlb_flush_set <= {addr_dtlb_index_width{1'b1}};
		dtlb_state <= `LM32_TLB_STATE_FLUSH;
		dtlb_updating <= 0;
		dtlb_miss_addr <= `LM32_WORD_WIDTH'd0;
	end
	else
	begin
		case (dtlb_state)

		`LM32_TLB_STATE_CHECK:
		begin
			dtlb_updating <= 0;
			dtlb_flushing <= 0;
			if (dtlb_miss == `TRUE)
			begin
				dtlb_miss_addr <= address_m;
				$display("WARNING : DTLB MISS on addr 0x%08X at time %t", address_m, $time);
			end
			if (csr_write_enable && csr_write_data[0])
			begin
				if (csr == `LM32_CSR_TLB_PADDRESS)
				begin
					dtlb_updating <= 1;
				end
				else if (csr == `LM32_CSR_TLB_VADDRESS)
				begin
					dtlb_updating <= 0;
					case (csr_write_data[5:1])
					`LM32_DTLB_CTRL_FLUSH:
					begin
						dtlb_flushing <= 1;
						dtlb_flush_set <= {addr_dtlb_index_width{1'b1}};
						dtlb_state <= `LM32_TLB_STATE_FLUSH;
					end

					`LM32_TLB_CTRL_INVALIDATE_ENTRY:
					begin
						dtlb_flushing <= 1;
//						dtlb_flush_set <= dtlb_update_vaddr_csr_reg[`LM32_DTLB_IDX_RNG];
						dtlb_flush_set <= csr_write_data[`LM32_DTLB_IDX_RNG];
						dtlb_updating <= 0;
						dtlb_state <= `LM32_TLB_STATE_CHECK;
					end

					endcase
				end
				else
					dtlb_updating <= 0;
			end
		end

		`LM32_TLB_STATE_FLUSH:
		begin
			dtlb_updating <= 0;
			if (dtlb_flush_set == {addr_dtlb_index_width{1'b0}})
				dtlb_state <= `LM32_TLB_STATE_CHECK;
			dtlb_flush_set <= dtlb_flush_set - 1'b1;
		end

		endcase
	end
end

`ifdef CFG_VERBOSE_DISPLAY_ENABLED
always @(posedge clk_i)
begin
	if (dtlb_write_port_enable)
	begin
		$display("[DTLB data : %d] Writing 0x%08X to 0x%08X", $time, dtlb_write_data, dtlb_data_write_address);
	end
end
`endif

endmodule

`endif

