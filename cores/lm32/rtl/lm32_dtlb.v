`include "lm32_include.v"

`ifdef CFG_MMU_ENABLED

`define LM32_TLB_OP_RNG                     5:1
`define LM32_TLB_OP_NOOP                    5'h0
`define LM32_TLB_OP_FLUSH                   5'h1
`define LM32_TLB_OP_INVALIDATE              5'h2

`define LM32_DTLB_STATE_RNG                 1:0
`define LM32_DTLB_STATE_CHECK               2'b01
`define LM32_DTLB_STATE_FLUSH               2'b10

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
    miss_int,
    csr_read_data
    );

/////////////////////////////////////////////////////
// Parameters
/////////////////////////////////////////////////////

parameter sets = 1024;                  // Number of lines of DTLB
parameter page_size = 4096;             // System page size

`define LM32_DTLB_IDX_RNG       addr_index_msb:addr_index_lsb
`define LM32_DTLB_ADDRESS_PFN_RNG   addr_pfn_msb:addr_pfn_lsb
`define LM32_PAGE_OFFSET_RNG        addr_page_offset_msb:addr_page_offset_lsb
`define LM32_DTLB_INVALID_ADDRESS   { vpfn_width{1'b1} }

localparam addr_page_offset_lsb = 0;
localparam addr_page_offset_msb = addr_page_offset_lsb + `CLOG2(page_size) - 1;
localparam addr_index_width = `CLOG2(sets);
localparam addr_index_lsb = addr_page_offset_msb + 1;
localparam addr_index_msb = addr_index_lsb + addr_index_width - 1;
localparam addr_pfn_lsb = addr_page_offset_msb + 1;
localparam addr_pfn_msb = `LM32_WORD_WIDTH - 1;
localparam vpfn_width = `LM32_WORD_WIDTH - `CLOG2(page_size);
localparam addr_tag_width = vpfn_width - addr_index_width;
localparam addr_tag_lsb = addr_index_msb + 1;
localparam addr_tag_msb = addr_tag_lsb + addr_tag_width - 1;

`define LM32_DTLB_TAG_INVALID       { addr_tag_width{ 1'b0 } }
`define LM32_DTLB_LOOKUP_RANGE      vpfn_width-1:0

/* The following define is the range containing the TAG inside the read_data wire which contains the DTLB value from BlockRAM
 * Indeed read_data contains { VALID_BIT, TAG_VALUE, LOOKUP_VALUE }
 * LM32_DTLB_TAG_RANGE is the range to extract the TAG_VALUE */
`define LM32_DTLB_TAG_RANGE     vpfn_width+addr_tag_width-1:vpfn_width

/* The following define is the range containing the TAG inside a memory address like update_vaddr for instance. */
`define LM32_DTLB_ADDR_TAG_RNG      addr_tag_msb:addr_tag_lsb
`define LM32_DTLB_VALID_BIT     vpfn_width+addr_tag_width

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

input [`LM32_CSR_RNG] csr;              // CSR read/write index
input [`LM32_WORD_RNG] csr_write_data;          // Data to write to specified CSR
input csr_write_enable;                 // CSR write enable
input exception_x;                  // An exception occured in the X stage
input exception_m;
input eret_q_x;

/////////////////////////////////////////////////////
// Outputs
/////////////////////////////////////////////////////

output [`LM32_WORD_RNG] physical_load_store_address_m;
wire   [`LM32_WORD_RNG] physical_load_store_address_m;
output [`LM32_WORD_RNG] csr_read_data;
wire   [`LM32_WORD_RNG] csr_read_data;
output miss_int;
wire   miss_int;
output stall_request;
wire   stall_request;

/////////////////////////////////////////////////////
// Internal nets and registers
/////////////////////////////////////////////////////

wire [addr_index_width-1:0] data_read_address;
wire [addr_index_width-1:0] data_write_address;
wire data_read_port_enable;
wire write_port_enable;
wire [vpfn_width + addr_tag_width + 1 - 1:0] write_data; // +1 is for valid_bit
wire [vpfn_width + addr_tag_width + 1 - 1:0] read_data; // +1 is for valid_bit

reg [`LM32_WORD_RNG] update_vaddr;
reg [`LM32_WORD_RNG] update_paddr;
reg [`LM32_DTLB_STATE_RNG] state;                         // Current state of FSM
reg update;
reg invalidate;
reg [addr_index_width-1:0] flush_set;
wire miss;
reg miss_q = `FALSE;
reg [`LM32_WORD_RNG] miss_address;
wire data_valid;
wire [`LM32_DTLB_LOOKUP_RANGE] lookup;
wire checking;
wire flushing;
wire csr_dtlb_selected;

/////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Instantiations
/////////////////////////////////////////////////////

lm32_ram
  #(
    // ----- Parameters -------
    .data_width (vpfn_width + addr_tag_width + 1),
    .address_width (addr_index_width)
// Modified for Milkymist: removed non-portable RAM parameters
    ) data_ram
    (
     // ----- Inputs -------
     .read_clk (clk_i),
     .write_clk (clk_i),
     .reset (rst_i),
     .read_address (data_read_address),
     .enable_read (data_read_port_enable),
     .write_address (data_write_address),
     .enable_write (`TRUE),
     .write_enable (write_port_enable),
     .write_data (write_data),
     // ----- Outputs -------
     .read_data (read_data)
     );

/////////////////////////////////////////////////////
// Combinational logic
/////////////////////////////////////////////////////

// Compute address to use to index into the DTLB data memory

assign data_read_address = address_x[`LM32_DTLB_IDX_RNG];
assign tag_read_address = address_x[`LM32_DTLB_IDX_RNG];

// tlb_update_address will receive data from a CSR register
assign data_write_address = (flushing == `TRUE)
                            ? flush_set
                            : update_vaddr[`LM32_DTLB_IDX_RNG];

assign data_read_port_enable = (stall_x == `FALSE) || !stall_m;
assign write_port_enable = update || invalidate;

assign physical_load_store_address_m = (enable == `FALSE)
                ? address_m
                : {lookup, address_m[`LM32_PAGE_OFFSET_RNG]};

assign write_data = (invalidate == `TRUE)
             ? {`FALSE, {addr_tag_width{1'b0}}, {vpfn_width{1'b0}}}
             : {`TRUE, {update_vaddr[`LM32_DTLB_ADDR_TAG_RNG]}, update_paddr[`LM32_DTLB_ADDRESS_PFN_RNG]};

assign read_tag = read_data[`LM32_DTLB_TAG_RANGE];
assign data_valid = read_data[`LM32_DTLB_VALID_BIT];
assign lookup = read_data[`LM32_DTLB_LOOKUP_RANGE];
assign csr_read_data = miss_address;
assign miss = (enable == `TRUE) && (load_q_m || store_q_m) && ~(data_valid);
assign miss_int = (miss || miss_q);

assign checking = state[0];
assign flushing = state[1];
assign stall_request = flushing;
assign csr_dtlb_selected = (csr_write_data[0] == `TRUE);

/////////////////////////////////////////////////////
// Sequential logic
/////////////////////////////////////////////////////

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
        miss_q <= `FALSE;
    else
    begin
        if (miss && ~miss_q)
            miss_q <= `TRUE;
        else if (miss_q && exception_m)
            miss_q <= `FALSE;
    end
end

// Store last address that caused a DTLB miss
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
        miss_address <= {`LM32_WORD_WIDTH{1'b0}};
    else
    begin
        if ((checking ==`TRUE) && (miss == `TRUE))
        begin
            //$display("WARNING : DTLB MISS on addr 0x%08X at time %t", address_m, $time);
            miss_address <= address_m;
        end
    end
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
    begin
        //$display("DTLB STATE MACHINE RESET");
        update_vaddr <= {`LM32_WORD_WIDTH{1'b0}};
        update_paddr <= {`LM32_WORD_WIDTH{1'b0}};
        invalidate <= 1;
        flush_set <= {addr_index_width{1'b1}};
        state <= `LM32_TLB_STATE_FLUSH;
        update <= 0;
    end
    else
    begin
        case (state)

        `LM32_TLB_STATE_CHECK:
        begin
            update <= 0;
            invalidate <= 0;
            if ((csr_write_enable == `TRUE) && (csr_dtlb_selected == `TRUE))
            begin
                if (csr == `LM32_CSR_TLB_PADDRESS)
                begin
                    update_paddr <= {csr_write_data[31:1], 1'b0};
                    update <= 1;
                end
                else if (csr == `LM32_CSR_TLB_VADDRESS)
                begin
                    update_vaddr <= {csr_write_data[31:1], 1'b0};
                    case (csr_write_data[`LM32_TLB_OP_RNG])
                    `LM32_TLB_OP_FLUSH:
                    begin
                        invalidate <= 1;
                        flush_set <= {addr_index_width{1'b1}};
                        state <= `LM32_TLB_STATE_FLUSH;
                    end

                    `LM32_TLB_OP_INVALIDATE:
                    begin
                        invalidate <= 1;
                        flush_set <= csr_write_data[`LM32_DTLB_IDX_RNG];
                        state <= `LM32_TLB_STATE_CHECK;
                    end

                    endcase
                end
            end
        end

        `LM32_TLB_STATE_FLUSH:
        begin
            if (flush_set == {addr_index_width{1'b0}})
                state <= `LM32_TLB_STATE_CHECK;
            flush_set <= flush_set - 1'b1;
        end

        endcase
    end
end

`ifdef CFG_VERBOSE_DISPLAY_ENABLED
always @(posedge clk_i)
begin
    if (write_port_enable)
    begin
        $display("[DTLB data : %d] Writing 0x%08X to 0x%08X", $time, write_data, data_write_address);
    end
end
`endif

endmodule

`endif

