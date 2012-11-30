//   ==================================================================
//   >>>>>>>>>>>>>>>>>>>>>>> COPYRIGHT NOTICE <<<<<<<<<<<<<<<<<<<<<<<<<
//   ------------------------------------------------------------------
//   Copyright (c) 2011-2012 Yann Sionneau <yann.sionneau@gmail.com>
//   Copyright (c) 2012 Michael Walle <michael@walle.cc>
//   ALL RIGHTS RESERVED
//   ------------------------------------------------------------------
//
//   Permission:
//
//      The authors grants permission to use this code pursuant to the
//      terms of the Lattice Semiconductor Corporation Open Source License
//      Agreement.
//
//   Disclaimer:
//
//      The authors provides no warranty regarding the use or functionality
//      of this code. It is the user's responsibility to verify the user's
//      design for consistency and functionality through the use of formal
//      verification methods.
//
//   --------------------------------------------------------------------
//                         FILE DETAILS
// Project          : LatticeMico32
// File             : lm32_dtlb.v
// Title            : Data Translation Lookaside Buffer
// Dependencies     : lm32_include.v
//
// =============================================================================

`include "lm32_include.v"

`ifdef CFG_MMU_ENABLED

`define LM32_DTLB_STATE_RNG                 1:0
`define LM32_DTLB_STATE_CHECK               2'b01
`define LM32_DTLB_STATE_FLUSH               2'b10

`define LM32_DTLB_OFFSET_RNG                offset_msb:offset_lsb
`define LM32_DTLB_IDX_RNG                   index_msb:index_lsb
`define LM32_DTLB_VPFN_RNG                  vpfn_msb:vpfn_lsb
`define LM32_DTLB_TAG_RNG                   tag_msb:tag_lsb
`define LM32_DTLB_ADDR_RNG                  (index_width-1):0
`define LM32_DTLB_DATA_WIDTH                (vpfn_width+tag_width+1)
`define LM32_DTLB_DATA_RNG                  (`LM32_DTLB_DATA_WIDTH-1):0


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
    load_d,
    store_d,
    load_q_x,
    store_q_x,
    load_q_m,
    store_q_m,
    tlbpaddr,
    tlbvaddr,
    update,
    flush,
    invalidate,
    // ----- Outputs -----
    physical_load_store_address_m,
    stall_request,
    miss
    );

/////////////////////////////////////////////////////
// Parameters
/////////////////////////////////////////////////////

parameter entries = 1024;               // Number of entries in DTLB
parameter page_size = 4096;             // DTLB page size

localparam offset_width = `CLOG2(page_size);
localparam index_width = `CLOG2(entries);
localparam offset_lsb = 0;
localparam offset_msb = (offset_lsb+offset_width-1);
localparam index_lsb = (offset_msb+1);
localparam index_msb = (index_lsb+index_width-1);
localparam tag_lsb = (index_msb+1);
localparam tag_msb = (`LM32_WORD_WIDTH-1);
localparam tag_width = (tag_msb-tag_lsb+1);
localparam vpfn_lsb = (offset_msb+1);
localparam vpfn_msb = (`LM32_WORD_WIDTH-1);
localparam vpfn_width = (vpfn_msb-vpfn_lsb+1);

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
input load_d;                                           // Load instruction in D stage
input store_d;                                          // Store instruction in D stage
input load_q_x;                                         // Load instruction in X stage
input store_q_x;                                        // Store instruction in X stage
input load_q_m;                                         // Load instruction in M stage
input store_q_m;                                        // Store instruction in M stage

input [`LM32_WORD_RNG] tlbvaddr;
input [`LM32_WORD_RNG] tlbpaddr;
input update;
input flush;
input invalidate;

/////////////////////////////////////////////////////
// Outputs
/////////////////////////////////////////////////////

output [`LM32_WORD_RNG] physical_load_store_address_m;
wire   [`LM32_WORD_RNG] physical_load_store_address_m;
output [`LM32_WORD_RNG] csr_read_data;
wire   [`LM32_WORD_RNG] csr_read_data;
output miss;
wire   miss;
output stall_request;
wire   stall_request;

/////////////////////////////////////////////////////
// Internal nets and registers
/////////////////////////////////////////////////////

wire [`LM32_DTLB_ADDR_RNG] read_address;
wire [`LM32_DTLB_ADDR_RNG] write_address;
wire [`LM32_DTLB_DATA_RNG] write_data; // +1 is for valid_bit
wire [`LM32_DTLB_TAG_RNG] tlbe_tag_x;
wire [`LM32_DTLB_VPFN_RNG] tlbe_pfn_x;
wire tlbe_valid_x;
wire checking;
wire flushing;
wire write_port_enable;

reg [`LM32_DTLB_STATE_RNG] state;                         // Current state of FSM
reg [`LM32_DTLB_ADDR_RNG] flush_set;
reg [`LM32_DTLB_VPFN_RNG] tlbe_pfn_m;
reg lookup;


/////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////

////////////////////////////////////////////////////
// Instantiations
/////////////////////////////////////////////////////

lm32_ram
  #(
    // ----- Parameters -------
    .data_width (`LM32_DTLB_DATA_WIDTH),
    .address_width (index_width)
// Modified for Milkymist: removed non-portable RAM parameters
    ) data_ram
    (
     // ----- Inputs -------
     .read_clk (clk_i),
     .write_clk (clk_i),
     .reset (rst_i),
     .read_address (read_address),
     .enable_read (lookup),
     .write_address (write_address),
     .enable_write (`TRUE),
     .write_enable (write_port_enable),
     .write_data (write_data),
     // ----- Outputs -------
     .read_data ({tlbe_pfn_x, tlbe_tag_x, tlbe_valid_x})
     );

/////////////////////////////////////////////////////
// Combinational logic
/////////////////////////////////////////////////////

// Compute address to use to index into the DTLB data memory
assign read_address = address_x[`LM32_DTLB_IDX_RNG];

// tlb_update_address will receive data from a CSR register
assign write_address = (flushing == `TRUE)
                            ? flush_set
                            : tlbvaddr[`LM32_DTLB_IDX_RNG];

assign write_port_enable = (update == `TRUE) || (invalidate == `TRUE) || (flushing == `TRUE);

assign physical_load_store_address_m = (enable == `FALSE)
                ? address_m
                : {tlbe_pfn_m, address_m[`LM32_DTLB_OFFSET_RNG]};

assign write_data = ((invalidate == `TRUE) || (flushing))
             ? {{`LM32_DTLB_DATA_WIDTH-1{1'b0}}, `FALSE}
             : {tlbpaddr[`LM32_DTLB_VPFN_RNG], tlbvaddr[`LM32_DTLB_TAG_RNG], `TRUE};

assign tlbe_match = ({tlbe_tag_x, tlbe_valid_x} == {address_x[`LM32_DTLB_TAG_RNG], `TRUE});

assign miss = ((enable == `TRUE) && ((load_q_x == `TRUE) || (store_q_x == `TRUE)) && (tlbe_match == `FALSE) && (lookup == `FALSE));

assign checking = state[0];
assign flushing = state[1];
assign stall_request = (flushing == `TRUE) || (lookup == `TRUE);

/////////////////////////////////////////////////////
// Sequential logic
/////////////////////////////////////////////////////

// Lookup logic
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
        lookup <= `FALSE;
    else
    begin
        if ((enable == `TRUE) && (stall_x == `FALSE) && ((load_d == `TRUE) || (store_d == `TRUE)))
            lookup <= `TRUE;
        else
            lookup <= `FALSE;
    end
end

// X/M stage registers
always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
        tlbe_pfn_m <= {vpfn_width{1'bx}};
    else if (stall_m == `FALSE)
        tlbe_pfn_m <= tlbe_pfn_x;
end

always @(posedge clk_i `CFG_RESET_SENSITIVITY)
begin
    if (rst_i == `TRUE)
    begin
        flush_set <= {index_width{1'b1}};
        state <= `LM32_DTLB_STATE_FLUSH;
    end
    else
    begin
        case (state)

        `LM32_DTLB_STATE_CHECK:
        begin
            if (flush == `TRUE) begin
                flush_set <= {index_width{1'b1}};
                state <= `LM32_DTLB_STATE_FLUSH;
            end
        end

        `LM32_DTLB_STATE_FLUSH:
        begin
            if (flush_set == {index_width{1'b0}})
                state <= `LM32_DTLB_STATE_CHECK;
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
        $display("[DTLB data : %d] Writing 0x%08X to 0x%08X", $time, write_data, write_address);
    end
end
`endif

endmodule

`endif

