/*
 * Milkymist SoC
 * Copyright (C) 2007, 2008, 2009, 2010 Sebastien Bourdeauducq
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

module softusb_timer(
	input usb_clk,
	input usb_rst,

	input io_we,
	input [5:0] io_a,
	input [7:0] io_di,
	output reg [7:0] io_do,

	output reg [1:0] irq,
	input [1:0] irq_ack
);

/* prescaler */
reg [15:0] prescaler;
initial prescaler <= 16'd0;
always @(posedge usb_clk)
	prescaler <= prescaler + 16'd1;

wire prescaler_32768 = prescaler[14:0] == 15'h0;
wire prescaler_2048  = prescaler[10:0] == 11'h0;
wire prescaler_128   = prescaler[6:0]  ==  7'h0;

reg cnt0_ce;
always @(*) begin
	case(cnt0_clk_sel)
		2'd0: cnt0_ce = 1'b1;
		2'd1: cnt0_ce = prescaler_128;
		2'd2: cnt0_ce = prescaler_2048;
		2'd3: cnt0_ce = prescaler_32768;
	endcase
end

reg cnt1_ce;
always @(*) begin
	case(cnt1_clk_sel)
		2'd0: cnt1_ce = 1'b1;
		2'd1: cnt1_ce = prescaler_128;
		2'd2: cnt1_ce = prescaler_2048;
		2'd3: cnt1_ce = prescaler_32768;
	endcase
end

always @(posedge usb_clk) begin
	if(usb_rst) begin
		irq <= 2'b0;
	end else begin
		if(cnt0_en & cnt0_irq_en & (cnt0_val == cnt0_ovrf))
			irq[0] <= 1'b1;
		if(cnt1_en & cnt1_irq_en & (cnt1_val == cnt1_ovrf))
			irq[1] <= 1'b1;
		if(irq_ack[0])
			irq[0] <= 1'b0;
		if(irq_ack[1])
			irq[1] <= 1'b0;
	end
end

reg cnt0_en;
reg cnt0_irq_en;
reg cnt0_ar;
reg [1:0] cnt0_clk_sel;
reg [15:0] cnt0_val;
reg [15:0] cnt0_ovrf;

reg cnt1_en;
reg cnt1_irq_en;
reg cnt1_ar;
reg [1:0] cnt1_clk_sel;
reg [7:0] cnt1_val;
reg [7:0] cnt1_ovrf;
always @(posedge usb_clk) begin
	if(usb_rst) begin
		io_do <= 8'd0;
		cnt0_clk_sel <= 2'b0;
		cnt0_en <= 1'b0;
		cnt0_irq_en <= 1'b0;
		cnt0_ar <= 1'b0;
		cnt0_val <= 16'd0;
		cnt0_ovrf <= 16'd0;
		cnt1_clk_sel <= 2'b0;
		cnt1_en <= 1'b0;
		cnt1_irq_en <= 1'b0;
		cnt1_ar <= 1'b0;
		cnt1_val <= 8'd0;
		cnt1_ovrf <= 8'd0;
		cnt1_clk_sel <= 2'b0;
	end else begin
		io_do <= 8'd0;
		if(cnt0_en & cnt0_ce)
			cnt0_val <= cnt0_val + 1'b1;
		if(cnt1_en & cnt1_ce)
			cnt1_val <= cnt1_val + 1'b1;
		if(cnt0_val == cnt0_ovrf) begin
			if(cnt0_ar)
				cnt0_val <= 16'b0;
			else
				cnt0_en <= 1'b0;
		end
		if(cnt1_val == cnt1_ovrf) begin
			if(cnt1_ar)
				cnt1_val <= 16'b0;
			else
				cnt1_en <= 1'b0;
		end
		case(io_a)
			6'h20: io_do <= {2'b0, cnt0_clk_sel, 1'b0, cnt0_ar, cnt0_irq_en, cnt0_en};
			6'h21: io_do <= cnt0_val[7:0];
			6'h22: io_do <= cnt0_val[15:8];
			6'h23: io_do <= cnt0_ovrf[7:0];
			6'h24: io_do <= cnt0_ovrf[15:8];
			6'h25: io_do <= {2'b0, cnt1_clk_sel, 1'b0, cnt1_ar, cnt1_irq_en, cnt1_en};
			6'h26: io_do <= cnt1_val;
			6'h27: io_do <= cnt1_ovrf;
		endcase
		if(io_we) begin
			case(io_a)
				6'h20: begin
					cnt0_clk_sel <= io_di[5:4];
					cnt0_ar <= io_di[2];
					cnt0_irq_en <= io_di[1];
					cnt0_en <= io_di[0];
				end
				6'h21,
				6'h22: cnt0_val <= 16'd0;
				6'h23: cnt0_ovrf[7:0] <= io_di;
				6'h24: cnt0_ovrf[15:8] <= io_di;
				6'h25: begin
					cnt1_clk_sel <= io_di[5:4];
					cnt1_ar <= io_di[2];
					cnt1_irq_en <= io_di[1];
					cnt1_en <= io_di[0];
				end
				6'h26: cnt1_val <= 8'd0;
				6'h27: cnt1_ovrf <= io_di;
			endcase
		end
	end
end

endmodule
