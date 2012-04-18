/*
 * Milkymist SoC
 * Copyright (C) 2012 Michael Walle
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
 
`timescale  1 ns / 1 ps


module tb_timer();

reg usb_clk;
initial usb_clk = 1'b0;
always #10 usb_clk = ~usb_clk;
reg usb_rst;

task waitclock;
begin
	@(posedge usb_clk);
	#1;
end
endtask

reg [5:0] io_a;
reg [7:0] io_di;
reg io_we;
reg io_re;

task iowrite;
input [5:0] address;
input [7:0] data;
begin
	io_a = address;
	io_di = data;
	io_we = 1'b1;
	waitclock;
	io_we = 1'b0;
	waitclock;
	$display("IO Write: %x=%x", address, data);
end
endtask

/* IRQ ack logic */
wire [1:0] irq;
reg [1:0] irq_r;
reg [1:0] irq_rr;
always @(posedge usb_clk) begin
	irq_r <= irq;
	irq_rr <= irq_r;
end

always @(posedge usb_clk)
	if(irq[0] & ~irq_r[0]) $display("timer0 irq");

always @(posedge usb_clk)
	if(irq[1] & ~irq_r[0]) $display("timer1 irq");

wire irq_ack = irq_r & ~irq_rr;


softusb_timer timer(
	.usb_clk(usb_clk),
	.usb_rst(usb_rst),

	.io_we(io_we),
	.io_a(io_a),
	.io_di(io_di),
	.io_do(),

	.irq(irq),
	.irq_ack(irq_ack)
);

initial begin
	$dumpfile("softusb_timer.vcd");
	$dumpvars(0, timer);

	usb_rst = 1'b1;
	io_di = 8'b0;
	io_re = 1'b0;
	io_we = 1'b0;
	io_a = 6'b0;
	waitclock;
	waitclock;

	usb_rst = 1'b0;
	#100;

	iowrite('h23, 'h00);
	iowrite('h20, 'h13);

	#10000;

	$finish;
end

endmodule
