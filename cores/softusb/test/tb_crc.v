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


module tb_crc();

reg usb_clk;
initial usb_clk = 1'b0;
always #10 usb_clk = ~usb_clk;

reg crc_reset;
reg crc_ce;
reg data_in;

task waitclock;
begin
	@(posedge usb_clk);
	#1;
end
endtask

task usb_token;
input [63:0] data;
input integer len;
integer i;
begin
	crc_reset = 1'b1;
	waitclock;
	crc_reset = 1'b0;
	waitclock;
	waitclock;
	for (i=0;i<len;i=i+1) begin
		data_in = data[len-i-1];
		crc_ce = 1'b1;
		waitclock;
		crc_ce = 1'b0;
		waitclock;
	end
end
endtask

wire crc5_valid;
wire crc16_valid;

softusb_crc crc(
	.usb_clk(usb_clk),

	.crc_reset(crc_reset),
	.crc_ce(crc_ce),
	.data(data_in),

	.crc5_valid(crc5_valid),
	.crc16_valid(crc16_valid)
);

initial begin
	$dumpfile("softusb_crc.vcd");
	$dumpvars(0, crc);

	crc_reset = 1'b1;
	data_in = 1'b0;
	crc_ce = 1'b0;
	waitclock;
	waitclock;

	crc_reset = 1'b0;
	#100;

	/* this are examples from
	 * http://www.usb.org/developers/whitepapers/crcdes.pdf */
	usb_token(64'h08F4, 16);
	$display("crc5=0x%x", crc.crc5);
	if(~crc5_valid)
		$display("  ERROR: incorrect");
	#100;
	usb_token(64'hA8F7, 16);
	$display("crc5=0x%x", crc.crc5);
	if(~crc5_valid)
		$display("  ERROR: incorrect");
	#100;
	usb_token(64'h0E4E, 16);
	$display("crc5=0x%x", crc.crc5);
	if(~crc5_valid)
		$display("  ERROR: incorrect");
	#100;
	usb_token(64'h8017, 16);
	$display("crc5=0x%x", crc.crc5);
	if(~crc5_valid)
		$display("  ERROR: incorrect");
	#1000;
	usb_token(64'h008040C0F75E, 48);
	$display("crc16=0x%x", crc.crc16);
	if(~crc16_valid)
		$display("  ERROR: incorrect");
	#100;
	usb_token(64'hC4A2E6917038, 48);
	$display("crc16=0x%x", crc.crc16);
	if(~crc16_valid)
		$display("  ERROR: incorrect");
	#100;
	$finish;
end

endmodule
