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

module softusb_crc(
	input usb_clk,

	input crc_reset,
	input data,
	input crc_ce,

	output reg [4:0] crc5,
	output reg [15:0] crc16,

	output crc5_valid,
	output crc16_valid
);

always @(posedge usb_clk) begin
	if(crc_reset)
		crc5 <= 5'h1f;
	else if (crc_ce) begin
		crc5[0] <= data ^ crc5[4];
		crc5[1] <= crc5[0];
		crc5[2] <= crc5[1] ^ data ^ crc5[4];
		crc5[3] <= crc5[2];
		crc5[4] <= crc5[3];
	end
end
assign crc5_valid = (crc5 == 5'b01100);

always @(posedge usb_clk) begin
	if(crc_reset)
		crc16 <= 16'hffff;
	else if (crc_ce) begin
		crc16[0]  <= data ^ crc16[15];
		crc16[1]  <= crc16[0];
		crc16[2]  <= crc16[1] ^ data ^ crc16[15];
		crc16[3]  <= crc16[2];
		crc16[4]  <= crc16[3];
		crc16[5]  <= crc16[4];
		crc16[6]  <= crc16[5];
		crc16[7]  <= crc16[6];
		crc16[8]  <= crc16[7];
		crc16[9]  <= crc16[8];
		crc16[10] <= crc16[9];
		crc16[11] <= crc16[10];
		crc16[12] <= crc16[11];
		crc16[13] <= crc16[12];
		crc16[14] <= crc16[13];
		crc16[15] <= crc16[14] ^ data ^ crc16[15];
	end
end
assign crc16_valid = (crc16 == 16'b1000000000001101);

endmodule
