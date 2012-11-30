.include "macros.inc"
.text
.global _start

_start:
_reset_handler:
	xor r0, r0, r0
	mvhi r1, hi(_start)
	ori r1, r1, lo(_start)
	wcsr eba, r1
	wcsr deba, r1
	bi _main
	nop
	nop

_breakpoint_handler:
	ori r25, r25, 1
	addi ra, ba, 4
	ret
	nop
	nop
	nop
	nop
	nop

_instruction_bus_error_handler:
	ori r25, r25, 2
	addi ra, ea, 4
	ret
	nop
	nop
	nop
	nop
	nop

_watchpoint_handler:
	ori r25, r25, 4
	addi ra, ba, 4
	ret
	nop
	nop
	nop
	nop
	nop

_data_bus_error_handler:
	ori r25, r25, 8
	addi ra, ea, 4
	ret
	nop
	nop
	nop
	nop
	nop

_divide_by_zero_handler:
	ori r25, r25, 16
	addi ra, ea, 4
	ret
	nop
	nop
	nop
	nop
	nop

_interrupt_handler:
	ori r25, r25, 32
	addi ra, ea, 4
	ret
	nop
	nop
	nop
	nop
	nop

_system_call_handler:
	ori r25, r25, 64
	addi ra, ea, 4
	# clear eusr flag to exit user mode
	rcsr r24, PSW
	andi r24, r24, 0xfbff
	wcsr PSW, r24
	ret
	nop
	nop

_dtlb_miss_handler:
	ori r25, r25, 128
	rcsr r24, TLBVADDR
	addi ra, ea, 4
	ret
	nop
	nop
	nop
	nop

_itlb_miss_handler:
	ori r25, r25, 256
	rcsr r24, TLBVADDR
	ret
	nop
	nop
	nop
	nop
	nop
