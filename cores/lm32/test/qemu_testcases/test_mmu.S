.include "macros.inc"

initial_page:
start

mvhi r1, hi(data2)
ori r1, r1, lo(data2)
mvhi r2, hi(data1)
ori r2, r2, lo(data1)
calli update_dtlb

test_name MMU_DTLB_OFF
    lw r3, (r1+0)
    check_r3 0xc0cac01a

test_name MMU_DTLB_ON
    calli enable_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_r3 0xdeadbabe

test_name MMU_DTLB_MISS_1
    calli enable_dtlb
1:
    lw r3, (r1+0x1004)
    calli disable_dtlb
    check_excp 128

test_name MMU_DTLB_MISS_2
    check_reg ea 1b

test_name MMU_DTLB_MISS_3
    check_reg r24 data2+0x1004

test_name MMU_DTLB_MISS_4
    calli enable_dtlb
    lw r3, (r1+0x1004)
    lw r3, (r1+0x1004)
    calli disable_dtlb
    check_excp 128

test_name MMU_DTLB_MISS_5
    calli enable_dtlb
    mvi r3, 0x1234
    sw (r1+4), r3
    lw r3, (r1+0x1004)
    calli disable_dtlb
    check_excp 128

test_name MMU_DTLB_MISS_6
    lw r3, (r2+4)
    check_r3 0x1234

test_name MMU_DTLB_UPDATE_1
    calli flush_dtlb
    calli enable_dtlb
    ori r10, r1, 1
    ori r11, r2, 1
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11       # TLB entry setup
    lw r3, (r1+0)            # .. directly followed by a load
    calli disable_dtlb
    check_r3 0xdeadbabe

test_name MMU_DTLB_UPDATE_2
    calli flush_dtlb
    calli enable_dtlb
    ori r10, r1, 1
    ori r11, r2, 1
    wcsr TLBVADDR, r10
    lw r3, (r1+0)            # A load
    wcsr TLBPADDR, r11       # .. directly followed an TLB update
    calli disable_dtlb
    check_excp 128

test_name MMU_DTLB_FLUSH
    calli enable_dtlb
    calli flush_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_excp 128

test_name MMU_DTLB_INV
    mvhi r1, hi(data2)
    ori r1, r1, lo(data2)
    mvhi r2, hi(data1)
    ori r2, r2, lo(data1)
    calli update_dtlb

    calli enable_dtlb
    calli invalidate_dtlb
    lw r3, (r1+0)
    calli disable_dtlb
    check_excp 128

# make sure we have a mapping for the current code
mvhi r1, hi(initial_page)
ori r1, r1, lo(initial_page)
mv r2, r1
calli update_itlb

mvhi r1, hi(code2)
ori r1, r1, lo(code2)
mvhi r2, hi(code1)
ori r2, r2, lo(code1)
calli update_itlb

test_name MMU_ITLB_OFF
    calli code2
    check_r3 0x55

test_name MMU_ITLB_ON
    calli enable_itlb
    calli code2
    check_r3 0xaa

test_name MMU_ITLB_MISS_1
    calli enable_itlb
    calli code2+0x1004
    check_excp 256

test_name MMU_ITLB_MISS_2
    check_reg ea code2+0x1004

test_name MMU_ITLB_MISS_3
    check_reg r24 code2+0x1004

test_name MMU_ITLB_MISS_4
    wcsr DCC, r0
    nop
    nop
    nop
    calli enable_itlb
    lw r1, (r0+0)
    calli code2+0x1004
    check_excp 256

end

enable_itlb:
    rcsr r10, PSW
    ori r10, r10, 0x8
    wcsr PSW, r10
    nop
    nop
    nop
    ret

disable_itlb:
    rcsr r10, PSW
    andi r10, r10, 0xfff7
    wcsr PSW, r10
    nop
    nop
    nop
    ret

flush_itlb:
    mvi r10, 2
    wcsr TLBVADDR, r10
    ret

update_itlb:
    wcsr TLBVADDR, r1
    wcsr TLBPADDR, r2
    ret

invalidate_itlb:
    ori r10, r1, 4
    wcsr TLBVADDR, r10
    ret

enable_dtlb:
    rcsr r10, PSW
    ori r10, r10, 0x40
    wcsr PSW, r10
    ret

disable_dtlb:
    rcsr r10, PSW
    andi r10, r10, 0xffbf
    wcsr PSW, r10
    ret

flush_dtlb:
    mvi r10, 3
    wcsr TLBVADDR, r10
    ret

update_dtlb:
    ori r10, r1, 1
    ori r11, r2, 1
    wcsr TLBVADDR, r10
    wcsr TLBPADDR, r11
    ret

invalidate_dtlb:
    ori r10, r1, 5
    wcsr TLBVADDR, r10
    ret

.align 0x1000
code1:
    mvi r3, 0xaa
    # disable itlb
    rcsr r10, PSW
    andi r10, r10, 0xfff7
    wcsr PSW, r10
    nop
    nop
    nop
    ret

.align 0x1000
code2:
    mvi r3, 0x55
    # disable itlb
    rcsr r10, PSW
    andi r10, r10, 0xfff7
    wcsr PSW, r10
    nop
    nop
    nop
    ret

.data
.align 0x1000
data1: .long 0xdeadbabe
.align 0x1000
data2: .long 0xc0cac01a
