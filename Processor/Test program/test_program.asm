.data
MOST_NEG:		.word -2147483648 # most negative integer
MOST_POS:		.word 2147483647 # most positive integer
SMALL_NEG:		.word -101 # negative integer
SMALL_POS:		.word 101 # positive integer
MULT_SMALL_NEG:		.word -5 # multiplication negative integer
MULT_DIV_SMALL_POS:	.word 7 # multiplication & divisor positive integer
DEN_NEG:		.word -50 # negative denominator ($s)
DEN_POS:		.word 50 # positive denominator ($s)
DIV_NEG:		.word -7 # positive divisor ($t)
ALL_ONE:		.word -1 # one content
VALUE_ONE:		.word 1 # for division
FOUR:			.word 4 # used for max_int * 4
ADDR_IMM:		.word 0x10010020
INV_POS:		.word -102 # inverse of positive integer

# store the results
RESULT_BGEZ:		.word 0
RESULT_BEQ:		.word 0

RESULT_MULT_LO:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
RESULT_MULT_HI:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

RESULT_DIV_LO:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
RESULT_DIV_HI:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

RESULT_AND:		.word 0,0,0,0,0,0,0,0
RESULT_OR:		.word 0,0,0,0,0,0
RESULT_ORI:		.word 0,0,0,0,0,0
RESULT_SLT:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

RESULT_ADD:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
RESULT_ADDI:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
RESULT_SUB:		.word 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

.text
.globl test_lui

test_lui:
   lui $3 0xFFFF # ALL_ONES -1 result : -65536
   lui $4 0x0000 # All_ZEROS 0 result : 0
   lui $5 0xFF9C # IMM_NEG -100 result : -6553600
   lui $6 100 # IMM_POS 0x0064 result : 6553600
   
   lui $19 0x1001 # set up data upper address
   ori $19 $19 0x10010008 # $19 store the address 0x00040008
   
test_lw:
   lw $3 MOST_NEG # load the most negative integer
   lw $4 MOST_POS # load the most positive integer
   lw $5 SMALL_NEG # load a negative integer
   lw $6 SMALL_POS # load a positive integer
   lw $7 MULT_SMALL_NEG # load a negative integer for the multiplication
   lw $8 MULT_DIV_SMALL_POS # load a positive integer for the multiplication & division
   lw $9 DEN_NEG # load a negative integer for the division
   lw $10 DEN_POS # load a positive integer for the division
   lw $11 DIV_NEG # load a negative integer for the division
   lw $12 ALL_ONE # load 0xffffffff
   lw $13 FOUR # load a integer for max_int * 4
   lw $14 ADDR_IMM #
   lw $15 INV_POS # inverse of small_pos
   lw $16 0($19) # load an integer from an address
   lw $17 12($19)
   lw $18 VALUE_ONE
   lw $0 SMALL_POS # load an integer into RF[0]
   lw $0 0($19) # load an integer from an address into RF[0]
   
test_bgez:
   bgez $3, test_beq # most negative
sn_bgez:
   bgez $5, test_beq # small negative
zf_bgez:   
   bgez $0, sp_bgez # zero, jump forward
   nop
sp_bgez:
   bgez $6, zb_bgez # small positive
   nop
mp_bgez:
   bgez $4, end_bgez # most positive
   nop
zb_bgez:
   bgez $0, mp_bgez # zero, jump back
   nop
end_bgez:
   sw $4, RESULT_BGEZ # store a value to specify success

test_beq:
   beq $0, $12, test_mult # 0, 1
zv_beq:
   beq $0, $6, test_mult # 0, value
oz_beq:
   beq $12, $0, test_mult # 1, 0
ov_beq:
   beq $12, $6, test_mult # 1, value
vz_beq:
   beq $6, $0, test_mult # value , 0
vo_beq:
   beq $6, $12, test_mult # value, 1
zz_f_beq:
   beq $0, $0, zz_b_beq # 0 , 0   forward jump
   nop
oo_beq:
   beq $0, $0, vv_beq # 1, 1
   nop
vv_beq:
   beq $0, $0, end_beq # value, value
   nop
zz_b_beq:
   beq $0, $0, oo_beq # 0 , 0   back jump
   nop
end_beq:
   sw $4, RESULT_BEQ # store a value to specify success
 
test_mult:
   mult $3, $4		# mn * mp
   mflo $26		# 0x80000000
   mfhi $27		# 0xc0000000
   sw $26, RESULT_MULT_LO+0
   sw $27, RESULT_MULT_HI+0

   mult $3, $8		# mn * sp
   mflo $26		# 0x80000000
   mfhi $27		# 0xfffffffc
   sw $26, RESULT_MULT_LO+4
   sw $27, RESULT_MULT_HI+4
   
   mult $3, $0		# mn * r0
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+8
   sw $27, RESULT_MULT_HI+8

   mult $3, $7		# mn * sn
   mflo $26		# 0x80000000
   mfhi $27		# 0x00000002
   sw $26, RESULT_MULT_LO+12
   sw $27, RESULT_MULT_HI+12
   
   mult $3, $3		# mn * mn
   mflo $26		# 0x00000000
   mfhi $27		# 0x40000000
   sw $26, RESULT_MULT_LO+16
   sw $27, RESULT_MULT_HI+16
   
   
   mult $7, $4		# sn * mp
   mflo $26		# 0x80000005
   mfhi $27		# 0xfffffffd
   sw $26, RESULT_MULT_LO+20
   sw $27, RESULT_MULT_HI+20
   
   mult $7, $8		# sn * sp
   mflo $26		# 0xffffffdd
   mfhi $27		# 0xffffffff
   sw $26, RESULT_MULT_LO+24
   sw $27, RESULT_MULT_HI+24
   
   mult $7, $0		# sn * r0
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+28
   sw $27, RESULT_MULT_HI+28
   
   mult $7, $7		# sn * sn
   mflo $26		# 0x00000019
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+32
   sw $27, RESULT_MULT_HI+32
   
   mult $7, $3		# sn * mn
   mflo $26		# 0x80000000
   mfhi $27		# 0x00000002
   sw $26, RESULT_MULT_LO+36
   sw $27, RESULT_MULT_HI+36


   mult $0, $4		# r0 * mp
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+40
   sw $27, RESULT_MULT_HI+40

   mult $0, $8		# r0 * sp
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+44
   sw $27, RESULT_MULT_HI+44

   mult $0, $0		# r0 * r0
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+48
   sw $27, RESULT_MULT_HI+48

   mult $0, $7		# r0 * sn
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+52
   sw $27, RESULT_MULT_HI+52

   mult $0, $3		# r0 * mn
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+56
   sw $27, RESULT_MULT_HI+56

   
   mult $8, $4		# sp * mp
   mflo $26		# 0x7ffffff9
   mfhi $27		# 0x00000003
   sw $26, RESULT_MULT_LO+60
   sw $27, RESULT_MULT_HI+60

   mult $8, $8		# sp * sp
   mflo $26		# 0x00000031
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+64
   sw $27, RESULT_MULT_HI+64

   mult $8, $0		# sp * r0
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+68
   sw $27, RESULT_MULT_HI+68

   mult $8, $7		# sp * sn
   mflo $26		# 0xffffffdd
   mfhi $27		# 0xffffffff
   sw $26, RESULT_MULT_LO+72
   sw $27, RESULT_MULT_HI+72

   mult $8, $3		# sp * mn
   mflo $26		# 0x80000000
   mfhi $27		# 0xfffffffc
   sw $26, RESULT_MULT_LO+76
   sw $27, RESULT_MULT_HI+76

   
   mult $4, $4		# mp * mp
   mflo $26		# 0x00000001
   mfhi $27		# 0x3fffffff
   sw $26, RESULT_MULT_LO+80
   sw $27, RESULT_MULT_HI+80

   mult $4, $8		# mp * sp
   mflo $26		# 0x7ffffff9
   mfhi $27		# 0x00000003
   sw $26, RESULT_MULT_LO+84
   sw $27, RESULT_MULT_HI+84

   mult $4, $0		# mp * r0
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_MULT_LO+88
   sw $27, RESULT_MULT_HI+88

   mult $4, $7		# mp * sn
   mflo $26		# 0x80000005
   mfhi $27		# 0xfffffffd
   sw $26, RESULT_MULT_LO+92
   sw $27, RESULT_MULT_HI+92

   mult $4, $3		# mp * mn
   mflo $26		# 0x80000000
   mfhi $27		# 0xc0000000
   sw $26, RESULT_MULT_LO+96
   sw $27, RESULT_MULT_HI+96


test_div:
   div $3, $4		# mn / mp
   mflo $26		# 0xffffffff
   mfhi $27		# 0xffffffff
   sw $26, RESULT_DIV_LO+0
   sw $27, RESULT_DIV_HI+0

   div $3, $8		# mn / sp
   mflo $26		# 0xedb6db6e
   mfhi $27		# 0xfffffffe
   sw $26, RESULT_DIV_LO+4
   sw $27, RESULT_DIV_HI+4

   div $3, $18		# mn / 1
   mflo $26		# 0x80000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+8
   sw $27, RESULT_DIV_HI+8

   div $3, $11		# mn / sn
   mflo $26		# 0x12492492
   mfhi $27		# 0xfffffffe
   sw $26, RESULT_DIV_LO+12
   sw $27, RESULT_DIV_HI+12

   div $3, $3		# mn / mn
   mflo $26		# 0x00000001
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+16
   sw $27, RESULT_DIV_HI+16

   
   div $9, $4		# sn / mp
   mflo $26		# 0x00000000
   mfhi $27		# 0xffffffce
   sw $26, RESULT_DIV_LO+20
   sw $27, RESULT_DIV_HI+20

   div $9, $8		# sn / sp
   mflo $26		# 0xfffffff9
   mfhi $27		# 0xffffffff
   sw $26, RESULT_DIV_LO+24
   sw $27, RESULT_DIV_HI+24

   div $9, $18		# sn / 1
   mflo $26		# 0xffffffce
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+28
   sw $27, RESULT_DIV_HI+28

   div $9, $11		# sn / sn
   mflo $26		# 0x00000007
   mfhi $27		# 0xffffffff
   sw $26, RESULT_DIV_LO+32
   sw $27, RESULT_DIV_HI+32

   div $9, $3		# sn / mn
   mflo $26		# 0x00000000
   mfhi $27		# 0xffffffce
   sw $26, RESULT_DIV_LO+36
   sw $27, RESULT_DIV_HI+36

   
   div $0, $4		# r0 / mp
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+40
   sw $27, RESULT_DIV_HI+40

   div $0, $8		# r0 / sp
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+44
   sw $27, RESULT_DIV_HI+44

   div $0, $18		# r0 / 1
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+48
   sw $27, RESULT_DIV_HI+48

   div $0, $11		# r0 / sn
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+52
   sw $27, RESULT_DIV_HI+52

   div $0, $3		# r0 / mn
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+56
   sw $27, RESULT_DIV_HI+56

   
   div $10, $4		# sp / mp
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000032
   sw $26, RESULT_DIV_LO+60
   sw $27, RESULT_DIV_HI+60

   div $10, $8		# sp / sp
   mflo $26		# 0x00000007
   mfhi $27		# 0x00000001
   sw $26, RESULT_DIV_LO+64
   sw $27, RESULT_DIV_HI+64

   div $10, $18		# sp / 1
   mflo $26		# 0x00000032
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+68
   sw $27, RESULT_DIV_HI+68

   div $10, $11		# sp / sn
   mflo $26		# 0xfffffff9
   mfhi $27		# 0x00000001
   sw $26, RESULT_DIV_LO+72
   sw $27, RESULT_DIV_HI+72

   div $10, $3		# sp / mn
   mflo $26		# 0x00000000
   mfhi $27		# 0x00000032
   sw $26, RESULT_DIV_LO+76
   sw $27, RESULT_DIV_HI+76

   
   div $4, $4		# mp / mp
   mflo $26		# 0x00000001
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+80
   sw $27, RESULT_DIV_HI+80

   div $4, $8		# mp / sp
   mflo $26		# 0x12492492
   mfhi $27		# 0x00000001
   sw $26, RESULT_DIV_LO+84
   sw $27, RESULT_DIV_HI+84

   div $4, $18		# mp / 1
   mflo $26		# 0x7fffffff
   mfhi $27		# 0x00000000
   sw $26, RESULT_DIV_LO+88
   sw $27, RESULT_DIV_HI+8

   div $4, $11		# mp / sn
   mflo $26		# 0xedb6db6e
   mfhi $27		# 0x00000001
   sw $26, RESULT_DIV_LO+92
   sw $27, RESULT_DIV_HI+92

   div $4, $3		# mp / mn
   mflo $26		# 0x00000000
   mfhi $27		# 0x7fffffff
   sw $26, RESULT_DIV_LO+96
   sw $27, RESULT_DIV_HI+96

   
   div $6, $0		# div by 0
   mflo $26		# same value as previous
   mfhi $27		# same value as previous
   sw $26, RESULT_DIV_LO+100
   sw $27, RESULT_DIV_HI+100


test_and:
   and $25, $12, $12		# 1 & 1
   sw $25, RESULT_AND+0		# store result
   
   and $25, $12, $6		# 1 & value
   sw $25, RESULT_AND+4		# store result
   
   and $25, $12, $0		# 1 & 0
   sw $25, RESULT_AND+8		# store result
   
   and $25, $0, $12		# 0 & 1
   sw $25, RESULT_AND+12	# store result
   
   and $25, $6, $12		# value & 1
   sw $25, RESULT_AND+16	# store result
   
   and $25, $6, $6		# value & value
   sw $25, RESULT_AND+20	# store result
   
   and $25, $6, $0		# value & 0
   sw $25, RESULT_AND+24	# store result
   
   and $25, $6, $15		# value & inverse value
   sw $25, RESULT_AND+28	# store result
   
   
test_or:
   or $25, $12, $6		# 1 | value
   sw $25, RESULT_OR+0		# store result

   or $25, $0, $6		# 0 | value
   sw $25, RESULT_OR+4		# store result

   or $25, $6, $12		# value | 1
   sw $25, RESULT_OR+8		# store result

   or $25, $6, $6		# value | value
   sw $25, RESULT_OR+12		# store result

   or $25, $6, $0		# value | 0
   sw $25, RESULT_OR+16		# store result

   or $25, $6, $15		# value | inverse value
   sw $25, RESULT_OR+20		# store result
   
   
test_ori:
   ori $25, $12, 101		# 1 | value
   sw $25, RESULT_ORI+0		# store result

   ori $25, $0, 101		# 0 | value
   sw $25, RESULT_ORI+4		# store result

   ori $25, $6, 65535		# value | 1
   sw $25, RESULT_ORI+8		# store result

   ori $25, $6, 101		# value | value
   sw $25, RESULT_ORI+12	# store result

   ori $25, $6, 0		# value | 0
   sw $25, RESULT_ORI+16	# store result

   ori $25, $6, 65434		# value | inverse value
   sw $25, RESULT_ORI+20	# store result


test_slt:
   slt $25, $3, $4		# mn < mp
   sw $25, RESULT_SLT+0		# store result
   slt $25, $3, $6		# mn < sp
   sw $25, RESULT_SLT+4		# store result
   slt $25, $3, $0		# mn < r0
   sw $25, RESULT_SLT+8		# store result
   slt $25, $3, $5		# mn < sn
   sw $25, RESULT_SLT+12	# store result
   slt $25, $3, $3		# mn < mn
   sw $25, RESULT_SLT+16	# store result
   
   slt $25, $5, $4		# sn < mp
   sw $25, RESULT_SLT+20	# store result
   slt $25, $5, $6		# sn < sp
   sw $25, RESULT_SLT+24	# store result
   slt $25, $5, $0		# sn < r0
   sw $25, RESULT_SLT+28	# store result
   slt $25, $5, $5		# sn < sn
   sw $25, RESULT_SLT+32	# store result
   slt $25, $5, $3		# sn < mn
   sw $25, RESULT_SLT+36	# store result

   slt $25, $0, $4		# r0 < mp
   sw $25, RESULT_SLT+40	# store result
   slt $25, $0, $6		# r0 < sp
   sw $25, RESULT_SLT+44	# store result
   slt $25, $0, $0		# r0 < r0
   sw $25, RESULT_SLT+48	# store result
   slt $25, $0, $5		# r0 < sn
   sw $25, RESULT_SLT+52	# store result
   slt $25, $0, $3		# r0 < mn
   sw $25, RESULT_SLT+56	# store result
   
   slt $25, $6, $4		# sp < mp
   sw $25, RESULT_SLT+60	# store result
   slt $25, $6, $6		# sp < sp
   sw $25, RESULT_SLT+64	# store result
   slt $25, $6, $0		# sp < r0
   sw $25, RESULT_SLT+68	# store result
   slt $25, $6, $5		# sp < sn
   sw $25, RESULT_SLT+72	# store result
   slt $25, $6, $3		# sp < mn
   sw $25, RESULT_SLT+76	# store result
   
   slt $25, $4, $4		# mp < mp
   sw $25, RESULT_SLT+80	# store result
   slt $25, $4, $6		# mp < sp
   sw $25, RESULT_SLT+84	# store result
   slt $25, $4, $0		# mp < r0
   sw $25, RESULT_SLT+88	# store result
   slt $25, $4, $5		# mp < sn
   sw $25, RESULT_SLT+92	# store result
   slt $25, $4, $3		# mp < mn
   sw $25, RESULT_SLT+96	# store result


test_add:
   add $25, $3, $4		# mn + mp
   sw $25, RESULT_ADD+0		# store result
   add $25, $3, $6		# mn + sp
   sw $25, RESULT_ADD+4		# store result
   add $25, $3, $0		# mn + r0
   sw $25, RESULT_ADD+8		# store result
   
   add $25, $5, $4		# sn + mp
   sw $25, RESULT_ADD+12	# store result
   add $25, $5, $6		# sn + sp
   sw $25, RESULT_ADD+16	# store result
   add $25, $5, $0		# sn + r0
   sw $25, RESULT_ADD+20	# store result
   add $25, $5, $5		# sn + sn
   sw $25, RESULT_ADD+24	# store result
   
   add $25, $0, $4		# r0 + mp
   sw $25, RESULT_ADD+28	# store result
   add $25, $0, $6		# r0 + sp
   sw $25, RESULT_ADD+32	# store result
   add $25, $0, $0		# r0 + r0
   sw $25, RESULT_ADD+36	# store result
   add $25, $0, $5		# r0 + sn
   sw $25, RESULT_ADD+40	# store result
   add $25, $0, $3		# r0 + mn
   sw $25, RESULT_ADD+44	# store result
   
   add $25, $6, $6		# sp + sp
   sw $25, RESULT_ADD+48	# store result
   add $25, $6, $0		# sp + r0
   sw $25, RESULT_ADD+52	# store result
   add $25, $6, $5		# sp + sn
   sw $25, RESULT_ADD+56	# store result
   add $25, $6, $3		# sp + mn
   sw $25, RESULT_ADD+60	# store result
   
   add $25, $4, $0		# mp + r0
   sw $25, RESULT_ADD+64	# store result
   add $25, $4, $5		# mp + sn
   sw $25, RESULT_ADD+68	# store result
   add $25, $4, $3		# mp + mn
   sw $25, RESULT_ADD+72	# store result


test_addi:
   addi $25, $3, 32767		# mn + mp
   sw $25, RESULT_ADDI+0	# store result
   addi $25, $3, 101		# mn + sp
   sw $25, RESULT_ADDI+4	# store result
   addi $25, $3, 0		# mn + r0
   sw $25, RESULT_ADDI+8	# store result
   
   addi $25, $5, 32767		# sn + mp
   sw $25, RESULT_ADDI+12	# store result
   addi $25, $5, 101		# sn + sp
   sw $25, RESULT_ADDI+16	# store result
   addi $25, $5, 0		# sn + r0
   sw $25, RESULT_ADDI+20	# store result
   addi $25, $5, -101		# sn + sn
   sw $25, RESULT_ADDI+24	# store result
   addi $25, $5, -32768		# sn + mn
   sw $25, RESULT_ADDI+28	# store result
   
   addi $25, $0, 32767		# r0 + mp
   sw $25, RESULT_ADDI+32	# store result
   addi $25, $0, 101		# r0 + sp
   sw $25, RESULT_ADDI+36	# store result
   addi $25, $0, 0		# r0 + r0
   sw $25, RESULT_ADDI+40	# store result
   addi $25, $0, -101		# r0 + sn
   sw $25, RESULT_ADDI+44	# store result
   addi $25, $0, -32768		# r0 + mn
   sw $25, RESULT_ADDI+48	# store result
   
   addi $25, $6, 32767		# sp + mp
   sw $25, RESULT_ADDI+52	# store result
   addi $25, $6, 101		# sp + sp
   sw $25, RESULT_ADDI+56	# store result
   addi $25, $6, 0		# sp + r0
   sw $25, RESULT_ADDI+60	# store result
   addi $25, $6, -101		# sp + sn
   sw $25, RESULT_ADDI+64	# store result
   addi $25, $6, -32768		# sp + mn
   sw $25, RESULT_ADDI+68	# store result
   
   addi $25, $4, 0		# mp + r0
   sw $25, RESULT_ADDI+72	# store result
   addi $25, $4, -101		# mp + sn
   sw $25, RESULT_ADDI+76	# store result
   addi $25, $4, -32768		# mp + mn
   sw $25, RESULT_ADDI+80	# store result


test_sub:
   sub $25, $3, $0		# mn - r0
   sw $25, RESULT_SUB+0		# store result
   sub $25, $3, $5		# mn - sn
   sw $25, RESULT_SUB+4		# store result
   sub $25, $3, $3		# mn - mn
   sw $25, RESULT_SUB+8		# store result
   
   sub $25, $5, $6		# sn - sp
   sw $25, RESULT_SUB+12	# store result
   sub $25, $5, $0		# sn - r0
   sw $25, RESULT_SUB+16	# store result
   sub $25, $5, $5		# sn - sn
   sw $25, RESULT_SUB+20	# store result
   sub $25, $5, $3		# sn - mn
   sw $25, RESULT_SUB+24	# store result
   
   sub $25, $0, $4		# r0 - mp
   sw $25, RESULT_SUB+28	# store result
   sub $25, $0, $6		# r0 - sp
   sw $25, RESULT_SUB+32	# store result
   sub $25, $0, $0		# r0 - r0
   sw $25, RESULT_SUB+36	# store result
   sub $25, $0, $5		# r0 - sn
   sw $25, RESULT_SUB+40	# store result
   
   sub $25, $6, $4		# sp - mp
   sw $25, RESULT_SUB+44	# store result
   sub $25, $6, $6		# sp - sp
   sw $25, RESULT_SUB+48	# store result
   sub $25, $6, $0		# sp - r0
   sw $25, RESULT_SUB+52	# store result
   sub $25, $6, $5		# sp - sn
   sw $25, RESULT_SUB+56	# store result
   
   sub $25, $4, $4		# mp - mp
   sw $25, RESULT_SUB+60	# store result
   sub $25, $4, $6		# mp - sp
   sw $25, RESULT_SUB+64	# store result
   sub $25, $4, $0		# mp - r0
   sw $25, RESULT_SUB+68	# store result


test_sw:
   sw $7 -8($19)
   sw $8 -4($19)
   sw $9 0($19)
   sw $10 4($19)
   sw $11 ADDR_IMM
   sw $12 SMALL_POS
   sw $0  ($19) # store RF[0] to an address
   
end:
   nop # halt
