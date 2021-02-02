-- File memory.vhd
-- A memory model with properties:
-- a) seperate data and text segments
--   A MIPS assembler generates the code for TEXT and DATA segment.
--   VARIABLE prg:text_segment:= <program>
--   VARIABLE data:data_segment:= <data?
--   Check that the text_base_address and data_base_size are correct (package memory_config)
-- b) a handshake protocol is used.
--  
-- read from memory
--
--         data read from memory valid
--              <--------> 
--
--       +---------------+
--    ---+               +------------- read; note: a_bus must be valid when read is '1'.
--             +----------------+
--    ---------+                +------ ready
--
-- write to memory
--
--  data to be written must be valid
--        <--------------> 
--
--       +---------------+
--    ---+               +------------- write; note: a_bus must be valid when write is '1'.
--             +----------------+
--    ---------+                +------ ready
--
-- c) if during a read/write the address is not in the data or text segment a violation is reported
--

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
LIBRARY work;
USE work.memory_config.ALL;

ENTITY memory IS
   PORT(d_busout : OUT std_logic_vector(31 DOWNTO 0);
        d_busin  : IN  std_logic_vector(31 DOWNTO 0);
        a_bus    : IN  std_logic_vector(31 DOWNTO 0);
        clk      : IN  std_ulogic;
        write    : IN  std_ulogic;
        read     : IN  std_ulogic;
        ready    : OUT std_ulogic
       );
END memory;

ARCHITECTURE behaviour OF memory IS
   ALIAS word_address : std_logic_vector(31 DOWNTO 2) IS a_bus(31 DOWNTO 2);
   SIGNAL d_busouti : std_logic_vector(31 DOWNTO 0);
   CONSTANT unknown : std_logic_vector(31 DOWNTO 0) := (OTHERS=>'X');  
   TYPE states IS (idle, rd_wr_nrdy, rd_wr_rdy);
   SIGNAL state : states := idle; -- models state of handshake protocol
BEGIN

   PROCESS
      TYPE text_segment IS ARRAY 
         (natural RANGE text_base_address/4 TO text_base_address/4+text_base_size) -- in model has each memory location 4 bytes, therefore divide by 4
         OF string(8 DOWNTO 1);
      TYPE data_segment IS ARRAY 
         (natural RANGE data_base_address/4 TO data_base_address/4+data_base_size)
         OF string(8 DOWNTO 1);
       
      VARIABLE prg:text_segment:=
      (
-- Code        Basic                     Source
"3c03ffff", --  lui $3,0x0000ffff     40      lui $3 0xFFFF # ALL_ONES -1 result : -65536
"3c040000", --  lui $4,0x00000000     41      lui $4 0x0000 # All_ZEROS 0 result : 0
"3c05ff9c", --  lui $5,0x0000ff9c     42      lui $5 0xFF9C # IMM_NEG -100 result : -6553600
"3c060064", --  lui $6,0x00000064     43      lui $6 100 # IMM_POS 0x0064 result : 6553600
"3c131001", --  lui $19,0x00001001    45      lui $19 0x1001 # set up data upper address
"3c011001", --  lui $1,0x00001001     46      ori $19 $19 0x10010008 # $19 store the address 0x00040008
"34210008", --  ori $1,$1,0x00000008       
"02619825", --  or $19,$19,$1              
"3c011001", --  lui $1,0x00001001     49      lw $3 MOST_NEG # load the most negative integer
"8c230000", --  lw $3,0x00000000($1)       
"3c011001", --  lui $1,0x00001001     50      lw $4 MOST_POS # load the most positive integer
"8c240004", --  lw $4,0x00000004($1)       
"3c011001", --  lui $1,0x00001001     51      lw $5 SMALL_NEG # load a negative integer
"8c250008", --  lw $5,0x00000008($1)       
"3c011001", --  lui $1,0x00001001     52      lw $6 SMALL_POS # load a positive integer
"8c26000c", --  lw $6,0x0000000c($1)       
"3c011001", --  lui $1,0x00001001     53      lw $7 MULT_SMALL_NEG # load a negative integer for the multiplication
"8c270010", --  lw $7,0x00000010($1)       
"3c011001", --  lui $1,0x00001001     54      lw $8 MULT_DIV_SMALL_POS # load a positive integer for the multiplication & division
"8c280014", --  lw $8,0x00000014($1)       
"3c011001", --  lui $1,0x00001001     55      lw $9 DEN_NEG # load a negative integer for the division
"8c290018", --  lw $9,0x00000018($1)       
"3c011001", --  lui $1,0x00001001     56      lw $10 DEN_POS # load a positive integer for the division
"8c2a001c", --  lw $10,0x0000001c($1)      
"3c011001", --  lui $1,0x00001001     57      lw $11 DIV_NEG # load a negative integer for the division
"8c2b0020", --  lw $11,0x00000020($1)      
"3c011001", --  lui $1,0x00001001     58      lw $12 ALL_ONE # load 0xffffffff
"8c2c0024", --  lw $12,0x00000024($1)      
"3c011001", --  lui $1,0x00001001     59      lw $13 FOUR # load a integer for max_int * 4
"8c2d002c", --  lw $13,0x0000002c($1)      
"3c011001", --  lui $1,0x00001001     60      lw $14 ADDR_IMM #
"8c2e0030", --  lw $14,0x00000030($1)      
"3c011001", --  lui $1,0x00001001     61      lw $15 INV_POS # inverse of small_pos
"8c2f0034", --  lw $15,0x00000034($1)      
"8e700000", --  lw $16,0x00000000($19)62      lw $16 0($19) # load an integer from an address
"8e71000c", --  lw $17,0x0000000c($19)63      lw $17 12($19)
"3c011001", --  lui $1,0x00001001     64      lw $18 VALUE_ONE
"8c320028", --  lw $18,0x00000028($1)      
"3c011001", --  lui $1,0x00001001     65      lw $0 SMALL_POS # load an integer into RF[0]
"8c20000c", --  lw $0,0x0000000c($1)       
"8e600000", --  lw $0,0x00000000($19) 66      lw $0 0($19) # load an integer from an address into RF[0]
"0461000b", --  bgez $3,0x0000000b    69      bgez $3, test_beq # most negative
"04a1000a", --  bgez $5,0x0000000a    71      bgez $5, test_beq # small negative
"04010001", --  bgez $0,0x00000001    73      bgez $0, sp_bgez # zero, jump forward
"00000000", --  nop                   74      nop
"04c10003", --  bgez $6,0x00000003    76      bgez $6, zb_bgez # small positive
"00000000", --  nop                   77      nop
"04810003", --  bgez $4,0x00000003    79      bgez $4, end_bgez # most positive
"00000000", --  nop                   80      nop
"0401fffd", --  bgez $0,0xfffffffd    82      bgez $0, mp_bgez # zero, jump back
"00000000", --  nop                   83      nop
"3c011001", --  lui $1,0x00001001     85      sw $4, RESULT_BGEZ # store a value to specify success
"ac240038", --  sw $4,0x00000038($1)       
"100c000f", --  beq $0,$12,0x0000000f 88      beq $0, $12, test_mult # 0, 1
"1006000e", --  beq $0,$6,0x0000000e  90      beq $0, $6, test_mult # 0, value
"1180000d", --  beq $12,$0,0x0000000d 92      beq $12, $0, test_mult # 1, 0
"1186000c", --  beq $12,$6,0x0000000c 94      beq $12, $6, test_mult # 1, value
"10c0000b", --  beq $6,$0,0x0000000b  96      beq $6, $0, test_mult # value , 0
"10cc000a", --  beq $6,$12,0x0000000a 98      beq $6, $12, test_mult # value, 1
"10000005", --  beq $0,$0,0x00000005  100     beq $0, $0, zz_b_beq # 0 , 0   forward jump
"00000000", --  nop                   101     nop
"10000001", --  beq $0,$0,0x00000001  103     beq $0, $0, vv_beq # 1, 1
"00000000", --  nop                   104     nop
"10000003", --  beq $0,$0,0x00000003  106     beq $0, $0, end_beq # value, value
"00000000", --  nop                   107     nop
"1000fffb", --  beq $0,$0,0xfffffffb  109     beq $0, $0, oo_beq # 0 , 0   back jump
"00000000", --  nop                   110     nop
"3c011001", --  lui $1,0x00001001     112     sw $4, RESULT_BEQ # store a value to specify success
"ac24003c", --  sw $4,0x0000003c($1)       
"00640018", --  mult $3,$4            115     mult $3, $4		# mn * mp
"0000d012", --  mflo $26              116     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              117     mfhi $27		# 0xc0000000
"3c011001", --  lui $1,0x00001001     118     sw $26, RESULT_MULT_LO+0
"ac3a0040", --  sw $26,0x00000040($1)      
"3c011001", --  lui $1,0x00001001     119     sw $27, RESULT_MULT_HI+0
"ac3b00a4", --  sw $27,0x000000a4($1)      
"00680018", --  mult $3,$8            121     mult $3, $8		# mn * sp
"0000d012", --  mflo $26              122     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              123     mfhi $27		# 0xfffffffc
"3c011001", --  lui $1,0x00001001     124     sw $26, RESULT_MULT_LO+4
"ac3a0044", --  sw $26,0x00000044($1)      
"3c011001", --  lui $1,0x00001001     125     sw $27, RESULT_MULT_HI+4
"ac3b00a8", --  sw $27,0x000000a8($1)      
"00600018", --  mult $3,$0            127     mult $3, $0		# mn * r0
"0000d012", --  mflo $26              128     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              129     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     130     sw $26, RESULT_MULT_LO+8
"ac3a0048", --  sw $26,0x00000048($1)      
"3c011001", --  lui $1,0x00001001     131     sw $27, RESULT_MULT_HI+8
"ac3b00ac", --  sw $27,0x000000ac($1)      
"00670018", --  mult $3,$7            133     mult $3, $7		# mn * sn
"0000d012", --  mflo $26              134     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              135     mfhi $27		# 0x00000002
"3c011001", --  lui $1,0x00001001     136     sw $26, RESULT_MULT_LO+12
"ac3a004c", --  sw $26,0x0000004c($1)      
"3c011001", --  lui $1,0x00001001     137     sw $27, RESULT_MULT_HI+12
"ac3b00b0", --  sw $27,0x000000b0($1)      
"00630018", --  mult $3,$3            139     mult $3, $3		# mn * mn
"0000d012", --  mflo $26              140     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              141     mfhi $27		# 0x40000000
"3c011001", --  lui $1,0x00001001     142     sw $26, RESULT_MULT_LO+16
"ac3a0050", --  sw $26,0x00000050($1)      
"3c011001", --  lui $1,0x00001001     143     sw $27, RESULT_MULT_HI+16
"ac3b00b4", --  sw $27,0x000000b4($1)      
"00e40018", --  mult $7,$4            146     mult $7, $4		# sn * mp
"0000d012", --  mflo $26              147     mflo $26		# 0x80000005
"0000d810", --  mfhi $27              148     mfhi $27		# 0xfffffffd
"3c011001", --  lui $1,0x00001001     149     sw $26, RESULT_MULT_LO+20
"ac3a0054", --  sw $26,0x00000054($1)      
"3c011001", --  lui $1,0x00001001     150     sw $27, RESULT_MULT_HI+20
"ac3b00b8", --  sw $27,0x000000b8($1)      
"00e80018", --  mult $7,$8            152     mult $7, $8		# sn * sp
"0000d012", --  mflo $26              153     mflo $26		# 0xffffffdd
"0000d810", --  mfhi $27              154     mfhi $27		# 0xffffffff
"3c011001", --  lui $1,0x00001001     155     sw $26, RESULT_MULT_LO+24
"ac3a0058", --  sw $26,0x00000058($1)      
"3c011001", --  lui $1,0x00001001     156     sw $27, RESULT_MULT_HI+24
"ac3b00bc", --  sw $27,0x000000bc($1)      
"00e00018", --  mult $7,$0            158     mult $7, $0		# sn * r0
"0000d012", --  mflo $26              159     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              160     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     161     sw $26, RESULT_MULT_LO+28
"ac3a005c", --  sw $26,0x0000005c($1)      
"3c011001", --  lui $1,0x00001001     162     sw $27, RESULT_MULT_HI+28
"ac3b00c0", --  sw $27,0x000000c0($1)      
"00e70018", --  mult $7,$7            164     mult $7, $7		# sn * sn
"0000d012", --  mflo $26              165     mflo $26		# 0x00000019
"0000d810", --  mfhi $27              166     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     167     sw $26, RESULT_MULT_LO+32
"ac3a0060", --  sw $26,0x00000060($1)      
"3c011001", --  lui $1,0x00001001     168     sw $27, RESULT_MULT_HI+32
"ac3b00c4", --  sw $27,0x000000c4($1)      
"00e30018", --  mult $7,$3            170     mult $7, $3		# sn * mn
"0000d012", --  mflo $26              171     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              172     mfhi $27		# 0x00000002
"3c011001", --  lui $1,0x00001001     173     sw $26, RESULT_MULT_LO+36
"ac3a0064", --  sw $26,0x00000064($1)      
"3c011001", --  lui $1,0x00001001     174     sw $27, RESULT_MULT_HI+36
"ac3b00c8", --  sw $27,0x000000c8($1)      
"00040018", --  mult $0,$4            177     mult $0, $4		# r0 * mp
"0000d012", --  mflo $26              178     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              179     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     180     sw $26, RESULT_MULT_LO+40
"ac3a0068", --  sw $26,0x00000068($1)      
"3c011001", --  lui $1,0x00001001     181     sw $27, RESULT_MULT_HI+40
"ac3b00cc", --  sw $27,0x000000cc($1)      
"00080018", --  mult $0,$8            183     mult $0, $8		# r0 * sp
"0000d012", --  mflo $26              184     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              185     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     186     sw $26, RESULT_MULT_LO+44
"ac3a006c", --  sw $26,0x0000006c($1)      
"3c011001", --  lui $1,0x00001001     187     sw $27, RESULT_MULT_HI+44
"ac3b00d0", --  sw $27,0x000000d0($1)      
"00000018", --  mult $0,$0            189     mult $0, $0		# r0 * r0
"0000d012", --  mflo $26              190     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              191     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     192     sw $26, RESULT_MULT_LO+48
"ac3a0070", --  sw $26,0x00000070($1)      
"3c011001", --  lui $1,0x00001001     193     sw $27, RESULT_MULT_HI+48
"ac3b00d4", --  sw $27,0x000000d4($1)      
"00070018", --  mult $0,$7            195     mult $0, $7		# r0 * sn
"0000d012", --  mflo $26              196     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              197     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     198     sw $26, RESULT_MULT_LO+52
"ac3a0074", --  sw $26,0x00000074($1)      
"3c011001", --  lui $1,0x00001001     199     sw $27, RESULT_MULT_HI+52
"ac3b00d8", --  sw $27,0x000000d8($1)      
"00030018", --  mult $0,$3            201     mult $0, $3		# r0 * mn
"0000d012", --  mflo $26              202     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              203     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     204     sw $26, RESULT_MULT_LO+56
"ac3a0078", --  sw $26,0x00000078($1)      
"3c011001", --  lui $1,0x00001001     205     sw $27, RESULT_MULT_HI+56
"ac3b00dc", --  sw $27,0x000000dc($1)      
"01040018", --  mult $8,$4            208     mult $8, $4		# sp * mp
"0000d012", --  mflo $26              209     mflo $26		# 0x7ffffff9
"0000d810", --  mfhi $27              210     mfhi $27		# 0x00000003
"3c011001", --  lui $1,0x00001001     211     sw $26, RESULT_MULT_LO+60
"ac3a007c", --  sw $26,0x0000007c($1)      
"3c011001", --  lui $1,0x00001001     212     sw $27, RESULT_MULT_HI+60
"ac3b00e0", --  sw $27,0x000000e0($1)      
"01080018", --  mult $8,$8            214     mult $8, $8		# sp * sp
"0000d012", --  mflo $26              215     mflo $26		# 0x00000031
"0000d810", --  mfhi $27              216     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     217     sw $26, RESULT_MULT_LO+64
"ac3a0080", --  sw $26,0x00000080($1)      
"3c011001", --  lui $1,0x00001001     218     sw $27, RESULT_MULT_HI+64
"ac3b00e4", --  sw $27,0x000000e4($1)      
"01000018", --  mult $8,$0            220     mult $8, $0		# sp * r0
"0000d012", --  mflo $26              221     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              222     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     223     sw $26, RESULT_MULT_LO+68
"ac3a0084", --  sw $26,0x00000084($1)      
"3c011001", --  lui $1,0x00001001     224     sw $27, RESULT_MULT_HI+68
"ac3b00e8", --  sw $27,0x000000e8($1)      
"01070018", --  mult $8,$7            226     mult $8, $7		# sp * sn
"0000d012", --  mflo $26              227     mflo $26		# 0xffffffdd
"0000d810", --  mfhi $27              228     mfhi $27		# 0xffffffff
"3c011001", --  lui $1,0x00001001     229     sw $26, RESULT_MULT_LO+72
"ac3a0088", --  sw $26,0x00000088($1)      
"3c011001", --  lui $1,0x00001001     230     sw $27, RESULT_MULT_HI+72
"ac3b00ec", --  sw $27,0x000000ec($1)      
"01030018", --  mult $8,$3            232     mult $8, $3		# sp * mn
"0000d012", --  mflo $26              233     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              234     mfhi $27		# 0xfffffffc
"3c011001", --  lui $1,0x00001001     235     sw $26, RESULT_MULT_LO+76
"ac3a008c", --  sw $26,0x0000008c($1)      
"3c011001", --  lui $1,0x00001001     236     sw $27, RESULT_MULT_HI+76
"ac3b00f0", --  sw $27,0x000000f0($1)      
"00840018", --  mult $4,$4            239     mult $4, $4		# mp * mp
"0000d012", --  mflo $26              240     mflo $26		# 0x00000001
"0000d810", --  mfhi $27              241     mfhi $27		# 0x3fffffff
"3c011001", --  lui $1,0x00001001     242     sw $26, RESULT_MULT_LO+80
"ac3a0090", --  sw $26,0x00000090($1)      
"3c011001", --  lui $1,0x00001001     243     sw $27, RESULT_MULT_HI+80
"ac3b00f4", --  sw $27,0x000000f4($1)      
"00880018", --  mult $4,$8            245     mult $4, $8		# mp * sp
"0000d012", --  mflo $26              246     mflo $26		# 0x7ffffff9
"0000d810", --  mfhi $27              247     mfhi $27		# 0x00000003
"3c011001", --  lui $1,0x00001001     248     sw $26, RESULT_MULT_LO+84
"ac3a0094", --  sw $26,0x00000094($1)      
"3c011001", --  lui $1,0x00001001     249     sw $27, RESULT_MULT_HI+84
"ac3b00f8", --  sw $27,0x000000f8($1)      
"00800018", --  mult $4,$0            251     mult $4, $0		# mp * r0
"0000d012", --  mflo $26              252     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              253     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     254     sw $26, RESULT_MULT_LO+88
"ac3a0098", --  sw $26,0x00000098($1)      
"3c011001", --  lui $1,0x00001001     255     sw $27, RESULT_MULT_HI+88
"ac3b00fc", --  sw $27,0x000000fc($1)      
"00870018", --  mult $4,$7            257     mult $4, $7		# mp * sn
"0000d012", --  mflo $26              258     mflo $26		# 0x80000005
"0000d810", --  mfhi $27              259     mfhi $27		# 0xfffffffd
"3c011001", --  lui $1,0x00001001     260     sw $26, RESULT_MULT_LO+92
"ac3a009c", --  sw $26,0x0000009c($1)      
"3c011001", --  lui $1,0x00001001     261     sw $27, RESULT_MULT_HI+92
"ac3b0100", --  sw $27,0x00000100($1)      
"00830018", --  mult $4,$3            263     mult $4, $3		# mp * mn
"0000d012", --  mflo $26              264     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              265     mfhi $27		# 0xc0000000
"3c011001", --  lui $1,0x00001001     266     sw $26, RESULT_MULT_LO+96
"ac3a00a0", --  sw $26,0x000000a0($1)      
"3c011001", --  lui $1,0x00001001     267     sw $27, RESULT_MULT_HI+96
"ac3b0104", --  sw $27,0x00000104($1)      
"0064001a", --  div $3,$4             271     div $3, $4		# mn / mp
"0000d012", --  mflo $26              272     mflo $26		# 0xffffffff
"0000d810", --  mfhi $27              273     mfhi $27		# 0xffffffff
"3c011001", --  lui $1,0x00001001     274     sw $26, RESULT_DIV_LO+0
"ac3a0108", --  sw $26,0x00000108($1)      
"3c011001", --  lui $1,0x00001001     275     sw $27, RESULT_DIV_HI+0
"ac3b0170", --  sw $27,0x00000170($1)      
"0068001a", --  div $3,$8             277     div $3, $8		# mn / sp
"0000d012", --  mflo $26              278     mflo $26		# 0xedb6db6e
"0000d810", --  mfhi $27              279     mfhi $27		# 0xfffffffe
"3c011001", --  lui $1,0x00001001     280     sw $26, RESULT_DIV_LO+4
"ac3a010c", --  sw $26,0x0000010c($1)      
"3c011001", --  lui $1,0x00001001     281     sw $27, RESULT_DIV_HI+4
"ac3b0174", --  sw $27,0x00000174($1)      
"0072001a", --  div $3,$18            283     div $3, $18		# mn / 1
"0000d012", --  mflo $26              284     mflo $26		# 0x80000000
"0000d810", --  mfhi $27              285     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     286     sw $26, RESULT_DIV_LO+8
"ac3a0110", --  sw $26,0x00000110($1)      
"3c011001", --  lui $1,0x00001001     287     sw $27, RESULT_DIV_HI+8
"ac3b0178", --  sw $27,0x00000178($1)      
"006b001a", --  div $3,$11            289     div $3, $11		# mn / sn
"0000d012", --  mflo $26              290     mflo $26		# 0x12492492
"0000d810", --  mfhi $27              291     mfhi $27		# 0xfffffffe
"3c011001", --  lui $1,0x00001001     292     sw $26, RESULT_DIV_LO+12
"ac3a0114", --  sw $26,0x00000114($1)      
"3c011001", --  lui $1,0x00001001     293     sw $27, RESULT_DIV_HI+12
"ac3b017c", --  sw $27,0x0000017c($1)      
"0063001a", --  div $3,$3             295     div $3, $3		# mn / mn
"0000d012", --  mflo $26              296     mflo $26		# 0x00000001
"0000d810", --  mfhi $27              297     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     298     sw $26, RESULT_DIV_LO+16
"ac3a0118", --  sw $26,0x00000118($1)      
"3c011001", --  lui $1,0x00001001     299     sw $27, RESULT_DIV_HI+16
"ac3b0180", --  sw $27,0x00000180($1)      
"0124001a", --  div $9,$4             302     div $9, $4		# sn / mp
"0000d012", --  mflo $26              303     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              304     mfhi $27		# 0xffffffce
"3c011001", --  lui $1,0x00001001     305     sw $26, RESULT_DIV_LO+20
"ac3a011c", --  sw $26,0x0000011c($1)      
"3c011001", --  lui $1,0x00001001     306     sw $27, RESULT_DIV_HI+20
"ac3b0184", --  sw $27,0x00000184($1)      
"0128001a", --  div $9,$8             308     div $9, $8		# sn / sp
"0000d012", --  mflo $26              309     mflo $26		# 0xfffffff9
"0000d810", --  mfhi $27              310     mfhi $27		# 0xffffffff
"3c011001", --  lui $1,0x00001001     311     sw $26, RESULT_DIV_LO+24
"ac3a0120", --  sw $26,0x00000120($1)      
"3c011001", --  lui $1,0x00001001     312     sw $27, RESULT_DIV_HI+24
"ac3b0188", --  sw $27,0x00000188($1)      
"0132001a", --  div $9,$18            314     div $9, $18		# sn / 1
"0000d012", --  mflo $26              315     mflo $26		# 0xffffffce
"0000d810", --  mfhi $27              316     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     317     sw $26, RESULT_DIV_LO+28
"ac3a0124", --  sw $26,0x00000124($1)      
"3c011001", --  lui $1,0x00001001     318     sw $27, RESULT_DIV_HI+28
"ac3b018c", --  sw $27,0x0000018c($1)      
"012b001a", --  div $9,$11            320     div $9, $11		# sn / sn
"0000d012", --  mflo $26              321     mflo $26		# 0x00000007
"0000d810", --  mfhi $27              322     mfhi $27		# 0xffffffff
"3c011001", --  lui $1,0x00001001     323     sw $26, RESULT_DIV_LO+32
"ac3a0128", --  sw $26,0x00000128($1)      
"3c011001", --  lui $1,0x00001001     324     sw $27, RESULT_DIV_HI+32
"ac3b0190", --  sw $27,0x00000190($1)      
"0123001a", --  div $9,$3             326     div $9, $3		# sn / mn
"0000d012", --  mflo $26              327     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              328     mfhi $27		# 0xffffffce
"3c011001", --  lui $1,0x00001001     329     sw $26, RESULT_DIV_LO+36
"ac3a012c", --  sw $26,0x0000012c($1)      
"3c011001", --  lui $1,0x00001001     330     sw $27, RESULT_DIV_HI+36
"ac3b0194", --  sw $27,0x00000194($1)      
"0004001a", --  div $0,$4             333     div $0, $4		# r0 / mp
"0000d012", --  mflo $26              334     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              335     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     336     sw $26, RESULT_DIV_LO+40
"ac3a0130", --  sw $26,0x00000130($1)      
"3c011001", --  lui $1,0x00001001     337     sw $27, RESULT_DIV_HI+40
"ac3b0198", --  sw $27,0x00000198($1)      
"0008001a", --  div $0,$8             339     div $0, $8		# r0 / sp
"0000d012", --  mflo $26              340     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              341     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     342     sw $26, RESULT_DIV_LO+44
"ac3a0134", --  sw $26,0x00000134($1)      
"3c011001", --  lui $1,0x00001001     343     sw $27, RESULT_DIV_HI+44
"ac3b019c", --  sw $27,0x0000019c($1)      
"0012001a", --  div $0,$18            345     div $0, $18		# r0 / 1
"0000d012", --  mflo $26              346     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              347     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     348     sw $26, RESULT_DIV_LO+48
"ac3a0138", --  sw $26,0x00000138($1)      
"3c011001", --  lui $1,0x00001001     349     sw $27, RESULT_DIV_HI+48
"ac3b01a0", --  sw $27,0x000001a0($1)      
"000b001a", --  div $0,$11            351     div $0, $11		# r0 / sn
"0000d012", --  mflo $26              352     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              353     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     354     sw $26, RESULT_DIV_LO+52
"ac3a013c", --  sw $26,0x0000013c($1)      
"3c011001", --  lui $1,0x00001001     355     sw $27, RESULT_DIV_HI+52
"ac3b01a4", --  sw $27,0x000001a4($1)      
"0003001a", --  div $0,$3             357     div $0, $3		# r0 / mn
"0000d012", --  mflo $26              358     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              359     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     360     sw $26, RESULT_DIV_LO+56
"ac3a0140", --  sw $26,0x00000140($1)      
"3c011001", --  lui $1,0x00001001     361     sw $27, RESULT_DIV_HI+56
"ac3b01a8", --  sw $27,0x000001a8($1)      
"0144001a", --  div $10,$4            364     div $10, $4		# sp / mp
"0000d012", --  mflo $26              365     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              366     mfhi $27		# 0x00000032
"3c011001", --  lui $1,0x00001001     367     sw $26, RESULT_DIV_LO+60
"ac3a0144", --  sw $26,0x00000144($1)      
"3c011001", --  lui $1,0x00001001     368     sw $27, RESULT_DIV_HI+60
"ac3b01ac", --  sw $27,0x000001ac($1)      
"0148001a", --  div $10,$8            370     div $10, $8		# sp / sp
"0000d012", --  mflo $26              371     mflo $26		# 0x00000007
"0000d810", --  mfhi $27              372     mfhi $27		# 0x00000001
"3c011001", --  lui $1,0x00001001     373     sw $26, RESULT_DIV_LO+64
"ac3a0148", --  sw $26,0x00000148($1)      
"3c011001", --  lui $1,0x00001001     374     sw $27, RESULT_DIV_HI+64
"ac3b01b0", --  sw $27,0x000001b0($1)      
"0152001a", --  div $10,$18           376     div $10, $18		# sp / 1
"0000d012", --  mflo $26              377     mflo $26		# 0x00000032
"0000d810", --  mfhi $27              378     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     379     sw $26, RESULT_DIV_LO+68
"ac3a014c", --  sw $26,0x0000014c($1)      
"3c011001", --  lui $1,0x00001001     380     sw $27, RESULT_DIV_HI+68
"ac3b01b4", --  sw $27,0x000001b4($1)      
"014b001a", --  div $10,$11           382     div $10, $11		# sp / sn
"0000d012", --  mflo $26              383     mflo $26		# 0xfffffff9
"0000d810", --  mfhi $27              384     mfhi $27		# 0x00000001
"3c011001", --  lui $1,0x00001001     385     sw $26, RESULT_DIV_LO+72
"ac3a0150", --  sw $26,0x00000150($1)      
"3c011001", --  lui $1,0x00001001     386     sw $27, RESULT_DIV_HI+72
"ac3b01b8", --  sw $27,0x000001b8($1)      
"0143001a", --  div $10,$3            388     div $10, $3		# sp / mn
"0000d012", --  mflo $26              389     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              390     mfhi $27		# 0x00000032
"3c011001", --  lui $1,0x00001001     391     sw $26, RESULT_DIV_LO+76
"ac3a0154", --  sw $26,0x00000154($1)      
"3c011001", --  lui $1,0x00001001     392     sw $27, RESULT_DIV_HI+76
"ac3b01bc", --  sw $27,0x000001bc($1)      
"0084001a", --  div $4,$4             395     div $4, $4		# mp / mp
"0000d012", --  mflo $26              396     mflo $26		# 0x00000001
"0000d810", --  mfhi $27              397     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     398     sw $26, RESULT_DIV_LO+80
"ac3a0158", --  sw $26,0x00000158($1)      
"3c011001", --  lui $1,0x00001001     399     sw $27, RESULT_DIV_HI+80
"ac3b01c0", --  sw $27,0x000001c0($1)      
"0088001a", --  div $4,$8             401     div $4, $8		# mp / sp
"0000d012", --  mflo $26              402     mflo $26		# 0x12492492
"0000d810", --  mfhi $27              403     mfhi $27		# 0x00000001
"3c011001", --  lui $1,0x00001001     404     sw $26, RESULT_DIV_LO+84
"ac3a015c", --  sw $26,0x0000015c($1)      
"3c011001", --  lui $1,0x00001001     405     sw $27, RESULT_DIV_HI+84
"ac3b01c4", --  sw $27,0x000001c4($1)      
"0092001a", --  div $4,$18            407     div $4, $18		# mp / 1
"0000d012", --  mflo $26              408     mflo $26		# 0x7fffffff
"0000d810", --  mfhi $27              409     mfhi $27		# 0x00000000
"3c011001", --  lui $1,0x00001001     410     sw $26, RESULT_DIV_LO+88
"ac3a0160", --  sw $26,0x00000160($1)      
"3c011001", --  lui $1,0x00001001     411     sw $27, RESULT_DIV_HI+8
"ac3b0178", --  sw $27,0x00000178($1)      
"008b001a", --  div $4,$11            413     div $4, $11		# mp / sn
"0000d012", --  mflo $26              414     mflo $26		# 0xedb6db6e
"0000d810", --  mfhi $27              415     mfhi $27		# 0x00000001
"3c011001", --  lui $1,0x00001001     416     sw $26, RESULT_DIV_LO+92
"ac3a0164", --  sw $26,0x00000164($1)      
"3c011001", --  lui $1,0x00001001     417     sw $27, RESULT_DIV_HI+92
"ac3b01cc", --  sw $27,0x000001cc($1)      
"0083001a", --  div $4,$3             419     div $4, $3		# mp / mn
"0000d012", --  mflo $26              420     mflo $26		# 0x00000000
"0000d810", --  mfhi $27              421     mfhi $27		# 0x7fffffff
"3c011001", --  lui $1,0x00001001     422     sw $26, RESULT_DIV_LO+96
"ac3a0168", --  sw $26,0x00000168($1)      
"3c011001", --  lui $1,0x00001001     423     sw $27, RESULT_DIV_HI+96
"ac3b01d0", --  sw $27,0x000001d0($1)      
"00c0001a", --  div $6,$0             426     div $6, $0		# div by 0
"0000d012", --  mflo $26              427     mflo $26		# same value as previous
"0000d810", --  mfhi $27              428     mfhi $27		# same value as previous
"3c011001", --  lui $1,0x00001001     429     sw $26, RESULT_DIV_LO+100
"ac3a016c", --  sw $26,0x0000016c($1)      
"3c011001", --  lui $1,0x00001001     430     sw $27, RESULT_DIV_HI+100
"ac3b01d4", --  sw $27,0x000001d4($1)      
"018cc824", --  and $25,$12,$12       434     and $25, $12, $12		# 1 & 1
"3c011001", --  lui $1,0x00001001     435     sw $25, RESULT_AND+0		# store result
"ac3901d8", --  sw $25,0x000001d8($1)      
"0186c824", --  and $25,$12,$6        437     and $25, $12, $6		# 1 & value
"3c011001", --  lui $1,0x00001001     438     sw $25, RESULT_AND+4		# store result
"ac3901dc", --  sw $25,0x000001dc($1)      
"0180c824", --  and $25,$12,$0        440     and $25, $12, $0		# 1 & 0
"3c011001", --  lui $1,0x00001001     441     sw $25, RESULT_AND+8		# store result
"ac3901e0", --  sw $25,0x000001e0($1)      
"000cc824", --  and $25,$0,$12        443     and $25, $0, $12		# 0 & 1
"3c011001", --  lui $1,0x00001001     444     sw $25, RESULT_AND+12	# store result
"ac3901e4", --  sw $25,0x000001e4($1)      
"00ccc824", --  and $25,$6,$12        446     and $25, $6, $12		# value & 1
"3c011001", --  lui $1,0x00001001     447     sw $25, RESULT_AND+16	# store result
"ac3901e8", --  sw $25,0x000001e8($1)      
"00c6c824", --  and $25,$6,$6         449     and $25, $6, $6		# value & value
"3c011001", --  lui $1,0x00001001     450     sw $25, RESULT_AND+20	# store result
"ac3901ec", --  sw $25,0x000001ec($1)      
"00c0c824", --  and $25,$6,$0         452     and $25, $6, $0		# value & 0
"3c011001", --  lui $1,0x00001001     453     sw $25, RESULT_AND+24	# store result
"ac3901f0", --  sw $25,0x000001f0($1)      
"00cfc824", --  and $25,$6,$15        455     and $25, $6, $15		# value & inverse value
"3c011001", --  lui $1,0x00001001     456     sw $25, RESULT_AND+28	# store result
"ac3901f4", --  sw $25,0x000001f4($1)      
"0186c825", --  or $25,$12,$6         460     or $25, $12, $6		# 1 | value
"3c011001", --  lui $1,0x00001001     461     sw $25, RESULT_OR+0		# store result
"ac3901f8", --  sw $25,0x000001f8($1)      
"0006c825", --  or $25,$0,$6          463     or $25, $0, $6		# 0 | value
"3c011001", --  lui $1,0x00001001     464     sw $25, RESULT_OR+4		# store result
"ac3901fc", --  sw $25,0x000001fc($1)      
"00ccc825", --  or $25,$6,$12         466     or $25, $6, $12		# value | 1
"3c011001", --  lui $1,0x00001001     467     sw $25, RESULT_OR+8		# store result
"ac390200", --  sw $25,0x00000200($1)      
"00c6c825", --  or $25,$6,$6          469     or $25, $6, $6		# value | value
"3c011001", --  lui $1,0x00001001     470     sw $25, RESULT_OR+12		# store result
"ac390204", --  sw $25,0x00000204($1)      
"00c0c825", --  or $25,$6,$0          472     or $25, $6, $0		# value | 0
"3c011001", --  lui $1,0x00001001     473     sw $25, RESULT_OR+16		# store result
"ac390208", --  sw $25,0x00000208($1)      
"00cfc825", --  or $25,$6,$15         475     or $25, $6, $15		# value | inverse value
"3c011001", --  lui $1,0x00001001     476     sw $25, RESULT_OR+20		# store result
"ac39020c", --  sw $25,0x0000020c($1)      
"35990065", --  ori $25,$12,0x00000065480     ori $25, $12, 101		# 1 | value
"3c011001", --  lui $1,0x00001001     481     sw $25, RESULT_ORI+0		# store result
"ac390210", --  sw $25,0x00000210($1)      
"34190065", --  ori $25,$0,0x00000065 483     ori $25, $0, 101		# 0 | value
"3c011001", --  lui $1,0x00001001     484     sw $25, RESULT_ORI+4		# store result
"ac390214", --  sw $25,0x00000214($1)      
"34d9ffff", --  ori $25,$6,0x0000ffff 486     ori $25, $6, 65535		# value | 1
"3c011001", --  lui $1,0x00001001     487     sw $25, RESULT_ORI+8		# store result
"ac390218", --  sw $25,0x00000218($1)      
"34d90065", --  ori $25,$6,0x00000065 489     ori $25, $6, 101		# value | value
"3c011001", --  lui $1,0x00001001     490     sw $25, RESULT_ORI+12	# store result
"ac39021c", --  sw $25,0x0000021c($1)      
"34d90000", --  ori $25,$6,0x00000000 492     ori $25, $6, 0		# value | 0
"3c011001", --  lui $1,0x00001001     493     sw $25, RESULT_ORI+16	# store result
"ac390220", --  sw $25,0x00000220($1)      
"34d9ff9a", --  ori $25,$6,0x0000ff9a 495     ori $25, $6, 65434		# value | inverse value
"3c011001", --  lui $1,0x00001001     496     sw $25, RESULT_ORI+20	# store result
"ac390224", --  sw $25,0x00000224($1)      
"0064c82a", --  slt $25,$3,$4         500     slt $25, $3, $4		# mn < mp
"3c011001", --  lui $1,0x00001001     501     sw $25, RESULT_SLT+0		# store result
"ac390228", --  sw $25,0x00000228($1)      
"0066c82a", --  slt $25,$3,$6         502     slt $25, $3, $6		# mn < sp
"3c011001", --  lui $1,0x00001001     503     sw $25, RESULT_SLT+4		# store result
"ac39022c", --  sw $25,0x0000022c($1)      
"0060c82a", --  slt $25,$3,$0         504     slt $25, $3, $0		# mn < r0
"3c011001", --  lui $1,0x00001001     505     sw $25, RESULT_SLT+8		# store result
"ac390230", --  sw $25,0x00000230($1)      
"0065c82a", --  slt $25,$3,$5         506     slt $25, $3, $5		# mn < sn
"3c011001", --  lui $1,0x00001001     507     sw $25, RESULT_SLT+12	# store result
"ac390234", --  sw $25,0x00000234($1)      
"0063c82a", --  slt $25,$3,$3         508     slt $25, $3, $3		# mn < mn
"3c011001", --  lui $1,0x00001001     509     sw $25, RESULT_SLT+16	# store result
"ac390238", --  sw $25,0x00000238($1)      
"00a4c82a", --  slt $25,$5,$4         511     slt $25, $5, $4		# sn < mp
"3c011001", --  lui $1,0x00001001     512     sw $25, RESULT_SLT+20	# store result
"ac39023c", --  sw $25,0x0000023c($1)      
"00a6c82a", --  slt $25,$5,$6         513     slt $25, $5, $6		# sn < sp
"3c011001", --  lui $1,0x00001001     514     sw $25, RESULT_SLT+24	# store result
"ac390240", --  sw $25,0x00000240($1)      
"00a0c82a", --  slt $25,$5,$0         515     slt $25, $5, $0		# sn < r0
"3c011001", --  lui $1,0x00001001     516     sw $25, RESULT_SLT+28	# store result
"ac390244", --  sw $25,0x00000244($1)      
"00a5c82a", --  slt $25,$5,$5         517     slt $25, $5, $5		# sn < sn
"3c011001", --  lui $1,0x00001001     518     sw $25, RESULT_SLT+32	# store result
"ac390248", --  sw $25,0x00000248($1)      
"00a3c82a", --  slt $25,$5,$3         519     slt $25, $5, $3		# sn < mn
"3c011001", --  lui $1,0x00001001     520     sw $25, RESULT_SLT+36	# store result
"ac39024c", --  sw $25,0x0000024c($1)      
"0004c82a", --  slt $25,$0,$4         522     slt $25, $0, $4		# r0 < mp
"3c011001", --  lui $1,0x00001001     523     sw $25, RESULT_SLT+40	# store result
"ac390250", --  sw $25,0x00000250($1)      
"0006c82a", --  slt $25,$0,$6         524     slt $25, $0, $6		# r0 < sp
"3c011001", --  lui $1,0x00001001     525     sw $25, RESULT_SLT+44	# store result
"ac390254", --  sw $25,0x00000254($1)      
"0000c82a", --  slt $25,$0,$0         526     slt $25, $0, $0		# r0 < r0
"3c011001", --  lui $1,0x00001001     527     sw $25, RESULT_SLT+48	# store result
"ac390258", --  sw $25,0x00000258($1)      
"0005c82a", --  slt $25,$0,$5         528     slt $25, $0, $5		# r0 < sn
"3c011001", --  lui $1,0x00001001     529     sw $25, RESULT_SLT+52	# store result
"ac39025c", --  sw $25,0x0000025c($1)      
"0003c82a", --  slt $25,$0,$3         530     slt $25, $0, $3		# r0 < mn
"3c011001", --  lui $1,0x00001001     531     sw $25, RESULT_SLT+56	# store result
"ac390260", --  sw $25,0x00000260($1)      
"00c4c82a", --  slt $25,$6,$4         533     slt $25, $6, $4		# sp < mp
"3c011001", --  lui $1,0x00001001     534     sw $25, RESULT_SLT+60	# store result
"ac390264", --  sw $25,0x00000264($1)      
"00c6c82a", --  slt $25,$6,$6         535     slt $25, $6, $6		# sp < sp
"3c011001", --  lui $1,0x00001001     536     sw $25, RESULT_SLT+64	# store result
"ac390268", --  sw $25,0x00000268($1)      
"00c0c82a", --  slt $25,$6,$0         537     slt $25, $6, $0		# sp < r0
"3c011001", --  lui $1,0x00001001     538     sw $25, RESULT_SLT+68	# store result
"ac39026c", --  sw $25,0x0000026c($1)      
"00c5c82a", --  slt $25,$6,$5         539     slt $25, $6, $5		# sp < sn
"3c011001", --  lui $1,0x00001001     540     sw $25, RESULT_SLT+72	# store result
"ac390270", --  sw $25,0x00000270($1)      
"00c3c82a", --  slt $25,$6,$3         541     slt $25, $6, $3		# sp < mn
"3c011001", --  lui $1,0x00001001     542     sw $25, RESULT_SLT+76	# store result
"ac390274", --  sw $25,0x00000274($1)      
"0084c82a", --  slt $25,$4,$4         544     slt $25, $4, $4		# mp < mp
"3c011001", --  lui $1,0x00001001     545     sw $25, RESULT_SLT+80	# store result
"ac390278", --  sw $25,0x00000278($1)      
"0086c82a", --  slt $25,$4,$6         546     slt $25, $4, $6		# mp < sp
"3c011001", --  lui $1,0x00001001     547     sw $25, RESULT_SLT+84	# store result
"ac39027c", --  sw $25,0x0000027c($1)      
"0080c82a", --  slt $25,$4,$0         548     slt $25, $4, $0		# mp < r0
"3c011001", --  lui $1,0x00001001     549     sw $25, RESULT_SLT+88	# store result
"ac390280", --  sw $25,0x00000280($1)      
"0085c82a", --  slt $25,$4,$5         550     slt $25, $4, $5		# mp < sn
"3c011001", --  lui $1,0x00001001     551     sw $25, RESULT_SLT+92	# store result
"ac390284", --  sw $25,0x00000284($1)      
"0083c82a", --  slt $25,$4,$3         552     slt $25, $4, $3		# mp < mn
"3c011001", --  lui $1,0x00001001     553     sw $25, RESULT_SLT+96	# store result
"ac390288", --  sw $25,0x00000288($1)      
"0064c820", --  add $25,$3,$4         557     add $25, $3, $4		# mn + mp
"3c011001", --  lui $1,0x00001001     558     sw $25, RESULT_ADD+0		# store result
"ac39028c", --  sw $25,0x0000028c($1)      
"0066c820", --  add $25,$3,$6         559     add $25, $3, $6		# mn + sp
"3c011001", --  lui $1,0x00001001     560     sw $25, RESULT_ADD+4		# store result
"ac390290", --  sw $25,0x00000290($1)      
"0060c820", --  add $25,$3,$0         561     add $25, $3, $0		# mn + r0
"3c011001", --  lui $1,0x00001001     562     sw $25, RESULT_ADD+8		# store result
"ac390294", --  sw $25,0x00000294($1)      
"00a4c820", --  add $25,$5,$4         564     add $25, $5, $4		# sn + mp
"3c011001", --  lui $1,0x00001001     565     sw $25, RESULT_ADD+12	# store result
"ac390298", --  sw $25,0x00000298($1)      
"00a6c820", --  add $25,$5,$6         566     add $25, $5, $6		# sn + sp
"3c011001", --  lui $1,0x00001001     567     sw $25, RESULT_ADD+16	# store result
"ac39029c", --  sw $25,0x0000029c($1)      
"00a0c820", --  add $25,$5,$0         568     add $25, $5, $0		# sn + r0
"3c011001", --  lui $1,0x00001001     569     sw $25, RESULT_ADD+20	# store result
"ac3902a0", --  sw $25,0x000002a0($1)      
"00a5c820", --  add $25,$5,$5         570     add $25, $5, $5		# sn + sn
"3c011001", --  lui $1,0x00001001     571     sw $25, RESULT_ADD+24	# store result
"ac3902a4", --  sw $25,0x000002a4($1)      
"0004c820", --  add $25,$0,$4         573     add $25, $0, $4		# r0 + mp
"3c011001", --  lui $1,0x00001001     574     sw $25, RESULT_ADD+28	# store result
"ac3902a8", --  sw $25,0x000002a8($1)      
"0006c820", --  add $25,$0,$6         575     add $25, $0, $6		# r0 + sp
"3c011001", --  lui $1,0x00001001     576     sw $25, RESULT_ADD+32	# store result
"ac3902ac", --  sw $25,0x000002ac($1)      
"0000c820", --  add $25,$0,$0         577     add $25, $0, $0		# r0 + r0
"3c011001", --  lui $1,0x00001001     578     sw $25, RESULT_ADD+36	# store result
"ac3902b0", --  sw $25,0x000002b0($1)      
"0005c820", --  add $25,$0,$5         579     add $25, $0, $5		# r0 + sn
"3c011001", --  lui $1,0x00001001     580     sw $25, RESULT_ADD+40	# store result
"ac3902b4", --  sw $25,0x000002b4($1)      
"0003c820", --  add $25,$0,$3         581     add $25, $0, $3		# r0 + mn
"3c011001", --  lui $1,0x00001001     582     sw $25, RESULT_ADD+44	# store result
"ac3902b8", --  sw $25,0x000002b8($1)      
"00c6c820", --  add $25,$6,$6         584     add $25, $6, $6		# sp + sp
"3c011001", --  lui $1,0x00001001     585     sw $25, RESULT_ADD+48	# store result
"ac3902bc", --  sw $25,0x000002bc($1)      
"00c0c820", --  add $25,$6,$0         586     add $25, $6, $0		# sp + r0
"3c011001", --  lui $1,0x00001001     587     sw $25, RESULT_ADD+52	# store result
"ac3902c0", --  sw $25,0x000002c0($1)      
"00c5c820", --  add $25,$6,$5         588     add $25, $6, $5		# sp + sn
"3c011001", --  lui $1,0x00001001     589     sw $25, RESULT_ADD+56	# store result
"ac3902c4", --  sw $25,0x000002c4($1)      
"00c3c820", --  add $25,$6,$3         590     add $25, $6, $3		# sp + mn
"3c011001", --  lui $1,0x00001001     591     sw $25, RESULT_ADD+60	# store result
"ac3902c8", --  sw $25,0x000002c8($1)      
"0080c820", --  add $25,$4,$0         593     add $25, $4, $0		# mp + r0
"3c011001", --  lui $1,0x00001001     594     sw $25, RESULT_ADD+64	# store result
"ac3902cc", --  sw $25,0x000002cc($1)      
"0085c820", --  add $25,$4,$5         595     add $25, $4, $5		# mp + sn
"3c011001", --  lui $1,0x00001001     596     sw $25, RESULT_ADD+68	# store result
"ac3902d0", --  sw $25,0x000002d0($1)      
"0083c820", --  add $25,$4,$3         597     add $25, $4, $3		# mp + mn
"3c011001", --  lui $1,0x00001001     598     sw $25, RESULT_ADD+72	# store result
"ac3902d4", --  sw $25,0x000002d4($1)      
"20797fff", --  addi $25,$3,0x00007fff602     addi $25, $3, 32767		# mn + mp
"3c011001", --  lui $1,0x00001001     603     sw $25, RESULT_ADDI+0	# store result
"ac3902d8", --  sw $25,0x000002d8($1)      
"20790065", --  addi $25,$3,0x00000065604     addi $25, $3, 101		# mn + sp
"3c011001", --  lui $1,0x00001001     605     sw $25, RESULT_ADDI+4	# store result
"ac3902dc", --  sw $25,0x000002dc($1)      
"20790000", --  addi $25,$3,0x00000000606     addi $25, $3, 0		# mn + r0
"3c011001", --  lui $1,0x00001001     607     sw $25, RESULT_ADDI+8	# store result
"ac3902e0", --  sw $25,0x000002e0($1)      
"20b97fff", --  addi $25,$5,0x00007fff609     addi $25, $5, 32767		# sn + mp
"3c011001", --  lui $1,0x00001001     610     sw $25, RESULT_ADDI+12	# store result
"ac3902e4", --  sw $25,0x000002e4($1)      
"20b90065", --  addi $25,$5,0x00000065611     addi $25, $5, 101		# sn + sp
"3c011001", --  lui $1,0x00001001     612     sw $25, RESULT_ADDI+16	# store result
"ac3902e8", --  sw $25,0x000002e8($1)      
"20b90000", --  addi $25,$5,0x00000000613     addi $25, $5, 0		# sn + r0
"3c011001", --  lui $1,0x00001001     614     sw $25, RESULT_ADDI+20	# store result
"ac3902ec", --  sw $25,0x000002ec($1)      
"20b9ff9b", --  addi $25,$5,0xffffff9b615     addi $25, $5, -101		# sn + sn
"3c011001", --  lui $1,0x00001001     616     sw $25, RESULT_ADDI+24	# store result
"ac3902f0", --  sw $25,0x000002f0($1)      
"20b98000", --  addi $25,$5,0xffff8000617     addi $25, $5, -32768		# sn + mn
"3c011001", --  lui $1,0x00001001     618     sw $25, RESULT_ADDI+28	# store result
"ac3902f4", --  sw $25,0x000002f4($1)      
"20197fff", --  addi $25,$0,0x00007fff620     addi $25, $0, 32767		# r0 + mp
"3c011001", --  lui $1,0x00001001     621     sw $25, RESULT_ADDI+32	# store result
"ac3902f8", --  sw $25,0x000002f8($1)      
"20190065", --  addi $25,$0,0x00000065622     addi $25, $0, 101		# r0 + sp
"3c011001", --  lui $1,0x00001001     623     sw $25, RESULT_ADDI+36	# store result
"ac3902fc", --  sw $25,0x000002fc($1)      
"20190000", --  addi $25,$0,0x00000000624     addi $25, $0, 0		# r0 + r0
"3c011001", --  lui $1,0x00001001     625     sw $25, RESULT_ADDI+40	# store result
"ac390300", --  sw $25,0x00000300($1)      
"2019ff9b", --  addi $25,$0,0xffffff9b626     addi $25, $0, -101		# r0 + sn
"3c011001", --  lui $1,0x00001001     627     sw $25, RESULT_ADDI+44	# store result
"ac390304", --  sw $25,0x00000304($1)      
"20198000", --  addi $25,$0,0xffff8000628     addi $25, $0, -32768		# r0 + mn
"3c011001", --  lui $1,0x00001001     629     sw $25, RESULT_ADDI+48	# store result
"ac390308", --  sw $25,0x00000308($1)      
"20d97fff", --  addi $25,$6,0x00007fff631     addi $25, $6, 32767		# sp + mp
"3c011001", --  lui $1,0x00001001     632     sw $25, RESULT_ADDI+52	# store result
"ac39030c", --  sw $25,0x0000030c($1)      
"20d90065", --  addi $25,$6,0x00000065633     addi $25, $6, 101		# sp + sp
"3c011001", --  lui $1,0x00001001     634     sw $25, RESULT_ADDI+56	# store result
"ac390310", --  sw $25,0x00000310($1)      
"20d90000", --  addi $25,$6,0x00000000635     addi $25, $6, 0		# sp + r0
"3c011001", --  lui $1,0x00001001     636     sw $25, RESULT_ADDI+60	# store result
"ac390314", --  sw $25,0x00000314($1)      
"20d9ff9b", --  addi $25,$6,0xffffff9b637     addi $25, $6, -101		# sp + sn
"3c011001", --  lui $1,0x00001001     638     sw $25, RESULT_ADDI+64	# store result
"ac390318", --  sw $25,0x00000318($1)      
"20d98000", --  addi $25,$6,0xffff8000639     addi $25, $6, -32768		# sp + mn
"3c011001", --  lui $1,0x00001001     640     sw $25, RESULT_ADDI+68	# store result
"ac39031c", --  sw $25,0x0000031c($1)      
"20990000", --  addi $25,$4,0x00000000642     addi $25, $4, 0		# mp + r0
"3c011001", --  lui $1,0x00001001     643     sw $25, RESULT_ADDI+72	# store result
"ac390320", --  sw $25,0x00000320($1)      
"2099ff9b", --  addi $25,$4,0xffffff9b644     addi $25, $4, -101		# mp + sn
"3c011001", --  lui $1,0x00001001     645     sw $25, RESULT_ADDI+76	# store result
"ac390324", --  sw $25,0x00000324($1)      
"20998000", --  addi $25,$4,0xffff8000646     addi $25, $4, -32768		# mp + mn
"3c011001", --  lui $1,0x00001001     647     sw $25, RESULT_ADDI+80	# store result
"ac390328", --  sw $25,0x00000328($1)      
"0060c822", --  sub $25,$3,$0         651     sub $25, $3, $0		# mn - r0
"3c011001", --  lui $1,0x00001001     652     sw $25, RESULT_SUB+0		# store result
"ac39032c", --  sw $25,0x0000032c($1)      
"0065c822", --  sub $25,$3,$5         653     sub $25, $3, $5		# mn - sn
"3c011001", --  lui $1,0x00001001     654     sw $25, RESULT_SUB+4		# store result
"ac390330", --  sw $25,0x00000330($1)      
"0063c822", --  sub $25,$3,$3         655     sub $25, $3, $3		# mn - mn
"3c011001", --  lui $1,0x00001001     656     sw $25, RESULT_SUB+8		# store result
"ac390334", --  sw $25,0x00000334($1)      
"00a6c822", --  sub $25,$5,$6         658     sub $25, $5, $6		# sn - sp
"3c011001", --  lui $1,0x00001001     659     sw $25, RESULT_SUB+12	# store result
"ac390338", --  sw $25,0x00000338($1)      
"00a0c822", --  sub $25,$5,$0         660     sub $25, $5, $0		# sn - r0
"3c011001", --  lui $1,0x00001001     661     sw $25, RESULT_SUB+16	# store result
"ac39033c", --  sw $25,0x0000033c($1)      
"00a5c822", --  sub $25,$5,$5         662     sub $25, $5, $5		# sn - sn
"3c011001", --  lui $1,0x00001001     663     sw $25, RESULT_SUB+20	# store result
"ac390340", --  sw $25,0x00000340($1)      
"00a3c822", --  sub $25,$5,$3         664     sub $25, $5, $3		# sn - mn
"3c011001", --  lui $1,0x00001001     665     sw $25, RESULT_SUB+24	# store result
"ac390344", --  sw $25,0x00000344($1)      
"0004c822", --  sub $25,$0,$4         667     sub $25, $0, $4		# r0 - mp
"3c011001", --  lui $1,0x00001001     668     sw $25, RESULT_SUB+28	# store result
"ac390348", --  sw $25,0x00000348($1)      
"0006c822", --  sub $25,$0,$6         669     sub $25, $0, $6		# r0 - sp
"3c011001", --  lui $1,0x00001001     670     sw $25, RESULT_SUB+32	# store result
"ac39034c", --  sw $25,0x0000034c($1)      
"0000c822", --  sub $25,$0,$0         671     sub $25, $0, $0		# r0 - r0
"3c011001", --  lui $1,0x00001001     672     sw $25, RESULT_SUB+36	# store result
"ac390350", --  sw $25,0x00000350($1)      
"0005c822", --  sub $25,$0,$5         673     sub $25, $0, $5		# r0 - sn
"3c011001", --  lui $1,0x00001001     674     sw $25, RESULT_SUB+40	# store result
"ac390354", --  sw $25,0x00000354($1)      
"00c4c822", --  sub $25,$6,$4         676     sub $25, $6, $4		# sp - mp
"3c011001", --  lui $1,0x00001001     677     sw $25, RESULT_SUB+44	# store result
"ac390358", --  sw $25,0x00000358($1)      
"00c6c822", --  sub $25,$6,$6         678     sub $25, $6, $6		# sp - sp
"3c011001", --  lui $1,0x00001001     679     sw $25, RESULT_SUB+48	# store result
"ac39035c", --  sw $25,0x0000035c($1)      
"00c0c822", --  sub $25,$6,$0         680     sub $25, $6, $0		# sp - r0
"3c011001", --  lui $1,0x00001001     681     sw $25, RESULT_SUB+52	# store result
"ac390360", --  sw $25,0x00000360($1)      
"00c5c822", --  sub $25,$6,$5         682     sub $25, $6, $5		# sp - sn
"3c011001", --  lui $1,0x00001001     683     sw $25, RESULT_SUB+56	# store result
"ac390364", --  sw $25,0x00000364($1)      
"0084c822", --  sub $25,$4,$4         685     sub $25, $4, $4		# mp - mp
"3c011001", --  lui $1,0x00001001     686     sw $25, RESULT_SUB+60	# store result
"ac390368", --  sw $25,0x00000368($1)      
"0086c822", --  sub $25,$4,$6         687     sub $25, $4, $6		# mp - sp
"3c011001", --  lui $1,0x00001001     688     sw $25, RESULT_SUB+64	# store result
"ac39036c", --  sw $25,0x0000036c($1)      
"0080c822", --  sub $25,$4,$0         689     sub $25, $4, $0		# mp - r0
"3c011001", --  lui $1,0x00001001     690     sw $25, RESULT_SUB+68	# store result
"ac390370", --  sw $25,0x00000370($1)      
"ae67fff8", --  sw $7,0xfffffff8($19) 694     sw $7 -8($19)
"ae68fffc", --  sw $8,0xfffffffc($19) 695     sw $8 -4($19)
"ae690000", --  sw $9,0x00000000($19) 696     sw $9 0($19)
"ae6a0004", --  sw $10,0x00000004($19)697     sw $10 4($19)
"3c011001", --  lui $1,0x00001001     698     sw $11 ADDR_IMM
"ac2b0030", --  sw $11,0x00000030($1)      
"3c011001", --  lui $1,0x00001001     699     sw $12 SMALL_POS
"ac2c000c", --  sw $12,0x0000000c($1)      
"ae600000", --  sw $0,0x00000000($19) 700     sw $0  ($19) # store RF[0] to an address
"00000000", --  nop                   703     nop # halt
OTHERS => "00000000" 
      );
  
      VARIABLE data:data_segment:=
           ("80000000", "7fffffff", "ffffff9b", "00000065", "fffffffb", "00000007", "ffffffce",
            "00000032", "fffffff9", "ffffffff", "00000001", "00000004", "10010020", "ffffff9a",  OTHERS=>"00000000");
  
      VARIABLE address:natural;  
      VARIABLE data_out:std_logic_vector(31 DOWNTO 0);
    
   BEGIN
      WAIT UNTIL rising_edge(clk);
      address:=to_integer(unsigned(word_address));
      -- check text segments
      IF (address >= text_base_address/4) AND (address <=text_base_address/4 + text_base_size) THEN  
         d_busouti <= unknown;    
         IF write='1' THEN
            prg(address):=binvec2hex(d_busin);
         ELSIF read='1' THEN
            d_busouti <= hexvec2bin(prg(address));
         END IF;
      ELSIF (address >= data_base_address/4) AND (address <=data_base_address/4 + data_base_size) THEN
         d_busouti <= unknown;
         IF write='1' THEN
            data(address):=binvec2hex(d_busin);
         ELSIF read='1' THEN
            d_busouti <= hexvec2bin(data(address));
         END IF;    
      ELSIF read='1' OR write='1' THEN  -- address not in text/data segment; read/write not valid.
         REPORT "out of memory range" SEVERITY warning;
         d_busouti <= unknown;
      END IF;
   END PROCESS;
  
   d_busout <= d_busouti WHEN state=rd_wr_rdy ELSE unknown;

   -- code below is used to model handshake; variable 'dly' can also be another value than 1 (in state idle) 
   handshake_protocol:PROCESS
      VARIABLE dly : natural; -- nmb of delays models delay 
   BEGIN
      WAIT UNTIL clk='1';
      CASE state IS
         WHEN idle        => IF read='1' OR write='1' THEN state<=rd_wr_nrdy; END IF; dly:=1;
         WHEN rd_wr_nrdy  => IF dly>0 THEN dly:=dly-1; ELSE state<=rd_wr_rdy; END IF;
         WHEN rd_wr_rdy   => IF read='0' AND write='0' THEN state<=idle; END IF;
      END CASE;
   END PROCESS;

   ready <= '1' WHEN state=rd_wr_rdy ELSE '0';
  
   ASSERT NOT (read='1' AND write='1') REPORT "memory: read and write are active" SEVERITY error;
  
   ASSERT (a_bus(1 DOWNTO 0)="00") OR (state=idle) REPORT "memory: not an aligned address" SEVERITY error;   
  
END behaviour;