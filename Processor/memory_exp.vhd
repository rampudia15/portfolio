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

ARCHITECTURE exp_program OF memory IS
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
-- Code         Basic                      Source
"04010013", --  bgez $0,19            21   b	ser # branch always to ser; is assembled to: bgez $0, ser
"34070001", --  ori $7,$0,1           26   xn:	ori $7, $0, 1    # res
"0005082a", --  slt $1,$0,$5          27   nxt:	ble $5, $0, fexp # if [R5]<=[$0] branch; assembled to slt $1, $0, %5 and beq $1, $0, fexp
"10200005", --  beq $1,$0,5                
"00e60018", --  mult $7,$6            28   	mult $7, $6      # LO=res * x
"00003812", --  mflo $7               29   	mflo $7          # res=res * x
"200f0001", --  addi $15,$0,1         30   	addi $15, $0, 1
"00af2822", --  sub $5,$5,$15         31   	sub $5, $5, $15  # n=n-1
"0401fff9", --  bgez $0,-7            32   	b nxt
"04010015", --  bgez $0,21            33   fexp:	b fxn
"34090001", --  ori $9,$0,1           38   f:	ori $9, $0, 1    # res
"200f0001", --  addi $15,$0,1         39   nfac:	addi $15, $0, 1
"0008082a", --  slt $1,$0,$8          40   	ble $8, $0, ffac # branch if [$8]<=[$0]; assembled to slt and beq
"10200005", --  beq $1,$0,5                
"01280018", --  mult $9,$8            41   	mult $9, $8      # res=res*n
"00004812", --  mflo $9               42   	mflo $9
"200f0001", --  addi $15,$0,1         43   	addi $15, $0, 1
"010f4022", --  sub $8,$8,$15         44   	sub $8, $8, $15  # n=n-1	
"0401fff8", --  bgez $0,-8            45   	b nfac           
"04010010", --  bgez $0,16            46   ffac:	b ff
"3c011001", --  lui $1,4097           50     lw $10, N              # number of terms; assembled in LUI and LW (this depends on location of N)
"8c2a0000", --  lw $10,0($1)               
"3c011001", --  lui $1,4097           51     lw $13, X              # value x
"8c2d0004", --  lw $13,4($1)               
"340b0001", --  ori $11,$0,1          52   	ori $11, $0, 1    # index 
"340c03e8", --  ori $12,$0,1000       53   	ori $12, $0, 1000 # approximation exp (first term always 1) (multiplied with 1000)
"016a082a", --  slt $1,$11,$10        57   ntrm:	ble $10, $11, rdy	
"1020000d", --  beq $1,$0,13               
"000b2825", --  or $5,$0,$11          58   	or $5, $0, $11    # calculate x^n
"000d3025", --  or $6,$0,$13          59   	or $6, $0, $13
"0401ffe2", --  bgez $0,-30           60   	b xn    # branch always to xn; calculate x^n
"200f03e8", --  addi $15,$0,1000      62   	addi $15, $0, 1000
"00ef0018", --  mult $7,$15           63   	mult $7, $15      # LO=multiply with 1000
"00003812", --  mflo $7               64   	mflo $7
"000b4025", --  or $8,$0,$11          65   	or $8, $0, $11 
"0401ffe6", --  bgez $0,-26           66   	b f               # branch always to f; calculate n!
"00e9001a", --  div $7,$9             68   	div  $7, $9       # 100"2^n/n! in $14
"00007012", --  mflo $14              69   	mflo $14
"018e6020", --  add $12,$12,$14       70   	add $12, $12, $14 # sn=s(n-1)+term
"216b0001", --  addi $11,$11,1        71   	add $11, $11, 1   # i++
"0401fff1", --  bgez $0,-15           72   	b ntrm  # branch always to ntrm
"3c011001", --  lui $1,4097           73   rdy:	sw $12,EX # e^x in EX
"ac2c0008", --  sw $12,8($1)               
"00000000", --  nop                   74   	nop           
OTHERS => "00000000" 
      );
  
      VARIABLE data:data_segment:=
           ("00000007", "ffffffff",  OTHERS=>"00000000");
  
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
  
END exp_program;
