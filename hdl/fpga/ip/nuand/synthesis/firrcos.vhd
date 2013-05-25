-- Root Raised Cosine with rolloff = 0.5, 2x interpolating filter
-- Fixed point 1.0 = 16384
-- Taps created using:
-- >> format long ; h = firrcos( 48, 0.25/2, 0.5, 1.0, 'rolloff', 'sqrt' ) ; round(h'.*16384)

library ieee ;
    use ieee.std_logic_1164.all ;
    use ieee.numeric_std.all ;

entity firrcos is
  port (
    clock       :   in  std_logic ;
    reset       :   in  std_logic ;
    in_i        :   in  signed(15 downto 0) ;
    in_q        :   in  signed(15 downto 0) ;
    in_valid    :   in  std_logic ;

    out_i       :   out signed(15 downto 0) ;
    out_q       :   out signed(15 downto 0) ;
    out_valid   :   out std_logic
  ) ;
end entity ;

architecture systolic of firrcos is

    type integer_array_t is array(natural range <>) of integer ;

    constant TAPS : integer_array_t := (
          18,
           7,
         -17,
         -24,
          -3,
          24,
          20,
         -16,
         -41,
         -16,
          44,
          67,
          12,
         -67,
         -61,
          63,
         174,
          63,
        -307,
        -642,
        -435,
         642,
        2370,
        3992,
        4656,
        3992,
        2370,
         642,
        -435,
        -642,
        -307,
          63,
         174,
          63,
         -61,
         -67,
          12,
          67,
          44,
         -16,
         -41,
         -16,
          20,
          24,
          -3,
         -24,
         -17,
           7,
          18
    ) ;

    type state_t is array(TAPS'range) of signed(2*in_i'length-1 downto 0) ;

    signal state_i : state_t ;
    signal state_q : state_t ;

    signal oot_valid : std_logic ;

begin

    convolve : process( clock, reset )
    begin
        if( reset = '1' ) then
            state_i <= (others =>(others =>'0')) ;
            state_q <= (others =>(others =>'0')) ;
            oot_valid <= '0' ;
        elsif( rising_edge( clock ) ) then
            oot_valid <= in_valid ;
            if( in_valid = '1' ) then
                state_i(state_i'low) <= in_i * to_signed(TAPS(TAPS'low),in_i'length) ;
                state_q(state_q'low) <= in_q * to_signed(TAPS(TAPS'low),in_q'length) ;
                for i in TAPS'low + 1 to TAPS'high loop
                    state_i(i) <= state_i(i-1) + in_i * to_signed(TAPS(i),in_i'length) ;
                    state_q(i) <= state_q(i-1) + in_q * to_signed(TAPS(i),in_q'length) ;
                end loop ;
            end if ;
        end if ;
    end process ;

    oot : process( clock, reset )
    begin
        if( reset = '1' ) then
            out_i <= (others =>'0') ;
            out_q <= (others =>'0') ;
            out_valid <= '0' ;
        elsif( rising_edge( clock ) ) then
            out_valid <= oot_valid ;
            if( oot_valid = '1' ) then
                out_i <= resize(shift_right(state_i(state_i'high) + 8192, 14),out_i'length) ;
                out_q <= resize(shift_right(state_q(state_q'high) + 8192, 14),out_q'length) ;
            end if ;
        end if ;
    end process ;

end architecture ;