library ieee ;
    use ieee.std_logic_1164.all ;
    use ieee.numeric_std.all ;
    use ieee.math_real.all ;

entity firrcos_tb is
end entity ; -- firrcos_tb

architecture arch of firrcos_Tb is

    signal clock        :   std_logic               := '1' ;
    signal reset        :   std_logic               := '1' ;

    signal in_i         :   signed(15 downto 0)     := (others =>'0') ;
    signal in_q         :   signed(15 downto 0)     := (others =>'0') ;
    signal in_valid     :   std_logic               := '0' ;

    signal out_i        :   signed(15 downto 0) ;
    signal out_q        :   signed(15 downto 0) ;
    signal out_valid    :   std_logic ;

    signal tx_data      :   signed(15 downto 0) ;
    signal tx_iq_sel    :   std_logic ;

    procedure nop( signal clock : in std_logic ; count : integer ) is
    begin
        for i in 1 to count loop
            wait until rising_edge( clock ) ;
        end loop ;
    end procedure ;

    type integer_array_t is array(natural range <>) of integer ;

    constant QPSK_I : integer_array_t := ( 4096, 0, 0, -4096 ) ;
    constant QPSK_Q : integer_array_t := ( 0, 4096, -4096, 0 ) ;

begin

    clock <= not clock after 1 ns ;

    U_firrcos : entity work.firrcos(systolic)
      port map (
        clock       =>  clock,
        reset       =>  reset,

        in_i        =>  in_i,
        in_q        =>  in_q,
        in_valid    =>  in_valid,

        out_i       =>  out_i,
        out_q       =>  out_q,
        out_valid   =>  out_valid
      ) ;

    tb : process
        variable seed1 : integer := 1234 ;
        variable seed2 : integer := 5678 ;
        variable rand : real ;
    begin
        reset <= '1' ;
        nop( clock, 10 ) ;

        reset <= '0' ;
        nop( clock, 10 ) ;

        for i in 0 to 1000 loop
            if( i mod 4 = 0 ) then
                uniform( seed1, seed2, rand ) ;
                rand := floor(rand * 3.999999) ;
                in_i <= to_signed(QPSK_I(integer(rand)), in_i'length ) ;
                in_q <= to_signed(QPSK_Q(integer(rand)), in_q'length ) ;
            else
                in_i <= (others =>'0') ;
                in_q <= (others =>'0') ;
            end if ;
            in_valid <= '1' ;
            wait until rising_edge(clock) ;
            in_valid <= '0' ;
            wait until rising_edge(clock) ;
        end loop ;

        report "-- End of Simulation --" severity failure ;
    end process ;

    serialize : process( clock, reset )
    begin
        if( reset = '1' ) then
            tx_data <= (others =>'0') ;
            tx_iq_sel <= '1' ;
        elsif( rising_edge(clock) ) then
            tx_iq_sel <= out_valid ;
            if( out_valid = '1' ) then
                tx_data <= out_i ;
            else
                tx_data <= out_q ;
            end if ;
        end if ;
    end process ;

end architecture ; -- arch