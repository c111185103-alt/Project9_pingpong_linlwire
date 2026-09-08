library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ball_counter is
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        en       : in  std_logic;
        set_pos  : in  std_logic;
        set_val  : in  unsigned(2 downto 0);
        pos      : out unsigned(2 downto 0);
        dir      : out std_logic;
        at_left  : out std_logic;
        at_right : out std_logic
    );
end ball_counter;

architecture Behavioral of ball_counter is
    signal pos_r : unsigned(2 downto 0) := (others => '0');
    signal dir_r : std_logic := '1';
begin
    process(clk, rst)
    begin
        if rst = '1' then
            pos_r <= (others => '0');
            dir_r <= '1';
        elsif rising_edge(clk) then
            if set_pos = '1' then
                pos_r <= set_val;
                if set_val = 0 then
                    dir_r <= '1';
                elsif set_val = 7 then
                    dir_r <= '0';
                end if;
            elsif en = '1' then
                if dir_r = '1' then
                    if pos_r = 6 then
                        pos_r <= to_unsigned(7, 3);
                        dir_r <= '0';
                    else
                        pos_r <= pos_r + 1;
                    end if;
                else
                    if pos_r = 1 then
                        pos_r <= to_unsigned(0, 3);
                        dir_r <= '1';
                    else
                        pos_r <= pos_r - 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    pos      <= pos_r;
    dir      <= dir_r;
    at_left  <= '1' when pos_r = 0 else '0';
    at_right <= '1' when pos_r = 7 else '0';
end Behavioral;
