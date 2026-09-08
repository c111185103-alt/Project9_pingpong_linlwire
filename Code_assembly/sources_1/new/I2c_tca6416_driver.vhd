library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;



entity i2c_tca6416_driver is
    generic (
        CLK_FREQ_HZ : integer := 100_000_000;
        I2C_FREQ_HZ : integer := 100_000;
        I2C_ADDR    : std_logic_vector(6 downto 0) := "0100000"
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        seg       : in  std_logic_vector(6 downto 0);
        digit_sel : in  std_logic;
        scl       : out std_logic;
        sda       : inout std_logic
    );
end i2c_tca6416_driver;

architecture Behavioral of i2c_tca6416_driver is

    constant HALF_CYCLES : integer := CLK_FREQ_HZ / I2C_FREQ_HZ / 2;

    signal half_timer : integer range 0 to HALF_CYCLES - 1 := 0;
    signal half_tick  : std_logic := '0';

    type bstate_t is (S_IDLE, S_START, S_BIT_LOW, S_BIT_HIGH,
                       S_ACK_LOW, S_ACK_HIGH, S_STOP1, S_STOP2, S_GAP);
    signal bstate : bstate_t := S_IDLE;

    signal bit_cnt  : integer range 0 to 7 := 0;
    signal byte_idx : integer range 0 to 2 := 0;

    signal txn_sel  : integer range 0 to 4 := 0;
    signal gap_cnt  : integer range 0 to 3 := 0;

    signal scl_i      : std_logic := '1';
    signal sda_drive0 : std_logic := '0';

    signal seg_latched  : std_logic_vector(6 downto 0) := (others => '0');
    signal dsel_latched : std_logic := '0';

    signal cur_byte : std_logic_vector(7 downto 0);

begin

    scl <= scl_i;
    sda <= '0' when sda_drive0 = '1' else 'Z';

    process(clk, rst)
    begin
        if rst = '1' then
            half_timer <= 0;
            half_tick  <= '0';
        elsif rising_edge(clk) then
            if half_timer = HALF_CYCLES - 1 then
                half_timer <= 0;
                half_tick  <= '1';
            else
                half_timer <= half_timer + 1;
                half_tick  <= '0';
            end if;
        end if;
    end process;

    process(txn_sel, byte_idx, seg_latched, dsel_latched)
        variable reg_addr  : std_logic_vector(7 downto 0);
        variable data_byte : std_logic_vector(7 downto 0);
    begin
        case txn_sel is
            when 0 => reg_addr := x"06"; data_byte := x"00";
            when 1 => reg_addr := x"07"; data_byte := x"00";
            when 2 => reg_addr := x"02"; data_byte := "00000000";
            when 3 =>
                reg_addr := x"03";
                if dsel_latched = '1' then
                    data_byte := "00000001";
                else
                    data_byte := "00000010";
                end if;
            when others => reg_addr := x"02"; data_byte := "0" & seg_latched;
        end case;

        case byte_idx is
            when 0      => cur_byte <= I2C_ADDR & '0';
            when 1      => cur_byte <= reg_addr;
            when others => cur_byte <= data_byte;
        end case;
    end process;

    process(clk, rst)
    begin
        if rst = '1' then
            bstate       <= S_IDLE;
            scl_i        <= '1';
            sda_drive0   <= '0';
            bit_cnt      <= 0;
            byte_idx     <= 0;
            txn_sel      <= 0;
            gap_cnt      <= 0;
            seg_latched  <= (others => '0');
            dsel_latched <= '0';
        elsif rising_edge(clk) then
            if half_tick = '1' then
                case bstate is

                    when S_IDLE =>
                        scl_i      <= '1';
                        sda_drive0 <= '0';
                        bit_cnt  <= 0;
                        byte_idx <= 0;
                        bstate   <= S_START;

                    when S_START =>
                        scl_i      <= '1';
                        sda_drive0 <= '1';
                        bstate     <= S_BIT_LOW;

                    when S_BIT_LOW =>
                        scl_i      <= '0';
                        sda_drive0 <= not cur_byte(7 - bit_cnt);
                        bstate     <= S_BIT_HIGH;

                    when S_BIT_HIGH =>
                        scl_i <= '1';
                        if bit_cnt = 7 then
                            bstate <= S_ACK_LOW;
                        else
                            bit_cnt <= bit_cnt + 1;
                            bstate  <= S_BIT_LOW;
                        end if;

                    when S_ACK_LOW =>
                        scl_i      <= '0';
                        sda_drive0 <= '0';
                        bstate     <= S_ACK_HIGH;

                    when S_ACK_HIGH =>
                        scl_i <= '1';
                        if byte_idx = 2 then
                            bstate <= S_STOP1;
                        else
                            byte_idx <= byte_idx + 1;
                            bit_cnt  <= 0;
                            bstate   <= S_BIT_LOW;
                        end if;

                    when S_STOP1 =>
                        scl_i      <= '0';
                        sda_drive0 <= '1';
                        bstate     <= S_STOP2;

                    when S_STOP2 =>
                        scl_i      <= '1';
                        sda_drive0 <= '0';
                        bstate     <= S_GAP;
                        gap_cnt    <= 0;

                    when S_GAP =>
                        if gap_cnt = 3 then
                            case txn_sel is
                                when 0 =>
                                    txn_sel <= 1;
                                when 1 =>
                                    txn_sel      <= 2;
                                    seg_latched  <= seg;
                                    dsel_latched <= digit_sel;
                                when 2 =>
                                    txn_sel <= 3;
                                when 3 =>
                                    txn_sel <= 4;
                                when others =>
                                    txn_sel      <= 2;
                                    seg_latched  <= seg;
                                    dsel_latched <= digit_sel;
                            end case;
                            bstate <= S_IDLE;
                        else
                            gap_cnt <= gap_cnt + 1;
                        end if;

                end case;
            end if;
        end if;
    end process;

end Behavioral;
