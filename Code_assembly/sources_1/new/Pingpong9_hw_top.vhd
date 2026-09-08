library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pingpong9_hw_top is
    generic (
        TICK_DIVISOR      : integer := 25_000_000;
        DEBOUNCE_LIM      : integer := 1_000_000;
        HIT_WINDOW_CYCLES : integer := 50_000_000;
        BLANK_HOLD_CYCLES : integer := 100_000_000;
        SCAN_DIVISOR      : integer := 200_000;
        GRST_DEBOUNCE_LIM : integer := 1_000_000;

        LINK_BIT_FREQ_HZ : integer := 200_000;

        GRST_PROPAGATE_DELAY_CYCLES : integer := 10_000_000
    );
    port (
        clk      : in  std_logic;
        btn_GRST : in  std_logic;
        btn_raw  : in  std_logic;
        sw_master : in std_logic;
        led_out  : out std_logic_vector(7 downto 0);

        score_tens : out unsigned(3 downto 0);
        score_ones : out unsigned(3 downto 0);

        seg_scl  : out std_logic;
        seg_sda  : inout std_logic;

        link_wire : inout std_logic
    );
end pingpong9_hw_top;

architecture Structural of pingpong9_hw_top is

    component debounce is
        generic (
            DEBOUNCE_LIMIT : integer := 1_000_000
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            btn_in    : in  std_logic;
            btn_out   : out std_logic;
            btn_level : out std_logic
        );
    end component;

    component pingpong_game is
        generic (
            TICK_DIVISOR      : integer;
            DEBOUNCE_LIM      : integer;
            HIT_WINDOW_CYCLES : integer;
            BLANK_HOLD_CYCLES : integer;
            SCAN_DIVISOR      : integer
        );
        port (
            clk        : in  std_logic;
            rst        : in  std_logic;
            btn_raw    : in  std_logic;
            is_master  : in  std_logic;
            led_out    : out std_logic_vector(7 downto 0);
            seg        : out std_logic_vector(6 downto 0);
            digit_sel  : out std_logic;
            score_tens : out unsigned(3 downto 0);
            score_ones : out unsigned(3 downto 0);
            handoff_tx : out std_logic;
            handoff_rx : in  std_logic;
            score_tx   : out std_logic;
            score_rx   : in  std_logic
        );
    end component;

    component i2c_tca6416_driver is
        generic (
            CLK_FREQ_HZ : integer;
            I2C_FREQ_HZ : integer;
            I2C_ADDR    : std_logic_vector(6 downto 0)
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            seg       : in  std_logic_vector(6 downto 0);
            digit_sel : in  std_logic;
            scl       : out std_logic;
            sda       : inout std_logic
        );
    end component;

    component link_wire_drv is
        generic (
            CLK_FREQ_HZ      : integer;
            LINK_BIT_FREQ_HZ : integer
        );
        port (
            clk              : in  std_logic;
            rst              : in  std_logic;
            is_master        : in  std_logic;
            link_wire        : inout std_logic;
            handoff_tx_pulse : in  std_logic;
            score_tx_pulse   : in  std_logic;
            handoff_rx_pulse : out std_logic;
            score_rx_pulse   : out std_logic;
            grst_tx_pulse    : in  std_logic;
            grst_rx_pulse    : out std_logic
        );
    end component;

    signal rst_level : std_logic;
    signal combined_rst : std_logic;

    signal seg_i         : std_logic_vector(6 downto 0);
    signal digit_sel_i   : std_logic;

    signal handoff_tx_i, handoff_rx_i : std_logic;
    signal score_tx_i, score_rx_i     : std_logic;
    signal grst_tx_i, grst_rx_i       : std_logic;

    signal sw_master_db     : std_logic;
    signal is_master_latched : std_logic := '1';

    signal led_out_i : std_logic_vector(7 downto 0);

    signal rst_level_prev  : std_logic := '0';
    signal grst_delay_run  : std_logic := '0';
    signal grst_delay_cnt  : integer range 0 to GRST_PROPAGATE_DELAY_CYCLES - 1 := 0;
    signal grst_reset_now  : std_logic := '0';

begin

    U_DB_GRST : debounce
        generic map (DEBOUNCE_LIMIT => GRST_DEBOUNCE_LIM)
        port map (clk => clk, rst => '0', btn_in => btn_GRST, btn_out => open, btn_level => rst_level);

    U_DB_SWMASTER : debounce
        generic map (DEBOUNCE_LIMIT => DEBOUNCE_LIM)
        port map (clk => clk, rst => '0', btn_in => sw_master, btn_out => open, btn_level => sw_master_db);

    combined_rst <= rst_level or grst_reset_now;

    process(clk)
    begin
        if rising_edge(clk) then
            if combined_rst = '1' then
                is_master_latched <= sw_master_db;
            end if;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then
            rst_level_prev <= rst_level;
            grst_tx_i <= rst_level_prev and not rst_level;

            if grst_rx_i = '1' then
                grst_delay_run <= '1';
                grst_delay_cnt <= 0;
                grst_reset_now <= '0';
            elsif grst_delay_run = '1' then
                if grst_delay_cnt = GRST_PROPAGATE_DELAY_CYCLES - 1 then
                    grst_delay_run <= '0';
                    grst_reset_now <= '1';
                else
                    grst_delay_cnt <= grst_delay_cnt + 1;
                    grst_reset_now <= '0';
                end if;
            else
                grst_reset_now <= '0';
            end if;
        end if;
    end process;

    U_GAME : pingpong_game
        generic map (
            TICK_DIVISOR      => TICK_DIVISOR,
            DEBOUNCE_LIM      => DEBOUNCE_LIM,
            HIT_WINDOW_CYCLES => HIT_WINDOW_CYCLES,
            BLANK_HOLD_CYCLES => BLANK_HOLD_CYCLES,
            SCAN_DIVISOR      => SCAN_DIVISOR
        )
        port map (
            clk        => clk,
            rst        => combined_rst,
            btn_raw    => btn_raw,
            is_master  => is_master_latched,
            led_out    => led_out_i,
            seg        => seg_i,
            digit_sel  => digit_sel_i,
            score_tens => score_tens,
            score_ones => score_ones,
            handoff_tx => handoff_tx_i,
            handoff_rx => handoff_rx_i,
            score_tx   => score_tx_i,
            score_rx   => score_rx_i
        );

    U_I2C_SEG : i2c_tca6416_driver
        generic map (
            CLK_FREQ_HZ => 100_000_000,
            I2C_FREQ_HZ => 100_000,
            I2C_ADDR    => "0100000"
        )
        port map (
            clk       => clk,
            rst       => '0',
            seg       => seg_i,
            digit_sel => digit_sel_i,
            scl       => seg_scl,
            sda       => seg_sda
        );

    U_LINK_WIRE : link_wire_drv
        generic map (
            CLK_FREQ_HZ      => 100_000_000,
            LINK_BIT_FREQ_HZ => LINK_BIT_FREQ_HZ
        )
        port map (
            clk              => clk,
            rst              => combined_rst,
            is_master        => is_master_latched,
            link_wire        => link_wire,
            handoff_tx_pulse => handoff_tx_i,
            score_tx_pulse   => score_tx_i,
            handoff_rx_pulse => handoff_rx_i,
            score_rx_pulse   => score_rx_i,
            grst_tx_pulse    => grst_tx_i,
            grst_rx_pulse    => grst_rx_i
        );

    led_mux_gen : for i in 0 to 7 generate
        led_out(i) <= led_out_i(i) when is_master_latched = '1' else led_out_i(7 - i);
    end generate led_mux_gen;

end Structural;
