library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_pingpong_game is
end tb_pingpong_game;

architecture sim of tb_pingpong_game is

    constant CLK_PERIOD : time := 10 ns;

    constant TICK_DIVISOR      : integer := 4;
    constant DEBOUNCE_LIM      : integer := 3;
    constant HIT_WINDOW_CYCLES : integer := 60;
    constant BLANK_HOLD_CYCLES : integer := 15;
    constant TRAVEL_WAIT       : integer := TICK_DIVISOR * 12;
    constant SCAN_DIVISOR      : integer := 10;

    signal clk, rst   : std_logic := '0';
    signal btn_raw    : std_logic := '0';
    signal led_out    : std_logic_vector(7 downto 0);
    signal seg        : std_logic_vector(6 downto 0);
    signal digit_sel  : std_logic;
    signal score_tens, score_ones : unsigned(3 downto 0);
    signal handoff_tx, score_tx : std_logic;
    signal handoff_rx, score_rx : std_logic := '0';

    signal errors : integer := 0;
    signal handoff_tx_cnt : integer := 0;
    signal score_tx_cnt   : integer := 0;

    procedure wait_clks(n : integer; signal clk : in std_logic) is
    begin
        for i in 1 to n loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

    procedure press_btn(signal b : out std_logic; signal clk : in std_logic) is
    begin
        b <= '1';
        wait_clks(DEBOUNCE_LIM + 2, clk);
        b <= '0';
        wait_clks(DEBOUNCE_LIM + 2, clk);
    end procedure;

    procedure check(cond : boolean; msg : string; signal errors : inout integer) is
    begin
        if not cond then
            report "FAIL: " & msg severity error;
            errors <= errors + 1;
        else
            report "PASS: " & msg;
        end if;
    end procedure;

begin

    UUT : entity work.pingpong_game
        generic map (
            TICK_DIVISOR      => TICK_DIVISOR,
            DEBOUNCE_LIM      => DEBOUNCE_LIM,
            HIT_WINDOW_CYCLES => HIT_WINDOW_CYCLES,
            BLANK_HOLD_CYCLES => BLANK_HOLD_CYCLES,
            SCAN_DIVISOR      => SCAN_DIVISOR
        )
        port map (
            clk => clk, rst => rst, btn_raw => btn_raw,
            is_master => '1',
            led_out => led_out, seg => seg, digit_sel => digit_sel,
            score_tens => score_tens, score_ones => score_ones,
            handoff_tx => handoff_tx, handoff_rx => handoff_rx,
            score_tx => score_tx, score_rx => score_rx
        );

    clk <= not clk after CLK_PERIOD / 2;

    stim : process
    begin
        rst <= '1';
        wait_clks(5, clk);
        rst <= '0';
        wait_clks(2, clk);

        check(led_out = "00000001", "reset: ball resting at LED0 (pos=0)", errors);

        press_btn(btn_raw, clk);

        wait_clks(TRAVEL_WAIT, clk);

        check(led_out = "00000000", "after handoff sent, board goes S_AWAY: LEDs blank", errors);
        check(handoff_tx_cnt = 1, "handoff_tx pulsed exactly once during 0->7 travel", errors);

        handoff_rx <= '1';
        wait_clks(1, clk);
        handoff_rx <= '0';
        wait_clks(3, clk);
        check(led_out = "10000000", "handoff_rx received: ball re-enters at pos7 (LED7)", errors);

        wait_clks(TRAVEL_WAIT, clk);
        check(led_out = "00000001", "after inbound travel, ball reached pos0, waiting for hit", errors);

        press_btn(btn_raw, clk);

        wait_clks(TRAVEL_WAIT, clk);
        check(led_out = "00000000", "after bounce-back reaches pos7 again, handed off, LEDs blank (S_AWAY)", errors);

        handoff_rx <= '1';
        wait_clks(1, clk);
        handoff_rx <= '0';
        wait_clks(TRAVEL_WAIT, clk);
        check(led_out = "00000001", "second inbound arrival at pos0", errors);

        wait_clks(HIT_WINDOW_CYCLES + 5, clk);
        wait_clks(BLANK_HOLD_CYCLES + 10, clk);
        check(led_out = "00000001", "after miss: blanked then back to WAIT_SERVE at own pos0", errors);
        check(score_tens = 0 and score_ones = 0, "a LOCAL miss must NOT increment my own score", errors);

        score_rx <= '1';
        wait_clks(1, clk);
        score_rx <= '0';
        wait_clks(3, clk);
        check(score_tens = 0 and score_ones = 1, "score_rx bumps my local score to 1", errors);

        press_btn(btn_raw, clk);
        wait_clks(TRAVEL_WAIT, clk);

        handoff_rx <= '1';
        wait_clks(1, clk);
        handoff_rx <= '0';
        wait_clks(TICK_DIVISOR * 2, clk);

        press_btn(btn_raw, clk);
        wait_clks(BLANK_HOLD_CYCLES + 10, clk);
        check(led_out = "00000001", "after foul: also resets to own pos0, WAIT_SERVE", errors);

        wait_clks(10, clk);
        check(handoff_tx_cnt = 3, "exactly 3 handoff_tx pulses total (3 outbound crossings)", errors);
        check(score_tx_cnt = 2, "exactly 2 score_tx pulses total (1 timeout miss + 1 foul)", errors);

        report "=== TESTBENCH DONE, total errors: " & integer'image(errors) & " ===";
        wait;
    end process;

    count_handoff : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                handoff_tx_cnt <= 0;
            elsif handoff_tx = '1' then
                handoff_tx_cnt <= handoff_tx_cnt + 1;
            end if;
        end if;
    end process;

    count_score : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                score_tx_cnt <= 0;
            elsif score_tx = '1' then
                score_tx_cnt <= score_tx_cnt + 1;
            end if;
        end if;
    end process;

end sim;
