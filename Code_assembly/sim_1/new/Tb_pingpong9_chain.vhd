library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_pingpong9_chain is
end tb_pingpong9_chain;

architecture sim of tb_pingpong9_chain is

    constant CLK_PERIOD : time := 10 ns;

    constant TICK_DIVISOR      : integer := 4;
    constant DEBOUNCE_LIM      : integer := 3;
    constant HIT_WINDOW_CYCLES : integer := 40_000;
    constant BLANK_HOLD_CYCLES : integer := 20;
    constant SCAN_DIVISOR      : integer := 10;
    constant LINK_BIT_FREQ     : integer := 1_000_000;
    constant TRAVEL_WAIT       : integer := TICK_DIVISOR * 12;

    constant GRST_PROPAGATE_DELAY_CYCLES : integer := 8_000;

    signal clk_a, clk_b : std_logic := '0';
    signal grst_a, grst_b : std_logic := '0';

    signal btn_a, btn_b : std_logic := '0';
    signal led_a, led_b : std_logic_vector(7 downto 0);
    signal score_tens_a, score_ones_a : unsigned(3 downto 0);
    signal score_tens_b, score_ones_b : unsigned(3 downto 0);

    signal link_wire : std_logic;
    signal seg_scl_a, seg_scl_b : std_logic;
    signal seg_sda_a, seg_sda_b : std_logic;

    signal errors : integer := 0;

    procedure check(cond : boolean; msg : string; signal errors : inout integer) is
    begin
        if not cond then
            report "FAIL: " & msg severity error;
            errors <= errors + 1;
        else
            report "PASS: " & msg;
        end if;
    end procedure;

    procedure wait_clks(n : integer; signal clk : in std_logic) is
    begin
        for i in 1 to n loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

    procedure press(signal b : out std_logic; signal clk : in std_logic) is
    begin
        b <= '1';
        wait_clks(DEBOUNCE_LIM + 10, clk);
        b <= '0';
        wait_clks(DEBOUNCE_LIM + 10, clk);
    end procedure;

begin

    seg_scl_a <= 'H'; seg_sda_a <= 'H';
    seg_scl_b <= 'H'; seg_sda_b <= 'H';

    link_wire <= 'H';

    BOARD_A : entity work.pingpong9_hw_top
        generic map (
            TICK_DIVISOR => TICK_DIVISOR, DEBOUNCE_LIM => DEBOUNCE_LIM,
            HIT_WINDOW_CYCLES => HIT_WINDOW_CYCLES, BLANK_HOLD_CYCLES => BLANK_HOLD_CYCLES,
            SCAN_DIVISOR => SCAN_DIVISOR, LINK_BIT_FREQ_HZ => LINK_BIT_FREQ,
            GRST_DEBOUNCE_LIM => DEBOUNCE_LIM,
            GRST_PROPAGATE_DELAY_CYCLES => GRST_PROPAGATE_DELAY_CYCLES
        )
        port map (
            clk => clk_a, btn_GRST => grst_a, btn_raw => btn_a, sw_master => '1', led_out => led_a,
            score_tens => score_tens_a, score_ones => score_ones_a,
            seg_scl => seg_scl_a, seg_sda => seg_sda_a,
            link_wire => link_wire
        );

    BOARD_B : entity work.pingpong9_hw_top
        generic map (
            TICK_DIVISOR => TICK_DIVISOR, DEBOUNCE_LIM => DEBOUNCE_LIM,
            HIT_WINDOW_CYCLES => HIT_WINDOW_CYCLES, BLANK_HOLD_CYCLES => BLANK_HOLD_CYCLES,
            SCAN_DIVISOR => SCAN_DIVISOR, LINK_BIT_FREQ_HZ => LINK_BIT_FREQ,
            GRST_DEBOUNCE_LIM => DEBOUNCE_LIM,
            GRST_PROPAGATE_DELAY_CYCLES => GRST_PROPAGATE_DELAY_CYCLES
        )
        port map (
            clk => clk_b, btn_GRST => grst_b, btn_raw => btn_b, sw_master => '0', led_out => led_b,
            score_tens => score_tens_b, score_ones => score_ones_b,
            seg_scl => seg_scl_b, seg_sda => seg_sda_b,
            link_wire => link_wire
        );

    clk_a <= not clk_a after CLK_PERIOD / 2;
    clk_b <= not clk_b after (CLK_PERIOD / 2) + 1 ns;

    stim : process
    begin
        press(grst_a, clk_a);
        press(grst_b, clk_b);
        wait_clks(5, clk_a);

        wait for 400 us;

        check(led_a = "00000001", "board A reset: ball at its own pos0", errors);
        check(led_b = "00000000", "board B reset: idle, no ball (S_AWAY, LEDs blank)", errors);

        press(btn_a, clk_a);
        wait_clks(TRAVEL_WAIT, clk_a);
        check(led_a = "00000000", "A: ball left toward net, A now blank (S_AWAY)", errors);

        wait for 300 us;
        wait_clks(TRAVEL_WAIT, clk_b);
        check(led_b = "10000000", "B: ball reached B's own pos0, waiting for B's hit", errors);

        press(btn_b, clk_b);
        wait_clks(TRAVEL_WAIT, clk_b);
        check(led_b = "00000000", "B: bounced back out, handed off again, B now blank", errors);

        check(score_tens_a = 0 and score_ones_a = 0,
              "A: score still 0 right after B's successful hit (no spurious miss on B's side)", errors);

        wait for 300 us;
        wait_clks(TRAVEL_WAIT, clk_a);
        check(led_a = "00000001", "A: ball reached A's own pos0 again, waiting for A's hit", errors);

        wait_clks(HIT_WINDOW_CYCLES + 10, clk_a);
        wait_clks(BLANK_HOLD_CYCLES + 10, clk_a);
        check(led_a = "00000001", "A: after miss, blanked then back to A's own WAIT_SERVE", errors);

        wait for 300 us;
        check(score_tens_b = 0 and score_ones_b = 1,
              "B: score bumped to 1 after A's miss (score_tx/score_rx via real link_wire)", errors);


        press(grst_a, clk_a);

        wait for 300 us;
        wait_clks(GRST_PROPAGATE_DELAY_CYCLES + 1000, clk_b);

        check(led_a = "00000001", "A: after local GRST, back to A's own WAIT_SERVE", errors);
        check(led_b = "00000000", "B: reset via link_wire grst propagation (grst_b was NEVER pressed!)", errors);
        check(score_tens_b = 0 and score_ones_b = 0,
              "B: score cleared back to 0 -- proof this is a REAL reset, not just idle S_AWAY", errors);
        check(score_tens_a = 0 and score_ones_a = 0,
              "A: score also cleared by its own local GRST", errors);

        report "=== TESTBENCH DONE, total errors: " & integer'image(errors) & " ===";
        wait;
    end process;

end sim;
