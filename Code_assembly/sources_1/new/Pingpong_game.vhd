library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pingpong_game is
    generic (
        TICK_DIVISOR      : integer := 25_000_000;
        DEBOUNCE_LIM      : integer := 1_000_000;
        HIT_WINDOW_CYCLES : integer := 50_000_000;
        BLANK_HOLD_CYCLES : integer := 100_000_000;
        SCAN_DIVISOR      : integer := 200_000
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        btn_raw   : in  std_logic;

        is_master : in  std_logic;

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
end pingpong_game;

architecture Behavioral of pingpong_game is

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

    component ball_counter is
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
    end component;

    type state_t is (S_MOVING, S_WAIT_HIT, S_POINT, S_BLANK, S_WAIT_SERVE, S_AWAY);

    signal state, next_state : state_t := S_WAIT_SERVE;

    signal btn_db : std_logic;

    signal ball_pos     : unsigned(2 downto 0);
    signal ball_dir     : std_logic;
    signal at_left, at_right : std_logic;
    signal ball_en      : std_logic;
    signal ball_set_pos : std_logic;
    signal ball_set_val : unsigned(2 downto 0);
    signal leaving_to_away : std_logic;

    signal game_tick : std_logic := '0';
    signal tick_cnt   : integer range 0 to TICK_DIVISOR - 1 := 0;

    signal wait_cnt : integer range 0 to HIT_WINDOW_CYCLES - 1 := 0;
    signal moved_since_entry : std_logic := '0';
    signal blank_cnt : integer range 0 to BLANK_HOLD_CYCLES - 1 := 0;

    signal score_tens_i, score_ones_i : unsigned(3 downto 0) := (others => '0');

    signal scan_cnt    : integer range 0 to SCAN_DIVISOR - 1 := 0;
    signal digit_sel_i : std_logic := '0';

begin

    U_DB_BTN : debounce
        generic map (DEBOUNCE_LIMIT => DEBOUNCE_LIM)
        port map (clk => clk, rst => rst, btn_in => btn_raw, btn_out => open, btn_level => btn_db);

    U_BALL : ball_counter
        port map (clk => clk, rst => rst, en => ball_en,
                  set_pos => ball_set_pos, set_val => ball_set_val,
                  pos => ball_pos, dir => ball_dir, at_left => at_left, at_right => at_right);

    process(clk, rst)
    begin
        if rst = '1' then
            tick_cnt  <= 0;
            game_tick <= '0';
        elsif rising_edge(clk) then
            if tick_cnt = TICK_DIVISOR - 1 then
                tick_cnt  <= 0;
                game_tick <= '1';
            else
                tick_cnt  <= tick_cnt + 1;
                game_tick <= '0';
            end if;
        end if;
    end process;

    process(clk, rst)
    begin
        if rst = '1' then
            if is_master = '1' then
                state <= S_WAIT_SERVE;
            else
                state <= S_AWAY;
            end if;
        elsif rising_edge(clk) then
            state <= next_state;
        end if;
    end process;

    process(clk, rst)
    begin
        if rst = '1' then
            wait_cnt          <= 0;
            moved_since_entry <= '0';
            blank_cnt         <= 0;
        elsif rising_edge(clk) then
            case state is
                when S_MOVING =>
                    wait_cnt  <= 0;
                    blank_cnt <= 0;
                    if game_tick = '1' then
                        moved_since_entry <= '1';
                    end if;

                when S_WAIT_HIT =>
                    moved_since_entry <= '0';
                    blank_cnt         <= 0;
                    if wait_cnt < HIT_WINDOW_CYCLES - 1 then
                        wait_cnt <= wait_cnt + 1;
                    end if;

                when S_BLANK =>
                    moved_since_entry <= '0';
                    wait_cnt          <= 0;
                    if blank_cnt < BLANK_HOLD_CYCLES - 1 then
                        blank_cnt <= blank_cnt + 1;
                    end if;

                when others =>
                    wait_cnt          <= 0;
                    moved_since_entry <= '0';
                    blank_cnt         <= 0;
            end case;
        end if;
    end process;

    process(state, btn_db, at_left, wait_cnt, ball_dir, moved_since_entry,
            handoff_rx, score_rx, blank_cnt, leaving_to_away)
    begin
        next_state <= state;
        case state is
            when S_MOVING =>
                if ball_dir = '0' and btn_db = '1' then
                    next_state <= S_POINT;
                elsif moved_since_entry = '1' and at_left = '1' then
                    next_state <= S_WAIT_HIT;
                elsif leaving_to_away = '1' then
                    next_state <= S_AWAY;
                end if;

            when S_WAIT_HIT =>
                if btn_db = '1' then
                    next_state <= S_MOVING;
                elsif wait_cnt >= HIT_WINDOW_CYCLES - 1 then
                    next_state <= S_POINT;
                end if;

            when S_POINT =>
                next_state <= S_BLANK;

            when S_BLANK =>
                if blank_cnt >= BLANK_HOLD_CYCLES - 1 then
                    next_state <= S_WAIT_SERVE;
                end if;

            when S_WAIT_SERVE =>
                if btn_db = '1' then
                    next_state <= S_MOVING;
                end if;

            when S_AWAY =>
                if handoff_rx = '1' then
                    next_state <= S_MOVING;
                end if;
        end case;
    end process;

    leaving_to_away <= '1' when (state = S_MOVING and moved_since_entry = '1'
                                  and at_right = '1' and game_tick = '1') else '0';
    ball_en <= '1' when (state = S_MOVING and game_tick = '1' and leaving_to_away = '0') else '0';

    ball_set_pos <= '1' when (state = S_POINT) or (state = S_AWAY and handoff_rx = '1') else '0';
    ball_set_val <= to_unsigned(0, 3) when state = S_POINT else
                     to_unsigned(7, 3) when (state = S_AWAY and handoff_rx = '1') else
                     (others => '0');

    handoff_tx <= leaving_to_away;
    score_tx   <= '1' when state = S_POINT else '0';

    process(clk, rst)
    begin
        if rst = '1' then
            score_tens_i <= (others => '0');
            score_ones_i <= (others => '0');
        elsif rising_edge(clk) then
            if score_rx = '1' then
                if score_ones_i = 9 then
                    score_ones_i <= (others => '0');
                    if score_tens_i = 9 then
                        score_tens_i <= (others => '0');
                    else
                        score_tens_i <= score_tens_i + 1;
                    end if;
                else
                    score_ones_i <= score_ones_i + 1;
                end if;
            end if;
        end if;
    end process;

    score_tens <= score_tens_i;
    score_ones <= score_ones_i;

    process(ball_pos, state)
        variable led_v : std_logic_vector(7 downto 0);
    begin
        led_v := (others => '0');
        if state /= S_BLANK and state /= S_AWAY then
            led_v(to_integer(ball_pos)) := '1';
        end if;
        led_out <= led_v;
    end process;

    process(clk, rst)
    begin
        if rst = '1' then
            scan_cnt    <= 0;
            digit_sel_i <= '0';
        elsif rising_edge(clk) then
            if scan_cnt = SCAN_DIVISOR - 1 then
                scan_cnt    <= 0;
                digit_sel_i <= not digit_sel_i;
            else
                scan_cnt <= scan_cnt + 1;
            end if;
        end if;
    end process;

    digit_sel <= digit_sel_i;

    process(digit_sel_i, score_tens_i, score_ones_i)
        variable sel_digit : unsigned(3 downto 0);
    begin
        if digit_sel_i = '1' then
            sel_digit := score_tens_i;
        else
            sel_digit := score_ones_i;
        end if;

        case to_integer(sel_digit) is
            when 0      => seg <= "0111111";
            when 1      => seg <= "0000110";
            when 2      => seg <= "1011011";
            when 3      => seg <= "1001111";
            when 4      => seg <= "1100110";
            when 5      => seg <= "1101101";
            when 6      => seg <= "1111101";
            when 7      => seg <= "0000111";
            when 8      => seg <= "1111111";
            when 9      => seg <= "1101111";
            when others => seg <= "0000000";
        end case;
    end process;

end Behavioral;
