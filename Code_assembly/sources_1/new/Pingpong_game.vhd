library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Project 9核心遊戲模組。延伸自pingpong_top.vhd(Project 5/6)，改成雙板分離架構。
-- 每片板子各跑一份這個模組，而且永遠只追蹤「自己這邊」的球況，不知道對面板子的狀態。
--
-- 本地pos=0永遠是「自己這端的hit-window」(跟Project5/6單板版的每邊語意一致)。
-- 本地pos=7永遠是「面網端」——在S_MOVING狀態下走到pos=7不再代表hit-window，
-- 而是代表球已經離開這片板子，要透過跨板link把球交接給對面板子。
--
-- 因為ball_counter的set_pos/set_val介面本身就有「set_val=0自動設dir='1'、
-- set_val=7自動設dir='0'」的慣例，跨板交接完全不需要在ball_counter.vhd裡加新邏輯：
-- 接球的那片板子只要用set_val=7去pulse set_pos，球就已經自動往自己的pos=0方向走了，不用額外接線。
--
-- 計分不再由雙方各自本地計算：本地漏接/犯規「永遠」代表對面(另一片板子)得分，
-- 所以這個模組從來不會因為本地事件就增加自己的分數——它只會pulse score_tx
-- (通知對面板子去加它自己的分數)，而自己的score_tens/ones只會在收到外部
-- score_rx脈波時才增加。這樣原本pingpong_top.vhd裡award_to_a/hit_side_is_right
-- 那種「兩邊分數互相判斷」的記帳邏輯完全拿掉了，因為單板實體從來不需要知道「是哪一邊」——
-- 永遠只有唯一的「本地這一邊」。
entity pingpong_game is
    generic (
        TICK_DIVISOR      : integer := 25_000_000;   -- 球移動的tick除頻值，模擬時建議override成小數值
        DEBOUNCE_LIM      : integer := 1_000_000;    -- 按鍵/GRST去彈跳門檻，模擬時建議override成小數值
        HIT_WINDOW_CYCLES : integer := 50_000_000;   -- 球到達後容許擊球的時間窗，約0.5秒
        BLANK_HOLD_CYCLES : integer := 100_000_000;  -- 本地漏接後的熄燈停頓時間，約1秒
        SCAN_DIVISOR      : integer := 200_000       -- 七段顯示器數位切換頻率，約250Hz
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        btn_raw   : in  std_logic;   -- 這片板子唯一的擊球/發球按鍵，未去彈跳原始訊號

        -- runtime決定這片板子這次是master還是slave：'1'=master(開機球在我這邊，
        -- state從S_WAIT_SERVE開始)、'0'=slave(開機球不在我這邊，state從S_AWAY開始)。
        -- 取代原本的START_IN_AWAY generic，由上層依SW0開關(鎖存過)餵進來——詳見
        -- pingpong9_hw_top.vhd。因為這是runtime訊號、不是編譯期generic，GSR沒辦法
        -- 再幫忙讓開機第一瞬間就自動對；燒完board之後，第一次必須先按過一次GRST
        -- (或等對面shared-GRST relay過來)，state才會依is_master正確初始化。
        is_master : in  std_logic;

        led_out    : out std_logic_vector(7 downto 0);   -- LED0~LED7，LED0=自己這端，LED7=面網端
        seg        : out std_logic_vector(6 downto 0);
        digit_sel  : out std_logic;
        score_tens : out unsigned(3 downto 0);
        score_ones : out unsigned(3 downto 0);

        -- 跨板link介面，全部都是單一clk寬的脈波
        handoff_tx : out std_logic;   -- pulse：球剛離開我、朝net端過去了，通知對面板子
        handoff_rx : in  std_logic;   -- pulse：對面板子說球現在是我的了(從pos7進場)
        score_tx   : out std_logic;   -- pulse：我剛漏接/犯規了，通知對面板子加分
        score_rx   : in  std_logic    -- pulse：對面板子說我得分了，加我自己的本地分數
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

    -- S_AWAY：球目前不在這片板子上，等待handoff_rx(球要進場)或score_rx
    -- (對面漏接了，我只需要加分、繼續等，球此刻仍停在對面板子上)。
    type state_t is (S_MOVING, S_WAIT_HIT, S_POINT, S_BLANK, S_WAIT_SERVE, S_AWAY);

    -- 以前這裡的初始值是用function依編譯期generic(START_IN_AWAY)決定，讓GSR
    -- 開機那一瞬間就已經是對的、不必等按GRST。改成is_master是runtime port之後，
    -- 這招用不了了(generic才能餵進宣告時的初始值，runtime訊號不行)——這裡改回
    -- 單一固定的預設值，兩板燒完bitstream、GSR剛跑完的那一瞬間會暫時都長得一樣
    -- (都是S_WAIT_SERVE)，要等第一次combined_rst(本地按GRST或對面shared-GRST
    -- relay過來)發生後，Process 2才會依is_master把state導正成正確的起始狀態。
    signal state, next_state : state_t := S_WAIT_SERVE;

    signal btn_db : std_logic;

    signal ball_pos     : unsigned(2 downto 0);
    signal ball_dir     : std_logic;
    signal at_left, at_right : std_logic;
    signal ball_en      : std_logic;
    signal ball_set_pos : std_logic;
    signal ball_set_val : unsigned(2 downto 0);
    signal leaving_to_away : std_logic;   -- pulse：這一拍要把state從S_MOVING切到S_AWAY(pos=7且game_tick到了)

    signal game_tick : std_logic := '0';
    signal tick_cnt   : integer range 0 to TICK_DIVISOR - 1 := 0;

    signal wait_cnt : integer range 0 to HIT_WINDOW_CYCLES - 1 := 0;
    signal moved_since_entry : std_logic := '0';   -- 防止「剛進入這個位置」(發球或交接進場)
                                                     -- 立刻被誤判成又觸發了一次到達
    signal blank_cnt : integer range 0 to BLANK_HOLD_CYCLES - 1 := 0;

    signal score_tens_i, score_ones_i : unsigned(3 downto 0) := (others => '0');

    signal scan_cnt    : integer range 0 to SCAN_DIVISOR - 1 := 0;
    signal digit_sel_i : std_logic := '0';   -- '0'=個位, '1'=十位

begin

    U_DB_BTN : debounce
        generic map (DEBOUNCE_LIMIT => DEBOUNCE_LIM)
        port map (clk => clk, rst => rst, btn_in => btn_raw, btn_out => open, btn_level => btn_db);

    U_BALL : ball_counter
        port map (clk => clk, rst => rst, en => ball_en,
                  set_pos => ball_set_pos, set_val => ball_set_val,
                  pos => ball_pos, dir => ball_dir, at_left => at_left, at_right => at_right);

    -- ===== Process 1：game_tick除頻器 =====
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

    -- ===== Process 2：FSM狀態暫存器 =====
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

    -- ===== Process 3：計時器 + moved_since_entry記帳 =====
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

                when others =>   -- S_POINT, S_WAIT_SERVE, S_AWAY
                    wait_cnt          <= 0;
                    moved_since_entry <= '0';
                    blank_cnt         <= 0;
            end case;
        end if;
    end process;

    -- ===== Process 4：FSM次態邏輯 =====
    process(state, btn_db, at_left, wait_cnt, ball_dir, moved_since_entry,
            handoff_rx, score_rx, blank_cnt, leaving_to_away)
    begin
        next_state <= state;
        case state is
            when S_MOVING =>
                -- 犯規：球還在朝我這邊來的路上(還沒到pos=0)就按了自己的鍵，算搶按
                if ball_dir = '0' and btn_db = '1' then
                    next_state <= S_POINT;
                elsif moved_since_entry = '1' and at_left = '1' then
                    next_state <= S_WAIT_HIT;              -- 到達自己這端的hit-window
                elsif leaving_to_away = '1' then
                    next_state <= S_AWAY;                  -- 到達面網端，交接出去
                end if;

            when S_WAIT_HIT =>
                if btn_db = '1' then
                    next_state <= S_MOVING;                -- 及時接到，球彈回(方向翻轉在ball_counter內完成)
                elsif wait_cnt >= HIT_WINDOW_CYCLES - 1 then
                    next_state <= S_POINT;                 -- 漏接
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
                    next_state <= S_MOVING;                -- 球要進場了，從pos=7出現
                end if;
                -- score_rx在這裡不需要換狀態，只需要加分(Process 6)
        end case;
    end process;

    leaving_to_away <= '1' when (state = S_MOVING and moved_since_entry = '1'
                                  and at_right = '1' and game_tick = '1') else '0';
    -- 多等一個完整game_tick才真正離開S_MOVING(而不是一到at_right就立刻切走)，
    -- 是為了讓LED7至少亮滿一個tick週期，跟pos1~6每個位置一樣看得到；
    -- state暫存器每個clk都會更新，若不等game_tick，LED7只會亮1個clk(10ns)
    -- 就被S_AWAY的熄燈邏輯蓋掉，肉眼跟示波器都抓不到。
    -- 排除leaving_to_away那一拍是避免ball_counter在交接的同一拍又多走一步
    -- (從pos=7再減成pos=6)；反正handoff_rx進來時pos會被強制set回7，這步
    -- 多走一格不會被顯示出來，但排除掉比較乾淨、不留一個暫時性的錯誤值。
    ball_en <= '1' when (state = S_MOVING and game_tick = '1' and leaving_to_away = '0') else '0';

    -- 本地漏接/犯規永遠重置回「自己的pos=0」(我輸了，換我發球)；
    -- S_AWAY狀態下收到handoff_rx永遠設成pos=7(球從面網端進場，
    -- ball_counter內部會自動把方向翻成往pos=0走)
    ball_set_pos <= '1' when (state = S_POINT) or (state = S_AWAY and handoff_rx = '1') else '0';
    ball_set_val <= to_unsigned(0, 3) when state = S_POINT else
                     to_unsigned(7, 3) when (state = S_AWAY and handoff_rx = '1') else
                     (others => '0');

    handoff_tx <= leaving_to_away;
    score_tx   <= '1' when state = S_POINT else '0';

    -- ===== Process 5：本地分數，只能被外部進來的score_rx加分 =====
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

    -- ===== Process 6：球位置 -> LED0~LED7 one-hot，S_BLANK/S_AWAY時全熄 =====
    process(ball_pos, state)
        variable led_v : std_logic_vector(7 downto 0);
    begin
        led_v := (others => '0');
        if state /= S_BLANK and state /= S_AWAY then
            led_v(to_integer(ball_pos)) := '1';
        end if;
        led_out <= led_v;
    end process;

    -- ===== Process 7：七段顯示器數位切換tick =====
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

    -- ===== Process 8：BCD -> 七段解碼，只顯示這片板子自己的分數 =====
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
--2026-09-07起改用SW0 runtime決定master/slave，兩板燒同一份bit，不再需要
--per-board generic override，也不再需要pingpong9_boardA/boardB.xdc二選一。
--新的燒錄流程只需要啟用一份統一的XDC(見pingpong9_top.xdc)：
--set_property is_enabled false [get_files pingpong9_boardA.xdc]
--set_property is_enabled false [get_files pingpong9_boardB.xdc]
--set_property is_enabled false [get_files debug_boardA.xdc]
--set_property is_enabled false [get_files debug_boardB.xdc]
--set_property is_enabled true  [get_files pingpong9_top.xdc]
--(不用再set_property generic，兩板bitstream完全相同)
--
--舊版(generic-based，兩板各燒不同bit)留檔備查，若要回退：
--board A: is_enabled true pingpong9_boardA.xdc + generic {IS_I2C_MASTER=true}
--board B: is_enabled true pingpong9_boardB.xdc + generic {IS_I2C_MASTER=false}