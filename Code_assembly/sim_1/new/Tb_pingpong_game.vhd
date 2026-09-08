library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- pingpong_game.vhd的單體(單一UUT)測試平台，涵蓋：reset初始狀態、
-- 發球/擊球/漏接/犯規四種路徑、跨板handoff_tx/handoff_rx跟score_tx/score_rx
-- 的收送次數統計，全部只對「單一板子」的行為做驗證，不涉及真正的單線link實體匯流排
-- (跨板link本身的協定正確性由tb_link_wire.vhd另外驗證)。
entity tb_pingpong_game is
end tb_pingpong_game;

architecture sim of tb_pingpong_game is

    constant CLK_PERIOD : time := 10 ns;

    -- 模擬用的小型generic，加速跑測試
    constant TICK_DIVISOR      : integer := 4;
    constant DEBOUNCE_LIM      : integer := 3;
    -- 注意：HIT_WINDOW_CYCLES/BLANK_HOLD_CYCLES算的是原始clk數，跟TICK_DIVISOR
    -- 無關(沿用原本pingpong_top.vhd自己的慣例)。刻意保留比一趟移動(~7*TICK_DIVISOR
    -- 個clk)跟一次press_btn往返(~2*(DEBOUNCE_LIM+2)個clk)都還要寬裕的margin，
    -- 避免兩者互相卡到時間點造成誤判。
    constant HIT_WINDOW_CYCLES : integer := 60;
    constant BLANK_HOLD_CYCLES : integer := 15;
    constant TRAVEL_WAIT       : integer := TICK_DIVISOR * 12;  -- >7個tick，仍然遠小於HIT_WINDOW_CYCLES
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

    -- 等待n個clk上升緣
    procedure wait_clks(n : integer; signal clk : in std_logic) is
    begin
        for i in 1 to n loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

    -- 按鍵按住足夠去彈跳的時間再放開
    procedure press_btn(signal b : out std_logic; signal clk : in std_logic) is
    begin
        b <= '1';
        wait_clks(DEBOUNCE_LIM + 2, clk);
        b <= '0';
        wait_clks(DEBOUNCE_LIM + 2, clk);
    end procedure;

    -- 通用檢查程序：條件不成立就記一次錯誤並印FAIL，成立就印PASS
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

    -- ===== 受測體：單一板子的pingpong_game =====
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
            -- is_master='1'：這份testbench沿用舊版預設(START_IN_AWAY=false)的
            -- 行為，reset後直接進S_WAIT_SERVE、球在自己手上，見下面第一個check
            is_master => '1',
            led_out => led_out, seg => seg, digit_sel => digit_sel,
            score_tens => score_tens, score_ones => score_ones,
            handoff_tx => handoff_tx, handoff_rx => handoff_rx,
            score_tx => score_tx, score_rx => score_rx
        );

    clk <= not clk after CLK_PERIOD / 2;

    -- ===== 主要激勵/檢查流程 =====
    stim : process
    begin
        rst <= '1';
        wait_clks(5, clk);
        rst <= '0';
        wait_clks(2, clk);

        check(led_out = "00000001", "reset: ball resting at LED0 (pos=0)", errors);

        -- ===== 發球，球從0走到7，途中應該剛好pulse一次handoff_tx =====
        press_btn(btn_raw, clk);

        -- 0->7全程要走7個game_tick = 7*TICK_DIVISOR個clk；給寬裕margin
        wait_clks(TRAVEL_WAIT, clk);

        check(led_out = "00000000", "after handoff sent, board goes S_AWAY: LEDs blank", errors);
        check(handoff_tx_cnt = 1, "handoff_tx pulsed exactly once during 0->7 travel", errors);

        -- ===== 模擬收到對面板子送來的handoff =====
        handoff_rx <= '1';
        wait_clks(1, clk);
        handoff_rx <= '0';
        wait_clks(3, clk);
        check(led_out = "10000000", "handoff_rx received: ball re-enters at pos7 (LED7)", errors);

        -- 讓球從7走回0
        wait_clks(TRAVEL_WAIT, clk);
        check(led_out = "00000001", "after inbound travel, ball reached pos0, waiting for hit", errors);

        -- ===== 成功接到：在時間窗到期前按下 =====
        press_btn(btn_raw, clk);

        -- 讓球再度0->7出去，確認第二次handoff_tx最終有觸發
        wait_clks(TRAVEL_WAIT, clk);
        check(led_out = "00000000", "after bounce-back reaches pos7 again, handed off, LEDs blank (S_AWAY)", errors);

        -- ===== 接下來測漏接：再收一次handoff，讓時間窗過期 =====
        handoff_rx <= '1';
        wait_clks(1, clk);
        handoff_rx <= '0';
        wait_clks(TRAVEL_WAIT, clk);   -- 7->0走一趟
        check(led_out = "00000001", "second inbound arrival at pos0", errors);

        -- 故意不按，讓HIT_WINDOW_CYCLES個game_tick過完
        wait_clks(HIT_WINDOW_CYCLES + 5, clk);
        wait_clks(BLANK_HOLD_CYCLES + 10, clk);
        check(led_out = "00000001", "after miss: blanked then back to WAIT_SERVE at own pos0", errors);
        check(score_tens = 0 and score_ones = 0, "a LOCAL miss must NOT increment my own score", errors);

        -- ===== 收到外部score_rx(對面板子告訴我：我得分了) =====
        score_rx <= '1';
        wait_clks(1, clk);
        score_rx <= '0';
        wait_clks(3, clk);
        check(score_tens = 0 and score_ones = 1, "score_rx bumps my local score to 1", errors);

        -- ===== 犯規測試：要先真的走到S_AWAY(發球、球出去、真的被交接掉)，
        -- 之後收到handoff_rx才符合真實協定的時序 =====
        press_btn(btn_raw, clk);              -- 上一球漏接後換我發球
        wait_clks(TRAVEL_WAIT, clk);          -- 0->7走完，交接出去，此刻真的是S_AWAY

        handoff_rx <= '1';                    -- 對面板子把球彈回來
        wait_clks(1, clk);
        handoff_rx <= '0';
        wait_clks(TICK_DIVISOR * 2, clk);     -- 走幾步inbound，但還沒到pos0

        press_btn(btn_raw, clk);              -- 犯規：方向還是0(球還沒到)就搶按
        wait_clks(BLANK_HOLD_CYCLES + 10, clk);
        check(led_out = "00000001", "after foul: also resets to own pos0, WAIT_SERVE", errors);

        wait_clks(10, clk);
        check(handoff_tx_cnt = 3, "exactly 3 handoff_tx pulses total (3 outbound crossings)", errors);
        check(score_tx_cnt = 2, "exactly 2 score_tx pulses total (1 timeout miss + 1 foul)", errors);

        report "=== TESTBENCH DONE, total errors: " & integer'image(errors) & " ===";
        wait;
    end process;

    -- ===== 併行脈波計數器，跟stim流程各自獨立計時，不受stim等待影響 =====
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
