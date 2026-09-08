library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- 完整依賴鏈測試平台：真的實體化兩份pingpong9_hw_top(板A + 板B)，
-- 用兩個不同步的clock各自驅動，透過真正的link_wire單線協定互相交握，
-- 驗證球在兩片真實板子之間來回交接、以及漏接後計分的端到端行為。
-- BCD分數怎麼從score_rx脈波累加、跟score_tx/score_rx脈波怎麼透過單線link
-- 傳遞，各自的協定細節已經分別由tb_pingpong_game.vhd跟tb_link_wire.vhd
-- 單獨驗證過；這份testbench要驗證的重點，是兩個「真實板子等級」的實體之間，
-- 球真的能正確地來回交接：A發球、B穩穩接住彈回去、A這次漏接，
-- 漏接後A透過link_wire的score_tx/score_rx讓B被加1分(兩板都從0開始)。
entity tb_pingpong9_chain is
end tb_pingpong9_chain;

architecture sim of tb_pingpong9_chain is

    constant CLK_PERIOD : time := 10 ns;

    -- 模擬用的小型generic；HIT_WINDOW/BLANK_HOLD算的是原始clk。
    constant TICK_DIVISOR      : integer := 4;
    constant DEBOUNCE_LIM      : integer := 3;
    -- HIT_WINDOW_CYCLES刻意跟tb_pingpong_game.vhd(單板、無跨板延遲)不一樣：
    -- 那邊用100(=1us)完全沒問題，因為按鍵緊接著球到就馬上按。這裡是跨板情境，
    -- 球從A送出到B真的收到，實測透過link_wire大約要180us(單線協定round trip
    -- 數輪)，如果HIT_WINDOW_CYCLES太短，B自己的等待窗口會在testbench的
    -- "wait for 750 us"margin跑完前就先逾時漏接了——LED剛好又跟"還在等擊球"
    -- 長得一樣(都是pos0=00000001)，之前完全沒被LED check抓到，卻真的觸發了
    -- score_tx，讓A被誤加分。40_000(=400us)對實測180us留了約2.2倍margin，
    -- 比原本抓的80_000(800us，4.4倍)緊一半，模擬波形會好觀察許多，同時仍然
    -- 安全撐得過實際跨板延遲。這個值沒辦法再跟著TICK_DIVISOR等比例縮小——
    -- 它被link_wire協定本身的實際傳輸時間卡住，跟TICK_DIVISOR(本地球移動速度)
    -- 是兩件不相干的事，詳見LINK_BIT_FREQ的說明。
    constant HIT_WINDOW_CYCLES : integer := 40_000;
    constant BLANK_HOLD_CYCLES : integer := 20;
    constant SCAN_DIVISOR      : integer := 10;
    constant LINK_BIT_FREQ     : integer := 1_000_000;   -- 模擬加速用，跟tb_link_wire.vhd
                                                            -- 用同一個數值(原本5MHz會讓
                                                            -- LOW_SHORT小於link_wire.vhd的
                                                            -- GLITCH_FILTER_CYCLES=8，bit='1'
                                                            -- 的邊緣會被濾波器濾掉，詳見
                                                            -- tb_link_wire.vhd裡的說明)。
                                                            -- 1MHz已經接近安全上限，不建議再往上調：
                                                            -- LOW_SHORT=BIT_CYCLES/4，要維持對
                                                            -- GLITCH_FILTER_CYCLES=8有安全margin
                                                            -- (目前3倍)，理論極限約1.04MHz，再快
                                                            -- margin會變太緊，等於走回頭路踩那個
                                                            -- 已經修好的bug。
    constant TRAVEL_WAIT       : integer := TICK_DIVISOR * 12;   -- 一趟移動，>7個tick

    -- 共用GRST：對面收到grst請求後，要等這麼久(以clk計)才真的觸發自己的reset，
    -- 確保自己的ack有機會先透過至少一輪完整frame送出去(詳見Pingpong9_hw_top.vhd
    -- 該generic的說明)。真實硬體預設100ms(10_000_000 cycles)，模擬時大幅縮小；
    -- 但不能縮得比一輪完整link round trip還短(在LINK_BIT_FREQ=1MHz下，一輪
    -- master TX+gap+slave TX+gap約24us=2400clk)，否則會在對面ack真的送出去
    -- 之前就先reset、重蹈設計時要避免的"雞生蛋蛋生雞"問題——8000clk(=80us)
    -- 留了超過3倍margin，跟本檔案其他timing margin抓的比例一致。
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

    -- 等待n個clk上升緣
    procedure wait_clks(n : integer; signal clk : in std_logic) is
    begin
        for i in 1 to n loop
            wait until rising_edge(clk);
        end loop;
    end procedure;

    -- 板子自己的GRST是靠去彈跳後的按下觸發；板子開機時透過在t=0短暫拉高
    -- btn_GRST來重置，跟真實硬體開機的行為一致
    procedure press(signal b : out std_logic; signal clk : in std_logic) is
    begin
        b <= '1';
        wait_clks(DEBOUNCE_LIM + 10, clk);
        b <= '0';
        wait_clks(DEBOUNCE_LIM + 10, clk);
    end procedure;

begin

    seg_scl_a <= 'H'; seg_sda_a <= 'H';   -- 這裡沒有真正的TCA6416模型；本地
    seg_scl_b <= 'H'; seg_sda_b <= 'H';   -- 七段匯流排不是這份testbench要驗證的重點

    link_wire <= 'H';

    -- ===== 板A：I2C master，開機時球在自己手上(S_WAIT_SERVE) =====
    -- IS_I2C_MASTER generic移除了，master/slave改成靠sw_master(runtime port)
    -- 決定；testbench裡直接接常數'1'/'0'模擬SW0撥定的位置，效果跟舊版一樣，
    -- 只是角色要等第一次press(grst_*)過後(下面stim process開頭就會做)才會
    -- 真的鎖進is_master_latched。
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

    -- ===== 板B：I2C slave，開機時球不在自己手上(S_AWAY) =====
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

    -- 兩片板子、兩顆各自獨立的震盪器：彼此不會phase-lock
    clk_a <= not clk_a after CLK_PERIOD / 2;
    clk_b <= not clk_b after (CLK_PERIOD / 2) + 1 ns;

    -- ===== 主要激勵/檢查流程 =====
    stim : process
    begin
        -- 開機重置：兩片板子各自按一次GRST
        press(grst_a, clk_a);
        press(grst_b, clk_b);
        wait_clks(5, clk_a);

        -- 兩板"各自"按GRST，代表兩邊都會各自觸發一次shared-GRST relay通知
        -- 對面(見Pingpong9_hw_top.vhd的grst_tx_i產生邏輯：rst_level放開的那一刻
        -- 就送)。如果沒有這段settle wait，直接接著往下跑遊戲流程，大約
        -- 200us後(round trip+GRST_PROPAGATE_DELAY_CYCLES=8000clk才會真的觸發)
        -- 這兩個relay請求才會真的抵達對面、把對面board"又重置一次"，剛好蓋掉
        -- 這段時間內已經成功建立的遊戲狀態(例如已經收到的handoff)——2026-09-07
        -- 用get_value直接probe到這個現象：B成功收到handoff、state正確進了
        -- S_WAIT_HIT，過沒多久卻又被打回S_AWAY，就是這個關掉的relay在作祟。
        -- 這裡先等到兩邊的relay都真的觸發完、settle下來，再開始真正的遊戲
        -- 流程，避免後面的handoff/score check被這個「遲到的第二次reset」打斷。
        wait for 400 us;

        check(led_a = "00000001", "board A reset: ball at its own pos0", errors);
        check(led_b = "00000000", "board B reset: idle, no ball (S_AWAY, LEDs blank)", errors);

        -- ===== A發球 =====
        press(btn_a, clk_a);
        wait_clks(TRAVEL_WAIT, clk_a);
        check(led_a = "00000000", "A: ball left toward net, A now blank (S_AWAY)", errors);

        -- 給link足夠時間把handoff傳過去(單線協定master/slave一輪round trip)，
        -- 再讓B走完自己那一段的移動。這個等待時間有上下限夾在中間，不能隨便choose：
        -- 下限要大於實測的link傳輸時間(~180us)，確保check時ball真的已經到了；
        -- 上限則絕對不能超過HIT_WINDOW_CYCLES換算的實際時間(40_000*12ns=480us)，
        -- 否則B會在testbench按下btn_b之前就自己先逾時漏接(S_WAIT_HIT->S_WAIT_SERVE)
        -- ——這正是原本的bug：舊版用750us等待，遠超過480us的視窗，B早就自己漏接了，
        -- 但led_out在S_WAIT_HIT和S_WAIT_SERVE時剛好都是pos0=00000001，check照樣PASS，
        -- 完全沒被抓到。300us上下都留了~120us~180us的margin，兩邊都安全。
        wait for 300 us;
        wait_clks(TRAVEL_WAIT, clk_b);
        -- B是slave(is_master_latched='0')，pingpong9_hw_top.vhd的led_mux_gen會把
        -- pingpong_game算出來的led_out_i(邏輯順序，bit0=自己這端)左右反過來才送
        -- 到led_out(對應鏡射擺放的實體LED順序)，所以B板pos0在led_out上看到的是
        -- bit7、不是bit0——跟board A(master，不反接)"00000001"故意不同。
        check(led_b = "10000000", "B: ball reached B's own pos0, waiting for B's hit", errors);

        -- ===== B及時接到，彈回去 =====
        press(btn_b, clk_b);
        wait_clks(TRAVEL_WAIT, clk_b);
        check(led_b = "00000000", "B: bounced back out, handed off again, B now blank", errors);

        -- 這個check是曾經真正抓到bug的關鍵：如果B的HIT_WINDOW太短、在testbench
        -- 按下btn_b之前就自己先逾時漏接一輪，A會在這裡就已經被誤加分——雖然LED
        -- 序列看起來完全正常(漏接後LED剛好又跟"等擊球"長得一樣)。刻意在B成功
        -- 回擊之後、A自己那次故意漏接之前，就先檢查A分數還是0，不用等到最後才發現。
        check(score_tens_a = 0 and score_ones_a = 0,
              "A: score still 0 right after B's successful hit (no spurious miss on B's side)", errors);

        -- 同樣的道理，這裡等的是A自己的HIT_WINDOW_CYCLES視窗，理由跟上面B那段
        -- 完全對稱：300us > 實測傳輸時間、< 480us視窗上限
        wait for 300 us;
        wait_clks(TRAVEL_WAIT, clk_a);
        check(led_a = "00000001", "A: ball reached A's own pos0 again, waiting for A's hit", errors);

        -- ===== 這次A漏接(故意讓時間窗過期) =====
        wait_clks(HIT_WINDOW_CYCLES + 10, clk_a);
        wait_clks(BLANK_HOLD_CYCLES + 10, clk_a);
        check(led_a = "00000001", "A: after miss, blanked then back to A's own WAIT_SERVE", errors);

        -- 給score_tx(A)->score_rx(B)足夠的link round trip時間(理由同handoff：
        -- 單線協定frame要連續兩輪內容一致才算數，不能只等一輪)，
        -- 確認A的漏接真的透過link_wire讓B被加分——兩板分數都是從0開始。
        -- 這裡沒有上面那種HIT_WINDOW上限顧慮(後面沒人在等視窗)，但一併縮短
        -- 到跟其他等待一致，維持整份testbench的觀察尺度統一
        wait for 300 us;
        check(score_tens_b = 0 and score_ones_b = 1,
              "B: score bumped to 1 after A's miss (score_tx/score_rx via real link_wire)", errors);

        -- 計分邏輯本身(score_rx脈波 -> BCD計數器)已經由tb_pingpong_game.vhd
        -- 單獨徹底驗證過，score_tx/score_rx脈波透過真正link的傳遞路徑也已經
        -- 由tb_link_wire.vhd驗證過；上面這次score check則是在兩片「真實板子等級」
        -- 的實體之間，端到端地確認同一套機制真的走得通，不是只在各自獨立的
        -- 子測試裡驗證過就算數。

        -- ===== 共用GRST：只按A的GRST，驗證B也會透過link_wire的grst請求機制被重置 =====
        -- 這裡故意"只"按grst_a、完全不碰grst_b，排除"B自己被獨立按了GRST"這個
        -- 混淆因素——如果B真的被重置，唯一可能的來源就是A透過link_wire傳過去的
        -- grst請求。用score_ones_b(目前=1，上面那段score測試留下來的)必須被清回0
        -- 當判斷依據，比看led_out可靠很多：led_out在很多狀態下都長一樣
        -- (S_WAIT_HIT/S_WAIT_SERVE等常常都是同一個pattern，這份專案已經因為這個
        -- aliasing踩過雷)，但score只有"真的被rst"才會歸零，正常遊戲事件不可能
        -- 讓分數無中生有地減少，是這裡唯一乾淨、不會跟其他狀態混淆的證據。
        press(grst_a, clk_a);

        -- 300us：跟前面handoff/score檢查同一個link round trip margin，
        -- 讓A的grst請求真的透過link_wire傳到B；GRST_PROPAGATE_DELAY_CYCLES+1000：
        -- 讓B收到請求後自己的延遲計時器(以clk_b計)也跑完，兩段依序疊加，
        -- 保證check的時候B該做的事都已經做完了
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
