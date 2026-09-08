library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- link_wire.vhd(單線協定版)的協定層測試平台：master跟slave兩個實體真的接在
-- 同一條link_wire上，用兩個「刻意不同步」的100MHz clock分別驅動，模擬兩片
-- 實體板子各自跑自己的震盪器、彼此沒有phase-lock的真實情況，藉此驗證
-- slave端的2-FF同步器、還有PWM脈寬編碼取樣時機的設計是否真的夠用。
entity tb_link_wire is
end tb_link_wire;

architecture sim of tb_link_wire is

    constant CLK_PERIOD : time := 10 ns;   -- 100MHz

    -- 模擬加速用。原本設5MHz(bit slot=20個clk，LOW_SHORT=5個clk)，實測發現
    -- LOW_SHORT(5)小於link_wire.vhd裡GLITCH_FILTER_CYCLES(8，寫死常數、不隨
    -- BIT_CYCLES縮放)，代表bit='1'的短低脈衝撐不到濾波器確認門檻，接收端完全
    -- 抓不到這個bit的邊緣，handoff/score永遠傳不過去——這是testbench自己加速
    -- 倍率設太快才會踩到，真實硬體用預設的200kHz(LOW_SHORT=125個clk)完全沒事。
    -- 改成1MHz：bit slot=100個clk，LOW_SHORT=25個clk，是濾波器門檻的3倍多，
    -- 足夠margin；SAMPLE_POINT落在slot中間(50個clk)也還是遠大於同步器延遲。
    constant LINK_BIT_FREQ : integer := 1_000_000;

    signal clk_m, clk_s, rst : std_logic := '0';
    signal link_wire : std_logic;

    signal m_handoff_tx, m_score_tx, m_handoff_rx, m_score_rx : std_logic := '0';
    signal s_handoff_tx, s_score_tx, s_handoff_rx, s_score_rx : std_logic := '0';
    signal m_grst_tx, m_grst_rx, s_grst_tx, s_grst_rx         : std_logic := '0';

    -- 用sticky latch鎖存，讓(比較慢的)stim流程能可靠地觀察到「活在另一個
    -- clock domain上、只維持一拍」的脈波，不會因為時機不對而漏看
    signal m_handoff_rx_latch, m_score_rx_latch, m_grst_rx_latch : std_logic := '0';
    signal s_handoff_rx_latch, s_score_rx_latch, s_grst_rx_latch : std_logic := '0';
    signal clear_latches : std_logic := '0';

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

begin

    link_wire <= 'H';   -- 弱上拉，任一方都能主動拉成強'0'

    -- ===== 受測體：一個master + 一個slave，真的共用同一條link_wire =====
    MASTER_INST : entity work.link_wire_drv
        generic map (CLK_FREQ_HZ => 100_000_000, LINK_BIT_FREQ_HZ => LINK_BIT_FREQ)
        port map (clk => clk_m, rst => rst, is_master => '1', link_wire => link_wire,
                   handoff_tx_pulse => m_handoff_tx, score_tx_pulse => m_score_tx,
                   handoff_rx_pulse => m_handoff_rx, score_rx_pulse => m_score_rx,
                   grst_tx_pulse => m_grst_tx, grst_rx_pulse => m_grst_rx);

    SLAVE_INST : entity work.link_wire_drv
        generic map (CLK_FREQ_HZ => 100_000_000, LINK_BIT_FREQ_HZ => LINK_BIT_FREQ)
        port map (clk => clk_s, rst => rst, is_master => '0', link_wire => link_wire,
                   handoff_tx_pulse => s_handoff_tx, score_tx_pulse => s_score_tx,
                   handoff_rx_pulse => s_handoff_rx, score_rx_pulse => s_score_rx,
                   grst_tx_pulse => s_grst_tx, grst_rx_pulse => s_grst_rx);

    -- 兩個各自自由跑、刻意不同步的100MHz clock：實際兩片板子彼此不會
    -- phase-lock，這正是slave端一定要有自己的2-FF同步器的原因
    clk_m <= not clk_m after CLK_PERIOD / 2;
    clk_s <= not clk_s after (CLK_PERIOD / 2) + 1 ns;

    -- ===== master端rx latch：捕捉單拍脈波，clear_latches時歸零 =====
    latch_m : process(clk_m, rst)
    begin
        if rst = '1' then
            m_handoff_rx_latch <= '0';
            m_score_rx_latch   <= '0';
            m_grst_rx_latch    <= '0';
        elsif rising_edge(clk_m) then
            if clear_latches = '1' then
                m_handoff_rx_latch <= '0';
                m_score_rx_latch   <= '0';
                m_grst_rx_latch    <= '0';
            else
                if m_handoff_rx = '1' then
                    m_handoff_rx_latch <= '1';
                end if;
                if m_score_rx = '1' then
                    m_score_rx_latch <= '1';
                end if;
                if m_grst_rx = '1' then
                    m_grst_rx_latch <= '1';
                end if;
            end if;
        end if;
    end process;

    -- ===== slave端rx latch：捕捉單拍脈波，clear_latches時歸零 =====
    latch_s : process(clk_s, rst)
    begin
        if rst = '1' then
            s_handoff_rx_latch <= '0';
            s_score_rx_latch   <= '0';
            s_grst_rx_latch    <= '0';
        elsif rising_edge(clk_s) then
            if clear_latches = '1' then
                s_handoff_rx_latch <= '0';
                s_score_rx_latch   <= '0';
                s_grst_rx_latch    <= '0';
            else
                if s_handoff_rx = '1' then
                    s_handoff_rx_latch <= '1';
                end if;
                if s_score_rx = '1' then
                    s_score_rx_latch <= '1';
                end if;
                if s_grst_rx = '1' then
                    s_grst_rx_latch <= '1';
                end if;
            end if;
        end if;
    end process;

    -- ===== 主要激勵/檢查流程 =====
    -- 注意：新的單線協定在frame開始時就把pending事件snapshot鎖進frame_byte，
    -- 送到一半不會被新事件打斷(避免PWM波形中途改變造成誤判)，代價是如果事件
    -- 剛好發生在「這一輪的snapshot點之後」，就要等到下一輪才會被送出去——
    -- 最壞情況等於多等1輪，所以下面每個檢查都預留至少2輪round trip的margin。
    stim : process
    begin
        rst <= '1';
        clear_latches <= '1';
        wait for 100 ns;
        rst <= '0';
        clear_latches <= '0';
        wait for 100 ns;

        -- ===== master自己本地的handoff事件 -> slave必須看到handoff_rx =====
        m_handoff_tx <= '1';
        wait for CLK_PERIOD;
        m_handoff_tx <= '0';
        wait for 200 us;   -- bit rate降到1/5，round trip margin跟著等比放大

        check(s_handoff_rx_latch = '1', "master handoff_tx -> slave sees handoff_rx", errors);
        check(s_score_rx_latch = '0', "...and NOT a spurious score_rx", errors);
        check(m_handoff_rx_latch = '0', "master itself must not loop its own event back", errors);
        check(m_score_rx_latch = '0', "...and master's OWN score_rx must not spuriously fire either", errors);

        clear_latches <= '1';
        wait for 100 ns;
        clear_latches <= '0';

        -- ===== slave自己本地的score事件 -> master要在後續round trip內看到score_rx =====
        -- 脈寬故意拉到3*CLK_PERIOD(30ns)：s_score_tx是外部直接餵給slave的刺激，
        -- slave走clk_s(週期跟clk_m不同、刻意不同步)，脈寬只給1個CLK_PERIOD(10ns)
        -- 曾經在某次delta-cycle排序下剛好跟clk_s的邊緣完全對齊，導致那一拍完全沒
        -- 被clk_s採到；拉寬到3倍，不管跟clk_s怎麼對齊都保證至少跨過1個完整邊緣。
        -- 真實硬體上這個訊號是同一顆clk餵的，不會有這個問題，純粹是testbench
        -- 外部雙clock domain激勵才會踩到的margin。
        s_score_tx <= '1';
        wait for 3 * CLK_PERIOD;
        s_score_tx <= '0';
        wait for 200 us;

        check(m_score_rx_latch = '1', "slave score_tx -> master sees score_rx", errors);
        check(m_handoff_rx_latch = '0', "...and NOT a spurious handoff_rx", errors);

        clear_latches <= '1';
        wait for 100 ns;
        clear_latches <= '0';

        -- 額外等待，讓上一輪score事件的ACK回音(peer_ack_score/peer_ack_handoff
        -- 這些echo欄位)完全穩定下來，不要讓它們的殘留變化跟下面「同時觸發」的
        -- 新事件疊在一起——不然frame內容會連續好幾輪都在變，永遠湊不齊「連續
        -- 兩輪內容相同」這個確認條件，導致下面的check需要遠超過200us margin。
        wait for 200 us;

        -- ===== 兩邊同時觸發，兩個方向都要能各自被看到 =====
        -- s_handoff_tx脈寬同樣拉寬到3*CLK_PERIOD，理由跟上面slave score_tx一樣
        -- margin拉到500us(單一事件的2.5倍)：handoff/score兩個bit分屬同一個
        -- frame_byte，只要任一bit的ack還沒回穩，整個byte比對就會跟著不match，
        -- 兩個事件的settle時間會互相拖累、不是單純疊加，實測需要比單一事件
        -- 多好幾輪round trip才會真正穩定。
        m_score_tx   <= '1';
        s_handoff_tx <= '1';
        wait for CLK_PERIOD;
        m_score_tx   <= '0';
        wait for 2 * CLK_PERIOD;
        s_handoff_tx <= '0';
        wait for 500 us;

        check(s_score_rx_latch = '1', "simultaneous: slave sees master's score event", errors);
        check(m_handoff_rx_latch = '1', "simultaneous: master sees slave's handoff event", errors);

        clear_latches <= '1';
        wait for 100 ns;
        clear_latches <= '0';
        wait for 200 us;   -- 讓上一輪的ack回音完全穩定，理由跟前面score/handoff那段一樣

        -- ===== 共用GRST：master本地觸發grst_tx -> slave要在後續round trip內看到grst_rx =====
        -- 這裡只測link_wire_drv本身新增的bit4/bit5協定行為(grst_tx_pulse/grst_rx_pulse)，
        -- 不包含Pingpong9_hw_top.vhd裡"收到grst_rx後延遲GRST_PROPAGATE_DELAY_CYCLES才真正
        -- 觸發自己rst"那段——那段邏輯要接到真正的rst_level才有意義，得在Tb_pingpong9_chain.vhd
        -- (真正instantiate pingpong9_hw_top)那層才測得到，這裡驗證的是"請求真的能透過link
        -- 正確送達對面、且不干擾handoff/score"這件事。
        m_grst_tx <= '1';
        wait for CLK_PERIOD;
        m_grst_tx <= '0';
        wait for 200 us;

        check(s_grst_rx_latch = '1', "master grst_tx -> slave sees grst_rx", errors);
        check(s_handoff_rx_latch = '0', "...and NOT a spurious handoff_rx", errors);
        check(s_score_rx_latch = '0', "...and NOT a spurious score_rx", errors);
        check(m_grst_rx_latch = '0', "master itself must not loop its own grst event back", errors);

        clear_latches <= '1';
        wait for 100 ns;
        clear_latches <= '0';
        wait for 200 us;

        -- ===== 反方向：slave本地觸發grst_tx -> master要看到grst_rx =====
        -- 脈寬拉到3*CLK_PERIOD，理由跟前面slave score_tx那段一樣
        -- (s_grst_tx是外部直接餵給slave的刺激，slave走clk_s，要保證跨過至少一個完整邊緣)
        s_grst_tx <= '1';
        wait for 3 * CLK_PERIOD;
        s_grst_tx <= '0';
        wait for 200 us;

        check(m_grst_rx_latch = '1', "slave grst_tx -> master sees grst_rx", errors);
        check(m_handoff_rx_latch = '0', "...and NOT a spurious handoff_rx", errors);
        check(m_score_rx_latch = '0', "...and NOT a spurious score_rx", errors);
        check(s_grst_rx_latch = '0', "slave itself must not loop its own grst event back", errors);

        report "=== TESTBENCH DONE, total errors: " & integer'image(errors) & " ===";
        wait;
    end process;

end sim;
