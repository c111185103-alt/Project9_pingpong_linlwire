library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Project 9最外層的硬體包裝層，代表「一片板子」的完整電路。
-- 延伸自pingpong_hw_top.vhd(Project 5/6)，改成雙板分離拓樸。
--
-- ===== master/slave改為runtime決定(2026-09-07) =====
-- 兩片板子燒「完全相同」的.bit檔，不再靠generic分兩份build。這片板子這次要
-- 扮演master還是slave，是靠SW0這顆實體開關決定：開機/按GRST放開的那一刻，
-- 讀SW0目前的位置鎖存進is_master_latched，之後遊戲進行中就算SW0被碰到也不會
-- 再變動(要重新按一次GRST才會重新讀取)。is_master_latched再往下餵給U_GAME(決定
-- 開機球在哪一端)、U_LINK_WIRE(決定跑master_gen那份邏輯還是slave_gen那份)、
-- 以及LED輸出mux(決定LED接腳順序要不要反接，見下面說明)。
--
-- 操作流程：兩板燒同一份bit -> 各自撥SW0(一ON一OFF) -> 按其中一板的GRST即可
-- (shared-GRST relay會在~GRST_PROPAGATE_DELAY_CYCLES之後讓對面板子也自動
-- 重置、順便鎖存它自己的SW0位置，不用兩板都手動按)。
--
-- 每片板子同時存在「兩條完全獨立」的匯流排：
--   seg_scl/seg_sda  -> 這片板子自己專屬的TCA6416(U11/U12，接腳沿用Project5/6不變)，
--                        真正的I2C，驅動自己的兩位數七段顯示器。
--   link_wire        -> 全新的點對點單線協定，接到對面板子，走Raspberry Pi
--                        GPIO2(Y5)一根腳(JA2排針第3腳)。教授規定兩板之間只能拉
--                        2條線(含共地線)，兩板又是各自獨立供電、沒有共同回路，
--                        所以GND是必要的第2條，因此link這邊只留1條訊號線
--                        (link_wire.vhd內部改成自訂的單線協定，不再是I2C)。
--                        GND走JA2排針第6腳(標準Raspberry Pi接頭的GND腳，跟
--                        GPIO2實體上相鄰)，不是借用GPIO3(W5)那根腳——GPIO3
--                        本身是GPIO腳、不是地，硬把它當GND接會有風險，所以
--                        單純空接不用。
-- seg_scl/seg_sda這兩條絕對不能在schematic或XDC裡跟link_wire接在一起——
-- i2c_tca6416_driver.vhd只跟本地TCA6416對話，link_wire.vhd只跟對面板子對話。
--
-- ===== LED接腳為什麼要mux(2026-09-07新增) =====
-- 兩片板子實際擺在桌上是鏡射的(面對面對打)，所以「自己這端pos=0」在兩板上
-- 對應到的實體LED位置左右相反；舊版(generic分開build)靠兩份不同的XDC直接把
-- led_out(0..7)接到相反順序的實體腳位解決。現在兩板共用同一份XDC(同一組
-- 固定實體接腳)，這個「要不要反過來接」改成在這裡用is_master_latched即時
-- mux：沿用舊convention，master那一板維持正序、slave那一板輸出反序。也就是
-- 說操作習慣不變——哪一板你撥成master，就把它擺在原本"板A"的正常方向；
-- 撥成slave的那板照舊擺在鏡射的那一側。
entity pingpong9_hw_top is
    generic (
        -- 硬體預設值，跟pingpong_hw_top.vhd(P5/6)使用的數值相同
        TICK_DIVISOR      : integer := 25_000_000;
        DEBOUNCE_LIM      : integer := 1_000_000;
        HIT_WINDOW_CYCLES : integer := 50_000_000;
        BLANK_HOLD_CYCLES : integer := 100_000_000;
        SCAN_DIVISOR      : integer := 200_000;
        GRST_DEBOUNCE_LIM : integer := 1_000_000;   -- 特地跟DEBOUNCE_LIM分開：GRST在
                                                       -- pingpong_hw_top.vhd(P5/6)裡原本是
                                                       -- 寫死的數字，沒有接自己的generic，
                                                       -- 這裡也開放出來，讓模擬可以用同樣的方式override

        LINK_BIT_FREQ_HZ : integer := 200_000;   -- 單線協定的位元速率，相對0.5秒的hit window還有充裕餘裕

        -- 共用GRST：收到對面的grst請求後，要等這麼久才真的觸發本地reset，
        -- 確保自己的ack(bit5)有機會先透過至少一輪完整frame送出去，不然一收到
        -- 就馬上reset，會把還沒送出去的ack狀態一起清空，對面永遠收不到確認、
        -- 無限重送。100ms在100MHz下綽綽有餘(一輪frame實際只需要幾十~幾百us)，
        -- 模擬時建議override成小數值
        GRST_PROPAGATE_DELAY_CYCLES : integer := 10_000_000
    );
    port (
        clk      : in  std_logic;                     -- Y9,  GCLK 100MHz
        btn_GRST : in  std_logic;                      -- P16, BTNC (S6)，板子重置鍵
        btn_raw  : in  std_logic;                       -- 這片板子自己的擊球/發球按鍵(S8/BTNR，兩板統一)
        sw_master : in std_logic;                       -- SW0，決定這片板子這次是master('1')還是slave('0')
        led_out  : out std_logic_vector(7 downto 0);    -- LED0~LED7

        -- debug/模擬觀察用：真實硬體上分數是走seg_scl/seg_sda(I2C)顯示，
        -- 這兩個port真實硬體上接JA2排針上未使用的spare GPIO腳位(不影響任何
        -- 功能，純粹讓示波器/邏輯分析儀或testbench能直接看到這片板子目前的
        -- BCD分數，不用再靠seg解碼反推——接腳配置見pingpong9_top.xdc)。
        -- 2026-09-07一度嘗試改成不接頂層port、testbench改用VHDL-2008 external
        -- name直接probe內部訊號，但external name觸發了Vivado 2018.2 xsim的
        -- kernel FATAL_ERROR崩潰(對component instantiation底下的unsigned訊號
        -- 做external name，這個版本的xsim有已知問題)，改回原本的port直連
        -- 寫法，靠XDC給這兩個port分配JA2排針上真正的spare pin解決IO
        -- placement infeasible的問題。
        score_tens : out unsigned(3 downto 0);
        score_ones : out unsigned(3 downto 0);

        seg_scl  : out std_logic;                       -- U11，本地七段顯示器I2C(專用，沿用P5/6不變)
        seg_sda  : inout std_logic;                      -- U12

        link_wire : inout std_logic                      -- Raspberry Pi GPIO2 (Y5)，跨板link唯一的訊號線；
                                                            -- GND另外用實體線接兩板，不算在這個port裡
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
    signal combined_rst : std_logic;   -- 本地GRST OR 對面轉過來的grst請求，餵給U_GAME/U_LINK_WIRE的真正rst

    signal seg_i         : std_logic_vector(6 downto 0);
    signal digit_sel_i   : std_logic;

    signal handoff_tx_i, handoff_rx_i : std_logic;
    signal score_tx_i, score_rx_i     : std_logic;
    signal grst_tx_i, grst_rx_i       : std_logic;

    -- ===== SW0 -> is_master_latched：debounce過的SW0，只在combined_rst='1'
    -- 期間持續跟隨SW0、放開後就定格鎖住，遊戲進行中不會再被SW0後續變化影響 =====
    signal sw_master_db     : std_logic;
    signal is_master_latched : std_logic := '1';   -- 預設值只在GSR那一瞬間有意義，實際角色要等第一次combined_rst才會鎖定

    -- U_GAME算出來的LED pattern(邏輯順序：0=自己這端，7=面網端)，muxed成
    -- led_out實際接腳順序前的中繼訊號，見架構最後的led_mux_gen
    signal led_out_i : std_logic_vector(7 downto 0);

    -- ===== 共用GRST：偵測rst_level"放開"的邊緣、以及收到對面grst請求後的延遲計時 =====
    -- 這個process刻意不接rst_level(或combined_rst)當自己的同步reset——它的任務
    -- 就是要在rst_level本身發生變化時做出反應，若也被同一個訊號歸零，使用者一放開
    -- GRST的那個瞬間就會被自己清空，永遠抓不到那個edge；grst_delay_run/grst_delay_cnt
    -- 同理也不能被combined_rst打斷，不然對面剛好也在這段延遲期間觸發我，會被重新清零。
    signal rst_level_prev  : std_logic := '0';
    signal grst_delay_run  : std_logic := '0';
    signal grst_delay_cnt  : integer range 0 to GRST_PROPAGATE_DELAY_CYCLES - 1 := 0;
    signal grst_reset_now  : std_logic := '0';

begin

    -- GRST去彈跳的輸出直接接pingpong_game的rst；去彈跳「自己」在settling期間
    -- 不會被reset鎖住(跟pingpong_hw_top.vhd同一套理由：開機當下按鍵原始電位還不穩定，
    -- 所以只有debounce的「輸出」拿去做全域reset，debounce電路本身不能被reset卡住)
    U_DB_GRST : debounce
        generic map (DEBOUNCE_LIMIT => GRST_DEBOUNCE_LIM)
        port map (clk => clk, rst => '0', btn_in => btn_GRST, btn_out => open, btn_level => rst_level);

    -- SW0同樣先去彈跳，理由跟GRST一樣：開機當下開關/接點電位還不穩定，
    -- debounce電路本身不能被combined_rst卡住
    U_DB_SWMASTER : debounce
        generic map (DEBOUNCE_LIMIT => DEBOUNCE_LIM)
        port map (clk => clk, rst => '0', btn_in => sw_master, btn_out => open, btn_level => sw_master_db);

    combined_rst <= rst_level or grst_reset_now;

    -- ===== is_master鎖存：combined_rst='1'期間(本地GRST按著，或收到對面
    -- shared-GRST relay過來那一拍)持續跟隨SW0，一放開就定格鎖住，遊戲進行
    -- 期間SW0再怎麼動也不會被讀到，要重新按GRST才會重新鎖存一次 =====
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
            -- rst_level"放開"(reset結束)的下一拍，脈衝一次grst_tx_i，通知對面
            -- 我剛被本地GRST重置過；按下的那一瞬間不能用，因為link_wire_drv
            -- 這時候自己也還在rst，pending_grst設了也會被同一個reset蓋掉
            grst_tx_i <= rst_level_prev and not rst_level;

            if grst_rx_i = '1' then
                grst_delay_run <= '1';
                grst_delay_cnt <= 0;
                grst_reset_now <= '0';
            elsif grst_delay_run = '1' then
                if grst_delay_cnt = GRST_PROPAGATE_DELAY_CYCLES - 1 then
                    grst_delay_run <= '0';
                    grst_reset_now <= '1';   -- 延遲夠久了(對面的ack應該已經送出去過至少一輪)，現在才真的觸發本地reset
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

    -- ===== LED輸出mux：master正序、slave反序，取代舊版靠兩份XDC反接的做法 =====
    led_mux_gen : for i in 0 to 7 generate
        led_out(i) <= led_out_i(i) when is_master_latched = '1' else led_out_i(7 - i);
    end generate led_mux_gen;

end Structural;
