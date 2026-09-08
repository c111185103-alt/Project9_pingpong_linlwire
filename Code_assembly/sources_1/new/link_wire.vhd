library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Project 9的跨板link。跟i2c_tca6416_driver.vhd(那顆維持原樣不動，
-- 還是走U11/U12專用接腳、真正的I2C，驅動本地七段顯示器)是完全不同、獨立的匯流排。
--
-- *** 這裡不是I2C ***：教授規定兩片板子之間只能拉「兩條線」溝通，而且共地線
-- (如果需要)也要算在這兩條裡面。兩片板子各自獨立變壓器供電、彼此沒有共同回路，
-- 所以GND這條線是必要的——「1條訊號線 + 1條GND線」正好2條，因此這裡改成
-- 自己設計的「單線協定」，只用JA2的GPIO2(Y5)這一根腳當訊號線(GPIO3/W5不用了，
-- 空出來的實體線改拉GND)。
--
-- ===== 位元編碼：脈寬調變(PWM)，每個bit「各自」用自己的下降緣自我定時 =====
-- 傳送端在每個固定長度的time slot開頭把線拉低，bit=0拉低久(LOW_LONG)，
-- bit=1拉低短(LOW_SHORT)，然後放開(釋放)到這個slot結束——每個slot開頭都一定
-- 會重新拉低一次(不管上一個bit是什麼值)，所以每個bit邊界都保證有一個乾淨的
-- 下降緣可以抓。接收端「每收一個bit」都重新找一次下降緣、從那一刻開始倒數，
-- 數到slot中間的SAMPLE_POINT時去看線的電位(bit=0的話線還在低，bit=1的話線
-- 已經放開變高)，藉此分辨0/1，收完就丟掉這個時間基準、等下一個bit自己的邊緣。
-- 這跟紅外線遙控器NEC協定、DHT11溫濕度感測器的編碼方式是同一種原理。
--
-- 這裡刻意「每個bit都重新抓邊緣」、而不是「只在frame開頭抓一次edge、之後全部
-- 用同一個時間基準往後推」——因為兩片板子各自跑自己的震盪器，clock頻率不會
-- 完全一樣(即使標稱都是100MHz，實際上也會有一點誤差)，如果整個frame(9個slot)
-- 都靠同一個起點往後累加，誤差會越滾越大，到最後幾個bit可能已經取樣到完全
-- 錯的時間點；改成每個bit各自抓自己的邊緣，誤差不會累積，只需要在「一個bit
-- slot」的時間內兩邊走時不要差太多即可，容忍度大很多。
--
-- ===== Frame格式 =====
-- [START：拉低START_CYCLES，比任何資料bit的拉低時間都長，接收端不靠它算時間，
--  只是拿來確認「有一輪新的傳輸開始了」] + [START結束後強制放開一段
--  START_GUARD_CYCLES，確保START跟bit0之間一定有一個乾淨的邊緣可以抓，
--  不會因為bit0剛好也要拉低而跟START的低電位黏在一起分不出邊界]
-- + [8個資料bit，bit0=handoff，bit1=score，bit2=ack_handoff，bit3=ack_score，
--    bit4=pending_grst(對面剛被本地GRST重置，通知你也重置一下)，
--    bit5=ack_grst，bit6~7永遠是0，沒有用到]
-- 單線是point-to-point(只有這兩片板子接在一起)，不需要像I2C那樣傳位址byte。
--
-- ===== Master/Slave輪流講話 =====
-- master送完自己的frame -> 放開線、轉成聽的角色，等對面(slave)的START；
-- slave收完master的frame -> 短暫延遲(換手時間) -> 換它送自己的frame回去；
-- slave送完 -> 回到聽的角色，等下一輪master的START。如此不斷循環，
-- 等同於原本I2C版本「有事先write、再read對面」的輪詢邏輯，只是換了物理層。
-- master在等對面START時、以及接收過程中若某個bit的邊緣遲遲等不到，都會逾時
-- 放棄、回去重新開始，避免對面板子還沒開機/還沒放開重置時卡死；slave同樣在
-- 接收中途逾時的話，就放棄這次接收、回去重新等下一次乾淨的START，不會卡死。
--
-- ===== master/slave角色改為runtime決定(2026-09-07) =====
-- 原本IS_MASTER是generic，兩板燒不同的bitstream；改成is_master這個runtime
-- input port之後，兩板燒「完全相同」的.bit檔，角色由上層(pingpong9_hw_top)
-- 依SW0開關鎖存後餵進來。因為generic只能在編譯期決定，master/slave兩份
-- 邏輯不能再用「if IS_MASTER generate」二選一合成，改成兩份都無條件合成
-- (用block，不是generate if)，永遠都在跑，只是輸出(drive0/handoff_rx_pulse/
-- score_rx_pulse/grst_rx_pulse)由is_master在最後mux一次，決定哪一份的結果
-- 才真正生效。沒被選中的那份邏輯仍會照常運作(讀真實的link_wire、算自己的
-- 內部狀態)，只是運算結果被mux丟棄，不會造成任何錯誤，純粹多耗一點點資源。
entity link_wire_drv is
    generic (
        CLK_FREQ_HZ      : integer := 100_000_000;
        LINK_BIT_FREQ_HZ : integer := 200_000   -- 單線協定的位元速率(決定每個time slot多長)
    );
    port (
        clk : in std_logic;
        rst : in std_logic;

        -- runtime決定這片板子這次要扮演master還是slave：'1'=master、'0'=slave。
        -- 由上層依SW0開關(鎖存過)餵進來，取代原本的IS_MASTER generic。
        is_master : in std_logic;

        link_wire : inout std_logic;   -- 唯一的訊號線(JA2 GPIO2/Y5)；GND另外用實體線接兩板，不算在這個port裡

        handoff_tx_pulse : in  std_logic;
        score_tx_pulse   : in  std_logic;
        handoff_rx_pulse : out std_logic;
        score_rx_pulse   : out std_logic;

        -- 共用GRST廣播用：grst_tx_pulse由上層在本地rst_level"放開"後脈衝一次
        -- (不是按下的那一刻——按下時這個模組自己也還在rst，設了也會被同一個reset蓋掉)，
        -- grst_rx_pulse則是收到對面的請求時脈衝一次，上層收到後要"延遲"一段時間
        -- 才能真的觸發自己的rst，理由見上面entity開頭的說明
        grst_tx_pulse    : in  std_logic;
        grst_rx_pulse    : out std_logic
    );
end link_wire_drv;

architecture Behavioral of link_wire_drv is

    -- master_blk/slave_blk各自算出自己的drive0/rx脈波，這裡再依is_master
    -- mux一次才是真正的輸出/驅動這條實體線
    signal drive0_m, drive0_s, drive0 : std_logic := '0';
    signal handoff_rx_m, score_rx_m, grst_rx_m : std_logic := '0';
    signal handoff_rx_s, score_rx_s, grst_rx_s : std_logic := '0';

    -- Debug用：這個是master_blk/slave_blk共用的top-level訊號，兩板都能直接看到
    -- 「FPGA本身現在到底有沒有主動拉低這條線」，不用進到block區塊裡面找
    attribute mark_debug : string;
    attribute mark_debug of drive0     : signal is "true";
    attribute mark_debug of is_master  : signal is "true";
    -- attribute specification必須跟訊號宣告在同一個declarative region，
    -- handoff_rx_m/s等訊號雖然在邏輯上"屬於"master_blk/slave_blk，但因為
    -- 兩份角色的結果要在架構層級mux，訊號本身宣告在這裡，debug屬性也要跟著放這裡
    attribute mark_debug of handoff_rx_m : signal is "true";
    attribute mark_debug of score_rx_m   : signal is "true";
    attribute mark_debug of grst_rx_m    : signal is "true";
    attribute mark_debug of handoff_rx_s : signal is "true";
    attribute mark_debug of score_rx_s   : signal is "true";
    attribute mark_debug of grst_rx_s    : signal is "true";

    -- ===== 協定時間參數，全部從LINK_BIT_FREQ_HZ推導出來 =====
    constant BIT_CYCLES         : integer := CLK_FREQ_HZ / LINK_BIT_FREQ_HZ;  -- 一個time slot的clk數
    constant LOW_SHORT          : integer := BIT_CYCLES / 4;        -- bit='1'：拉低這麼久
    constant LOW_LONG           : integer := (BIT_CYCLES * 3) / 4;  -- bit='0'：拉低這麼久
    constant SAMPLE_POINT       : integer := BIT_CYCLES / 2;        -- 接收端在slot內的取樣時機(在LOW_SHORT/LOW_LONG中間，兩邊都留有餘裕)
    constant START_CYCLES       : integer := BIT_CYCLES * 2;        -- START主體拉低的長度
    constant START_GUARD_CYCLES : integer := BIT_CYCLES;            -- START結束後強制放開這麼久，確保跟bit0之間一定有邊緣
    constant START_SLOT_CYCLES  : integer := START_CYCLES + START_GUARD_CYCLES;
    constant FRAME_CYCLES       : integer := START_SLOT_CYCLES + 8 * BIT_CYCLES;  -- START slot + 8個bit，一個完整frame的總長
    constant GAP_CYCLES         : integer := BIT_CYCLES;            -- 換手/frame間的閒置時間
    constant BIT_WAIT_TIMEOUT   : integer := BIT_CYCLES * 4;        -- 等單一個bit自己邊緣的逾時上限(留了充裕margin)
    constant RX_TIMEOUT         : integer := FRAME_CYCLES * 4;      -- master等slave開始回應的逾時上限

    -- 抗雜訊濾波門檻：訊號要連續穩定這麼多個clk才算數，用來濾掉實體接線
    -- 接觸不良時常見的短暫毛刺。刻意設得遠小於LOW_SHORT(協定裡最短的合法
    -- 低電位持續時間)，濾雜訊的同時不會誤濾掉正常訊號。
    constant GLITCH_FILTER_CYCLES : integer := 8;

    -- slave在S_GAP1要多等的margin：master自己「送完frame轉聽slave」那段
    -- (S_TX_GAP)跟slave「收完frame到開始回覆」那段(S_GAP1)，兩個常數原本都是
    -- GAP_CYCLES、而且是從同一個時間點(master開始送frame那一刻)起算，理論上
    -- 會同時結束——slave開始送自己的START那一瞬間，master可能還沒真的切進
    -- S_RX_WAIT開始聽，master的edge偵測只認得到"從1變0"的那個瞬間，一旦錯過
    -- 就整個偵測不到，這樣slave這邊送的東西master會完全收不到。讓slave在
    -- S_GAP1多等這一段margin，確保master一定已經真的在聽了slave才開始送。
    constant SLAVE_REPLY_GAP_CYCLES : integer := GAP_CYCLES * 3;

    -- master在S_GAP(收完slave回覆、準備送下一輪新START之前)要多等的margin：
    -- 跟SLAVE_REPLY_GAP_CYCLES是同一個問題的另一個方向。master在slave傳最後
    -- 一個bit的「取樣中點」就已經判定收完、開始倒數S_GAP；但slave自己要等到
    -- 這個bit「整個slot結束」才真正送完、才開始倒數S_GAP2——兩邊倒數起點就已經
    -- 差了SAMPLE_POINT(=BIT_CYCLES/2)，如果两邊都只倒數同樣的GAP_CYCLES，
    -- master一定會提早(約BIT_CYCLES/2)開始送下一輪新START，這時候slave可能
    -- 都還沒退回S_RX_WAIT(還卡在S_GAP2這個純倒數、完全不看線路的狀態)，
    -- 導致slave整個錯過真正的START邊緣，只好誤把後面bit0自己的邊緣當成
    -- START——收到的每個bit都因此錯位一格(2026-09-07踩到，用獨立最小重現
    -- 測試在cycle等級精確定位過)。跟SLAVE_REPLY_GAP_CYCLES用同一個3倍margin
    -- 慣例，確保slave一定已經真的回到S_RX_WAIT在監聽了，master才開始送。
    constant MASTER_NEXT_TX_GAP_CYCLES : integer := GAP_CYCLES * 3;

    -- ===== 兩級同步器 + 毛刺濾波：master_blk/slave_blk現在永遠同時存在，
    -- 兩份邏輯都在讀同一條實體線，沒必要各自濾一次，抽出來共用一份 =====
    signal wire_s0, wire_s1, wire_s_prev : std_logic := '1';
    signal wire_clean, wire_clean_prev : std_logic := '1';
    signal glitch_cnt : integer range 0 to GLITCH_FILTER_CYCLES - 1 := 0;

    attribute mark_debug of wire_s1     : signal is "true";
    attribute mark_debug of wire_s_prev : signal is "true";
    attribute mark_debug of wire_clean  : signal is "true";

begin

    -- 開路集極(open-drain)：只能主動拉低或放開，跟原本I2C的scl/sda同一套邏輯
    link_wire <= '0' when drive0 = '1' else 'Z';

    -- 依is_master決定哪一份角色的結果才是真正的輸出；沒被選中的那份仍在背景
    -- 運作，只是這裡被丟棄，不會影響硬體正確性
    drive0           <= drive0_m     when is_master = '1' else drive0_s;
    handoff_rx_pulse <= handoff_rx_m when is_master = '1' else handoff_rx_s;
    score_rx_pulse   <= score_rx_m   when is_master = '1' else score_rx_s;
    grst_rx_pulse    <= grst_rx_m    when is_master = '1' else grst_rx_s;

    -- ===== 共用：兩級同步器 + 毛刺濾波 =====
    process(clk, rst)
    begin
        if rst = '1' then
            wire_s0 <= '1'; wire_s1 <= '1'; wire_s_prev <= '1';
            wire_clean <= '1'; wire_clean_prev <= '1'; glitch_cnt <= 0;
        elsif rising_edge(clk) then
            wire_s0     <= to_x01(link_wire);
            wire_s1     <= wire_s0;
            wire_s_prev <= wire_s1;

            -- 濾波：wire_s1要連續GLITCH_FILTER_CYCLES個clk都跟目前wire_clean不同，
            -- 才真的採信這次改變、更新wire_clean，濾掉比這個門檻還短的毛刺
            wire_clean_prev <= wire_clean;
            if wire_s1 = wire_clean then
                glitch_cnt <= 0;
            elsif glitch_cnt = GLITCH_FILTER_CYCLES - 1 then
                wire_clean <= wire_s1;
                glitch_cnt <= 0;
            else
                glitch_cnt <= glitch_cnt + 1;
            end if;
        end if;
    end process;

    -- =====================================================================
    -- MASTER角色邏輯：主動送出自己的frame，送完轉聽對面slave的回應，逾時就重新開始。
    -- 標籤刻意沿用"master_gen"時代就有的debug XDC習慣的同義詞master_blk，
    -- 內部訊號名稱(mstate/cnt/frame_byte/...)維持不變，方便比對舊版。
    -- =====================================================================
    master_blk : block

        type mstate_t is (S_TX, S_TX_GAP, S_RX_WAIT, S_RX, S_GAP);
        signal mstate : mstate_t := S_TX;

        signal cnt : integer range 0 to RX_TIMEOUT - 1 := 0;   -- 用在S_TX/S_TX_GAP/S_RX_WAIT/S_GAP的計時

        signal frame_byte : std_logic_vector(7 downto 0) := (others => '0');  -- 這一輪要送出去的byte(送之前snapshot鎖定，送到一半不會被新事件打斷)
        signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');  -- 正在從對面收進來的byte

        -- ===== S_RX用：每個bit各自的邊緣追蹤 =====
        signal bit_idx     : integer range 0 to 7 := 0;
        signal bit_timer   : integer range 0 to BIT_CYCLES - 1 := 0;   -- 從「這個bit自己的邊緣」數到SAMPLE_POINT
        signal bit_started : std_logic := '0';                          -- 是否已經抓到這個bit自己的邊緣
        signal wait_timer  : integer range 0 to BIT_WAIT_TIMEOUT - 1 := 0;  -- 等這個bit邊緣的逾時計時

        -- ===== S_TX用：跟S_RX同一種「累加計數器」寫法，不能用除法/取餘數
        -- (cnt-START_SLOT_CYCLES)/BIT_CYCLES這種寫法在模擬上完全正確，但BIT_CYCLES
        -- 不是2的次方數，硬體上會被合成器展開成一整棵除法器，深到單一個100MHz clock
        -- 週期內走不完，實際燒板後timing根本不過(WNS一路到-11ns等級)，只是模擬
        -- 測不出來、只有真的跑Implementation才會抓到。改成兩個各自累加的計數器
        -- (tx_slot_pos數到BIT_CYCLES-1就歸零、順便把tx_bit_idx加1)，純粹是比較器
        -- 跟加法器，沒有除法器，速度快很多 =====
        signal tx_bit_idx  : integer range 0 to 7 := 0;
        signal tx_slot_pos : integer range 0 to BIT_CYCLES - 1 := 0;

        signal pending_handoff : std_logic := '0';
        signal pending_score   : std_logic := '0';
        signal pending_grst    : std_logic := '0';   -- 共用GRST：我剛被本地GRST重置了，等著通知對面

        -- ===== ACK重傳機制：pending_handoff/pending_score不再是"送出去就清掉"，
        -- 而是要等對面在牠的回覆裡蓋章確認(bit2=ack_handoff, bit3=ack_score)才清掉，
        -- 沒收到確認就會在下一輪繼續帶著同一筆資料重送，不會因為單次沒送到就永久遺失。
        -- prev_rx_handoff/prev_rx_score則是用來把handoff_rx_m/score_rx_m做成edge-triggered
        -- (只在這筆資料第一次出現時脈衝一次)，避免同一筆資料因為對面一直重送、
        -- 在確認回來之前被pingpong_game重複觸發(score尤其不能重複加)。
        signal peer_ack_handoff, peer_ack_score, peer_ack_grst : std_logic := '0';
        signal prev_rx_handoff, prev_rx_score, prev_rx_grst    : std_logic := '0';

        -- rx_confirm: 同一個frame要連續收到兩輪內容一模一樣才當真，
        -- 濾掉單輪雜訊把某個bit翻轉、被誤判成"對面新送來一筆request"的狀況
        signal rx_confirm : std_logic_vector(7 downto 0) := (others => '0');

        -- ===== Debug用：標記這幾個訊號給ILA看，用RTL層級的乾淨名稱，
        -- 不用每次重新synthesis後再去netlist瀏覽視窗大海撈針找被合成器改過名的訊號 =====
        attribute mark_debug : string;
        attribute mark_debug of mstate           : signal is "true";
        attribute mark_debug of cnt               : signal is "true";
        attribute mark_debug of frame_byte         : signal is "true";
        attribute mark_debug of rx_byte           : signal is "true";
        attribute mark_debug of bit_idx           : signal is "true";
        attribute mark_debug of tx_bit_idx        : signal is "true";
        attribute mark_debug of tx_slot_pos       : signal is "true";
        attribute mark_debug of bit_timer         : signal is "true";
        attribute mark_debug of bit_started       : signal is "true";
        attribute mark_debug of pending_handoff   : signal is "true";
        attribute mark_debug of pending_score     : signal is "true";
        attribute mark_debug of pending_grst      : signal is "true";
        attribute mark_debug of peer_ack_grst     : signal is "true";

    begin

        process(clk, rst)
            variable low_thresh : integer range 0 to BIT_CYCLES;
        begin
            if rst = '1' then
                mstate <= S_TX;
                cnt    <= 0;
                drive0_m <= '0';
                frame_byte <= (others => '0');
                rx_byte    <= (others => '0');
                bit_idx     <= 0;
                bit_timer   <= 0;
                bit_started <= '0';
                wait_timer  <= 0;
                tx_bit_idx  <= 0;
                tx_slot_pos <= 0;
                pending_handoff <= '0';
                pending_score   <= '0';
                pending_grst    <= '0';
                handoff_rx_m <= '0';
                score_rx_m   <= '0';
                grst_rx_m    <= '0';
                peer_ack_handoff <= '0'; peer_ack_score <= '0'; peer_ack_grst <= '0';
                prev_rx_handoff  <= '0'; prev_rx_score  <= '0'; prev_rx_grst  <= '0';
                rx_confirm <= (others => '0');

            elsif rising_edge(clk) then
                -- 隨時鎖存本地遊戲FSM送來的待送事件，跟目前FSM在哪個狀態無關，先鎖住避免漏接
                if handoff_tx_pulse = '1' then
                    pending_handoff <= '1';
                end if;
                if score_tx_pulse = '1' then
                    pending_score <= '1';
                end if;
                if grst_tx_pulse = '1' then
                    pending_grst <= '1';
                end if;

                -- rx脈波預設只維持一拍，除非下面重新拉高，否則自動清除
                handoff_rx_m <= '0';
                score_rx_m   <= '0';
                grst_rx_m    <= '0';

                case mstate is

                    -- ===== 送出frame_byte：START + guard + 8個PWM編碼的bit =====
                    when S_TX =>
                        if cnt < START_CYCLES then
                            drive0_m    <= '1';
                            tx_bit_idx  <= 0;
                            tx_slot_pos <= 0;
                        elsif cnt < START_SLOT_CYCLES then
                            drive0_m    <= '0';
                            tx_bit_idx  <= 0;
                            tx_slot_pos <= 0;
                        else
                            if frame_byte(tx_bit_idx) = '0' then
                                low_thresh := LOW_LONG;
                            else
                                low_thresh := LOW_SHORT;
                            end if;
                            if tx_slot_pos < low_thresh then
                                drive0_m <= '1';
                            else
                                drive0_m <= '0';
                            end if;

                            if tx_slot_pos = BIT_CYCLES - 1 then
                                tx_slot_pos <= 0;
                                tx_bit_idx  <= tx_bit_idx + 1;
                            else
                                tx_slot_pos <= tx_slot_pos + 1;
                            end if;
                        end if;

                        if cnt = FRAME_CYCLES - 1 then
                            cnt    <= 0;
                            drive0_m <= '0';
                            mstate <= S_TX_GAP;
                        else
                            cnt <= cnt + 1;
                        end if;

                    -- ===== 送完的緩衝時間，確保線真的穩定放開了才開始聽 =====
                    when S_TX_GAP =>
                        drive0_m <= '0';
                        if cnt = GAP_CYCLES - 1 then
                            cnt    <= 0;
                            mstate <= S_RX_WAIT;
                        else
                            cnt <= cnt + 1;
                        end if;

                    -- ===== 等對面slave的START(下降緣)，逾時就放棄改重新送一輪 =====
                    when S_RX_WAIT =>
                        drive0_m <= '0';
                        if wire_clean_prev = '1' and wire_clean = '0' then
                            bit_idx     <= 0;
                            bit_started <= '0';
                            wait_timer  <= 0;
                            cnt         <= 0;
                            mstate      <= S_RX;
                        elsif cnt = RX_TIMEOUT - 1 then
                            cnt    <= 0;
                            -- 這裡直接補做原本只有S_GAP才會做的frame_byte打包，不然pending_handoff/
                            -- pending_score會一直卡在佇列裡，要等到哪天運氣好完整收滿一輪(進過S_GAP)
                            -- 才會被送出去；實測過連線品質不穩時，光靠"完整成功一輪"這個條件，
                            -- 佇列卡住六秒到八十秒都有可能。改成逾時也一併打包，這樣不管上一輪
                            -- RX有沒有成功，下一次送出去的frame一定帶著當下最新的pending狀態，
                            -- 重試頻率等於一個RX_TIMEOUT週期就有一次機會，不必賭到哪次奇蹟般全收完。
                            frame_byte      <= "00" & peer_ack_grst & pending_grst & peer_ack_score & peer_ack_handoff & pending_score & pending_handoff;
                            mstate <= S_TX;   -- 對面沒回應(可能還沒開機/還沒放開重置)，別卡死，直接開始下一輪
                        else
                            cnt <= cnt + 1;
                        end if;

                    -- ===== 接收slave的frame：每個bit各自抓自己的邊緣、數到SAMPLE_POINT取樣 =====
                    when S_RX =>
                        drive0_m <= '0';
                        if bit_started = '0' then
                            -- 還沒看到這個bit自己的下降緣，繼續等(逾時就放棄，回去重新等乾淨的START)
                            if wire_clean_prev = '1' and wire_clean = '0' then
                                bit_started <= '1';
                                bit_timer   <= 0;
                                wait_timer  <= 0;
                            elsif wait_timer = BIT_WAIT_TIMEOUT - 1 then
                                mstate <= S_RX_WAIT;
                            else
                                wait_timer <= wait_timer + 1;
                            end if;
                        else
                            -- 已經看到這個bit的邊緣，數到SAMPLE_POINT就取樣
                            if bit_timer = SAMPLE_POINT then
                                rx_byte(bit_idx) <= wire_clean;
                                if bit_idx = 7 then
                                    if rx_byte = rx_confirm then
                                        handoff_rx_m    <= rx_byte(0) and not prev_rx_handoff;
                                        score_rx_m      <= rx_byte(1) and not prev_rx_score;
                                        prev_rx_handoff <= rx_byte(0);
                                        prev_rx_score   <= rx_byte(1);
                                        peer_ack_handoff <= rx_byte(0);
                                        peer_ack_score   <= rx_byte(1);
                                        if rx_byte(2) = '1' then
                                            pending_handoff <= '0';
                                        end if;
                                        if rx_byte(3) = '1' then
                                            pending_score <= '0';
                                        end if;
                                        grst_rx_m       <= rx_byte(4) and not prev_rx_grst;
                                        prev_rx_grst    <= rx_byte(4);
                                        peer_ack_grst   <= rx_byte(4);
                                        if rx_byte(5) = '1' then
                                            pending_grst <= '0';
                                        end if;
                                    end if;
                                    rx_confirm   <= rx_byte;
                                    cnt          <= 0;
                                    mstate       <= S_GAP;
                                else
                                    bit_idx     <= bit_idx + 1;
                                    bit_started <= '0';   -- 換下一個bit，重新等它自己的邊緣
                                    wait_timer  <= 0;
                                end if;
                            else
                                bit_timer <= bit_timer + 1;
                            end if;
                        end if;

                    -- ===== 一輪結束的緩衝時間，順便把這段時間內累積的pending事件鎖進下一輪的frame_byte =====
                    -- 這裡用MASTER_NEXT_TX_GAP_CYCLES(=GAP_CYCLES*3)而不是plain
                    -- GAP_CYCLES，理由見該常數宣告處的說明：master判定"收完"比
                    -- slave判定"送完"早了SAMPLE_POINT，兩邊若用同樣的gap長度，
                    -- master會在slave真的退回S_RX_WAIT之前就搶先送出新START。
                    when S_GAP =>
                        drive0_m <= '0';
                        if cnt = MASTER_NEXT_TX_GAP_CYCLES - 1 then
                            frame_byte      <= "00" & peer_ack_grst & pending_grst & peer_ack_score & peer_ack_handoff & pending_score & pending_handoff;
                            cnt    <= 0;
                            mstate <= S_TX;
                        else
                            cnt <= cnt + 1;
                        end if;

                end case;

            end if;
        end process;

    end block master_blk;

    -- =====================================================================
    -- SLAVE角色邏輯：被動等master的START，收完再輪到自己送，送完回去繼續聽。
    -- 在S_RX_WAIT不設逾時——slave本來就該乖乖等，master不理它的話就一直等下去；
    -- 但一旦開始接收(進了S_RX)，如果中途某個bit的邊緣遲遲等不到，就放棄這次
    -- 接收、回去重新等下一次乾淨的START，不會卡在半吊子的接收狀態出不來。
    -- =====================================================================
    slave_blk : block

        type sstate_t is (S_RX_WAIT, S_RX, S_GAP1, S_TX, S_GAP2);
        signal sstate : sstate_t := S_RX_WAIT;

        signal cnt : integer range 0 to FRAME_CYCLES - 1 := 0;   -- 用在S_GAP1/S_TX/S_GAP2的計時

        signal frame_byte : std_logic_vector(7 downto 0) := (others => '0');
        signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');

        -- ===== S_RX用：每個bit各自的邊緣追蹤 =====
        signal bit_idx     : integer range 0 to 7 := 0;
        signal bit_timer   : integer range 0 to BIT_CYCLES - 1 := 0;
        signal bit_started : std_logic := '0';
        signal wait_timer  : integer range 0 to BIT_WAIT_TIMEOUT - 1 := 0;

        -- S_TX用：跟master_blk同樣理由，改成累加計數器，不能用除法/取餘數(合成不出好timing)
        signal tx_bit_idx  : integer range 0 to 7 := 0;
        signal tx_slot_pos : integer range 0 to BIT_CYCLES - 1 := 0;

        signal pending_handoff : std_logic := '0';
        signal pending_score   : std_logic := '0';
        signal pending_grst    : std_logic := '0';   -- 共用GRST：我剛被本地GRST重置了，等著通知對面

        -- ===== ACK重傳機制：pending_handoff/pending_score不再是"送出去就清掉"，
        -- 而是要等對面在牠的回覆裡蓋章確認(bit2=ack_handoff, bit3=ack_score)才清掉，
        -- 沒收到確認就會在下一輪繼續帶著同一筆資料重送，不會因為單次沒送到就永久遺失。
        -- prev_rx_handoff/prev_rx_score則是用來把handoff_rx_s/score_rx_s做成edge-triggered
        -- (只在這筆資料第一次出現時脈衝一次)，避免同一筆資料因為對面一直重送、
        -- 在確認回來之前被pingpong_game重複觸發(score尤其不能重複加)。
        signal peer_ack_handoff, peer_ack_score, peer_ack_grst : std_logic := '0';
        signal prev_rx_handoff, prev_rx_score, prev_rx_grst    : std_logic := '0';

        -- rx_confirm: 同一個frame要連續收到兩輪內容一模一樣才當真，
        -- 濾掉單輪雜訊把某個bit翻轉、被誤判成"對面新送來一筆request"的狀況
        signal rx_confirm : std_logic_vector(7 downto 0) := (others => '0');

        -- ===== Debug用：跟master_blk同樣理由，標記給ILA看，避免重新synthesis後
        -- 訊號被合成器改名，還要重新去netlist瀏覽視窗大海撈針 =====
        attribute mark_debug : string;
        attribute mark_debug of sstate           : signal is "true";
        attribute mark_debug of cnt               : signal is "true";
        attribute mark_debug of frame_byte         : signal is "true";
        attribute mark_debug of rx_byte           : signal is "true";
        attribute mark_debug of bit_idx           : signal is "true";
        attribute mark_debug of tx_bit_idx        : signal is "true";
        attribute mark_debug of tx_slot_pos       : signal is "true";
        attribute mark_debug of bit_timer         : signal is "true";
        attribute mark_debug of bit_started       : signal is "true";
        attribute mark_debug of pending_handoff   : signal is "true";
        attribute mark_debug of pending_score     : signal is "true";
        attribute mark_debug of pending_grst      : signal is "true";
        attribute mark_debug of peer_ack_grst     : signal is "true";

    begin

        process(clk, rst)
            variable low_thresh : integer range 0 to BIT_CYCLES;
        begin
            if rst = '1' then
                sstate <= S_RX_WAIT;
                cnt    <= 0;
                drive0_s <= '0';
                frame_byte <= (others => '0');
                rx_byte    <= (others => '0');
                bit_idx     <= 0;
                bit_timer   <= 0;
                bit_started <= '0';
                wait_timer  <= 0;
                tx_bit_idx  <= 0;
                tx_slot_pos <= 0;
                pending_handoff <= '0';
                pending_score   <= '0';
                pending_grst    <= '0';
                handoff_rx_s <= '0';
                score_rx_s   <= '0';
                grst_rx_s    <= '0';
                peer_ack_handoff <= '0'; peer_ack_score <= '0'; peer_ack_grst <= '0';
                prev_rx_handoff  <= '0'; prev_rx_score  <= '0'; prev_rx_grst  <= '0';
                rx_confirm <= (others => '0');

            elsif rising_edge(clk) then
                if handoff_tx_pulse = '1' then
                    pending_handoff <= '1';
                end if;
                if score_tx_pulse = '1' then
                    pending_score <= '1';
                end if;
                if grst_tx_pulse = '1' then
                    pending_grst <= '1';
                end if;

                handoff_rx_s <= '0';
                score_rx_s   <= '0';
                grst_rx_s    <= '0';

                case sstate is

                    -- ===== 等master的START(下降緣) =====
                    when S_RX_WAIT =>
                        drive0_s <= '0';
                        if wire_clean_prev = '1' and wire_clean = '0' then
                            bit_idx     <= 0;
                            bit_started <= '0';
                            wait_timer  <= 0;
                            sstate      <= S_RX;
                        end if;

                    -- ===== 接收master的frame：每個bit各自抓自己的邊緣、數到SAMPLE_POINT取樣 =====
                    when S_RX =>
                        drive0_s <= '0';
                        if bit_started = '0' then
                            if wire_clean_prev = '1' and wire_clean = '0' then
                                bit_started <= '1';
                                bit_timer   <= 0;
                                wait_timer  <= 0;
                            elsif wait_timer = BIT_WAIT_TIMEOUT - 1 then
                                sstate <= S_RX_WAIT;   -- 等太久沒等到，放棄這次接收，回去重新等乾淨的START
                            else
                                wait_timer <= wait_timer + 1;
                            end if;
                        else
                            if bit_timer = SAMPLE_POINT then
                                rx_byte(bit_idx) <= wire_clean;
                                if bit_idx = 7 then
                                    if rx_byte = rx_confirm then
                                        handoff_rx_s    <= rx_byte(0) and not prev_rx_handoff;
                                        score_rx_s      <= rx_byte(1) and not prev_rx_score;
                                        prev_rx_handoff <= rx_byte(0);
                                        prev_rx_score   <= rx_byte(1);
                                        peer_ack_handoff <= rx_byte(0);
                                        peer_ack_score   <= rx_byte(1);
                                        if rx_byte(2) = '1' then
                                            pending_handoff <= '0';
                                        end if;
                                        if rx_byte(3) = '1' then
                                            pending_score <= '0';
                                        end if;
                                        grst_rx_s       <= rx_byte(4) and not prev_rx_grst;
                                        prev_rx_grst    <= rx_byte(4);
                                        peer_ack_grst   <= rx_byte(4);
                                        if rx_byte(5) = '1' then
                                            pending_grst <= '0';
                                        end if;
                                    end if;
                                    rx_confirm   <= rx_byte;
                                    cnt          <= 0;
                                    sstate       <= S_GAP1;
                                else
                                    bit_idx     <= bit_idx + 1;
                                    bit_started <= '0';
                                    wait_timer  <= 0;
                                end if;
                            else
                                bit_timer <= bit_timer + 1;
                            end if;
                        end if;

                    -- ===== 換手緩衝時間，順便把pending事件鎖進要送出去的frame_byte =====
                    when S_GAP1 =>
                        drive0_s <= '0';
                        if cnt = SLAVE_REPLY_GAP_CYCLES - 1 then
                            frame_byte      <= "00" & peer_ack_grst & pending_grst & peer_ack_score & peer_ack_handoff & pending_score & pending_handoff;
                            cnt    <= 0;
                            sstate <= S_TX;
                        else
                            cnt <= cnt + 1;
                        end if;

                    -- ===== 輪到slave送自己的frame_byte回去 =====
                    when S_TX =>
                        if cnt < START_CYCLES then
                            drive0_s    <= '1';
                            tx_bit_idx  <= 0;
                            tx_slot_pos <= 0;
                        elsif cnt < START_SLOT_CYCLES then
                            drive0_s    <= '0';
                            tx_bit_idx  <= 0;
                            tx_slot_pos <= 0;
                        else
                            if frame_byte(tx_bit_idx) = '0' then
                                low_thresh := LOW_LONG;
                            else
                                low_thresh := LOW_SHORT;
                            end if;
                            if tx_slot_pos < low_thresh then
                                drive0_s <= '1';
                            else
                                drive0_s <= '0';
                            end if;

                            if tx_slot_pos = BIT_CYCLES - 1 then
                                tx_slot_pos <= 0;
                                tx_bit_idx  <= tx_bit_idx + 1;
                            else
                                tx_slot_pos <= tx_slot_pos + 1;
                            end if;
                        end if;

                        if cnt = FRAME_CYCLES - 1 then
                            cnt    <= 0;
                            drive0_s <= '0';
                            sstate <= S_GAP2;
                        else
                            cnt <= cnt + 1;
                        end if;

                    -- ===== 送完的緩衝時間，確保線穩定放開了才回去聽下一輪 =====
                    when S_GAP2 =>
                        drive0_s <= '0';
                        if cnt = GAP_CYCLES - 1 then
                            cnt    <= 0;
                            sstate <= S_RX_WAIT;
                        else
                            cnt <= cnt + 1;
                        end if;

                end case;

            end if;
        end process;

    end block slave_blk;

end Behavioral;
