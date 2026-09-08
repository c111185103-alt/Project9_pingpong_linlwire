library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- 按鍵去彈跳(debounce)模組：
-- 兩級同步(避免metastability) + 穩定計數判定，訊號要連續穩定滿DEBOUNCE_LIMIT個
-- clk週期沒有變化才承認狀態改變，並在確認的那一拍額外送出一個clk寬的btn_out脈波
entity debounce is
    generic (
        DEBOUNCE_LIMIT : integer := 1_000_000   -- 100MHz下約10ms
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        btn_in    : in  std_logic;              -- 按鍵原始/未去彈跳訊號
        btn_out   : out std_logic;              -- 去彈跳後的上升緣脈波(僅一拍)
        btn_level : out std_logic               -- 去彈跳後的穩定電位(給開關類用途，如SW0)
    );
end debounce;

architecture Behavioral of debounce is
    signal cnt                  : integer range 0 to DEBOUNCE_LIMIT - 1 := 0;
    signal btn_stable           : std_logic := '0';
    signal btn_sync0, btn_sync1 : std_logic := '0';
    signal btn_prev             : std_logic := '0';
begin
    -- ===== 兩級同步器：把外部非同步的按鍵訊號拉進本地clock domain =====
    process(clk)
    begin
        if rising_edge(clk) then
            btn_sync0 <= btn_in;
            btn_sync1 <= btn_sync0;
        end if;
    end process;

    -- ===== 穩定計數 + 邊緣偵測 =====
    process(clk, rst)
    begin
        if rst = '1' then
            cnt        <= 0;
            btn_stable <= '0';
            btn_prev   <= '0';
            btn_out    <= '0';
        elsif rising_edge(clk) then
            btn_out  <= '0';
            btn_prev <= btn_stable;

            if btn_sync1 = btn_stable then
                cnt <= 0;
            else
                -- 訊號跟目前穩定值不同，持續計數；滿DEBOUNCE_LIMIT才承認新狀態
                if cnt = DEBOUNCE_LIMIT - 1 then
                    btn_stable <= btn_sync1;
                    cnt        <= 0;
                else
                    cnt <= cnt + 1;
                end if;
            end if;

            if btn_stable = '1' and btn_prev = '0' then
                btn_out <= '1';        -- 剛確認變成按下那一拍送出脈波
            end if;
        end if;
    end process;

    btn_level <= btn_stable;
end Behavioral;
