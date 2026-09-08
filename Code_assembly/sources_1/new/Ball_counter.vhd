library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- 這顆模組管理球在0~7位置間的移動，只負責「位置」跟「方向」，不知道自己是「哪一方」。
-- 沿用自Project1/2的「跑馬燈」概念延伸而來，每個en脈波移動一格(跟Project4的除頻/致能作法相同)，
-- 這顆本身不做tick除頻，由上層FSM在該走的那一拍才拉一次en。
--
-- set_pos/set_val：外部可以強制指定位置(例如發球或跨板交接時，球要瞬間出現在某一端)，
-- 同時依指定值自動決定新方向：set_val=0代表從自己這端出發(方向設成往右/往net端)，
-- set_val=7代表從對面(net端)進來(方向設成往左/往自己端)。
entity ball_counter is
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        en       : in  std_logic;                  -- 只在MOVING狀態、且到了tick那一拍才拉高
        set_pos  : in  std_logic;                   -- 拉高一個clk，強制把位置改成set_val
        set_val  : in  unsigned(2 downto 0);
        pos      : out unsigned(2 downto 0);        -- 球目前位置0~7
        dir      : out std_logic;                   -- '1'=往右(net端)移動, '0'=往左(自己端)移動
        at_left  : out std_logic;                   -- pos = 0
        at_right : out std_logic                    -- pos = 7
    );
end ball_counter;

architecture Behavioral of ball_counter is
    signal pos_r : unsigned(2 downto 0) := (others => '0');
    signal dir_r : std_logic := '1';               -- '1' = 往右(net端), '0' = 往左(自己端)
begin
    -- ===== 位置暫存器：優先處理set_pos強制設位，其次才是en致能移動 =====
    process(clk, rst)
    begin
        if rst = '1' then
            pos_r <= (others => '0');
            dir_r <= '1';
        elsif rising_edge(clk) then
            if set_pos = '1' then
                -- 強制設位，同時依慣例自動決定方向：從哪一端出現，就往另一端走
                pos_r <= set_val;
                if set_val = 0 then
                    dir_r <= '1';        -- 從自己端出發，往右(net端)走
                elsif set_val = 7 then
                    dir_r <= '0';        -- 從net端進來，往左(自己端)走
                end if;
            elsif en = '1' then
                -- 正常移動：碰到邊界就翻轉方向，同一拍內完成翻轉+不越界
                if dir_r = '1' then
                    if pos_r = 6 then
                        pos_r <= to_unsigned(7, 3);
                        dir_r <= '0';               -- 到了net端，方向反轉準備回頭
                    else
                        pos_r <= pos_r + 1;
                    end if;
                else
                    if pos_r = 1 then
                        pos_r <= to_unsigned(0, 3);
                        dir_r <= '1';               -- 到了自己端，方向反轉準備出發
                    else
                        pos_r <= pos_r - 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    pos      <= pos_r;
    dir      <= dir_r;
    at_left  <= '1' when pos_r = 0 else '0';
    at_right <= '1' when pos_r = 7 else '0';
end Behavioral;
