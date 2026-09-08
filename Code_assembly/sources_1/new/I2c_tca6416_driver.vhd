library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- 驅動板上TCA6416PW I2C GPIO expander(U15)，把pingpong_top送出的
-- seg(gfedcba) + digit_sel持續轉成I2C write指令，週期性地重複寫出。
--
-- TCA6416暫存器位址(已固定)：
--   0x02 = Output Port 0 (P00~P07，對應LED_A~LED_H，也就是七段的a~g + DP)
--   0x03 = Output Port 1 (P10~P17，P10=LEDB0(D9) P11=LEDB1(D1))
--   0x06 = Configuration Port 0 (0=輸出,1=輸入，開機預設全部是輸入，一定要先設成輸出)
--   0x07 = Configuration Port 1
--
-- 每次要完整寫出去分三個步驟，順序固定：
--   1) output0 先清零 (0x00)
--   2) output1 選數位
--   3) output0 再寫真正的段碼出去
-- 因為I2C一次只能寫一個暫存器，寫入電位切換之間會有短暫過渡，如果順序顛倒(段碼先出)，
-- 會讓「舊段碼在錯的數位上」跟「新數位配舊段碼」短暫同時出現，兩顆七段顯示器會混在一起，肉眼會看到殘影。
-- 這個三步驟就是確保每次切換之前先把上一輪的殘留清乾淨，才輪到真正要增加/更新的段碼，避免產生殘影。


entity i2c_tca6416_driver is
    generic (
        CLK_FREQ_HZ : integer := 100_000_000;
        I2C_FREQ_HZ : integer := 100_000;                      -- I2C standard mode 100kHz
        I2C_ADDR    : std_logic_vector(6 downto 0) := "0100000"  -- TCA6416, A0接地時位址 = 0x20
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        seg       : in  std_logic_vector(6 downto 0);      -- 目前段碼(gfedcba)，來自pingpong_top
        digit_sel : in  std_logic;                       -- 0=個位, 1=十位
        scl       : out std_logic;
        sda       : inout std_logic                      -- 開路集極(open-drain)的I2C線
    );
end i2c_tca6416_driver;

architecture Behavioral of i2c_tca6416_driver is

    constant HALF_CYCLES : integer := CLK_FREQ_HZ / I2C_FREQ_HZ / 2;  -- SCL每半週期要等的clk數

    signal half_timer : integer range 0 to HALF_CYCLES - 1 := 0;
    signal half_tick  : std_logic := '0';

    type bstate_t is (S_IDLE, S_START, S_BIT_LOW, S_BIT_HIGH,
                       S_ACK_LOW, S_ACK_HIGH, S_STOP1, S_STOP2, S_GAP);
    signal bstate : bstate_t := S_IDLE;

    signal bit_cnt  : integer range 0 to 7 := 0;
    signal byte_idx : integer range 0 to 2 := 0;          -- 0=I2C位址, 1=暫存器位址, 2=資料

    signal txn_sel  : integer range 0 to 4 := 0;          -- txn_sel: 0=cfg0, 1=cfg1, 2=output0先歸零, 3=output1選數位, 4=output0寫真正的段碼。
    signal gap_cnt  : integer range 0 to 3 := 0;

    signal scl_i      : std_logic := '1';
    signal sda_drive0 : std_logic := '0';                  -- '1'=主動拉低SDA, '0'=釋放(浮接被上拉)

    signal seg_latched  : std_logic_vector(6 downto 0) := (others => '0');
    signal dsel_latched : std_logic := '0';

    signal cur_byte : std_logic_vector(7 downto 0);

begin

    scl <= scl_i;
    sda <= '0' when sda_drive0 = '1' else 'Z';   -- 開路集極輸出，只拉低不主動拉高

    -- ===== SCL半週期計時器 =====
    process(clk, rst)
    begin
        if rst = '1' then
            half_timer <= 0;
            half_tick  <= '0';
        elsif rising_edge(clk) then
            if half_timer = HALF_CYCLES - 1 then
                half_timer <= 0;
                half_tick  <= '1';
            else
                half_timer <= half_timer + 1;
                half_tick  <= '0';
            end if;
        end if;
    end process;

    -- ===== 依txn_sel/byte_idx組合出現在要送出的那顆byte =====
    process(txn_sel, byte_idx, seg_latched, dsel_latched)
        variable reg_addr  : std_logic_vector(7 downto 0);
        variable data_byte : std_logic_vector(7 downto 0);
    begin
        case txn_sel is
            when 0 => reg_addr := x"06"; data_byte := x"00";              -- config port0 設成輸出
            when 1 => reg_addr := x"07"; data_byte := x"00";              -- config port1 設成輸出
            when 2 => reg_addr := x"02"; data_byte := "00000000";         -- output0先歸零
            when 3 =>
                reg_addr := x"03";
                if dsel_latched = '1' then
                    data_byte := "00000001";  -- 十位 -> LEDB0
                else
                    data_byte := "00000010";  -- 個位 -> LEDB1
                end if;
            when others => reg_addr := x"02"; data_byte := "0" & seg_latched;  -- 最後才寫真正的段碼
        end case;

        case byte_idx is
            when 0      => cur_byte <= I2C_ADDR & '0';   -- 元件位址 + Write
            when 1      => cur_byte <= reg_addr;
            when others => cur_byte <= data_byte;
        end case;
    end process;

    -- ===== 主狀態機：每個half_tick前進一步。config(txn_sel 0,1)開機後各送一次，
    -- 之後就在「先歸零(2) -> 選數位(3) -> 寫段碼(4)」三步驟間持續循環 =====
    process(clk, rst)
    begin
        if rst = '1' then
            bstate       <= S_IDLE;
            scl_i        <= '1';
            sda_drive0   <= '0';
            bit_cnt      <= 0;
            byte_idx     <= 0;
            txn_sel      <= 0;
            gap_cnt      <= 0;
            seg_latched  <= (others => '0');
            dsel_latched <= '0';
        elsif rising_edge(clk) then
            if half_tick = '1' then
                case bstate is

                    when S_IDLE =>
                        scl_i      <= '1';
                        sda_drive0 <= '0';
                        bit_cnt  <= 0;
                        byte_idx <= 0;
                        bstate   <= S_START;

                    when S_START =>
                        scl_i      <= '1';
                        sda_drive0 <= '1';        -- SCL高時SDA下降 = START
                        bstate     <= S_BIT_LOW;

                    when S_BIT_LOW =>
                        scl_i      <= '0';
                        sda_drive0 <= not cur_byte(7 - bit_cnt);
                        bstate     <= S_BIT_HIGH;

                    when S_BIT_HIGH =>
                        scl_i <= '1';
                        if bit_cnt = 7 then
                            bstate <= S_ACK_LOW;
                        else
                            bit_cnt <= bit_cnt + 1;
                            bstate  <= S_BIT_LOW;
                        end if;

                    when S_ACK_LOW =>
                        scl_i      <= '0';
                        sda_drive0 <= '0';        -- 釋放SDA讓從機拉低回ACK (不強制驅動)
                        bstate     <= S_ACK_HIGH;

                    when S_ACK_HIGH =>
                        scl_i <= '1';
                        if byte_idx = 2 then
                            bstate <= S_STOP1;
                        else
                            byte_idx <= byte_idx + 1;
                            bit_cnt  <= 0;
                            bstate   <= S_BIT_LOW;
                        end if;

                    when S_STOP1 =>
                        scl_i      <= '0';
                        sda_drive0 <= '1';        -- 先確保SDA是低
                        bstate     <= S_STOP2;

                    when S_STOP2 =>
                        scl_i      <= '1';
                        sda_drive0 <= '0';        -- SCL高時放開SDA(上升) = STOP
                        bstate     <= S_GAP;
                        gap_cnt    <= 0;

                    when S_GAP =>
                        if gap_cnt = 3 then
                            case txn_sel is
                                when 0 =>
                                    txn_sel <= 1;
                                when 1 =>
                                    txn_sel      <= 2;
                                    seg_latched  <= seg;      -- 開始鎖存這一輪(先歸零->選數位->寫段碼)要用的資料
                                    dsel_latched <= digit_sel;
                                when 2 =>
                                    txn_sel <= 3;             -- 已經歸零，換選數位
                                when 3 =>
                                    txn_sel <= 4;             -- 數位已選好，寫真正的段碼
                                when others =>                -- txn_sel = 4，一輪跑完，重新開始下一輪
                                    txn_sel      <= 2;
                                    seg_latched  <= seg;
                                    dsel_latched <= digit_sel;
                            end case;
                            bstate <= S_IDLE;
                        else
                            gap_cnt <= gap_cnt + 1;
                        end if;

                end case;
            end if;
        end if;
    end process;

end Behavioral;
