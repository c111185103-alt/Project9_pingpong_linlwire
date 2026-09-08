library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity link_wire_drv is
    generic (
        CLK_FREQ_HZ      : integer := 100_000_000;
        LINK_BIT_FREQ_HZ : integer := 200_000
    );
    port (
        clk : in std_logic;
        rst : in std_logic;

        is_master : in std_logic;

        link_wire : inout std_logic;

        handoff_tx_pulse : in  std_logic;
        score_tx_pulse   : in  std_logic;
        handoff_rx_pulse : out std_logic;
        score_rx_pulse   : out std_logic;

        grst_tx_pulse    : in  std_logic;
        grst_rx_pulse    : out std_logic
    );
end link_wire_drv;

architecture Behavioral of link_wire_drv is

    signal drive0_m, drive0_s, drive0 : std_logic := '0';
    signal handoff_rx_m, score_rx_m, grst_rx_m : std_logic := '0';
    signal handoff_rx_s, score_rx_s, grst_rx_s : std_logic := '0';

    attribute mark_debug : string;
    attribute mark_debug of drive0     : signal is "true";
    attribute mark_debug of is_master  : signal is "true";
    attribute mark_debug of handoff_rx_m : signal is "true";
    attribute mark_debug of score_rx_m   : signal is "true";
    attribute mark_debug of grst_rx_m    : signal is "true";
    attribute mark_debug of handoff_rx_s : signal is "true";
    attribute mark_debug of score_rx_s   : signal is "true";
    attribute mark_debug of grst_rx_s    : signal is "true";

    constant BIT_CYCLES         : integer := CLK_FREQ_HZ / LINK_BIT_FREQ_HZ;
    constant LOW_SHORT          : integer := BIT_CYCLES / 4;
    constant LOW_LONG           : integer := (BIT_CYCLES * 3) / 4;
    constant SAMPLE_POINT       : integer := BIT_CYCLES / 2;
    constant START_CYCLES       : integer := BIT_CYCLES * 2;
    constant START_GUARD_CYCLES : integer := BIT_CYCLES;
    constant START_SLOT_CYCLES  : integer := START_CYCLES + START_GUARD_CYCLES;
    constant FRAME_CYCLES       : integer := START_SLOT_CYCLES + 8 * BIT_CYCLES;
    constant GAP_CYCLES         : integer := BIT_CYCLES;
    constant BIT_WAIT_TIMEOUT   : integer := BIT_CYCLES * 4;
    constant RX_TIMEOUT         : integer := FRAME_CYCLES * 4;

    constant GLITCH_FILTER_CYCLES : integer := 8;

    constant SLAVE_REPLY_GAP_CYCLES : integer := GAP_CYCLES * 3;

    constant MASTER_NEXT_TX_GAP_CYCLES : integer := GAP_CYCLES * 3;

    signal wire_s0, wire_s1, wire_s_prev : std_logic := '1';
    signal wire_clean, wire_clean_prev : std_logic := '1';
    signal glitch_cnt : integer range 0 to GLITCH_FILTER_CYCLES - 1 := 0;

    attribute mark_debug of wire_s1     : signal is "true";
    attribute mark_debug of wire_s_prev : signal is "true";
    attribute mark_debug of wire_clean  : signal is "true";

begin

    link_wire <= '0' when drive0 = '1' else 'Z';

    drive0           <= drive0_m     when is_master = '1' else drive0_s;
    handoff_rx_pulse <= handoff_rx_m when is_master = '1' else handoff_rx_s;
    score_rx_pulse   <= score_rx_m   when is_master = '1' else score_rx_s;
    grst_rx_pulse    <= grst_rx_m    when is_master = '1' else grst_rx_s;

    process(clk, rst)
    begin
        if rst = '1' then
            wire_s0 <= '1'; wire_s1 <= '1'; wire_s_prev <= '1';
            wire_clean <= '1'; wire_clean_prev <= '1'; glitch_cnt <= 0;
        elsif rising_edge(clk) then
            wire_s0     <= to_x01(link_wire);
            wire_s1     <= wire_s0;
            wire_s_prev <= wire_s1;

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

    master_blk : block

        type mstate_t is (S_TX, S_TX_GAP, S_RX_WAIT, S_RX, S_GAP);
        signal mstate : mstate_t := S_TX;

        signal cnt : integer range 0 to RX_TIMEOUT - 1 := 0;

        signal frame_byte : std_logic_vector(7 downto 0) := (others => '0');
        signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');

        signal bit_idx     : integer range 0 to 7 := 0;
        signal bit_timer   : integer range 0 to BIT_CYCLES - 1 := 0;
        signal bit_started : std_logic := '0';
        signal wait_timer  : integer range 0 to BIT_WAIT_TIMEOUT - 1 := 0;

        signal tx_bit_idx  : integer range 0 to 7 := 0;
        signal tx_slot_pos : integer range 0 to BIT_CYCLES - 1 := 0;

        signal pending_handoff : std_logic := '0';
        signal pending_score   : std_logic := '0';
        signal pending_grst    : std_logic := '0';

        signal peer_ack_handoff, peer_ack_score, peer_ack_grst : std_logic := '0';
        signal prev_rx_handoff, prev_rx_score, prev_rx_grst    : std_logic := '0';

        signal rx_confirm : std_logic_vector(7 downto 0) := (others => '0');

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
                if handoff_tx_pulse = '1' then
                    pending_handoff <= '1';
                end if;
                if score_tx_pulse = '1' then
                    pending_score <= '1';
                end if;
                if grst_tx_pulse = '1' then
                    pending_grst <= '1';
                end if;

                handoff_rx_m <= '0';
                score_rx_m   <= '0';
                grst_rx_m    <= '0';

                case mstate is

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

                    when S_TX_GAP =>
                        drive0_m <= '0';
                        if cnt = GAP_CYCLES - 1 then
                            cnt    <= 0;
                            mstate <= S_RX_WAIT;
                        else
                            cnt <= cnt + 1;
                        end if;

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
                            frame_byte      <= "00" & peer_ack_grst & pending_grst & peer_ack_score & peer_ack_handoff & pending_score & pending_handoff;
                            mstate <= S_TX;
                        else
                            cnt <= cnt + 1;
                        end if;

                    when S_RX =>
                        drive0_m <= '0';
                        if bit_started = '0' then
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
                                    bit_started <= '0';
                                    wait_timer  <= 0;
                                end if;
                            else
                                bit_timer <= bit_timer + 1;
                            end if;
                        end if;

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

    slave_blk : block

        type sstate_t is (S_RX_WAIT, S_RX, S_GAP1, S_TX, S_GAP2);
        signal sstate : sstate_t := S_RX_WAIT;

        signal cnt : integer range 0 to FRAME_CYCLES - 1 := 0;

        signal frame_byte : std_logic_vector(7 downto 0) := (others => '0');
        signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');

        signal bit_idx     : integer range 0 to 7 := 0;
        signal bit_timer   : integer range 0 to BIT_CYCLES - 1 := 0;
        signal bit_started : std_logic := '0';
        signal wait_timer  : integer range 0 to BIT_WAIT_TIMEOUT - 1 := 0;

        signal tx_bit_idx  : integer range 0 to 7 := 0;
        signal tx_slot_pos : integer range 0 to BIT_CYCLES - 1 := 0;

        signal pending_handoff : std_logic := '0';
        signal pending_score   : std_logic := '0';
        signal pending_grst    : std_logic := '0';

        signal peer_ack_handoff, peer_ack_score, peer_ack_grst : std_logic := '0';
        signal prev_rx_handoff, prev_rx_score, prev_rx_grst    : std_logic := '0';

        signal rx_confirm : std_logic_vector(7 downto 0) := (others => '0');

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

                    when S_RX_WAIT =>
                        drive0_s <= '0';
                        if wire_clean_prev = '1' and wire_clean = '0' then
                            bit_idx     <= 0;
                            bit_started <= '0';
                            wait_timer  <= 0;
                            sstate      <= S_RX;
                        end if;

                    when S_RX =>
                        drive0_s <= '0';
                        if bit_started = '0' then
                            if wire_clean_prev = '1' and wire_clean = '0' then
                                bit_started <= '1';
                                bit_timer   <= 0;
                                wait_timer  <= 0;
                            elsif wait_timer = BIT_WAIT_TIMEOUT - 1 then
                                sstate <= S_RX_WAIT;
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

                    when S_GAP1 =>
                        drive0_s <= '0';
                        if cnt = SLAVE_REPLY_GAP_CYCLES - 1 then
                            frame_byte      <= "00" & peer_ack_grst & pending_grst & peer_ack_score & peer_ack_handoff & pending_score & pending_handoff;
                            cnt    <= 0;
                            sstate <= S_TX;
                        else
                            cnt <= cnt + 1;
                        end if;

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
