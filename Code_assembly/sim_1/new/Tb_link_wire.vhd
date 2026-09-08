library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_link_wire is
end tb_link_wire;

architecture sim of tb_link_wire is

    constant CLK_PERIOD : time := 10 ns;

    constant LINK_BIT_FREQ : integer := 1_000_000;

    signal clk_m, clk_s, rst : std_logic := '0';
    signal link_wire : std_logic;

    signal m_handoff_tx, m_score_tx, m_handoff_rx, m_score_rx : std_logic := '0';
    signal s_handoff_tx, s_score_tx, s_handoff_rx, s_score_rx : std_logic := '0';
    signal m_grst_tx, m_grst_rx, s_grst_tx, s_grst_rx         : std_logic := '0';

    signal m_handoff_rx_latch, m_score_rx_latch, m_grst_rx_latch : std_logic := '0';
    signal s_handoff_rx_latch, s_score_rx_latch, s_grst_rx_latch : std_logic := '0';
    signal clear_latches : std_logic := '0';

    signal errors : integer := 0;

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

    link_wire <= 'H';

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

    clk_m <= not clk_m after CLK_PERIOD / 2;
    clk_s <= not clk_s after (CLK_PERIOD / 2) + 1 ns;

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

    stim : process
    begin
        rst <= '1';
        clear_latches <= '1';
        wait for 100 ns;
        rst <= '0';
        clear_latches <= '0';
        wait for 100 ns;

        m_handoff_tx <= '1';
        wait for CLK_PERIOD;
        m_handoff_tx <= '0';
        wait for 200 us;

        check(s_handoff_rx_latch = '1', "master handoff_tx -> slave sees handoff_rx", errors);
        check(s_score_rx_latch = '0', "...and NOT a spurious score_rx", errors);
        check(m_handoff_rx_latch = '0', "master itself must not loop its own event back", errors);
        check(m_score_rx_latch = '0', "...and master's OWN score_rx must not spuriously fire either", errors);

        clear_latches <= '1';
        wait for 100 ns;
        clear_latches <= '0';

        s_score_tx <= '1';
        wait for 3 * CLK_PERIOD;
        s_score_tx <= '0';
        wait for 200 us;

        check(m_score_rx_latch = '1', "slave score_tx -> master sees score_rx", errors);
        check(m_handoff_rx_latch = '0', "...and NOT a spurious handoff_rx", errors);

        clear_latches <= '1';
        wait for 100 ns;
        clear_latches <= '0';

        wait for 200 us;

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
        wait for 200 us;

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
