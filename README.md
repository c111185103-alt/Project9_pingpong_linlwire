# Project 9 — 雙板 FPGA 乒乓球對戰（單線跨板協定）

平台：EGO-XZ7（Zynq-7000, XC7Z020），Vivado 2018.2

兩片 FPGA 開發板各自跑一份 `pingpong_game` 遊戲邏輯，球在 16 顆 LED 上左右移動，玩家按板上按鈕擊球；球飛出自己這端畫面時，透過**只用一條訊號線＋一條共用GND**（教授規定板間佈線上限為 2 條實體線）跨板交接給對面繼續。因為線路被壓到極限，`link_wire` 是一個從零設計、跑在單一雙向 wire 上的輪詢式協定，取代了原本規劃、需要至少 SCL/SDA 兩線的 I2C 方案。

---

## 目錄

1. [專案簡介](#1-專案簡介)
2. [系統架構](#2-系統架構)
3. [通訊協定設計：link_wire](#3-通訊協定設計link_wire)
4. [遊戲邏輯 FSM](#4-遊戲邏輯-fsm)
5. [驗證與模擬結果](#5-驗證與模擬結果)
6. [除錯歷程精選](#6-除錯歷程精選)
7. [建置與模擬方式](#7-建置與模擬方式)

> 本 repo 收錄架構圖、狀態機圖與驗證圖表；圖檔皆以 [draw.io](https://www.diagrams.net/) 建立，原始 `.drawio` 檔一併附上，可自行開啟編輯。**由 draw.io 匯出的圖若在網頁裡看不清楚，圖說下方都附了「🔍 看大圖」連結，點開即為原始解析度的圖檔。**

---

## 1. 專案簡介

- 雙板各自獨立跑遊戲邏輯與本地 7 段顯示器（透過 `I2c_tca6416_driver` 驅動板上的 TCA6416 GPIO expander），僅在「球需要交接給對面」與「分數變動」時才需要跨板同步。
- 板間限定 2 條實體線：1 條共用 GND + 1 條雙向訊號線（`link_wire`），因此協定必須自己處理「誰現在該送、誰現在該收」的仲裁，不能像標準 I2C 一樣靠獨立的 SCL 時脈線同步。
- 兩板各自本地 GRST 重置就已經很穩定；另外實作了「其中一板 GRST，兩板一起重置」的 shared-GRST 功能（見[第 6 節](#6-除錯歷程精選)），方便單人測試時不用兩手同時按兩顆鈕。
- 已驗證的行為：發球、擊球、雙板來回交接可穩定進行多回合，雙板 7 段分數顯示正確。

## 2. 系統架構

頂層 `Pingpong9_hw_top` 由四個子模組組成：

| 子模組 | 角色 |
| --- | --- |
| `debounce`（GRST 用/按鍵用各一份） | 板上按鈕去彈跳 |
| `pingpong_game (U_GAME)` | 遊戲 FSM：球位置、擊球判定、分數 |
| `i2c_tca6416_driver (U_I2C_SEG)` | 本板 7 段顯示器 I2C 驅動 |
| `link_wire_drv (U_LINK_WIRE)` | 跨板單線協定 driver（本專案的核心） |

`U_GAME` 與 `U_LINK_WIRE` 之間交換 `handoff_tx/rx`、`score_tx/rx` 脈衝；`combined_rst`（本地 `rst_level` OR 跨板 `grst_reset_now`）同時餵給這兩個模組。

**頂層方塊圖**（含實際接腳配置：`btn_GRST`=A/B板皆S6、`clk`=Y9 100MHz GCLK、`btn_raw`=A板S9/B板S5、`link_wire`=JA2腳3/Raspberry Pi GPIO2/Y5）：

![頂層方塊圖](Upload/image/方塊圖_電路圖.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/方塊圖_電路圖.drawio.png)

**模組拆解圖**（每個子模組底下再展開各自的 process/generate 分支）：

![模組拆解圖](Upload/image/Breakdown.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/Breakdown.drawio.png)

## 3. 通訊協定設計：link_wire

`link_wire` 是跑在單一開集極（open-drain 風格）雙向線上的輪詢式協定：兩板不斷交換固定長度的 frame，類似 I2C polling，而不是有事件才觸發的中斷式協定。

**Frame 內容（`frame_byte`，8 bit）：**

| bit | 意義 |
| --- | --- |
| bit0 | `handoff`：球交接給對面 |
| bit1 | `score`：對面漏接，我方得分 |
| bit2 | `ack_handoff`：確認收到對面的 handoff |
| bit3 | `ack_score`：確認收到對面的 score |
| bit4 | `pending_grst`：我方剛被本地 GRST，通知對面 |
| bit5 | `peer_ack_grst`：確認收到對面的 GRST 通知 |

每個 bit 用 PWM 方式編碼在一段固定週期裡：`0` 拉低較長（LOW_LONG），`1` 拉低較短（LOW_SHORT），frame 結構是 `START + guard + 8 個 data bit`。`pending_handoff`/`pending_score`/`pending_grst` 要等到對面回傳對應的 ack bit 才會清除——沒收到 ack 就會在下一輪重送，這是一個簡單的 ACK-based retry 機制。收到的 bit 還會做「連續兩輪 frame 讀到同一個值才觸發」的 debounce（`rx_confirm`），避免單一 bit 雜訊誤觸發得分/交接。

### master_gen / slave_gen 狀態機

`link_wire_drv` 內部依 `IS_MASTER` generic 分成兩份幾乎對稱的狀態機：

![master_gen 狀態機](Upload/image/Mstate_FSM.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/Mstate_FSM.drawio.png)

![slave_gen 狀態機](Upload/image/Sstate_FSM.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/Sstate_FSM.drawio.png)

master_gen 與 slave_gen 的 state 機結構對稱：兩者每一輪都是「收 1 個 frame ＋ 送 1 個 frame」，差別只在起始動作的先後順序——master 先送（TX）後收（RX），slave 先收（RX）後送（TX）。若依此對齊比較：master 的 `S_TX_GAP` 對應 slave 的 `S_GAP2`（皆為 TX 之後的 gap），master 的 `S_GAP` 對應 slave 的 `S_GAP1`（皆為 RX 之後的 gap）。

刻意保留兩處不對稱：
1. 只有 master 的 `S_RX_WAIT` 設有 `RX_TIMEOUT` 逾時重啟，slave 等待不設逾時——避免兩邊各自逾時重啟互相打架，只能有一邊負責重啟整個循環。
2. slave 送回覆前的等待用 `SLAVE_REPLY_GAP_CYCLES`（＝`GAP_CYCLES` 的 3 倍），比 master 對應位置的等待更長，確保 master 真的已切到聽的狀態才送出，避免漏收。

### Shared-GRST（一板重置、兩板都重置）

這個功能重用了既有的 bit+ACK+`rx_confirm` frame 機制（bit4/bit5），而不是另外做一條獨立的廣播協定——早期版本用「直接把線拉低一段時間」的獨立廣播機制，結果每片板自己開機時的重置動作會被自己的偵測邏輯誤判成「對面剛重置」，因此整個機制被拿掉重做。新版收到對面 `pending_grst` 後不會立刻重置，而是先等 `GRST_PROPAGATE_DELAY_CYCLES`（預設 100ms）確保自己的 ack bit 已經真的送出去一輪，才觸發本地重置——否則本地重置會連帶重置 `link_wire_drv` 本身，讓 ack 還沒送出就被打斷，對面因收不到 ack 而永遠重送。

## 4. 遊戲邏輯 FSM

`pingpong_game` 6 個狀態，涵蓋發球、球移動、擊球視窗、球在對面板、得分、熄燈停頓：

![遊戲主 FSM](Upload/image/FSM.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/FSM.drawio.png)

## 5. 驗證與模擬結果

三份 testbench 各自負責不同層級：

| Testbench | 驗證範圍 | 建議 `xsim.simulate.runtime` |
| --- | --- | --- |
| `Tb_pingpong_game.vhd` | 單板遊戲 FSM | ~10us |
| `Tb_link_wire.vhd` | link_wire 協定層（含 handoff/score/GRST 雙向交換） | 2200-2500us |
| `Tb_pingpong9_chain.vhd` | 雙板端對端（GRST→發球→跨板交接→得分） | 1900-2000us |

### link_wire 協定層波形（`Tb_link_wire.vhd`）

全景圖：`link_wire` 全程持續 toggling（類似 I2C polling），`clk_m`/`clk_s` 刻意用不同週期模擬兩板獨立振盪器，驗證跨時脈域的 2-FF 同步是否足夠：

![波形全景](Upload/image/annotated_1_overview.png)

第一個 frame 到第一個帶有真實 handoff 資料的 frame（乾淨 frame 結構 → 全零 no-op 輪詢 → t=28~38.9us 真正帶資料的一輪）：

![第一個有效 frame](Upload/image/annotated_2_first_frame.png)

PWM bit 編碼細節（t=24~42us，master 內部訊號展開）：bit0(handoff=1) 為 LOW_SHORT 短脈衝，其餘 bit(=0) 為 LOW_LONG 長脈衝，各 1us 一個 bit：

![bit 編碼細節](Upload/image/annotated_3_bit_encoding.png)

對面（slave）收端解碼過程（t=165~210us）：`sstate` 停在 `S_RX_WAIT` 直到偵測到 master 送出的 frame 邊緣才進 `S_RX`，逐 bit 取樣後在 `S_GAP1` 打包 ack 準備回覆：

![RX 解碼細節](Upload/image/annotated_4_rx_decode.png)

### 雙板端對端時序（`Tb_pingpong9_chain.vhd`）

AOV（Activity-on-Vertex，事件依賴關係）與 TSPEC（實際模擬時間軸）皆取自真實模擬結果，非估算值：

![Chain AOV](Upload/image/Chain_AOV.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/Chain_AOV.drawio.png)

![Chain TSPEC](Upload/image/Chain_TSPEC.drawio.png)
🔍 [看大圖](https://raw.githubusercontent.com/c111185103-alt/Project9_pingpong_linlwire/main/Upload/image/Chain_TSPEC.drawio.png)

實測關鍵時間點（`Tb_pingpong9_chain.vhd`，9/9 checks pass，errors=0）：

| t (ns) | 事件 |
| --- | --- |
| 605 | 兩板 GRST 完成 |
| 1,345 | A 按鈕離手，A 發球 |
| 301,914 | B 收到 handoff，等待擊球 |
| 302,802 | B 按鈕成功接住彈回 |
| 603,275 | A 收到球，等待擊球 |
| 1,003,675 | A 故意不接，`HIT_WINDOW` 逾時漏接 |
| 1,303,675 | B 確認得分（`score_ones_b=1`），測試結束 |

單趟跨板 `link_wire` 往返約 300us，占全程時間約 69%，是整條鏈路的時間瓶頸。此表對應的兩張圖是在 shared-GRST 功能重做（第 6 節 Bug 2）**之前**跑出的結果；新增的 GRST 往返已在 `Tb_link_wire.vhd`/`Tb_pingpong9_chain.vhd` 各自的獨立測試區塊中驗證通過，但尚未回頭更新進這兩張圖的時間軸。

## 6. 除錯歷程精選

### Bug 1：`HIT_WINDOW_CYCLES` 太短，導致自己判定漏接還誤傳給對面加分

雙板端對端測試一開始全部 FAIL，但 LED 畫面完全正常。直接用 `get_value` 探測內部 `state` 訊號才發現：球實際跨板送達需要約 180us 的真實協定時間，遠長於當時沿用單板測試的 `HIT_WINDOW_CYCLES=100`（僅 1us）——接球端在測試腳本按下按鈕之前就已經自己判定漏接，並把這個「假漏接」的 score 正確地透過 `link_wire` 傳給對面，看起來像是對面莫名其妙得分。這在 LED 上完全看不出來，因為「還在等球」跟「已經漏接、回到等待發球」兩個狀態剛好用同一個 LED pattern 顯示。修法：把 `HIT_WINDOW_CYCLES` 依實測延遲重新抓比例（最終收斂到 40,000），並新增一個在「成功接球」後立刻檢查對面分數仍為 0 的回歸測試——原本沒有任何一個既有檢查點，能在這個 bug 重現時抓到它。

### Bug 2：shared-GRST 自己誤判自己的重置為對面重置

Shared-GRST 第一版做法是「重置時把 `link_wire` 拉低一段固定時間，對面偵測到夠長的低電位就跟著重置」。上線後兩片板各自開機時的重置動作，被自己板上的偵測邏輯誤判成「對面剛剛重置」，導致 `pending_handoff`/`pending_score` 在還沒真正送出前就被清空。改用第 3 節描述的新設計——重用既有的 bit+ACK+`rx_confirm` 機制而非獨立廣播——徹底避開了「自己讀到自己線路狀態」這個根因，因為 master/slave 嚴格輪流才能驅動線路，RX 只會在自己指定的收訊狀態才取樣，沒有誤讀自己輸出的窗口。

## 7. 建置與模擬方式

- 工具鏈：Vivado 2018.2（原專案曾用 2022.2 建立，因故改回 2018.2，專案改用 Tcl `add_files`重建）。
- 三份 testbench 各自的 `TopModule` 與建議 `xsim.simulate.runtime` 請參照[第 5 節](#5-驗證與模擬結果)的表格；`.xpr` 的 `sim_1` fileset 只有一組共用 runtime 設定，切換 top module 模擬時記得一併調整。
- `Tb_pingpong9_chain.vhd` 內的 `GRST_PROPAGATE_DELAY_CYCLES` 在模擬中會 override 成 8,000（80us）以加速模擬；實際硬體上的預設值是 10,000,000（100ms）。
