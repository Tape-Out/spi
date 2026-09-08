`ifndef SPIM_MMIO_V
`define SPIM_MMIO_V

`timescale 1ns/1ps

`ifndef SPIM_DEFAULT_CLK_DIV
`define SPIM_DEFAULT_CLK_DIV 100
`endif

`ifndef SPIM_FIFO_DEPTH
`define SPIM_FIFO_DEPTH 16
`endif

/**
 * SPI Master MMIO Module
 *
 * SPI (Serial Peripheral Interface) 是一种同步串行通信协议，主要用于短距离通信
 * 典型应用：传感器、存储器、显示控制器等外设
 *
 * SPI 四线接口：
 * - SCK: 串行时钟（由主机产生）
 * - MOSI: 主机输出，从机输入
 * - MISO: 主机输入，从机输出
 * - CS_N: 片选信号（低有效）
 *
 * SPI 模式由 CPOL 和 CPHA 决定：
 * - Mode 0: CPOL=0, CPHA=0 - 时钟空闲低电平，数据在第一个边沿采样
 * - Mode 1: CPOL=0, CPHA=1 - 时钟空闲低电平，数据在第二个边沿采样
 * - Mode 2: CPOL=1, CPHA=0 - 时钟空闲高电平，数据在第一个边沿采样
 * - Mode 3: CPOL=1, CPHA=1 - 时钟空闲高电平，数据在第二个边沿采样
 */
module spim_mmio #(
    parameter [31:0]  BASE_ADDR     = 32'h8100_2000,      // 模块基地址
    parameter [31:0]  CLK_FREQ      = 32'd100_000_000,    // 系统时钟频率 100MHz
    parameter integer DEFAULT_CLK_DIV = `SPIM_DEFAULT_CLK_DIV,  // 默认时钟分频
    parameter integer FIFO_DEPTH    = `SPIM_FIFO_DEPTH,   // FIFO深度
    parameter integer MAX_CLK_DIV   = 32'd10_000          // 最大时钟分频
)(
    input  wire                     clk,                  // 系统时钟
    input  wire                     resetn,               // 低有效复位

    // Memory Mapped IO 接口
    input  wire                     mem_valid,            // 内存访问有效
    input  wire                     mem_instr,            // 指令访问（通常忽略）
    output reg                      mem_ready,            // 访问完成
    input  wire [31:0]              mem_addr,             // 访问地址
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,            // 写入数据
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,            // 字节使能
    output reg  [31:0]              mem_rdata,            // 读取数据

    // SPI 物理接口
    output reg                      spi_sck,              // SPI 串行时钟
    output reg                      spi_mosi,             // SPI 主机输出从机输入
    input  wire                     spi_miso,             // SPI 主机输入从机输出
    output reg                      spi_cs_n,             // SPI 片选（低有效）

    // 中断接口
    output reg                      irq,                  // 中断请求
    input  wire                     eoi                   // 中断结束确认
);
    reg irq_next;

    // 寄存器定义
    reg [31:0] ctrl_reg;           // 控制寄存器
    reg [31:0] clk_div_reg;        // 时钟分频寄存器
    reg [31:0] status_reg;         // 状态寄存器

    // FIFO 存储器
    reg [7:0]  tx_fifo [0:FIFO_DEPTH-1];  // 发送FIFO
    reg [7:0]  rx_fifo [0:FIFO_DEPTH-1];  // 接收FIFO

    // FIFO 指针和计数器
    reg [$clog2(FIFO_DEPTH)-1:0] tx_rd_ptr, tx_wr_ptr;  // TX FIFO 读/写指针
    reg [$clog2(FIFO_DEPTH)-1:0] rx_wr_ptr, rx_rd_ptr;  // RX FIFO 写/读指针
    reg [$clog2(FIFO_DEPTH)  :0] tx_count , rx_count ;  // FIFO 数据计数

    // SPI 传输控制信号
    reg [7:0]   tx_shift;          // 发送移位寄存器
    reg [7:0]   rx_shift;          // 接收移位寄存器
    reg [3:0]   bit_cnt;           // 位计数器 (0-7)
    reg         transfer_active;   // 传输进行中标志
    reg [31:0]  clk_div_cnt;       // 时钟分频计数器
    reg         sck_phase;         // SCK 相位: 0=第一相位, 1=第二相位
    reg         cs_active;         // 片选激活标志

    // 时钟分频完成标志
    wire clk_tick = (clk_div_cnt == 0);

    // TX FIFO 计数增量（当写入TX数据时）
    wire [$clog2(FIFO_DEPTH):0] tx_count_inc;

    // 写掩码和数据预处理
    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    // 寄存器地址映射
    localparam [31:0]
        SPIM_TX_DATA        = BASE_ADDR + 32'h00,  // 发送数据寄存器
        SPIM_RX_DATA        = BASE_ADDR + 32'h04,  // 接收数据寄存器
        SPIM_STATUS         = BASE_ADDR + 32'h08,  // 状态寄存器
        SPIM_CTRL           = BASE_ADDR + 32'h0C,  // 控制寄存器
        SPIM_CLK_DIV        = BASE_ADDR + 32'h10,  // 时钟分频寄存器
        SPIM_CS_SETUP       = BASE_ADDR + 32'h14;  // 片选设置寄存器

    // 控制寄存器位定义
    wire ctrl_spi_en         = ctrl_reg[0];        // SPI 使能
    wire ctrl_cpol           = ctrl_reg[1];        // 时钟极性: 0=空闲低电平, 1=空闲高电平
    wire ctrl_cpha           = ctrl_reg[2];        // 时钟相位: 0=数据在第一个边沿采样, 1=数据在第二个边沿采样
    wire ctrl_lsb_first      = ctrl_reg[3];        // 数据传输顺序: 0=MSB优先, 1=LSB优先
    wire ctrl_auto_cs        = ctrl_reg[4];        // 自动片选控制
    wire ctrl_cs_level       = ctrl_reg[5];        // 片选有效电平: 0=低有效, 1=高有效
    wire ctrl_tx_irq_en      = ctrl_reg[6];        // TX 中断使能
    wire ctrl_rx_irq_en      = ctrl_reg[7];        // RX 中断使能
    wire [2:0] ctrl_cs_sel   = ctrl_reg[10:8];     // 片选选择（支持最多8个从设备）

    // 状态寄存器位定义
    wire status_tx_ready     = (tx_count != FIFO_DEPTH);  // TX FIFO 未满，可以写入
    wire status_rx_ready     = (rx_count != 0);           // RX FIFO 非空，可以读取
    wire status_tx_empty     = (tx_count == 0);           // TX FIFO 为空
    wire status_rx_full      = (rx_count == FIFO_DEPTH);  // RX FIFO 已满
    wire status_transfer_busy= transfer_active;           // SPI 传输进行中
    wire status_tx_overrun   = status_reg[5];             // TX FIFO 溢出
    wire status_rx_overrun   = status_reg[6];             // RX FIFO 溢出

    // 状态寄存器组合
    wire [31:0] status_wire = {
        24'b0,                      // 高位保留
        status_rx_overrun,          // bit 6: RX 溢出
        status_tx_overrun,          // bit 5: TX 溢出
        status_transfer_busy,       // bit 4: 传输忙
        status_rx_full,             // bit 3: RX FIFO 满
        status_tx_empty,            // bit 2: TX FIFO 空
        status_rx_ready,            // bit 1: RX 数据就绪
        status_tx_ready             // bit 0: TX 准备好
    };

    // 零扩展到32位函数（用于FIFO计数）
    function [31:0] zext32;
        input [$clog2(FIFO_DEPTH):0] in;
        begin
            zext32 = 32'b0;
            zext32[$clog2(FIFO_DEPTH):0] = in;
        end
    endfunction

    // 零扩展到32位函数（用于8位数据）
    function [31:0] zext32_8;
        input [7:0] in;
        begin
            zext32_8 = {24'b0, in};
        end
    endfunction

    // =========================================================================
    // SPI 时钟分频器
    // =========================================================================
    /**
     * 时钟分频原理：
     * - 根据 clk_div_reg 对系统时钟进行分频
     * - 产生 SPI 传输所需的 SCK 时钟
     * - 每个 SPI 位需要两个相位，因此实际 SCK 频率 = 系统时钟 / (2 * clk_div_reg)
     */
    always @(posedge clk) begin: CLK_DIVIDER
        if (!resetn)
            clk_div_cnt <= 0;
        else if (ctrl_spi_en && transfer_active) begin
            if (clk_div_cnt == 0)
                clk_div_cnt <= (clk_div_reg == 0 ? 1 : clk_div_reg);
            else
                clk_div_cnt <= clk_div_cnt - 1;
        end
    end

    // TX FIFO 写入计数增量
    assign tx_count_inc = (mem_valid && !mem_instr && mem_addr == SPIM_TX_DATA &&
                          (zext32(tx_count) < FIFO_DEPTH)) ? 1 : 0;

    // =========================================================================
    // SPI 传输状态机 - SPI 协议核心
    // =========================================================================
    /**
     * SPI 数据传输过程：
     * 1. 检测到 TX FIFO 有数据且不处于传输状态时开始传输
     * 2. 拉低 CS_N 信号选中从设备
     * 3. 根据 CPOL 和 CPHA 设置初始时钟状态
     * 4. 每个 SPI 位传输分为两个相位：
     *    - 第一相位：设置 MOSI 数据，切换 SCK
     *    - 第二相位：采样 MISO 数据，切换 SCK
     * 5. 传输完8位后，拉高 CS_N 结束传输
     * 6. 将接收到的数据存入 RX FIFO
     */
    always @(posedge clk) begin: SPI_TRANSFER
        if (!resetn) begin
            // 复位状态
            spi_sck <= 1'b0;
            spi_mosi <= 1'b0;
            spi_cs_n <= 1'b1;           // 片选默认无效（高电平）
            transfer_active <= 0;
            bit_cnt <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            sck_phase <= 0;
            cs_active <= 0;
            tx_rd_ptr <= 0;
            rx_wr_ptr <= 0;
            tx_count <= 0;
            rx_count <= 0;
            status_reg <= 0;
        end else if (ctrl_spi_en) begin
            // 片选信号控制
            if (ctrl_auto_cs) begin
                // 自动片选模式：传输期间自动控制 CS_N
                spi_cs_n <= ~(cs_active && (ctrl_cs_sel == 0)); // 简化版：只支持 CS0
            end else begin
                // 手动片选模式
                spi_cs_n <= ~ctrl_cs_level;
            end

            // 检查是否可以开始新的传输
            if (!transfer_active && tx_count > 0) begin: START_TRANSFER
                // 开始新的 SPI 传输
                transfer_active <= 1'b1;
                cs_active <= 1'b1;           // 激活片选
                tx_shift <= tx_fifo[tx_rd_ptr];  // 从 TX FIFO 读取数据
                tx_rd_ptr <= tx_rd_ptr + 1;      // 移动读指针
                tx_count <= tx_count - 1 + tx_count_inc;  // 更新 TX 计数
                bit_cnt <= 0;                  // 重置位计数器
                sck_phase <= 0;                // 从第一相位开始

                // 根据 CPOL 和 CPHA 设置初始时钟状态
                // CPOL=0,CPHA=0: 起始时钟为0; CPOL=0,CPHA=1: 起始时钟为0
                // CPOL=1,CPHA=0: 起始时钟为1; CPOL=1,CPHA=1: 起始时钟为1
                spi_sck <= ctrl_cpol ^ ctrl_cpha;
            end else if (transfer_active && clk_tick) begin
                // SPI 时钟节拍到达，进行位传输
                if (!sck_phase) begin: FIRST_PHASE
                    // 第一相位：设置输出数据并切换时钟
                    if (ctrl_lsb_first) begin
                        // LSB 优先：发送最低位
                        spi_mosi <= tx_shift[0];
                        tx_shift <= {1'b0, tx_shift[7:1]};  // 右移
                    end else begin
                        // MSB 优先：发送最高位
                        spi_mosi <= tx_shift[7];
                        tx_shift <= {tx_shift[6:0], 1'b0};  // 左移
                    end
                    sck_phase <= 1'b1;  // 进入第二相位

                    // 切换时钟：根据 CPHA 决定时钟边沿
                    // CPHA=0: 数据在第一个边沿采样，此时产生时钟边沿
                    // CPHA=1: 数据在第二个边沿采样，此时保持时钟
                    spi_sck <= ctrl_cpol ^ ~ctrl_cpha;
                end else begin: SECOND_PHASE
                    // 第二相位：采样输入数据并切换时钟
                    if (ctrl_lsb_first) begin
                        // LSB 优先：接收数据到最高位
                        rx_shift <= {spi_miso, rx_shift[7:1]};  // 右移并入
                    end else begin
                        // MSB 优先：接收数据到最低位
                        rx_shift <= {rx_shift[6:0], spi_miso};  // 左移并入
                    end
                    sck_phase <= 1'b0;  // 回到第一相位
                    spi_sck <= ctrl_cpol ^ ctrl_cpha;  // 恢复时钟到空闲状态

                    bit_cnt <= bit_cnt + 1;  // 增加位计数

                    // 检查是否完成8位传输
                    if (bit_cnt == 7) begin: TRANSFER_COMPLETE
                        transfer_active <= 0;  // 结束传输
                        cs_active <= 0;        // 取消片选

                        // 将接收到的数据保存到 RX FIFO
                        if (zext32(rx_count) < FIFO_DEPTH) begin
                            rx_fifo[rx_wr_ptr] <= rx_shift;
                            rx_wr_ptr <= rx_wr_ptr + 1;
                            rx_count <= rx_count + 1;
                        end else begin
                            // RX FIFO 已满，报告溢出错误
                            status_reg[6] <= 1'b1; // RX overrun
                        end
                    end
                end
            end else begin
                // 更新 TX FIFO 计数（处理并发写入）
                tx_count <= tx_count + tx_count_inc;
            end
        end else begin
            // SPI 未使能，保持默认状态
            spi_sck <= 1'b0;
            spi_mosi <= 1'b0;
            spi_cs_n <= 1'b1;
        end
    end

    // =========================================================================
    // 中断生成逻辑
    // =========================================================================
    always @(*) begin: IRQ_GEN
        irq_next = 0;
        if (ctrl_spi_en) begin
            // 在以下情况下产生中断：
            // - TX 就绪且中断使能
            // - RX 就绪且中断使能
            // - 发生溢出错误
            if ((ctrl_tx_irq_en && status_tx_ready) ||
                (ctrl_rx_irq_en && status_rx_ready) ||
                (status_tx_overrun || status_rx_overrun)) begin
                irq_next = 1;
            end
        end
    end

    // 中断触发器
    always @(posedge clk) begin: IRQ_FF
        if (!resetn)
            irq <= 0;
        else
            irq <= eoi ? 0 : irq_next;  // EOI 信号清除中断
    end

    // =========================================================================
    // 存储器读接口
    // =========================================================================
    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            rx_rd_ptr <= 0;
            mem_rdata <= 0;
            mem_ready <= 0;
        end else begin
            mem_ready <= mem_valid && !mem_instr;  // 非指令访问时准备就绪

            if (mem_valid && (!mem_instr) && mem_wstrb == 0) begin
                // 读操作
                case (mem_addr)
                    SPIM_TX_DATA: begin
                        // 读取 TX FIFO 剩余空间
                        mem_rdata <= zext32(tx_count);
                    end
                    SPIM_RX_DATA: begin
                        // 读取 RX FIFO 数据
                        if (rx_count > 0) begin
                            mem_rdata <= zext32_8(rx_fifo[rx_rd_ptr]);
                            rx_rd_ptr <= rx_rd_ptr + 1;  // 移动读指针
                            rx_count <= rx_count - 1;    // 减少计数
                        end else begin
                            mem_rdata <= 0;  // RX FIFO 为空时返回0
                        end
                    end
                    SPIM_STATUS:     mem_rdata <= status_wire;  // 状态寄存器
                    SPIM_CTRL:       mem_rdata <= ctrl_reg;     // 控制寄存器
                    SPIM_CLK_DIV:    mem_rdata <= clk_div_reg;  // 时钟分频寄存器
                    SPIM_CS_SETUP:   mem_rdata <= {29'b0, ctrl_cs_sel};  // 片选设置
                    default:         mem_rdata <= 0;            // 默认返回0
                endcase
            end else begin
                mem_rdata <= 0;
            end
        end
    end

    // =========================================================================
    // 存储器写接口
    // =========================================================================
    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            // 复位初始化
            ctrl_reg <= 0;
            clk_div_reg <= CLK_FREQ / (DEFAULT_CLK_DIV * 2); // 计算默认分频值
            tx_wr_ptr <= 0;
            status_reg <= 0;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb != 0) begin
                // 写操作
                case(mem_addr)
                    SPIM_TX_DATA: begin
                        // 写入 TX FIFO
                        if (zext32(tx_count) < FIFO_DEPTH) begin
                            tx_fifo[tx_wr_ptr] <= wdata[7:0];  // 存储数据
                            tx_wr_ptr <= tx_wr_ptr + 1;        // 移动写指针
                        end else begin
                            // TX FIFO 已满，报告溢出
                            status_reg[5] <= 1'b1; // TX overrun
                        end
                    end
                    SPIM_CTRL: begin
                        // 写入控制寄存器
                        ctrl_reg[10:0] <= wdata[10:0];  // 只使用低11位
                    end
                    SPIM_CLK_DIV: begin
                        // 写入时钟分频寄存器
                        clk_div_reg <= wdata;
                    end
                    SPIM_STATUS: begin
                        // 写1清除状态标志
                        if (wdata[5]) status_reg[5] <= 0; // Clear TX overrun
                        if (wdata[6]) status_reg[6] <= 0; // Clear RX overrun
                    end
                    default: ; // 忽略其他地址
                endcase
            end
        end
    end

endmodule

`endif
