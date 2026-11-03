//------------------------------------------------------------------------------
// mode_ctrl : 板载 KEY0 的消抖与按下脉冲
//
// 消抖部分移植自本团队 exp6 项目的 io_debounce.v：
//   按键是机械触点，属异步输入，直接采样有两个问题——
//     1) 亚稳态：先用两级触发器同步；
//     2) 机械抖动：闭合/断开瞬间有几到几十毫秒的电平抖动，不消抖会把
//        一次按键误判成多次。
//   做法是同步后的电平必须连续保持 2^DEBOUNCE_BITS 个周期不变才认可，
//   并在 0->1 的那一拍给出单周期 press 脉冲。
//
// 本模块只负责产生"按下"事件，不再自己保存模式值。模式寄存器和上位机
// 命令的合并都在 control_state 里完成，这样物理按键和上位机命令作用在
// 同一份状态上，不会各自维护一份而失步。
//
// 【时钟选择】
//   本模块须挂在始终运行的时钟上（如 PLL 输出的 100MHz），不能用
//   MIPI_RX_BYTE_CLK —— 后者由摄像头 D-PHY 恢复，摄像头不出流时会停，
//   那样按键将失效。
//------------------------------------------------------------------------------
module mode_ctrl #(
    parameter DEBOUNCE_BITS = 20        // 2^20 @100MHz ≈ 10.5ms
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       key_n,            // 板载按键，低有效（按下为 0）

    output reg        press             // 每次按下给一个单周期脉冲
);

    // 板载按键按下为低，先取反成"按下=1"再消抖
    wire key_active = ~key_n;

    reg sync0, sync1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync0 <= 1'b0;
            sync1 <= 1'b0;
        end else begin
            sync0 <= key_active;
            sync1 <= sync0;
        end
    end

    reg [DEBOUNCE_BITS-1:0] cnt;
    reg                     level;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt   <= {DEBOUNCE_BITS{1'b0}};
            level <= 1'b0;
            press <= 1'b0;
        end else begin
            press <= 1'b0;                      // 默认无脉冲
            if (sync1 == level) begin
                cnt <= {DEBOUNCE_BITS{1'b0}};   // 与当前稳定值一致，不计时
            end else begin
                cnt <= cnt + 1'b1;
                if (cnt == {DEBOUNCE_BITS{1'b1}}) begin
                    level <= sync1;
                    if (sync1) press <= 1'b1;   // 仅 0->1（按下）产生脉冲
                    cnt   <= {DEBOUNCE_BITS{1'b0}};
                end
            end
        end
    end

endmodule
