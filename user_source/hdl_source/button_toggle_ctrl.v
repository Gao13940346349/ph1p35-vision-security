// Debounced active-low button that toggles a persistent control state once
// per press. A release must be observed before the first press is accepted,
// so a DIP switch left low during power-up cannot change the reset default.
module button_toggle_ctrl #(
    parameter integer DEBOUNCE_BITS = 20,
    parameter         RESET_VALUE   = 1'b0
) (
    input  wire clk,
    input  wire rst_n,
    input  wire button_n,
    output reg  state,
    // One-cycle pulse on each accepted press. The top level merges this with
    // host commands into the shared state, so it cannot be derived from state
    // (a host write would otherwise look like a press).
    output reg  press
);
    reg sync_n_0;
    reg sync_n_1;
    reg stable_n;
    reg armed;
    reg [DEBOUNCE_BITS-1:0] debounce_count;
    reg [DEBOUNCE_BITS-1:0] release_count;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_n_0 <= 1'b1;
            sync_n_1 <= 1'b1;
            stable_n <= 1'b1;
            armed <= 1'b0;
            debounce_count <= {DEBOUNCE_BITS{1'b0}};
            release_count <= {DEBOUNCE_BITS{1'b0}};
            state <= RESET_VALUE;
            press <= 1'b0;
        end else begin
            press <= 1'b0;                      // default: no press this cycle
            sync_n_0 <= button_n;
            sync_n_1 <= sync_n_0;

            if (sync_n_1 == stable_n) begin
                debounce_count <= {DEBOUNCE_BITS{1'b0}};
            end else if (&debounce_count) begin
                debounce_count <= {DEBOUNCE_BITS{1'b0}};
                stable_n <= sync_n_1;
                if (sync_n_1) begin
                    // Released: allow the next complete press.
                    armed <= 1'b1;
                end else if (armed) begin
                    state <= ~state;
                    press <= 1'b1;
                    armed <= 1'b0;
                end
            end else begin
                debounce_count <= debounce_count + 1'b1;
            end

            // Qualify the initial released level for a full debounce interval.
            // The two reset-high synchronizer samples must not arm a DIP that
            // was already left low before power-up.
            if (!armed && sync_n_1) begin
                if (&release_count)
                    armed <= 1'b1;
                else
                    release_count <= release_count + 1'b1;
            end else begin
                release_count <= {DEBOUNCE_BITS{1'b0}};
            end
        end
    end
endmodule

// Shared 1 ms tick; ten independent 8-sample active-low debouncers.
// A held key at power-up is ignored until it has first been released.
module keypad10 #(parameter integer TICK_CYCLES=100000)(
    input wire clk,rst_n,
    input wire [9:0] keys_n,
    output reg [9:0] press
);
    localparam integer CW=TICK_CYCLES<2 ? 1 : $clog2(TICK_CYCLES);
    reg [CW-1:0] tick_count;
    (* async_reg="true" *) reg [9:0] sync0,sync1;
    reg [9:0] stable,armed;
    reg [3:0] count[0:9];
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            tick_count<=0;sync0<=0;sync1<=0;stable<=0;armed<=0;press<=0;
            for(k=0;k<10;k=k+1) count[k]<=0;
        end else begin
            sync0<=keys_n;sync1<=sync0;press<=0;
            if(tick_count==TICK_CYCLES-1) begin
                tick_count<=0;
                for(k=0;k<10;k=k+1) begin
                    if(sync1[k]==stable[k]) count[k]<=0;
                    else if(count[k]==7) begin
                        count[k]<=0;stable[k]<=sync1[k];
                        if(sync1[k]) armed[k]<=1;
                        else begin press[k]<=armed[k];armed[k]<=0;end
                    end else count[k]<=count[k]+1'b1;
                end
            end else tick_count<=tick_count+1'b1;
        end
    end
endmodule

// Independent reset supervisor: never reset this block by its own output.
// At 50 MHz, require 10 ms stable press/release; power-up held stays reset.
module system_reset_key #(parameter integer STABLE_CYCLES=500000)(
    input wire clk, pll_locked, key_n,
    output reg reset_n
);
    localparam integer CW=STABLE_CYCLES<2 ? 1 : $clog2(STABLE_CYCLES);
    (* async_reg="true" *) reg key0,key1;
    reg [CW-1:0] count;
    always @(posedge clk or negedge pll_locked) begin
        if(!pll_locked) begin key0<=0;key1<=0;count<=0;reset_n<=0;end
        else begin
            key0<=key_n;key1<=key0;
            if(key1==reset_n) count<=0;
            else if(count==STABLE_CYCLES-1) begin reset_n<=key1;count<=0;end
            else count<=count+1'b1;
        end
    end
endmodule

// Asynchronous assertion, two local edges before reset release. The raw
// supervisor signal must not drive a synchronous data/enable in another domain.
module local_reset_release(input wire clk, async_reset_n, output wire reset_n);
    (* async_reg="true" *) reg [1:0] release_sync;
    always @(posedge clk or negedge async_reset_n) begin
        if(!async_reset_n) release_sync<=2'b00;
        else release_sync<={release_sync[0],1'b1};
    end
    assign reset_n=release_sync[1];
endmodule
