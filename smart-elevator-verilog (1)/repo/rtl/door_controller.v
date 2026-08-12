//=============================================================================
// Module      : door_controller
// Description : Owns the door dwell timer and obstruction safety interlock.
//
//               - door_open_o goes high whenever the FSM requests the door
//                 open (door_open_req_i) OR an obstruction is detected.
//                 The obstruction term means the door physically cannot be
//                 forced shut by the FSM while something is blocking it,
//                 even if the FSM has already moved on to DOOR_CLOSE.
//               - timer_cnt counts dwell time. Any obstruction resets the
//                 counter to 0, so the dwell period restarts every time the
//                 door is blocked (mirrors real elevator "hold door" logic).
//               - timer_expired_o pulses/holds high once the full dwell
//                 time has elapsed with no obstruction present.
//=============================================================================
module door_controller #(
    parameter DOOR_OPEN_CYCLES = 10
)(
    input  wire clk,
    input  wire rst_n,

    input  wire door_open_req_i,   // FSM commands the door to be open
    input  wire obstruction_i,     // sensor: active-high while blocked

    output reg  door_open_o,       // door is physically open
    output reg  timer_expired_o    // dwell time elapsed, no obstruction
);

    reg [15:0] timer_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            door_open_o     <= 1'b0;
            timer_cnt       <= 16'd0;
            timer_expired_o <= 1'b0;
        end else if (door_open_req_i || obstruction_i) begin
            door_open_o <= 1'b1;
            if (obstruction_i) begin
                // Blocked: hold the door open indefinitely, restart dwell.
                timer_cnt       <= 16'd0;
                timer_expired_o <= 1'b0;
            end else if (timer_cnt < DOOR_OPEN_CYCLES - 1) begin
                timer_cnt       <= timer_cnt + 16'd1;
                timer_expired_o <= 1'b0;
            end else begin
                timer_expired_o <= 1'b1;
            end
        end else begin
            // FSM no longer requests the door open and nothing is blocking it.
            door_open_o     <= 1'b0;
            timer_cnt       <= 16'd0;
            timer_expired_o <= 1'b0;
        end
    end

endmodule
