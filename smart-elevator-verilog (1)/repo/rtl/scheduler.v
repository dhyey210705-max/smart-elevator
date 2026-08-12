//=============================================================================
// Module      : scheduler
// Description : Combinational SCAN/LOOK-style request scheduler.
//
//               Rule: keep moving in the current direction as long as there
//               is at least one pending request further along that
//               direction. Only reverse once no request remains ahead.
//               This avoids the classic "elevator changes mind every floor"
//               problem and matches how real elevator dispatchers behave.
//
//               req_above_o / req_below_o are relative to current_floor_i.
//               req_at_current_o is simply the pending bit for the floor
//               the car is on right now.
//=============================================================================
module scheduler #(
    parameter NUM_FLOORS = 8,
    parameter FLOOR_BITS = 3
)(
    input  wire [NUM_FLOORS-1:0] pending_requests_i,
    input  wire [FLOOR_BITS-1:0] current_floor_i,
    input  wire [1:0]            current_direction_i,  // 00=IDLE 01=UP 10=DOWN

    output wire                  req_at_current_o,
    output wire                  req_above_o,
    output wire                  req_below_o,
    output reg  [1:0]            next_direction_o
);

    localparam DIR_IDLE = 2'b00;
    localparam DIR_UP   = 2'b01;
    localparam DIR_DOWN = 2'b10;

    integer i;
    reg above_flag;
    reg below_flag;

    assign req_at_current_o = pending_requests_i[current_floor_i];

    // Scan the pending-requests bitmap relative to the current floor.
    always @(*) begin
        above_flag = 1'b0;
        below_flag = 1'b0;
        for (i = 0; i < NUM_FLOORS; i = i + 1) begin
            if (i > current_floor_i && pending_requests_i[i])
                above_flag = 1'b1;
            if (i < current_floor_i && pending_requests_i[i])
                below_flag = 1'b1;
        end
    end

    assign req_above_o = above_flag;
    assign req_below_o = below_flag;

    // SCAN decision: continue current direction while work remains that
    // way, otherwise reverse, otherwise go idle.
    always @(*) begin
        case (current_direction_i)
            DIR_UP: begin
                if (req_above_o)
                    next_direction_o = DIR_UP;
                else if (req_at_current_o)
                    next_direction_o = DIR_IDLE;
                else if (req_below_o)
                    next_direction_o = DIR_DOWN;
                else
                    next_direction_o = DIR_IDLE;
            end

            DIR_DOWN: begin
                if (req_below_o)
                    next_direction_o = DIR_DOWN;
                else if (req_at_current_o)
                    next_direction_o = DIR_IDLE;
                else if (req_above_o)
                    next_direction_o = DIR_UP;
                else
                    next_direction_o = DIR_IDLE;
            end

            default: begin // DIR_IDLE
                if (req_above_o)
                    next_direction_o = DIR_UP;
                else if (req_below_o)
                    next_direction_o = DIR_DOWN;
                else
                    next_direction_o = DIR_IDLE;
            end
        endcase
    end

endmodule
