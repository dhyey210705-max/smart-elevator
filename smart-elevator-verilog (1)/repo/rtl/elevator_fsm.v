//=============================================================================
// Module      : elevator_fsm
// Description : Main control FSM. Owns the current-floor counter and the
//               travel-direction register, requests door open/close from
//               door_controller, pulses clear_request_o back to
//               request_manager, and yields to safety_controller overrides.
//
// Design notes:
//  - Every floor arrival funnels back through S_IDLE for a one-cycle
//    re-evaluation (req_at_current_i / next_direction_i are combinational
//    functions of current_floor_o, which only updates on this same clock
//    edge). Routing through IDLE avoids any same-cycle race between the
//    floor counter updating and the request lookup being valid, at the
//    cost of a 1-cycle bounce when the car passes a floor with no request.
//  - DOOR_CLOSE only returns to IDLE once door_is_open_i (the *actual*
//    door_controller output, which stays high while obstructed) reads
//    low - so the FSM can never start moving with the door physically
//    still open, even if the FSM already asked for it to close.
//  - EMERGENCY forces door_open_req_o low so a car stopped mid-shaft can
//    never open its doors; a car already stopped at a floor with the door
//    open will have that door commanded shut. This is a deliberate,
//    debatable trade-off (see safety_controller comments).
//=============================================================================
module elevator_fsm #(
    parameter NUM_FLOORS = 8,
    parameter FLOOR_BITS = 3,
    parameter MOVE_TICKS = 10
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Scheduler interface
    output reg  [FLOOR_BITS-1:0]  current_floor_o,
    output reg  [1:0]             current_direction_o,   // 00=IDLE 01=UP 10=DOWN
    input  wire                   req_at_current_i,
    input  wire [1:0]             next_direction_i,
    output wire                   clear_request_o,

    // Door controller interface
    output reg                    door_open_req_o,
    input  wire                   door_timer_expired_i,
    input  wire                   door_is_open_i,

    // Safety controller interface
    input  wire                   emergency_active_i,
    input  wire                   fire_active_i,
    input  wire                   maintenance_active_i,
    input  wire                   maint_move_up_i,
    input  wire                   maint_move_down_i,

    // Status
    output wire                   moving_up_o,
    output wire                   moving_down_o,
    output wire [3:0]             state_o
);

    localparam S_IDLE        = 4'd0;
    localparam S_MOVE_UP     = 4'd1;
    localparam S_MOVE_DOWN   = 4'd2;
    localparam S_DOOR_OPEN   = 4'd3;
    localparam S_DOOR_WAIT   = 4'd4;
    localparam S_DOOR_CLOSE  = 4'd5;
    localparam S_EMERGENCY   = 4'd6;
    localparam S_FIRE_MODE   = 4'd7;
    localparam S_MAINTENANCE = 4'd8;

    localparam DIR_IDLE = 2'b00;
    localparam DIR_UP   = 2'b01;
    localparam DIR_DOWN = 2'b10;

    reg [3:0]  state, next_state;
    reg [15:0] move_timer;

    assign state_o       = state;
    assign moving_up_o   = (state == S_MOVE_UP);
    assign moving_down_o = (state == S_MOVE_DOWN) ||
                            (state == S_FIRE_MODE && current_floor_o != {FLOOR_BITS{1'b0}});

    // Pulse the clear only for the one cycle where the dwell timer has just
    // expired while we are still in DOOR_WAIT (i.e. right before closing).
    assign clear_request_o = (state == S_DOOR_WAIT) && door_timer_expired_i;

    //-------------------------------------------------------------------
    // Next-state logic (combinational)
    //-------------------------------------------------------------------
    always @(*) begin
        next_state = state;

        if (emergency_active_i) begin
            next_state = S_EMERGENCY;
        end else if (fire_active_i) begin
            next_state = S_FIRE_MODE;
        end else if (maintenance_active_i) begin
            next_state = S_MAINTENANCE;
        end else begin
            case (state)
                S_IDLE: begin
                    if (req_at_current_i)
                        next_state = S_DOOR_OPEN;
                    else if (next_direction_i == DIR_UP)
                        next_state = S_MOVE_UP;
                    else if (next_direction_i == DIR_DOWN)
                        next_state = S_MOVE_DOWN;
                    else
                        next_state = S_IDLE;
                end

                S_MOVE_UP: begin
                    if (current_floor_o == NUM_FLOORS - 1)
                        next_state = S_IDLE;              // defensive boundary guard
                    else if (move_timer == MOVE_TICKS - 1)
                        next_state = S_IDLE;               // arriving; re-evaluate there
                    else
                        next_state = S_MOVE_UP;
                end

                S_MOVE_DOWN: begin
                    if (current_floor_o == {FLOOR_BITS{1'b0}})
                        next_state = S_IDLE;              // defensive boundary guard
                    else if (move_timer == MOVE_TICKS - 1)
                        next_state = S_IDLE;
                    else
                        next_state = S_MOVE_DOWN;
                end

                S_DOOR_OPEN: next_state = S_DOOR_WAIT;

                S_DOOR_WAIT: begin
                    if (door_timer_expired_i)
                        next_state = S_DOOR_CLOSE;
                    else
                        next_state = S_DOOR_WAIT;
                end

                S_DOOR_CLOSE: begin
                    if (!door_is_open_i)
                        next_state = S_IDLE;
                    else
                        next_state = S_DOOR_CLOSE;
                end

                // Reached only once the corresponding *_active_i has
                // already dropped (the if/else chain above re-forces the
                // override state every cycle while it's still active).
                S_EMERGENCY:   next_state = S_IDLE;
                S_FIRE_MODE:   next_state = S_DOOR_CLOSE;   // close door, then resume
                S_MAINTENANCE: next_state = S_IDLE;

                default: next_state = S_IDLE;
            endcase
        end
    end

    //-------------------------------------------------------------------
    // State register
    //-------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    //-------------------------------------------------------------------
    // Datapath: floor counter, move timer, direction, door request
    //-------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_floor_o     <= {FLOOR_BITS{1'b0}};
            current_direction_o <= DIR_IDLE;
            move_timer           <= 16'd0;
            door_open_req_o      <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    move_timer      <= 16'd0;
                    door_open_req_o <= 1'b0;
                    if (!req_at_current_i)
                        current_direction_o <= next_direction_i;
                end

                S_MOVE_UP: begin
                    current_direction_o <= DIR_UP;
                    if (current_floor_o != NUM_FLOORS - 1) begin
                        if (move_timer == MOVE_TICKS - 1) begin
                            current_floor_o <= current_floor_o + 1'b1;
                            move_timer      <= 16'd0;
                        end else begin
                            move_timer <= move_timer + 16'd1;
                        end
                    end
                end

                S_MOVE_DOWN: begin
                    current_direction_o <= DIR_DOWN;
                    if (current_floor_o != {FLOOR_BITS{1'b0}}) begin
                        if (move_timer == MOVE_TICKS - 1) begin
                            current_floor_o <= current_floor_o - 1'b1;
                            move_timer      <= 16'd0;
                        end else begin
                            move_timer <= move_timer + 16'd1;
                        end
                    end
                end

                S_DOOR_OPEN: begin
                    door_open_req_o <= 1'b1;
                end

                S_DOOR_WAIT: begin
                    door_open_req_o <= 1'b1;
                    if (door_timer_expired_i)
                        current_direction_o <= DIR_IDLE;   // re-decided from IDLE next
                end

                S_DOOR_CLOSE: begin
                    door_open_req_o <= 1'b0;
                end

                S_EMERGENCY: begin
                    move_timer      <= 16'd0;
                    door_open_req_o <= 1'b0;
                end

                S_FIRE_MODE: begin
                    if (current_floor_o != {FLOOR_BITS{1'b0}}) begin
                        current_direction_o <= DIR_DOWN;
                        // Force the door shut while the car is in transit,
                        // regardless of what state (e.g. S_DOOR_OPEN/WAIT)
                        // we were interrupted from with door_open_req_o=1.
                        door_open_req_o <= 1'b0;
                        if (move_timer == MOVE_TICKS - 1) begin
                            current_floor_o <= current_floor_o - 1'b1;
                            move_timer      <= 16'd0;
                        end else begin
                            move_timer <= move_timer + 16'd1;
                        end
                    end else begin
                        door_open_req_o <= 1'b1;   // hold door open at ground floor
                    end
                end

                S_MAINTENANCE: begin
                    move_timer      <= 16'd0;
                    door_open_req_o <= 1'b0;
                    if (maint_move_up_i && current_floor_o != NUM_FLOORS - 1)
                        current_floor_o <= current_floor_o + 1'b1;
                    else if (maint_move_down_i && current_floor_o != {FLOOR_BITS{1'b0}})
                        current_floor_o <= current_floor_o - 1'b1;
                end

                default: begin
                    move_timer <= 16'd0;
                end
            endcase
        end
    end

endmodule
