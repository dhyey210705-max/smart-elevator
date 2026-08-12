//=============================================================================
// Module      : elevator_top
// Description : Top-level integration for the Smart Elevator Controller.
//               Wires together the request manager, scheduler, door
//               controller, safety controller and the main FSM.
//
// Parameters  :
//   NUM_FLOORS       - Number of floors served (floors 0 .. NUM_FLOORS-1).
//   FLOOR_BITS       - Width of the floor index bus. Must satisfy
//                       FLOOR_BITS >= ceil(log2(NUM_FLOORS)). Kept as an
//                       explicit parameter (instead of computed with
//                       $clog2) to stay within strict Verilog-2001 syntax -
//                       if you change NUM_FLOORS, update FLOOR_BITS too.
//   MOVE_TICKS       - Clock cycles to travel between two adjacent floors.
//   DOOR_OPEN_CYCLES - Door dwell time in clock cycles once fully open and
//                       unobstructed.
//=============================================================================
module elevator_top #(
    parameter NUM_FLOORS       = 8,
    parameter FLOOR_BITS       = 3,
    parameter MOVE_TICKS       = 10,
    parameter DOOR_OPEN_CYCLES = 10
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Cabin (internal) floor buttons, one bit per floor
    input  wire [NUM_FLOORS-1:0]   floor_request_i,
    // Hall call buttons
    input  wire [NUM_FLOORS-1:0]   up_request_i,
    input  wire [NUM_FLOORS-1:0]   down_request_i,

    // Safety / sensor / mode inputs
    input  wire                    door_obstruction_i,
    input  wire                    emergency_stop_i,
    input  wire                    fire_mode_i,
    input  wire                    maintenance_mode_i,
    input  wire                    maint_move_up_i,
    input  wire                    maint_move_down_i,

    // Status outputs
    output wire [FLOOR_BITS-1:0]   current_floor_o,
    output wire                    moving_up_o,
    output wire                    moving_down_o,
    output wire                    door_open_o,
    output wire                    alarm_o,
    output wire [3:0]              state_o
);

    // ---------------------------------------------------------------
    // Internal interconnect
    // ---------------------------------------------------------------
    wire [NUM_FLOORS-1:0] pending_requests;
    wire                  clear_request;

    wire                  req_at_current;
    wire                  req_above_dbg;   // available for future debug/UI use
    wire                  req_below_dbg;   // available for future debug/UI use
    wire [1:0]             next_direction;
    wire [1:0]             current_direction;

    wire                  door_open_req;
    wire                  door_timer_expired;

    wire                  emergency_active;
    wire                  fire_active;
    wire                  maintenance_active;

    // ---------------------------------------------------------------
    // Request storage
    // ---------------------------------------------------------------
    request_manager #(
        .NUM_FLOORS (NUM_FLOORS),
        .FLOOR_BITS (FLOOR_BITS)
    ) u_request_manager (
        .clk                (clk),
        .rst_n              (rst_n),
        .floor_request_i    (floor_request_i),
        .up_request_i       (up_request_i),
        .down_request_i     (down_request_i),
        .current_floor_i    (current_floor_o),
        .clear_request_i    (clear_request),
        .pending_requests_o (pending_requests)
    );

    // ---------------------------------------------------------------
    // SCAN-style request scheduler
    // ---------------------------------------------------------------
    scheduler #(
        .NUM_FLOORS (NUM_FLOORS),
        .FLOOR_BITS (FLOOR_BITS)
    ) u_scheduler (
        .pending_requests_i  (pending_requests),
        .current_floor_i     (current_floor_o),
        .current_direction_i (current_direction),
        .req_at_current_o    (req_at_current),
        .req_above_o         (req_above_dbg),
        .req_below_o         (req_below_dbg),
        .next_direction_o    (next_direction)
    );

    // ---------------------------------------------------------------
    // Door controller (open / dwell timer / obstruction)
    // ---------------------------------------------------------------
    door_controller #(
        .DOOR_OPEN_CYCLES (DOOR_OPEN_CYCLES)
    ) u_door_controller (
        .clk             (clk),
        .rst_n           (rst_n),
        .door_open_req_i (door_open_req),
        .obstruction_i   (door_obstruction_i),
        .door_open_o     (door_open_o),
        .timer_expired_o (door_timer_expired)
    );

    // ---------------------------------------------------------------
    // Safety / mode arbitration
    // ---------------------------------------------------------------
    safety_controller u_safety_controller (
        .emergency_stop_i     (emergency_stop_i),
        .fire_mode_i          (fire_mode_i),
        .maintenance_mode_i   (maintenance_mode_i),
        .emergency_active_o   (emergency_active),
        .fire_active_o        (fire_active),
        .maintenance_active_o (maintenance_active),
        .alarm_o              (alarm_o)
    );

    // ---------------------------------------------------------------
    // Main FSM
    // ---------------------------------------------------------------
    elevator_fsm #(
        .NUM_FLOORS (NUM_FLOORS),
        .FLOOR_BITS (FLOOR_BITS),
        .MOVE_TICKS (MOVE_TICKS)
    ) u_elevator_fsm (
        .clk                  (clk),
        .rst_n                (rst_n),

        .current_floor_o      (current_floor_o),
        .current_direction_o  (current_direction),
        .req_at_current_i     (req_at_current),
        .next_direction_i     (next_direction),
        .clear_request_o      (clear_request),

        .door_open_req_o      (door_open_req),
        .door_timer_expired_i (door_timer_expired),
        .door_is_open_i       (door_open_o),

        .emergency_active_i   (emergency_active),
        .fire_active_i        (fire_active),
        .maintenance_active_i (maintenance_active),
        .maint_move_up_i      (maint_move_up_i),
        .maint_move_down_i    (maint_move_down_i),

        .moving_up_o          (moving_up_o),
        .moving_down_o        (moving_down_o),
        .state_o              (state_o)
    );

endmodule
