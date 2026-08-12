//=============================================================================
// Module      : request_manager
// Description : Latches cabin (internal) button presses and hall-call
//               up/down buttons into a single "pending requests" bitmap,
//               one bit per floor. A request bit is cleared when the FSM
//               pulses clear_request_i while the elevator is at that floor.
//
//               New requests OR in every cycle; the clear pulse ANDs out
//               only the bit for current_floor_i. Both happen in the same
//               always block so a request arriving at the exact floor being
//               cleared is not accidentally lost (it will simply be OR'd
//               back in on a later cycle if the button is still asserted,
//               since inputs here are treated as level signals).
//=============================================================================
module request_manager #(
    parameter NUM_FLOORS = 8,
    parameter FLOOR_BITS = 3
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire [NUM_FLOORS-1:0] floor_request_i,   // cabin buttons
    input  wire [NUM_FLOORS-1:0] up_request_i,       // hall call - up
    input  wire [NUM_FLOORS-1:0] down_request_i,     // hall call - down

    input  wire [FLOOR_BITS-1:0] current_floor_i,
    input  wire                  clear_request_i,    // 1-cycle pulse from FSM

    output reg  [NUM_FLOORS-1:0] pending_requests_o
);

    wire [NUM_FLOORS-1:0] new_requests;
    wire [NUM_FLOORS-1:0] clear_mask;

    // Combine all three request sources. Duplicate/overlapping requests
    // (e.g. cabin + hall call for the same floor) collapse naturally since
    // this is a bitwise OR into a single bit per floor.
    assign new_requests = floor_request_i | up_request_i | down_request_i;

    // One-hot mask selecting current_floor_i, only active while clear_request_i
    // is asserted.
    assign clear_mask = clear_request_i ? ({{(NUM_FLOORS-1){1'b0}}, 1'b1} << current_floor_i)
                                         : {NUM_FLOORS{1'b0}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pending_requests_o <= {NUM_FLOORS{1'b0}};
        else
            pending_requests_o <= (pending_requests_o | new_requests) & ~clear_mask;
    end

endmodule
