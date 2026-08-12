//=============================================================================
// Module      : safety_controller
// Description : Centralizes the priority policy between the three override
//               modes so there is exactly one place in the design that
//               decides "who wins".
//
//               Priority (highest to lowest): EMERGENCY > FIRE_MODE > MAINTENANCE.
//
//               Justification: emergency stop is modeled as a direct
//               passenger-safety action (e.g. someone is trapped or hurt)
//               and must halt everything immediately, even ahead of a fire
//               recall. This is a deliberate, debatable design choice -
//               real Phase-I fire service logic in many building codes
//               actually overrides car-level stop requests. Both positions
//               are defensible; this project documents the choice made and
//               why, which is exactly the kind of trade-off worth discussing
//               in an interview.
//=============================================================================
module safety_controller (
    input  wire emergency_stop_i,
    input  wire fire_mode_i,
    input  wire maintenance_mode_i,

    output wire emergency_active_o,
    output wire fire_active_o,
    output wire maintenance_active_o,
    output wire alarm_o
);

    assign emergency_active_o   = emergency_stop_i;
    assign fire_active_o        = fire_mode_i && !emergency_stop_i;
    assign maintenance_active_o = maintenance_mode_i && !emergency_stop_i && !fire_mode_i;
    assign alarm_o              = emergency_stop_i || fire_mode_i;

endmodule
