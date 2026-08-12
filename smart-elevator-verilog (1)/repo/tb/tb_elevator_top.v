//=============================================================================
// Module      : tb_elevator_top
// Description : Self-checking testbench for the Smart Elevator Controller.
//               Every test section prints an explicit PASS/FAIL for each
//               assertion via the check() task, plus a final summary.
//
// Simulation speed: MOVE_TICKS and DOOR_OPEN_CYCLES are overridden to small
// values (5 and 4) purely so the regression finishes in a reasonable number
// of cycles. The RTL parameters themselves default to 10/10 for a more
// realistic simulation - override at instantiation, same as here, for any
// timing-sensitive re-check.
//=============================================================================
`timescale 1ns/1ps

module tb_elevator_top;

    localparam TB_NUM_FLOORS       = 8;
    localparam TB_FLOOR_BITS       = 3;
    localparam TB_MOVE_TICKS       = 5;
    localparam TB_DOOR_OPEN_CYCLES = 4;

    // FSM state codes (must mirror elevator_fsm.v localparams)
    localparam S_IDLE        = 4'd0;
    localparam S_MOVE_UP     = 4'd1;
    localparam S_MOVE_DOWN   = 4'd2;
    localparam S_DOOR_OPEN   = 4'd3;
    localparam S_DOOR_WAIT   = 4'd4;
    localparam S_DOOR_CLOSE  = 4'd5;
    localparam S_EMERGENCY   = 4'd6;
    localparam S_FIRE_MODE   = 4'd7;
    localparam S_MAINTENANCE = 4'd8;

    reg clk;
    reg rst_n;

    reg [TB_NUM_FLOORS-1:0] floor_request_i;
    reg [TB_NUM_FLOORS-1:0] up_request_i;
    reg [TB_NUM_FLOORS-1:0] down_request_i;

    reg door_obstruction_i;
    reg emergency_stop_i;
    reg fire_mode_i;
    reg maintenance_mode_i;
    reg maint_move_up_i;
    reg maint_move_down_i;

    wire [TB_FLOOR_BITS-1:0] current_floor_o;
    wire                     moving_up_o;
    wire                     moving_down_o;
    wire                     door_open_o;
    wire                     alarm_o;
    wire [3:0]                state_o;

    integer pass_count;
    integer fail_count;

    //---------------------------------------------------------------
    // DUT
    //---------------------------------------------------------------
    elevator_top #(
        .NUM_FLOORS       (TB_NUM_FLOORS),
        .FLOOR_BITS       (TB_FLOOR_BITS),
        .MOVE_TICKS        (TB_MOVE_TICKS),
        .DOOR_OPEN_CYCLES  (TB_DOOR_OPEN_CYCLES)
    ) dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .floor_request_i     (floor_request_i),
        .up_request_i        (up_request_i),
        .down_request_i      (down_request_i),
        .door_obstruction_i  (door_obstruction_i),
        .emergency_stop_i    (emergency_stop_i),
        .fire_mode_i         (fire_mode_i),
        .maintenance_mode_i  (maintenance_mode_i),
        .maint_move_up_i     (maint_move_up_i),
        .maint_move_down_i   (maint_move_down_i),
        .current_floor_o     (current_floor_o),
        .moving_up_o         (moving_up_o),
        .moving_down_o       (moving_down_o),
        .door_open_o         (door_open_o),
        .alarm_o             (alarm_o),
        .state_o             (state_o)
    );

    //---------------------------------------------------------------
    // Clock: 10ns period
    //---------------------------------------------------------------
    initial clk = 1'b0;
    always #5 clk = ~clk;

    //---------------------------------------------------------------
    // Global watchdog - catches a hung simulation outright
    //---------------------------------------------------------------
    initial begin
        #2_000_000;
        $display("\n*** GLOBAL WATCHDOG TIMEOUT - simulation hung ***");
        $display("=== SUMMARY: %0d PASSED, %0d FAILED (incomplete run) ===", pass_count, fail_count);
        $finish;
    end

    //---------------------------------------------------------------
    // Basic check task
    //---------------------------------------------------------------
    task check(input integer actual, input integer expected);
        begin
            if (actual === expected) begin
                pass_count = pass_count + 1;
                $display("  PASS  (got %0d, expected %0d)", actual, expected);
            end else begin
                fail_count = fail_count + 1;
                $display("  FAIL  (got %0d, expected %0d)  @ time %0t", actual, expected, $time);
            end
        end
    endtask

    //---------------------------------------------------------------
    // Stimulus helper tasks
    //---------------------------------------------------------------
    task reset_dut;
        begin
            rst_n               = 1'b0;
            floor_request_i     = {TB_NUM_FLOORS{1'b0}};
            up_request_i        = {TB_NUM_FLOORS{1'b0}};
            down_request_i      = {TB_NUM_FLOORS{1'b0}};
            door_obstruction_i  = 1'b0;
            emergency_stop_i    = 1'b0;
            fire_mode_i         = 1'b0;
            maintenance_mode_i  = 1'b0;
            maint_move_up_i     = 1'b0;
            maint_move_down_i   = 1'b0;
            repeat (3) @(posedge clk);
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    task press_floor(input integer f);
        begin
            floor_request_i[f] = 1'b1;
            @(posedge clk);
            floor_request_i[f] = 1'b0;
            #1;   // let the DUT's nonblocking updates from this edge settle
        end
    endtask

    task press_up(input integer f);
        begin
            up_request_i[f] = 1'b1;
            @(posedge clk);
            up_request_i[f] = 1'b0;
            #1;
        end
    endtask

    task press_down(input integer f);
        begin
            down_request_i[f] = 1'b1;
            @(posedge clk);
            down_request_i[f] = 1'b0;
            #1;
        end
    endtask

    task press_maint_up;
        begin
            maint_move_up_i = 1'b1;
            @(posedge clk);
            maint_move_up_i = 1'b0;
            @(posedge clk);
        end
    endtask

    task press_maint_down;
        begin
            maint_move_down_i = 1'b1;
            @(posedge clk);
            maint_move_down_i = 1'b0;
            @(posedge clk);
        end
    endtask

    task wait_cycles(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1)
                @(posedge clk);
        end
    endtask

    task wait_for_floor(input integer target_floor, input integer timeout_cycles, output integer timed_out);
        integer t;
        begin
            t = 0;
            timed_out = 0;
            while ((current_floor_o !== target_floor) && (t < timeout_cycles)) begin
                @(posedge clk);
                t = t + 1;
            end
            if (current_floor_o !== target_floor)
                timed_out = 1;
        end
    endtask

    task wait_for_state(input [3:0] target_state, input integer timeout_cycles, output integer timed_out);
        integer t;
        begin
            t = 0;
            timed_out = 0;
            while ((state_o !== target_state) && (t < timeout_cycles)) begin
                @(posedge clk);
                t = t + 1;
            end
            if (state_o !== target_state)
                timed_out = 1;
        end
    endtask

    task wait_door_open(input integer timeout_cycles, output integer timed_out);
        integer t;
        begin
            t = 0;
            timed_out = 0;
            while ((door_open_o !== 1'b1) && (t < timeout_cycles)) begin
                @(posedge clk);
                t = t + 1;
            end
            if (door_open_o !== 1'b1)
                timed_out = 1;
        end
    endtask

    task wait_door_closed(input integer timeout_cycles, output integer timed_out);
        integer t;
        begin
            t = 0;
            timed_out = 0;
            while ((door_open_o !== 1'b0) && (t < timeout_cycles)) begin
                @(posedge clk);
                t = t + 1;
            end
            if (door_open_o !== 1'b0)
                timed_out = 1;
        end
    endtask

    // Waits for arrival at target_floor AND a full door-open/door-close
    // service cycle. Use only for floors that are actually expected to
    // be serviced (i.e. genuinely requested).
    task goto_floor(input integer target_floor, input integer timeout_cycles);
        integer to1, to2, to3;
        begin
            wait_for_floor(target_floor, timeout_cycles, to1);
            $display("  -> arrived floor %0d (timeout=%0d)", current_floor_o, to1);
            check(to1, 0);
            wait_door_open(30, to2);
            check(to2, 0);
            wait_door_closed(30, to3);
            check(to3, 0);
        end
    endtask

    //=================================================================
    // MAIN TEST SEQUENCE
    //=================================================================
    integer to;
    integer i;
    integer open_cycles;
    integer rand_floor;
    integer rand_kind;

    initial begin
        pass_count = 0;
        fail_count = 0;
        $dumpfile("elevator_waves.vcd");
        $dumpvars(0, tb_elevator_top);

        //-------------------------------------------------------
        $display("\n=== TEST 1: Reset behavior ===");
        reset_dut;
        check(state_o, S_IDLE);
        check(current_floor_o, 0);
        check(door_open_o, 0);
        check(moving_up_o, 0);
        check(moving_down_o, 0);
        check(alarm_o, 0);
        check(dut.pending_requests, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 2: Single floor request ===");
        reset_dut;
        press_floor(3);
        goto_floor(3, 100);
        check(dut.pending_requests[3], 0);

        //-------------------------------------------------------
        $display("\n=== TEST 3: Multiple requests above current floor (SCAN order) ===");
        reset_dut;
        press_floor(2);
        press_floor(5);
        press_floor(7);
        goto_floor(2, 100);
        goto_floor(5, 100);
        goto_floor(7, 100);
        check(dut.pending_requests, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 4: Multiple requests below current floor (SCAN order) ===");
        reset_dut;
        press_floor(7);
        goto_floor(7, 100);
        press_floor(5);
        press_floor(2);
        press_floor(0);
        goto_floor(5, 100);
        goto_floor(2, 100);
        goto_floor(0, 100);
        check(dut.pending_requests, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 5: Requests in both directions from IDLE ===");
        reset_dut;
        press_floor(4);
        goto_floor(4, 100);
        press_floor(6);   // above
        press_floor(1);   // below
        // Documented scheduler tie-break: from DIR_IDLE, UP is checked
        // before DOWN, so floor 6 is expected to be served first.
        goto_floor(6, 100);
        goto_floor(1, 100);
        check(dut.pending_requests, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 6: Request arriving while moving ===");
        reset_dut;
        press_floor(7);
        // Let it get moving, then inject a request mid-flight
        wait_for_state(S_MOVE_UP, 20, to);
        check(to, 0);
        wait_cycles(3);
        press_floor(4);
        check(dut.pending_requests[4], 1);   // captured immediately
        goto_floor(4, 100);                   // SCAN stops here first
        goto_floor(7, 100);                   // then continues up
        check(dut.pending_requests, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 7: Duplicate requests ===");
        reset_dut;
        press_floor(5);
        press_floor(5);   // duplicate press before service
        goto_floor(5, 100);
        check(dut.pending_requests[5], 0);
        // No further motion should occur - confirm it stays idle
        wait_cycles(10);
        check(moving_up_o, 0);
        check(moving_down_o, 0);
        check(state_o, S_IDLE);

        //-------------------------------------------------------
        $display("\n=== TEST 8: Door dwell timer ===");
        reset_dut;
        press_floor(3);
        wait_for_floor(3, 100, to);
        check(to, 0);
        wait_door_open(30, to);
        check(to, 0);
        // Count cycles the door stays open (unobstructed)
        open_cycles = 0;
        while (door_open_o === 1'b1) begin
            @(posedge clk);
            open_cycles = open_cycles + 1;
        end
        $display("  measured open duration = %0d cycles (DOOR_OPEN_CYCLES=%0d)", open_cycles, TB_DOOR_OPEN_CYCLES);
        // Tolerance-based check: bounded by the fixed 2-cycle FSM<->door
        // handshake latency on each side, not an exact cycle-count guess.
        check((open_cycles >= TB_DOOR_OPEN_CYCLES) && (open_cycles <= TB_DOOR_OPEN_CYCLES + 4), 1);

        //-------------------------------------------------------
        $display("\n=== TEST 9: Door obstruction holds the door open ===");
        reset_dut;
        press_floor(2);
        wait_for_floor(2, 100, to);
        check(to, 0);
        wait_door_open(30, to);
        check(to, 0);
        // Obstruct partway through the dwell
        wait_cycles(2);
        door_obstruction_i = 1'b1;
        wait_cycles(6);                 // hold well past the base dwell time
        check(door_open_o, 1);           // must still be open
        door_obstruction_i = 1'b0;
        wait_door_closed(30, to);
        check(to, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 10: Emergency stop ===");
        reset_dut;
        press_floor(7);
        wait_for_state(S_MOVE_UP, 20, to);
        check(to, 0);
        wait_cycles(2);
        emergency_stop_i = 1'b1;
        @(posedge clk);
        @(posedge clk);
        check(state_o, S_EMERGENCY);
        check(moving_up_o, 0);
        check(moving_down_o, 0);
        check(door_open_o, 0);
        check(alarm_o, 1);
        emergency_stop_i = 1'b0;
        wait_for_state(S_IDLE, 20, to);
        check(to, 0);
        check(alarm_o, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 11: Fire mode recalls to ground floor ===");
        reset_dut;
        press_floor(6);
        goto_floor(6, 100);
        press_floor(2);           // a distractor request fire mode should ignore
        fire_mode_i = 1'b1;
        wait_for_floor(0, 100, to);
        check(to, 0);
        wait_door_open(30, to);
        check(to, 0);
        check(alarm_o, 1);
        // Door should hold open indefinitely while fire mode is active
        wait_cycles(TB_DOOR_OPEN_CYCLES + 5);
        check(door_open_o, 1);
        fire_mode_i = 1'b0;
        wait_door_closed(30, to);
        check(to, 0);
        wait_for_state(S_IDLE, 20, to);
        check(to, 0);
        check(alarm_o, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 11b: Fire mode triggered while door is open mid-dwell ===");
        // Regression test for a bug where S_FIRE_MODE only forced the door
        // shut once the car reached the ground floor - if fire_mode_i was
        // asserted while the door was already open/dwelling away from
        // floor 0, door_open_req_o kept its stale value of 1 and the car
        // began moving down with the door still commanded open.
        reset_dut;
        press_floor(5);
        wait_for_floor(5, 100, to);
        check(to, 0);
        wait_door_open(30, to);
        check(to, 0);
        @(posedge clk);   // stay mid-dwell, well before the timer expires
        fire_mode_i = 1'b1;
        // Wait until the car actually leaves floor 5.
        wait_for_state(S_MOVE_DOWN, 10, to);
        // Some designs bounce through S_FIRE_MODE without ever reporting
        // S_MOVE_DOWN; fall back to watching current_floor_o directly.
        wait_cycles(2);
        while (current_floor_o == 5) @(posedge clk);
        check(door_open_o, 0);   // door must be shut before/while moving
        wait_cycles(3);
        check(door_open_o, 0);   // and must stay shut for the whole descent
        wait_for_floor(0, 100, to);
        check(to, 0);
        wait_door_open(30, to);
        check(to, 0);
        check(alarm_o, 1);
        fire_mode_i = 1'b0;
        wait_door_closed(30, to);
        check(to, 0);
        wait_for_state(S_IDLE, 20, to);
        check(to, 0);

        //-------------------------------------------------------
        $display("\n=== TEST 12: Maintenance mode (manual jog, requests ignored) ===");
        reset_dut;
        press_floor(5);            // pressed while entering maintenance
        maintenance_mode_i = 1'b1;
        wait_for_state(S_MAINTENANCE, 20, to);
        check(to, 0);
        // Normal requests must not cause motion while in maintenance
        wait_cycles(10);
        check(current_floor_o, 0);
        // Manual jog: up x2, down x1
        press_maint_up;
        press_maint_up;
        check(current_floor_o, 2);
        press_maint_down;
        check(current_floor_o, 1);
        maintenance_mode_i = 1'b0;
        wait_for_state(S_IDLE, 20, to);
        check(to, 0);
        // The earlier cabin request (floor 5) should still be latched
        // and now get served normally.
        check(dut.pending_requests[5], 1);
        goto_floor(5, 100);

        //-------------------------------------------------------
        $display("\n=== TEST 13: Ground-floor boundary ===");
        reset_dut;
        down_request_i[0] = 1'b1;   // physically meaningless button
        wait_cycles(20);
        check(current_floor_o, 0);
        check(moving_down_o, 0);
        down_request_i[0] = 1'b0;

        //-------------------------------------------------------
        $display("\n=== TEST 14: Top-floor boundary ===");
        reset_dut;
        press_floor(TB_NUM_FLOORS - 1);
        goto_floor(TB_NUM_FLOORS - 1, 100);
        up_request_i[TB_NUM_FLOORS - 1] = 1'b1;  // physically meaningless button
        wait_cycles(20);
        check(current_floor_o, TB_NUM_FLOORS - 1);
        check(moving_up_o, 0);
        up_request_i[TB_NUM_FLOORS - 1] = 1'b0;

        //-------------------------------------------------------
        $display("\n=== TEST 15: Randomized requests (no starvation / no lost requests) ===");
        reset_dut;
        for (i = 0; i < 25; i = i + 1) begin
            rand_floor = $random % TB_NUM_FLOORS;
            if (rand_floor < 0) rand_floor = -rand_floor;
            rand_kind  = $random % 3;
            if (rand_kind < 0) rand_kind = -rand_kind;
            case (rand_kind)
                0: press_floor(rand_floor);
                1: press_up(rand_floor);
                default: press_down(rand_floor);
            endcase
            wait_cycles(3 + ($random % 8 < 0 ? -($random % 8) : ($random % 8)));
        end
        // Drain: stop issuing new requests, give it generous time to
        // service everything that was captured.
        wait_cycles(400);
        $display("  final pending_requests bitmap = %b", dut.pending_requests);
        check(dut.pending_requests, 0);

        //=================================================================
        $display("\n=====================================================");
        $display("=== SUMMARY: %0d PASSED, %0d FAILED (out of %0d) ===", pass_count, fail_count, pass_count + fail_count);
        $display("=====================================================");
        if (fail_count == 0)
            $display("*** ALL TESTS PASSED ***");
        else
            $display("*** %0d TEST(S) FAILED - see FAIL lines above ***", fail_count);

        $finish;
    end

endmodule
