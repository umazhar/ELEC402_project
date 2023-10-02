module drone_fsm (
    input logic clk,
    input logic reset,
    input logic [1:0] cmd,
    input logic thermal_found,
    input logic [1:0] img_pass,
    input logic return,
    input logic [7:0] altitude,
    input logic err,
    input logic sensor_check,
    input logic man_ctrl,
    input logic target_found,
    input logic charge_status,
    output logic [3:0] state
);

    typedef enum logic [3:0] {
        S_IDLE = 4'b0000,
        S_INIT = 4'b0001,
        S_FAIL = 4'b0010,
        S_TAKEOFF = 4'b0011,
        S_SEARCH = 4'b0100,
        S_IMAGE_CAPTURE = 4'b0101,
        S_RETURN_BASE = 4'b0110,
        S_MANUAL_CONTROL = 4'b0111,
        S_TARGET_FOUND = 4'b1000,
        S_MAINTENANCE = 4'b1001
    } statetype;

    statetype state;
    statetype next_state;

    logic [15:0] timer;  
    parameter SEARCH_TIMER = 16'd1000;  // 1000 cycles

    always_ff @(posedge clk or posedge reset) 
        if (reset)  state <= S_IDLE;
        else        state <= next_state;
    
    always_comb 
        case(current_state)
            S_IDLE: 
                if (cmd == 2'b01) 
                    next_state = S_INIT;

            S_INIT:
                if (sensor_check)
                    next_state = S_TAKEOFF;
                else if (err)
                    next_state = S_FAIL;

            S_FAIL:
                if (cmd == 2'b01) 
                    next_state = S_MAINTENANCE;

            S_TAKEOFF:
                if (altitude > 8'h32)  // If height > 50 (metres), transistion to search mode
                    next_state = S_SEARCH;
                else if (err = 1)
                    next_state = S_FAIL;
                
            S_SEARCH:
                if (thermal_found)
                    next_state = S_TARGET_FOUND;
                else if (target_found)
                    next_state = S_RETURN_BASE;
                else if (err)
                    next_state = S_FAIL;
                else if (man_ctrl)
                    next_state = S_MANUAL_CONTROL

            S_TARGET_FOUND:
                if (thermal_found & img_pass = 2'b10) //if image passes and thermal signature is still 1 
                    next_state = S_IMAGE_CAPTURE;
                if (img_pass = 2'b01) //if image pass fails
                    next_state - S_SEARCH

            //add timer
            S_IMAGE_CAPTURE:
                if (timer >= SEARCH_TIMER)
                    next_state = S_SEARCH;

            S_RETURN_BASE:
                if (charge_status = 1) //drone has reached base and is charging thus it is now in idle 
                    next_state = S_IDLE;
                else if (cmd = 2'b10)
                    next_state = S_MAINTENANCE

            S_MANUAL_CONTROL:
                if (man_ctrl = 0)
                    next_state = S_SEARCH;  // Example transition, adjust as needed
                else if (cmd = 2'b11)
                    next_state = S_MAINTENANCE

            S_MAINTENANCE:
                if (!err)  // Assuming exit from maintenance when no error
                    next_state = S_IDLE;
                if (man_ctrl = 1)
                    next_state = S_MANUAL_CONTROL;
                    
            default: 
                next_state = S_IDLE;  // Default next state

        endcase

    always_comb begin
    case(state)
        S_IDLE:
            output_signal = 4'b0001;
        S_INIT:
            output_signal = 4'b0010;
        default:
            output_signal = 4'b0000;
    endcase
end


    assign state_out = state;

endmodule
