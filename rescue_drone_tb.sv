module tb_drone_fsm;
    // Internal signals
    logic clk, reset, thermal_found, err, sensor_check, man_ctrl, target_found, is_charging;
    logic [1:0] cmd, img_pass, propellor_status;
    logic [7:0] altitude;
    logic [3:0] state_out, led;
    logic charge_status, check_sensors, check_propellers, flight_error, image_scan, destination, manual_control, maintenance_req;

    // Instantiate the module under test (MUT)
    drone_fsm uut (.clk(clk), .reset(reset), .cmd(cmd), .thermal_found(thermal_found), .img_pass(img_pass), .altitude(altitude),
                   .err(err), .sensor_check(sensor_check), .man_ctrl(man_ctrl), .target_found(target_found), .is_charging(is_charging),
                   .state_out(state_out), .led(led), .propellor_status(propellor_status), .charge_status(charge_status), 
                   .check_sensors(check_sensors), .check_propellers(check_propellers), .flight_error(flight_error), 
                   .image_scan(image_scan), .destination(destination), .manual_control(manual_control), .maintenance_req(maintenance_req));

    // Clock generation
    always begin #5 clk = ~clk; end

    initial begin
        clk = 0; reset = 1; cmd = 2'b00; thermal_found = 0; img_pass = 2'b00; altitude = 8'b00000000; err = 0; sensor_check = 0;
        man_ctrl = 0; target_found = 0; is_charging = 0;
        #10 reset = 0;    					//  reset
        #20 cmd = 2'b01; #20 cmd = 2'b00;   			// Command to initialize the drone and clear the command
        #20 sensor_check = 1;   				// Sensor check passes, transition to takeoff
        #20 altitude = 8'b01000000;  				// Altitude increase to transition to search
        #20 thermal_found = 1;  				// Finding a thermal signature, transition to target found
        #20 img_pass = 2'b10;   				// Passing the image check, transition to image capture
        #100;  							// Timer expiry in image capture, back to search
        #50     						// Wait
        #20 cmd = 2'b01; #10 cmd = 2'b00;   			// Command to go to maintenance, clear command
        #20 err = 0; man_ctrl = 1; thermal_found = 0;  		// Clear error, manual control activation
        #20 man_ctrl = 0;   					// Deactivate manual control, back to search
        #20 target_found = 1;    				// Finding a target, transition to return base
        #20 is_charging = 1; 					// Drone is charging, transition to idle
	#20 cmd = 2'b01; #10 cmd = 2'b00;
	#20 sensor_check = 0; err = 1'b1;
	#20s

	#50 $finish;  // End of test
    end

    // Monitoring
    initial $monitor("At time %t, State = %b, LED = %b", $time, state_out, led);
endmodule
