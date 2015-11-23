This code was KTH lab assignment for academic purpose. Code is written in C++ under the Micro OS-II library structure. The goal of the lab was to create conceptual awareness of the Real Time Operating System programming issues and the efficient use of library resources to strike out the programming debacles over limited hardware resources.


Cruise Control Application
When activated, the cruise control system maintains the speed of the car at a constant value that has been set by the driver.

The system has the following inputs:

Engine (ON/OFF). The engine is turned on, in case the signal ENGINE is active. The engine can only be turned off, if the speed of the car is 0 m/s.

Cruise Control (ON/OFF). The cruise control is turned on, if the signal CRUISE_CONTROL is activated and if the car is in top gear (TOP_GEAR is active) and if the velocity is at least 20ms and the signals GAS_PEDAL and BRAKE_PEDAL are inactive.

Gas Pedal (ON/OFF). The car shall accelerate, if the signal GAS_PEDAL is active. The cruise control shall be deactivated, if GAS_PEDAL is active.

Brake (ON/OFF). The car shall brake, when the signal BRAKE is active. Also the cruise control shall be deactivated, if the signal BRAKE is activated.

Gear (HIGH/LOW). The car has two different gear positions (high, low) indicated by the signal TOP_GEAR. If TOP_GEAR is active the the gear position is high, otherwise low. The cruise control is deactivated, when the gear position is moved to low. The inputs are connected to the following IO-units on the DE2-board:

----------------------------------
Signals 			Pin 	LED
----------------------------------
ENGINE 				SW0 	LEDR0
TOP_GEAR 			SW1 	LEDR1
CRUISE_CONTROL		KEY1	LEDG2
BRAKE_PEDAL			KEY2	LEDG4
GAS_PEDAL			KEY3 	LEDG6
----------------------------------

The control task uses a constant throttle of 40. The task VehicleTask implements the behavior of the car and its functionality. The code is executed on the Altera DE2-board.

Main Highlights:
1. Soft Timers are used to Implement Periodic Tasks
2. ButtonIO and SwitchIO, read the buttons and switches on the DE2-board periodically. The task SwitchIO creates the signals ENGINE and TOP_GEAR, while the task ButtonIO creates the signals CRUISE_CONTROL, GAS_PEDAL and BRAKE_PEDAL. Red LEDs are used to indicate that a tactile switch is active and the green LEDs to indicate that a button is active
3. Control Law:
The braking functionality is implemented inside the VehicleTask, whereas the ControlTask sets the throttle. The control law reacts according to the state of the buttons and switches. When the cruise control is activated, the current velocity is maintained with a maximum deviation of 2ms for velocities of at least 25ms . The green LED LEDG0 to indicate that the cruise control is active.
4. The rest of the  (dummy) functions show position and target velocity. The position indicates the current position of the vehicle on the track with the six leftmost red LEDs.

---------------------------------
LED 	 		Position 		
---------------------------------	
LEDR17			[0m, 400m]
LEDR16			[400m, 800m]
LEDR15			[800m, 1200m]
LEDR14			[1200m, 1600m]
LEDR13			[1600m, 2000m]
LEDR12			[2000m, 2400m]
---------------------------------
The seven segement display (HEX5 and HEX4) will also show the position of the vehicle target velocity, which the cruisecontrol is trying to maintain. The display resets to 0 when the cruise control is deactivated.
