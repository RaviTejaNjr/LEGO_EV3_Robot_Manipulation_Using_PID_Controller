classdef grp2_pid < handle
    properties
        % Robot sensors & motors
        mylego
        mytouchsensor1
        mytouchsensor3
        mysonicsensor
        motorA
        motorB
        motorC

        % Link lengths
        l0 = 65
        l1 = 50
        l2 = 95
        l3 = 185
        l4 = 110

        % Gear Ratio (with correction factor adjusted)
        grbase = 3 % 3 * 1.15 (1.15 --- correction factor)
        grarm = 5   % 5 * 1.15

        % station variables
        position = 'A'
        stationA_angle  % Platform A angle
        stationB_angle  % Platform B angle
        stationC_angle  % Platform C angle
        stationA_height % Platform A height
        stationB_height % Platform B height
        stationC_height % Platform C height

        % variables for PID controller of motorC
        kp = 0.1
        ki = 0.1
        kd = 0.01
        
        % variables for PID controller of motorB
        kp1 = 0.5
        kd1 = 0.15
        ki1 = 0.45

        time
        time_minus
        time_elapsed
        error
        previous_error
        input
        platform_variable
        cum_error
        rate_error
        platform 
    end
    methods
        % Initializing legoev3 Configuration
        function grp2 = grp2_pid(input,ax,ay,bx,by,cx,cy)
            grp2.mylego = input;
            grp2.mytouchsensor1 = touchSensor(grp2.mylego, 1);
            grp2.mytouchsensor3 = touchSensor(grp2.mylego, 3);
            grp2.mysonicsensor = sonicSensor(grp2.mylego, 2);
            grp2.motorA = motor(grp2.mylego, 'A');
            grp2.motorB = motor(grp2.mylego, 'B');
            grp2.motorC = motor(grp2.mylego, 'C');
            start(grp2.motorB)
            start(grp2.motorC)
            start(grp2.motorA)

            % Inverse kinematics for station theta
            grp2.stationA_angle = atan2d(ay, ax);
            grp2.stationB_angle = atan2d(by, bx);
            grp2.stationC_angle = atan2d(cy, cx);
        end

        function home(grp2)
            % Calibrating motor B
            while(~readTouch(grp2.mytouchsensor3))
                grp2.motorB.Speed = -10;
            end
            grp2.motorB.Speed = 0;
            resetRotation(grp2.motorB);

            % Calibrating motor A
            while(~readTouch(grp2.mytouchsensor1))
                grp2.motorC.Speed = 30;
            end
            grp2.motorC.Speed = 0;
            resetRotation(grp2.motorC);

            % Calibrating Gripper Motor
            grp2.motorA.Speed = -30;
            pause(1)
            grp2.motorA.Speed = 0;
            resetRotation(grp2.motorA);

            % Converting Theta 1 values to encoder values
            grp2.stationA_angle = floor(grp2.stationA_angle * grp2.grbase);
            grp2.stationB_angle = floor(grp2.stationB_angle * grp2.grbase);
            grp2.stationC_angle = floor(grp2.stationC_angle * grp2.grbase);
            grp2.movement('B')
            pause(1)
        end

        function station_heights(grp2)
            grp2.movement('A')
            pause(1);
            grp2.stationA_height = readDistance(grp2.mysonicsensor) * 1000;
            if (grp2.stationA_height) < 200
                e = 20;
                grp2.stationA_height = grp2.stationA_height + e;
            end
            disp(legoev3.stationA_height)
            grp2.movement('B')
            pause(1);
            grp2.stationB_height = readDistance(grp2.mysonicsensor) * 1000;
            if (grp2.stationB_height) < 200
                e = 20;
                grp2.stationB_height = grp2.stationB_height + e;
            end
            disp(legoev3.stationB_height)
            grp2.movement('C')
            pause(1);
            grp2.stationC_height = readDistance(grp2.mysonicsensor) * 1000;
            if (grp2.stationC_height) < 200
                e = 20;
                grp2.stationC_height = grp2.stationC_height + e;
            end
            disp(legoev3.stationC_height)
            grp2.movement('B')
        end

        % Detemining Theta2 & converting to encoder values
        function invkin(grp2)
            grp2.stationA_height1 = ((asind((grp2.stationA_height - grp2.l0 - grp2.l1 - (grp2.l2 * sind(45)) + grp2.l4) / grp2.l3) + 45));
            grp2.stationB_height1 = ((asind((grp2.stationB_height - grp2.l0 - grp2.l1 - (grp2.l2 * sind(45)) + grp2.l4) / grp2.l3) + 45));
            grp2.stationC_height1 = ((asind((grp2.stationC_height - grp2.l0 - grp2.l1 - (grp2.l2 * sind(45)) + grp2.l4) / grp2.l3) + 45));
        end

        function claw_close(grp2)
            while (readRotation(grp2.motorA) > 14)
                grp2.motorA.Speed = -20;
            end
            grp2.motorA.Speed = 0;
        end

        function claw_open(grp2)
            while (readRotation(grp2.motorA) < 80)
                grp2.motorA.Speed = 20;
            end
            grp2.motorA.Speed = 0;
        end
        
        % movement Station A or B or C
        function movement(grp2, station)
            grp2.platform = station;
            if grp2.platform == 'A'
                grp2.platform_variable = grp2.stationA_angle;
            elseif grp2.platform == 'B'
                grp2.platform_variable = grp2.stationB_angle;
            else
                grp2.platform_variable = grp2.stationC_angle;
            end
            grp2.error = grp2.platform_variable +  readRotation(grp2.motorC);
            tic();
            grp2.time_minus = 0;
            grp2.cum_error = 0;
            grp2.previous_error = 0; 
            while(grp2.error > 5 || grp2.error < -5)
                grp2.time = toc;                                                                                     % getting current time
                grp2.error = readRotation(grp2.motorC) + grp2.platform_variable;                                     % determining error
                grp2.time_elapsed = (grp2.time - grp2.time_minus);                                                   % computing time elapsed from previous time
                grp2.rate_error = (grp2.error - grp2.previous_error) / grp2.time_elapsed;                            % rate of error
                grp2.cum_error = grp2.cum_error + (grp2.error * grp2.time_elapsed);                                  % compute integral of error
                grp2.motorC.Speed = -(grp2.kp * grp2.error + grp2.ki * grp2.cum_error + grp2.kd * grp2.rate_error);  % PID output
                grp2.previous_error = grp2.error;                                                                    % assigning error to previous error
                grp2.time_minus = grp2.time;                                                                         % assigning time to previous time
            end
            grp2.motorC.Speed = 0;
            grp2.position  = grp2.platform;
        end

        % arm movement to donwward
        function arm_down(grp2)
            if grp2.position  == 'A'
                height = grp2.stationA_height1;
            elseif grp2.position  == 'B'
                height = grp2.stationB_height1;
            else
                height = grp2.stationC_height1;
            end
            class(height)
            a = double(90) - double(height);
            grp2.error = (a * double(grp2.grarm)) - double(readRotation(grp2.motorB));
            % PID control parameters for arm_down
    
            grp2.time = tic();
            grp2.time_minus = 0;
            grp2.cum_error = 0;
            grp2.previous_error = 0; 
            while (abs(grp2.error) >= 3)
                grp2.time_elapsed = toc(grp2.time);
                grp2.error = (a * double(grp2.grarm)) - double(readRotation(grp2.motorB));
                grp2.rate_error = (grp2.error - grp2.previous_error) / grp2.time_elapsed;
                grp2.cum_error = grp2.cum_error + (grp2.error * grp2.time_elapsed);
                grp2.motorB.Speed = (grp2.kp1 * grp2.error + grp2.ki1 * grp2.cum_error + grp2.kd1 * grp2.rate_error);
                grp2.previous_error = grp2.error;
                grp2.time_minus = grp2.time;
            end
            grp2.motorB.Speed = 0;
        end
        
        % arm movement upward
        function arm_up(grp2)
            while (readRotation(grp2.motorB) > 0)
                grp2.motorB.Speed = -30;
                readRotation(grp2.motorB);
            end
            grp2.motorB.Speed = 0;
        end

        % placing the payload
        function place(grp2)
            grp2.arm_down();
            grp2.claw_open();
            grp2.arm_up();
            grp2.claw_close();
        end

        % picking the payload
        function pickup(grp2)
            grp2.claw_open();
            grp2.arm_down();
            grp2.claw_close();
            grp2.arm_up();
        end
    end
end
