classdef positions < handle
    properties
        movement
        pickup
        place
    end
    
    methods
        function pos = positions()
        end
        function B_C(pos, robot)
            robot.movement('B');
            robot.pickup();
            robot.movement('C');
            robot.place();
            robot.movement('B');
        end
        
        function C_A(pos, robot)
            robot.movement('C');
            robot.pickup();
            robot.movement('A');
            robot.place();
            robot.movement('B');
        end
        
        function A_B(pos, robot)
            robot.movement('A');
            robot.pickup();
            robot.movement('B');
            robot.place();
            robot.movement('B');
        end
        
        function B_A(pos, robot)
            robot.movement('B');
            robot.pickup();
            robot.movement('A');
            robot.place();
            robot.movement('B');
        end
        
        function A_C(pos, robot)
            robot.movement('A');
            robot.pickup();
            robot.movement('C');
            robot.place();
            robot.movement('B');
        end
        
        function C_B(pos, robot)
            robot.movement('C');
            robot.pickup();
            robot.movement('B');
            robot.place();
            robot.movement('B');
        end
    end
end
