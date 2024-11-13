clear;
clc;
mylego = legoev3('usb');
% x & y co-ordinates of the platforms
ax = 110;
ay = 0;
bx = 0;
by = 110;
cx = -110;
cy = 0;

legoev3 = grp2_pid(mylego,ax,ay,bx,by,cx,cy);
pos = positions();
disp('Home position')
legoev3.home()
disp('Reading Platform Heights')
legoev3.station_heights();
disp('Performing Inverse kinematics using geometical approach')
legoev3.invkin();
disp(legoev3.stationB_height)

disp('Performing action -- B -> C')
pos.B_C(legoev3);
pause(1);
disp('Performing action -- C -> A')
pos.C_A(legoev3);
pause(1);
disp('Performing action -- A -> B')
pos.A_B(legoev3);
pause(1);
disp('Performing action -- B -> A')
pos.B_A(legoev3);
pause(1);
disp('Performing action -- A -> C')
pos.A_C(legoev3);
pause(1);
disp('Performing action -- C -> B')
pos.C_B(legoev3);
pause(1);