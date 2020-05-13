
%Initiate ros node (Matlab Node)
ipaddress = '192.168.0.5';
rosinit(ipaddress,11311);

%import robot model
gen3_lite_robot = importrobot('GEN3-LITE.urdf');
%show(gen3_lite_robot);

