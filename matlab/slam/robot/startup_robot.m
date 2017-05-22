function startup_robot(IP)
%% functions available for penguinPi
%pb.setMotorSpeeds(R,L);
%ticks = pb.getMotorTicks();
%img = pb.getImageFromCamera();

%% add subfolders into path
addpath('./robot_default')
addpath('./motion')
addpath('./ekf')
addpath('./vision')

%% initialize robot
pb = PiBot(IP);
pb.setMotorSpeeds(0,0);

end