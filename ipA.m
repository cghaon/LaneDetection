%% Use ipA to send information
clear;clc;fclose(instrfindall)

%% Define computer-specific variables
% ip address of ipA.
ipA = '192.168.50.243'; portA = 9090;
% ip address of ipB
ipB = '192.168.50.206'; portB = 9091;

%% Create UDP Object
udpA = udp(ipB,portB,'LocalPort',portA);

%% Connect to UDP Object
fopen(udpA);
fprintf('ipA is online');

%% Sending information to ipB
while(1)
    % …… Your other process code
    % test
    package = 'Zhi Fang Tan is my son';
    fwrite(udpA, package)
    fprintf('Package shipped')
    pause(1)
end


 % Remember to execute fclose(instrfindall) after you have stopped
 % the script because the UDP object will not be shut down even if the code
 % is stopped, 
 % and the port will be occupied when you try to run the code again.