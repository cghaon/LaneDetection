%% Use ipB to receive information
clear;clc;fclose(instrfindall)

%% Define computer-specific variables
% ip address of ipA.
ipA = '192.168.50.243'; portA = 9090;
% ip address of ipB
ipB = '192.168.50.206'; portB = 9091;

%% Create UDP Object
udpB = udp(ipA,portA,'LocalPort',portB);
fprintf('ipB is online');

%% Connect to UDP Object
fopen(udpB);

%% Receiving information from ipA
while(1)
    % find whether there’s new message available in the buffer
    if udpB.BytesAvailable > 0
        data = fread(udpB, udpB.BytesAvailable)
        fprintf('Package received');
        % clear the input buffer after read the message to make sure that 
        % the latest message will be read in the next loop
        flushinput(udpB)
    end
% …… Your other process code
% test
fprintf(data)

end