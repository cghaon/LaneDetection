clc
clear all
fclose(instrfindall);
%% Use ipB to receive information
clear;clc;

%% Define computer-specific variables
% ip address of ipA.
ipA = '192.168.50.243'; portA = 9090;
% ip address of ipB
ipB = '192.168.50.206'; portB = 9091;
%% Create UDP Object
udpB = udp(ipA,portA,'LocalPort',portB);
fprintf('ipB is online\n');

%% Connect to UDP Object
fopen(udpB);

%%
%load picture
load('rcnn_stop.mat')
load('rcnn_school.mat')
path='CounterClockwise.mp4';
obj=VideoReader(path);
frame_number=floor(obj.Duration * obj.FrameRate);
count = 1;
whileloop = 0;
%%
while(1)
    %receive
        if udpB.BytesAvailable > 0
            i = fread(udpB, udpB.BytesAvailable,'uint32');
            if i == 0
                break
            end
            I=read(obj,i);
            I=im2uint8(I);
            bboxes1 = detect(rcnn_school,I);%detect school sign
            bboxes2 = detect(rcnn_stop,I); %detect stop sign
            if ~isempty(bboxes1)
                try
                    data = [1,bboxes1];
                catch
                    data = [1,bboxes1(1,:)];
                end
                fwrite(udpB, data,'uint32');
                clear bboxes1;
                fprintf('%i school\n', i);
            elseif ~isempty(bboxes2)
                try
                    data = [2,bboxes2];
                catch
                    data = [2,bboxes2(1,:)];
                end
                fwrite(udpB, data,'uint32');
                clear bboxes2;
                fprintf('%i stop\n',i);
            
            elseif isempty(bboxes1) && isempty(bboxes2)
                data = 0;
                fwrite(udpB,data,'uint32');
                fprintf('%i none\n',i);
            end
            clear bboxes1;
            clear bboxes2;
            whileloop = 0;
        else
            whileloop = whileloop + 1;
        end
        if(whileloop > 10000)
            fwrite(udpB,0,'uint32');
            whileloop = 0;
            fprintf('bagdrop none\n');
        end
end
%% close all the connection
fclose(instrfindall);