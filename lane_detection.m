clc
clear all

figure(1);
v = VideoReader('Clockwise.mp4');

H_left = zeros(2,4);
H_left(1,1)=1;H_left(2,2)=1;
H_right = zeros(2,4);
H_right(1,3)=1;H_right(2,4)=1;

R = zeros(2,2);
R(1,1)=20;R(2,2)=20;
P = eye(4) * 100000;
X = [0;0;0;0];
Q = eye(4)*50;

while hasFrame(v)
    frame = readFrame(v);
    shape = size(frame);
    hsv = rgb2hsv(frame);
    mask_hsv = hsv(:,:,1) > 140/256 & hsv(:,:,2) > 60/256;
    img_gray = rgb2gray(frame);
    img_mask = edge(mask_hsv,'sobel','vertical');
    a=[shape(2)*0.4, shape(2)*0.6, shape(2), 0];
    b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
    bw=roipoly(frame,a,b);
    BW=(img_mask(:,:,1)&bw);
    
    %%% Hough Transform & Line Extraction %%%
    [hough_mat,theta,rho] = hough(BW,'Theta',-70:0.5:70);
    Peaks=houghpeaks(hough_mat,4);
    lines = houghlines(BW,theta,rho,Peaks,'FillGap',50,'MinLength',20);
    
    hold on;
    angle_thres = 0.01;
    leftlines=[];rightlines=[];
    for i = 1:length(lines)
       x1=lines(i).point1(1);y1=lines(i).point1(2);
       x2=lines(i).point2(1);y2=lines(i).point2(2);
       if(x1>=shape(2)/2) && ((y2-y1)/(x2-x1)>angle_thres)
           rightlines = [rightlines;x1,y1;x2,y2];
       elseif(x1<=shape(2)/2) && ((y2-y1)/(x2-x1)<(-1*angle_thres))
           leftlines = [leftlines;x1,y1;x2,y2];
       end
    end
    draw_y = [shape(1)*0.6,shape(1)];
    
    %%% Kalman Filter %%%
    P = P + Q;
    if(~isempty(leftlines))
        PL = polyfit(leftlines(:,2),leftlines(:,1),1);
        draw_lx = polyval(PL,draw_y);
        K = P*H_left'*(H_left*P*H_left'+R)^-1;
        X = X + K*(draw_lx' - H_left*X);
        P = (eye(4)-K*H_left)*P;
    end
    if(~isempty(rightlines))
        PR = polyfit(rightlines(:,2),rightlines(:,1),1);
        draw_rx = polyval(PR,draw_y);
        K = P*H_right'*(H_right*P*H_right'+R)^-1;
        X = X + K*(draw_rx' - H_right*X);
        P = (eye(4)-K*H_right)*P;
    end
    disp(X);
    disp(P);
    imshow(frame);
    hold on;
%     plot(draw_lx,draw_y,'LineWidth',2,'Color','red');
    plot(X(1:2),draw_y,'LineWidth',2,'Color','red');
    hold on;
%     plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
    plot(X(3:4),draw_y,'LineWidth',2,'Color','red');
    drawnow;
end
