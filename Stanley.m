clc
clear all
%% get image
path='CounterClockwise.mp4';
obj=VideoReader(path);
I=read(obj,1);
I=im2uint8(I);
%%
%line detection
H_left = zeros(2,4);
H_left(1,1)=1;H_left(2,2)=1;
H_right = zeros(2,4);
H_right(1,3)=1;H_right(2,4)=1;

R = zeros(2,2);
R(1,1)=20;R(2,2)=20;
P = eye(4) * 100000;
X = [0;0;0;0];
Q = eye(4)*50;

    shape = size(I);
    hsv = rgb2hsv(I); 
    mask_hsv = hsv(:,:,1) > 140/256 & hsv(:,:,2) > 60/256;
    img_gray = rgb2gray(I);
    img_mask = edge(mask_hsv,'sobel','vertical');
    a=[shape(2)*0.4, shape(2)*0.6, shape(2), 0];
    b=[shape(1)*0.6, shape(1)*0.6, shape(1),shape(1)];
    bw=roipoly(I,a,b);
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
    imshow(I);
    hold on;
%     plot(draw_lx,draw_y,'LineWidth',2,'Color','red');
    plot(X(1:2),draw_y,'LineWidth',2,'Color','red');
    hold on;
%     plot(draw_rx,draw_y,'LineWidth',2,'Color','red');
    plot(X(3:4),draw_y,'LineWidth',2,'Color','red');
    drawnow;
%%
%The extracted left and right lane markers are represented with two points for each
X_lf = X(1) - 510;
Z_lf = 838 - draw_y(1);
X_ln = X(2) + 0.342 * Z_lf - 510;
Z_ln = 100;
X_rf = X(3) - 510;
Z_rf = 838 - draw_y(1);
X_rn = X(4) - 510 - 0.342*Z_rf;
Z_rn = 100;
P_lf = [X_lf, Z_lf];%left far
P_ln = [X_ln, Z_ln];%left near
P_rf = [X_rf, Z_rf];%right far
P_rn = [X_rn, Z_rn];%right near
%get center line
X_cf = 0.5 * (X_rf + X_lf); 
X_cn = 0.5 * (X_rn + X_ln);
Z_cf = 0.5 * (Z_rf + Z_lf);
Z_cn = 0.5 * (Z_rn + Z_ln);
%%
%calculate departure angle
if (X_cf - X_cn) < 0
    arctan = atan((Z_cf - Z_cn)/(X_cf - X_cn)) * 360 / (2 * pi);
    DA = -arctan -90; % departure angle
else
    arctan = atan((Z_cf - Z_cn)/(X_cf - X_cn)) * 360 / (2 * pi);
    DA = -arctan + 90;
end
%%
%calculate departure distance
C_pos = -60; %60mm 
e_fa = X_cn + (X_cf - X_cn) / (Z_cf - Z_cn) * (Z_cn - C_pos); %departure distance
%%
k1 = 0.5; %parameter 1 for DA
k2 = 0.01;%parameter 2 for departture distance
v = 50;%current velocity
SC = k1 * DA;
SC = SC + atan(k2*e_fa/v) * 360 / (2 * pi);
%%
%create image
position = [500,500]; %position
printString = "steering angle: " + SC; 
newPicture = insertText(I, position, printString);
imshow(newPicture);
