clc
clear all

load('rcnn_stop.mat')
load('rcnn_school.mat')
path='CounterClockwise.mp4';
writerObj=VideoWriter('schooltrace4','MPEG-4'); 
open(writerObj); 
obj=VideoReader(path);
frame_number=floor(obj.Duration * obj.FrameRate);
for i=1:frame_number
    I=read(obj,i);
    I=im2uint8(I);
    bboxes1 = detect(rcnn_school,I);
    annotation1 = 'school sign';
    I = insertObjectAnnotation(I,'rectangle',bboxes1,annotation1,'Color','cyan');
    bboxes2 = detect(rcnn_stop,I);
    annotation2 = 'stop sign';
    I = insertObjectAnnotation(I,'rectangle',bboxes2,annotation2);
    imshow(I)
    if ~isempty(bboxes1)
        %send there is a school sign
    end
    if ~isempty(bboxes2)
        %send there is a stop sign
    end
    clear bboxes;
    writeVideo(writerObj,I); 
end
close(writerObj);