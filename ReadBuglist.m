function [quad_init_x,quad_init_y] = ReadBuglist( x1,x2,x3,x4,y1,y2,y3,y4,test )
%READBUGLIST Summary of this function goes here
%   Detailed explanation goes here
quad_init_x=[x1(test);x2(test);x3(test);x4(test)];
quad_init_y=[y1(test);y2(test);y3(test);y4(test)];
end

