function [Length,x1,x2,x3,x4,y1,y2,y3,y4] = ReadBuglistlength(phns)
[usedtime,x1,x2,x3,x4,y1,y2,y3,y4]=textread(phns,'%f%f%f%f%f%f%f%f%f');
Length=length(usedtime(:,1));

end

