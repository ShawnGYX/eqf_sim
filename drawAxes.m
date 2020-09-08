function drawAxes(R,x,sze,sty)
%DRAWAXES.M Summary of this function goes here
%   Detailed explanation goes here
if nargin < 4
    sty = '-';
end
if nargin < 3
    sze = 1;
end
    
plot3([x(1), x(1) + sze*R(1,1)], [x(2), x(2) + sze*R(2,1)], [x(3), x(3) + sze*R(3,1)], ['r',sty], ...
      [x(1), x(1) + sze*R(1,2)], [x(2), x(2) + sze*R(2,2)], [x(3), x(3) + sze*R(3,2)], ['k',sty], ...
      [x(1), x(1) + sze*R(1,3)], [x(2), x(2) + sze*R(2,3)], [x(3), x(3) + sze*R(3,3)], ['b',sty], ...
      x(1), x(2), x(3), 'ko');

end

