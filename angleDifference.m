% outDiff = angleDifference(angleI, angleJ)
% This function takes the difference between two angles, 0 <= angleI < 2pi,
% 0 <= angleJ < 2pi, such that the resultant angle is also within 0 and 2pi
% outDiff - difference between angleI and angleJ, output is between 0 and
% 2pi
% angleI - angle between 0 and 2pi
% angleJ - angle between 0 and 2pi
function outDiff = angleDifference(angleI, angleJ)

two_pi = 2*pi;
outDiff = angleI + two_pi - angleJ;

if outDiff < 0 
    outDiff = outDiff + two_pi;
elseif outDiff > two_pi
    outDiff = outDiff - two_pi;
end
    

