% after picking pouch it must be droped and picked from a weight machine

% move to weight machine
% trans = [-0.35 0.7 0.3];
% rot = [pi -pi 0]; %[z ,y ,x]
% g_val = 0.541;
% run moveTo.m;

% decrease z and drop pouch (only touches the weight machine)
trans = [-0.35 0.7 0.128];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.541;   
run moveTo.m;

% grab pouch 
% g_val = 0.541; -> unable to grab
% run moveTo.m;

% increase z
trans = [-0.35 0.7 0.3];
rot = [pi -pi 0]; %[z ,y ,x]
g_val = 0.541;
run moveTo.m;