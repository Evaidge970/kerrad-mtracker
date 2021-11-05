function [qr, ur] = TrajectoryGenerator(t)

% Lissajou curves

% Parameters
A0x = 0; A0y = -0.3;
A1x = 0.06; A1y = 0.16;
wx = 2.0; wy = 1.0;
phix = 0; phiy = 0;

% Position variables
xr = A1x*sin(wx*t+phix) + A0x;
yr = A1y*sin(wy*t+phiy) + A0y;

% Compute these variables!
dxr = wx*A1x*cos(wx*t+phix);
dyr = wy*A1y*cos(wy*t+phiy);
ddxr = -wx*wx*A1x*sin(wx*t+phix);
ddyr = -wy*wy*A1y*sin(wy*t+phiy);

thr = atan2(dyr, dxr);
wr = (ddyr*dxr - dyr*ddxr)/(dxr*dxr+dyr*dyr);
vr = sqrt(dxr*dxr+dyr*dyr);

qr = [xr; yr; thr];
ur = [wr; vr];


end

