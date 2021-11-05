function out = ComputeWheelsVelocities(u)

r = 0.025;  % wheels radius
b = 0.145;  % wheel base

W = [r/b -r/b; r/2 r/2];
W = W^-1;

out = W * u;
end

