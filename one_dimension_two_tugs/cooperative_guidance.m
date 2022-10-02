function [pd1, pd2] = cooperative_guidance(p0, p1, p2, t)


end


%% 
function pd = leader_reference_mode(t, dt, w, z, r)

m0 = diag([5.25e7, 5.25e7, 7.5e10]);
ma0 = diag([1.7e7, 4.9e7, 5.4e10]);
D0 = diag([2.4e6, 4.8e6, 3.9e7]);
M0 = m0 + ma0;

for i=1:N+1,
   y2(i,:) = [x v];   
   x_dot = v;
   v_dot = w^2*(r2-x) - 2*z*w*v - delta*abs(v)*v;
   v = v + h*v_dot;
   x = x + h*x_dot;
end

end


