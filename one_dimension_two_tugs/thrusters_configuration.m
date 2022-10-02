%% thruster configuration
function T = thrusters_configuration(a,L)
% function thruster_configuration is used to obtain the configuration


T = zeros(3,3);   % initialization

for i = 1 : 3
    T(1,i) = cos(a(i));
    T(2,i) = sin(a(i));
    T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
end

end