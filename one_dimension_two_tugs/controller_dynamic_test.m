name = 'tug1'; 
link = [47.5; 32.5];
init_position  = [0, 0, 0]';
init_velocity  = [0, 0, 0]';
dt = 1;
tug1 = DPTug(name, link,  init_position, init_velocity, dt);

pd = [1, 0, 0]';
vd = [0, 0, 0]';


tau = tug1.BacksteppingController(pd, vd, tug1.p_, tug1.v_)
% [tau_r, f, a] = ThrusterAllocation(tug1)
[p_next, v_next] = ShipDynamic(tug1)