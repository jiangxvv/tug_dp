classdef DPTug
    %% 变量
    properties
        Name_;
        Mode_;  %(two_tug, four_tug)
        M_;  D_;
        dt_; t_;
        tau_; lamda_;  kp_; ki_; kd_; integrator_;  s_; % controller parameters
        tau_r_;  f_range_; a_range_;  f0_;  a0_;  N_;   L_;  f_;  a_;    % thrusters allocate
        p0_ = zeros(3, 1); v0_; p_; v_; pd_; vd_; ad_; p_init_;   v_init_;
        p_history_;  v_history_;  pd_history_; vd_history_; ad_history_; p0_history_; v0_history_;
        tau_history_;    tau_r_history_;  f_history_;   a_history_; tau_cable_history_;
        time_history_;
        K_;  Length_;   Link_;
    end
    
    %% 方法
    methods
        % construction function 
        function obj = DPTug(name, link,  init_position, init_velocity, dt)
            obj.p_init_ = init_position;     obj.p_ = obj.p_init_;     obj.p_history_ = obj.p_;
            obj.v_init_ = init_velocity;     obj.v_ = obj.v_init_;     obj.v_history_ = obj.v_;
            obj.p0_history_ = zeros(3, 1);
            obj.integrator_ = zeros(3, 1);
            obj.Name_ = name;
            obj.dt_ = dt;
            obj.pd_ = zeros(3, 1);  obj.vd_ = zeros(3,1);
            
            obj.lamda_ = eye(3);
            obj.kp_ = 5e5*diag([1; 1; 1e3]);
            obj.kd_ = 1e4*diag([1; 1; 1e3]);
            obj.ki_ = diag([1e2; 1e2; 1e5]);
            obj.M_ = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
            obj.D_ = diag([obj.M_(1, 1)/100, obj.M_(2, 2)/40, obj.M_(3, 3)/20]);
            
            obj.N_ = 3;
            obj.f_range_ = [0, 900];
            obj.a_range_ = [-pi, pi];
            obj.f0_ = 100*ones(3, 1);   obj.a0_ = pi/3*ones(3, 1);
            obj.f_history_ = obj.f0_;    obj.a_history_ = obj.a0_;
            obj.tau_r_history_ = zeros(3,1);
            obj.tau_history_ = zeros(3, 1); 
            obj.tau_cable_history_ = zeros(2, 1);
            obj.L_ = [6, -2.7, 0; 6, 2.7, 0; -11, 0, 0];
            %cable parameters
            obj.Length_ = 400;  obj.K_ = 600;   obj.Link_ = link;
            obj.pd_history_ = obj.pd_;   obj.vd_history_ = obj.vd_;
            obj.time_history_ = 0;
        end
        
        function obj = Controller(obj)
            fai = obj.p_(3);
            R = [cos(fai), -sin(fai), 0; -sin(fai), cos(fai), 0; 0, 0, 1];
            Rt = R';
            kp = 5;     k = 1;
            ep = Rt * (obj.pd_ - obj.p_);
            ev = Rt * obj.vd_ - obj.v_;
            e = ev + kp * ep;
            C = CCVector(obj, obj.v_);
            S = [0, -obj.v_(3), 0; obj.v_(3), 0, 0; 0, 0, 0];
            tau = C + obj.D_ * obj.v_ + obj.M_ * (-S * (Rt * obj.vd_ + kp * ep)...
                + Rt * obj.ad_ + kp * ev) + ep + k * e;
            tau = tau /1e3; 
%                 tau_cable = obj.CableForce(obj);
%                 tau = tau - tau_cable;
            obj.tau_ = tau;
            obj.tau_history_ = [obj.tau_history_, tau];
            
        end
        
        function obj = BacksteppingController(obj)
                p = obj.p_; v = obj.v_;
                pd = obj.pd_; vd = obj.vd_;
                fai = p(3); dot_fai = v(3);
                R = [cos(fai), -sin(fai), 0; sin(fai), cos(fai), 0; 0, 0, 1];
                Rt = R';
                dot_R = dot_fai * [-sin(fai), -cos(fai), 0; cos(fai), -sin(fai), 0; 0, 0, 0];

                ep = p - pd;
                dot_pr = vd - obj.lamda_ * ep;
                ddot_pr = -obj.lamda_ * (R * v -vd);
                s = R * v - dot_pr;  obj.s_ = s;
                
                nu_r = Rt *dot_pr;
                dot_nu_r = Rt * ddot_pr - Rt * dot_R * Rt * dot_pr;

                imax = [100, 100, 100]; imin = -imax;
                
                for i=1:3
                    obj.integrator_(i) = obj.integrator_(i) + ep(i) * obj.dt_;
                    if(obj.integrator_(i) > imax(i))
                        obj.integrator_(i) =imax(i);
                    end
                    if (obj.integrator_(i) < imin(i))
                        obj.integrator_(i) =imin(i);
                    end
                end
                C = CCVector(obj, obj.v_);
                tau = obj.M_ * dot_nu_r + obj.D_ * nu_r + C...
                        - Rt * (obj.kp_ * ep + obj.kd_ * obj.s_ + obj.ki_ * obj.integrator_);
                tau = tau /1e3; 
%                 tau_cable = obj.CableForce(obj);
%                 tau = tau - tau_cable;
                obj.tau_ = tau;
                obj.tau_history_ = [obj.tau_history_, tau];
%                 obj.tau_cable_history_ = [obj.tau_cable_history_, tau_cable];
                
        end
        
        % backstepping controller with cable force
        function obj = BacksteppingControlerWithCable(obj)
                p = obj.p_; v = obj.v_;
                pd = obj.pd_; vd = obj.vd_;
                fai = p(3); dot_fai = v(3);
                R = [cos(fai), -sin(fai), 0; sin(fai), cos(fai), 0; 0, 0, 1];
                Rt = R';
                dot_R = dot_fai * [-sin(fai), -cos(fai), 0; cos(fai), -sin(fai), 0; 0, 0, 0];

                ep = p - pd;
                dot_pr = vd - obj.lamda_ * ep;
                ddot_pr = -obj.lamda_ * (R * v -vd);
                s = R * v - dot_pr;  obj.s_ = s;
                
                nu_r = Rt *dot_pr;
                dot_nu_r = Rt * ddot_pr - Rt * dot_R * Rt * dot_pr;

                imax = [100, 100, 100]; imin = -imax;
                
                for i=1:3
                    obj.integrator_(i) = obj.integrator_(i) + ep(i) * obj.dt_;
                    if(obj.integrator_(i) > imax(i))
                        obj.integrator_(i) =imax(i);
                    end
                    if (obj.integrator_(i) < imin(i))
                        obj.integrator_(i) =imin(i);
                    end
                end

                tau = obj.M_ * dot_nu_r + obj.D_ * nu_r + C_ * nu_r...
                        - Rt * (obj.kp_ * ep + obj.kd_ * obj.s_ + obj.ki_ * obj.integrator_);
                tau = tau /1e3; 
                tau_cable = obj.CableForce(obj);
                tau = tau - tau_cable;
                obj.tau_ = tau;
                obj.tau_history_ = [obj.tau_history_, tau];
                obj.tau_cable_history_ = [obj.tau_cable_history_, tau_cable];
                
        end
        
        % thrusters allocation
        function obj = ThrusterAllocation(obj)
            x = zeros(9,1);
            obj.f0_ = 100*ones(3, 1);   obj.a0_ = pi/3*ones(3, 1);
            x0 = [obj.f0_; obj.a0_; zeros(3, 1)];
            lb = [obj.f_range_(1)*ones(3, 1); obj.a_range_(1)*ones(3,1); -1e10*ones(3,1)];
            ub = [obj.f_range_(2)*ones(3, 1); obj.a_range_(2)*ones(3,1); 1e10*ones(3,1)];

            options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'display', 'off');
            [x, fval] = fmincon(@(x)obj.ObjectFunction(x), x0, [], [], [], [], ...
                                    lb, ub, @(x)obj.NonlinearConstrain(x), options);

            f = x(1:3);
            a = x(4:6);
            s = x(7:9);

            T = obj.ThrusterConfiguration( a, obj.L_);
            tau_r = T*f;    
            obj.tau_r_ = tau_r;     obj.f_ = f;     obj.a_ = a;
            obj.f0_ = f;    obj.a0_ = a;
            obj.tau_r_history_ = [obj.tau_r_history_, tau_r];
            obj.f_history_ = [obj.f_history_, f];
            obj.a_history_ = [obj.a_history_, a];
        end
        

        % thruster configuration
        function T = ThrusterConfiguration( obj, a, L)
        % function thruster_configuration is used to obtain the configuration
            T = zeros(3, 3);
            for i = 1 : 3
                T(1,i) = cos(a(i));
                T(2,i) = sin(a(i));
                T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
            end
        end
        
        % object function
        function J = ObjectFunction(obj, x)
            % 误差权重
            P = 1e6*eye(3);
            Q = 1e12 * eye(3);
            Omega = 1e10*eye(3);
            
            f = x(1:3);
            a = x(4:6);
            s = x(7:9);
            a0 = obj.a0_;
            da = a - a0;
            J = s'*Q*s + f'*P*f + da'*Omega*da;
        end
        
        % nonlinear constrain
        function [c, ceq] = NonlinearConstrain(obj, x)

            f = x(1:3);
            a = x(4:6);
            s = x(7:9);

            ceq = zeros(3,1);
            L = [6, -2.7, 0; 6, 2.7, 0; -11, 0, 0];
            T = obj.ThrusterConfiguration( a, L);
            
            tau = obj.tau_;

            ceq(1:3) = tau-T*f-s;
            c = [];

        end
        
        % compute the cable force in the tug body framework
        function f_cable = CableForce(obj)
            p0 = obj.p0_;   p1 = obj.p_;    K = obj.K_;
            L = obj.L_;     Length = obj.Length_;   Link = obj.Link_;
            fai0 = p0(3);   R0 = [cos(fai0), -sin(fai0); sin(fai0), cos(fai0)];
            fai1 = p1(3);   R1 = [cos(fai1), -sin(fai1); sin(fai1), cos(fai1)];
            
            
            pe_cable_start = R0*Link+p0(1:2);
            pe_cable_end = p1(1:2);
            
            pe_cable = pe_cable_end - pe_cable_start;  % 矢量方向从母船指向拖船
            length = sqrt(pe_cable(1)^2 + pe_cable(2)^2);
            if length<Length 
                fc = 0;
            else
                fc = (length - Length) * K;
            end
            % the angle of cable in the earth coordinate
            alpha=atan2(-pe_cable(2), -pe_cable(1));
            belta = alpha - fai1;
            
            f_cable = fc*[cos(belta), sin(belta)]';
            
        end
            
        % Rotation matrix
        function R=RotationMatrix(~, fai)
%             fai=p(3);
            R = [cos(fai), -sin(fai) 0; 
                sin(fai), cos(fai), 0;
                0,         0,       1];
        end

        % C-C force vector 
        function C = CCVector(obj, vel)
            M = obj.M_;
            C = zeros(3, 1);
            u1 = vel(1); u2 = vel(2); u3 = vel(3);
            C = [-M(2,2)*u2*u3-M(2,3)*u3^2; 
                 M(1,1)*u1*u3; 
                 M(3,2)*u1*u3+(-M(1,1)+M(2,2))*u1*u2];
        end
        
        % set position of mothership p0
        function obj = setPose0(obj, p0)
            obj.p0_ = p0;
            obj.p0_history_ = [obj.p0_history_, p0];
        end
        
        function obj = setPd(obj, pd, vd, ad)
            obj.pd_ = pd;   obj.vd_ = vd;   obj.ad_ = ad;
            obj.pd_history_ = [obj.pd_history_, pd];   
            obj.vd_history_ = [obj.vd_history_, vd];
            obj.ad_history_ = [obj.ad_history_, ad];
        end
        
        function obj = setTau(obj, tau)
            obj.tau_ = tau;
        end
        
        % compute the motion equations
        function obj = ShipDynamic(obj)
            
            dt = obj.dt_;   M = obj.M_;
            D = obj.D_;     
            F = obj.tau_;
%             F = obj.tau_r_;
            p0 = obj.p_;     v0 = obj.v_;
            % yaw angle
            fai = p0(3);
            R = obj.RotationMatrix(fai);
            C = obj.CCVector(v0);

            % R-k4 method
            k1=zeros(6,1);
            k2=zeros(6,1);
            k3=zeros(6,1);
            k4=zeros(6,1);

            % state, position and velocity
            y=zeros(6,1);
            y=[v0; p0];

            v0_temp=zeros(3,1);
            k1(1:3)=inv(M)*(F-C-D*v0);
            k1(4:6)=R*v0;

            % k2
            y_temp=y+dt/2*k1;%自变量
            v0_temp=y_temp(1:3);
            C = obj.CCVector(v0_temp);
            fai=y_temp(6);
            R = obj.RotationMatrix(fai);
            k2(1:3)=inv(M)*(F-C-D*v0_temp);
            k2(4:6)=R*y_temp(1:3);

            % k3
            y_temp=y+dt/2*k2;
            v0_temp=y_temp(1:3);
            C = obj.CCVector(v0_temp);
            fai=y_temp(6);
            R = obj.RotationMatrix(fai);
            k3(1:3)=inv(M)*(F-C-D*v0_temp);
            k3(4:6)=R*y_temp(1:3);

            % k4
            y_temp=y+dt*k3;
            v0_temp=y_temp(1:3);
            C = obj.CCVector(v0_temp);
            fai=y_temp(6);
            R = obj.RotationMatrix(fai);
            k4(1:3)=inv(M)*(F-C-D*v0_temp);
            k4(4:6)=R*y_temp(1:3);

            % sum k1~k4
            y_next = y+dt/6*(k1+2*k2+2*k3+k4);

            % position and velocity of next step
            p_next = y_next(4:6);
            v_next = y_next(1:3);
            
            obj.p_ = p_next;    obj.v_ = v_next;
            obj.p_history_ = [obj.p_history_, p_next];
            obj.v_history_ = [obj.v_history_, p_next];
            obj.time_history_ = [obj.time_history_, obj.time_history_(end)+dt];
        end
        
    end
    
end