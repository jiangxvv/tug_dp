classdef CooperativeGuidance
    properties
        p0_; p1_; p2_;
        pd1_;   pd2_;     vd1_;   vd2_;
        input_;
        M0_;     D0_;   M1_;    D1_;
        leader_kp_;    leader_ki_;    leader_kd_;
        follower_kp_;  follower_ki_;  follower_kd_;
        L01_;    L02_;    L12_;
        w_; z_;
        
        
        
    end
    
    methods
        % construction function
        function obj = CooperativeGuidance()
            obj.w_ = 0.005;     obj.z_ = 0.9;
            
        end
        
        
        % virtual leader's reference model depended on the input
        function [pd0, vd0] = ReferenceModle(obj)
           p0 = obj.p0_;    fai0 = p0(3);
            % nonlinear damping
           x = 0;
           v = 0;
           r2 = 10;
           y2 = zeros(N+1,2);
           for i=1:N+1,
                y2(i,:) = [x v];   
                 x_dot = v;
                 v_dot = w^2*(r2-x) - 2*z*w*v - delta*abs(v)*v;
                 v = v + h*v_dot;
                 x = x + h*x_dot;
           end
           end
        
        % virtual leader desired position
        function U = PIDLeader(obj)
            
        end
        
        % consensus formation guidance
        function [pd1, vd1, pd2, vd2] = FormationGuidance(obj)
            
        end
        
        % set mothership desired destination(input)
        function input = setInput(obj, input)
            obj.input_ = input;
        end
        
        % get tug's desired position
        function [pd1, vd1] = getTug1Pd(obj)
            pd1 = obj.pd1_;
            vd1 =obj.vd1_;
        end
        
        function [pd2, vd2] = getTug2Pd(obj)
            pd2 = obj.pd2_;
            vd2 =obj.vd2_;
        end
        
        % set p0, p1, p2
        function setPosition(obj, p0, p1, p2)
            obj.p0_ = p0;
            obj.p1_ = p1;
            obj.p2_ = p2;
        end
       
        
        
        
        
        
        
    end
end