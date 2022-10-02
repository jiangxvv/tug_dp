classdef Clock
    
    properties (Access = public)
        hour = 0;
        minute = 0;
        second = 0;
    end
    
    methods
        function obj = Clock(h, m, s)
            obj.hour = h;
            obj.minute = m;
            obj.second = s;
        end
        
        function obj = setHour(obj, h)
            obj.hour = h;
        end
        
        function hour = getHour(obj)
            hour = obj.hour;
        end
        
        function  show(obj, a)
            disp([num2str(obj.hour),':', num2str(obj.minute), ':',num2str(obj.second)]);
              disp(a);
        end
        
    end
    
end
        
        
            
        
    