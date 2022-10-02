classdef(ConstructOnLoad=true,Sealed) MyClock  
    %在命令窗口输入 T= MyClock(8,30,20)运行程序
    %ConstructOnLoad=true,加载该类时会调用该对象的构造函数。
    properties
        Hour;   %时
        Minute;    %分
        Second;   %秒
    end
    methods
        function obj=MyClock(h,m,s)  %构造函数
            obj.Hour=h;
            obj.Minute=m;
            obj.Second=s;
        end
        function Show(obj)
            disp([num2str(obj.Hour),':',...
                num2str(obj.Minute,':',num2str(obj.Second))]);
        end
        function value = get.Hour(object)
            value = obj.Hour;
            disp('Querying Hour value')
        end
        function obj = set.Hour(obj,value)
            if~(value>0)
                error('Property value must be positive')
            else
                obj.Hour = value;
            end
            disp('Setting Hour value!')
        end
        
    end
end