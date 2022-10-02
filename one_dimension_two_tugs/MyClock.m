classdef(ConstructOnLoad=true,Sealed) MyClock  
    %����������� T= MyClock(8,30,20)���г���
    %ConstructOnLoad=true,���ظ���ʱ����øö���Ĺ��캯����
    properties
        Hour;   %ʱ
        Minute;    %��
        Second;   %��
    end
    methods
        function obj=MyClock(h,m,s)  %���캯��
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