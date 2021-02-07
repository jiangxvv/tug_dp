#ifndef SINGLE_H
#define SINGLE_H
#include<iostream>

class Singleton{
    public:
        static Singleton *instance();
    private:
        Singleton();
        virtual ~Singleton();
        Singleton(const Singleton&){};
        Singleton& operator=(const Singleton&){};
    private:
        static Singleton m_instance;

        //参数设置在这里，所有的参数都是以private形式保存以保证其安全
    private:
        int int_test = 0;
        double double_test = 1;
        bool bool_test = false;

    public:
        //参数获取，注意在这里要返回函数参数的引用
        const int& get_int_test(){
            return int_test;
        }
        const double& get_double_test(){
            return double_test;
        }
        const bool& get_bool_test(){
            return bool_test;
        }


        //参数设置，这里没有什么特别需要注意的地方
        void set_int_test(int test){
            int_test = test;
        }
        void set_double_test(double test){
            double_test = test;
        }
        void set_bool_test(bool test){
            bool_test = test;
        }
};
#endif // SINGLE_H
