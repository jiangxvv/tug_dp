#include <QCoreApplication>
#include<iostream>

#include"single.h"
//注意一定要import这个头文件

using namespace std;
int main(int argc, char *argv[])
{
    //使用Singleton::instance()就可以获取参数管理器的实例了，不需要生成
    cout<<Singleton::instance()->get_int_test()<<endl;
    cout<<Singleton::instance()->get_double_test()<<endl;
    cout<<Singleton::instance()->get_bool_test()<<endl;

    Singleton::instance()->set_int_test(5);
    cout<<Singleton::instance()->get_int_test()<<endl;
}
