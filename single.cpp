#include"single.h"
Singleton Singleton::m_instance;

Singleton::Singleton()
{
    std::cout<<"LOG: contructure parameter manager"<<std::endl;
}

Singleton::~Singleton()
{
    std::cout<<"LOG: deconstructure parameter manager"<<std::endl;
}

Singleton *Singleton::instance()
{
    return &m_instance;
}
