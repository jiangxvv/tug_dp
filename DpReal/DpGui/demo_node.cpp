#include "demo_interface.h"
#include "core_window.h"
#include <QApplication>

class main_example : public QApplication
{
public:
  main_example(int& argc, char** argv) : QApplication(argc, argv)
  {
    ros::init(argc, argv, "gui_example", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle);
  }

  int exec()
  {
    // ExampleInterface window("Oil_rig/Oil_rig/observer/state");
    // window.show();
    CoreWindow widget("Oil_rig/observer/state");
    widget.show();

    return QApplication::exec();
  }

private:
  ros::NodeHandlePtr nh_;
};

int main(int argc, char** argv)
{
  main_example app(argc, argv);
  return app.exec();
}
