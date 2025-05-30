#include <QApplication>
#include <QFile>
#include <rclcpp/rclcpp.hpp>
#include "main_gui.h"
#include "amr_node.h"
#include "cstdlib"
#include <QDebug>
#include <QDir>

static void siginthandler(int /*param*/)
{
  QApplication::quit();
}

int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc, argv);

    

    rclcpp::init(argc, argv);

    // Retrieve robot name dynamically from the environment variable
    const char* robot_name = std::getenv("ROBOT_MODEL");
    if(robot_name == nullptr) {
        qDebug() << "Environment variable 'ROBOT_MODEL' not set.";
        return 1;
    }

    rclcpp::NodeOptions options;
    options.arguments({
        "--ros-args",
        "--remap", std::string("__ns:=/") + robot_name,
        "--remap", "__node:=amr_v4_qt",
        "--remap", "/tf:=tf",
        "--remap", "/tf_static:=tf_static"
    });
        
    // Pass the dynamically retrieved robot name to AmrNode
    auto amrNode_ = std::make_shared<AmrNode>(std::string(robot_name), options);
    amrNode_->setSelfPtr(amrNode_);
    auto mainGui = std::make_shared<MainGui>(amrNode_);
    
    QString filePath = QDir::homePath() + "/AMRMAIN/src/amr_v4_qt/include/styles.qss";
    QFile file(filePath);
    if(file.open(QFile::ReadOnly)) {
        QString stylesheet = QLatin1String(file.readAll());
        app.setStyleSheet(stylesheet);
    } else {
        qDebug() << "No style sheet found, fix the path, Current file path is:." << filePath;
    }

    mainGui->setWindowFlags(Qt::FramelessWindowHint);
        QScreen *screen = QGuiApplication::primaryScreen();
    if (!screen) {
        qWarning() << "No primary screen found!";
        return 1;
    }
    QRect screenGeometry = screen->geometry();
    int screenWidth = screenGeometry.width();
    int screenHeight = screenGeometry.height();

    int windowWidth = static_cast<int>(screenWidth * 0.8);
    int windowHeight = static_cast<int>(screenHeight * 0.8);
    mainGui->resize(windowWidth, windowHeight);
    mainGui->move((screenWidth - windowWidth) / 2, (screenHeight - windowHeight) / 2); // Center window
    
    mainGui->show();

    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(amrNode_);

    signal(SIGINT, siginthandler);

    // while (rclcpp::ok()) {
    //     exec.spin_some();
    //     app.processEvents();
    // }

    // exec.remove_node(amrNode_);
    // rclcpp::shutdown();

    return app.exec();
}
