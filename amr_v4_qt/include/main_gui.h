#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QMainWindow>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QString>
#include <QPushButton>
#include <QStackedWidget>
#include "motor_screen.h"
#include "home_screen.h"
#include "main_screen.h"
#include "scrolling_label.h"
#include "amr_node.h"
#include <QProcess>
#include <QMouseEvent>
#include <QScreen>
#include <QThread>
#include <QTimer>
#include <QApplication>
#include <QGuiApplication>
#include <QPainter>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include <QMouseEvent>
#include <QWheelEvent>


class MainGui : public QMainWindow {
    Q_OBJECT


public:
    explicit MainGui(std::shared_ptr<AmrNode> amrNode_,
        QWidget *parent = nullptr);

    int getWifiSignalStrength();

    void setupLayout();
    
    void updateBattery(const float &voltage,
                            const float &current,
                            const float &soc,
                            const QString &temperature);
    void onTabClicked();
    void switchToDefaultScreen();
    void switchToTabScreen(QObject* sender);
    void updateWifi();
    void updateMotors(const QString &output_current_right,
                      const QString &output_current_left,
                      const QString &output_current_pin,
                      const QString &error_right,
                      const QString &error_left,
                      const QString &error_pin);
    void updateError(const bool &slam_lidar,
                     const bool &estop_lidar,
                     const bool &camera);
    void updateMode();
    void updateRobot(const QString &data);
    void updateClock();
    void updateEstop(const bool &data);
    void updatePin();
    void checkLidars();
    void lidarThread();
    void closeWindow();
    void minimizeWindow();
    void updateStationId(const QString &stationId);  //update station ID
    void toggleChargingButton(); 
    

    ~MainGui();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event) override;
    //void wheelEvent(QWheelEvent *event) override;


    

private slots:

    void embedRVizCore();


private:
    bool estopActive;
    bool driveMode;
    bool pinState;
    bool m_dragging;
    QPoint m_lastMousePos;
    QRect m_normalGeometry;
    bool m_resizing;
    QSize m_initialSize;
    bool handleTouchEvent(QTouchEvent *event);
    rclcpp::NodeOptions nodeOptions;


    std::shared_ptr<AmrNode> amrNode;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    std::shared_ptr<rviz_common::VisualizationManager> vizManager;


    rviz_common::RenderPanel* renderPanel;
    rviz_common::ViewManager* viewManager;
    rviz_common::ViewController* currentView;
    rviz_common::Display* mapDisplay;
    rviz_common::Display* markerDisplay;
    rviz_common::Display* globalPathDisplay;
    rviz_common::Display* globalCostmapDisplay;
    rviz_common::Display* robotDisplay;
    rviz_common::Display* polygonDisplay;
    rviz_common::Display* poseArrayDisplay;
    rviz_common::Display* nameDisplay;
    rviz_common::Display* gridDisplay;

    QWidget* rvizContainer;
    QWidget* inputOverlay;



    MainScreen* mainScreen;
    HomeScreen*  homeScreen;
    MotorScreen* motorScreen;
    QStackedWidget* mainContent;
    QWidget* centralWidget;
    QHBoxLayout* mainLayout;
    QProcess *chargingProcess = nullptr;
    QPointF lastTouchCenter;

    QTimer* followTimer;
    double lastCmdVelLinear = 0.0;
    bool followEnabled = false;
    
    QProcess *rvizProcess;
    void embedRViz();
    void cleanupRViz();
    void onCmdVelReceived(geometry_msgs::msg::Twist::SharedPtr msg);
    void updateViewPosition(double x, double y);
    void updateRobotViewFromTF();
    void createAndSetupInputOverlay();
    void pollMouseAndPan(QEvent* event);
    const char* robot_name;

    QTimer* inputPoller;
    bool dragging = false;
    QPoint lastCursorPos;
    
   

};

#endif // MAIN_GUI_H
