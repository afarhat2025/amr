#include "main_gui.h"
#include "amr_node.h"
#include <QVBoxLayout>
#include <QWindow>
#include <QPushButton>
#include <QStackedWidget>
#include <QProcess>
#include <QTimer>
#include <QLabel>
#include <QDebug>
#include <QDateTime>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <QDir>
#include <QIODevice>
#include <yaml-cpp/yaml.h>
#include <OgreColourValue.h>
#include <rviz_common/display_group.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_default_plugins/displays/robot_model/robot_model_display.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/view_manager.hpp>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
    


MainGui::MainGui(std::shared_ptr<AmrNode> amr_node,
    QWidget *parent)
: QMainWindow(parent), 
amrNode(amr_node)
{
    centralWidget = new QWidget(this);
    QTimer* timer = new QTimer(this);

    estopActive = false;
    driveMode = true; // false is manual, true is auto  
    pinState = true;


    mainScreen = new MainScreen();
    homeScreen = new HomeScreen();
    motorScreen = new MotorScreen();

    mainScreen->setStyleSheet(R"(
            #MainScreen {
                background-color:rgb(206, 235, 245);
                border: 3px solid rgb(28, 114, 201);
                border-radius: 10px;
            }
    )");


    auto mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->addWidget(mainScreen);

    QString Path = QDir::homePath() + "/AMRMAIN/src/amr_v4_qt//";
    QString assetPath = Path+"fisher_logo.png";
    QPixmap fisherLogo(assetPath);
    homeScreen->ui.fisherLogo->setPixmap(fisherLogo);
    motorScreen->ui.fisherLogo->setPixmap(fisherLogo);

    homeScreen->ui.chargeButton->setEnabled(false);
    homeScreen->ui.chargeButton->setStyleSheet("QPushButton { background: lightgray; color: black }");

    // this section controls the main content on the screen
    mainScreen->ui.mainContent->addWidget(homeScreen);
    mainScreen->ui.mainContent->addWidget(motorScreen);


    mainScreen->ui.mainContent->setCurrentWidget(homeScreen);
    
    // this section connects the tabs(push buttons) to their main screens
    connect(homeScreen->ui.motorScreenButton, &QPushButton::clicked, this, &MainGui::onTabClicked);
    connect(motorScreen->ui.homeScreenButton, &QPushButton::clicked, this, &MainGui::onTabClicked);

    // connecting modeSwitch button
    connect(homeScreen->ui.modeButton, &QPushButton::clicked, this, &MainGui::updateMode); // Connect the mode switch button
    connect(homeScreen->ui.pinButton, &QPushButton::clicked, this, &MainGui::updatePin);
    
    connect(homeScreen->ui.chargeButton, &QPushButton::clicked, this, &MainGui::toggleChargingButton);

    // this section connects the ros2 nodes to their respective screen sections
    connect(amr_node.get(), &AmrNode::changedBattery, this, &MainGui::updateBattery);
    connect(amr_node.get(), &AmrNode::changedMotor, this, &MainGui::updateMotors);
    connect(amr_node.get(), &AmrNode::changedError, this, &MainGui::updateError);
    connect(amr_node.get(), &AmrNode::changedRobot, this, &MainGui::updateRobot);
    connect(amr_node.get(), &AmrNode::changedEstop, this, &MainGui::updateEstop);
    // Connect the changedStation signal to the updateStationId slot
    connect(amrNode.get(), &AmrNode::changedStation, this, &MainGui::updateStationId);


    connect(mainScreen->ui.exitButton, &QPushButton::clicked, this, &MainGui::closeWindow);
    connect(mainScreen->ui.minimizeButton, &QPushButton::clicked, this, &MainGui::minimizeWindow);

    connect(timer, &QTimer::timeout, this, &MainGui::updateWifi);
    timer->start(1000); // Start the timer with a 1-second interval

    setAttribute(Qt::WA_AcceptTouchEvents);

    QTimer::singleShot(0, this, &MainGui::embedRViz);

    // TF Buffer and Listener
    tfBuffer = std::make_shared<tf2_ros::Buffer>(amrNode->get_clock());
    tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, amrNode);

    // Subscribe to cmd_vel
    cmdVelSub = amrNode->create_subscription<geometry_msgs::msg::Twist>(
        "diffbot_base_controller/cmd_vel_unstamped", 10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
            onCmdVelReceived(msg);
        }
    );

    QTimer* tfPollTimer = new QTimer(this);
    connect(tfPollTimer, &QTimer::timeout, this, &MainGui::updateRobotViewFromTF);
    tfPollTimer->start(100);  // update every 0.5 seconds

    followTimer = new QTimer(this);
    followTimer->setSingleShot(true);
    connect(followTimer, &QTimer::timeout, this, [this]() {
        followEnabled = true;
        //qDebug() << "[Follow] Auto-follow enabled after 5 seconds.";
    });

    setCentralWidget(centralWidget);
    setWindowFlags(Qt::FramelessWindowHint);

}

MainGui::~MainGui()
{
    cleanupRViz();
}


void MainGui::onTabClicked() {
    QObject* senderObj = sender();
    switchToTabScreen(senderObj);
}

void MainGui::switchToTabScreen(QObject* senderObj) {
    if(senderObj == homeScreen->ui.motorScreenButton) {
        mainScreen->ui.mainContent->setCurrentWidget(motorScreen);
    }
    else if(senderObj == motorScreen->ui.homeScreenButton) {
        mainScreen->ui.mainContent->setCurrentWidget(homeScreen);
    }
}
void MainGui::updateError(const bool &slam_lidar, const bool &estop_lidar, const bool &camera)
{
    if(slam_lidar)
    {
        homeScreen->ui.hesaiLabel->setStyleSheet("QLabel { background: green; color: white }");
    }
    else
    {
        homeScreen->ui.hesaiLabel->setStyleSheet("QLabel { background: red; color: black}");
    }
    if(estop_lidar)
    {
        homeScreen->ui.sickLabel->setStyleSheet("QLabel { background: green; color: white }");
    }
    else
    {
        homeScreen->ui.sickLabel->setStyleSheet("QLabel { background: red; color: black }");
    }
    if(camera)
    {
        homeScreen->ui.cameraLabel->setStyleSheet("QLabel { background: green; color: white }");
    }
    else
    {
        homeScreen->ui.cameraLabel->setStyleSheet("QLabel { background: red; color: black }");
    }
    
    

}
void MainGui::updateEstop(const bool &data) {
   
    if(data){
        homeScreen->ui.estopLabel->setStyleSheet("QLabel { background: #ff0000 }");
        motorScreen->ui.estopLabel->setStyleSheet("QLabel { background: #ff0000 }");
    } else{
        // homeScreen->ui.estopLabel->setStyleSheet("QLabel { background: #555 }");
        // motorScreen->ui.estopLabel->setStyleSheet("QLabel { background: #555 }");
        homeScreen->ui.estopLabel->setStyleSheet("");
        motorScreen->ui.estopLabel->setStyleSheet("");
    }
}


void MainGui::updateStationId(const QString &stationId)
{
    homeScreen->ui.stationIdLabel->setText("Station:" + stationId);  // Update the QLabel text
}


void MainGui::updateWifi() {
    // update this to use the wifi logo based on strength
    int signalStrength = getWifiSignalStrength();
    int squareSize = 60; // Set the size of the square
    QString Path = QDir::homePath() + "/AMRMAIN/src/amr_v4_qt/assets/";

    int minDbm = -90;
    int maxDbm = -30;
   
    if (signalStrength != -1) {
        int percentage = (signalStrength - minDbm) * 100 / (maxDbm - minDbm);
        QString wifiLogoPath;
        if (percentage >= 70) {
            wifiLogoPath = Path + "wifi3.png";  // Strongest signal
            //qDebug() << "Selected WiFi logo: wifi1.png (Signal Strength: " << percentage << "dBm)";
        } else if (percentage >= 40 && percentage < 70) {
            wifiLogoPath = Path + "wifi2.png";  // Medium signal
            //qDebug() << "Selected WiFi logo: wifi2.png (Signal Strength: " << percentage << "dBm)";
        } else {
            wifiLogoPath = Path + "wifi1.png";  // Very weak signal
            //qDebug() << "Selected WiFi logo: wifi4.png (Signal Strength: " << percentage << "dBm)";
        }


        if (!wifiLogoPath.isEmpty()) {
            QPixmap wifiMap(wifiLogoPath);
            wifiMap = wifiMap.scaled(squareSize, squareSize, Qt::KeepAspectRatio, Qt::SmoothTransformation); // Scale pixmap to squareSize while keeping aspect ratio
            mainScreen->ui.wifiLabel->setPixmap(wifiMap);
        }

    } else {
        // Optionally, set a default or placeholder pixmap when WiFi strength is N/A
        QPixmap defaultPixmap(Path+"wifiDefault.png");
        defaultPixmap = defaultPixmap.scaled(squareSize, squareSize, Qt::KeepAspectRatio, Qt::SmoothTransformation); // Scale pixmap to squareSize while keeping aspect ratio
        mainScreen->ui.wifiLabel->setPixmap(defaultPixmap);
    }

    // Set the QLabel to a fixed square size
    //mainScreen->ui.wifiLabel->setFixedSize(squareSize, squareSize);
    mainScreen->ui.wifiLabel->setAlignment(Qt::AlignCenter);
    //mainScreen->ui.wifiLabel->setScaledContents(true); // This will ignore aspect ratio; remove if you want to keep the pixmap's aspect ratio.
}

int MainGui::getWifiSignalStrength() {
    QProcess process;
    QString program = "iw";
    QStringList arguments;
    arguments << "dev" << "wlan0" << "link";  
    process.start(program, arguments);
    process.waitForFinished();

    QString result = process.readAllStandardOutput();

    QRegularExpression regex("signal:\\s*(-?\\d+)");
    QRegularExpressionMatch match = regex.match(result);
    if (match.hasMatch()) {
        int signalStrength = match.captured(1).toInt(); 
        return signalStrength;
    } else {
        qDebug() << "Signal strength not found in the output.";
        return -1; 
    }
}


void MainGui::updateBattery(const float &voltage,
                            const float &current,
                            const float &soc,
                            const QString &temperature)
{
    mainScreen->ui.percentageLabel->setText(QString("%1").arg(soc) + "%");
    mainScreen->ui.voltageLabel->setText(QString("%1").arg(voltage) + "V");
    mainScreen->ui.currentLabel->setText(QString("%1").arg(current) + "A");
    mainScreen->ui.tempLabel->setText(QString("%1").arg(temperature) + "°C");

}
void MainGui::toggleChargingButton()
{
    if (homeScreen->ui.chargeButton->text() == "Start Charging") {
        amrNode->createChargingPublisher(); // Ensure the publisher is created
        homeScreen->ui.chargeButton->setText("Charging");
        homeScreen->ui.chargeButton->setStyleSheet("QPushButton { background: green; color: white }");
        amrNode->publishChargingMessage(true); // Start charging
        amrNode->updateChargingState(true);    // Start listening to charge current
        
        connect(amrNode.get(), &AmrNode::changedBattery, this, [this](float voltage, float current, float soc) {
            Q_UNUSED(voltage); // Mark unused variables
            Q_UNUSED(soc);
            mainScreen->ui.currentLabel->setText(QString("%1 A").arg(current));
        });
    } else {
        amrNode->publishChargingMessage(false); // Stop charging
        amrNode->updateChargingState(false);    // Stop listening to charge current
        amrNode->destroyChargingPublisher();    // Destroy the publisher
        homeScreen->ui.chargeButton->setText("Start Charging");
        homeScreen->ui.chargeButton->setStyleSheet("QPushButton { background: lightgray; color: black }");
        //homeScreen->ui.chargeCurrent->setText("Charge Current: 0.0 A");
        mainScreen->ui.currentLabel->setText("0.0 A");
    }
}

void MainGui::updateRobot(const QString &data)
{
    mainScreen->ui.scrollingLabel->setText(QString("%1").arg(data));
}

void MainGui::updateMotors(const QString &output_current_right,
                      const QString &output_current_left,
                      const QString &output_current_pin,
                      const QString &error_right,
                      const QString &error_left,
                      const QString &error_pin)
{
    motorScreen->ui.rightMotorCurrent->setText(QString("%1").arg(output_current_right));
    motorScreen->ui.leftMotorCurrent->setText(QString("%1").arg(output_current_left));
    motorScreen->ui.pinMotorCurrent->setText(QString("%1").arg(output_current_pin));
    motorScreen->ui.rightMotorError->setText(QString("%1").arg(error_right));
    motorScreen->ui.leftMotorError->setText(QString("%1").arg(error_left));
    motorScreen->ui.pinMotorError->setText(QString("%1").arg(error_pin));
}

void MainGui::updateMode()
{
    driveMode = !driveMode;
    amrNode->mode_callback(driveMode);

    if (driveMode) {
        homeScreen->ui.modeButton->setText(QString("%1").arg("Mode: Auto"));
        homeScreen->ui.modeButton->setStyleSheet("QPushButton { background: green; color: white }");
        homeScreen->ui.chargeButton->setEnabled(false); // Disable charging in Auto mode
        homeScreen->ui.chargeButton->setStyleSheet("QPushButton { background: lightgray; color: black }");
    } else {
        homeScreen->ui.modeButton->setText(QString("%1").arg("Mode: Manual"));
        homeScreen->ui.modeButton->setStyleSheet("QPushButton { background: yellow; color: black }");
        homeScreen->ui.chargeButton->setEnabled(true); // Enable charging in Manual mode
        homeScreen->ui.chargeButton->setStyleSheet("QPushButton { background: green; color: white }");
    }
}

void MainGui::updatePin()
{
    pinState = !pinState;
    amrNode->pin_callback(pinState);
    if(pinState)
    {
        homeScreen->ui.pinButton->setText(QString("%1").arg("Pin: Up"));
    } else {
        homeScreen->ui.pinButton->setText(QString("%1").arg("Pin: Down"));
    }
}

void MainGui::minimizeWindow()
{
    this->QWidget::setWindowState(Qt::WindowMinimized);
}

void MainGui::closeWindow()
{
    this->close();
    exit(0);
    
}
void MainGui::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        m_lastMousePos = event->globalPos();
        m_initialSize = size();
        m_dragging = true;
        m_resizing = (event->pos().x() >= width() - 10 && event->pos().y() >= height() - 10);
        event->accept();
    } else {
        QMainWindow::mousePressEvent(event);
    }
}

void MainGui::mouseMoveEvent(QMouseEvent *event) {
    if (m_dragging) {
        if (m_resizing) {
            // Resizing logic
            QPoint delta = event->globalPos() - m_lastMousePos;
            QSize newSize = m_initialSize + QSize(delta.x(), delta.y());
            // Get the screen resolution
            QScreen *screen = QGuiApplication::primaryScreen();
            if (!screen) {
                qWarning() << "No primary screen found!";
                return;
            }
            QRect screenGeometry = screen->geometry();
            int maxWidth = static_cast<int>(screenGeometry.width() * 0.8);
            int maxHeight = static_cast<int>(screenGeometry.height() * 0.8);
            // Ensure the window size is within the allowed bounds
            if (newSize.width() >= minimumWidth() && newSize.width() <= maxWidth &&
                newSize.height() >= minimumHeight() && newSize.height() <= maxHeight) {
                resize(newSize.width(), newSize.height());
            }
        } else {
            // Dragging logic
            QPoint delta = event->globalPos() - m_lastMousePos;
            move(x() + delta.x(), y() + delta.y());
            m_lastMousePos = event->globalPos();
        }
        event->accept();
    } else {
        QMainWindow::mouseMoveEvent(event);
    }
}

void MainGui::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        m_dragging = false;
        m_resizing = false;  // Reset resizing flag
        event->accept();
    } else {
        QMainWindow::mouseReleaseEvent(event);
    }
}

bool MainGui::eventFilter(QObject* watched, QEvent* event) {
    pollMouseAndPan(event);
   
    return false; // Allow RViz to process it too
}

void MainGui::pollMouseAndPan(QEvent* event) {
    QRect renderRect = renderPanel->rect();
    QPoint cursorInRender = renderPanel->mapFromGlobal(QCursor::pos());

    if (renderPanel->underMouse() || homeScreen->ui.map_widget->underMouse() || renderRect.contains(cursorInRender)) {
        Qt::MouseButtons buttons = QApplication::mouseButtons();
        QPoint globalCursorPos = QCursor::pos();
        QPoint localCursorPos = renderPanel->mapFromGlobal(globalCursorPos);

        // Handle zoom if it's a wheel event
        if (event && event->type() == QEvent::Wheel) {
            QWheelEvent* wheelEvent = dynamic_cast<QWheelEvent*>(event);
            if (wheelEvent && currentView) {
                // Use ViewportMouseEvent for scaling logic
                rviz_common::ViewportMouseEvent vme(renderPanel, wheelEvent, 0, 0);

                double scale = currentView->subProp("Scale")->getValue().toDouble();
                int delta = vme.wheel_delta;

                double zoomFactor = (delta > 0) ? 1.1 : 0.9;
                scale *= zoomFactor;
                currentView->subProp("Scale")->setValue(scale);

                //qDebug() << "[ViewportMouse] Zoom delta:" << delta << "New scale:" << scale;
            }
        }

        // Handle dragging
        if (buttons & Qt::LeftButton) {
            if (!dragging) {
                dragging = true;
                lastCursorPos = localCursorPos;
                return;
            }

            QPoint delta = localCursorPos - lastCursorPos;
            lastCursorPos = localCursorPos;

            if (currentView) {
                double scale = currentView->subProp("Scale")->getValue().toDouble();
                double x = currentView->subProp("X")->getValue().toDouble();
                double y = currentView->subProp("Y")->getValue().toDouble();

                currentView->subProp("X")->setValue(x - delta.x() / scale);
                currentView->subProp("Y")->setValue(y + delta.y() / scale);
            }
        } else {
            dragging = false;
        }
    } else {
        dragging = false;  // Stop dragging if we're outside
    }
}


void MainGui::embedRViz() {
    QWidget* containerParent = homeScreen->ui.map_widget;

    // Safety check for map_widget
    if (!containerParent || reinterpret_cast<uintptr_t>(containerParent) < 0x1000) {
        qWarning() << "[RViz] map_widget is invalid or null. Using homeScreen instead.";
        containerParent = homeScreen;
    }

    // Create RViz container and layout
    rvizContainer = new QWidget(containerParent);
    QVBoxLayout* rvizLayout = new QVBoxLayout();
    rvizLayout->setContentsMargins(0, 0, 0, 0);
    rvizContainer->setLayout(rvizLayout);

    // Add RenderPanelf
    renderPanel = new rviz_common::RenderPanel(rvizContainer);
    rvizLayout->addWidget(renderPanel);

    // Inject RViz container into the UI layout (QVBox inside map_widget)
    QLayout* existingLayout = containerParent->layout();
    if (!existingLayout) {
        QVBoxLayout* newLayout = new QVBoxLayout(containerParent);
        newLayout->setContentsMargins(0, 0, 0, 0);
        containerParent->setLayout(newLayout);
        existingLayout = newLayout;
    }
    existingLayout->addWidget(rvizContainer);

    renderPanel->show();
    QApplication::processEvents();

    // inputPoller = new QTimer(this);
    // connect(inputPoller, &QTimer::timeout, this, &MainGui::pollMouseAndPan);
    // inputPoller->start(30);  // ~33 FPS polling


    embedRVizCore();
}

void MainGui::embedRVizCore() {
    if (!renderPanel) {
        qWarning() << "[RViz] renderPanel or rosNodeAbstraction missing!";
        return;
    }

    vizManager = std::make_shared<rviz_common::VisualizationManager>(
        renderPanel, amrNode, nullptr, amrNode->get_clock());

    renderPanel->setMouseTracking(true);
    //renderPanel->installEventFilter(this);
    renderPanel->initialize(vizManager.get());
    renderPanel->setFocusPolicy(Qt::StrongFocus);
    vizManager->initialize();
    vizManager->setFixedFrame("map");
    vizManager->getViewManager()->initialize();
    vizManager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");

    viewManager = vizManager->getViewManager();
    currentView = viewManager->getCurrent();

    if (currentView) {
       
        currentView->subProp("Scale")->setValue(13.5);  // larger = zoomed in
        currentView->subProp("X")->setValue(-4);
        currentView->subProp("Y")->setValue(-4);
    }
    

    //vizManager->updateBackgroundColor(QColor(26, 95, 180));
    vizManager->startUpdate();

    gridDisplay = vizManager->createDisplay("rviz_default_plugins/Grid", "Grid View", true);
    if (gridDisplay == nullptr) {
        throw std::runtime_error("Error creating grid display");
    }

    mapDisplay = vizManager->createDisplay("rviz_default_plugins/Map", "adjustable map", true);
    if (mapDisplay == NULL) {
        throw std::runtime_error("Error creating map display");
    }
    mapDisplay->subProp("Topic")->setValue("map");
    mapDisplay->subProp("Topic")->subProp("Depth")->setValue(1);
    mapDisplay->subProp("Topic")->subProp("History Policy")->setValue("Keep Last");
    mapDisplay->subProp("Topic")->subProp("Reliability Policy")->setValue("Reliable");
    mapDisplay->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");
    mapDisplay->subProp("Topic")->subProp("Filter size")->setValue(10);
    mapDisplay->subProp("Update Topic")->setValue("map_updates");
    mapDisplay->subProp("Alpha")->setValue(0.9);
    mapDisplay->subProp("Color Scheme")->setValue("map");
    mapDisplay->subProp("Draw Behind")->setValue(true);
    mapDisplay->subProp("Resolution")->setValue(0.05f);
    mapDisplay->subProp("Width")->setValue(1045);
    mapDisplay->subProp("Width")->setValue(764);

    robotDisplay = vizManager->createDisplay("rviz_default_plugins/RobotModel", "Robot Model", true);
    if (robotDisplay == nullptr) {
        throw std::runtime_error("Error creating robot display");
    }
    robotDisplay->subProp("Topic")->setValue("robot_description");
    robotDisplay->subProp("Update Interval")->setValue(0.1);
    robotDisplay->subProp("Description Topic")->setValue("robot_description");
    
    globalCostmapDisplay = vizManager->createDisplay("rviz_default_plugins/Map", "Global Planner Costmap", true);
    if (globalCostmapDisplay == NULL) {
        throw std::runtime_error("Error creating costmap display");
    }
    globalCostmapDisplay->subProp("Topic")->setValue("global_costmap/costmap");
    globalCostmapDisplay->subProp("Topic")->subProp("Depth")->setValue(1);
    globalCostmapDisplay->subProp("Topic")->subProp("History Policy")->setValue("Keep Last");
    globalCostmapDisplay->subProp("Topic")->subProp("Reliability Policy")->setValue("Reliable");
    globalCostmapDisplay->subProp("Topic")->subProp("Durability Policy")->setValue("Transient Local");
    globalCostmapDisplay->subProp("Topic")->subProp("Filter size")->setValue(10);
    globalCostmapDisplay->subProp("Update Topic")->setValue("global_costmap/costmap_updates");
    globalCostmapDisplay->subProp("Alpha")->setValue("0.3");
    globalCostmapDisplay->subProp("Color Scheme")->setValue("costmap");
    globalCostmapDisplay->subProp("Resolution")->setValue(0.05);
    globalCostmapDisplay->subProp("Width")->setValue(1045);
    globalCostmapDisplay->subProp("Width")->setValue(764);
  

    globalPathDisplay = vizManager->createDisplay("rviz_default_plugins/Path", "Global Path Display", true);
    if (globalCostmapDisplay == NULL) {
        throw std::runtime_error("Error creating paths display");
    }
    globalPathDisplay->subProp("Topic")->setValue("plan");
    globalPathDisplay->subProp("Line Style")->setValue("Lines");
    globalPathDisplay->subProp("Color")->setValue(QColor(255, 0, 0));
    globalPathDisplay->subProp("Pose Style")->setValue("Arrows");
    globalPathDisplay->subProp("Shaft Length")->setValue("0.02");
    globalPathDisplay->subProp("Head Length")->setValue("0.02");
    globalPathDisplay->subProp("Shaft Diameter")->setValue("0.005");
    globalPathDisplay->subProp("Head Diameter")->setValue("0.02");

    polygonDisplay = vizManager->createDisplay("rviz_default_plugins/Polygon", "Polygon Display", true);
    if (globalCostmapDisplay == NULL) {
        throw std::runtime_error("Error creating costmap display");
    }
    polygonDisplay->subProp("Topic")->setValue("global_costmap/published_footprint");
    polygonDisplay->subProp("Color")->setValue(QColor(25, 255, 0));

    poseArrayDisplay = vizManager->createDisplay("rviz_default_plugins/PoseArray", "PoseArray Display", true);
    if (globalCostmapDisplay == NULL) {
        throw std::runtime_error("Error creating PoseArray display");
    }
    poseArrayDisplay->subProp("Topic")->setValue("landmarks");

    qApp->installEventFilter(this);
}

void MainGui::onCmdVelReceived(geometry_msgs::msg::Twist::SharedPtr msg)
{
    lastCmdVelLinear = msg->linear.x;

    if (lastCmdVelLinear > 0.03 && !followEnabled) {
        if (followTimer == nullptr) {
            followTimer = new QTimer(this);
            followTimer->setSingleShot(true);

            connect(followTimer, &QTimer::timeout, this, [this]() {
                followEnabled = true;
                //qDebug() << "Auto-follow enabled after 5s.";
            });
        }

        if (!followTimer->isActive()) {
            //qDebug() << "Movement detected. Starting 5s timer.";
            followTimer->start(5000);
        }

    } else if (lastCmdVelLinear < 0.01) {
        if (followTimer && followTimer->isActive()) {
            //qDebug() << "Movement stopped. Canceling timer.";
            followTimer->stop();
        }

        followEnabled = false;  // stop following if robot stopped
    }
}

void MainGui::updateRobotViewFromTF()
{
    try {
        geometry_msgs::msg::TransformStamped transformStamped = tfBuffer->lookupTransform("map", "amr_1/base_link", tf2::TimePointZero);
        double x = transformStamped.transform.translation.x;
        double y = transformStamped.transform.translation.y;

        updateViewPosition(x, y); 
    }
    catch (const tf2::TransformException &ex) {
        qDebug() << "[TF Error]" << ex.what();
    }
}

void MainGui::updateViewPosition(double x, double y)
{
    if (!currentView) return;

    // Robot is moving — after 5 seconds, switch to TopDownOrtho and zoom out
    if (followEnabled) {
        currentView = viewManager->getCurrent();
        

        currentView->subProp("X")->setValue(x);   // Note: flipped axes
        currentView->subProp("Y")->setValue(y);
        currentView->subProp("Scale")->setValue(25.0); // Zoomed out
    } 
}

void MainGui::cleanupRViz()
{
    if (vizManager) {
        vizManager->stopUpdate();
        vizManager.reset();
    }

    if (renderPanel) {
        renderPanel->setParent(nullptr);
        delete renderPanel;
        renderPanel = nullptr;
    }
    
    if (rvizContainer) {
        rvizContainer->deleteLater();
        rvizContainer = nullptr;
    }
    
}