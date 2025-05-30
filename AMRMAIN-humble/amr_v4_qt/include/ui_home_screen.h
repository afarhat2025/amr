/********************************************************************************
** Form generated from reading UI file 'home_screen.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HOME_SCREEN_H
#define UI_HOME_SCREEN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HomeScreen
{
public:
    QPushButton *motorScreenButton;
    QLabel *estopLabel;
    QLabel *fisherLogo;
    QLabel *stationIdLabel;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *left_main_vert_layout;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *modeButton;
    QPushButton *pinButton;
    QPushButton *chargeButton;
    QLabel *systemLabel;
    QHBoxLayout *system_status_layout;
    QLabel *cameraLabel;
    QLabel *hesaiLabel;
    QLabel *sickLabel;
    QFrame *map_widget;

    void setupUi(QWidget *HomeScreen)
    {
        if (HomeScreen->objectName().isEmpty())
            HomeScreen->setObjectName(QString::fromUtf8("HomeScreen"));
        HomeScreen->resize(1280, 580);
        motorScreenButton = new QPushButton(HomeScreen);
        motorScreenButton->setObjectName(QString::fromUtf8("motorScreenButton"));
        motorScreenButton->setGeometry(QRect(20, 420, 200, 100));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(motorScreenButton->sizePolicy().hasHeightForWidth());
        motorScreenButton->setSizePolicy(sizePolicy);
        motorScreenButton->setMinimumSize(QSize(200, 100));
        motorScreenButton->setMaximumSize(QSize(300, 100));
        QFont font;
        font.setPointSize(30);
        motorScreenButton->setFont(font);
        estopLabel = new QLabel(HomeScreen);
        estopLabel->setObjectName(QString::fromUtf8("estopLabel"));
        estopLabel->setGeometry(QRect(420, 420, 220, 100));
        QFont font1;
        font1.setPointSize(50);
        estopLabel->setFont(font1);
        estopLabel->setFrameShape(QFrame::Box);
        estopLabel->setLineWidth(2);
        estopLabel->setAlignment(Qt::AlignCenter);
        fisherLogo = new QLabel(HomeScreen);
        fisherLogo->setObjectName(QString::fromUtf8("fisherLogo"));
        fisherLogo->setGeometry(QRect(160, 530, 321, 51));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(fisherLogo->sizePolicy().hasHeightForWidth());
        fisherLogo->setSizePolicy(sizePolicy1);
        fisherLogo->setFrameShape(QFrame::NoFrame);
        fisherLogo->setPixmap(QPixmap(QString::fromUtf8("../../../../../../../.designer/assets/fisher_logo.png")));
        fisherLogo->setScaledContents(true);
        fisherLogo->setAlignment(Qt::AlignCenter);
        stationIdLabel = new QLabel(HomeScreen);
        stationIdLabel->setObjectName(QString::fromUtf8("stationIdLabel"));
        stationIdLabel->setGeometry(QRect(20, 10, 380, 100));
        stationIdLabel->setMinimumSize(QSize(380, 100));
        stationIdLabel->setMaximumSize(QSize(120, 40));
        QFont font2;
        font2.setPointSize(26);
        stationIdLabel->setFont(font2);
        stationIdLabel->setFrameShape(QFrame::Box);
        stationIdLabel->setLineWidth(3);
        stationIdLabel->setAlignment(Qt::AlignCenter);
        verticalLayoutWidget = new QWidget(HomeScreen);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(20, 110, 616, 311));
        left_main_vert_layout = new QVBoxLayout(verticalLayoutWidget);
        left_main_vert_layout->setObjectName(QString::fromUtf8("left_main_vert_layout"));
        left_main_vert_layout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        modeButton = new QPushButton(verticalLayoutWidget);
        modeButton->setObjectName(QString::fromUtf8("modeButton"));
        modeButton->setMinimumSize(QSize(200, 100));
        modeButton->setFont(font);

        horizontalLayout_2->addWidget(modeButton);

        pinButton = new QPushButton(verticalLayoutWidget);
        pinButton->setObjectName(QString::fromUtf8("pinButton"));
        pinButton->setMinimumSize(QSize(200, 100));
        pinButton->setBaseSize(QSize(240, 100));
        pinButton->setFont(font);

        horizontalLayout_2->addWidget(pinButton);

        chargeButton = new QPushButton(verticalLayoutWidget);
        chargeButton->setObjectName(QString::fromUtf8("chargeButton"));
        chargeButton->setMinimumSize(QSize(200, 100));
        chargeButton->setBaseSize(QSize(240, 100));
        chargeButton->setFont(font);

        horizontalLayout_2->addWidget(chargeButton);


        left_main_vert_layout->addLayout(horizontalLayout_2);

        systemLabel = new QLabel(verticalLayoutWidget);
        systemLabel->setObjectName(QString::fromUtf8("systemLabel"));
        sizePolicy.setHeightForWidth(systemLabel->sizePolicy().hasHeightForWidth());
        systemLabel->setSizePolicy(sizePolicy);
        systemLabel->setMinimumSize(QSize(300, 60));
        systemLabel->setMaximumSize(QSize(10000, 40));
        QFont font3;
        font3.setPointSize(35);
        systemLabel->setFont(font3);
        systemLabel->setFrameShape(QFrame::Box);
        systemLabel->setLineWidth(3);
        systemLabel->setAlignment(Qt::AlignCenter);
        systemLabel->setMargin(0);
        systemLabel->setIndent(0);

        left_main_vert_layout->addWidget(systemLabel);

        system_status_layout = new QHBoxLayout();
        system_status_layout->setObjectName(QString::fromUtf8("system_status_layout"));
        system_status_layout->setContentsMargins(-1, -1, -1, 30);
        cameraLabel = new QLabel(verticalLayoutWidget);
        cameraLabel->setObjectName(QString::fromUtf8("cameraLabel"));
        sizePolicy.setHeightForWidth(cameraLabel->sizePolicy().hasHeightForWidth());
        cameraLabel->setSizePolicy(sizePolicy);
        cameraLabel->setMinimumSize(QSize(200, 80));
        cameraLabel->setMaximumSize(QSize(120, 40));
        cameraLabel->setFont(font);
        cameraLabel->setLayoutDirection(Qt::LeftToRight);
        cameraLabel->setFrameShape(QFrame::Box);
        cameraLabel->setFrameShadow(QFrame::Plain);
        cameraLabel->setLineWidth(3);
        cameraLabel->setAlignment(Qt::AlignCenter);
        cameraLabel->setMargin(0);
        cameraLabel->setIndent(0);

        system_status_layout->addWidget(cameraLabel);

        hesaiLabel = new QLabel(verticalLayoutWidget);
        hesaiLabel->setObjectName(QString::fromUtf8("hesaiLabel"));
        sizePolicy.setHeightForWidth(hesaiLabel->sizePolicy().hasHeightForWidth());
        hesaiLabel->setSizePolicy(sizePolicy);
        hesaiLabel->setMinimumSize(QSize(200, 80));
        hesaiLabel->setMaximumSize(QSize(120, 40));
        QFont font4;
        font4.setPointSize(28);
        hesaiLabel->setFont(font4);
        hesaiLabel->setLayoutDirection(Qt::LeftToRight);
        hesaiLabel->setFrameShape(QFrame::Box);
        hesaiLabel->setFrameShadow(QFrame::Plain);
        hesaiLabel->setLineWidth(3);
        hesaiLabel->setScaledContents(false);
        hesaiLabel->setAlignment(Qt::AlignCenter);
        hesaiLabel->setWordWrap(false);
        hesaiLabel->setMargin(0);
        hesaiLabel->setIndent(0);

        system_status_layout->addWidget(hesaiLabel);

        sickLabel = new QLabel(verticalLayoutWidget);
        sickLabel->setObjectName(QString::fromUtf8("sickLabel"));
        sizePolicy.setHeightForWidth(sickLabel->sizePolicy().hasHeightForWidth());
        sickLabel->setSizePolicy(sizePolicy);
        sickLabel->setMinimumSize(QSize(200, 80));
        sickLabel->setMaximumSize(QSize(120, 40));
        sickLabel->setFont(font);
        sickLabel->setFrameShape(QFrame::Box);
        sickLabel->setFrameShadow(QFrame::Plain);
        sickLabel->setLineWidth(3);
        sickLabel->setAlignment(Qt::AlignCenter);
        sickLabel->setMargin(0);
        sickLabel->setIndent(0);

        system_status_layout->addWidget(sickLabel);


        left_main_vert_layout->addLayout(system_status_layout);

        map_widget = new QFrame(HomeScreen);
        map_widget->setObjectName(QString::fromUtf8("map_widget"));
        map_widget->setGeometry(QRect(660, -20, 791, 541));
        map_widget->setMouseTracking(true);
        map_widget->setFocusPolicy(Qt::StrongFocus);
        map_widget->setFrameShape(QFrame::Box);
        map_widget->setFrameShadow(QFrame::Raised);

        retranslateUi(HomeScreen);

        QMetaObject::connectSlotsByName(HomeScreen);
    } // setupUi

    void retranslateUi(QWidget *HomeScreen)
    {
        HomeScreen->setWindowTitle(QCoreApplication::translate("HomeScreen", "Form", nullptr));
        motorScreenButton->setText(QCoreApplication::translate("HomeScreen", "Motor\n"
"Screen", nullptr));
        estopLabel->setText(QCoreApplication::translate("HomeScreen", "Estop", nullptr));
        fisherLogo->setText(QString());
        stationIdLabel->setText(QCoreApplication::translate("HomeScreen", "Station_id", nullptr));
        modeButton->setText(QCoreApplication::translate("HomeScreen", "Mode: Auto/Manual", nullptr));
        pinButton->setText(QCoreApplication::translate("HomeScreen", "Pin: Up/Down", nullptr));
        chargeButton->setText(QCoreApplication::translate("HomeScreen", "Start\n"
"Charging", nullptr));
        systemLabel->setText(QCoreApplication::translate("HomeScreen", "System Status:", nullptr));
        cameraLabel->setText(QCoreApplication::translate("HomeScreen", "Camera ", nullptr));
        hesaiLabel->setText(QCoreApplication::translate("HomeScreen", "Hesai Lidar", nullptr));
        sickLabel->setText(QCoreApplication::translate("HomeScreen", "Sick Lidar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class HomeScreen: public Ui_HomeScreen {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HOME_SCREEN_H
