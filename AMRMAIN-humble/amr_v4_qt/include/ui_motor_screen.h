/********************************************************************************
** Form generated from reading UI file 'motor_screen.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MOTOR_SCREEN_H
#define UI_MOTOR_SCREEN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MotorScreen
{
public:
    QLabel *fisherLogo;
    QFrame *leftMotorFrame;
    QLabel *leftMotorLabel;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_12;
    QLabel *pinMotorCurrentLabel_3;
    QLabel *leftMotorCurrent;
    QHBoxLayout *horizontalLayout_13;
    QLabel *pinMotorErrorLabel_3;
    QLabel *leftMotorError;
    QFrame *rightMotorFrame;
    QLabel *rightMotorLabel;
    QWidget *verticalLayoutWidget_7;
    QVBoxLayout *verticalLayout_7;
    QHBoxLayout *horizontalLayout_14;
    QLabel *pinMotorCurrentLabel_4;
    QLabel *rightMotorCurrent;
    QHBoxLayout *horizontalLayout_15;
    QLabel *pinMotorErrorLabel_4;
    QLabel *rightMotorError;
    QFrame *pinFrame;
    QLabel *pinLabel;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_8;
    QLabel *pinMotorCurrentLabel;
    QLabel *pinMotorCurrent;
    QHBoxLayout *horizontalLayout_9;
    QLabel *pinMotorErrorLabel;
    QLabel *pinMotorError;
    QLabel *estopLabel;
    QPushButton *homeScreenButton;
    QLabel *rightArrow;
    QLabel *leftArrow;

    void setupUi(QWidget *MotorScreen)
    {
        if (MotorScreen->objectName().isEmpty())
            MotorScreen->setObjectName(QString::fromUtf8("MotorScreen"));
        MotorScreen->resize(1280, 580);
        fisherLogo = new QLabel(MotorScreen);
        fisherLogo->setObjectName(QString::fromUtf8("fisherLogo"));
        fisherLogo->setGeometry(QRect(800, 470, 420, 100));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(fisherLogo->sizePolicy().hasHeightForWidth());
        fisherLogo->setSizePolicy(sizePolicy);
        fisherLogo->setFrameShape(QFrame::NoFrame);
        fisherLogo->setPixmap(QPixmap(QString::fromUtf8("../../../../.designer/assets/fisher_logo.png")));
        fisherLogo->setScaledContents(true);
        fisherLogo->setAlignment(Qt::AlignCenter);
        leftMotorFrame = new QFrame(MotorScreen);
        leftMotorFrame->setObjectName(QString::fromUtf8("leftMotorFrame"));
        leftMotorFrame->setGeometry(QRect(10, 10, 440, 220));
        leftMotorFrame->setFrameShape(QFrame::StyledPanel);
        leftMotorFrame->setFrameShadow(QFrame::Raised);
        leftMotorLabel = new QLabel(leftMotorFrame);
        leftMotorLabel->setObjectName(QString::fromUtf8("leftMotorLabel"));
        leftMotorLabel->setGeometry(QRect(9, 9, 421, 41));
        QFont font;
        font.setPointSize(35);
        leftMotorLabel->setFont(font);
        leftMotorLabel->setAlignment(Qt::AlignCenter);
        verticalLayoutWidget_6 = new QWidget(leftMotorFrame);
        verticalLayoutWidget_6->setObjectName(QString::fromUtf8("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(10, 60, 421, 151));
        verticalLayout_6 = new QVBoxLayout(verticalLayoutWidget_6);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        pinMotorCurrentLabel_3 = new QLabel(verticalLayoutWidget_6);
        pinMotorCurrentLabel_3->setObjectName(QString::fromUtf8("pinMotorCurrentLabel_3"));
        QFont font1;
        font1.setPointSize(30);
        pinMotorCurrentLabel_3->setFont(font1);
        pinMotorCurrentLabel_3->setAlignment(Qt::AlignCenter);

        horizontalLayout_12->addWidget(pinMotorCurrentLabel_3);

        leftMotorCurrent = new QLabel(verticalLayoutWidget_6);
        leftMotorCurrent->setObjectName(QString::fromUtf8("leftMotorCurrent"));
        leftMotorCurrent->setFont(font1);
        leftMotorCurrent->setAlignment(Qt::AlignCenter);

        horizontalLayout_12->addWidget(leftMotorCurrent);


        verticalLayout_6->addLayout(horizontalLayout_12);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        pinMotorErrorLabel_3 = new QLabel(verticalLayoutWidget_6);
        pinMotorErrorLabel_3->setObjectName(QString::fromUtf8("pinMotorErrorLabel_3"));
        pinMotorErrorLabel_3->setFont(font1);
        pinMotorErrorLabel_3->setAlignment(Qt::AlignCenter);

        horizontalLayout_13->addWidget(pinMotorErrorLabel_3);

        leftMotorError = new QLabel(verticalLayoutWidget_6);
        leftMotorError->setObjectName(QString::fromUtf8("leftMotorError"));
        leftMotorError->setFont(font1);
        leftMotorError->setAlignment(Qt::AlignCenter);

        horizontalLayout_13->addWidget(leftMotorError);


        verticalLayout_6->addLayout(horizontalLayout_13);

        rightMotorFrame = new QFrame(MotorScreen);
        rightMotorFrame->setObjectName(QString::fromUtf8("rightMotorFrame"));
        rightMotorFrame->setGeometry(QRect(800, 11, 440, 220));
        rightMotorFrame->setFrameShape(QFrame::StyledPanel);
        rightMotorFrame->setFrameShadow(QFrame::Raised);
        rightMotorLabel = new QLabel(rightMotorFrame);
        rightMotorLabel->setObjectName(QString::fromUtf8("rightMotorLabel"));
        rightMotorLabel->setGeometry(QRect(9, 9, 421, 41));
        rightMotorLabel->setFont(font);
        rightMotorLabel->setAlignment(Qt::AlignCenter);
        verticalLayoutWidget_7 = new QWidget(rightMotorFrame);
        verticalLayoutWidget_7->setObjectName(QString::fromUtf8("verticalLayoutWidget_7"));
        verticalLayoutWidget_7->setGeometry(QRect(10, 60, 421, 151));
        verticalLayout_7 = new QVBoxLayout(verticalLayoutWidget_7);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_14 = new QHBoxLayout();
        horizontalLayout_14->setObjectName(QString::fromUtf8("horizontalLayout_14"));
        pinMotorCurrentLabel_4 = new QLabel(verticalLayoutWidget_7);
        pinMotorCurrentLabel_4->setObjectName(QString::fromUtf8("pinMotorCurrentLabel_4"));
        pinMotorCurrentLabel_4->setFont(font1);
        pinMotorCurrentLabel_4->setAlignment(Qt::AlignCenter);

        horizontalLayout_14->addWidget(pinMotorCurrentLabel_4);

        rightMotorCurrent = new QLabel(verticalLayoutWidget_7);
        rightMotorCurrent->setObjectName(QString::fromUtf8("rightMotorCurrent"));
        rightMotorCurrent->setFont(font1);
        rightMotorCurrent->setAlignment(Qt::AlignCenter);

        horizontalLayout_14->addWidget(rightMotorCurrent);


        verticalLayout_7->addLayout(horizontalLayout_14);

        horizontalLayout_15 = new QHBoxLayout();
        horizontalLayout_15->setObjectName(QString::fromUtf8("horizontalLayout_15"));
        pinMotorErrorLabel_4 = new QLabel(verticalLayoutWidget_7);
        pinMotorErrorLabel_4->setObjectName(QString::fromUtf8("pinMotorErrorLabel_4"));
        pinMotorErrorLabel_4->setFont(font1);
        pinMotorErrorLabel_4->setAlignment(Qt::AlignCenter);

        horizontalLayout_15->addWidget(pinMotorErrorLabel_4);

        rightMotorError = new QLabel(verticalLayoutWidget_7);
        rightMotorError->setObjectName(QString::fromUtf8("rightMotorError"));
        rightMotorError->setFont(font1);
        rightMotorError->setAlignment(Qt::AlignCenter);

        horizontalLayout_15->addWidget(rightMotorError);


        verticalLayout_7->addLayout(horizontalLayout_15);

        pinFrame = new QFrame(MotorScreen);
        pinFrame->setObjectName(QString::fromUtf8("pinFrame"));
        pinFrame->setGeometry(QRect(405, 240, 440, 220));
        pinFrame->setFrameShape(QFrame::StyledPanel);
        pinFrame->setFrameShadow(QFrame::Raised);
        pinLabel = new QLabel(pinFrame);
        pinLabel->setObjectName(QString::fromUtf8("pinLabel"));
        pinLabel->setGeometry(QRect(9, 9, 421, 41));
        pinLabel->setFont(font);
        pinLabel->setAlignment(Qt::AlignCenter);
        verticalLayoutWidget_4 = new QWidget(pinFrame);
        verticalLayoutWidget_4->setObjectName(QString::fromUtf8("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(10, 60, 421, 151));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        pinMotorCurrentLabel = new QLabel(verticalLayoutWidget_4);
        pinMotorCurrentLabel->setObjectName(QString::fromUtf8("pinMotorCurrentLabel"));
        pinMotorCurrentLabel->setFont(font1);
        pinMotorCurrentLabel->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(pinMotorCurrentLabel);

        pinMotorCurrent = new QLabel(verticalLayoutWidget_4);
        pinMotorCurrent->setObjectName(QString::fromUtf8("pinMotorCurrent"));
        pinMotorCurrent->setFont(font1);
        pinMotorCurrent->setAlignment(Qt::AlignCenter);

        horizontalLayout_8->addWidget(pinMotorCurrent);


        verticalLayout_4->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        pinMotorErrorLabel = new QLabel(verticalLayoutWidget_4);
        pinMotorErrorLabel->setObjectName(QString::fromUtf8("pinMotorErrorLabel"));
        pinMotorErrorLabel->setFont(font1);
        pinMotorErrorLabel->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(pinMotorErrorLabel);

        pinMotorError = new QLabel(verticalLayoutWidget_4);
        pinMotorError->setObjectName(QString::fromUtf8("pinMotorError"));
        pinMotorError->setFont(font1);
        pinMotorError->setAlignment(Qt::AlignCenter);

        horizontalLayout_9->addWidget(pinMotorError);


        verticalLayout_4->addLayout(horizontalLayout_9);

        estopLabel = new QLabel(MotorScreen);
        estopLabel->setObjectName(QString::fromUtf8("estopLabel"));
        estopLabel->setGeometry(QRect(410, 470, 220, 100));
        QFont font2;
        font2.setPointSize(50);
        estopLabel->setFont(font2);
        estopLabel->setFrameShape(QFrame::Box);
        estopLabel->setLineWidth(2);
        estopLabel->setAlignment(Qt::AlignCenter);
        homeScreenButton = new QPushButton(MotorScreen);
        homeScreenButton->setObjectName(QString::fromUtf8("homeScreenButton"));
        homeScreenButton->setGeometry(QRect(20, 460, 281, 100));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(homeScreenButton->sizePolicy().hasHeightForWidth());
        homeScreenButton->setSizePolicy(sizePolicy1);
        homeScreenButton->setMinimumSize(QSize(200, 50));
        homeScreenButton->setMaximumSize(QSize(300, 100));
        homeScreenButton->setFont(font1);
        rightArrow = new QLabel(MotorScreen);
        rightArrow->setObjectName(QString::fromUtf8("rightArrow"));
        rightArrow->setGeometry(QRect(910, 260, 161, 111));
        rightArrow->setPixmap(QPixmap(QString::fromUtf8("../assets/arrowL.png")));
        rightArrow->setScaledContents(true);
        leftArrow = new QLabel(MotorScreen);
        leftArrow->setObjectName(QString::fromUtf8("leftArrow"));
        leftArrow->setGeometry(QRect(190, 260, 161, 111));
        leftArrow->setPixmap(QPixmap(QString::fromUtf8("../assets/arrowL.png")));
        leftArrow->setScaledContents(true);

        retranslateUi(MotorScreen);

        QMetaObject::connectSlotsByName(MotorScreen);
    } // setupUi

    void retranslateUi(QWidget *MotorScreen)
    {
        MotorScreen->setWindowTitle(QCoreApplication::translate("MotorScreen", "Form", nullptr));
        fisherLogo->setText(QString());
        leftMotorLabel->setText(QCoreApplication::translate("MotorScreen", "Left Motor", nullptr));
        pinMotorCurrentLabel_3->setText(QCoreApplication::translate("MotorScreen", "Current:", nullptr));
        leftMotorCurrent->setText(QCoreApplication::translate("MotorScreen", "0", nullptr));
        pinMotorErrorLabel_3->setText(QCoreApplication::translate("MotorScreen", "Error:", nullptr));
        leftMotorError->setText(QCoreApplication::translate("MotorScreen", "None", nullptr));
        rightMotorLabel->setText(QCoreApplication::translate("MotorScreen", "Right Motor", nullptr));
        pinMotorCurrentLabel_4->setText(QCoreApplication::translate("MotorScreen", "Current:", nullptr));
        rightMotorCurrent->setText(QCoreApplication::translate("MotorScreen", "0", nullptr));
        pinMotorErrorLabel_4->setText(QCoreApplication::translate("MotorScreen", "Error:", nullptr));
        rightMotorError->setText(QCoreApplication::translate("MotorScreen", "None", nullptr));
        pinLabel->setText(QCoreApplication::translate("MotorScreen", "Pin Motor", nullptr));
        pinMotorCurrentLabel->setText(QCoreApplication::translate("MotorScreen", "Current:", nullptr));
        pinMotorCurrent->setText(QCoreApplication::translate("MotorScreen", "0", nullptr));
        pinMotorErrorLabel->setText(QCoreApplication::translate("MotorScreen", "Error:", nullptr));
        pinMotorError->setText(QCoreApplication::translate("MotorScreen", "None", nullptr));
        estopLabel->setText(QCoreApplication::translate("MotorScreen", "Estop", nullptr));
        homeScreenButton->setText(QCoreApplication::translate("MotorScreen", "Home Screen", nullptr));
        rightArrow->setText(QString());
        leftArrow->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MotorScreen: public Ui_MotorScreen {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MOTOR_SCREEN_H
