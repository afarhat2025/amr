/********************************************************************************
** Form generated from reading UI file 'main_screen.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_SCREEN_H
#define UI_MAIN_SCREEN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QWidget>
#include "scrolling_label.h"

QT_BEGIN_NAMESPACE

class Ui_MainScreen
{
public:
    QStackedWidget *mainContent;
    QWidget *homePage;
    QWidget *motorPage;
    ScrollingLabel *scrollingLabel;
    QPushButton *exitButton;
    QPushButton *minimizeButton;
    QLabel *wifiLabel;
    QLabel *percentageLabel;
    QLabel *currentLabel;
    QLabel *voltageLabel;
    QLabel *tempLabel;

    void setupUi(QWidget *MainScreen)
    {
        if (MainScreen->objectName().isEmpty())
            MainScreen->setObjectName(QString::fromUtf8("MainScreen"));
        MainScreen->resize(1280, 800);
        MainScreen->setMinimumSize(QSize(200, 0));
        mainContent = new QStackedWidget(MainScreen);
        mainContent->setObjectName(QString::fromUtf8("mainContent"));
        mainContent->setGeometry(QRect(0, 210, 1280, 580));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mainContent->sizePolicy().hasHeightForWidth());
        mainContent->setSizePolicy(sizePolicy);
        mainContent->setMinimumSize(QSize(1, 1));
        mainContent->setFrameShape(QFrame::NoFrame);
        mainContent->setLineWidth(2);
        homePage = new QWidget();
        homePage->setObjectName(QString::fromUtf8("homePage"));
        mainContent->addWidget(homePage);
        motorPage = new QWidget();
        motorPage->setObjectName(QString::fromUtf8("motorPage"));
        mainContent->addWidget(motorPage);
        scrollingLabel = new ScrollingLabel(MainScreen);
        scrollingLabel->setObjectName(QString::fromUtf8("scrollingLabel"));
        scrollingLabel->setGeometry(QRect(10, 100, 1231, 100));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(255);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(scrollingLabel->sizePolicy().hasHeightForWidth());
        scrollingLabel->setSizePolicy(sizePolicy1);
        QFont font;
        font.setPointSize(50);
        scrollingLabel->setFont(font);
        scrollingLabel->setFrameShape(QFrame::Box);
        scrollingLabel->setTextFormat(Qt::PlainText);
        scrollingLabel->setScaledContents(false);
        scrollingLabel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        scrollingLabel->setWordWrap(true);
        scrollingLabel->setIndent(0);
        exitButton = new QPushButton(MainScreen);
        exitButton->setObjectName(QString::fromUtf8("exitButton"));
        exitButton->setGeometry(QRect(1200, 10, 61, 51));
        QFont font1;
        font1.setPointSize(25);
        exitButton->setFont(font1);
        exitButton->setAutoFillBackground(true);
        minimizeButton = new QPushButton(MainScreen);
        minimizeButton->setObjectName(QString::fromUtf8("minimizeButton"));
        minimizeButton->setEnabled(true);
        minimizeButton->setGeometry(QRect(1130, 10, 61, 51));
        minimizeButton->setFont(font1);
        wifiLabel = new QLabel(MainScreen);
        wifiLabel->setObjectName(QString::fromUtf8("wifiLabel"));
        wifiLabel->setGeometry(QRect(0, 20, 181, 71));
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(wifiLabel->sizePolicy().hasHeightForWidth());
        wifiLabel->setSizePolicy(sizePolicy2);
        wifiLabel->setMaximumSize(QSize(16777215, 16777215));
        QFont font2;
        font2.setPointSize(36);
        wifiLabel->setFont(font2);
        wifiLabel->setFrameShape(QFrame::NoFrame);
        wifiLabel->setAlignment(Qt::AlignCenter);
        percentageLabel = new QLabel(MainScreen);
        percentageLabel->setObjectName(QString::fromUtf8("percentageLabel"));
        percentageLabel->setGeometry(QRect(170, 20, 141, 71));
        percentageLabel->setFont(font2);
        percentageLabel->setAlignment(Qt::AlignCenter);
        currentLabel = new QLabel(MainScreen);
        currentLabel->setObjectName(QString::fromUtf8("currentLabel"));
        currentLabel->setGeometry(QRect(320, 20, 171, 71));
        currentLabel->setFont(font2);
        currentLabel->setAlignment(Qt::AlignCenter);
        voltageLabel = new QLabel(MainScreen);
        voltageLabel->setObjectName(QString::fromUtf8("voltageLabel"));
        voltageLabel->setGeometry(QRect(500, 20, 181, 71));
        voltageLabel->setFont(font2);
        voltageLabel->setAlignment(Qt::AlignCenter);
        tempLabel = new QLabel(MainScreen);
        tempLabel->setObjectName(QString::fromUtf8("tempLabel"));
        tempLabel->setGeometry(QRect(700, 20, 171, 71));
        tempLabel->setFont(font2);
        tempLabel->setAlignment(Qt::AlignCenter);

        retranslateUi(MainScreen);

        QMetaObject::connectSlotsByName(MainScreen);
    } // setupUi

    void retranslateUi(QWidget *MainScreen)
    {
        MainScreen->setWindowTitle(QCoreApplication::translate("MainScreen", "Form", nullptr));
        scrollingLabel->setText(QCoreApplication::translate("MainScreen", "Robot Status", nullptr));
        exitButton->setText(QCoreApplication::translate("MainScreen", "X", nullptr));
        minimizeButton->setText(QCoreApplication::translate("MainScreen", "-", nullptr));
        wifiLabel->setText(QCoreApplication::translate("MainScreen", "Wifi", nullptr));
        percentageLabel->setText(QCoreApplication::translate("MainScreen", "100%", nullptr));
        currentLabel->setText(QCoreApplication::translate("MainScreen", "100A", nullptr));
        voltageLabel->setText(QCoreApplication::translate("MainScreen", "100V", nullptr));
        tempLabel->setText(QCoreApplication::translate("MainScreen", "100 \302\260C", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainScreen: public Ui_MainScreen {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_SCREEN_H
