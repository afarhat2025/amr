#include "motor_screen.h"
#include <QUiLoader>
#include <QFile>
#include <QVBoxLayout>
#include <QDebug>
#include "ui_motor_screen.h"

MotorScreen::MotorScreen(QWidget* parent) : QWidget(parent) {

    ui.setupUi(this);
    
    
}