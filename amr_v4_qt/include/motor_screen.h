#ifndef MOTORS_H
#define MOTORS_H

#include <QWidget>
#include <QLabel>
#include <ui_motor_screen.h>

class MotorScreen : public QWidget {
    Q_OBJECT


public:
    explicit MotorScreen(QWidget *parent = nullptr);
    Ui::MotorScreen ui;
    
private:
    
};

#endif // MOTORS_H