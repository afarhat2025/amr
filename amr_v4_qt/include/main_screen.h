#ifndef MAIN_SCREEN_H
#define MAIN_SCREEN_H

#include <QWidget>
#include <ui_main_screen.h>

class MainScreen : public QWidget {
    Q_OBJECT


public:
    explicit MainScreen(QWidget *parent = nullptr);
    Ui::MainScreen ui;
    
private:
    
};

#endif // MAIN_SCREEN_H