#ifndef HOME_SCREEN_H
#define HOME_SCREEN_H

#include <QWidget>
#include <ui_home_screen.h>

class HomeScreen : public QWidget {
    Q_OBJECT


public:
    explicit HomeScreen(QWidget *parent = nullptr);
    Ui::HomeScreen ui;
    
private:
    
};

#endif // HOME_SCREEN_H