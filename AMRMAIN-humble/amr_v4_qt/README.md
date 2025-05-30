# AGV GUI

AGV GUI is a gui application designed to display diagnostics in our new AGV systems. This Will allow easier maintenece and upkeep with the robots.

# Project Structure

The GUI Folder belongs under your ~/amr_v4_cont/src folder
```bash
amr_v4_qt/
│
├── include/             # Header files
│
├── src/                 # Source and UI files
│   ├── source files     # Actual source code files
│   └── ui files         # User interface files
│
├── package.xml          # Package configuration
└── CMakeLists.txt       # CMake build configuration
```

# Installs
```bash
sudo apt install qt5-default qtbase5-dev qtcreator
```

# Creating the UI

Create a QWidget in QT Designer and add items necessary for your project. You can name the objects in the inspector in the right side of the screen as well as adjust any properties. Then once you're all done go to file and save the file under you projects src directory.
Your next step will be to convert the UI file into a header file and move it to your include folder using the following command...
```bash
cd arm_v4_qt/src
uic -o ~amr_v4_cont/src/amr_v4_qt/include/ui_main_screen.h main_screen.ui
```
# Coding the UI  
In order to alter UI elements in the code you need to setup the UI file as a class in c++. You do this with the following header and cpp fles (example):

main_screen.h
```C++
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
```


main_screen.cpp
```CPP
#include "main_screen.h"


MainScreen::MainScreen(QWidget* parent) : QWidget(parent) {

    ui.setupUi(this);
    
    
}
```

Now in your code you can access elements of the UI using the following syntax:

```C++
mainScreen = new MainScreen();
mainScreen->ui.percentageLabel->setText(QString("%1").arg(percentage) + "%");
mainScreen->ui.voltageLabel->setText(QString("%1").arg(voltage) + "V");
mainScreen->ui.currentLabel->setText(QString("%1").arg(current) + "A");
```


