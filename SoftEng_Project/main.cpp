
#include <QApplication>
#include <QMainWindow>
#include <iostream>
#include "mainwindow.h"

int main(int argc, char *argv[]){
    QApplication a (argc, argv);
    MainWindow gui;
    gui.show();
    return a.exec();

}
