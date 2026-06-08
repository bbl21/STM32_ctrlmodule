#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("RF Frontend Controller");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("CtrlModule");

    MainWindow w;
    w.show();

    return app.exec();
}
