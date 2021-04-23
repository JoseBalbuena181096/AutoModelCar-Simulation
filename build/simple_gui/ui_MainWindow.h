/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGraphicsView *graphicsViewRobot;
    QPushButton *btnFwd;
    QSlider *vsSpeed;
    QPushButton *btnBwd;
    QSlider *hsSteering;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(577, 267);
        MainWindow->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 9px;\n"
"    margin-top: 0.5em;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 10px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
"\n"
"QRadioButton{\n"
"	spacing: 1px;\n"
"}\n"
"QRadioButton::indicator{\n"
"	width: 16px;\n"
"	height: 16px;\n"
"}"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        graphicsViewRobot = new QGraphicsView(centralWidget);
        graphicsViewRobot->setObjectName(QStringLiteral("graphicsViewRobot"));
        graphicsViewRobot->setGeometry(QRect(0, 0, 325, 245));
        btnFwd = new QPushButton(centralWidget);
        btnFwd->setObjectName(QStringLiteral("btnFwd"));
        btnFwd->setEnabled(true);
        btnFwd->setGeometry(QRect(400, 50, 50, 40));
        btnFwd->setIconSize(QSize(32, 32));
        btnFwd->setCheckable(false);
        vsSpeed = new QSlider(centralWidget);
        vsSpeed->setObjectName(QStringLiteral("vsSpeed"));
        vsSpeed->setGeometry(QRect(480, 50, 21, 131));
        vsSpeed->setMaximum(100);
        vsSpeed->setValue(25);
        vsSpeed->setOrientation(Qt::Vertical);
        btnBwd = new QPushButton(centralWidget);
        btnBwd->setObjectName(QStringLiteral("btnBwd"));
        btnBwd->setGeometry(QRect(400, 140, 50, 40));
        btnBwd->setIconSize(QSize(32, 32));
        hsSteering = new QSlider(centralWidget);
        hsSteering->setObjectName(QStringLiteral("hsSteering"));
        hsSteering->setGeometry(QRect(360, 10, 131, 20));
        hsSteering->setMinimum(-45);
        hsSteering->setMaximum(45);
        hsSteering->setOrientation(Qt::Horizontal);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(500, 40, 41, 17));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(500, 170, 41, 17));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(500, 100, 41, 17));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(495, 10, 31, 17));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(337, 10, 31, 17));
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "UPV-AutoModel GUI by Marcosoft", Q_NULLPTR));
        btnFwd->setText(QString());
        btnBwd->setText(QString());
        label->setText(QApplication::translate("MainWindow", "2", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "m/s", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "-45\302\260", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "45\302\260", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
