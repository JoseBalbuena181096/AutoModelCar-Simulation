#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    scCamera = new QGraphicsScene(0,0,320,240,ui->graphicsViewRobot);
    giCamera = scCamera->addPixmap(pmCamera);

    QIcon icoFwd(":/images/btnUp"  );
    QIcon icoBwd(":/images/btnDown");
    
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);

    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this,  SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this,  SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->hsSteering, SIGNAL(valueChanged(int)), this, SLOT(hsSteeringValueChanged(int)));
}

MainWindow::~MainWindow()
{    
    delete ui;
}

void MainWindow::setRosNode(QtRosNode* qtRosNode)
{
    this->qtRosNode = qtRosNode;
    //Connect signals from QtRosNode to MainWindow
    //For example, when ros finishes or when a rostopic is received
    QObject::connect(qtRosNode, SIGNAL(onRosNodeFinished()), this, SLOT(close()));
    QObject::connect(qtRosNode, SIGNAL(updateGraphics()), this, SLOT(updateGraphicsReceived()));
}

//
//SLOTS FOR SIGNALS EMITTED IN THE MAINWINDOW
//
void MainWindow::closeEvent(QCloseEvent *event)
{
    this->qtRosNode->gui_closed = true;
    this->qtRosNode->wait();
    //event->accept();
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    QImage img = QImage::fromData(qtRosNode->msgCompressedImage.data.data(), qtRosNode->msgCompressedImage.data.size());
    if(img.isNull())
    {
        std::cout << "NULL image" << std::endl;
        return;
    }
    //std::cout << "Image: " << img.width() << "x" << img.height() << std::endl;
    //pmCamera.fromImage(img);
    //giCamera->setPixmap(pmCamera);
}

void MainWindow::btnFwdPressed()
{
    qtRosNode->speed    = (ui->vsSpeed->value() / 50.0); //Max speed is 2 m/s
}

void MainWindow::btnFwdReleased()
{
    qtRosNode->speed    = 0;
}

void MainWindow::btnBwdPressed()
{
    qtRosNode->speed    = -(ui->vsSpeed->value() / 50.0); //Max speed is 2 m/s
}

void MainWindow::btnBwdReleased()
{
    qtRosNode->speed    = 0;
}

void MainWindow::hsSteeringValueChanged(int value)
{
    float steering = -(ui->hsSteering->value()/180.0*M_PI); //Steering is in [-PI/6, PI/6] rad
    qtRosNode->publishSteering(steering);
}
