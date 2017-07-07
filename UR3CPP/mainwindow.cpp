#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent), settings(new Settings("settings.ini", this)),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->ur3 = new UR3Intermediator("192.168.146.128",30002);

    connect(this->ui->actionConnect,SIGNAL(triggered(bool)),this,SLOT(OnActionConnection()));
    //connect(this->ur3, SIGNAL(newJointPos(QVector<double>)),this,SLOT(OnNewJointPos(QVector<double>)));
    connect(this->ur3, SIGNAL(newPoseTCP(QVector<double>,char)),this, SLOT(OnNewTCP(QVector<double>,char)));
    connect(this->ur3,SIGNAL(ConnectionAction(char*,bool)),this,SLOT(ConnectedToInfo(char*,bool)));
    connect(this->ui->pushButton_Samurai,SIGNAL(clicked(bool)),this,SLOT(OnSamuraiCut()));
    connect(this->ui->pushButton_MoveJ,SIGNAL(clicked(bool)),this,SLOT(OnMoveJ()));
    connect(this->ui->pushButton_MoveL,SIGNAL(clicked(bool)),this,SLOT(OnMoveL()));
    connect(this->ui->pushButton_SpeedJ,SIGNAL(pressed()),this,SLOT(OnSpeedJ()));
    connect(this->ui->pushButton_Home,SIGNAL(clicked(bool)),this,SLOT(Home()));
    ur3->ConnectToRobot();
    connect(this->ui->actionSettings, SIGNAL(triggered(bool)), this, SLOT(showSettings()));

    settings->read(ur3);

}

MainWindow::~MainWindow()
{
    settings->serialize(ur3);
    delete settings;
    delete ui;
}

void MainWindow::showSettings()
{
    settings->moduleChanged(ur3);
    SettingsWindow *ustawieniaOkno = settings->getWindow(); //pobranie wskaznika do okna ustawien
    ustawieniaOkno->exec(); //wyswietlenie modalnego okna dialogowego
}

void MainWindow::OnConnectedDialogInfo(QString ip,int port)
{
    this->ur3->setIpAddress(ip);
    this->ur3->setPort(port);
    this->ur3->ConnectToRobot();
}

void MainWindow::OnMoveJ()
{

    this->ur3->MoveJ(QVector<double>(
    {0.6236825723301582, -1.477339167481995,
     2.478097719134525, -2.5575642827418985, -1.5571205043625342, 2.781621040141847}));
}

void MainWindow::OnSpeedJ()
{
    this->ur3->SpeedJ(QVector<double>({0,0,0,0,.1,.1}));
}

void MainWindow::OnMoveL()
{
}

void MainWindow::OnSamuraiCut()
{

    ur3->TEST_POINTLIST();
}

void MainWindow::OnActionConnection()
{
    this->ur3->ConnectToRobot();
//    this->connectDialog = new ConnectDialog();
//    connect(connectDialog,SIGNAL(ConnectClicked(QString,int)),this,SLOT(OnConnectedDialogInfo(QString,int)));
//    connectDialog->show();
}

void MainWindow::Home()
{
    this->ur3->Home();
}

void MainWindow::OnNewJointPos(QVector<double> pose)
{
    this->ui->lineEditBase->setText(QString::number(pose[0]));
    this->ui->lineEditShoulder->setText(QString::number(pose[1]));
    this->ui->lineEditElbow->setText(QString::number(pose[2]));
    this->ui->lineEditWrist1->setText(QString::number(pose[3]));
    this->ui->lineEditWrist2->setText(QString::number(pose[4]));
    this->ui->lineEditWrist3->setText(QString::number(pose[5]));
}

void MainWindow::OnNewTCP(QVector<double> data, char c)
{
    switch(c){
    case 'p':
        this->ui->lineEditBase->setText(QString::number(data[0]));
        this->ui->lineEditShoulder->setText(QString::number(data[1]));
        this->ui->lineEditElbow->setText(QString::number(data[2]));
        this->ui->lineEditWrist1->setText(QString::number(data[3]));
        this->ui->lineEditWrist2->setText(QString::number(data[4]));
        this->ui->lineEditWrist3->setText(QString::number(data[5]));
        break;
    case 't':
        this->ui->lineEditX->setText(QString::number(data[0]));
        this->ui->lineEditY->setText(QString::number(data[1]));
        this->ui->lineEditZ->setText(QString::number(data[2]));
        this->ui->lineEditRX->setText(QString::number(data[3]));
        this->ui->lineEditRY->setText(QString::number(data[4]));
        this->ui->lineEditRZ->setText(QString::number(data[5]));
        break;
    }



}

void MainWindow::ConnectedToInfo(char* Ip, bool Achieved)
{
    QString ip = QString(Ip);
    if (Achieved)
    {
        this->ui->lineEdit_Connection->setText("Connected with " + ip);
    }
    else
    {
        this->ui->lineEdit_Connection->setText("Connection to IP " + ip + " failed");
    }
}
