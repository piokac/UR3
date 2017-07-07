#include "UR3Intermediator.h"
#include <QtGlobal>
#include <QtCore>
#include <QtEndian>
#include <QHostAddress>
#include <QDebug>
#include "UR3Message.h"

 void UR3Intermediator::TEST_QUEUE()
{
    QString cmd1 = "movej([-0.1, -1.26, 1.71, -1.02, -0.56, 0.19], a=1.0, v=0.1)\n";
    QString cmd2 = "movej([-0.2, -1.96, 1.21, -1.62, -1.86, 1.79], a=1.0, v=0.1)\n";
    QString cmd3 = "movej([-0.7, -1.06, 1.11, -1.62, -1.26, 1.19], a=1.0, v=0.1)\n";
    emit newCommand(cmd1);
    emit newCommand(cmd2);
    emit newCommand(cmd3);
 }

 void UR3Intermediator::TEST_POINTLIST()
 {
     auto t1 = QVector<double>({.0,-1.5708,.0,-1.5708,.0,.0});
     auto t2 = QVector<double>({1.0,-1.0,1.0,-1.5708,1.0,1.0});
     auto t3 = QVector<double>({.0,-1.5708,.0,-1.5708,.0,.0});

     QVector<QVector<double>> testList;
     testList.append(t1);
     testList.append(t2);
     testList.append(t3);

     this->OnPositionVector(testList);
 }

char *strdup (const char *s)
{
    char* d = (char*)malloc(strlen (s) + 1);   // Space for length plus nul
    if (d == NULL) return NULL;          // No memory
    strcpy (d,s);                        // Copy the characters
    return d;                            // Return the new string
}

static double bytesSwap(double v)
{
    union {
        uint64_t i;
        double d;
    } conv;
    conv.d = v;
    conv.i = _byteswap_uint64(conv.i);
    return conv.d;

}
static double RoundDouble(double val,int prec)
{
    auto precision = pow(10,prec);
    return round(val * precision) / precision;
}

void UR3Intermediator::MoveToPoint(QVector<double> q, double JointAcceleration, double JointSpeed)
{
    //TODO: nie dziala, zla logika

    CartesianInfoData CurrentCartesianInfo = this->ActualRobotInfo.getCartesianInfoData();
    double x = RoundDouble(CurrentCartesianInfo.getX(),4);
    double y = RoundDouble(CurrentCartesianInfo.getY(),4);
    double z = RoundDouble(CurrentCartesianInfo.getZ(),4);
    double rx = RoundDouble(CurrentCartesianInfo.getRx(),4);
    double ry = RoundDouble(CurrentCartesianInfo.getRy(),4);
    double rz = RoundDouble(CurrentCartesianInfo.getRz(),4);

    QVector<double> cord = QVector<double>({x+(q[0]/1000.0),y+(q[1]/1000.0),z+(q[2]/1000.0),rx,ry,rz});
    MoveL(cord);
}

void UR3Intermediator::MoveJ(QVector<double> JointPosition, double JointAcceleration, double JointSpeed)
{



    QString test = "movej([-0.1, -1.66, 1.71, -1.62, -1.56, 1.19], a=1.0, v=0.1)\n";

    QString command = "movej([" +
            QString::number(JointPosition[0]) + ", " +
            QString::number(JointPosition[1]) + ", " +
            QString::number(JointPosition[2]) + ", " +
            QString::number(JointPosition[3]) + ", " +
            QString::number(JointPosition[4]) + ", " +
            QString::number(JointPosition[5]) + "], " +
            "a=" + QString::number(JointAcceleration)+ ", " +
            "v=" + QString::number(JointSpeed)+ ")\n";

    //_running = true;
    emit newCommand(command);


}

void UR3Intermediator::SpeedJ(QVector<double> qd, double a, double t)
{
    QString command = "speedj([" +
            QString::number(qd[0]) + ", " +
            QString::number(qd[1]) + ", " +
            QString::number(qd[2]) + ", " +
            QString::number(qd[3]) + ", " +
            QString::number(qd[4]) + ", " +
            QString::number(qd[5]) + "], " +
            "a=" + QString::number(a)+ ", " +
            "t=" + QString::number(t)+ ")\n";

    emit newCommand(command);

}

void UR3Intermediator::SpeedL(QVector<double> qd, double a, double t)
{
    QString command = "speedl([" +
            QString::number(qd[0]) + ", " +
            QString::number(qd[1]) + ", " +
            QString::number(qd[2]) + ", " +
            QString::number(qd[3]) + ", " +
            QString::number(qd[4]) + ", " +
            QString::number(qd[5]) + "], " +
            "a=" + QString::number(a)+ ", " +
            "t=" + QString::number(t)+ ")\n";

    _running = true;
    emit newCommand(command);

}

void UR3Intermediator::SamuraiCut()
{
    MoveJ(QVector<double>({.0,-1.5708,.0,-1.5708,.0,.0}),2.0);
    MoveJ(QVector<double>({1.5962,0,.0,-1.5708,.0,.0}),2.0);
}

void UR3Intermediator::Home()
{
    MoveJ(QVector<double>({.0,-1.5708,.0,-1.5708,.0,.0}));
}


void UR3Intermediator::MoveL(QVector<double> TargetPose, double toolAcceleration, double toolSpeed, double time, double blendRadius)
{
    if(_connected && !_running)
    {

        QString command = "movel([" +
                QString::number(TargetPose[0]) + ", " +
                QString::number(TargetPose[1]) + ", " +
                QString::number(TargetPose[2]) + ", " +
                QString::number(TargetPose[3]) + ", " +
                QString::number(TargetPose[4]) + ", " +
                QString::number(TargetPose[5]) + "], " +
                "a=" + QString::number(toolAcceleration)+ ", " +
                "v=" + QString::number(toolSpeed)+ ", " +
                "t=" + QString::number(time)+", "+
                "r=" + QString::number(blendRadius)+
                ")\n";

        _running = true;
        emit newCommand(command);

    }

}

UR3Intermediator::UR3Intermediator():_connected(false), _running(false),Port(30002),IpAddress("192.168.149.128")
{
    this->_socket = new QTcpSocket();
    connect(this->_socket,SIGNAL(readyRead()),this,SLOT(OnSocketNewBytesWritten()));
    connect(this->_socket,SIGNAL(disconnected()),this,SLOT(disconnected()));

}

UR3Intermediator::UR3Intermediator(QString ipAddress, int port):_connected(false), _running(false),Port(port),IpAddress(ipAddress)
{
    this->_socket = new QTcpSocket();
     _timer = new QTimer();

    connect(this->_timer,SIGNAL(timeout()),this,SLOT(onTimerEvent()));
    connect(this->_socket,SIGNAL(readyRead()),this,SLOT(OnSocketNewBytesWritten()));
    connect(this->_socket,SIGNAL(disconnected()),this,SLOT(disconnected()));
    connect(this,SIGNAL(newCommand(QString)),this,SLOT(OnNewCommand(QString)));

    _timer->start(100);
}

int UR3Intermediator::getPort() const
{
    return Port;
}

void UR3Intermediator::setPort(int value)
{
    Port = value;
}

QString UR3Intermediator::getIpAddress() const
{
    return IpAddress;
}

void UR3Intermediator::setIpAddress(const QString &value)
{
    IpAddress = value;
}

void UR3Intermediator::GetRobotData()
{
    int size = 0;
    unsigned int offset = 0;
    if(mutex.tryLock())
    {
        _data = _DataFlow.data();
        memcpy(&size, &_data[offset], sizeof(size));
        size = _byteswap_ulong(size);
        if(_DataFlow.size()<size)
        {
            mutex.unlock();
            return;
        }

        offset += sizeof(size);
        while(offset<size)
        {
            unsigned char Type;
            memcpy(&Type,&_data[offset],sizeof(Type));
            offset+=sizeof(Type);
            int messageType = Type;
            switch(messageType)
            {
            case ROBOT_MESSAGE:
            {

                break;
            }
            case ROBOT_STATE:
            {
                //qDebug()<<"ROBOT_STATE";
                GetRobotMessage(_data, offset, size);
                break;
            }
            case PROGRAM_STATE_MESSAGE:
            {
                break;
            }

            default:
                offset = size;
                break;
            }

        }
        _DataFlow = _DataFlow.mid(size);
        mutex.unlock();
        //TEST();
    }

}
bool UR3Intermediator::CheckIfRunning()
{
    return ActualRobotInfo.robotModeData.getIsProgramRunning();

}


void UR3Intermediator::CheckJointsPosChanged()
{
    QVector<JointData> jointsData = this->ActualRobotInfo.getJointsData();
    double firstJointPos = RoundDouble(jointsData[0].getActualJointPosition(),4);
    double secondJointPos = RoundDouble(jointsData[1].getActualJointPosition(),4);
    double thirdJointPos = RoundDouble(jointsData[2].getActualJointPosition(),4);
    double fourthJointPos = RoundDouble(jointsData[3].getActualJointPosition(),4);
    double fifthJointPos = RoundDouble(jointsData[4].getActualJointPosition(),4);
    double sixthJointPos = RoundDouble(jointsData[5].getActualJointPosition(),4);
    QVector<double> current = QVector<double>
            ({firstJointPos,secondJointPos,thirdJointPos,fourthJointPos,fifthJointPos,sixthJointPos});

    emit newPoseTCP(current, 'p');

}


void UR3Intermediator::CheckPolozenieChanged()
{
    CartesianInfoData CurrentCartesianInfo = this->ActualRobotInfo.getCartesianInfoData();

    double x = RoundDouble(CurrentCartesianInfo.getX(),4);
    double y = RoundDouble(CurrentCartesianInfo.getY(),4);
    double z = RoundDouble(CurrentCartesianInfo.getZ(),4);
    double rx = RoundDouble(CurrentCartesianInfo.getRx(),4);
    double ry = RoundDouble(CurrentCartesianInfo.getRy(),4);
    double rz = RoundDouble(CurrentCartesianInfo.getRz(),4);

    QVector<double> current = QVector<double>({x,y,z,rx,ry,rz});
    emit newPoseTCP(current, 't');
}


void UR3Intermediator::GetRobotMessage(char *data, unsigned int &offset, int size)
{
    while(size>offset){
        int sizeOfPackage;
        memcpy(&sizeOfPackage, &data[offset], sizeof(sizeOfPackage));
        sizeOfPackage = _byteswap_ulong(sizeOfPackage);
        //sizeOfPackage = qFromBigEndian<int>(sizeOfPackage);
        offset+=sizeof(sizeOfPackage);

        unsigned char packageType;
        memcpy(&packageType, &data[offset], sizeof(packageType));
        offset+=sizeof(packageType);
        switch(packageType){
        case ROBOT_MODE_DATA:
            this->ActualRobotInfo.setRobotModeData(_data, offset);
            /*if(ActualRobotInfo.robotModeData.getIsEmergencyStopped() || ActualRobotInfo.robotModeData.getIsProtectiveStopped())
            {
                _socket->write(QString("end_force_mode()\n").toLatin1().data());
                _socket->waitForBytesWritten();
            }
            */

            //CheckIfRunning();
            break;
        case JOINT_DATA:
            this->ActualRobotInfo.setJointsData(_data, offset);
            CheckJointsPosChanged();
            break;
        case TOOL_DATA:
            this->ActualRobotInfo.setToolData(_data,offset);
            break;
        case MASTERBOARD_DATA:
            this->ActualRobotInfo.setMasterboardData(_data, offset);
            break;
        case CARTESIAN_INFO:
            this->ActualRobotInfo.setCartesianInfoData(_data,offset);
            CheckPolozenieChanged();
            break;
        case KINEMATICS_INFO:
            break;
        case CONFIGURATION_DATA:
            break;
        case FORCE_MODE_DATA:
            break;
        case ADDITIONAL_INFO:
            break;
        case CALIBRATION_DATA:
            break;
        case SAFETY_DATA:
            break;
        }

        offset +=sizeOfPackage - 5; //-5 poniewaz wczesniej przesunalem o sizeofpackage i packagetype

    }

}

void UR3Intermediator::ReadDataFlow()
{
    if(_connected)
    {
        if(mutex.tryLock())
        {
            _DataFlow.push_back( this->_socket->readAll());

            mutex.unlock();
        }

    }
}

void UR3Intermediator::onTimerEvent()
{

    if(!CheckIfRunning() && _commandsQueue.length() > 0)
    {
        QString cmd = _commandsQueue.dequeue();
        //_running = true;
        _socket->write(cmd.toLatin1().data());
        _socket->waitForBytesWritten();

        for(int i = 0; i<6; i++){
            qDebug()<<this->ActualRobotInfo.jointsData[i].getActualJointPosition();
        }
        qDebug()<<endl;
    }

}

bool UR3Intermediator::ConnectToRobot()
{

    if (_connected == false)
    {
        _socket->connectToHost(IpAddress,Port);
        if(_socket->waitForConnected())
        {
            _connected = true;
        }
        else
        {
            _connected = false;
        }
    }
    emit ConnectionAction(this->IpAddress.toLatin1().data(),_connected);
    return _connected;


}

void UR3Intermediator::disconnected()
{
    _connected = false;
    _socket->deleteLater();
    _socket = new QTcpSocket();
    ConnectToRobot();
}

void UR3Intermediator::OnPositionVector(QVector<QVector<double>> listOfPosition )
{
    for (auto ptr = listOfPosition.begin() ; ptr != listOfPosition.end(); ++ptr)
    {
        if(ptr->size() != 6 )
        {
            return;
        }

        this->MoveJ(*ptr);
    }

}

void UR3Intermediator::OnNewCommand(QString cmd)
{
    _commandsQueue.enqueue(cmd);
}


void UR3Intermediator::OnTcpChanged()
{

}

void UR3Intermediator::OnSocketNewBytesWritten()
{
    this->ReadDataFlow();
    GetRobotData();
}
