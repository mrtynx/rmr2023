#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include "robot.h"


//BERKI MARTIN 98310

#include "odometry.h"
#include "control_system.h"
#include <fstream>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
//    ipaddress = "192.168.1.11";
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;




    datacounter=0;


}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    static int EncoderRightPrev = robotdata.EncoderRight;
    static int EncoderLeftPrev = robotdata.EncoderLeft;
    static int EncoderRightDiff = 0;
    static int EncoderLeftDiff = 0;
    static int setpointCounter = 0;

    static double coords[3] = {0.0, 0.0, 0.0};
    static double deadband_angle = 0.15;

    static double setpoint[2];

    if(datacounter == 0)
    {
        setpoint[0] = setpoint_arr[0][0];
        setpoint[1] = setpoint_arr[0][1];
        setpointCounter++;
    }

    EncoderLeftDiff =  Odometry::normalizeDiff(int(robotdata.EncoderLeft - EncoderLeftPrev));
    EncoderRightDiff =  Odometry::normalizeDiff(int(robotdata.EncoderRight - EncoderRightPrev));

    EncoderLeftPrev = robotdata.EncoderLeft;
    EncoderRightPrev = robotdata.EncoderRight;

    Odometry::curveLocalization(EncoderLeftDiff, EncoderRightDiff, coords);


    if(!Control::robotReachedTarget(setpoint, coords, 0.8))
    {
        if(abs(Control::getAngleError(setpoint, coords)) > deadband_angle) Control::setRobotAngle(setpoint, coords, &robot);

        else Control::setRobotPosition(setpoint, coords, &robot);
    }

    else if(setpointCounter < sizeof(setpoint_arr)/sizeof(double)/2)
    {
        setpoint[0] = setpoint_arr[setpointCounter][0];
        setpoint[1] = setpoint_arr[setpointCounter][1];
        setpointCounter++;
    }

    else
    {
        robot.setTranslationSpeed(0);
    }



//    Debug
//    std::cout<<Control::getAngleError(setpoint, coords)<<std::endl;
//    std::cout<<Odometry::rad2deg(angle_err)<<std::endl;
//    std::cout<<"Reached dist: "<<Control::robotReachedTarget(setpoint, coords, 0.5)<<std::endl;
//    std::cout<<"Setpoint: "<<setpoint[0]<<" , "<<setpoint[1]<<" Setpoint counter: "<<setpointCounter<<std::endl;
//    std::cout<<"Left: "<<EncoderLeftDiff<<" Right: "<<EncoderRightDiff<<endl;
//    std::cout<<Control::setRobotAngle(-179,Odometry::rad2deg(coords[2]),&robot)<<std::endl;

    if(datacounter % 5)
    {
        emit uiValuesChanged(coords[0]*100, coords[1]*100, Odometry::rad2deg(coords[2]));
    }

    datacounter++;

    return 0;

}


///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    static int check = 0;

    if(check == 0)
    {
        std::ofstream file("C:\\Users\\Y740\\Desktop\\lidar_log\\lidar.csv", std::ios_base::app);
        check++;
        for(int i=0; i<= laserData.numberOfScans; i++)
        {
//            int sq = laserData.Data[i].scanQuality;
            double sa = laserData.Data[i].scanAngle;
            double sd = laserData.Data[i].scanDistance;
            file<<sa<<","<< sd<<"\n";
        }
        file.close();
    }
    else std::cout<<"Failed to open a file"<<"\n";

    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    //ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
robot.setRotationSpeed(-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);

}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}

void MainWindow::on_tableWidget_cellEntered(int row, int column)
{
    QString x = ui->tableWidget->item(0, 0)->text();
    double val = x.toDouble();
    std::cout<<val<<std::endl;
}


void MainWindow::on_setButton_clicked()
{

    for(int i=0; i < ui->tableWidget->rowCount(); i++)
    {
        for(int j=0; j < ui->tableWidget->columnCount(); j++)
        {
            QString text = ui->tableWidget->item(i,j)->text();
            bool is_ok;
            double val = text.toDouble(&is_ok);

            if(is_ok) setpoint_arr[i][j] = val;
            else std::cout<<"Problem reading setpoint"<<::std::endl;

        }
    }
}


void MainWindow::on_pushButton_10_clicked()
{
    for(int i=0; i < ui->tableWidget->rowCount(); i++)
    {
        for(int j=0; j < ui->tableWidget->columnCount(); j++)
        {
            setpoint_arr[i][j] = default_setpoint[i][j];
            ui->tableWidget->setItem(i,j, new QTableWidgetItem(QString::number(default_setpoint[i][j])));
        }
    }
}

