#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include "robot.h"


//BERKI MARTIN 98310

#include "odometry.h"
#include "control_system.h"
#include "mapping.h"
#include <fstream>
#include <vector>
#include <array>
#include <algorithm>

using namespace std;

static double coords[3] = {0.0, 0.0, 0.0};

std::vector<std::array<double, 2>> setpoint_vec;

bool manual_mode = false;
bool setpoint_mode = false;
bool mapping_mode = false;
bool map_now = false;
bool end_mapping = false;

double setpoint_ramp_pos[2] = {0.0, 0.0};
double setpoint_ramp_rot[2] = {0.0, 0.0};


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

    if(setpoint_mode || mapping_mode)
    {
        static int EncoderRightPrev = robotdata.EncoderRight;
        static int EncoderLeftPrev = robotdata.EncoderLeft;
        static int EncoderRightDiff = 0;
        static int EncoderLeftDiff = 0;
        static int setpointCounter = 0;

        static double deadband_angle = 0.08;

        static double setpoint[2];

        if(datacounter == 0)
        {
            setpoint[0] = setpoint_vec[0][0];
            setpoint[1] = setpoint_vec[0][1];
            setpointCounter++;
        }

        EncoderLeftDiff =  Odometry::normalizeDiff(int(robotdata.EncoderLeft - EncoderLeftPrev));
        EncoderRightDiff =  Odometry::normalizeDiff(int(robotdata.EncoderRight - EncoderRightPrev));

        EncoderLeftPrev = robotdata.EncoderLeft;
        EncoderRightPrev = robotdata.EncoderRight;

        Odometry::curveLocalization(EncoderLeftDiff, EncoderRightDiff, coords);

        double angle_err = Control::getAngleError(setpoint, coords);
        angle_err = Control::normalizeAngleError(angle_err);

        if(!Control::robotReachedTarget(setpoint, coords, 1))
        {           


            if(abs(angle_err) > deadband_angle)
            {
                Control::setpointRamp(setpoint_ramp_rot, setpoint, 0.1);
                Control::setRobotAngle(setpoint_ramp_rot, coords, &robot);
            }


            else
            {
                Control::setpointRamp(setpoint_ramp_pos, setpoint, 0.1);
                Control::setRobotPosition(setpoint_ramp_pos, coords, &robot);
            }

        }
        else if(setpointCounter < setpoint_vec.size())
        {
            if(mapping_mode)
            {
                double robot_angle = Odometry::rad2deg(coords[2]);

                if(!((robot_angle >= 89.5) && (robot_angle <= 90.5)))
                {
                        Control::setRobotMappingAngle(&robot, M_PI/2-coords[2]);
                }
                else
                {
                    map_now = true;
                    while(map_now)
                    {
                        ;;
                    }


                    setpoint_ramp_pos[0] = 0;
                    setpoint_ramp_pos[1] = 0;
                    setpoint_ramp_rot[0] = 0;
                    setpoint_ramp_rot[1] = 0;
                    setpoint[0] = setpoint_vec[setpointCounter][0];
                    setpoint[1] = setpoint_vec[setpointCounter][1];
                    setpointCounter++;
                }

            }


        }

        else
        {

            if(mapping_mode && !end_mapping)
            {
                map_now = true;
                while(map_now)
                {
                    ;;
                }
                end_mapping = true;
            }
            robot.setTranslationSpeed(0);

        }


        //    Debug
//            std::cout<<angle_err<<",   "<<std::endl;
//            std::cout<<Odometry::rad2deg(angle_err1)<<std::endl;
        //    std::cout<<"Reached dist: "<<Control::robotReachedTarget(setpoint, coords, 0.5)<<std::endl;
//            std::cout<<"Setpoint: "<<setpoint[0]<<" , "<<setpoint[1]<<" Setpoint counter: "<<setpointCounter<<std::endl;
//            std::cout<<"Left: "<<EncoderLeftDiff<<" Right: "<<EncoderRightDiff<<endl;
    }






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

    if(map_now)
    {
        robot.setTranslationSpeed(0);
        Mapping::mapAreaToFile(&laserData, coords, "C:\\Users\\Y740\\Desktop\\lidar_log\\data_lidar.csv");
        map_now = false;
    }

    if(end_mapping)
    {

    }

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
    if(manual_mode) robot.setTranslationSpeed(500);

}

void MainWindow::on_pushButton_3_clicked() //back
{
    if(manual_mode) robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
    if(manual_mode) robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked()//right
{
    if(manual_mode) robot.setRotationSpeed(-3.14159/2);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    if(manual_mode) robot.setTranslationSpeed(0);

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
    setpoint_mode = true;

    for(int i=0; i < ui->tableWidget->rowCount(); i++)
    {

        QString x_str = ui->tableWidget->item(i,0)->text();
        QString y_str = ui->tableWidget->item(i,1)->text();
        setpoint_vec.push_back({MainWindow::Qstr2d(x_str), MainWindow::Qstr2d(y_str)});
    }
}



void MainWindow::on_mappingButton_clicked()
{
    mapping_mode = true;


    setpoint_vec.push_back({0.0, 0.0});
    setpoint_vec.push_back({0.0, 320.0});
    setpoint_vec.push_back({95.0, 320.0});
    setpoint_vec.push_back({95.0, 160.0});
    setpoint_vec.push_back({300.0, 160.0});
    setpoint_vec.push_back({300.0, 35.0});
    setpoint_vec.push_back({475.0, 35.0});
    setpoint_vec.push_back({475.0, 160.0});
    setpoint_vec.push_back({475.0, 35.0});
    setpoint_vec.push_back({260.0, 35.0});
    setpoint_vec.push_back({260.0, 360.0});
    setpoint_vec.push_back({430.0, 360.0});
}


void MainWindow::on_manualModeButton_clicked()
{
    manual_mode = true;
}


double MainWindow::Qstr2d(QString text)
{
    bool is_ok;
    double val = text.toDouble(&is_ok);
    if(is_ok) return val;
    else
    {
        std::cout<<"Problem type casting QString value"<<std::endl;
        return -1;
    }
}

void MainWindow::on_showGridButton_clicked()
{
    Mapping::mapAreaToGrid("C:\\Users\\Y740\\Desktop\\lidar_log\\data_lidar.csv");
}


void MainWindow::on_floodFillButton_clicked()
{
    vector<vector<int>> grid = Mapping::mapAreaToGrid("C:\\Users\\Y740\\Desktop\\lidar_log\\data_lidar.csv");
    pair<int, int> start = {6, 41};
    pair<int, int> target = {38, 38};
    vector<vector<int>> path = Mapping::findPath(grid, start, target);
    if (!path.empty()) {
        cout << "Path found:\n";
        for (const auto &p : path) {
            cout << "(" << p[0] << ", " << p[1] << ")\n";
            grid[p[1]][p[0]] = 1;

        }
        Mapping::gridToFile(grid, "C:\\Users\\Y740\\Desktop\\lidar_log\\grid.csv");
        Mapping::print_grid(grid);
    } else {
        cout << "No path found.\n";
    }
}

