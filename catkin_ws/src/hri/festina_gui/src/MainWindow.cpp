#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QtWidgets/QFileDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->recSavingVideo = false;
    this->facRecognizing = false;
    this->sktRecognizing = false;
    this->hriFollowing = false;
    this->hriFindingLegs = false;
    this->navDetectingObstacles = false;
    this->faceRecognition = false;

    QIcon icoFwd(":/images/btnUp");
    QIcon icoBwd(":/images/btnDown");
    QIcon icoLeft(":/images/btnLeft");
    QIcon icoRight(":/images/btnRight");
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);
    ui->btnLeft->setIcon(icoLeft);
    ui->btnRight->setIcon(icoRight);

    //Buttons for manual drive
    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this, SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->btnLeft, SIGNAL(pressed()), this, SLOT(btnLeftPressed()));
    QObject::connect(ui->btnLeft, SIGNAL(released()), this, SLOT(btnLeftReleased()));
    QObject::connect(ui->btnRight, SIGNAL(pressed()), this, SLOT(btnRightPressed()));
    QObject::connect(ui->btnRight, SIGNAL(released()), this, SLOT(btnRightReleased()));
    //Navigation
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnExecPath, SIGNAL(clicked()), this, SLOT(navBtnExecPath_pressed()));
    QObject::connect(ui->navTxtMove, SIGNAL(returnPressed()), this, SLOT(navMoveChanged()));
    QObject::connect(ui->navBtnStartObsDetection, SIGNAL(clicked()), this, SLOT(navObsDetectionEnableClicked()));
    //Hardware
    QObject::connect(ui->hdTxtPan, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    QObject::connect(ui->hdTxtTilt, SIGNAL(valueChanged(double)), this, SLOT(hdPanTiltChanged(double)));
    //Speech synthesis and recog
    QObject::connect(ui->spgTxtSay, SIGNAL(returnPressed()), this, SLOT(spgSayChanged()));
    QObject::connect(ui->sprTxtFakeRecog, SIGNAL(returnPressed()), this, SLOT(sprFakeRecognizedChanged()));
    //Vision
    QObject::connect(ui->recBtnSaveVideo, SIGNAL(clicked()), this, SLOT(recSaveVideoChanged()));
    QObject::connect(ui->recTxtImgFile, SIGNAL(returnPressed()), this, SLOT(recSaveImageChanged()));
    QObject::connect(ui->recBtnSaveImg, SIGNAL(clicked()), this, SLOT(recSaveImageChanged()));
    QObject::connect(ui->sktBtnStartRecog, SIGNAL(clicked()), this, SLOT(sktBtnStartClicked()));
    QObject::connect(ui->facBtnStartRecog, SIGNAL(clicked()), this, SLOT(facBtnStartClicked()));
    QObject::connect(ui->facTxtRecog, SIGNAL(returnPressed()), this, SLOT(facRecogPressed()));
    QObject::connect(ui->facTxtTrain, SIGNAL(returnPressed()), this, SLOT(facTrainPressed()));
    QObject::connect(ui->facTxtClear, SIGNAL(returnPressed()), this, SLOT(facClearPressed()));
    QObject::connect(ui->objTxtGoalObject, SIGNAL(returnPressed()), this, SLOT(objRecogObjectChanged()));
    QObject::connect(ui->vsnBtnFindLines, SIGNAL(clicked()), this, SLOT(vsnFindLinesClicked()));
    //HRI
    QObject::connect(ui->hriBtnStartFollow, SIGNAL(clicked()), this, SLOT(hriBtnFollowClicked()));
    QObject::connect(ui->hriBtnStartLegs, SIGNAL(clicked()), this, SLOT(hriBtnLegsClicked()));
    
    this->robotX = 0;
    this->robotY = 0;
    this->robotA = 0;
    /*QScrollArea *scrollArea = new QScrollArea;
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(this->ui->labelAnswerResp);*/
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

bool MainWindow::strToFloatArray(std::string str, std::vector<float>& result)
{
    result.clear();
    std::vector<std::string> parts;
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    for(size_t i=0; i < parts.size(); i++)
    {
        std::stringstream ssValue(parts[i]);
        float value;
        if(!(ssValue >> value))
            return false;
        result.push_back(value);
    }
    return true;
}

void MainWindow::stopRobot()
{
    JustinaHardware::stopRobot();
}

void MainWindow::btnFwdPressed()
{
    qtRosNode->linearSpeed = 0.3;
    qtRosNode->angularSpeed = 0.0;
}

void MainWindow::btnFwdReleased()
{
    qtRosNode->linearSpeed = 0.0;
    qtRosNode->angularSpeed = 0.0;
}

void MainWindow::btnBwdPressed()
{
    qtRosNode->linearSpeed = -0.3;
    qtRosNode->angularSpeed = 0.0;
}

void MainWindow::btnBwdReleased()
{
    qtRosNode->linearSpeed = 0.0;
    qtRosNode->angularSpeed = 0.0;
}

void MainWindow::btnLeftPressed()
{
    qtRosNode->linearSpeed = 0.0;
    qtRosNode->angularSpeed = 0.5;
}

void MainWindow::btnLeftReleased()
{
    qtRosNode->linearSpeed = 0.0;
    qtRosNode->angularSpeed = 0.0;
}

void MainWindow::btnRightPressed()
{
    qtRosNode->linearSpeed = 0.0;
    qtRosNode->angularSpeed = -0.5;
}

void MainWindow::btnRightReleased()
{
    qtRosNode->linearSpeed = 0.0;
    qtRosNode->angularSpeed = 0.0;
}


void MainWindow::navBtnCalcPath_pressed()
{
    float startX, startY, startA;
    float goalX = 0;
    float goalY = 0;
    float goalA;
    std::string start_location = "";
    std::string goal_location = "";
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    {
        this->ui->navTxtStartPose->setText("Robot");
        JustinaNavigation::getRobotPose(this->robotX, this->robotY, this->robotA);
        startX = this->robotX;
        startY = this->robotY;
        startA = this->robotA;
    }
    else if(parts.size() >= 2) //Given data correspond to numbers
    {
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else //Given data correspond to location
        start_location = parts[0];

    str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else
        goal_location = parts[0];

    int method = this->ui->navCbPlanningMethod->currentIndex();
    if(start_location.compare("") == 0 && goal_location.compare("") == 0)
        JustinaNavigation::planPath(startX, startY, goalX, goalY, this->calculatedPath, method);
    else if(start_location.compare("") == 0 && goal_location.compare("") != 0)
        JustinaNavigation::planPath(startX, startY, goal_location, this->calculatedPath, method);
    else if(start_location.compare("") != 0 && goal_location.compare("") == 0)
        JustinaNavigation::planPath(start_location, goalX, goalY, this->calculatedPath, method);
    else
        JustinaNavigation::planPath(start_location, goal_location, this->calculatedPath, method);
}

void MainWindow::navBtnExecPath_pressed()
{
    float goalX = 0;
    float goalY = 0;
    float goalA;
    std::string goal_location = "";
    std::vector<std::string> parts;

    std::string str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
        if(parts.size() > 2)
        {
            std::stringstream ssGoalAngle(parts[2]);
            if(!(ssGoalAngle >> goalA))
            {
                this->ui->navTxtStartPose->setText("Invalid format");
                return;
            }
            //this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
            JustinaNavigation::startGetClose(goalX, goalY, goalA);
        }
        else
        {
            // this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
            JustinaNavigation::startGetClose(goalX, goalY);
        }
        return;
    }
    else
    {
        goal_location = parts[0];
        //this->ui->navLblStatus->setText("Base Status: Moving to goal point...");
        JustinaNavigation::startGetClose(goal_location);
    }
}

void MainWindow::navMoveChanged()
{
    std::vector<std::string> parts;
    std::string str = this->ui->navTxtMove->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() < 1)
        return;

    if(parts[0].compare("l") == 0)
    {
        if(parts.size() < 2)
            return;
        std::stringstream ssLateral(parts[1]);
        float lateral;
        if(!(ssLateral >> lateral))
            return;
        JustinaNavigation::startMoveLateral(lateral);
        return;
    }

    float dist = 0;
    float angle = 0;
    std::stringstream ssDist(parts[0]);
    if(!(ssDist >> dist))
        return;
    if(parts.size() > 1)
    {
        std::stringstream ssAngle(parts[1]);
        if(!(ssAngle >> angle))
            return;
    }
    JustinaNavigation::startMoveDistAngle(dist, angle);
}

void MainWindow::navObsDetectionEnableClicked()
{
    if(this->navDetectingObstacles)
    {
        JustinaNavigation::enableObstacleDetection(false);
        this->navDetectingObstacles = false;
        this->ui->navBtnStartObsDetection->setText("Enable");
    }
    else
    {
        JustinaNavigation::enableObstacleDetection(true);
        this->navDetectingObstacles = true;
        this->ui->navBtnStartObsDetection->setText("Disable");
    }
}

void MainWindow::hdPanTiltChanged(double)
{
    float goalPan = this->ui->hdTxtPan->value();
    float goalTilt = this->ui->hdTxtTilt->value();
    std::cout << "QMainWindow.->Setting new head goal pose: " << goalPan << "  " << goalTilt  << std::endl;
    JustinaHardware::setHeadGoalPose(goalPan, goalTilt);
    //JustinaManip::startHdGoTo(goalPan, goalTilt);
    //JustinaHardware::setHeadGoalPose(goalPan, goalTilt);
    //JustinaManip::startHdGoTo(goalPan, goalTilt);
}

void MainWindow::spgSayChanged()
{
    std::string strToSay = this->ui->spgTxtSay->text().toStdString();
    std::cout << "QMainWindow.->Saying: " << strToSay << std::endl;
    JustinaHRI::say(strToSay);
}

void MainWindow::sprFakeRecognizedChanged()
{
    std::string strToFake = this->ui->sprTxtFakeRecog->text().toStdString();
    std::cout << "QMainWindow.->Faking recog speech: " << strToFake << std::endl;
    JustinaHRI::fakeSpeechRecognized(strToFake);
}

void MainWindow::recSaveVideoChanged()
{
    if(this->recSavingVideo)
    {
        std::cout << "QMainWindow.->Stop saving video." << std::endl;
        JustinaHardware::stopSavingCloud();
        this->ui->recBtnSaveVideo->setText("Start saving video");
        this->ui->recLblStatus->setText("Status: Stand by");
        this->recSavingVideo = false;
    }
    else
    {
        std::string fileName = this->ui->recTxtVideoFile->text().toStdString();
        if(!boost::filesystem::portable_posix_name(fileName))
        {
            std::cout << "QMainWindow.->File name for video is not a valid name :'(" << std::endl;
            this->ui->recLblStatus->setText("Status: Invalid file name...");
            return;
        }
        std::cout << "QMainWindow.->Starting to save video at: " << fileName << std::endl;
        JustinaHardware::startSavingCloud(fileName);
        this->ui->recBtnSaveVideo->setText("Stop saving video");
        this->ui->recLblStatus->setText("Status: saving video...");
        this->recSavingVideo = true;
    }
}

void MainWindow::recSaveImageChanged()
{
}

void MainWindow::sktBtnStartClicked()
{
    if(this->sktRecognizing)
    {
        JustinaVision::stopSkeletonFinding();
        this->sktRecognizing = false;
        this->ui->sktBtnStartRecog->setText("Start Skeletons");
    }
    else
    {
        JustinaVision::startSkeletonFinding();
        this->sktRecognizing = true;
        this->ui->sktBtnStartRecog->setText("Stop Skeletons");
    }
}

void MainWindow::facBtnStartClicked()
{
    if(this->facRecognizing)
    {
        JustinaVision::startFaceRecognition(false);
        JustinaVision::startFaceDetection(false);
        this->facRecognizing = false;
        this->ui->facBtnStartRecog->setText("Start Recognizer");
    }
    else
    {
        if(faceRecognition)
            JustinaVision::startFaceRecognition(true);
        else
            JustinaVision::startFaceDetection(true);
        this->facRecognizing = true;
        this->ui->facBtnStartRecog->setText("Stop Recognizing");
    }
}

void MainWindow::facRecogPressed()
{
    std::string id = this->ui->facTxtRecog->text().toStdString();
    if(id.compare("") == 0)
    {
        if(this->facRecognizing)
            JustinaVision::startFaceRecognition(true);
        faceRecognition = false;
        //std::cout << "QMainWindow.->Starting recognition without id" << std::endl;
        return;
    }
    else{
        if(this->facRecognizing)
            JustinaVision::startFaceDetection(true);
        faceRecognition = true;
    }
    if(!boost::filesystem::portable_posix_name(id))
    {
        //std::cout << "QMainWindow.->Invalid ID for face recognition. " << std::endl;
        return;
    }
    JustinaVision::setIdFaceRecognition(id);
}

void MainWindow::facTrainPressed()
{
    std::string str = this->ui->facTxtTrain->text().toStdString();
    std::vector<std::string> parts;
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() < 1)
        return;

    int numOfFrames = -1;
    if(!boost::filesystem::portable_posix_name(parts[0]))
    {
        std::cout << "QMainWindow.->Invalid ID for face training. " << std::endl;
        return;
    }
    if(parts.size() > 1)
    {
        std::stringstream ssValue(parts[1]);
        if(!(ssValue >> numOfFrames) || numOfFrames <= 0)
        {
            std::cout << "QMainWindow.->Invalid number of frames for face training. " << std::endl;
            return;
        }
    }
    
    if(numOfFrames <= 0)
    {
        std::cout << "QMainWindow.->Sending face training without number of frames. " << std::endl;
        //JustinaVision::facTrain(parts[0]);
        return;
    }
    std::cout << "QMainWindow.->Sending face training with " << numOfFrames << " number of frames. " << std::endl;
    //JustinaVision::facTrain(parts[0], numOfFrames);
    return;
}

void MainWindow::facClearPressed()
{
    std::string str = this->ui->facTxtClear->text().toStdString();
    if(str.compare("ALL") == 0)
    {
        std::cout << "QMainWindow.->Clearing all face recognition database" << std::endl;
        //JustinaVision::facClearAll();
        return;
    }
    if(!boost::filesystem::portable_posix_name(str))
    {
        std::cout << "QMainWindow.->Invalid ID for clearing face database. " << std::endl;
        return;
    }
    //JustinaVision::facClearByID(str);
}

void MainWindow::objRecogObjectChanged()
{
    std::vector<vision_msgs::VisionObject> recoObjList;
    JustinaRepresentation::initKDB("", true, 0);
    if(!JustinaVision::detectObjects(recoObjList))
    {
        std::cout << "MainWindow.->Cannot dectect objects :'( " << std::endl;
        return;
    }
    QString txtResult = "";
    this->ui->objTxtResults->setPlainText(txtResult);
    for(int i=0; i < recoObjList.size(); i++)
    {
        txtResult = "Id: " + QString::fromStdString(recoObjList[i].id);
        this->ui->objTxtResults->appendPlainText(txtResult);
        txtResult = "Centroid:";
        this->ui->objTxtResults->appendPlainText(txtResult);
        txtResult = QString::number(recoObjList[i].pose.position.x, 'f', 3) + "  " + QString::number(recoObjList[i].pose.position.y, 'f', 3) +
            "  " + QString::number(recoObjList[i].pose.position.z, 'f', 3);
        this->ui->objTxtResults->appendPlainText(txtResult);
        txtResult = "";
        this->ui->objTxtResults->appendPlainText(txtResult);
    }
}

void MainWindow::vsnFindLinesClicked()
{
    float x1, y1, z1, x2, y2, z2;
    JustinaVision::findLine(x1, y1, z1, x2, y2, z2);
}

//HRI
void MainWindow::hriBtnFollowClicked()
{
    if(this->hriFollowing)
    {
        this->ui->hriBtnStartFollow->setText("Start Follow");
        JustinaHRI::stopFollowHuman();
        this->hriFollowing = false;
    }
    else
    {
        this->ui->hriBtnStartFollow->setText("Stop Follow");
        JustinaHRI::startFollowHuman();
        this->hriFollowing = true;
    }
}

void MainWindow::hriBtnLegsClicked()
{
    if(this->hriFindingLegs)
    {
        JustinaHRI::enableLegFinder(false);
        this->ui->hriBtnStartLegs->setText("Start Leg Finder");
        this->hriFindingLegs = false;
    }
    else
    {
        JustinaHRI::enableLegFinder(true);
        this->ui->hriBtnStartLegs->setText("Stop Leg Finder");
        this->hriFindingLegs = true;
    }
}

//
//SLOTS FOR SIGNALS EMITTED IN THE QTROSNODE
//

void MainWindow::updateGraphicsReceived()
{
    float rX;
    float rY;
    float rT;
    JustinaNavigation::getRobotPose(rX, rY, rT);
    //std::cout << "MainWindow.->Current pose: " << currentX << "  " << currentY << "  " << currentA << std::endl;
    QString robotTxt = "Robot Pose: "+ QString::number(rX,'f',3) + "  " + QString::number(rY,'f',3) + "  " + QString::number(rT,'f',4);
    this->ui->navLblRobotPose->setText(robotTxt);
    this->robotX = rX;
    this->robotY = rY;
    this->robotA = rT;

    float pan;
    float tilt;
    JustinaHardware::getHeadCurrentPose(pan, tilt);
    QString headTxt = QString::number(pan, 'f', 4) + "  " + QString::number(tilt, 'f', 4);
    this->ui->hdLblHeadPose->setText(headTxt);
    this->headPan = pan;
    this->headTilt = tilt;

    if(JustinaNavigation::isGoalReached())
        this->ui->navLblStatus->setText("Base Status: Goal Reached (Y)");
    else
        this->ui->navLblStatus->setText("Base Status: Moving to goal pose...");


    std::string faceId = "";
    float facePosX = 0, facePosY = 0, facePosZ = 0;
    float faceConfidence = -1;
    int faceGender = -1;
    bool faceSmiling = false;
    if(JustinaVision::getMostConfidentFace(faceId, facePosX, facePosY, facePosZ, faceConfidence, faceGender, faceSmiling))
    {
        QString faceIdQ = QString::fromStdString("ResultID: " + faceId);
        this->ui->facLblResultID->setText(faceIdQ);
        QString facePos = "Position: " +QString::number(facePosX,'f',2)+" "+QString::number(facePosY,'f',2)+" "+QString::number(facePosZ,'f',2);
        this->ui->facLblResultPose->setText(facePos);
        if(faceGender == 0)
            this->ui->facLblResultGender->setText("Gender: Female");
        else if(faceGender == 1)
            this->ui->facLblResultGender->setText("Gender: Male");
        else
            this->ui->facLblResultGender->setText("Gender: Unknown");
        if(faceSmiling)
            this->ui->facLblResultSmile->setText("Smiling: Yes");
        else
            this->ui->facLblResultSmile->setText("Smiling: No");
    }

    if(JustinaNavigation::obstacleInFront())
        this->ui->navLblObstacleInFront->setText("Obs In Front: True");
    else
        this->ui->navLblObstacleInFront->setText("Obs In Front: False");

    if(JustinaNavigation::collisionRisk())
        this->ui->navLblRiskOfCollision->setText("Risk of Collision: True");
    else
        this->ui->navLblRiskOfCollision->setText("Risk of Collision: False");

    this->ui->sprLblLastRecog->setText(QString::fromStdString("Recog: " + JustinaHRI::lastRecogSpeech()));

    this->ui->pgbBatt1->setValue((JustinaHardware::leftArmBatteryPerc() + JustinaHardware::rightArmBatteryPerc())/2);
    this->ui->pgbBatt2->setValue((JustinaHardware::headBatteryPerc() + JustinaHardware::baseBatteryPerc())/2);
    QString batt1Txt = QString::number((JustinaHardware::leftArmBattery() + JustinaHardware::rightArmBattery())/2, 'f', 2) + " V";
    QString batt2Txt = QString::number((JustinaHardware::headBattery() + JustinaHardware::baseBattery())/2, 'f', 2) + " V";
    this->ui->lblBatt1Level->setText(batt1Txt);
    this->ui->lblBatt2Level->setText(batt2Txt);
}
