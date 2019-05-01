#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaRepresentation.h"
#include "QtRosNode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;
    float robotX;
    float robotY;
    float robotA;
    float headPan;
    float headTilt;

    nav_msgs::Path calculatedPath;
    bool recSavingVideo;
    bool sktRecognizing;
    bool facRecognizing;
    bool hriFollowing;
    bool hriFindingLegs;
    bool navDetectingObstacles;
    bool faceRecognition;

    std::map<std::string, std::vector<std::string> > locations;
    std::map<std::string, std::vector<std::string> > objects;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    void setPathKnownLoc(const std::string pathKnownLoc);

private:
    bool strToFloatArray(std::string str, std::vector<float>& result);

public slots:
    //Slots for signals emitted in this window (e.g.: pressing buttons)
    void stopRobot();
    //Buttons for manual drive
    void btnFwdPressed();
    void btnFwdReleased();
    void btnBwdPressed();    
    void btnBwdReleased();
    void btnLeftPressed();
    void btnLeftReleased();
    void btnRightPressed();
    void btnRightReleased();
    //Navigation
    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();
    void navMoveChanged();
    void navObsDetectionEnableClicked();
    //Hardware
    void hdPanTiltChanged(double d);
    //Speech synthesis and recog
    void spgSayChanged();
    void sprFakeRecognizedChanged();
    //Vision
    void recSaveVideoChanged();
    void recSaveImageChanged();
    void sktBtnStartClicked();
    void facBtnStartClicked();
    void facRecogPressed();
    void facTrainPressed();
    void facClearPressed();
    void objRecogObjectChanged();
    void vsnFindLinesClicked();
    //HRI
    void hriBtnFollowClicked();
    void hriBtnLegsClicked();
    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    
private slots:

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
