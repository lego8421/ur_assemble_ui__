#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLineEdit>
#include <QList>
#include <QTcpSocket>
#include <QTimer>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

#include <vector>


namespace Ui {
class MainWindow;
}


const double DEGTORAD = M_PI / 180.0;
const double RADTODEG = 180.0 / M_PI;
const double GRAVITY = 9.81;
const double EPSILON = 0.0001;

const int ROBOT_DOF = 6;

// Status
const int STATUS_DEFAULTS = 100;
const int ENABLE = 101;
const int DISABLE = 102;
const int JSCTRL = 200;
const int CSCTRL = 300;
const int DEMO = 400;

// Control Command
const int COMMAND_DEFAULTS = 100;
const int ROBOT_ENABLE = 101;
const int ROBOT_DISABLE = 102;
const int ROBOT_STOP = 103;

const int JTS_BIAS = 104;
const int CD_ON = 105;
const int CD_OFF = 106;
const int FRICTION_OBSERVER = 107;
const int FRICTION_MODEL = 108;

const int JS_HG_ON = 110;
const int JS_HG_OFF = 111;
const int CS_HG_ON = 112;
const int CS_HG_OFF = 113;
const int CS_HG_X_ON = 114;
const int CS_HG_X_OFF = 115;
const int CS_HG_Y_ON = 116;
const int CS_HG_Y_OFF = 117;
const int CS_HG_Z_ON = 118;
const int CS_HG_Z_OFF = 119;

const int JS_FRICTION_OBSERVER = 107;
const int CS_IMPEDANCE_INITIATION = 316;

const int SET_DYN_PARA_GRIPPER = 121;
const int SET_DYN_PARA_POINTER = 122;
const int SET_DYN_PARA_PAYLOAD = 123;

const int JSCTRL_START = 200;
const int JS_TARGET_PATH = 201;

const int CSCTRL_START = 300;
const int CS_TARGET_PATH = 301;
const int CS_CIRCULAR_PATH = 317;

const int CS_IMPEDANCE_CTRL_ON = 310;
const int CS_IMPEDANCE_CTRL_OFF = 311;

const int CS_FORCE_CTRL_ON = 320;
const int CS_FORCE_CTRL_OFF = 321;
const int CS_FORCE_BIAS = 322;

const int DEMO1_START = 400;
const int DEMO1_END = 401;
const int DEMO2_START = 402;
const int DEMO2_END = 403;

const int GRIPPER_OPEN = 500;
const int GRIPPER_CLOSED = 501;

const int SET_TCP = 600;
const int SET_PAYLOAD = 601;
const int SET_SAFETY = 602;

const int SCREWDRIVING_TARGET_PATH = 701;

const int COMMAND_ARCBLENDING = 800;
const int COMMAND_ARCBLENDING_ORIENTATION = 801;
const int COMMAND_ARCBLENDING_CHECK = 802;
const int COMMAND_ARCBLENDING_DIFFVEL = 803;

const int ASSEMBLE_TARGET_START = 900;

typedef struct _ControlParameter {
    uint32_t command;
    uint32_t status;
    bool isPathOperating;
    bool doImpedanceControl;

    // robot
    std::vector<double> actualQ;
    std::vector<double> actualQd;
    std::vector<double> actualTorque;
    std::vector<double> actualCurrent;

    std::vector<double> actualX;
    std::vector<double> actualXd;
    std::vector<double> actualForce;

    std::vector<double> forceBias;

    _ControlParameter() {
        command = COMMAND_DEFAULTS;
        status = STATUS_DEFAULTS;
        isPathOperating = false;

        actualQ.resize(ROBOT_DOF);
        actualQd.resize(ROBOT_DOF);
        actualTorque.resize(ROBOT_DOF);
        actualCurrent.resize(ROBOT_DOF);

        actualX.resize(6);
        actualXd.resize(6);
        actualForce.resize(6);

        forceBias.resize(6);

        doImpedanceControl = false;
    }
} ControlParameter;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void onUpdate();

private slots:
    void on_pushButtonSocketConnect_clicked();
    void on_pushButtonForceBias_clicked();
    void on_pushButtonTaskPose_clicked();

    void on_pushButtonSendScript_clicked();

    void on_pushButtonGotoPin_1_clicked();
    void on_pushButtonGotoPin_2_clicked();
    void on_pushButtonGotoPin_3_clicked();
    void on_pushButtonGotoPin_4_clicked();

    void on_pushButtonImpedanceOn_clicked();
    void on_pushButtonImpedanceOff_clicked();

    void on_pushButtonAssemblePin_1_clicked();

    void on_pushButtonAssemblePin_2_clicked();

    void on_pushButtonAssemblePin_3_clicked();

    void on_pushButtonAssemblePin_4_clicked();

private:
    void sendJson(QJsonObject json);
    void sendMessage(QByteArray message);
    bool parseJsonFromBuffer(QJsonObject &data);
    std::vector<double> toVector(const QJsonArray json);
    QJsonArray toJsonArray(const std::vector<double> vector);

private:
    Ui::MainWindow *ui;

    QList <QLineEdit *> actualQList_;
    QList <QLineEdit *> actualXList_;
    QList <QLineEdit *> actualForceList_;

    QTcpSocket *socket_;
    QByteArray buffer_;

    QTimer *timer_;

    ControlParameter parameter_;
};

#endif // MAINWINDOW_H
