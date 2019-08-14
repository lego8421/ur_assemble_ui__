/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ur_assemble_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ur_assemble_ui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc, argv) {
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	setWindowIcon(QIcon(":/images/icon.png"));
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(ui.pushButtonTaskPose, SIGNAL(clicked()), this, SLOT(pushButtonTaskPoseClickedCallback()));
	QObject::connect(ui.pushButtonMove, SIGNAL(clicked()), this, SLOT(pushButtonMoveClickedCallback()));

	QObject::connect(ui.pushButtonImpedanceOn, SIGNAL(clicked()), this, SLOT(pushButtonImpedanceOnClickedCallback()));
	QObject::connect(ui.pushButtonImpedanceOff, SIGNAL(clicked()), this, SLOT(pushButtonImpedanceOffClickedCallback()));
	QObject::connect(ui.pushButtonForceBias, SIGNAL(clicked()), this, SLOT(pushButtonForceBiasClickedCallback()));

	QObject::connect(ui.pushButtonPinPose_1, SIGNAL(clicked()), this, SLOT(pushButtonPinPose1ClickedCallback()));
	QObject::connect(ui.pushButtonPinPose_2, SIGNAL(clicked()), this, SLOT(pushButtonPinPose2ClickedCallback()));
	QObject::connect(ui.pushButtonPinPose_3, SIGNAL(clicked()), this, SLOT(pushButtonPinPose3ClickedCallback()));
	QObject::connect(ui.pushButtonPinPose_4, SIGNAL(clicked()), this, SLOT(pushButtonPinPose4ClickedCallback()));
	QObject::connect(ui.pushButtonMovePinPose, SIGNAL(clicked()), this, SLOT(pushButtonMovePinPoseClickedCallback()));
	QObject::connect(ui.pushButtonInsertPin, SIGNAL(clicked()), this, SLOT(pushButtonInsertPinClickedCallback()));

	actualQList_ << ui.lineEditActualQ_1
	             << ui.lineEditActualQ_2
	             << ui.lineEditActualQ_3
	             << ui.lineEditActualQ_4
	             << ui.lineEditActualQ_5
	             << ui.lineEditActualQ_6;

	actualXList_ << ui.lineEditActualX_1
	             << ui.lineEditActualX_2
	             << ui.lineEditActualX_3
	             << ui.lineEditActualX_4
	             << ui.lineEditActualX_5
	             << ui.lineEditActualX_6;

	actualForceList_ << ui.lineEditActualForce_1
	                 << ui.lineEditActualForce_2
	                 << ui.lineEditActualForce_3
	                 << ui.lineEditActualForce_4
	                 << ui.lineEditActualForce_5
	                 << ui.lineEditActualForce_6;

	targetXList_ << ui.lineEditTargetX_1
	             << ui.lineEditTargetX_2
	             << ui.lineEditTargetX_3
	             << ui.lineEditTargetX_4
	             << ui.lineEditTargetX_5
	             << ui.lineEditTargetX_6;

	pinXList_ << ui.lineEditPinX_1
	          << ui.lineEditPinX_2
	          << ui.lineEditPinX_3
	          << ui.lineEditPinX_4
	          << ui.lineEditPinX_5
	          << ui.lineEditPinX_6;

	qnode.init();

	timer_ = new QTimer(this);
	connect(timer_, SIGNAL(timeout()), this, SLOT(timerCallback()));
	timer_->start(1);
}

MainWindow::~MainWindow() {}

void MainWindow::timerCallback() {
	std::vector<double> actualQ = qnode.getActualQ();
	std::vector<double> actualX = qnode.getActualX();
	std::vector<double> actualForce = qnode.getActualForce();

	for (std::size_t i = 0; i < 6; i++) {
		actualQList_[i]->setText(QString::number(actualQ[i], 'f', 2));
		actualXList_[i]->setText(QString::number(actualX[i], 'f', 5));
		actualForceList_[i]->setText(QString::number(actualForce[i], 'f', 2));
	}
}

void MainWindow::pushButtonTaskPoseClickedCallback() {
	std::vector<double> taskPose = {0.615, -0.175, 0.241, -180.0, 0.0, -90.0};
	for (std::size_t i = 0; i < taskPose.size(); i++) {
		targetXList_[i]->setText(QString::number(taskPose[i], 'f', 5));
	}
}

void MainWindow::pushButtonMoveClickedCallback() {
	std::vector<double> pose;
	double maxVelocity = ui.doubleSpinBoxTargetVelocity->value();
	double acceleration = ui.doubleSpinBoxTargetAcceleration->value();
	for (std::size_t i = 0; i < targetXList_.size(); i++) {
		pose.push_back(targetXList_[i]->text().toDouble());
	}

	qnode.setTargetPose(pose, maxVelocity, acceleration);
}

void MainWindow::pushButtonImpedanceOnClickedCallback() {
	std::vector<double> stiffness(6);
	std::vector<double> nf(6);
	std::vector<double> zeta(6);
	std::vector<double> forceLimit(6);

	QStringList stiffnessList = ui.lineEditStiffness->text().split(",");
	QStringList nfList = ui.lineEditNf->text().split(",");
	QStringList zetaList = ui.lineEditZeta->text().split(",");
	QStringList forceLimitList = ui.lineEditForceLimit->text().split(",");

	for (std::size_t i = 0; i < 6; i++) {
		stiffness[i] = stiffnessList[i].toDouble();
		nf[i] = nfList[i].toDouble();
		zeta[i] = zetaList[i].toDouble();
		forceLimit[i] = forceLimitList[i].toDouble();
	}

	qnode.setImpedanceControl(true, stiffness, nf, zeta, forceLimit);
}

void MainWindow::pushButtonImpedanceOffClickedCallback() {
	std::vector<double> stiffness(6);
	std::vector<double> nf(6);
	std::vector<double> zeta(6);
	std::vector<double> forceLimit(6);

	QStringList stiffnessList = ui.lineEditStiffness->text().split(",");
	QStringList nfList = ui.lineEditNf->text().split(",");
	QStringList zetaList = ui.lineEditZeta->text().split(",");
	QStringList forceLimitList = ui.lineEditForceLimit->text().split(",");

	for (std::size_t i = 0; i < 6; i++) {
		stiffness[i] = stiffnessList[i].toDouble();
		nf[i] = nfList[i].toDouble();
		zeta[i] = zetaList[i].toDouble();
		forceLimit[i] = forceLimitList[i].toDouble();
	}

	qnode.setImpedanceControl(false, stiffness, nf, zeta, forceLimit);
}

void MainWindow::pushButtonForceBiasClickedCallback() {
    qnode.setForceBias();
}

void MainWindow::pushButtonPinPose1ClickedCallback() {
	std::vector<double> pinPose = {0.52732, -0.01939, 0.14800, -180.0, 0.0, -90.0};
    QString stiffness = "1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0";
    QString nf = "25.0, 25.0, 25.0, 25.0, 25.0, 25.0";
    QString zeta = "80.0, 80.0, 80.0, 40.0, 40.0, 40.0";
    QString forceLimit = "80.0, 80.0, 80.0, 80.0, 80.0, 80.0";

    ui.lineEditStiffness->setText(stiffness);
    ui.lineEditNf->setText(nf);
    ui.lineEditZeta->setText(zeta);
    ui.lineEditForceLimit->setText(forceLimit);

	for (std::size_t i = 0; i < 6; i++) {
		pinXList_[i]->setText(QString::number(pinPose[i], 'f', 5));
	}
}

void MainWindow::pushButtonPinPose2ClickedCallback() {
	std::vector<double> pinPose = {0.52621, -0.05054, 0.14800, -180.0, 0.0, -90.0};
    QString stiffness = "1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0";
    QString nf = "25.0, 25.0, 25.0, 25.0, 25.0, 25.0";
    QString zeta = "80.0, 80.0, 80.0, 40.0, 40.0, 40.0";
    QString forceLimit = "80.0, 80.0, 80.0, 80.0, 80.0, 80.0";

    ui.lineEditStiffness->setText(stiffness);
    ui.lineEditNf->setText(nf);
    ui.lineEditZeta->setText(zeta);
    ui.lineEditForceLimit->setText(forceLimit);

	for (std::size_t i = 0; i < 6; i++) {
		pinXList_[i]->setText(QString::number(pinPose[i], 'f', 5));
	}
}

void MainWindow::pushButtonPinPose3ClickedCallback() {
	std::vector<double> pinPose = {0.89750, -0.06900, 0.15200, -180.0, 0.0, -90.0};
    QString stiffness = "1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0";
    QString nf = "25.0, 25.0, 25.0, 25.0, 25.0, 25.0";
    QString zeta = "80.0, 80.0, 80.0, 40.0, 40.0, 40.0";
    QString forceLimit = "80.0, 80.0, 80.0, 80.0, 80.0, 80.0";

    ui.lineEditStiffness->setText(stiffness);
    ui.lineEditNf->setText(nf);
    ui.lineEditZeta->setText(zeta);
    ui.lineEditForceLimit->setText(forceLimit);
    
	for (std::size_t i = 0; i < 6; i++) {
		pinXList_[i]->setText(QString::number(pinPose[i], 'f', 5));
	}
}

void MainWindow::pushButtonPinPose4ClickedCallback() {
	std::vector<double> pinPose = {0.89797, -0.03800, 0.15350, -180.0, 0.0, -90.0};
    QString stiffness = "1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0";
    QString nf = "25.0, 25.0, 25.0, 25.0, 25.0, 25.0";
    QString zeta = "80.0, 80.0, 80.0, 40.0, 40.0, 40.0";
    QString forceLimit = "80.0, 80.0, 80.0, 80.0, 80.0, 80.0";

    ui.lineEditStiffness->setText(stiffness);
    ui.lineEditNf->setText(nf);
    ui.lineEditZeta->setText(zeta);
    ui.lineEditForceLimit->setText(forceLimit);
    
	for (std::size_t i = 0; i < 6; i++) {
		pinXList_[i]->setText(QString::number(pinPose[i], 'f', 5));
	}
}

void MainWindow::pushButtonMovePinPoseClickedCallback() {
    std::vector<double> pose;
	double maxVelocity = ui.doubleSpinBoxMoveVelocity->value();
	double acceleration = ui.doubleSpinBoxTargetAcceleration->value();
	for (std::size_t i = 0; i < pinXList_.size(); i++) {
		pose.push_back(pinXList_[i]->text().toDouble());
	}

	qnode.setTargetPose(pose, maxVelocity, acceleration);
}

void MainWindow::pushButtonInsertPinClickedCallback() {
    std::vector<double> pinPose(6);
	double pinDepth = ui.doubleSpinBoxPinDepth->value();
	std::vector<double> moveVelocity(6);
	std::vector<double> contactVelocity(6);
	std::vector<double> insertVelocity(6);
	std::vector<double> targetForce(6);
	std::vector<double> contactForce(6);
	std::vector<double> forceLimit(6);
	std::vector<double> stiffness(6);
	std::vector<double> zeta(6);

    QStringList targetForceList = ui.lineEditTargetForce->text().split(",");
    QStringList contactForceList = ui.lineEditContactForce->text().split(",");
	QStringList forceLimitList = ui.lineEditForceLimit->text().split(",");

	QStringList stiffnessList = ui.lineEditStiffness->text().split(",");
	QStringList zetaList = ui.lineEditZeta->text().split(",");

    for (std::size_t i=0; i<6; i++) {
        pinPose[i] = pinXList_[i]->text().toDouble();
        targetForce[i] = targetForceList[i].toDouble();
        contactForce[i] = contactForceList[i].toDouble();
        forceLimit[i] = forceLimitList[i].toDouble();
        stiffness[i] = stiffnessList[i].toDouble();
        zeta[i] = zetaList[i].toDouble();
    }

    for (std::size_t i=0; i<3; i++) {
        moveVelocity[i] = ui.doubleSpinBoxMoveVelocity->value();
        contactVelocity[i] = ui.doubleSpinBoxContactVelocity->value();
        insertVelocity[i] = ui.doubleSpinBoxInsertVelocity->value();
    
        moveVelocity[i+3] = 20.0;
        contactVelocity[i+3] = 2.0;
        insertVelocity[i+3] = 10.0;
    }

	qnode.setAssemblePin(pinPose, pinDepth, moveVelocity, contactVelocity, insertVelocity, targetForce, contactForce, forceLimit, stiffness, zeta);
}

}  // namespace ur_assemble_ui

