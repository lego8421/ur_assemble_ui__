#ifndef ur_assemble_ui_MAIN_WINDOW_H
#define ur_assemble_ui_MAIN_WINDOW_H

#include <QtGui>
#include <QTimer>
#include <QLineEdit>
#include <QList>

#include "ui_main_window.h"
#include "qnode.hpp"


namespace ur_assemble_ui {

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

public Q_SLOTS:
	void timerCallback();

	void pushButtonTaskPoseClickedCallback();
	void pushButtonMoveClickedCallback();

	void pushButtonImpedanceOnClickedCallback();
	void pushButtonImpedanceOffClickedCallback();
	void pushButtonForceBiasClickedCallback();

	void pushButtonPinPose1ClickedCallback();
	void pushButtonPinPose2ClickedCallback();
	void pushButtonPinPose3ClickedCallback();
	void pushButtonPinPose4ClickedCallback();
	void pushButtonMovePinPoseClickedCallback();
	void pushButtonInsertPinClickedCallback();
	
private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	QTimer *timer_;

    QList <QLineEdit *> actualQList_;
    QList <QLineEdit *> actualXList_;
    QList <QLineEdit *> actualForceList_;

    QList <QLineEdit *> targetXList_;
    QList <QLineEdit *> pinXList_;
};

}  // namespace ur_assemble_ui

#endif // ur_assemble_ui_MAIN_WINDOW_H
