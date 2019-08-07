#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QNetworkInterface>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
    ui->setupUi(this);

    actualQList_ << ui->lineEditJ1
                 << ui->lineEditJ2
                 << ui->lineEditJ3
                 << ui->lineEditJ4
                 << ui->lineEditJ5
                 << ui->lineEditJ6;

    actualXList_ << ui->lineEditX
                 << ui->lineEditY
                 << ui->lineEditZ
                 << ui->lineEditRoll
                 << ui->lineEditPitch
                 << ui->lineEditYaw;

    actualForceList_ << ui->lineEditX_2
                     << ui->lineEditY_2
                     << ui->lineEditZ_2
                     << ui->lineEditRoll_2
                     << ui->lineEditPitch_2
                     << ui->lineEditYaw_2;

    QHostAddress ip;
    const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
    for (const QHostAddress &address : QNetworkInterface::allAddresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost) {
            ip =  address;
            break;
        }
    }
    ui->lineEditAddress->setText(ip.toString());

    socket_ = new QTcpSocket(this);

    timer_ = new QTimer(this);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_pushButtonSocketConnect_clicked() {
    ui->pushButtonSocketConnect->setEnabled(false);

    QHostAddress address(ui->lineEditAddress->text());

    socket_->connectToHost(address, 8421);

    timer_->start(1);
    connect(timer_, &QTimer::timeout, this, &MainWindow::onUpdate);
}

bool MainWindow::parseJsonFromBuffer(QJsonObject &data) {
    int index = buffer_.indexOf('{');
    if (index == -1) {
        buffer_.clear();
        return false;
    }
    buffer_.remove(0, index);

    index = buffer_.indexOf('}');
    if (index == -1) {
        return false;
    }

    QByteArray json = buffer_.mid(0, index + 1);
    buffer_.remove(0, index + 1);

    QJsonDocument itemDoc = QJsonDocument::fromJson(json);
    data = itemDoc.object();

    return true;
}

std::vector<double> MainWindow::toVector(const QJsonArray json) {
    std::vector<double> ret;
    for(int i = 0; i < json.size(); i++) {
        ret.push_back(json.at(i).toDouble());
    }
    return ret;
}

QJsonArray MainWindow::toJsonArray(const std::vector<double> vector) {
    QJsonArray ret;
    for (std::size_t i = 0; i < vector.size(); i++) {
        ret.push_back(vector[i]);
    }
    return ret;
}

void MainWindow::onUpdate() {
    if (socket_->state() != QAbstractSocket::ConnectedState) {
        ui->pushButtonSocketConnect->setEnabled(true);
        timer_->stop();
        return;
    }

    socket_->waitForReadyRead(1);
    if (socket_->bytesAvailable()) {
        buffer_ += socket_->readAll();
    }

    QJsonObject json;
    while (parseJsonFromBuffer(json)) {
        if (json.keys().indexOf("actualQ") != -1) {
            parameter_.actualQ = toVector(json["actualQ"].toArray());
        }
        if (json.keys().indexOf("actualX") != -1) {
            parameter_.actualX = toVector(json["actualX"].toArray());
        }
        if (json.keys().indexOf("actualForce") != -1) {
            parameter_.actualForce = toVector(json["actualForce"].toArray());
        }

        if (json.keys().indexOf("status") != -1) {
            parameter_.status = json["status"].toInt();
        }
    }

    for (int i = 0; i < 6; i++) {
        actualQList_[i]->setText(QString::number(parameter_.actualQ[i], 'f', 2));
        actualXList_[i]->setText(QString::number(parameter_.actualX[i], 'f', 5));
        actualForceList_[i]->setText(QString::number(parameter_.actualForce[i], 'f', 2));
    }

    ui->lineEditStatus->setText(QString::number(parameter_.status));
}

void MainWindow::on_pushButtonForceBias_clicked() {
    QJsonObject json;
    json["command"] = CS_FORCE_BIAS;
    sendJson(json);
}

void MainWindow::on_pushButtonTaskPose_clicked() {
    QJsonObject json;

    QJsonArray targetQ = toJsonArray({0.0, -90.0, -120.0, -60.0, 90.0, 0.0});
    //QJsonArray targetQ = toJsonArray({0.0, -90.0, 0.0, -90.0, 0.0, 0.0});
    QJsonArray targetQd = toJsonArray({20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
    QJsonArray targetQdd = toJsonArray({15.0, 15.0, 15.0, 15.0, 15.0, 15.0});

    json["command"] = JS_TARGET_PATH;
    json["targetQ"] = targetQ;
    json["targetQd"] = targetQd;
    json["targetQdd"] = targetQdd;

    sendJson(json);

}

void MainWindow::sendJson(QJsonObject json) {
    if (socket_->state() == QAbstractSocket::ConnectedState) {
        QJsonDocument doc(json);
        sendMessage(doc.toJson(QJsonDocument::Compact));
    }
}

void MainWindow::sendMessage(QByteArray message) {
    if (socket_->state() == QAbstractSocket::ConnectedState) {
        socket_->write(message);
        socket_->flush();
    }
}

void MainWindow::on_pushButtonSendScript_clicked() {
    QString script = ui->lineEditScript->text();
    sendMessage(script.toUtf8());
    ui->lineEditScript->clear();
}


void MainWindow::on_pushButtonGotoPin_1_clicked() {
    QJsonObject json;

    QJsonArray targetX;
    QJsonArray targetXd = toJsonArray({0.1, 0.1, 0.1, 10.0, 10.0, 10.0});
    QJsonArray targetXdd = toJsonArray({1.0, 1.0, 1.0, 15.0, 15.0, 15.0});

    targetX.push_back(ui->lineEditPinX_1->text().toDouble());
    targetX.push_back(ui->lineEditPinY_1->text().toDouble());
    targetX.push_back(ui->lineEditPinZ_1->text().toDouble());
    targetX.push_back(ui->lineEditPinRoll_1->text().toDouble());
    targetX.push_back(ui->lineEditPinPitch_1->text().toDouble());
    targetX.push_back(ui->lineEditPinYaw_1->text().toDouble());

    json["command"] = CS_TARGET_PATH;
    json["targetX"] = targetX;
    json["targetXd"] = targetXd;
    json["targetXdd"] = targetXdd;

    sendJson(json);

}

void MainWindow::on_pushButtonGotoPin_2_clicked() {
    QJsonObject json;

    QJsonArray targetX;
    QJsonArray targetXd = toJsonArray({0.3, 0.3, 0.3, 20.0, 20.0, 20.0});
    QJsonArray targetXdd = toJsonArray({1.0, 1.0, 1.0, 15.0, 15.0, 15.0});

    targetX.push_back(ui->lineEditPinX_2->text().toDouble());
    targetX.push_back(ui->lineEditPinY_2->text().toDouble());
    targetX.push_back(ui->lineEditPinZ_2->text().toDouble());
    targetX.push_back(ui->lineEditPinRoll_2->text().toDouble());
    targetX.push_back(ui->lineEditPinPitch_2->text().toDouble());
    targetX.push_back(ui->lineEditPinYaw_2->text().toDouble());

    json["command"] = CS_TARGET_PATH;
    json["targetX"] = targetX;
    json["targetXd"] = targetXd;
    json["targetXdd"] = targetXdd;

    sendJson(json);
}

void MainWindow::on_pushButtonGotoPin_3_clicked() {
    QJsonObject json;

    QJsonArray targetX;
    QJsonArray targetXd = toJsonArray({0.3, 0.3, 0.3, 20.0, 20.0, 20.0});
    QJsonArray targetXdd = toJsonArray({1.0, 1.0, 1.0, 15.0, 15.0, 15.0});

    targetX.push_back(ui->lineEditPinX_3->text().toDouble());
    targetX.push_back(ui->lineEditPinY_3->text().toDouble());
    targetX.push_back(ui->lineEditPinZ_3->text().toDouble());
    targetX.push_back(ui->lineEditPinRoll_3->text().toDouble());
    targetX.push_back(ui->lineEditPinPitch_3->text().toDouble());
    targetX.push_back(ui->lineEditPinYaw_3->text().toDouble());

    json["command"] = CS_TARGET_PATH;
    json["targetX"] = targetX;
    json["targetXd"] = targetXd;
    json["targetXdd"] = targetXdd;

    sendJson(json);
}

void MainWindow::on_pushButtonGotoPin_4_clicked() {
    QJsonObject json;

    QJsonArray targetX;
    QJsonArray targetXd = toJsonArray({0.3, 0.3, 0.3, 20.0, 20.0, 20.0});
    QJsonArray targetXdd = toJsonArray({1.0, 1.0, 1.0, 15.0, 15.0, 15.0});

    targetX.push_back(ui->lineEditPinX_4->text().toDouble());
    targetX.push_back(ui->lineEditPinY_4->text().toDouble());
    targetX.push_back(ui->lineEditPinZ_4->text().toDouble());
    targetX.push_back(ui->lineEditPinRoll_4->text().toDouble());
    targetX.push_back(ui->lineEditPinPitch_4->text().toDouble());
    targetX.push_back(ui->lineEditPinYaw_4->text().toDouble());

    json["command"] = CS_TARGET_PATH;
    json["targetX"] = targetX;
    json["targetXd"] = targetXd;
    json["targetXdd"] = targetXdd;

    sendJson(json);

}

void MainWindow::on_pushButtonImpedanceOn_clicked() {
    QJsonObject json;
    json["command"] = CS_IMPEDANCE_CTRL_ON;
    sendJson(json);
}

void MainWindow::on_pushButtonImpedanceOff_clicked() {
    QJsonObject json;
    json["command"] = CS_IMPEDANCE_CTRL_OFF;
    sendJson(json);
}

void MainWindow::on_pushButtonAssemblePin_1_clicked() {
    QJsonObject json;

    QJsonArray pinPose;
    double pinDepth = ui->lineEditPinSize_1->text().toDouble();
    QJsonArray targetForce = toJsonArray({0.0, 0.0, 10.0, 0.0, 0.0, 0.0});
    QJsonArray forceLimit = toJsonArray({80.0, 80.0, 80.0, 20.0, 20.0, 20.0});
    QJsonArray contactForce = toJsonArray({20.0, 20.0, 20.0, 5.0, 5.0, 5.0});

    QJsonArray stiffness = toJsonArray({1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0});
    QJsonArray zeta = toJsonArray({80.0, 80.0, 80.0, 40.0, 40.0, 40.0});

    pinPose.push_back(ui->lineEditPinX_1->text().toDouble());
    pinPose.push_back(ui->lineEditPinY_1->text().toDouble());
    pinPose.push_back(ui->lineEditPinZ_1->text().toDouble());
    pinPose.push_back(ui->lineEditPinRoll_1->text().toDouble());
    pinPose.push_back(ui->lineEditPinPitch_1->text().toDouble());
    pinPose.push_back(ui->lineEditPinYaw_1->text().toDouble());

    json["command"] = ASSEMBLE_TARGET_START;
    json["pinPose"] = pinPose;
    json["pinDepth"] = pinDepth;
    json["targetForce"] = targetForce;
    json["forceLimit"] = forceLimit;
    json["contactForce"] = contactForce;

    json["stiffness"] = stiffness;
    json["zeta"] = zeta;

    sendJson(json);
}

void MainWindow::on_pushButtonAssemblePin_2_clicked() {
    QJsonObject json;

    QJsonArray pinPose;
    double pinDepth = ui->lineEditPinSize_2->text().toDouble();
    QJsonArray targetForce = toJsonArray({0.0, 0.0, 10.0, 0.0, 0.0, 0.0});
    QJsonArray forceLimit = toJsonArray({80.0, 80.0, 80.0, 20.0, 20.0, 20.0});
    QJsonArray contactForce = toJsonArray({20.0, 20.0, 20.0, 5.0, 5.0, 5.0});

    QJsonArray stiffness = toJsonArray({1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0});
    QJsonArray zeta = toJsonArray({80.0, 80.0, 80.0, 40.0, 40.0, 40.0});

    pinPose.push_back(ui->lineEditPinX_2->text().toDouble());
    pinPose.push_back(ui->lineEditPinY_2->text().toDouble());
    pinPose.push_back(ui->lineEditPinZ_2->text().toDouble());
    pinPose.push_back(ui->lineEditPinRoll_2->text().toDouble());
    pinPose.push_back(ui->lineEditPinPitch_2->text().toDouble());
    pinPose.push_back(ui->lineEditPinYaw_2->text().toDouble());

    json["command"] = ASSEMBLE_TARGET_START;
    json["pinPose"] = pinPose;
    json["pinDepth"] = pinDepth;
    json["targetForce"] = targetForce;
    json["forceLimit"] = forceLimit;
    json["contactForce"] = contactForce;

    json["stiffness"] = stiffness;
    json["zeta"] = zeta;

    sendJson(json);
}

void MainWindow::on_pushButtonAssemblePin_3_clicked() {
    QJsonObject json;

    QJsonArray pinPose;
    double pinDepth = ui->lineEditPinSize_3->text().toDouble();
    QJsonArray targetForce = toJsonArray({0.0, 0.0, 10.0, 0.0, 0.0, 0.0});
    QJsonArray forceLimit = toJsonArray({80.0, 80.0, 80.0, 20.0, 20.0, 20.0});
    QJsonArray contactForce = toJsonArray({20.0, 20.0, 20.0, 5.0, 5.0, 5.0});

    QJsonArray stiffness = toJsonArray({1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0});
    QJsonArray zeta = toJsonArray({80.0, 80.0, 80.0, 40.0, 40.0, 40.0});

    pinPose.push_back(ui->lineEditPinX_3->text().toDouble());
    pinPose.push_back(ui->lineEditPinY_3->text().toDouble());
    pinPose.push_back(ui->lineEditPinZ_3->text().toDouble());
    pinPose.push_back(ui->lineEditPinRoll_3->text().toDouble());
    pinPose.push_back(ui->lineEditPinPitch_3->text().toDouble());
    pinPose.push_back(ui->lineEditPinYaw_3->text().toDouble());

    json["command"] = ASSEMBLE_TARGET_START;
    json["pinPose"] = pinPose;
    json["pinDepth"] = pinDepth;
    json["targetForce"] = targetForce;
    json["forceLimit"] = forceLimit;
    json["contactForce"] = contactForce;

    json["stiffness"] = stiffness;
    json["zeta"] = zeta;

    sendJson(json);
}

void MainWindow::on_pushButtonAssemblePin_4_clicked() {
    QJsonObject json;

    QJsonArray pinPose;
    double pinDepth = ui->lineEditPinSize_4->text().toDouble();
    QJsonArray targetForce = toJsonArray({0.0, 0.0, 10.0, 0.0, 0.0, 0.0});
    QJsonArray forceLimit = toJsonArray({80.0, 80.0, 80.0, 20.0, 20.0, 20.0});
    QJsonArray contactForce = toJsonArray({20.0, 20.0, 20.0, 5.0, 5.0, 5.0});

    QJsonArray stiffness = toJsonArray({1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0});
    QJsonArray zeta = toJsonArray({80.0, 80.0, 80.0, 40.0, 40.0, 40.0});

    pinPose.push_back(ui->lineEditPinX_4->text().toDouble());
    pinPose.push_back(ui->lineEditPinY_4->text().toDouble());
    pinPose.push_back(ui->lineEditPinZ_4->text().toDouble());
    pinPose.push_back(ui->lineEditPinRoll_4->text().toDouble());
    pinPose.push_back(ui->lineEditPinPitch_4->text().toDouble());
    pinPose.push_back(ui->lineEditPinYaw_4->text().toDouble());

    json["command"] = ASSEMBLE_TARGET_START;
    json["pinPose"] = pinPose;
    json["pinDepth"] = pinDepth;
    json["targetForce"] = targetForce;
    json["forceLimit"] = forceLimit;
    json["contactForce"] = contactForce;

    json["stiffness"] = stiffness;
    json["zeta"] = zeta;

    sendJson(json);
}
