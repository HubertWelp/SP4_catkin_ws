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
#include "../src/main_window.hpp"
#include <QHBoxLayout>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pds {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(Koordinator* dlg, QWidget *parent)
    : QDialog(parent)
    , dialog(dlg)
{
//	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    setMinimumWidth(this->size().width());
    setMinimumHeight(this->size().height());
    move(10,10);
//    setStyleSheet("background-color:white;");


    QPixmap pxm;
    QPalette sample_palette;
    QFont font;
    QColor thgaBlue(0,61,125);
    QColor thgaRed(228,0,30);

    //
    buttonImages[0]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/maoam.png");
    buttonImages[1]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/snicker.png");
    buttonImages[2]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/mw3.png");
    buttonImages[3]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/schokoriegel.jpg");
    buttonImages[4]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/entnahme.png");
    buttonImages[5]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/ausfuehrend.png");
    buttonImages[6]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/nichtgefunden.png");
    buttonImages[7]=QPixmap("/home/student/catkin_ws/src/SP4Koordinator/resources/images/error.png");

    buttonIcon[0]=QIcon(buttonImages[0]);
    buttonIcon[1]=QIcon(buttonImages[1]);
    buttonIcon[2]=QIcon(buttonImages[2]);
    buttonIcon[3]=QIcon(buttonImages[3]);


    logoLabel = new QLabel(this);
    logoLabel->move(125,50);
    logoLabel->setFixedSize(250,100);
    logoLabel->setMaximumHeight(100);
    logoLabel->setAlignment(Qt::AlignCenter);
    logoLabel->setScaledContents(true);
    std::cout << "logoLabel: " << logoLabel->width() << " " << logoLabel->height() << std::endl;
//    if(!pxm.load("/home/Student/thga.jpg"))
    if(!pxm.load("/home/student/catkin_ws/src/SP4Koordinator/resources/images/thga.jpg"))
        std::cout << "Bild konnte nicht geladen werden\n";
    else
    {
        logoLabel->setPixmap(pxm);
    }
    //?
    nameLabel = new QLabel(this);
    font = nameLabel->font();
    font.setPointSize(30);
    font.setBold(false);
    nameLabel->setFont(font);
    sample_palette.setColor(QPalette::Window, Qt::white);
    sample_palette.setColor(QPalette::WindowText, thgaBlue);
    nameLabel->setAutoFillBackground(true);
    nameLabel->setFixedHeight(100);
    nameLabel->setMaximumHeight(100);
    nameLabel->setPalette(sample_palette);
    nameLabel->setAlignment(Qt::AlignCenter);
    nameLabel->setText("Guten Tag\nHerr Mustermann");

    QHBoxLayout* greetingLayout = new QHBoxLayout;
    greetingLayout->setSpacing(0);
    greetingLayout->addWidget(nameLabel);
    greetingLayout->addWidget(logoLabel);


    //?
    hinweisLabel = new QLabel(this);
    font = hinweisLabel->font();
    font.setPointSize(40);
    font.setBold(true);
    hinweisLabel->setFont(font);
    sample_palette.setColor(QPalette::Window, Qt::white);
    sample_palette.setColor(QPalette::WindowText, thgaRed);
    hinweisLabel->setAutoFillBackground(false);
    hinweisLabel->setPalette(sample_palette);
    hinweisLabel->setAlignment(Qt::AlignCenter);
    hinweisLabel->setMaximumHeight(100);
    hinweisLabel->setText("Wohin darf ich Sie begleiten?");
    //?
    for(int i=0;i<4;i++)
    {
        sorten[i] = new QPushButton(this);
        sorten[i]->show();
        sorten[i]->setIcon(buttonIcon[i]);
        sorten[i]->setMinimumHeight(50);
        sorten[i]->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
        sorten[i]->setIconSize(QSize(300,200));
        //sorten[i]->setMaximumHeight(500);
        /*font = sorten[i]->font();
        font.setPointSize(18);
        font.setBold(false);
        sorten[i]->setFont(font);*/
    }




    /*abbrechenButton = new QPushButton(this);
    abbrechenButton->setMinimumHeight(minButtonHeight);
    font = abbrechenButton->font();
    font.setPointSize(18);
    font.setBold(false);
    abbrechenButton->setFont(font);
    abbrechenButton->setText("abbrechen");*/




    QGridLayout* buttonLayout = new QGridLayout;
    buttonLayout->addWidget(sorten[0],0,0);
    buttonLayout->addWidget(sorten[1],0,1);
    buttonLayout->addWidget(sorten[2],1,0);
    buttonLayout->addWidget(sorten[3],1,1);
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addLayout(greetingLayout);
//    mainLayout->setMargin(30);
//    std::cout << "margin: " << mainLayout->margin() << std::endl;

    zustandLabel = new QLabel(this);
    //zustandLabel->move((width/2),height()-400);
    //zustandLabel->setFixedSize(width()-20,height()-200);
    zustandLabel->setMinimumSize(200,150);
    zustandLabel->setMaximumSize(800,600);
    zustandLabel->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
    zustandLabel->setAlignment(Qt::AlignCenter);
    zustandLabel->setScaledContents(false);
    QHBoxLayout* zustandsbilder = new QHBoxLayout;
    zustandsbilder->addWidget(zustandLabel);


    //    mainLayout->addStretch();
    mainLayout->addWidget(hinweisLabel);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addLayout(zustandsbilder);
    mainLayout->addStretch();

    setLayout(mainLayout);




    connect(dialog,SIGNAL(changed()),this,SLOT(aktualisiere()));

    connect(sorten[0],SIGNAL(clicked()),this,SLOT(onZiel1Button()));
    connect(sorten[1],SIGNAL(clicked()),this,SLOT(onZiel2Button()));
    connect(sorten[2],SIGNAL(clicked()),this,SLOT(onZiel3Button()));
    connect(sorten[3],SIGNAL(clicked()),this,SLOT(onZiel4Button()));


    timer = new QTimer(this);
    timer->setSingleShot(true);
    connect(timer,SIGNAL(timeout()),this,SLOT(onTimer()));
}

MainWindow::~MainWindow() {
    for(int i=0;i<4;i++)
    {
        delete sorten[i];
    }
    delete nameLabel;

    delete zustandLabel;
    delete hinweisLabel;
    delete logoLabel;
    delete timer;

}

void MainWindow::aktualisiere() {
    Dialogzustand* dz = dialog->getDialogzustand();
    QString current_state;

    if(dynamic_cast<Wartend*>(dz)) {
        current_state = "WAITING_FOR_SELECTION";
        handleWartend(dynamic_cast<Wartend*>(dz));
    }
    else if(Suchend* state = dynamic_cast<Suchend*>(dz)) {
        current_state = QString("SEARCHING_FOR_%1").arg(QString::fromLatin1(suessStr[state->getSuessigkeit()]));
        handleSuchend(state);
    }
    else if(dynamic_cast<Ausfuehrend*>(dz)) {
        current_state = "EXECUTING_PICKUP";
        handleAusfuehrend(dynamic_cast<Ausfuehrend*>(dz));
    }
    else if(Verabschiedend* state = dynamic_cast<Verabschiedend*>(dz)) {
        switch(state->getAusloeser()) {
            case 0: current_state = "OBJECT_NOT_FOUND"; break;
            case 1: current_state = "REQUEST_TIMEOUT"; break;
            case 2: current_state = "OBJECT_GIVEN"; break;
            default: current_state = "REQUEST_TIMEOUT";
        }
        handleVerabschiedend(state);
    }
    else {
        qWarning() << "Unknown dialog state";
        return;
    }

    // Emit the state changed signal
    Q_EMIT stateChanged(current_state);
}


void MainWindow::onZiel1Button()
{
    dialog->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::Maoam);
}

void MainWindow::onZiel2Button()
{
    dialog->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::Snickers);
}

void MainWindow::onZiel3Button()
{
    dialog->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::MilkyWay);
}

void MainWindow::onZiel4Button()
{
    dialog->objektAuswaehlen(Bildanalysator_Proxy::ObjektTyp::Schokoriegel);
}

void MainWindow::onTimer()
{
    dialog->onTimer();
}


void MainWindow::handleWartend(Wartend *dz)
{
    hinweisLabel->setMaximumHeight(100);
    nameLabel->setText("");
    hinweisLabel->setText("Waehlen Sie!");

    /*begruessungsText = "Guten Tag";
    nameLabel->setText(begruessungsText);
    hinweisLabel->setText("\nBitten halten Sie das Patientenarmband in den Lesebereich.");*/
    for(int i=0;i<4;i++)
    {

        sorten[i]->show();
    }

    zustandLabel->hide();

}

void MainWindow::handleSuchend(Suchend *dz)
{
    nameLabel->setText("");
    hinweisLabel->setText("Einen Moment bitte.");

    for(int i=0;i<4;i++)
    {

        sorten[i]->hide();
    }
    aktuelleSuessigkeit = dz->getSuessigkeit();
    zustandLabel->setPixmap(buttonImages[dz->getSuessigkeit()]);
    //zustandLabel->setAlignment(Qt::AlignCenter);
    if(zustandLabel->width()>buttonImages[dz->getSuessigkeit()].width())
    {
        zustandLabel->setPixmap(buttonImages[dz->getSuessigkeit()].scaled(zustandLabel->width(),zustandLabel->height(),Qt::KeepAspectRatio));
    }
    zustandLabel->show();
}


void MainWindow::handleVerabschiedend(Verabschiedend *dz)
{
    QString meldung;
    int e = dz->getAusloeser();
    if(e==0)
    {
        meldung = QString(suessStr[aktuelleSuessigkeit]) +"\nkonnte nicht gefunden\n werden";
        zustandLabel->setPixmap(buttonImages[6]);
        zustandLabel->setPixmap(buttonImages[6].scaled(zustandLabel->width(),zustandLabel->height(),Qt::KeepAspectRatio));
    }
    else if(e==1)
    {
        meldung = "Dienst nicht verfuegbar";
        zustandLabel->setPixmap(buttonImages[7]);
    }
    else if(e==2)
    {
        meldung = "Entnehmen Sie!";
        zustandLabel->setPixmap(buttonImages[4]);
    }
    hinweisLabel->setMaximumHeight(300);
    nameLabel->setText("");
    hinweisLabel->setText(meldung);
    for(int i=0;i<4;i++)
    {

        sorten[i]->hide();
    }
    zustandLabel->show();
    timer->stop();
    timer->start(5000);

}

void MainWindow::handleAusfuehrend(Ausfuehrend *dz)
{
    std::cout<<"\nIch rede mit dem Roboter\n";
    timer->stop();
    timer->start(90000);
    nameLabel->setText("");
    hinweisLabel->setText("Einen Moment bitte.");

    for(int i=0;i<4;i++)
    {

        sorten[i]->hide();
    }
    zustandLabel->setPixmap(buttonImages[5]);
   //zustandLabel->setAlignment(Qt::AlignCenter);
    zustandLabel->show();
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

}  // namespace pds

