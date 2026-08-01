#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QIcon icoFwd(":/images/btnUp");
    QIcon icoBwd(":/images/btnDown");
    QIcon icoLeft(":/images/btnLeft");
    QIcon icoRight(":/images/btnRight");
    QIcon icoTurnLeft(":/images/btnTurnLeft");
    QIcon icoTurnRight(":/images/btnTurnRight");
    ui->btnFwd->setIcon(icoFwd);
    ui->btnBwd->setIcon(icoBwd);
    ui->btnTurnLeft->setIcon(icoTurnLeft);
    ui->btnTurnRight->setIcon(icoTurnRight);
    //ui->plainTextEdit->setPlainText("hello");
    this->commNode = new RclComm();
    QObject::connect(ui->btnFwd, SIGNAL(pressed()), this, SLOT(btnFwdPressed()));
    QObject::connect(ui->btnFwd, SIGNAL(released()), this, SLOT(btnFwdReleased()));
    QObject::connect(ui->btnBwd, SIGNAL(pressed()), this, SLOT(btnBwdPressed()));
    QObject::connect(ui->btnBwd, SIGNAL(released()), this, SLOT(btnBwdReleased()));
    QObject::connect(ui->btnTurnLeft, SIGNAL(pressed()), this, SLOT(btnTurnLeftPressed()));
    QObject::connect(ui->btnTurnLeft, SIGNAL(released()), this, SLOT(btnTurnLeftReleased()));
    QObject::connect(ui->btnTurnRight, SIGNAL(pressed()), this, SLOT(btnTurnRightPressed()));
    QObject::connect(ui->btnTurnRight, SIGNAL(released()), this, SLOT(btnTurnRightReleased()));
    
    QObject::connect(ui->navTxtStartPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navTxtGoalPose, SIGNAL(returnPressed()), this, SLOT(navBtnCalcPath_pressed()));
    QObject::connect(ui->navBtnCalcPath, SIGNAL(clicked()), this, SLOT(navBtnCalcPath_pressed()));

    QObject::connect(ui->armTxtAngles1, SIGNAL(valueChanged(double)), this, SLOT(armSbAnglesValueChanged(double)));
    QObject::connect(ui->armTxtAngles2, SIGNAL(valueChanged(double)), this, SLOT(armSbAnglesValueChanged(double)));
    QObject::connect(ui->armTxtAngles3, SIGNAL(valueChanged(double)), this, SLOT(armSbAnglesValueChanged(double)));
    QObject::connect(ui->armTxtAngles4, SIGNAL(valueChanged(double)), this, SLOT(armSbAnglesValueChanged(double)));
    QObject::connect(ui->armTxtAngles5, SIGNAL(valueChanged(double)), this, SLOT(armSbAnglesValueChanged(double)));
    QObject::connect(ui->armTxtAngles6, SIGNAL(valueChanged(double)), this, SLOT(armSbAnglesValueChanged(double)));
    QObject::connect(ui->armTxtArticularGoal, SIGNAL(returnPressed()), this, SLOT(armTxtArticularGoalReturnPressed()));
    QObject::connect(ui->armBtnXp, SIGNAL(pressed()), this, SLOT(armBtnXpPressed()));
    QObject::connect(ui->armBtnXm, SIGNAL(pressed()), this, SLOT(armBtnXmPressed()));
    QObject::connect(ui->armBtnYp, SIGNAL(pressed()), this, SLOT(armBtnYpPressed()));
    QObject::connect(ui->armBtnYm, SIGNAL(pressed()), this, SLOT(armBtnYmPressed()));
    QObject::connect(ui->armBtnZp, SIGNAL(pressed()), this, SLOT(armBtnZpPressed()));
    QObject::connect(ui->armBtnZm, SIGNAL(pressed()), this, SLOT(armBtnZmPressed()));
    QObject::connect(ui->armBtnRollp, SIGNAL(pressed()), this, SLOT(armBtnRollpPressed()));
    QObject::connect(ui->armBtnRollm, SIGNAL(pressed()), this, SLOT(armBtnRollmPressed()));
    QObject::connect(ui->armBtnPitchp, SIGNAL(pressed()), this, SLOT(armBtnPitchpPressed()));
    QObject::connect(ui->armBtnPitchm, SIGNAL(pressed()), this, SLOT(armBtnPitchmPressed()));
    QObject::connect(ui->armBtnYawp, SIGNAL(pressed()), this, SLOT(armBtnYawpPressed()));
    QObject::connect(ui->armBtnYawm, SIGNAL(pressed()), this, SLOT(armBtnYawmPressed()));
    QObject::connect(ui->armBtnHome, SIGNAL(pressed()), this, SLOT(armBtnHomePressed()));
    QObject::connect(ui->armBtnNavigate, SIGNAL(pressed()), SLOT(armBtnNavigatePressed()));
    QObject::connect(ui->armTxtCartesianGoal, SIGNAL(returnPressed()), this, SLOT(armTxtCartesianGoalReturnPressed()));

    ros_timer = new QTimer(this);
    QObject::connect(ros_timer, &QTimer::timeout, this, &MainWindow::processRosMessages);
    ros_timer->start(20);
    this->updating_arm_q_controls = false;
    this->timer_counter = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::btnFwdPressed()
{
    //commNode->start_publishing_cmd_vel(0.3, 0, 0);
    commNode->publish_cmd_vel(0.3, 0, 0);
}

void MainWindow::btnFwdReleased()
{
    //commNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0.0, 0, 0);
}

void MainWindow::btnBwdPressed()
{
    //qtRosNode->start_publishing_cmd_vel(-0.3, 0, 0);
    commNode->publish_cmd_vel(-0.3, 0, 0);
}

void MainWindow::btnBwdReleased()
{
    //qtRosNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0, 0, 0);
}

void MainWindow::btnTurnLeftPressed()
{
    //qtRosNode->start_publishing_cmd_vel(0, 0, 0.5);
    commNode->publish_cmd_vel(0, 0, 0.5);
}

void MainWindow::btnTurnLeftReleased()
{
    //qtRosNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0, 0, 0);
}

void MainWindow::btnTurnRightPressed()
{
    //qtRosNode->start_publishing_cmd_vel(0, 0, -0.5);
    commNode->publish_cmd_vel(0, 0, -0.5);
}

void MainWindow::btnTurnRightReleased()
{
    //qtRosNode->stop_publishing_cmd_vel();
    commNode->publish_cmd_vel(0, 0, 0);
}


void MainWindow::navBtnCalcPath_pressed()
{
    float startX = 0;
    float startY = 0;
    float goalX = 0;
    float goalY = 0;
    std::vector<std::string> parts;
    
    std::string str = this->ui->navTxtStartPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    
    // if(str.compare("") == 0 || str.compare("robot") == 0) //take robot pose as start position
    // {
    //     this->ui->navTxtStartPose->setText("Robot");
    //     commNode->get_robot_pose(startX, startY, startA);
    // }
    // else
    if(parts.size() >= 2) //Given data correspond to numbers
    {
        std::stringstream ssStartX(parts[0]);
        std::stringstream ssStartY(parts[1]);
        if(!(ssStartX >> startX) || !(ssStartY >> startY))
        {
            this->ui->navTxtStartPose->setText("Invalid format");
            return;
        }
    }
    else
    {
	this->ui->navTxtStartPose->setText("Invalid format");
	return;
    }

    str = this->ui->navTxtGoalPose->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);
    if(parts.size() >= 2)
    {
        std::stringstream ssGoalX(parts[0]);
        std::stringstream ssGoalY(parts[1]);
        if(!(ssGoalX >> goalX) || !(ssGoalY >> goalY))
        {
            this->ui->navTxtGoalPose->setText("Invalid format");
            return;
        }
    }
    else
    {
	this->ui->navTxtGoalPose->setText("Invalid format");
	return;
    }
    nav_msgs::msg::Path path, smooth_path;
    if(!commNode->call_plan_path(startX, startY, goalX, goalY, path))
        return;
    //commNode->call_smooth_path(path, smooth_path);
}

void MainWindow::navBtnExecPath_pressed()
{
}

void MainWindow::update_arm_q_controls(std::vector<double>& Q){
    this->updating_arm_q_controls = true;
    QString s = "";
    for(int i=0; i<5; i++)
	s += QString::number(Q[i], 'f',2) + " ";
    s += QString::number(Q[5], 'f',2);
    ui->armTxtArticularGoal->setText(s);
    ui->armTxtAngles1->setValue(Q[0]);
    ui->armTxtAngles2->setValue(Q[1]);
    ui->armTxtAngles3->setValue(Q[2]);
    ui->armTxtAngles4->setValue(Q[3]);
    ui->armTxtAngles5->setValue(Q[4]);
    ui->armTxtAngles6->setValue(Q[5]);
    this->updating_arm_q_controls = false;
}

void MainWindow::armSbAnglesValueChanged(double d)
{
    if(this->updating_arm_q_controls)
	return;
    
    std::vector<double> Q(6);
    Q[0] = ui->armTxtAngles1->value();
    Q[1] = ui->armTxtAngles2->value();
    Q[2] = ui->armTxtAngles3->value();
    Q[3] = ui->armTxtAngles4->value();
    Q[4] = ui->armTxtAngles5->value();
    Q[5] = ui->armTxtAngles6->value();
    this->commNode->publish_arm_joint_traj(Q);
    this->update_arm_q_controls(Q);
}

void MainWindow::armSbGripperValueChanged(double d)
{
    if(this->updating_arm_q_controls)
	return;
}

void MainWindow::armTxtArticularGoalReturnPressed()
{
    if(this->updating_arm_q_controls)
	return;
    
    std::vector<double> Q(6);
    std::vector<std::string> parts;
    
    std::string str = this->ui->armTxtArticularGoal->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);

    if(parts.size() == 6) //Given data correspond to numbers
    {
      for(int i=0; i < 6; i++)
	{
	  std::stringstream ss(parts[i]);
	  if(!(ss >> Q[i]))
	    {
	      this->ui->armTxtArticularGoal->setText("Invalid format");
	      return;
	    }
	}
    }
    else
    {
	this->ui->armTxtArticularGoal->setText("Invalid format");
	return;
    }
    this->commNode->publish_arm_joint_traj(Q);
    this->update_arm_q_controls(Q);
}

void MainWindow::armTxtCartesianGoalReturnPressed()
{
    std::vector<double> X(6);
    std::vector<double> Q(6);
    std::vector<std::string> parts;
    
    std::string str = this->ui->armTxtCartesianGoal->text().toStdString();
    boost::algorithm::to_lower(str);
    boost::split(parts, str, boost::is_any_of(" ,\t\r\n"), boost::token_compress_on);

    if(parts.size() == 6) //Given data correspond to numbers
    {
      for(int i=0; i < 6; i++)
	{
	  std::stringstream ss(parts[i]);
	  if(!(ss >> X[i]))
	    {
	      this->ui->armTxtCartesianGoal->setText("Invalid format (x, y, z, R, P, Y)");
	      return;
	    }
	}
    }
    else
    {
	this->ui->armTxtCartesianGoal->setText("Invalid format (x, y, z, R, P, Y)");
	return;
    }
    if(this->commNode->call_ik_pose2pose(X[0], X[1], X[2], X[3], X[4], X[5], Q)){
	this->commNode->publish_arm_joint_traj(Q);
	this->update_arm_q_controls(Q);
    }
}

void MainWindow::change_cartesian(std::vector<double> delta_X){
    double x, y, z, roll, pitch, yaw;
    std::vector<double> Q(6);
    if(!this->commNode->call_fwd_kinematics(this->commNode->current_arm_joints, x, y, z, roll, pitch, yaw))
	return;
    x += delta_X[0];
    y += delta_X[1];
    z += delta_X[2];
    roll += delta_X[3];
    pitch += delta_X[4];
    yaw += delta_X[5];
    if(this->commNode->call_ik_pose2pose(x, y, z, roll, pitch, yaw, Q)){
	this->commNode->publish_arm_joint_traj(Q);
	this->update_arm_q_controls(Q);
	QString s = QString::number(x,'f',2) + " " + QString::number(y,'f',2) + " " + QString::number(z,'f',2) + " ";
	s += QString::number(roll,'f',2) + " " + QString::number(pitch,'f',2) + " " + QString::number(yaw,'f',2);
	this->ui->armTxtCartesianGoal->setText(s);
    }
}

void MainWindow::armBtnXpPressed()
{
    std::vector<double> delta_X = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnXmPressed()
{
    std::vector<double> delta_X = {-0.05, 0.0, 0.0, 0.0, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnYpPressed()
{
    std::vector<double> delta_X = {0.0, 0.05, 0.0, 0.0, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnYmPressed()
{
    std::vector<double> delta_X = {0.0, -0.05, 0.0, 0.0, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnZpPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.05, 0.0, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnZmPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, -0.05, 0.0, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnRollpPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.0, 0.1, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnRollmPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.0, -0.1, 0.0, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnPitchpPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.0, 0.0, 0.1, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnPitchmPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.0, 0.0, -0.1, 0.0};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnYawpPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.0, 0.0, 0.0, 0.1};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnYawmPressed()
{
    std::vector<double> delta_X = {0.0, 0.0, 0.0, 0.0, 0.0, -0.1};
    this->change_cartesian(delta_X);
}

void MainWindow::armBtnHomePressed(){
    std::vector<double> Q = {0,0,0,0,0,0};
    this->commNode->publish_arm_joint_traj(Q);
}

void MainWindow::armBtnNavigatePressed(){
    std::vector<double> Q = {0.00, -1.66, -0.72, 0.00, 1.17, 0.00};
    this->commNode->publish_arm_joint_traj(Q);
}

void MainWindow::arm_get_IK_and_update_ui(std::vector<double> cartesian)
{
}


void MainWindow::spgTxtSayReturnPressed()
{
}

void MainWindow::sprTxtFakeRecogReturnPressed()
{
}

void MainWindow::processRosMessages() {
  this->commNode->spin_some();
  this->ui->armLblCurrentQ1->setText(QString::number(this->commNode->current_arm_joints[0], 'f',3));
  this->ui->armLblCurrentQ2->setText(QString::number(this->commNode->current_arm_joints[1], 'f',3));
  this->ui->armLblCurrentQ3->setText(QString::number(this->commNode->current_arm_joints[2], 'f',3));
  this->ui->armLblCurrentQ4->setText(QString::number(this->commNode->current_arm_joints[3], 'f',3));
  this->ui->armLblCurrentQ5->setText(QString::number(this->commNode->current_arm_joints[4], 'f',3));
  this->ui->armLblCurrentQ6->setText(QString::number(this->commNode->current_arm_joints[5], 'f',3));

  this->timer_counter = (this->timer_counter + 1)%20;
  if(this->timer_counter == 0){
      double x, y, z, roll, pitch, yaw;
      if(this->commNode->call_fwd_kinematics(this->commNode->current_arm_joints, x, y, z, roll, pitch, yaw)){
	  this->ui->armLblCurrentX->setText(QString::number(x, 'f', 3));
	  this->ui->armLblCurrentY->setText(QString::number(y, 'f', 3));
	  this->ui->armLblCurrentZ->setText(QString::number(z, 'f', 3));
	  this->ui->armLblCurrentRoll->setText(QString::number(roll, 'f', 3));
	  this->ui->armLblCurrentPitch->setText(QString::number(pitch, 'f', 3));
	  this->ui->armLblCurrentYaw->setText(QString::number(yaw, 'f', 3));
      }else{
	  this->ui->armLblCurrentX->setText("NaN");
	  this->ui->armLblCurrentY->setText("NaN");
	  this->ui->armLblCurrentZ->setText("NaN");
	  this->ui->armLblCurrentRoll->setText("NaN");
	  this->ui->armLblCurrentPitch->setText("NaN");
	  this->ui->armLblCurrentYaw->setText("NaN");
      }
  }
  if(!rclcpp::ok())
    QApplication::quit();
}
