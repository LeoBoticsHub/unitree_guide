/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"

#include "Gait/WaveGenerator.h"

#include "interface/KeyBoard.h"
#include "interface/IOROS.h"

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// new class inherited from IOROS

class YYROS : public IOROS {
public:
        YYROS(CmdPanel *myCmdPanel);
        ~YYROS();
};
// use another control pannel instead of Keyboard
YYROS::YYROS(CmdPanel *myCmdPanel):IOROS::IOROS() {
        delete cmdPanel;
        cmdPanel = myCmdPanel;
}
// do nothing
YYROS::~YYROS() {}

class YYPanel : public CmdPanel {
public:
        YYPanel();
        ~YYPanel();
private:
        void* run (void *arg);
        static void* runyy(void *arg);
        pthread_t _tid;
        void checkCmdCallback(const std_msgs::Int32 i);
        void changeValueCallback(const geometry_msgs::Twist p);
        void stopRobotCmdVelCallback(const ros::TimerEvent& event);

        // ros specified variable
        // state change listener;
        ros::Subscriber yycmd;
        // velocity change listener;
        ros::Subscriber yyvalue;

        ros::Timer stopRobotCmdVel;
        ros::Time lastCmdVelTime;
};

YYPanel::YYPanel() {
        userCmd = UserCommand::NONE;
        userValue.setZero();
        ros::NodeHandle n;
        // register message callback functions
        yycmd = n.subscribe("set_mode", 1, &YYPanel::checkCmdCallback, this);
        yyvalue = n.subscribe("cmd_vel", 1, &YYPanel::changeValueCallback, this);
        pthread_create(&_tid, NULL, runyy, (void*)this);
        stopRobotCmdVel = n.createTimer(ros::Duration(0.9), &YYPanel::stopRobotCmdVelCallback, this);
        lastCmdVelTime = ros::Time::now();
}

YYPanel::~YYPanel() {
        pthread_cancel(_tid);
        pthread_join(_tid, NULL);
}

void* YYPanel::runyy(void *arg) {
        ((YYPanel*)arg)->run(NULL);
        return NULL;
}

void* YYPanel::run(void *arg) {
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin();
        return NULL;
}

void YYPanel::checkCmdCallback(std_msgs::Int32 i) {
        ROS_INFO("%d", i.data);
        //*
        UserCommand tmp;
        switch (i.data){
        case 1:
                tmp = UserCommand::L2_B;
                break;
        case 2:
                tmp = UserCommand::L2_A;
                break;
        case 3:
                tmp = UserCommand::L2_X;
                break;
        case 4:
                tmp = UserCommand::START;
                break;
#ifdef COMPILE_WITH_MOVE_BASE
        case 5:
                tmp = UserCommand::L2_Y;
                break;
#endif  // COMPILE_WITH_MOVE_BASE
        case 6:
                tmp = UserCommand::L1_X;
                break;
        case 9:
                tmp = UserCommand::L1_A;
                break;
        case 8:
                tmp = UserCommand::L1_Y;
                break;
        case 0:
                userValue.setZero();
                tmp = UserCommand::NONE;
                break;
        default:
                tmp = UserCommand::NONE;
                break;
        }
        userCmd = tmp;
        //*/
}

void YYPanel::changeValueCallback(const geometry_msgs::Twist p)
{
        userValue.lx = p.linear.y;
        userValue.ly = p.linear.x;
        userValue.rx = -p.angular.z;
        
        lastCmdVelTime = ros::Time::now();
}

// Checks wheter the robot needs to be stopped because cmd vel are not received anymore
void YYPanel::stopRobotCmdVelCallback(const ros::TimerEvent& event)
{
        if (ros::Time::now() - lastCmdVelTime > ros::Duration(1))
        {
                userValue.lx = 0;
                userValue.ly = 0;                
                userValue.rx = 0;
        }
}

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
        std::cout << "stop the controller" << std::endl;
        running = false;
}

int main(int argc, char **argv)
{
        /* set the print format */
        std::cout << std::fixed << std::setprecision(3);

        ros::init(argc, argv, "unitree_gazebo_servo");
        ros::NodeHandle n("~");

        IOInterface *ioInter;
        CtrlPlatform ctrlPlat;

        ioInter = new YYROS(new YYPanel());
        ctrlPlat = CtrlPlatform::GAZEBO;

        CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
        ctrlComp->ctrlPlatform = ctrlPlat;
        ctrlComp->dt = 0.001; // run at 1000hz
        ctrlComp->running = &running;

        std::string robot_model;
        ros::param::get("/robot_name", robot_model);

        if (robot_model == "go1")
        {
                ctrlComp->robotModel = new Go1Robot();
                ROS_INFO("Loading Go1 controller");
        }
        else if (robot_model == "b1")
        {
                ctrlComp->robotModel = new B1Robot();
                ROS_INFO("Loading B1 controller");
        }
        else
        {
                ROS_ERROR("Wrong robot model. Please provide a model between go1 and b1.");
                exit(1);
        }
        

        ctrlComp->waveGen = new WaveGenerator(0.8, 0.7, Vec4(0, 0.5, 0.5, 0)); // Trot
        // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
        // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
        //ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
        // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

        ctrlComp->geneObj();

        ControlFrame ctrlFrame(ctrlComp);

        // deal with Ctrl+C
        signal(SIGINT, ShutDown);

        while (running)
        {
                ctrlFrame.run();
        }

        delete ctrlComp;
        return 0;
}
