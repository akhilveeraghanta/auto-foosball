#include "virtual_table_communicator.h"

VirtualTableCommunicator::VirtualTableCommunicator(QObject * parent){}
VirtualTableCommunicator::~VirtualTableCommunicator(){}

void VirtualTableCommunicator::set_ball(const messages::Ball::ConstPtr& msg) {
    ball = *msg;
    emit refresh_gui();
}

void VirtualTableCommunicator::set_human_stick(const messages::Stick::ConstPtr& msg) {
    human_stick = *msg;
    emit refresh_gui();
}

void VirtualTableCommunicator::set_ai_stick(const messages::Stick::ConstPtr& msg){
    ai_stick = *msg;
    emit refresh_gui();
}

const messages::Ball& VirtualTableCommunicator::get_ball(){
    return ball;
}

const messages::Stick& VirtualTableCommunicator::get_human_stick(){
    return human_stick;
}

const messages::Stick& VirtualTableCommunicator::get_ai_stick(){
    return ai_stick;
}

