#include "virtual_table_communicator.h"

VirtualTableCommunicator::VirtualTableCommunicator(QObject * parent){}
VirtualTableCommunicator::~VirtualTableCommunicator(){}

 void VirtualTableCommunicator::send_ball_position(
        const messages::Ball::ConstPtr& msg) const {
    emit send_ball_position_to_gui(msg);
}
 void VirtualTableCommunicator::send_human_stick_position(
        const messages::Stick::ConstPtr& msg) const {
    emit send_human_stick_position_to_gui(msg);
}
 void VirtualTableCommunicator::send_ai_stick_position(
        const messages::Stick::ConstPtr& msg) const {
    emit send_ai_stick_position_to_gui(msg);
}

