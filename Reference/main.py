#!/usr/bin/env python3
# encoding: utf-8
import sys
import rospy
from ui import Ui_Form
from std_msgs.msg import String
from PyQt5 import QtGui, QtWidgets
from ainex_interfaces.msg import WalkingParam
from ainex_interfaces.srv import SetWalkingParam, GetWalkingParam, SetWalkingCommand
import threading

ROS_NODE_NAME = 'walking_controller'
class MainWindow(QtWidgets.QWidget, Ui_Form):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        self.pushButton_apply.pressed.connect(lambda: self.button_clicked('apply'))
        self.pushButton_start.pressed.connect(lambda: self.button_clicked('start'))
        self.pushButton_stop.pressed.connect(lambda: self.button_clicked('stop'))
        
        self.param_pub = rospy.Publisher('/walking/set_param', WalkingParam, queue_size=1)
        rospy.Service('/walking_module/update_param', GetWalkingParam, self.update_param)
        rospy.wait_for_service('/walking/get_param')
        
        res = rospy.ServiceProxy('/walking/get_param', GetWalkingParam)()
        self.update_param(res)

    def button_clicked(self, name):
        if name == 'start':
	        threading.Thread(target=self.call_service, args=('/walking/command', SetWalkingCommand, 'start')).start()
            #rospy.ServiceProxy('/walking/command', SetWalkingCommand)('start')
        elif name == 'stop':
	        threading.Thread(target=self.call_service, args=('/walking/command', SetWalkingCommand, 'stop')).start()
            #rospy.ServiceProxy('/walking/command', SetWalkingCommand)('stop')
        elif name == 'apply':
            param = WalkingParam()
            param.init_x_offset = float(self.lineEdit_init_x_offset.text())
            param.init_y_offset = float(self.lineEdit_init_y_offset.text())
            param.init_z_offset = float(self.lineEdit_init_z_offset.text())
            param.init_roll_offset = float(self.lineEdit_init_roll_offset.text())
            param.init_pitch_offset = float(self.lineEdit_init_pitch_offset.text())
            param.init_yaw_offset = float(self.lineEdit_init_yaw_offset.text())
            param.period_time = float(self.lineEdit_period_time.text())
            param.dsp_ratio = float(self.lineEdit_dsp_ratio.text())

            param.step_fb_ratio = float(self.lineEdit_step_fb_ratio.text())
            param.x_move_amplitude = float(self.lineEdit_x_move_amplitude.text())
            param.y_move_amplitude = float(self.lineEdit_y_move_amplitude.text())
            param.z_move_amplitude = float(self.lineEdit_z_move_amplitude.text())
            param.angle_move_amplitude = float(self.lineEdit_angle_move_amplitude.text())
            param.move_aim_on = False#(self.lineEdit_move_aim_on.text())
            param.arm_swing_gain = float(self.lineEdit_arm_swing_gain.text())
            param.y_swap_amplitude = float(self.lineEdit_y_swap_amplitude.text())
            param.z_swap_amplitude = float(self.lineEdit_z_swap_amplitude.text())
            param.pelvis_offset = float(self.lineEdit_pelvis_offset.text())
            param.hip_pitch_offset = float(self.lineEdit_hip_pitch_offset.text())
            
            self.param_pub.publish(param)

    def call_service(self, service, service_type, argument):
        rospy.ServiceProxy(service, service_type)(argument)

    def update_param(self, msg):
        self.lineEdit_init_x_offset.setText(str(round(msg.parameters.init_x_offset, 4)))
        self.lineEdit_init_y_offset.setText(str(round(msg.parameters.init_y_offset, 4)))
        self.lineEdit_init_z_offset.setText(str(round(msg.parameters.init_z_offset, 4)))
        self.lineEdit_init_roll_offset.setText(str(round(msg.parameters.init_roll_offset, 4)))
        self.lineEdit_init_pitch_offset.setText(str(round(msg.parameters.init_pitch_offset, 4)))
        self.lineEdit_init_yaw_offset.setText(str(round(msg.parameters.init_yaw_offset, 4)))
        self.lineEdit_period_time.setText(str(round(msg.parameters.period_time, 4)))
        self.lineEdit_dsp_ratio.setText(str(round(msg.parameters.dsp_ratio, 4)))
        self.lineEdit_step_fb_ratio.setText(str(round(msg.parameters.step_fb_ratio, 4)))
        self.lineEdit_x_move_amplitude.setText(str(round(msg.parameters.x_move_amplitude, 4)))
        self.lineEdit_y_move_amplitude.setText(str(round(msg.parameters.y_move_amplitude, 4)))
        self.lineEdit_z_move_amplitude.setText(str(round(msg.parameters.z_move_amplitude, 4)))

        self.lineEdit_angle_move_amplitude.setText(str(round(msg.parameters.angle_move_amplitude, 4)))
        self.lineEdit_arm_swing_gain.setText(str(round(msg.parameters.arm_swing_gain, 4)))
        self.lineEdit_y_swap_amplitude.setText(str(round(msg.parameters.y_swap_amplitude, 4)))
        self.lineEdit_z_swap_amplitude.setText(str(round(msg.parameters.z_swap_amplitude, 4)))
        self.lineEdit_pelvis_offset.setText(str(round(msg.parameters.pelvis_offset, 4)))
        self.lineEdit_hip_pitch_offset.setText(str(round(msg.parameters.hip_pitch_offset, 4)))

if __name__ == "__main__":  
    app = QtWidgets.QApplication(sys.argv)
    rospy.init_node('%s_node'%ROS_NODE_NAME, log_level=rospy.INFO)
    myshow = MainWindow()
    myshow.show()
    sys.exit(app.exec_())
