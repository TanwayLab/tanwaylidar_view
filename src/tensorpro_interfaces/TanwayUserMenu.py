#-*- coding:utf-8 -*-
#!/usr/bin/env python

#  Copyright (C) 2020 Tanway Technology Co., Ltd
#  License:　BSD 3-Clause License
#
## Version: V1.2
## Author: Elodie Shan
## Editor: LF Shen
## Updated Date: 2019-11-26
## Function: Check connnection state and set IP for Tensor-Pro Lidar

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtGui import QTextCursor

import os
import sys
import time
import socket
import struct
import re
import rospkg
import psutil

reload(sys)
sys.setdefaultencoding( "utf-8" )

#class MyWindow(QWidget):
class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Get path to UI file which should be in the "resource" folder of this package
        self.ui_file = os.path.join(rospkg.RosPack().get_path('tensorpro_view'), 'resource', 'InterfaceGUI.ui')
        loadUi(self.ui_file, self)
        self.setObjectName('ConfigMenu')

        self.Check.clicked.connect(self.check_button) #check button for connection check
        self.SetIP.clicked.connect(self.setIP_button) #SetIP button for IP Set

        self.OriLidarIP = ""
        self.OriLidarport = 0
        self.OriHostIP = ""
        self.OriHostport = 0

        self.NewLidarIP = ""
        self.NewLidarport = 5050
        self.NewHostIP = ""
        self.NewHostport = 5600

        self.check_flag = 0
        self.ip_flag = 0

    def check_button(self):
        if self.check_flag == 0:
            self.check_flag = 1
            self.Check.setText("结束")
            # Put start button code here
            self.get_card_bytes_info()

    def changeCheckButton_status(self):
        self.check_flag = 0
        self.Check.setText("连接雷达")
        self.OperateOut.insertPlainText("\n")

    def setIP_button(self):
        if self.ip_flag == 0:
            self.ip_flag = 1
            self.SetIP.setText("结束")
            # Put start button code here
            self.IPSetting()

    def changeIPButton_status(self):
        self.ip_flag = 0
        self.SetIP.setText("确定修改")
        self.OperateOut.insertPlainText("\n")

    def output_msg(self,text_window,text):
        cursor =  text_window.textCursor()
        cursor.movePosition(QTextCursor.End)
        text_window.setTextCursor(cursor)
        text_window.insertPlainText(text)

    def checkInfoLegality(self,ip,port):
        if re.match(r"(([1-9]?\d|1\d{2}|2[0-4]\d|25[0-5])\.){3}([1-9]?\d|1\d{2}|2[0-4]\d|25[0-5])$", ip): 
            try:
                p = int(port)
            except:
                self.output_msg(self.OperateOut, "端口输入有误，请检查所填信息．\n")
                return -1
            if p < 1 or p > 65535:
                self.output_msg(self.OperateOut,"端口需在0~65535范围内,请更改.\n") 
                return -1
            return 0
        else:
            self.output_msg(self.OperateOut,"IP地址输入有误,请检查所填信息.\n")
            return -1

    def checkIPLegality(self):
        lidarip = self.NewLidarIP.split(".")
        hostip = self.NewHostIP.split(".")
        for i in range(3):
            if lidarip[i] != hostip[i]:
                self.output_msg(self.OperateOut,"雷达和主机需在同一网段内,否则无法通信.\n")
                return -1
        if lidarip[3] == hostip[3]:
            self.output_msg(self.OperateOut,"雷达和主机的IP不可重复,请检查所填信息.\n")
            return -1
        return 0

    def checkMacLegality(self,mac):
        if re.match(r"^\s*([0-9a-fA-F]{2,2}:){5,5}[0-9a-fA-F]{2,2}\s*$", mac):
            return 0    
        else:
            self.output_msg(self.OperateOut,"Mac地址输入有误，请检查所填信息．\n")
            return -1

    def getMacInfo(self):
        self.LiDARMac = str(self.LiDARMac1.text())+":"+str(self.LiDARMac2.text())+":"+str(self.LiDARMac3.text())+":"+str(self.LiDARMac4.text())+":"+str(self.LiDARMac5.text())+":"+str(self.LiDARMac6.text())
        if self.checkMacLegality(self.LiDARMac) == -1:
            return -1
        return 0

    def getOriInfo(self):
        self.OriHostIP = str(self.OriHOSTIP1.text())+"."+ str(self.OriHOSTIP2.text())+"."+str(self.OriHOSTIP3.text())+"."+str(self.OriHOSTIP4.text())
        self.OriHostport = self.OriHOSTPort.text()
        self.OriLidarIP = str(self.OriLiDARIP1.text())+"."+ str(self.OriLiDARIP2.text())+"."+str(self.OriLiDARIP3.text())+"."+str(self.OriLiDARIP4.text())
        self.OriLidarport = self.OriLiDARPort.text()

        if self.OriHostIP == "" or self.OriHostport=="" or self.OriLidarIP == "" or self.OriLidarport=="":
           self.output_msg(self.OperateOut,"请先验证雷达通信情况，并输入完整的IP和端口信息.\n")
           return -1

        if self.checkInfoLegality(self.OriHostIP,self.OriHostport)==-1 or self.checkInfoLegality(self.OriLidarIP,self.OriLidarport)==-1:
           return -1
        return 0

    def getNewInfo(self):
        self.NewHostIP = str(self.NewHOSTIP1.text())+"."+ str(self.NewHOSTIP2.text())+"."+str(self.NewHOSTIP3.text())+"."+str(self.NewHOSTIP4.text())
        self.NewHostport = self.NewHOSTPort.text()
        if self.checkInfoLegality(self.NewHostIP,self.NewHostport)==-1:
            return -1
        self.NewLidarIP = str(self.NewLiDARIP1.text())+"."+ str(self.NewLiDARIP2.text())+"."+str(self.NewLiDARIP3.text())+"."+str(self.NewLiDARIP4.text())
        self.NewLidarport = self.NewLiDARPort.text()
        if self.checkInfoLegality(self.NewLidarIP,self.NewLidarport)==-1:
            return -1
        return 0

    def CheckConnection(self):
        if self.getOriInfo()==-1:
           self.changeCheckButton_status()
           return -1
        self.output_msg(self.OperateOut,"正在尝试接收雷达数据，请稍候...\n")
        udp_socket_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket_recv.settimeout(10)
        ip_port = (self.OriHostIP , int(self.OriHostport))

        try:
            udp_socket_recv.bind(ip_port) 
        except:
            self.output_msg(self.OperateOut,"无法与本地的IP端口绑定，请检查所填信息！\n") 
            udp_socket_recv.close()
            self.changeCheckButton_status()
            return -1
        n = 0
        while n<5:
            try:
                msg, addr = udp_socket_recv.recvfrom(1500)
            except socket.timeout:
                self.output_msg(self.OperateOut,"主机无法接收雷达数据，请检查网络设置！\n")
                udp_socket_recv.close()
                self.changeCheckButton_status()
                return -1    
            time.sleep(0.1)
            n += 1    
        text = "收到来自"+str(addr[0])+":"+str(addr[1])+"的信息,长度为"+str(len(msg))+"．\n"
        self.OperateOut.insertPlainText(text)
        if str(addr[0])!=self.OriLidarIP or str(addr[1])!=self.OriLidarport:
            self.output_msg(self.OperateOut,"数据来源与所填信息不符，请检查信息！\n")
        else:
            self.output_msg(self.OperateOut,"雷达数据传输正常，请继续操作．\n")
        udp_socket_recv.close()
        self.changeCheckButton_status()

    def IPSetting(self):
        if self.getMacInfo()==-1 or self.getOriInfo()==-1 or self.getNewInfo()==-1 or self.checkIPLegality() == -1:
            self.changeIPButton_status()
            return -1
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ip_port = (self.OriHostIP , int(self.OriHostport))
            client_socket.bind(ip_port) 
        except:
            self.output_msg(self.OperateOut,"无法与本地的IP端口绑定，请检查所填信息！\n")
            client_socket.close()
            self.changeIPButton_status()
            return -1

        #add_mac = "gnome-terminal --disable-factory -- /bin/sh -c 'sudo arp -s " + self.OriLidarIP + " " + self.LiDARMac + " && arp -a '"
        add_mac = "gnome-terminal --disable-factory -- /bin/sh -c 'sudo arp -a " + self.OriLidarIP + " -s " + self.OriLidarIP + " " + self.LiDARMac + "'" 
    
        if os.system(add_mac)>0:
            self.changeIPButton_status()
            return -1

        carray = [0xc0,0xa8,0x6f,0x33,0xc1,0xc1,0x13,0xba,0xc0,0xa8,0x6f,0xcc,0xa1,0xa1,0x15,0xe0]
        
        carray[0] = int(str(self.NewLiDARIP1.text()))
        carray[1] = int(str(self.NewLiDARIP2.text()))
        carray[2] = int(str(self.NewLiDARIP3.text()))
        carray[3] = int(str(self.NewLiDARIP4.text()))

        carray[6] = ((int(str(self.NewLidarport)))>>8)
        carray[7] = ((int(str(self.NewLidarport)))&255)

        carray[8] = int(str(self.NewHOSTIP1.text()))
        carray[9] = int(str(self.NewHOSTIP2.text()))
        carray[10] = int(str(self.NewHOSTIP3.text()))
        carray[11] = int(str(self.NewHOSTIP4.text()))

        carray[14] = ((int(str(self.NewHostport)))>>8)
        carray[15] = ((int(str(self.NewHostport)))&255)

        msg = struct.pack("%dB" % (len(carray)), *carray)

        try:
            client_socket.sendto(msg, (self.OriLidarIP , int(self.OriLidarport)))
        except:
            self.output_msg(self.OperateOut,"网络通信出错,请检查设置！\n")                                                   
            client_socket.close()
            self.changeIPButton_status()
            return -1
        time.sleep(1)
        client_socket.close()
        self.output_msg(self.OperateOut,"操作完成!\n")
        self.changeIPButton_status()

    def get_card_bytes_info(self):
        # bytes_sent
        self.output_msg(self.OperateOut,"新的函数！\n") 
        try:
            card_ip = []
            card_io = []
            interface = []
            for k, v in psutil.net_if_addrs().items():
                if k not in 'lo':
                    card_ip.append({'name': k, 'ip':v[0].address})

            for k, v in psutil.net_io_counters(pernic=True).items():
                if k not in 'lo':
                    card_io.append({'name': k, 'out': v.bytes_sent, 'in': v.bytes_recv})

            for i in range(len(card_ip)):

                card = {
                    'intName': card_io[i]['name'],
                    'ip': card_ip[i]['ip'],
                    'out': card_io[i]['out'],
                    'in': card_io[i]['in']
                }
                interface.append(card)
                self.output_msg(self.OperateOut, str(card_io[i]['name']) + card_ip[i]['ip'])
            return 0
        except AttributeError as e:
            print("Please use psutil version 3.0 or above")

