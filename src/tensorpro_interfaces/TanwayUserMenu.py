#-*- coding:utf-8 -*-
#!/usr/bin/env python

#  Copyright (C) 2020 Tanway Technology Co., Ltd
#  License:　BSD 3-Clause License
#
## Version: V1.2
## Author: LN
## Editor: LF Shen
## Updated Date: 2019-11-26
## Function: Check connnection state and set IP for Tensor-Pro Lidar

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QMainWindow,QApplication
from python_qt_binding.QtGui import QTextCursor

import os
import sys
import time
import socket
import struct
import re
import rospkg
import psutil
import ConfigParser

reload(sys)
sys.setdefaultencoding( "utf-8" )

#class MyWindow(QWidget):
class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Get path to UI file which should be in the "resource" folder of this package
        self.ui_file = os.path.join(rospkg.RosPack().get_path('tanwaylidar_view'), 'resource', 'InterfaceGUI.ui')
        loadUi(self.ui_file, self)
        self.setObjectName('ConfigMenu')

        self.Check.clicked.connect(self.check_button) #check button for connection check
        self.SetIP.clicked.connect(self.setIP_button) #SetIP button for IP Set

        #read ini
        if os.path.exists("twcconfig.ini"):
            try:
                config = ConfigParser.ConfigParser()
                config.readfp(open("twcconfig.ini"))
                read_ori_lidarip1 = config.get("ORI","lidar_ip1")
                read_ori_lidarip2 = config.get("ORI","lidar_ip2")
                read_ori_lidarip3 = config.get("ORI","lidar_ip3")
                read_ori_lidarip4 = config.get("ORI","lidar_ip4")
                read_ori_lidarport = config.get("ORI","lidar_port")
                read_ori_hostip1 = config.get("ORI","host_ip1")
                read_ori_hostip2 = config.get("ORI","host_ip2")
                read_ori_hostip3 = config.get("ORI","host_ip3")
                read_ori_hostip4 = config.get("ORI","host_ip4")
                read_ori_hostport = config.get("ORI","host_port")
        
                read_mac1 = config.get("ORI","lidar_mac1")
                read_mac2 = config.get("ORI","lidar_mac2")
                read_mac3 = config.get("ORI","lidar_mac3")
                read_mac4 = config.get("ORI","lidar_mac4")
                read_mac5 = config.get("ORI","lidar_mac5")
                read_mac6 = config.get("ORI","lidar_mac6")
        
                read_new_lidarip1 = config.get("NEW","lidar_ip1")
                read_new_lidarip2 = config.get("NEW","lidar_ip2")
                read_new_lidarip3 = config.get("NEW","lidar_ip3")
                read_new_lidarip4 = config.get("NEW","lidar_ip4")
                read_new_lidarport = config.get("NEW","lidar_port")
                read_new_hostip1 = config.get("NEW","host_ip1")
                read_new_hostip2 = config.get("NEW","host_ip2")
                read_new_hostip3 = config.get("NEW","host_ip3")
                read_new_hostip4 = config.get("NEW","host_ip4")
                read_new_hostport = config.get("NEW","host_port")
        
                self.OriLiDARIP1.setText(read_ori_lidarip1)
                self.OriLiDARIP2.setText(read_ori_lidarip2)
                self.OriLiDARIP3.setText(read_ori_lidarip3)
                self.OriLiDARIP4.setText(read_ori_lidarip4)
                self.OriLiDARPort.setText(read_ori_lidarport)
                
                self.OriHOSTIP1.setText(read_ori_hostip1)
                self.OriHOSTIP2.setText(read_ori_hostip2)
                self.OriHOSTIP3.setText(read_ori_hostip3)
                self.OriHOSTIP4.setText(read_ori_hostip4)
                self.OriHOSTPort.setText(read_ori_hostport)
        
                self.LiDARMac1.setText(read_mac1)
                self.LiDARMac2.setText(read_mac2)
                self.LiDARMac3.setText(read_mac3)
                self.LiDARMac4.setText(read_mac4)
                self.LiDARMac5.setText(read_mac5)
                self.LiDARMac6.setText(read_mac6)
        
                self.NewLiDARIP1.setText(read_new_lidarip1)
                self.NewLiDARIP2.setText(read_new_lidarip2)
                self.NewLiDARIP3.setText(read_new_lidarip3)
                self.NewLiDARIP4.setText(read_new_lidarip4)
                self.NewLiDARPort.setText(read_new_lidarport)
        
                self.NewHOSTIP1.setText(read_new_hostip1)
                self.NewHOSTIP2.setText(read_new_hostip2)
                self.NewHOSTIP3.setText(read_new_hostip3)
                self.NewHOSTIP4.setText(read_new_hostip4)
                self.NewHOSTPort.setText(read_new_hostport)
            except:
                print("Read config file(twcconfig.ini) error.")
            	
        #basic param
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
            self.Check.setText("检测中...")
            # begin connect
            self.CheckConnection()
            # reset button status
            self.check_flag = 0
            self.Check.setText("测试雷达连接")
            self.OperateOut.insertPlainText("\n")

    def setIP_button(self):
        if self.ip_flag == 0:
            self.ip_flag = 1
            self.SetIP.setText("修改中...")
            # begin change
            self.IPSetting()
            # reset button status
            self.ip_flag = 0
            self.SetIP.setText("修改配置")
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
                self.output_msg(self.OperateOut, "【错误】端口输入有误，请检查本机网卡IP和【HOST IP】信息是否一致。\n")
                return -1
            if p < 1 or p > 65535:
                self.output_msg(self.OperateOut,"【错误】端口需在0~65535范围内,请更改。\n") 
                return -1
            return 0
        else:
            self.output_msg(self.OperateOut,"【错误】IP地址输入有误,请检查所填信息。\n")
            return -1

    def checkIPLegality(self):
        lidarip = self.NewLidarIP.split(".")
        hostip = self.NewHostIP.split(".")
        for i in range(3):
            if lidarip[i] != hostip[i]:
                self.output_msg(self.OperateOut,"【错误】雷达和主机需在同一网段内,否则无法通信。\n")
                return -1
        if lidarip[3] == hostip[3]:
            self.output_msg(self.OperateOut,"【错误】雷达和主机的IP不可重复,请检查所填信息。\n")
            return -1
        return 0

    def checkMacLegality(self,mac):
        if re.match(r"^\s*([0-9a-fA-F]{2,2}:){5,5}[0-9a-fA-F]{2,2}\s*$", mac):
            return 0    
        else:
            self.output_msg(self.OperateOut,"【错误】Mac地址输入有误，请检查所填信息。\n")
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
           self.output_msg(self.OperateOut,"【错误】输入的IP地址或端口号不符合规则，请检查所填信息。\n")
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
        self.output_msg(self.OperateOut,"【提示】正在准备连接，请稍候...\n")
        QApplication.processEvents()
        if self.getOriInfo()==-1:
           return -1
        self.output_msg(self.OperateOut,"【提示】正在尝试接收雷达数据，请稍候...\n")
        udp_socket_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket_recv.settimeout(5)
        ip_port = (self.OriHostIP , int(self.OriHostport))

        try:
            udp_socket_recv.bind(ip_port) 
        except:
            self.output_msg(self.OperateOut,"【错误】无法与本地的IP端口绑定，请检查所填信息！\n") 
            udp_socket_recv.close()
            return -1
        n = 0
        while n<5:
            try:
                msg, addr = udp_socket_recv.recvfrom(1500)
            except socket.timeout:
                self.output_msg(self.OperateOut,"【失败】未接收到雷达数据，请检查网络配置是否正确！\n")
                udp_socket_recv.close()
                return -1    
            n += 1    
        text = "【提示】收到来自"+str(addr[0])+":"+str(addr[1])+"的信息,长度为"+str(len(msg))+"。\n"
        self.OperateOut.insertPlainText(text)
        if str(addr[0])!=self.OriLidarIP or str(addr[1])!=self.OriLidarport:
            self.output_msg(self.OperateOut,"【失败】数据来源与所填雷达连接信息不符，请修正雷达连接信息！\n")
        else:
            self.output_msg(self.OperateOut,"【成功】连接雷达成功！雷达数据传输正常，请继续操作。\n")
        udp_socket_recv.close()

    def IPSetting(self):
        self.output_msg(self.OperateOut,"【提示】正在准备修改，请稍候...\n")
        QApplication.processEvents()
        if self.getMacInfo()==-1 or self.getOriInfo()==-1 or self.getNewInfo()==-1 or self.checkIPLegality() == -1:
            return -1

        #write ini
        config = ConfigParser.ConfigParser()
        try:
            config.add_section("ORI")
            config.set("ORI","lidar_ip1", str(self.OriLiDARIP1.text()))
            config.set("ORI","lidar_ip2", str(self.OriLiDARIP2.text()))
            config.set("ORI","lidar_ip3", str(self.OriLiDARIP3.text()))
            config.set("ORI","lidar_ip4", str(self.OriLiDARIP4.text()))
            config.set("ORI","lidar_port", str(self.OriLiDARPort.text()))
            config.set("ORI","host_ip1", str(self.OriHOSTIP1.text()))
            config.set("ORI","host_ip2", str(self.OriHOSTIP2.text()))
            config.set("ORI","host_ip3", str(self.OriHOSTIP3.text()))
            config.set("ORI","host_ip4", str(self.OriHOSTIP4.text()))
            config.set("ORI","host_port", str(self.OriHOSTPort.text()))

            config.set("ORI","lidar_mac1", str(self.LiDARMac1.text()))
            config.set("ORI","lidar_mac2", str(self.LiDARMac2.text()))
            config.set("ORI","lidar_mac3", str(self.LiDARMac3.text()))
            config.set("ORI","lidar_mac4", str(self.LiDARMac4.text()))
            config.set("ORI","lidar_mac5", str(self.LiDARMac5.text()))
            config.set("ORI","lidar_mac6", str(self.LiDARMac6.text()))
        
            config.add_section("NEW")
            config.set("NEW","lidar_ip1", str(self.NewLiDARIP1.text()))
            config.set("NEW","lidar_ip2", str(self.NewLiDARIP2.text()))
            config.set("NEW","lidar_ip3", str(self.NewLiDARIP3.text()))
            config.set("NEW","lidar_ip4", str(self.NewLiDARIP4.text()))
            config.set("NEW","lidar_port", str(self.NewLiDARPort.text()))
            config.set("NEW","host_ip1", str(self.NewHOSTIP1.text()))
            config.set("NEW","host_ip2", str(self.NewHOSTIP2.text()))
            config.set("NEW","host_ip3", str(self.NewHOSTIP3.text()))
            config.set("NEW","host_ip4", str(self.NewHOSTIP4.text()))
            config.set("NEW","host_port", str(self.NewHOSTPort.text()))
            config.write(open("./twcconfig.ini", "w"))
        except:
            print("Failed to save the usage record(twcconfig.ini).")

        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ip_port = (self.OriHostIP , 6677)
            client_socket.bind(ip_port) 
        except:
            self.output_msg(self.OperateOut,"【错误】无法与本地的IP端口绑定，请检查所填信息！\n")
            client_socket.close()
            return -1


        carray = [0x00]*100 

        carray[0] = 0xFF
        carray[1] = 0xEE
        carray[2] = (100>>8)
        carray[3] = (100&255)
        carray[4] = 0x01
        carray[5] = 0x00
        carray[6] = 0x00 #改IP
        carray[7] = 0x01 
        #指定要操作的雷达
        #IP
        carray[8] = int(str(self.OriLiDARIP1.text()))
        carray[9] = int(str(self.OriLiDARIP2.text()))
        carray[10] = int(str(self.OriLiDARIP3.text()))
        carray[11] = int(str(self.OriLiDARIP4.text()))
        #MAC
        carray[12] = int(str(self.LiDARMac1.text()), 16)
        carray[13] = int(str(self.LiDARMac2.text()), 16)
        carray[14] = int(str(self.LiDARMac3.text()), 16)
        carray[15] = int(str(self.LiDARMac4.text()), 16)
        carray[16] = int(str(self.LiDARMac5.text()), 16)
        carray[17] = int(str(self.LiDARMac6.text()), 16)

        #new lidar ip
        carray[18] = int(str(self.NewLiDARIP1.text()))
        carray[19] = int(str(self.NewLiDARIP2.text()))
        carray[20] = int(str(self.NewLiDARIP3.text()))
        carray[21] = int(str(self.NewLiDARIP4.text()))
        #new lidar port
        carray[22] = ((int(str(self.NewLidarport)))>>8)
        carray[23] = ((int(str(self.NewLidarport)))&255)
        #new dest ip
        carray[24] = int(str(self.NewHOSTIP1.text()))
        carray[25] = int(str(self.NewHOSTIP2.text()))
        carray[26] = int(str(self.NewHOSTIP3.text()))
        carray[27] = int(str(self.NewHOSTIP4.text()))
        #new dest port
        carray[28] = ((int(str(self.NewHostport)))>>8)
        carray[29] = ((int(str(self.NewHostport)))&255)

        carray[30] = 0xAA
        carray[31] = 0xBB

        msg = struct.pack("%dB" % (len(carray)), *carray)

        try:
            client_socket_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            client_socket_send.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            client_socket_send.sendto(msg, (str(self.OriHOSTIP1.text())+"."+ str(self.OriHOSTIP2.text())+"."+str(self.OriHOSTIP3.text())+".255" , int(self.OriLidarport)))
        except:
            self.output_msg(self.OperateOut,"【错误】发送数据包时出现错误，请检查网络配置！\n")                                                   
            client_socket_send.close()
            return -1
        #time.sleep(1)
        self.output_msg(self.OperateOut,"【提示】命令已下发完成。\n")
        QApplication.processEvents()   
        #recv udp to check lidar status
        client_socket.settimeout(10)
        time.sleep(1)

        try:
            msg, addr = client_socket.recvfrom(1500)
        except socket.timeout:
            self.output_msg(self.OperateOut,"【失败】主机未接收到雷达反馈信息，请检查雷达MAC地址、网络设置是否正确配置或重试！\n")
            client_socket.close()
            return -1    

        if len(msg) == 100:
            if msg[5].encode('hex') == '00':
                self.output_msg(self.OperateOut,"【成功】雷达修改操作成功，请重新启动雷达以应用最新配置！\n")
            if msg[5].encode('hex') == '01':
                self.output_msg(self.OperateOut,"【失败】雷达修改操作失败。\n")
            if msg[5].encode('hex') == '02':
                self.output_msg(self.OperateOut,"【失败】雷达修改操作超时，请重试！\n")
        else:
            self.output_msg(self.OperateOut,"【提示】未接收到有效的硬件设备的反馈信息，部分雷达型号没有反馈信息，请手动检查设备状态是否修改成功！\n")
            
        client_socket.close()

    def get_card_bytes_info(self):
        # bytes_sent
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

