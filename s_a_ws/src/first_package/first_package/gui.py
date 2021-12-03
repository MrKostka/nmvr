import sys
import logging
import typing
import threading
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from PyQt5 import QtWidgets, QtCore, QtGui, Qt
from PyQt5.QtCore import Qt as little_qt

logging.basicConfig(format='%(asctime)s.%(msecs)03d - %(levelname)s \t %(message)s', level=logging.INFO, datefmt='%I:%M:%S:%m')

class GridMap(QtWidgets.QWidget):
    def __init__(
        self, parent: typing.Optional[QtWidgets.QWidget] = None, row_count=30, column_count=20, tile_size=30,top_left_x=10,top_left_y=10) -> None:
        super().__init__(parent=parent)

        self.setGeometry(top_left_x, top_left_y, tile_size * row_count + top_left_x, tile_size * column_count + top_left_y)
                
        self.row_count = row_count
        self.column_count = column_count
        self.tile_size = tile_size

        self.qrectfs = self.__createQRectFList(tile_size, row_count, column_count)
        self.walls = []
        self.robot_position = (0,0)   
        self.simulator = parent

    def paintEvent(self, event) -> None:
        painter = QtGui.QPainter()
        painter.begin(self)
        self.__drawGrid(event, painter)
        self.__readMapFile()
        self.__fillTiles(painter)
        self.__drawRobot(painter)
        painter.end()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.RightButton:
            for qrectf in self.qrectfs:
                if qrectf.contains(event.pos()):
                    qrectf_index_in_list = self.qrectfs.index(qrectf)
                    y = qrectf_index_in_list // self.row_count
                    x = qrectf_index_in_list % self.row_count
                    if not (x,y) == self.robot_position:                # toto vyries..............
                        if (x,y) in self.walls:
                            self.walls.pop(self.walls.index( (x,y) ))
                        else:
                            self.walls.append( (x,y) )
                        self.simulator.gui_node.publish_data(self.__create_json_data())

        elif event.button() == QtCore.Qt.LeftButton:            
            x = float(event.pos().x())
            y = float(event.pos().y())
            theta = 0.0

            # qline = QtCore.QRectF(event.pos().x(), event.pos().y(), self.simulator.gui_node.robot_pose.x, self.simulator.gui_node.robot_pose.y)
            # for qrectf in self.qrectfs:
            #     if qrectf.intersects(qline):
            #         x = float(qrectf.x())
            #         y = float(qrectf.y())

            self.simulator.gui_node.publish_goal_pose(x, y, theta)
            # logging.info('Left point click {}'.format(event.pos()))
        self.repaint()

    def __drawGrid(self, event, painter: QtGui.QPainter) -> None:
        painter.drawRects(self.qrectfs.copy())

    def __fillTiles(self, painter: QtGui.QPainter) -> None:
        for wall in self.walls:
            painter.fillRect(self.qrectfs[wall[0]+wall[1]*self.row_count], QtCore.Qt.SolidPattern)    

    def __drawRobot(self, painter: QtGui.QPainter) -> None:
        '''Method draws a robot icon in specific position.'''
        qpixmap = QtGui.QPixmap('/home/marek/School/Ing - Kybernetika/2.ZS/NMVR/nmvr/s_a_ws/src/first_package/first_package/imgs/robot_icon.png')

        qpixmap = qpixmap.scaled(int(qpixmap.width() * math.sqrt(2)), int(qpixmap.height() * math.sqrt(2)), QtCore.Qt.KeepAspectRatio)
        qpixmap = qpixmap.transformed(QtGui.QTransform().rotate( math.degrees(self.simulator.gui_node.robot_pose.theta)-90), QtCore.Qt.SmoothTransformation)


        painter.drawPixmap(
            int(self.simulator.gui_node.robot_pose.x),
            int(self.simulator.gui_node.robot_pose.y),
            self.tile_size*3,
            self.tile_size*3,
            qpixmap
            )

        red_pen = QtGui.QPen(little_qt.red, 3)
        painter.setPen(red_pen)
        font = QtGui.QFont()
        font.setFamily('Times')
        font.setPointSize(20)
        painter.setFont(font)

        painter.drawText( 20, 750, "position: \t[{},{}] \n orientation:\t {}".format(
                int(self.simulator.gui_node.robot_pose.x),
                int(self.simulator.gui_node.robot_pose.y),
                math.degrees(self.simulator.gui_node.robot_pose.theta)))
        # painter.restore()
    
    def __createQRectFList(self, tile_size, row_count, column_count) -> typing.List:
        '''Method creates QRectF objects, according to number of rows and column and also according to size of one tile.'''
        rects = []
        for j in range(0, tile_size*column_count, tile_size):
            for i in range(0, tile_size*row_count, tile_size):
                rects.append(QtCore.QRectF(i, j, tile_size, tile_size))
        return rects

    def __readMapFile(self) -> None:               
        '''Method opens .json file of the map, search for all ones and converts indices to list of tuples (x,y).'''
        data = self.simulator.gui_node.data
        self.walls = []
        if not data == None and len(data) > 0:
            for idx, row in enumerate(data['data']):
                row_index_list = [idx] * len(row)
                indices_of_ones_list = [index for index, value in enumerate(row) if value == 1]
                self.walls += list(zip(indices_of_ones_list,row_index_list))        

    def __create_json_data(self) -> None:
        my_array = []
        for j in range (0, self.column_count):
            sub_array = []
            for i in range (0, self.row_count):
                if (i, j) in self.walls:
                    sub_array.append(1)
                else:
                    sub_array.append(0)
            my_array.append(sub_array)
        # for wall in self.walls:
        #     my_array[wall[1]][wall[0]] = 1

        json_data = {"data":my_array}        

        str_data = str(json_data)
        str_data = str_data.replace("\'data\'", "\"data\"",)
        self.simulator.gui_node.data = str_data
        return str_data

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.subscription = self.create_subscription(String,'json_read', self.listener_callback,10)
        self.subscription
        self.pose_subscription = self.create_subscription(Pose,'pose', self.pose_listener_callback,10)
        self.pose_subscription
        self.publisher = self.create_publisher(String, 'json_write', 10)
        self.goal_publisher = self.create_publisher(Pose, 'goal', 10)
        self._data = None
        self._robot_pose = Pose()
        self.repaint_needed = False
        
    def listener_callback(self, msg):
        if self._data is None:
            self._data = json.loads(msg.data)
            # self.get_logger().info('Map received.')
        else:
            self.destroy_subscription(self.subscription)

    def pose_listener_callback(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        if data.x == self._robot_pose.x and data.y == self._robot_pose.y and data.theta == self._robot_pose.theta:
            self.repaint_needed = False
        else:
            self.repaint_needed = True

        self._robot_pose.x = data.x
        self._robot_pose.y = data.y
        self._robot_pose.theta = data.theta
        # self.get_logger().info("Gui node pose callback")
        # msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x, data.y, data.theta)
        # self.get_logger().info(msg)    
    
    def publish_data(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)   
        # self.get_logger().info('Map sent.')

    def publish_goal_pose(self, x, y, theta):
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        goal_pose.theta = theta
        self.goal_publisher.publish(goal_pose)   
        # self.get_logger().info('Pose sent.')

    def is_repaint_needed(self):
        foo = self.repaint_needed
        self.repaint_needed = False
        return foo

    @property
    def data(self):
        """Data property getter"""
        return self._data

    @data.setter
    def data(self, data):
        self._data = self._data = json.loads(data)

    @property
    def robot_pose(self):
        """Data property getter"""
        return self._robot_pose


class Simulator(QtWidgets.QMainWindow):
    def __init__(self, gui_node: GuiNode):
        """Simulator constructor"""
        super().__init__()
        # logging.info("Simulator created")

        self.setGeometry(200, 50, 1500, 1000)
        self.setWindowTitle('Simulator')
        
        self.gui_node: GuiNode = gui_node

        self.grid_map = GridMap(
            self, 
            row_count=60, 
            column_count=40, 
            tile_size=20,
            top_left_x=10,
            top_left_y=10)       

        self.use_gui_node()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.use_gui_node)
        self.timer.start(100)                  # aj zmena polohy - update vizualneho pohybu robota

    def use_gui_node(self):
        rclpy.spin_once(self.gui_node)

        if self.gui_node.is_repaint_needed():
            self.repaint()

class ChoicePopupWindow(QtWidgets.QDialog):
    def __init__(self, parent = None, wall = False) -> None:
        super().__init__(parent=parent)

        self.resize(300,150)
        self.setWindowTitle("Choose wisely")

        self.radio_button_wall = QtWidgets.QRadioButton(self)
        self.radio_button_empty = QtWidgets.QRadioButton(self)

        self.radio_button_wall.setGeometry(20, 75, 20, 20)
        self.radio_button_empty.setGeometry(100, 75, 20, 20)

        self.radio_button_wall.setObjectName("wall_button")
        self.radio_button_empty.setObjectName("empty_button")

        # self.radio_button_wall.toggled.connect(self.__wallAction)
        self.radio_button_wall.clicked.connect(self.__wallAction)
        self.radio_button_empty.clicked.connect(self.__emptyAction)
  
    def __wallAction(self):  
        logging.info("Wall button clicked")

    def __emptyAction(self):  
        logging.info("Empty button clicked")

def main():
    rclpy.init(args=None)  
    app = QtWidgets.QApplication(sys.argv)       
    gui_node = GuiNode() 
    simulator= Simulator(gui_node)
    simulator.show()     
    app.exec()
    gui_node.destroy_node()
    rclpy.shutdown()   

if __name__ == '__main__':    
    main()