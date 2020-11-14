from enum import Enum
import curses

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from roompi_interfaces.msg import PanTilt


class MainWindow:
    class Key(Enum):
        FORWARD = 1
        BACKWARD = 2
        LEFT = 3
        RIGHT = 4
        LOOK_UP = 5
        LOOK_DOWN = 6
        LOOK_LEFT = 7
        LOOK_RIGHT = 8

    KEY_MAPPING = {
        # Movement.
        ord("w"): Key.FORWARD,
        ord("a"): Key.LEFT,
        ord("s"): Key.BACKWARD,
        ord("d"): Key.RIGHT,
        # Looking.
        ord("i"): Key.LOOK_UP,
        ord("j"): Key.LOOK_LEFT,
        ord("k"): Key.LOOK_DOWN,
        ord("l"): Key.LOOK_RIGHT,
    }

    def __init__(self, stdscr):
        self._stdscr = stdscr
        self._stdscr.nodelay(True)
        curses.curs_set(0)

        # Initial window state.
        self._stdscr.clear()
        self._stdscr.addstr(0, 0, "Use WASD to move.")
        self._stdscr.addstr(1, 0, "Use IJKL to look.")

    def read_key(self):
        return self.KEY_MAPPING.get(self._stdscr.getch())

    def display(self):
        self._stdscr.refresh()


class RoompiKeyTeleop(Node):
    def __init__(self, window):
        super().__init__("roompi_key_teleop")

        self._window = window

        self._hz = self.declare_parameter("hz", 30).value

        self._linear_max_speed = self.declare_parameter(
            "linear_max_speed", 0.8
        ).value
        self._angular_max_speed = self.declare_parameter(
            "angular_max_speed", 0.8
        ).value
        self._pantilt_increment = self.declare_paramenter(
            "pantilt_increment", 1.0
        ).value

        self._linear_speed = 0.0
        self._angular_speed = 0.0
        self._pan_angle = 0.0
        self._tilt_angle = 0.0

        self._twist_pub = self.create_publisher(
            Twist, "key_vel", qos_profile_system_default
        )
        self._pantilt_pub = self.create_publisher(
            PanTilt, "pan_tilt", qos_profile_system_default
        )

        self.create_timer(1.0 / self._hz, self._refresh)

    def _refresh(self):
        self._update_state(self._window.read_key())
        self._publish_state()
        self._window.display()

    def _update_state(self, key):
        if key == MainWindow.Key.FORWARD:
            self._linear_speed = self._linear_max_speed

        elif key == MainWindow.Key.BACKWARD:
            self._linear_speed = -self._linear_max_speed

        elif key == MainWindow.Key.LEFT:
            self._angular_speed = self._angular_max_speed

        elif key == MainWindow.Key.RIGHT:
            self._angular_speed = -self._angular_max_speed

        elif key == MainWindow.Key.LOOK_UP:
            self._tilt_angle += self._pantilt_increment

        elif key == MainWindow.Key.LOOK_DOWN:
            self._tilt_angle -= self._pantilt_increment

        elif key == MainWindow.Key.LOOK_LEFT:
            self._pan_angle -= self._pantilt_increment

        elif key == MainWindow.Key.LOOK_RIGHT:
            self._pan_angle += self._pantilt_increment

        elif key is None:
            self._linear_speed = 0.0
            self._angular_speed = 0.0

    def _publish_state(self):
        twist_msg = Twist()
        twist_msg.linear.x = self._linear_speed
        twist_msg.angular.z = self._angular_speed
        self._twist_pub.publish(twist_msg)

        pantilt_msg = PanTilt()
        pantilt_msg.pan = self._pan_angle
        pantilt_msg.tilt = self._tilt_angle
        self._pantilt_pub.publish(pantilt_msg)


def main(stdscr):
    rclpy.init()
    node = RoompiKeyTeleop(MainWindow(stdscr))
    rclpy.spin(node)
    rclpy.shutdown()


curses.wrapper(main)
