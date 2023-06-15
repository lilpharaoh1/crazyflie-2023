from curses.ascii import CR
import rclpy
from rclpy.node import Node
from .connect_callback_group import ConnectCallbackGroup
from cflib.crazyflie import Crazyflie
import cflib.crtp

from std_msgs.msg import String

class ConnectNode(Node):
    CONNECT_TIME = 10 # s

    def __init__(self):
        super().__init__('connect_node')
        cflib.crtp.init_drivers(enable_debug_driver=False)

        self.declare_parameter('uris')
        self.uris = self.get_parameter('uris').get_parameter_value().string_array_value
       
        for uri in self.uris:
            cf = Crazyflie()
            cf.connected.add_callback(self._connected)
            cf.disconnected.add_callback(self._disconnected)
            cf.connection_failed.add_callback(self._connection_failed)
            cf.connection_lost.add_callback(self._connection_lost)
            cf.open_link(uri)

        self.connect_cbgroup = ConnectCallbackGroup('individual')
        self.connect_timer = self.create_timer(self.CONNECT_TIME, self.connect_cb, callback_group=self.connect_cbgroup)
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.i = 0

    def connect_cb(self):
        uris = self.get_parameter('uris').get_parameter_value()
        # self.get_logger().info(str(uris))

    def _connected(self, link_uri):
        print('Connected to %s.' % (link_uri))
        # if link_uri not in self._crazyflie_logs:
        #     log = CrazyflieLog(*self._crazyflies[link_uri])
        #     self._start_logs(log)
        #     self._crazyflie_logs[link_uri] = log
        # if link_uri not in self._controllers:
        #     controller = CrazyflieControl(*self._crazyflies[link_uri])
        #     self._controllers[link_uri] = controller

    def _connection_failed(self, link_uri, msg):
        """Callback when initial connection fails (i.e. no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        #self._crazyflies[link_uri][1].open_link(link_uri)


    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e.
        Crazyflie moves out of range)"""
        print('Connection to %s lost : %s' % (link_uri, msg))
        #self._crazyflies[link_uri][1].open_link(link_uri)


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        print('Reconnecting...')
        #self._crazyflies[link_uri][1].open_link(link_uri)


def main(args=None):
    rclpy.init(args=args)
    connect_node = ConnectNode()
    rclpy.spin(connect_node)
    connect_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()