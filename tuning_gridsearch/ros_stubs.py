"""
Minimal stand-ins for the ROS 2 middleware, so the real node classes of
G_ROS_interface/ROS2 (node_base.py and the sim_node_lidar_*.py filters) can be
constructed and driven in-process by the 2D tuning simulation, without rclpy,
DDS or Gazebo.

Only the transport is replaced: Node records its subscriptions, publishers and
timers instead of talking to the graph, publishers keep the last message, and
the clock is the simulation clock (SIM_CLOCK). The message classes carry just
the fields the nodes read and write. Everything the nodes compute runs
unchanged.

install() must run before anything imports node_base.
"""
import sys
import types


# ----------------------------------------------------------------------
# simulation clock (node.get_clock().now())
# ----------------------------------------------------------------------
class _SimClock:
    def __init__(self):
        self.t = 0.0

    def set(self, t):
        self.t = float(t)

SIM_CLOCK = _SimClock()


class _Time:
    def __init__(self, t):
        self.nanoseconds = int(round(t * 1e9))


class _Clock:
    def now(self):
        return _Time(SIM_CLOCK.t)


# ----------------------------------------------------------------------
# messages
# ----------------------------------------------------------------------
class Vec3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Quat:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec, self.nanosec = sec, nanosec

    @classmethod
    def from_sec(cls, t):
        ns = int(round(t * 1e9))
        return cls(ns // 1_000_000_000, ns % 1_000_000_000)


class Header:
    def __init__(self, stamp=None, frame_id=''):
        self.stamp = stamp if stamp is not None else Stamp()
        self.frame_id = frame_id


class Pose:
    def __init__(self, position=None, orientation=None):
        self.position = position if position is not None else Vec3()
        self.orientation = orientation if orientation is not None else Quat()


class _PoseWithCovariance:
    def __init__(self):
        self.pose = Pose()


class Twist:
    def __init__(self):
        self.linear = Vec3()
        self.angular = Vec3()


class _TwistWithCovariance:
    def __init__(self):
        self.twist = Twist()


class Odometry:
    def __init__(self):
        self.header = Header()
        self.pose = _PoseWithCovariance()
        self.twist = _TwistWithCovariance()


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class Image:
    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.encoding = ''
        self.is_bigendian = 0
        self.step = 0
        self.data = b''


class PointCloud2:
    def __init__(self):
        self.header = Header()
        self.point_step = 12
        self.data = b''


class Float32MultiArray:
    def __init__(self):
        self.data = []


# ----------------------------------------------------------------------
# node
# ----------------------------------------------------------------------
class Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.last = None

    def publish(self, msg):
        self.last = msg

    def get_subscription_count(self):
        return 1   # "the Gazebo bridge is listening"


class Logger:
    '''Collects warnings (counted per message) so the simulation can report them.'''
    def __init__(self):
        self.warn_counts = {}

    def _record(self, msg):
        self.warn_counts[msg] = self.warn_counts.get(msg, 0) + 1

    def info(self, msg, **kwargs):
        pass

    def debug(self, msg, **kwargs):
        pass

    def warn(self, msg, **kwargs):
        self._record(msg)

    warning = warn

    def error(self, msg, **kwargs):
        self._record(msg)


class Node:
    def __init__(self, node_name, *args, **kwargs):
        self._name = node_name
        self._logger = Logger()
        self.subscriptions = {}   # topic -> callback
        self.publishers = {}      # topic -> Publisher
        self.timers = []          # (period, callback)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback

    def create_publisher(self, msg_type, topic, qos):
        pub = Publisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return callback

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


class Parameter:
    class Type:
        BOOL = 'bool'

    def __init__(self, *args, **kwargs):
        pass


class _Executor:
    def __init__(self, *args, **kwargs):
        pass

    def add_node(self, node):
        pass

    def spin(self):
        pass


class _QoSPresetProfiles:
    class SENSOR_DATA:
        value = None


# ----------------------------------------------------------------------
# module installation
# ----------------------------------------------------------------------
def _module(name, **attrs):
    mod = types.ModuleType(name)
    mod.__dict__.update(attrs)
    return mod


def install():
    '''Register the stand-in modules under the ROS 2 module names (idempotent).'''
    if getattr(sys.modules.get('rclpy'), '_is_tuning_stub', False):
        return
    if 'nCBF3D.G_ROS_interface.ROS2.node_base' in sys.modules:
        # node_base already bound other classes; stubbing now would mix the two
        raise RuntimeError("ros_stubs.install() must run before node_base is imported")

    qos = _module('rclpy.qos', QoSPresetProfiles=_QoSPresetProfiles)
    mods = {
        'rclpy':            _module('rclpy', init=lambda *a, **k: None, shutdown=lambda *a, **k: None, qos=qos),
        'rclpy.node':       _module('rclpy.node', Node=Node),
        'rclpy.executors':  _module('rclpy.executors', SingleThreadedExecutor=_Executor,
                                    MultiThreadedExecutor=_Executor),
        'rclpy.parameter':  _module('rclpy.parameter', Parameter=Parameter),
        'rclpy.qos':        qos,
        'sensor_msgs':      _module('sensor_msgs'),
        'sensor_msgs.msg':  _module('sensor_msgs.msg', PointCloud2=PointCloud2, Image=Image),
        'nav_msgs':         _module('nav_msgs'),
        'nav_msgs.msg':     _module('nav_msgs.msg', Odometry=Odometry),
        'geometry_msgs':    _module('geometry_msgs'),
        'geometry_msgs.msg': _module('geometry_msgs.msg', Twist=Twist, PoseStamped=PoseStamped),
        'std_msgs':         _module('std_msgs'),
        'std_msgs.msg':     _module('std_msgs.msg', Float32MultiArray=Float32MultiArray),
    }
    for name, mod in mods.items():
        mod._is_tuning_stub = True
        sys.modules[name] = mod
