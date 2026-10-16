from __future__ import annotations
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from fabtino_core.battery import BatteryConfig, BatteryState


class BatteryNode(Node):
    """Simulated battery: integrates consumption from the filtered odometry velocity.

    Publishes ``/fabtino/battery`` (JSON, 2 Hz) with the state of charge AND the cost
    coefficients, so the global planner uses exactly the same model as this node.

    Requests on ``/battery/request`` (JSON):
        {"type": "set_soc", "soc_pct": 35}   # set the charge level (tests, docking)
        {"type": "charge"}                   # back to 100 %
    """

    def __init__(self):
        super().__init__('battery')
        defaults = BatteryConfig()
        for name, value in defaults.to_dict().items():
            self.declare_parameter(name, value)
        self.declare_parameter('initial_soc_pct', 100.0)
        # >1 speeds the drain up, handy to see a refusal happen in simulation.
        self.declare_parameter('consumption_scale', 1.0)
        config = BatteryConfig.from_dict({n: self.get_parameter(n).value for n in defaults.to_dict()})
        self.config = config
        self.state = BatteryState(config, float(self.get_parameter('initial_soc_pct').value),
                                  float(self.get_parameter('consumption_scale').value))
        self.pub = self.create_publisher(String, '/fabtino/battery', 10)
        self.create_subscription(Odometry, '/fabtino/odometry/filtered', self.odom_cb, 20)
        self.create_subscription(String, '/battery/request', self.request_cb, 10)
        self.create_timer(0.5, self.publish)

    def seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    def odom_cb(self, msg):
        self.state.update(self.seconds(), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def request_cb(self, msg):
        try:
            request = json.loads(msg.data)
            kind = request.get('type')
            if kind == 'set_soc':
                self.state.set_soc(float(request['soc_pct']))
            elif kind == 'charge':
                self.state.set_soc(100.0)
            else:
                raise ValueError('Unknown battery request type')
            self.publish()
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f'Invalid battery request: {exc}')

    def publish(self):
        msg = String()
        msg.data = json.dumps({'soc_pct': round(self.state.soc_pct, 3),
                               'energy_wh': round(self.state.energy_wh, 3),
                               'low': self.state.low,
                               'simulated': True,
                               'config': self.config.to_dict()})
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BatteryNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
