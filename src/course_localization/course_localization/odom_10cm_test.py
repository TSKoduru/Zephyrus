"""INITIAL TEST ONLY: odometry UP35cm -> FORWARD10cm -> DOWN. No PF/MPC."""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class OdomTenCmTest(Node):
    def __init__(self):
        super().__init__('ODOM_ONLY_UP_FORWARD_10CM_DOWN')
        self.declare_parameter('enable_motion', True)
        self.position = None; self.yaw = 0.; self.frame = 'odom'
        self.odom_at = self.state_at = self.map_at = -float('inf')
        self.flight = {}; self.map_health = {}; self.stage = 'WAIT'; self.origin = None
        self.target = None; self.entered = time.monotonic(); self.arrived = None; self.fault = ''
        self.pose_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.status = self.create_publisher(String, '/odom_test/mission_status', 10)
        self.hold = self.create_client(Trigger, '/skybeta/hold'); self.hold_sent = False
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.create_subscription(String, '/odom_test/map_status', self.on_map, 10)
        self.create_timer(.1, self.tick)
        self.get_logger().warn('INITIAL ODOMETRY TEST ONLY: UP 35cm, FORWARD 10cm, DOWN. NO PARTICLE FILTER. Pilot arms/controls Offboard.')

    def on_odom(self, msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        values=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1)>.01 or not -.05<=age<=.4:return
        if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':return
        self.position=values[:3]; self.frame=msg.header.frame_id
        self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.odom_at=time.monotonic()

    def on_state(self,msg):
        try:self.flight=json.loads(msg.data); self.state_at=time.monotonic()
        except ValueError:pass

    def on_map(self,msg):
        try:self.map_health=json.loads(msg.data); self.map_at=time.monotonic()
        except ValueError:pass

    def pose(self,target):
        msg=PoseStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id=self.frame
        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=map(float,target)
        # All-zero orientation asks the existing interface to retain heading.
        self.pose_pub.publish(msg)

    def descend(self,speed):
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='px4_local_enu'
        msg.twist.linear.z=float(speed);self.twist_pub.publish(msg)

    def stop(self,reason):
        self.fault=reason;self.stage='STOPPED'
        if not self.hold_sent and self.hold.service_is_ready():
            self.hold.call_async(Trigger.Request());self.hold_sent=True

    def tick(self):
        now=time.monotonic(); gates=[]
        if now-self.odom_at>.4:gates.append('fresh odometry required')
        if now-self.state_at>.6 or not self.flight.get('px4_link'):gates.append('flight interface unavailable')
        if self.count_publishers('/skybeta/setpoint/pose')>1 or self.count_publishers('/skybeta/setpoint/twist')>1:gates.append('OTHER MISSION PUBLISHER PRESENT')
        if self.flight.get('frozen'):gates.append('interface frozen')
        if self.flight and not self.flight.get('envelope_ok'):gates.append('outside envelope')
        if now-self.map_at>2 or not self.map_health.get('ready',False):gates.append('stereo map logger not ready')
        enabled=self.get_parameter('enable_motion').value is True
        active=bool(self.flight.get('armed') and self.flight.get('offboard_active'))
        if self.stage=='STOPPED':self.stop(self.fault)
        elif self.stage=='DONE':
            if not self.hold_sent and self.hold.service_is_ready():self.hold.call_async(Trigger.Request());self.hold_sent=True
        elif not enabled:pass
        elif gates:
            if self.stage!='WAIT':self.stop('; '.join(gates))
        elif self.stage=='WAIT':
            # Prime with zero velocity: no stale airborne pose is stored as a hold.
            self.descend(0.)
            if active:
                self.origin=self.position.copy();self.heading=self.yaw
                self.target=self.origin+np.array([0.,0.,.35]);self.stage='UP';self.entered=now
        elif not active:self.stop('Pilot left Offboard or disarmed; no automatic restart')
        elif now-self.entered>20:self.stop('Stage timeout')
        elif self.stage in ('UP','FORWARD_10CM'):
            self.pose(self.target)
            if np.linalg.norm(self.position-self.target)<.025:
                if self.arrived is None:self.arrived=now
                if now-self.arrived>=1.:
                    if self.stage=='UP':
                        self.target=self.target+np.array([.1*math.cos(self.heading),.1*math.sin(self.heading),0.])
                        self.stage='FORWARD_10CM'
                    else:self.stage='DOWN'
                    self.entered=now;self.arrived=None
            else:self.arrived=None
        elif self.stage=='DOWN':
            if self.flight.get('landed') is True:
                self.stage='DONE'
                if self.hold.service_is_ready():self.hold.call_async(Trigger.Request());self.hold_sent=True
            elif self.position[2]<self.origin[2]-.10:self.stop('Below starting height without landed confirmation')
            else:self.descend(-.08)
        self.status.publish(String(data=json.dumps(dict(test='ODOMETRY ONLY: UP35cm FORWARD10cm DOWN',stage=self.stage,
            enabled=enabled,gates=gates,fault=self.fault,particle_filter=False,odometry='/visual_slam/tracking/odometry'))))


def main(args=None):
    rclpy.init(args=args);node=OdomTenCmTest()
    try:rclpy.spin(node)
    except KeyboardInterrupt:node.stop('Operator stopped test')
    finally:node.destroy_node();rclpy.shutdown()
