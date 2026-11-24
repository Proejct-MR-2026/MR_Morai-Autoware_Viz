#!/usr/bin/env python3
# 신규 — 26.11.13 _ YSH _ 외란 시각화를 Autoware RViz 기본 화면 그대로 보이게 하는 노드 (viz_marker.py 대체).
#   자체 마커를 그리지 않고, e2e_simulator 에서 RViz 가 보던 Autoware 메시지만 낸다.
#
#   구독  /perception/object_recognition/tracking/objects  브리지가 낸 외란 후 객체 (Autoware 입력)
#         /daq/object_disturbance_status                  객체마다 GT(before)·외란 후·visible
#         /daq/disturbance                                걸린 외란·값
#         /viz/cue_state                                  수신기 상태 (시나리오 이름)
#         /localization/kinematic_state                   자차 위치 (viewer 프레임용)
#   발행  /perception/object_recognition/objects  (PredictedObjects) — 기본 RViz "Prediction" 이 그린다.
#           e2e 에서는 map_based_prediction 이 내는 토픽이다. 지도가 없으므로 여기서 외란 후 객체를 그대로 옮기고
#           예측 경로는 등속 직선 하나 (5 s, 0.5 s 간격). 숨김 프레임은 빈 목록 → 화면에서 객체가 사라진다.
#         /viz/debug/ground_truth/objects        (TrackedObjects) — 디버그 그룹 "GroundTruth". MORAI 정답 위치.
#         /viz/debug/disturbance_text            (StringStamped, 5 Hz) — 디버그 그룹 "DisturbanceStatus" 한 줄.
#         TF map → viewer                        자차 x,y (지도가 없을 때만. 지도가 있으면 map_tf_generator 가 낸다)
#         TF map → base_link                     --tf-from-odom 일 때만 (bag 다시 보기: bag 의 /tf 는 viewer 가 섞여 틀지 않는다.
#                                                브리지 TF 도 kinematic_state 와 같은 자세라 같은 값이다)
#
#   GT 상자 중심: ObjectStatus 위치는 MORAI 기준점(차량은 뒤차축). 같은 객체의 TrackedObject 중심과의 차이를 차체 좌표로
#   배워 GT 에 더한다 (외란은 기준점과 상자 중심을 같은 양만큼 옮긴다). 못 배웠으면 차량 1.45 m 앞.
import argparse
import collections
import json
import math
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from unique_identifier_msgs.msg import UUID
from autoware_internal_debug_msgs.msg import StringStamped
from autoware_perception_msgs.msg import (ObjectClassification, PredictedObject, PredictedObjects, PredictedPath,
                                          Shape, TrackedObject, TrackedObjects, TrackedObjectKinematics)
from morai_ros2_msgs.msg import DisturbanceStatus, ObjectDisturbanceStatusList

VEHICLE_CENTRE_FWD = 1.45     # 못 배웠을 때 차량 상자 중심 = 기준점 앞 [m]
PRED_HORIZON_S = 5.0          # 예측 경로 길이
PRED_STEP_S = 0.5             # 예측 경로 간격
VIEWER_MAX_HZ = 50.0


def uid_of(name):
    return uuid.uuid5(uuid.NAMESPACE_OID, 'morai:' + name).bytes


def stamp_ns(st):
    return int(st.sec) * 1000000000 + int(st.nanosec)


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_yaw(q, yaw):
    q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def finite(*vals):
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def cls_list(label):
    c = ObjectClassification()
    c.label = label
    c.probability = 1.0
    return [c]


class VizAutoware(Node):
    def __init__(self, viewer, tf_from_odom=False):
        super().__init__('viz_autoware')
        sub = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pred = self.create_publisher(PredictedObjects, '/perception/object_recognition/objects', pub)
        self.pub_gt = self.create_publisher(TrackedObjects, '/viz/debug/ground_truth/objects', pub)
        self.pub_text = self.create_publisher(StringStamped, '/viz/debug/disturbance_text', pub)
        self.create_subscription(TrackedObjects, '/perception/object_recognition/tracking/objects', self.on_tracked, sub)
        self.create_subscription(ObjectDisturbanceStatusList, '/daq/object_disturbance_status', self.on_ods, sub)
        self.create_subscription(DisturbanceStatus, '/daq/disturbance', self.on_status, sub)
        self.create_subscription(String, '/viz/cue_state', self.on_cue, latched)
        self.viewer = viewer
        self.tf_from_odom = tf_from_odom
        self.last_viewer_ns = 0
        if viewer or tf_from_odom:
            self.tf = TransformBroadcaster(self)
            self.create_subscription(Odometry, '/localization/kinematic_state', self.on_odom, sub)
        self.tracked = collections.OrderedDict()   # stamp_ns → {uuid: TrackedObject}
        self.known = {}                            # uuid → (classification, shape.type, twist) 마지막으로 본 것
        self.offsets = {}                          # name → (앞, 왼, 위) 상자 중심 − 기준점 (차체 좌표)
        self.status = None
        self.cue = {}
        self.n_gt = 0
        self.n_sent = 0
        self.last_ods_t = None
        self.create_timer(0.2, self.on_timer)
        self.get_logger().info('Autoware 표시 노드 — Prediction(/perception/object_recognition/objects) · '
                               'GT(/viz/debug/ground_truth/objects) · 상태 한 줄(/viz/debug/disturbance_text)'
                               + (' · TF map→viewer(자차)' if viewer else '')
                               + (' · TF map→base_link(kinematic_state)' if tf_from_odom else ''))

    # ── 외란 후 객체 → PredictedObjects (등속 직선 예측)
    def on_tracked(self, msg):
        out = PredictedObjects()
        out.header = msg.header
        frame = {}
        n = int(round(PRED_HORIZON_S / PRED_STEP_S))
        for o in msg.objects:
            key = bytes(o.object_id.uuid)
            frame[key] = o
            self.known[key] = (list(o.classification), o.shape.type, o.kinematics.twist_with_covariance)
            p = o.kinematics.pose_with_covariance.pose
            yaw = quat_yaw(p.orientation)
            v = o.kinematics.twist_with_covariance.twist.linear
            wx = math.cos(yaw) * v.x - math.sin(yaw) * v.y      # twist 는 객체 좌표 → world 로 돌린다
            wy = math.sin(yaw) * v.x + math.cos(yaw) * v.y
            path = PredictedPath()
            path.time_step = Duration(sec=int(PRED_STEP_S), nanosec=int((PRED_STEP_S % 1.0) * 1e9))
            path.confidence = 1.0
            for k in range(n + 1):
                q = Pose()
                q.position.x = p.position.x + wx * PRED_STEP_S * k
                q.position.y = p.position.y + wy * PRED_STEP_S * k
                q.position.z = p.position.z
                q.orientation = p.orientation
                path.path.append(q)
            po = PredictedObject()
            po.object_id = o.object_id
            po.existence_probability = o.existence_probability
            po.classification = o.classification
            po.kinematics.initial_pose_with_covariance = o.kinematics.pose_with_covariance
            po.kinematics.initial_twist_with_covariance = o.kinematics.twist_with_covariance
            po.kinematics.initial_acceleration_with_covariance = o.kinematics.acceleration_with_covariance
            po.kinematics.predicted_paths = [path]
            po.shape = o.shape
            out.objects.append(po)
        self.pub_pred.publish(out)
        self.n_sent = len(msg.objects)
        self.tracked[stamp_ns(msg.header.stamp)] = frame
        while len(self.tracked) > 40:
            self.tracked.popitem(last=False)

    def tracked_for(self, st):
        f = self.tracked.get(st)
        if f is not None:
            return f
        best, best_dt = None, 150_000_000            # 같은 stamp 가 아직 없으면 0.15 s 안의 가장 가까운 것
        for k, v in self.tracked.items():
            if abs(k - st) < best_dt:
                best, best_dt = v, abs(k - st)
        return best or {}

    # ── GT (before) → TrackedObjects
    def on_ods(self, msg):
        tracked = self.tracked_for(stamp_ns(msg.header.stamp))
        out = TrackedObjects()
        out.header = msg.header
        out.header.frame_id = 'map'
        groups = ((msg.npc_list, ObjectClassification.CAR),
                  (msg.pedestrian_list, ObjectClassification.PEDESTRIAN),
                  (msg.obstacle_list, ObjectClassification.UNKNOWN))
        n = 0
        for objs, label in groups:
            for o in objs:
                n += 1
                b = o.before
                if not finite(b.position.x, b.position.y, b.position.z, b.heading):
                    continue
                name = o.name
                key = uid_of(name)
                hb = math.radians(b.heading)
                L, W, H = b.size.y, b.size.x, b.size.z         # ObjectStatus size = (폭, 길이, 높이)
                t = tracked.get(key)
                if o.visible and t is not None and finite(o.position.x, o.position.y, o.position.z, o.heading):
                    tp = t.kinematics.pose_with_covariance.pose.position
                    ha = math.radians(o.heading)
                    wx, wy = tp.x - o.position.x, tp.y - o.position.y
                    fwd = math.cos(ha) * wx + math.sin(ha) * wy
                    left = -math.sin(ha) * wx + math.cos(ha) * wy
                    if abs(fwd) < 10 and abs(left) < 10:
                        self.offsets[name] = (fwd, left, tp.z - o.position.z)
                fwd, left, up = self.offsets.get(name, (VEHICLE_CENTRE_FWD if label == ObjectClassification.CAR
                                                        else 0.0, 0.0, H / 2.0))
                g = TrackedObject()
                g.object_id = UUID(uuid=list(key))
                g.existence_probability = 1.0
                cls, shape_type, twist = self.known.get(key, (cls_list(label), Shape.BOUNDING_BOX, None))
                g.classification = cls
                k = g.kinematics
                pose = k.pose_with_covariance.pose
                pose.position.x = b.position.x + math.cos(hb) * fwd - math.sin(hb) * left
                pose.position.y = b.position.y + math.sin(hb) * fwd + math.cos(hb) * left
                pose.position.z = b.position.z + up
                set_yaw(pose.orientation, hb)
                if twist is not None:
                    k.twist_with_covariance = twist
                k.orientation_availability = TrackedObjectKinematics.AVAILABLE
                g.shape.type = shape_type
                g.shape.dimensions.x = float(max(L, 0.1))
                g.shape.dimensions.y = float(max(W, 0.1))
                g.shape.dimensions.z = float(max(H, 0.1))
                out.objects.append(g)
        self.pub_gt.publish(out)
        self.n_gt = n
        self.last_ods_t = self.get_clock().now()

    def on_status(self, msg):
        self.status = msg

    def on_cue(self, msg):
        try:
            self.cue = json.loads(msg.data)
        except Exception:
            self.cue = {}

    def on_odom(self, msg):
        if self.tf_from_odom:
            b = TransformStamped()
            b.header.stamp = msg.header.stamp
            b.header.frame_id = 'map'
            b.child_frame_id = 'base_link'
            p = msg.pose.pose
            b.transform.translation.x, b.transform.translation.y, b.transform.translation.z = (
                p.position.x, p.position.y, p.position.z)
            b.transform.rotation = p.orientation
            self.tf.sendTransform(b)
        if not self.viewer:
            return
        ns = stamp_ns(msg.header.stamp)
        if ns and ns - self.last_viewer_ns < 1e9 / VIEWER_MAX_HZ:
            return
        self.last_viewer_ns = ns
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'viewer'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

    # ── 상태 한 줄 (5 Hz)
    def line(self):
        scn = (self.cue or {}).get('scenario')
        head = f'[{scn}] ' if scn else ''
        stale = (self.last_ods_t is None
                 or (self.get_clock().now() - self.last_ods_t).nanoseconds > 2_000_000_000)
        s = self.status
        if s is None or stale:
            return head + '외란 상태 없음 - 브리지 객체 프레임이 오지 않는다 (MORAI 연결 확인)'
        parts = []
        if s.position_on:
            parts.append(f'위치(종 {s.lon_offset_m:+.2f}±{s.lon_std_m:.2f}, 횡 {s.lat_offset_m:+.2f}±{s.lat_std_m:.2f} m)')
        if s.dropout_on:
            parts.append(f'간헐끊김(유지 {s.keep_prob:.2f}, 평균 {s.mean_gap_s:.2f} s)')
        if s.blackout_on:
            parts.append(f'완전끊김({s.blackout_duration_s:.1f} s ' + ('뒤 복구)' if s.blackout_recover else '- clear 까지)'))
        dist = ('외란 ' + ' + '.join(parts)) if parts else '외란 없음'
        if s.objects_visible:
            inp = f'Autoware 입력 {self.n_sent}/{self.n_gt}'
        else:
            inp = f'Autoware 입력 숨김 0/{self.n_gt}'
        return f'{head}{dist} · {inp}'

    def on_timer(self):
        m = StringStamped()
        m.stamp = self.get_clock().now().to_msg()
        m.data = self.line()
        self.pub_text.publish(m)


def main():
    rclpy.init()
    ap = argparse.ArgumentParser(description='외란 시각화 — Autoware RViz 기본 표시용 토픽')
    ap.add_argument('--no-viewer', action='store_true', help='TF map→viewer 를 내지 않는다 (지도가 있을 때)')
    ap.add_argument('--tf-from-odom', action='store_true',
                    help='TF map→base_link 를 kinematic_state 로 낸다 (bag 다시 보기, bag 의 /tf 를 틀지 않을 때)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    node = VizAutoware(viewer=not args.no_viewer, tf_from_odom=args.tf_from_odom)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
