#!/usr/bin/env python3
"""Local ROS end-to-end RGB compressed + compressedDepth -> TargetLocation test.

Real ROS messages/serialization, message_filters, cv_bridge and depth decoding.
YOLO is injected; TF uses a local synthetic broadcaster through real tf2 by default. No physical camera/model/
SLAM/covariance measurement is claimed. Refuses network discovery/server settings.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import time
from uuid import uuid4


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--depth-encoding',choices=('16UC1','32FC1'),default='16UC1')
    parser.add_argument('--exact-sync',action='store_true',help='test zero-skew TimeSynchronizer')
    parser.add_argument('--tf-mode',choices=('ros','injected'),default='ros',help='real local tf2 with synthetic values, or injected lookup')
    args=parser.parse_args()
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST' or os.environ.get('ROS_DISCOVERY_SERVER'):
        parser.error('use LOCALHOST discovery and unset ROS_DISCOVERY_SERVER')
    import cv2
    import numpy as np
    import yaml
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo,CompressedImage
    from rescue_interfaces.msg import MapState,TargetLocation,TargetObservation
    from rescue_perception.amr_target_node import create_node

    class FakeResult:
        def __init__(self,frame):
            self.frame=frame
            self.boxes=type('Boxes',(),dict(xyxy=np.array([[0.,0.,4.,4.]]),conf=np.array([.9]),cls=np.array([0.])))()
        def plot(self,**kwargs): return self.frame.copy()
    class FakeModel:
        names={0:'survivor'}
        calls=0
        def __init__(self,path): pass
        def predict(self,frame,**kwargs):
            self.calls+=1
            if self.calls==1: raise RuntimeError('intentional synthetic YOLO failure')
            return [FakeResult(frame)]
    tf_state={'available':True,'queries':[]}
    def lookup(target,source,stamp):
        tf_state['queries'].append((target,source,stamp))
        if not tf_state['available']: raise RuntimeError('intentional synthetic TF missing')
        msg=TransformStamped()
        msg.header.frame_id=target
        msg.child_frame_id=source
        msg.header.stamp.sec, msg.header.stamp.nanosec=divmod(stamp,10**9)
        msg.transform.translation.x,msg.transform.translation.y=1.,2.
        msg.transform.rotation.w=1.
        return msg

    node=source=executor=None
    with TemporaryDirectory() as directory:
        weights=Path(directory)/'fixture.pt'
        weights.write_bytes(b'synthetic-model-fixture; no trained weights')
        session=str(uuid4())
        now=time.time_ns()
        mapping=dict(alignment=dict(source_robot='robot_a',map_frame='test_map',mission_frame='test_mission',
            map_session_id=session,map_revision=0,transform_revision=1,authority_id='test-mapping-owner',
            clock_domain_id='test-clock',transform=dict(translation_xyz=[.5,-.5,0.],rotation_xyzw=[0.,0.,0.,1.]),
            valid_from_ns=now-1000000000,valid_until_ns=now+60000000000,position_error_m=.01,rotation_error_rad=0.,status='VALID'),
            covariance_verified=True,covariance_evidence_id='SYNTHETIC-FIXTURE-NOT-A-MEASUREMENT',sensor_variance_xyz=[.04,.04,.04])
        metadata=Path(directory)/'mapping.json'
        metadata.write_text(json.dumps(mapping))
        root=Path(__file__).resolve().parents[1]
        config=yaml.safe_load((root/'src/rescue_perception/config/amr_target_node.example.yaml').read_text())['amr_target_node']['ros__parameters']
        config.update(model_path=str(weights),model_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
            model_id='SYNTHETIC-model',model_imgsz=64,show=False,locations_enabled=True,projection_metadata_path=str(metadata),
            rgb_rectified=True,rgb_depth_alignment_verified=True,calibration_id='synthetic-cal',alignment_evidence_id='synthetic-registration',
            rgb_optical_frame='synthetic_optical',depth_optical_frame='synthetic_optical',
            rgb_stamp_semantics='CAPTURE',depth_stamp_semantics='CAPTURE',clock_domain_id='test-clock',clock_state='SYNCED',
            clock_error_bound_ns=0,receiver_clock_error_ns=0,alignment_authority='test-mapping-owner',min_valid_pixels=2,
            log_every=1,observations_topic='/target_smoke/observations',locations_topic='/target_smoke/locations',
            image_topic='/target_smoke/rgb',depth_topic='/target_smoke/depth',camera_info_topic='/target_smoke/info',
            annotated_topic='/target_smoke/annotated',map_state_topic='/target_smoke/map_state')
        if args.exact_sync: config['sync_slop_ns']=0
        ros_args=['--ros-args']
        for key,value in config.items():
            text=json.dumps(value) if isinstance(value,str) else (str(value).lower() if type(value) is bool else str(value))
            ros_args.extend(['-p',f'{key}:={text}'])
        rclpy.init(args=ros_args)
        try:
            node=create_node(model_factory=FakeModel,transform_lookup=lookup if args.tf_mode=='injected' else None)
            source=Node('amr_target_smoke')
            broadcaster=None
            if args.tf_mode=='ros':
                from tf2_ros import TransformBroadcaster
                broadcaster=TransformBroadcaster(source)
            sensor=QoSProfile(depth=3,reliability=ReliabilityPolicy.BEST_EFFORT)
            pubs={key:source.create_publisher(kind,config[key],sensor if kind!=MapState else 1)
                  for key,kind in (('image_topic',CompressedImage),('depth_topic',CompressedImage),
                                   ('camera_info_topic',CameraInfo),('map_state_topic',MapState))}
            observations=[]
            locations=[]
            source.create_subscription(TargetObservation,config['observations_topic'],observations.append,sensor)
            source.create_subscription(TargetLocation,config['locations_topic'],locations.append,10)
            executor=SingleThreadedExecutor()
            executor.add_node(node); executor.add_node(source)
            def spin(duration):
                end=time.monotonic()+duration
                while time.monotonic()<end: executor.spin_once(timeout_sec=.02)
            end=time.monotonic()+5.
            while time.monotonic()<end and not all(pub.get_subscription_count() for pub in pubs.values()): spin(.1)
            if not all(pub.get_subscription_count() for pub in pubs.values()): raise RuntimeError('ROS subscriptions not discovered')
            spin(.3)
            if broadcaster is not None and not broadcaster.pub_tf.get_subscription_count():
                raise RuntimeError('TF listener not discovered')
            bridge=CvBridge()
            frame=np.zeros((4,4,3),dtype=np.uint8)
            frame[:,:,1]=127
            def publish(depth_mm=2000,map_revision=0):
                stamp=source.get_clock().now().nanoseconds
                map_msg=MapState()
                map_msg.robot_id='robot_a'; map_msg.map_session_id=session; map_msg.map_revision=map_revision
                map_msg.current_native_map_frame='test_map'; map_msg.generated_at_ns=stamp-1000000
                map_msg.localization_quality='VALID'; map_msg.correction_detection_status='KNOWN'
                pubs['map_state_topic'].publish(map_msg)
                info=CameraInfo()
                info.width=info.height=4
                info.header.frame_id='synthetic_optical'
                info.header.stamp.sec,info.header.stamp.nanosec=divmod(stamp,10**9)
                info.p=[2.,0.,1.5,0.,0.,2.,1.5,0.,0.,0.,1.,0.]
                pubs['camera_info_topic'].publish(info)
                if broadcaster is not None and tf_state['available']:
                    broadcaster.sendTransform(lookup('test_map','synthetic_optical',stamp))
                spin(.05)  # cache calibration/map state before the synchronized images
                rgb=bridge.cv2_to_compressed_imgmsg(frame,dst_format='jpg')
                rgb.header=info.header
                values=np.full((4,4),depth_mm if args.depth_encoding=='16UC1' else (500 if depth_mm else 0),dtype=np.uint16)
                ok,png=cv2.imencode('.png',values)
                if not ok: raise RuntimeError('synthetic PNG encode failed')
                depth=CompressedImage()
                depth.header=info.header
                depth.format=args.depth_encoding+'; compressedDepth png'
                depth.data=struct.pack('<iff',0,0. if args.depth_encoding=='16UC1' else 1000.,0.)+png.tobytes()
                pubs['image_topic'].publish(rgb); pubs['depth_topic'].publish(depth)
                spin(.35)
                return stamp
            stamps=[]
            stamps.append(publish())  # intentional model failure -> no observation/location
            if observations or locations: raise AssertionError('model failure fabricated output')
            stamps.append(publish(depth_mm=0))
            if not observations or observations[-1].localization_status!='UNKNOWN' or locations:
                raise AssertionError('invalid depth did not stay UNKNOWN')
            tf_state['available']=False
            stamps.append(publish())
            if locations: raise AssertionError('missing TF produced a location')
            tf_state['available']=True
            valid_stamp=publish()
            if len(locations)!=1: raise AssertionError(f'expected one valid location, got {len(locations)}')
            location=locations[0]
            point=location.mission_position
            if not np.allclose([point.x,point.y,point.z],[1.5,1.5,2.],atol=1e-6): raise AssertionError(f'wrong XYZ: {point}')
            if location.observation.time.observed_at_ns!=valid_stamp: raise AssertionError('original stamp not preserved')
            if location.frame_ref.map_session_id!=session or location.frame_ref.mission_frame!='test_mission':
                raise AssertionError('frame provenance missing')
            if args.tf_mode=='injected' and tf_state['queries'][-1][2]!=valid_stamp:
                raise AssertionError('TF queried at latest instead of original stamp')
            publish(map_revision=1)
            if len(locations)!=1: raise AssertionError('changed map revision produced a new location')
            publish(map_revision=0)
            if len(locations)!=1: raise AssertionError('replayed lower revision restored permission')
            print(f'PASSED synthetic ROS {args.depth_encoding}: RGB compressed + depth PNG -> XYZ -> TargetLocation '
                  '(1.5, 1.5, 2.0); model failure, invalid depth, map revision change/replay, missing TF blocked. '
                  f'tf_mode={args.tf_mode}. Physical YOLO/camera/robot TF/SLAM/covariance measurement NOT_RUN.')
        finally:
            if executor is not None: executor.shutdown()
            for item in (source,node):
                if item is not None: item.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    return 0


if __name__=='__main__': raise SystemExit(main())
