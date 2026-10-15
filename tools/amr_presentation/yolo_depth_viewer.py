#!/usr/bin/env python3
"""Presentation viewer; upstream AMR selective depth/center ROI. No ROS publishing."""
import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent / 'vendor'))
from amr_source.selective_depth import SelectiveDepth
from amr_source.adapter import SourceBinding, camera_intrinsics, frame_meta, locate_center_depth
from amr_source.core import AlignmentEvidence, DepthPolicy, Detection
from amr_source.person_pipeline import DetectorOutput

SOURCE_SHA = '5669356d5f8b2ff04e895a90d40bd8015f25beba'
BG, CARD, WHITE, MUTED = (23,18,14), (40,32,25), (242,242,238), (176,159,142)
CYAN, GREEN, AMBER, RED = (230,202,64), (123,211,72), (67,192,255), (85,90,246)


def stamp(msg):
    return msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec


def meters(raw, encoding):
    if encoding not in ('16UC1','32FC1'):
        raise ValueError(f'Unsupported depth: {encoding}; use raw 16UC1/32FC1 Image')
    d = raw.astype(np.float32) * (.001 if encoding == '16UC1' else 1.)
    if d.ndim != 2:
        raise ValueError('Depth must have one channel')
    d[~np.isfinite(d) | (d <= 0)] = np.nan
    return d


def color_depth(depth, maximum=5.):
    out = cv2.applyColorMap((np.clip(np.nan_to_num(depth),0,maximum)/maximum*255).astype(np.uint8), cv2.COLORMAP_TURBO)
    out[~np.isfinite(depth)] = 0
    return out


def roi_geometry(box, intr, width, height, radius=5):
    """Visualization only; same bounds as upstream locate_center_depth."""
    x0,y0,x1,y1 = box
    u,v = (x0+x1)/2-.5, (y0+y1)/2-.5
    if intr.undistort is not None:
        xm,ym = (x0+x1)/2,(y0+y1)/2
        edges = [(x0,y0),(xm,y0),(x1,y0),(x1,ym),(x1,y1),(xm,y1),(x0,y1),(x0,ym)]
        points = intr.undistort([(x-.5,y-.5) for x,y in edges]+[(u,v)])
        u,v = points[-1]
        x0,y0 = min(p[0]+.5 for p in points[:-1]),min(p[1]+.5 for p in points[:-1])
        x1,y1 = max(p[0]+.5 for p in points[:-1]),max(p[1]+.5 for p in points[:-1])
    return (max(0,math.floor(x0),round(u)-radius), max(0,math.floor(y0),round(v)-radius),
            min(width,math.ceil(x1),round(u)+radius+1), min(height,math.ceil(y1),round(v)+radius+1)), (u,v)


class Rate:
    def __init__(self): self.times = deque(maxlen=180)
    def tick(self): self.times.append(time.monotonic())
    def value(self):
        t = list(self.times)
        now = time.monotonic()
        t = [x for x in t if now-x < 3]
        return (len(t)-1)/(t[-1]-t[0]) if len(t)>1 and now-t[-1]<1.5 else 0.


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.raw_rgb = self.rgb = self.info = self.result = self.preview_depth = None
        self.depths = deque(maxlen=90)
        self.analysis = None
        self.seq = self.decode_seq = self.skipped = 0
        self.error = ''
        self.rates = {k:Rate() for k in ('rgb','depth','decode','infer','ui')}
        self.stop = threading.Event()


def put(img, text, x, y, size=.6, color=WHITE, thickness=1):
    cv2.putText(img,str(text),(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,thickness,cv2.LINE_AA)


def fit(canvas, img, x, y, w, h):
    if img is None: return None
    ratio = min(w/img.shape[1],h/img.shape[0])
    nw,nh = round(img.shape[1]*ratio),round(img.shape[0]*ratio)
    ox,oy = x+(w-nw)//2,y+(h-nh)//2
    canvas[oy:oy+nh,ox:ox+nw] = cv2.resize(img,(nw,nh),interpolation=cv2.INTER_AREA)
    return ox,oy,nw,nh


def render(shared, a, recorder, note='', clicked=None):
    with shared.lock:
        rgb, result, preview = shared.rgb, shared.result, shared.preview_depth
        if shared.analysis is not None and time.monotonic()-shared.analysis['rgb']['received']<=a.stale:
            result=shared.analysis
        rates = {k:r.value() for k,r in shared.rates.items()}
        skipped, error = shared.skipped, shared.error
    if result and getattr(a,'compare',False):
        chosen=next((r for r in result['details'].get('comparison',[]) if r['roi_size']==a.roi_size),None)
        if chosen:
            result=dict(result,details=dict(result['details'],**chosen))
            result['details']['stage']=5 if chosen['status']=='LOCATED' else 3
            result['details']['spread_text']=f"{chosen['spread_m']:.3f}m" if chosen.get('spread_m') is not None else '--'
            annotated=result['rgb']['image'].copy()
            box=result['details']['selected']['bbox']; x0,y0,x1,y1=box
            cv2.rectangle(annotated,(int(x0),int(y0)),(int(x1),int(y1)),GREEN,2)
            put(annotated,'survivor '+str(round(result['details']['selected']['confidence'],3)),max(0,int(x0)),max(20,int(y0)-8),.5,GREEN)
            cx,cy=round((x0+x1)/2-.5),round((y0+y1)/2-.5); radius=a.roi_size//2
            cv2.rectangle(annotated,(max(int(x0),cx-radius),max(int(y0),cy-radius)),(min(int(x1)-1,cx+radius),min(int(y1)-1,cy+radius)),WHITE,2)
            result['annotated']=annotated
    now = time.monotonic()
    canvas = np.full((900,1600,3), BG, np.uint8)
    put(canvas,'ROKEY  /  SURVIVOR PERCEPTION',32,47,.92,WHITE,2)
    put(canvas,'YOLO  >  SELECTIVE DEPTH  >  CENTER ROI  >  CAMERA XYZ',32,79,.54,MUTED)
    rec_text = 'REC' if recorder.active else 'READY'
    put(canvas,rec_text,1420,46,.8,RED if recorder.active else GREEN,2)
    put(canvas,f'RGB RX {rates["rgb"]:.1f}   DEPTH RX {rates["depth"]:.1f}   DECODE {rates["decode"]:.1f}   INFER {rates["infer"]:.1f}   UI {rates["ui"]:.1f} fps',32,119,.58,CYAN)
    put(canvas,f'Skipped RGB {skipped}  |  Source {SOURCE_SHA[:7]}  |  {Path(a.model).name}',32,148,.48,MUTED)
    titles = ['01  LIVE RGB','02  ANALYZED RGB','03  DEPTH / CENTER ROI']
    for i,title in enumerate(titles):
        x = 24+i*526
        cv2.rectangle(canvas,(x,174),(x+510,600),CARD,-1)
        put(canvas,title,x+14,204,.63,CYAN,2)
    live_age = now-rgb['received'] if rgb else None
    fit(canvas,rgb['image'] if rgb else None,38,225,482,330)
    put(canvas,f'Receive age {live_age*1000:.0f} ms' if rgb else 'WAITING FOR RGB',38,580,.52,RED if live_age and live_age>a.stale else MUTED)
    depth_map = None
    details = {'status':'WAITING_FOR_INFERENCE', 'stage':0}
    if result:
        details = result['details']
        analyzed = result['annotated']
        fit(canvas,analyzed,564,225,482,330)
        age = now-result['rgb']['received']
        put(canvas,f'Result age {age*1000:.0f} ms | infer {result["infer_ms"]:.0f} ms',564,580,.47,RED if age>a.stale else MUTED)
        if result.get('depth_m') is not None:
            depth_map = result['depth_m']
            vis = color_depth(depth_map,a.max_depth)
            roi = details.get('roi')
            if roi:
                l,t,r,b = roi
                cv2.rectangle(vis,(l,t),(r-1,b-1),WHITE,2)
                cv2.drawMarker(vis,tuple(round(v) for v in details['center']),WHITE,cv2.MARKER_CROSS,16,1)
            depth_bounds = fit(canvas,vis,1090,225,482,330)
            if roi and r>l and b>t:
                patch = color_depth(depth_map[t:b,l:r],a.max_depth)
                canvas[440:539,1460:1559] = cv2.resize(patch,(99,99),interpolation=cv2.INTER_NEAREST)
                put(canvas,'ROI zoom',1460,434,.42,WHITE)
            put(canvas,f'MATCHED | stamp delta {details.get("skew_ms",0):.1f} ms',1090,580,.48,MUTED)
        else:
            depth_bounds = None
    else: depth_bounds = None
    if depth_map is None and preview:
        depth_map = preview['image']
        depth_bounds = fit(canvas,color_depth(depth_map,a.max_depth),1090,225,482,330)
        put(canvas,('STALE DEPTH PREVIEW' if now-preview['received']>a.stale else 'LIVE DEPTH / not a selected pair'),1090,580,.45,MUTED)
    if depth_map is None:
        put(canvas,'WAITING FOR DEPTH',1110,400,.6,MUTED)
    if clicked and depth_map is not None:
        u,v = clicked
        if 0<=v<depth_map.shape[0] and 0<=u<depth_map.shape[1]:
            z = depth_map[v,u]
            put(canvas,f'Click ({u},{v}): raw Z {z:.3f} m',1090,544,.48,WHITE,2)
    stage = details.get('stage',0)
    stale_result = bool(result and now-result['rgb']['received']>a.stale)
    if stale_result: stage=0
    labels = ['YOLO DETECT','CONF >= 0.90','DEPTH PAIR',f"{details.get('roi_size',11)} x {details.get('roi_size',11)} ROI",'CAMERA XYZ']
    for i,label in enumerate(labels):
        x=24+i*315
        color = GREEN if i<stage else (AMBER if i==stage else MUTED)
        cv2.rectangle(canvas,(x,622),(x+300,686),CARD,-1)
        cv2.rectangle(canvas,(x,622),(x+5,686),color,-1)
        put(canvas,label,x+17,661,.55,color,2)
    status = ('STALE RESULT / ' if stale_result else '') + details['status']
    put(canvas,status,32,731,.77,GREEN if status=='LOCATED' else AMBER,2)
    selected = details.get('selected')
    if selected:
        put(canvas,f'{selected["class_label"]}  confidence {selected["confidence"]:.3f}  |  bbox center ROI (radius {details.get("roi_size",11)//2} px)',32,766,.55,WHITE)
    if 'point_optical' in details and details['point_optical']:
        x,y,z = details['point_optical']
        put(canvas,f'Camera optical: X {x:+.3f} m   Y {y:+.3f} m   Z {z:.3f} m',32,802,.66,CYAN,2)
    else:
        put(canvas,'Camera XYZ: unavailable until geometry / timing gates pass',32,802,.51,MUTED)
    put(canvas,f'Valid pixels {details.get("valid_pixels","--")} | ROI spread {details.get("spread_text","--")} | depth calibration x{a.depth_scale:g} {a.depth_offset:+g}m',880,730,.48,MUTED)
    put(canvas,'Presentation preview | no TF / target publish / robot motion',880,766,.48,MUTED)
    put(canvas,'Age = PC receive-to-display; not sensor/network latency',880,800,.46,MUTED)
    if getattr(a,'compare',False):
        cv2.rectangle(canvas,(860,700),(1590,824),BG,-1)
        put(canvas,'SAME PAIR  /  ROI        Z (m)          RESULT',880,716,.48,CYAN)
        for i,row in enumerate(details.get('comparison',[])):
            z=f"{row['z_m']:.3f}" if row.get('z_m') is not None else '--'
            text=f"{row['roi_size']:>3} x {row['roi_size']:<3}       {z:<9}  {row['status']}"
            put(canvas,text[:77],880,736+i*18,.43,GREEN if row['status']=='LOCATED' else AMBER)
        put(canvas,f"[ / ] ROI select: {a.roi_size}x{a.roi_size} | C collect | S image | R video | Q quit",32,858,.55,WHITE)
        put(canvas,'ROI comparison preview | no target publication',880,838,.42,MUTED)
    else:
        put(canvas,'[ / ] ROI size    S snapshot    R recording    Q quit',32,858,.55,WHITE)
    put(canvas,(error or recorder.error or note)[:150],32,886,.43,RED if error or recorder.error else MUTED)
    meta = dict(source_sha=SOURCE_SHA,wall_time_utc=datetime.now(timezone.utc).isoformat(),
                monotonic=now,rates=rates,skipped_rgb=skipped,details=details,
                live_receive_age_s=live_age,analyzed_receive_age_s=now-result['rgb']['received'] if result else None,
                rgb_stamp_ns=stamp(result['rgb']['msg']) if result else None,
                no_target_publication=True, result_stale=stale_result,
                depth_stamp_ns=result.get('depth_stamp_ns') if result else None,
                model=a.model, geometry_config=str(getattr(a,'geometry',None)))
    return canvas,meta,depth_map,depth_bounds,result


class Recorder:
    """Writes composed frames directly. Dedicated thread, one-slot mailbox, fixed timeline."""
    def __init__(self, root, fps):
        self.root,self.fps = root,fps
        self.lock = threading.Lock()
        self.latest = None
        self.end = threading.Event()
        self.thread = None
        self.active = False
        self.error = ''
        self.folder = None
    def offer(self, canvas, meta):
        with self.lock: self.latest = (canvas,meta)
    def start(self):
        if self.thread and self.thread.is_alive(): return
        self.folder = self.root / ('video_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        self.folder.mkdir(parents=True)
        self.end.clear(); self.error=''; self.active=True
        self.thread = threading.Thread(target=self.run,daemon=True); self.thread.start()
    def stop(self): self.end.set()
    def run(self):
        writer=None
        try:
            writer=cv2.VideoWriter(str(self.folder/'presentation.avi'),cv2.VideoWriter_fourcc(*'MJPG'),self.fps,(1600,900))
            if not writer.isOpened(): raise RuntimeError('MJPG encoder unavailable')
            start=time.monotonic(); n=0
            with (self.folder/'frames.jsonl').open('w') as log:
                while not self.end.is_set():
                    due=start+n/self.fps
                    if self.end.wait(max(0,due-time.monotonic())): break
                    if time.monotonic()-due > 2:
                        raise RuntimeError('Recording stopped: encoder >2s behind; reduce --record-fps')
                    with self.lock: frame=self.latest
                    if frame is None: continue
                    canvas,meta=frame
                    writer.write(canvas)
                    log.write(json.dumps(dict(meta,video_frame=n,video_time_s=n/self.fps),allow_nan=False)+'\n')
                    n+=1
        except Exception as exc: self.error=str(exc)
        finally:
            if writer is not None: writer.release()
            self.active=False
    def close(self):
        self.stop()
        if self.thread: self.thread.join()


class Snapshots:
    def __init__(self, root):
        self.root=root; self.jobs=queue.Queue(maxsize=2); self.message=''
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start()
    def submit(self, canvas, meta, result):
        try: self.jobs.put_nowait((canvas,meta,result)); self.message='Snapshot queued'
        except queue.Full: self.message='Snapshot queue busy; try again'
    def run(self):
        while True:
            job=self.jobs.get()
            if job is None: self.jobs.task_done(); return
            canvas,meta,result=job
            try:
                folder=self.root/('image_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
                folder.mkdir(parents=True)
                images={'presentation.png':canvas}
                if result:
                    images.update({'rgb.png':result['rgb']['image'],'yolo.png':result['annotated']})
                    if result.get('depth_m') is not None: np.save(folder/'depth_m.npy',result['depth_m'])
                for name,img in images.items():
                    if not cv2.imwrite(str(folder/name),img): raise OSError(f'Cannot write {name}')
                (folder/'metadata.json').write_text(json.dumps(meta,indent=2,allow_nan=False))
                self.message=f'Saved {folder.resolve()}'
            except Exception as exc: self.message=f'Save FAILED: {exc}'
            finally: self.jobs.task_done()
    def close(self): self.jobs.put(None); self.thread.join()


def parse():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--compare',action='store_true',help='Enable paired ROI collection with C')
    p.add_argument('--roi-sizes',type=int,nargs='+',default=[5,11,21],help='1-5 distinct odd sizes, 5..101')
    p.add_argument('--roi-size',type=int,default=11,help='Initially displayed ROI size')
    p.add_argument('--reference-z',type=float,help='Measured optical-axis surface Z in metres, not slant range')
    p.add_argument('--trial-label',default='stationary_target')
    p.add_argument('--compare-seconds',type=float,default=30.)
    p.add_argument('--robot',default='robot6')
    p.add_argument('--rgb-topic',required=True)
    p.add_argument('--depth-topic',required=True,help='Raw sensor_msgs/msg/Image only')
    p.add_argument('--camera-info-topic',required=True,help='CameraInfo for the RGB geometry')
    p.add_argument('--compressed',action='store_true',help='RGB CompressedImage')
    p.add_argument('--model',required=True,help='Local survivor best.pt')
    p.add_argument('--device',default='cpu'); p.add_argument('--imgsz',type=int,default=640)
    p.add_argument('--geometry',type=Path,help='Measured geometry JSON; omitted = unverified')
    p.add_argument('--depth-scale',type=float,default=1.); p.add_argument('--depth-offset',type=float,default=0.)
    p.add_argument('--max-depth',type=float,default=5.)
    p.add_argument('--slop-ms',type=float,default=20.); p.add_argument('--wait-depth',type=float,default=1.5)
    p.add_argument('--stale',type=float,default=2.)
    p.add_argument('--ui-fps',type=float,default=20.); p.add_argument('--decode-fps',type=float,default=30.)
    p.add_argument('--depth-preview-fps',type=float,default=5.,help='0 disables extra full-depth preview decoding')
    p.add_argument('--record-fps',type=float,default=15.); p.add_argument('--torch-threads',type=int,default=2)
    p.add_argument('--output',type=Path,default=Path('captures'))
    a=p.parse_args()
    if not 1<=len(a.roi_sizes)<=5 or len(set(a.roi_sizes))!=len(a.roi_sizes) or any(n<5 or n>101 or n%2==0 for n in a.roi_sizes):
        p.error('--roi-sizes: 1-5 distinct odd sizes from 5 to 101')
    a.roi_sizes=sorted(a.roi_sizes)
    if a.roi_size not in a.roi_sizes: a.roi_size=a.roi_sizes[0]
    if a.reference_z is not None and (not math.isfinite(a.reference_z) or a.reference_z<=0): p.error('reference Z must be positive and finite')
    if not math.isfinite(a.compare_seconds) or a.compare_seconds<=0: p.error('compare duration must be positive')
    for name in ('depth_scale','max_depth','wait_depth','stale','ui_fps','decode_fps','record_fps'):
        if not math.isfinite(getattr(a,name)) or getattr(a,name)<=0: p.error(f'Invalid {name}')
    if not math.isfinite(a.depth_offset) or not math.isfinite(a.slop_ms) or a.slop_ms<0 or not math.isfinite(a.depth_preview_fps) or a.depth_preview_fps<0 or a.torch_threads<1 or a.imgsz<32 or a.max_depth<=.2: p.error('Invalid parameter')
    if not Path(a.model).is_file(): p.error('--model must point to an existing local detection model')
    return a


def configuration(a):
    g=dict(rgb_stamp_semantics='UNKNOWN',depth_stamp_semantics='UNKNOWN',clock_domain_id='UNCONFIGURED',
           calibration_id='UNCONFIGURED',alignment_evidence_id='UNCONFIGURED',rgb_optical_frame='UNCONFIGURED',
           depth_optical_frame='UNCONFIGURED',rgb_depth_alignment_verified=False,rgb_rectified=False,camera_info_max_age_ns=10**9)
    if a.geometry: g.update(json.loads(a.geometry.read_text()))
    if type(g['rgb_rectified']) is not bool or type(g['rgb_depth_alignment_verified']) is not bool: raise ValueError('geometry boolean fields must be true/false')
    if type(g['camera_info_max_age_ns']) is not int or g['camera_info_max_age_ns']<1: raise ValueError('Invalid CameraInfo age')
    binding=SourceBinding('robot_a','presentation_rgbd',str(uuid4()),g['clock_domain_id'],g['rgb_stamp_semantics'],g['depth_stamp_semantics'],g['calibration_id'],'UNKNOWN',None)
    alignment=AlignmentEvidence(g['alignment_evidence_id'],g['calibration_id'],g['rgb_optical_frame'],g['depth_optical_frame'],g['rgb_depth_alignment_verified'])
    policy=DepthPolicy(round(a.slop_ms*1e6),.2,a.max_depth,10,.5,.5,a.depth_scale,a.depth_offset)
    return g,binding,alignment,policy


def run(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo
    from cv_bridge import CvBridge
    from ultralytics import YOLO
    import torch
    torch.set_num_threads(a.torch_threads)
    cv2.setNumThreads(1)
    g,binding,alignment,policy=configuration(a)
    model=YOLO(a.model)
    if model.task!='detect' or list(model.names.values())!=['survivor']:
        raise ValueError('Use the project single-class survivor detection model')
    a.output.mkdir(parents=True,exist_ok=True)
    manifest=dict(source_sha=SOURCE_SHA,model_sha256=hashlib.sha256(Path(a.model).read_bytes()).hexdigest(),
                  arguments={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},geometry=g)
    (a.output/('session_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')).write_text(json.dumps(manifest,indent=2))
    from roi_comparison import Collector, evaluate
    collector=Collector(a.output,a.roi_sizes,a.trial_label,a.reference_z,a.compare_seconds,manifest) if a.compare else None
    state=Shared(); bridge=CvBridge(); rclpy.init(args=[]); node=Node('amr_presentation_viewer')
    qos=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    def rgb_callback(msg):
        with state.lock:
            state.seq+=1; state.raw_rgb=(state.seq,msg,time.monotonic()); state.rates['rgb'].tick()
    def depth_callback(msg):
        with state.lock:
            state.depths.append((msg,time.monotonic())); state.rates['depth'].tick()
    def info_callback(msg):
        with state.lock: state.info=msg
    node.create_subscription(CompressedImage if a.compressed else Image,a.rgb_topic,rgb_callback,qos)
    node.create_subscription(Image,a.depth_topic,depth_callback,qos)
    node.create_subscription(CameraInfo,a.camera_info_topic,info_callback,qos)
    def ros_loop():
        try:
            while not state.stop.is_set() and rclpy.ok(): rclpy.spin_once(node,timeout_sec=.05)
        except Exception as exc:
            with state.lock: state.error=f'ROS: {exc}'
    def decode_loop():
        last=0; last_d=None; next_depth=0.
        while not state.stop.is_set():
            start=time.monotonic()
            with state.lock: raw=state.raw_rgb; d=state.depths[-1] if state.depths else None
            try:
                if raw and raw[0]!=last:
                    seq,msg,received=raw; last=seq
                    image=bridge.compressed_imgmsg_to_cv2(msg,'bgr8') if a.compressed else bridge.imgmsg_to_cv2(msg,'bgr8')
                    with state.lock:
                        state.rgb=dict(seq=seq,msg=msg,image=image.copy(),received=received)
                        state.rates['decode'].tick()
                if a.depth_preview_fps>0 and d and d[0] is not last_d and start>=next_depth:
                    image=meters(bridge.imgmsg_to_cv2(d[0],'passthrough'),d[0].encoding)
                    last_d=d[0]; next_depth=start+1/a.depth_preview_fps
                    with state.lock: state.preview_depth=dict(image=image,received=d[1])
            except Exception as exc:
                with state.lock: state.error=f'Decode: {exc}'
            state.stop.wait(max(.001,1/a.decode_fps-(time.monotonic()-start)))
    def infer_loop():
        requests=SelectiveDepth(capacity=90,slop_ns=policy.max_stamp_skew_ns,wait_s=a.wait_depth,confidence=.9,label='survivor')
        last_seq=0; last_depth=None
        while not state.stop.is_set():
            with state.lock: rgb=state.rgb; info=state.info; ds=list(state.depths)
            try:
                # Messages are cached undecoded; source stamps, not arrival times, select pairs.
                fresh=[]
                for dm,received in reversed(ds):
                    if dm is last_depth: break
                    fresh.append((dm,received))
                for dm,received in reversed(fresh):
                    if stamp(dm)>0: requests.offer_depth(stamp(dm),dm,received)
                if ds: last_depth=ds[-1][0]
                if rgb and rgb['seq']!=last_seq and time.monotonic()-rgb['received']<=a.stale:
                    with state.lock: state.skipped+=max(0,rgb['seq']-last_seq-1)
                    last_seq=rgb['seq']; start=time.monotonic()
                    y=model.predict(rgb['image'],conf=.5,device=a.device,imgsz=a.imgsz,verbose=False)[0]
                    infer_ms=(time.monotonic()-start)*1000
                    outputs=[DetectorOutput(y.names[int(b.cls.item())],float(b.conf.item()),tuple(b.xyxy[0].cpu().tolist()),None) for b in y.boxes]
                    annotated=y.plot()
                    h,w=rgb['image'].shape[:2]
                    eligible=[o for o in outputs if o.confidence>=.9 and 0<o.bbox[0]<o.bbox[2]<w and 0<o.bbox[1]<o.bbox[3]<h]
                    details=dict(status='WAIT_DEPTH' if eligible else ('NO_ELIGIBLE_TARGET' if outputs else 'SEARCHING'),stage=2 if eligible else (1 if outputs else 0),detections=[asdict(o) for o in outputs])
                    if eligible: details['selected']=asdict(max(eligible,key=lambda o:o.confidence))
                    result=dict(rgb=rgb,annotated=annotated,infer_ms=infer_ms,details=details)
                    if stamp(rgb['msg'])>0:
                        requests.offer_rgb(stamp(rgb['msg']),w,h,outputs,(result,info),time.monotonic())
                    else: details.update(status='ZERO_RGB_STAMP',stage=1)
                    with state.lock: state.result=result; state.rates['infer'].tick()
                request=requests.take(time.monotonic())
                if request:
                    (base,ci), selected, dm, delta=request
                    display_size=a.roi_size
                    details=dict(base['details'],selected=asdict(selected),skew_ms=delta/1e6,stage=3,status='CHECKING_GEOMETRY',roi_size=display_size)
                    result=dict(base,details=details,depth_stamp_ns=stamp(dm))
                    result['annotated']=base['annotated'].copy()
                    x0,y0,x1,y1=selected.bbox
                    cx,cy=round((x0+x1)/2-.5),round((y0+y1)/2-.5)
                    radius=display_size//2
                    cv2.rectangle(result['annotated'],(cx-radius,cy-radius),(cx+radius,cy+radius),WHITE,2)
                    # Full decoding here is for the presentation depth panel, not localization.
                    result['depth_m']=meters(bridge.imgmsg_to_cv2(dm,'passthrough'),dm.encoding)
                    rm=base['rgb']['msg']; h,w=base['rgb']['image'].shape[:2]
                    rgb_meta=SimpleNamespace(header=rm.header,width=w,height=h)
                    comparison_start=time.perf_counter()
                    compared=evaluate(rgb_meta=frame_meta(rgb_meta,binding,binding.rgb_stamp_semantics,'rgb'),
                        depth_message=dm,camera_info=ci,binding=binding,alignment=alignment,policy=policy,
                        rectified=g['rgb_rectified'],max_info_age_ns=g['camera_info_max_age_ns'],selected=selected,
                        depth_m=result['depth_m'],sizes=a.roi_sizes if a.compare else [display_size],roi_geometry=roi_geometry)
                    chosen=next(row for row in compared if row['roi_size']==display_size)
                    details.update(chosen,stage=5 if chosen['status']=='LOCATED' else 3)
                    details['spread_text']=f"{chosen['spread_m']:.3f}m" if chosen.get('spread_m') is not None else '--'
                    details['comparison']=compared if a.compare else []
                    details['compare_total_ms']=(time.perf_counter()-comparison_start)*1000
                    if collector:
                        with state.lock:
                            rates={k:r.value() for k,r in state.rates.items()}; skipped=state.skipped
                        collector.submit(compared,dict(rgb_stamp_ns=stamp(rm),depth_stamp_ns=stamp(dm),skew_ms=delta/1e6,
                            infer_ms=base['infer_ms'],receive_age_s=time.monotonic()-base['rgb']['received'],
                            rgb_rx_fps=rates['rgb'],depth_rx_fps=rates['depth'],infer_fps=rates['infer'],
                            ui_fps=rates['ui'],skipped_rgb=skipped),base['rgb']['received'])
                    with state.lock: state.analysis=result
                requests.expire(time.monotonic())
            except Exception as exc:
                with state.lock: state.error=f'Inference / pairing: {exc}'
            state.stop.wait(.005)
    threads=[threading.Thread(target=f,daemon=True) for f in (ros_loop,decode_loop,infer_loop)]
    recorder=Recorder(a.output,a.record_fps); snapshots=Snapshots(a.output)
    window='ROKEY AMR Presentation'; clicked=None; click_state={}
    def mouse(event,x,y,flags,param):
        nonlocal clicked
        bounds=click_state.get('bounds'); d=click_state.get('depth')
        if event==cv2.EVENT_LBUTTONDOWN and bounds and d is not None:
            ox,oy,w,h=bounds
            if ox<=x<ox+w and oy<=y<oy+h:
                clicked=(min(d.shape[1]-1,int((x-ox)*d.shape[1]/w)),min(d.shape[0]-1,int((y-oy)*d.shape[0]/h)))
    try:
        cv2.namedWindow(window,cv2.WINDOW_AUTOSIZE); cv2.setMouseCallback(window,mouse)
        for t in threads: t.start()
        print(f'Output: {a.output.resolve()}\nS snapshot | R record start/stop | Q exit')
        while rclpy.ok():
            start=time.monotonic()
            if collector: collector.tick()
            canvas,meta,d,bounds,result=render(state,a,recorder,(collector.status() if collector else snapshots.message),clicked)
            recorder.offer(canvas,meta)
            click_state.update(bounds=bounds,depth=d)
            cv2.imshow(window,canvas)
            with state.lock: state.rates['ui'].tick()
            key=cv2.waitKey(1)&255
            if key in (ord('q'),27) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1: break
            if key in (ord('['),ord(']')):
                index=a.roi_sizes.index(a.roi_size)
                a.roi_size=a.roi_sizes[(index+(-1 if key==ord('[') else 1))%len(a.roi_sizes)]
            if key==ord('c') and collector:
                if collector.active: collector.stop()
                else: collector.start()
            if key==ord('s'): snapshots.submit(canvas,meta,result)
            if key==ord('r'):
                if recorder.active: recorder.stop()
                else: recorder.start()
            state.stop.wait(max(0,1/a.ui_fps-(time.monotonic()-start)))
    finally:
        state.stop.set()
        for t in threads:
            if t.ident is not None: t.join(timeout=5)
        if collector: collector.close()
        recorder.close(); snapshots.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__=='__main__':
    run(parse())
