"""Paired ROI evaluation and bounded asynchronous CSV collection; no ROS imports."""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time

import numpy as np
from amr_source.adapter import camera_intrinsics, frame_meta, locate_center_depth
from amr_source.core import Detection

FIELDS = ['trial_label','pair_id','rgb_stamp_ns','depth_stamp_ns','elapsed_s','roi_size',
          'roi_left','roi_top','roi_right','roi_bottom','roi_pixels','clipped','status',
          'valid_pixels','valid_fraction','spread_m','x_m','y_m','z_m','reference_z_m',
          'error_m','confidence','bbox','skew_ms','infer_ms','compare_ms','receive_age_s',
          'rgb_rx_fps','depth_rx_fps','infer_fps','ui_fps','skipped_rgb']


def evaluate(*, rgb_meta, depth_message, camera_info, binding, alignment, policy,
             rectified, max_info_age_ns, selected, depth_m, sizes, roi_geometry):
    """All sizes see exactly the same RGB stamp, selected bbox, depth and CameraInfo."""
    rows=[]
    intr=None
    issue=None
    try:
        if camera_info is None:
            raise ValueError('CAMERA_INFO_MISSING')
        t=camera_info.header.stamp
        if abs(t.sec*10**9+t.nanosec-rgb_meta.observed_at_ns)>max_info_age_ns:
            raise ValueError('CAMERA_INFO_OLD')
        intr=camera_intrinsics(camera_info,binding,rectified=rectified)
    except Exception as exc:
        issue=str(exc)
    detection=Detection(0,selected.class_label,selected.confidence,selected.bbox,None,'DETECTION_ONLY')
    geometry_ok=bool(intr and alignment.verified and (intr.rectified or intr.undistort is not None)
                     and rgb_meta.width==depth_message.width==intr.width
                     and rgb_meta.height==depth_message.height==intr.height
                     and alignment.rgb_frame_id==rgb_meta.frame_id==intr.frame_id
                     and alignment.depth_frame_id==depth_message.header.frame_id)
    for size in sizes:
        start=time.perf_counter()
        row=dict(roi_size=size,status=issue or 'NOT_EVALUATED',valid_pixels=0,
                 confidence=selected.confidence,bbox=json.dumps(selected.bbox),
                 roi=None,center=None,point_optical=None)
        try:
            if issue: raise ValueError(issue)
            located=locate_center_depth(rgb_meta,depth_message,binding,intr,alignment,detection,policy,radius=size//2)
            row.update(status=located.reason,valid_pixels=located.valid_pixels,point_optical=located.point_optical)
            if located.point_optical:
                row.update(zip(('x_m','y_m','z_m'),located.point_optical))
            if geometry_ok:
                roi,center=roi_geometry(selected.bbox,intr,rgb_meta.width,rgb_meta.height,size//2)
                l,t,r,b=roi; n=max(0,r-l)*max(0,b-t)
                row.update(roi=roi,center=center,roi_left=l,roi_top=t,roi_right=r,roi_bottom=b,
                           roi_pixels=n,clipped=n!=size*size)
                patch=depth_m[t:b,l:r]
                values=patch[np.isfinite(patch)&(patch>0)].astype(np.float64)*policy.scale+policy.offset_m
                values=values[(values>=policy.min_depth_m)&(values<=policy.max_depth_m)]
                row['valid_fraction']=len(values)/n if n else 0.
                row['spread_m']=float(np.ptp(values)) if len(values) else None
        except Exception as exc:
            row.update(status=str(exc),point_optical=None)
        row['compare_ms']=(time.perf_counter()-start)*1000
        rows.append(row)
    return rows


class Collector:
    """Collect only matched pairs received after C. Disk work never blocks inference/GUI."""
    def __init__(self, root, sizes, label, reference, seconds, manifest):
        self.root=Path(root); self.sizes=tuple(sizes); self.label=label
        self.reference=reference; self.seconds=seconds; self.manifest=manifest
        self.lock=threading.Lock(); self.active=None; self.message='C: start ROI collection'
        self.jobs=queue.Queue(maxsize=256); self.error=''; self.dropped=0
        self.thread=threading.Thread(target=self._write,daemon=True); self.thread.start()
    def start(self):
        with self.lock:
            if self.active or getattr(self,'pending_close',None): return
            now=time.monotonic()
            trial=dict(folder=str(self.root/('roi_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))),
                       start=now,stop=None,accepted_pairs=0,dropped_pairs=0,seen=set())
            config=dict(self.manifest,trial_label=self.label,roi_sizes=self.sizes,
                        reference_z_m=self.reference,duration_limit_s=self.seconds,
                        collection_started_utc=datetime.now(timezone.utc).isoformat(),
                        denominator='matched eligible RGB-depth pairs received after collection start',
                        min_valid_pixels=10,min_valid_fraction=.5,max_depth_spread_m=.5)
            try: self.jobs.put_nowait(('open',trial['folder'],config))
            except queue.Full: self.message='Writer busy; collection not started'; return
            self.active=trial; self.message='COLLECTING ROI DATA'
    def stop(self):
        with self.lock:
            trial=self.active
            if trial is None: return
            self.active=None
            trial['stop']=time.monotonic()
            end={k:v for k,v in trial.items() if k!='seen'}
            # Control messages have reserved eventual delivery; never block GUI.
            try: self.jobs.put_nowait(('close',trial['folder'],end))
            except queue.Full:
                self.pending_close=('close',trial['folder'],end)
            self.message=f'Collection ended: {trial["accepted_pairs"]} pairs; dropped {trial["dropped_pairs"]}'
    def tick(self):
        with self.lock:
            active=self.active
            due=active is not None and time.monotonic()-active['start']>=self.seconds
            pending=getattr(self,'pending_close',None)
            if pending:
                try: self.jobs.put_nowait(pending); self.pending_close=None
                except queue.Full: pass
        if due: self.stop()
    def submit(self, rows, context, rgb_received):
        with self.lock:
            trial=self.active; now=time.monotonic()
            if trial is None or now-trial['start']>=self.seconds or rgb_received<trial['start']: return
            key=(context['rgb_stamp_ns'],context['depth_stamp_ns'])
            if key in trial['seen']: return
            trial['seen'].add(key)
            batch=[]
            for row in rows:
                item={k:row.get(k) for k in FIELDS}
                item.update(context,trial_label=self.label,pair_id=f'{key[0]}:{key[1]}',
                            elapsed_s=now-trial['start'],reference_z_m=self.reference)
                if row.get('z_m') is not None and self.reference is not None:
                    item['error_m']=row['z_m']-self.reference
                batch.append(item)
            try:
                self.jobs.put_nowait(('rows',trial['folder'],batch)); trial['accepted_pairs']+=1
            except queue.Full:
                trial['dropped_pairs']+=1; self.dropped+=1
                self.message='CSV queue full: paired sample dropped (see trial metadata)'
    def status(self):
        with self.lock:
            t=self.active
            if t: return f'COLLECT {time.monotonic()-t["start"]:.1f}/{self.seconds:g}s | pairs {t["accepted_pairs"]} | dropped {t["dropped_pairs"]}'
            return self.error or self.message
    def _write(self):
        opened={}
        try:
            while True:
                task=self.jobs.get()
                if task is None: self.jobs.task_done(); break
                kind,folder,data=task
                try:
                    path=Path(folder)
                    if kind=='open':
                        path.mkdir(parents=True,exist_ok=False)
                        (path/'trial.json').write_text(json.dumps(data,indent=2,allow_nan=False))
                        stream=(path/'samples.csv').open('w',newline='')
                        writer=csv.DictWriter(stream,fieldnames=FIELDS,extrasaction='ignore')
                        writer.writeheader(); stream.flush(); opened[folder]=(stream,writer)
                    elif kind=='rows':
                        stream,writer=opened[folder]; writer.writerows(data); stream.flush()
                    elif kind=='close':
                        pair=opened.pop(folder,None)
                        if pair: pair[0].close()
                        (path/'collection_end.json').write_text(json.dumps(data,indent=2,allow_nan=False))
                except Exception as exc:
                    self.error=f'COLLECTION WRITE FAILED: {exc}'
                finally: self.jobs.task_done()
        finally:
            for stream,_ in opened.values(): stream.close()
    def close(self):
        self.stop()
        pending=getattr(self,'pending_close',None)
        if pending: self.jobs.put(pending); self.pending_close=None
        self.jobs.put(None); self.thread.join()
