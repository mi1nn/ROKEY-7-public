#!/usr/bin/env python3
"""Run the Git-selected AMR best.pt on local image files and save real results.

No ROS/robot connection. Resolves weights/hash/classes/imgsz from the repository
manifest, verifies SHA before loading, and never substitutes downloaded weights.
Use original independent images for evaluation. An already annotated image is
only a software smoke input, not fresh camera/accuracy evidence.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
for package in ('rescue_contracts','rescue_mapping','rescue_perception'):
    sys.path.insert(0,str(ROOT/'src'/package))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,nargs='+',required=True,help='local source images')
    parser.add_argument('--manifest',type=Path,default=ROOT/'assets/manifest/yolo26n-survivor-v2.json')
    parser.add_argument('--weights',type=Path,help='override path; manifest SHA/class checks still apply')
    parser.add_argument('--output-dir',type=Path,required=True,help='new directory; existing directories are refused')
    parser.add_argument('--device',default='cpu',help='cpu or explicit CUDA device; no device fallback')
    parser.add_argument('--confidence',type=float,default=.25)
    parser.add_argument('--input-note',default='Unverified local image; not an accuracy evaluation')
    args=parser.parse_args(argv)
    if not math.isfinite(args.confidence) or not 0<=args.confidence<=1:
        parser.error('confidence must be finite in [0,1]')
    try:
        from rescue_perception.amr_camera_support import validate_labels,verify_model
        from rescue_perception.amr_target_support import inference_from_result
        from rescue_perception.core import ModelManifest
        manifest=json.loads(args.manifest.read_text())
        selected=manifest['uploaded_model']
        weights=args.weights if args.weights is not None else ROOT/selected['path']
        path,digest=verify_model(weights,selected['sha256'])
        classes=tuple(selected['class_names'])
        if len(classes)!=1:
            raise ValueError('AMR manifest must select one detection class')
        imgsz=selected['imgsz']
        if type(imgsz) is not int or imgsz<=0:
            raise ValueError('manifest imgsz must be a positive integer')
        model_manifest=ModelManifest(manifest['selection']['experiment'],digest,classes)
        import cv2
        import torch
        config_dir=Path(os.environ.get('YOLO_CONFIG_DIR',str(ROOT/'.artifacts/ultralytics-config'))).expanduser().resolve()
        config_dir.mkdir(parents=True,exist_ok=True)
        os.environ['YOLO_CONFIG_DIR']=str(config_dir)
        import ultralytics
        if args.device!='cpu' and not torch.cuda.is_available():
            raise ValueError(f'requested device {args.device!r}; CUDA is unavailable (no CPU fallback)')
        model=ultralytics.YOLO(path)
        validate_labels(model.names,classes[0])
        # Decode all inputs before creating an output directory.
        inputs=[]
        for image_path in args.image:
            frame=cv2.imread(str(image_path),cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError(f'cannot decode local input: {image_path}')
            inputs.append((image_path,frame))
        args.output_dir.mkdir(parents=True,exist_ok=False)
        record={'measured_at_utc':datetime.now(timezone.utc).isoformat(),
                'model_sha256':digest,'model_id':model_manifest.model_id,'class_names':list(classes),
                'imgsz':imgsz,'confidence':args.confidence,'device':args.device,'input_note':args.input_note,
                'versions':{'ultralytics':ultralytics.__version__,'torch':torch.__version__,'opencv':cv2.__version__},
                'images':[]}
        for index,(image_path,frame) in enumerate(inputs):
            started=time.perf_counter()
            results=model.predict(frame,imgsz=imgsz,conf=args.confidence,device=args.device,verbose=False)
            elapsed_ms=(time.perf_counter()-started)*1000
            if len(results)!=1:
                raise ValueError('expected one YOLO result per image')
            result=results[0]
            inference=inference_from_result(result,model_manifest)
            filename=f'{index:03d}-{image_path.stem}-annotated.png'
            if not cv2.imwrite(str(args.output_dir/filename),result.plot(labels=True,conf=True)):
                raise RuntimeError('annotated output image could not be written')
            item={'input_name':image_path.name,'input_width':frame.shape[1],'input_height':frame.shape[0],
                  'annotated_image':filename,'predict_wall_ms':elapsed_ms,'yolo_speed_ms':result.speed,
                  'detections':[asdict(detection) for detection in inference.outputs]}
            record['images'].append(item)
            print(json.dumps(item,ensure_ascii=False,allow_nan=False))
        output=args.output_dir/'inference.json'
        output.write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print(f'REAL YOLO INFERENCE PASSED model_sha256={digest}; results={output}')
        return 0
    except (KeyboardInterrupt,):
        return 130
    except Exception as exc:
        print(f'YOLO inference failed: {type(exc).__name__}: {exc}',file=sys.stderr)
        return 2


if __name__=='__main__':
    raise SystemExit(main())
