#!/usr/bin/env python3
"""Smoke-check an existing feed-forward alpha ONNX without changing any bytes.
Requires numpy + onnxruntime in the selected Python environment. Not a balance,
training-normalizer, recurrent-policy or actuator calibration validation.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys

def digest(p:Path)->str:
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1<<20),b''):h.update(chunk)
    return h.hexdigest()

def main()->int:
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('onnx',type=Path);a=ap.parse_args()
    import numpy as np
    import onnxruntime as ort
    before=digest(a.onnx)
    s=ort.InferenceSession(str(a.onnx),providers=['CPUExecutionProvider'])
    ins=s.get_inputs();outs=s.get_outputs()
    if len(ins)!=1 or len(outs)!=1:
        raise ValueError('This helper only checks feed-forward policies. Keep the daemon\'s existing recurrent loader for multi-I/O graphs.')
    if len(ins[0].shape)!=2 or ins[0].shape[1]!=61 or len(outs[0].shape)!=2 or outs[0].shape[1]!=14:
        raise ValueError(f'Expected [1,61] -> [1,14], got {ins[0].shape} -> {outs[0].shape}')
    if ins[0].type!='tensor(float)':raise ValueError(f'Expected float32 input, got {ins[0].type}')
    obs=np.zeros((1,61),dtype=np.float32);obs[0,5]=-1.0
    for _ in range(10):
        action=s.run([outs[0].name],{ins[0].name:obs})[0]
        if action.shape!=(1,14) or not np.isfinite(action).all():raise ValueError('Invalid action output')
        # No fictitious head motion; only the raw action-history block is updated.
        obs[0,34:48]=action[0]
    after=digest(a.onnx)
    if before!=after:raise RuntimeError('ONNX changed during validation')
    print(json.dumps({'sha256':before,'input_shape':ins[0].shape,'output_shape':outs[0].shape,
                      'finite_inference_steps':10,'weight_file_unchanged':True,
                      'balance_validated':False,'normalization_semantics_validated':False},indent=2))
    return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,ValueError,RuntimeError,ImportError) as e:
        print(f'ERROR: {e}',file=sys.stderr);raise SystemExit(1)
