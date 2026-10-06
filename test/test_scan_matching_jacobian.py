#!/usr/bin/env python3
"""Check the production scan-matching expressions against pose derivatives."""
from pathlib import Path
import re
from types import SimpleNamespace
import numpy as np


def rotation(angles):
    sr, sp, sy = np.sin(angles)
    cr, cp, cy = np.cos(angles)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def test_point_to_plane_jacobian_matches_numeric_pose_derivatives():
    source = (Path(__file__).resolve().parents[1]/'src/mapOptmization.cpp').read_text()
    # The preceding commented LOAM equations use a different axis ordering.
    source = 'float arx = (-srx' + source.split('float arx = (-srx', 1)[1]
    expressions = {name: re.search(r'float '+name+r'\s*=\s*(.*?);', source, re.S)
                   .group(1).replace('\n', ' ') for name in ('arz', 'ary', 'arx')}
    generator = np.random.default_rng(42)
    maximum = 0.
    for _ in range(100):
        angles = generator.uniform(-1.4, 1.4, 3)
        point = generator.normal(size=3)*5
        normal = generator.normal(size=3)
        context = dict(pointOri=SimpleNamespace(**dict(zip('xyz', point))),
                       coeff=SimpleNamespace(**dict(zip('xyz', normal))),
                       srx=np.sin(angles[2]), crx=np.cos(angles[2]),
                       sry=np.sin(angles[1]), cry=np.cos(angles[1]),
                       srz=np.sin(angles[0]), crz=np.cos(angles[0]))
        actual = np.array([eval(expressions[name], {'__builtins__': {}}, context)
                           for name in ('arz', 'ary', 'arx')])
        expected = []
        for axis in range(3):
            step = np.zeros(3)
            step[axis] = 1e-6
            expected.append(normal @ ((rotation(angles+step)-rotation(angles-step)) @ point)/2e-6)
        maximum = max(maximum, float(np.max(np.abs(actual-expected))))
    assert maximum < 1e-6, f'Scan-matching Jacobian error {maximum}'
