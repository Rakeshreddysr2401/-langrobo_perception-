#!/usr/bin/env python3
"""Build the rover's URDF and CAD from params.yaml, then check the repo for drift.

    description/build            # regenerate rover.urdf + cad/, run the checks
    description/build --check    # checks only, write nothing (exit 1 on drift)
    build.py --rsp FILE [--lidar-yaw RAD]
                                 # robot_state_publisher params, for ./rover

ONE SOURCE. params.yaml holds raw measurements, the way they were taken. This
script derives every frame position from them (derive()), then writes:

    rover.urdf        base_link, chassis, wheels, the D555 and C1 cases, and the
                      camera0_link / laser frames. No <inertial> until weighed.
    cad/rover.step    the same parts as solids (build123d), for FreeCAD/Onshape
    cad/*.stl         one mesh per part

Nothing here invents a number: a part whose inputs are null is not drawn, and
the build lists what is still missing.

LIVE TF. ./rover runs `build.py --rsp` every time it brings up the pose or lidar
layer, so robot_state_publisher always serves the URDF derived from params.yaml
as it is NOW -- never a stale generated file. That --rsp path needs only PyYAML
(the system python3 has it), not build123d.

DRIFT. Some numbers are still copied by hand: nav2.yaml's footprint,
vo_node's fallback camera defaults, the firmware, the Phase 1 odometry
constants and the pivot scripts.
The check reads each copy and fails if it disagrees with what this derives, so a
new measurement cannot land in one place and not the others.
"""
import math
import re
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
TOL = 1e-4
# assumed = the owner said to use a placeholder until it is measured. Listed on
# every build so it cannot quietly become a fact.
STATUSES = ('measured', 'datasheet', 'calibrated', 'described', 'assumed', 'unknown')


# ── params ──────────────────────────────────────────────────────────────────
def entries(p, prefix=''):
    for k, v in p.items():
        if isinstance(v, dict) and 'status' in v and 'value' in v:
            yield f'{prefix}{k}', v
        elif isinstance(v, dict):
            yield from entries(v, f'{prefix}{k}.')


def load():
    p = yaml.safe_load((HERE / 'params.yaml').read_text())
    bad = []
    for path, e in entries(p):
        st, v = e.get('status'), e.get('value')
        if st not in STATUSES:
            bad.append(f'{path}: bad status {st!r}')
        elif (st == 'unknown') != (v is None):
            bad.append(f'{path}: value {v} with status {st}')
    for c in p.get('components') or []:
        if not (len(c.get('size') or []) == 3 and len(c.get('centre') or []) == 3):
            bad.append(f'component {c.get("name")}: needs size [x,y,z] and centre [x,y,z]')
    if bad:
        sys.exit('params.yaml is inconsistent:\n  ' + '\n  '.join(bad))
    return p


def derive(p):
    """Every position in base_link, from the raw measurements."""
    v = {k: e['value'] for k, e in entries(p)}
    g = {}
    fa, ra = v['wheels.front_axle_from_nose'], v['wheels.rear_axle_from_nose']
    g['nose_x'] = (fa + ra) / 2                   # base_link is midway between the axles
    g['tail_x'] = g['nose_x'] - v['frame.length']
    g['frame_cx'] = g['nose_x'] - v['frame.length'] / 2
    g['frame_top'] = v['frame.bottom_z'] + v['frame.height']
    g['wheelbase'] = ra - fa
    g['wheel_x'] = g['nose_x'] - fa               # front axle; rear is -wheel_x
    g['wheel_y'] = v['wheels.track'] / 2
    g['wheel_z'] = v['wheels.axle_z']
    g['wheel_r'] = v['wheels.diameter'] / 2

    glass = g['nose_x'] - v['camera.glass_inset']
    g['cam_case'] = (glass - v['camera.case_depth'] / 2, v['camera.case_y'], v['camera.lens_z'])
    # camera0_link = the left IR imager, half a baseline away from infra2. The
    # geometry gives it from the tape and the datasheet; the VO lever-arm fit
    # measures it directly, and wins when present (params.yaml explains).
    g['cam_geom'] = (glass, v['camera.case_y'] - v['camera.infra2_side'] * v['camera.baseline'] / 2)
    g['cam'] = (v.get('camera.vo_lever_x') or g['cam_geom'][0],
                v.get('camera.vo_lever_y') if v.get('camera.vo_lever_y') is not None else g['cam_geom'][1],
                v['camera.lens_z'])
    g['cam_pitch'] = v['camera.pitch']
    g['cam_roll'] = v['camera.roll']
    g['cam_yaw'] = v['camera.yaw']

    lx = g['nose_x'] - v['lidar.front_gap'] - v['lidar.case_size'] / 2
    g['lidar_case'] = (lx, v['lidar.y'], g['frame_top'] + v['lidar.case_height'] / 2)
    g['lidar'] = (lx, v['lidar.y'], g['frame_top'] + v['lidar.plane_above_base'])
    g['lidar_yaw'] = v['lidar.yaw']

    # the outer envelope: whatever sticks out furthest, in each direction
    tyre_out = g['wheel_y'] + v['wheels.width'] / 2
    g['env_front'] = max(g['nose_x'], g['wheel_x'] + g['wheel_r'], glass)
    g['env_rear'] = max(-g['tail_x'], g['wheel_x'] + g['wheel_r'])
    g['env_side'] = max(v['frame.width'] / 2, tyre_out, v['camera.case_width'] / 2)
    g['v'] = v
    return g


# ── URDF ────────────────────────────────────────────────────────────────────
def f(x):
    s = f'{x:.4f}'.rstrip('0').rstrip('.')
    return '0' if s in ('', '-0') else s


def origin(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return f'<origin xyz="{" ".join(map(f, xyz))}" rpy="{" ".join(map(f, rpy))}"/>'


def link(name, geometry=None, org=None, colour=None):
    if geometry is None:
        return f'  <link name="{name}"/>\n'
    o = org or origin()
    mat = f'\n      <material name="{colour[0]}"><color rgba="{colour[1]}"/></material>' if colour else ''
    return (f'  <link name="{name}">\n'
            f'    <visual>\n      {o}\n      <geometry>{geometry}</geometry>{mat}\n    </visual>\n'
            f'    <collision>\n      {o}\n      <geometry>{geometry}</geometry>\n    </collision>\n'
            f'  </link>\n')


def joint(name, child, xyz, rpy=(0, 0, 0)):
    return (f'  <joint name="{name}" type="fixed">\n'
            f'    <parent link="base_link"/><child link="{child}"/>\n'
            f'    {origin(xyz, rpy)}\n  </joint>\n')


def box(sx, sy, sz):
    return f'<box size="{f(sx)} {f(sy)} {f(sz)}"/>'


ORANGE = ('orange', '1 0.48 0.09 1')
BLACK = ('black', '0.13 0.13 0.16 1')
GREY = ('grey', '0.6 0.6 0.65 1')


def urdf(p, g):
    v = g['v']
    out = ['<?xml version="1.0"?>\n'
           '<!-- GENERATED by description/build from params.yaml. Do not edit. -->\n'
           '<robot name="rover">\n\n'
           '  <!-- on the floor, midway between the four wheel contact patches -->\n'
           '  <link name="base_link"/>\n\n']

    out.append(link('chassis_link', box(v['frame.length'], v['frame.width'], v['frame.height']),
                    colour=ORANGE))
    out.append(joint('chassis_joint', 'chassis_link',
                     (g['frame_cx'], 0, v['frame.bottom_z'] + v['frame.height'] / 2)))

    # fixed for now; continuous once /wheel_ticks -> /joint_states exists
    cyl = f'<cylinder radius="{f(g["wheel_r"])}" length="{f(v["wheels.width"])}"/>'
    for name, sx, sy in (('wheel_lf', 1, 1), ('wheel_lr', -1, 1), ('wheel_rf', 1, -1), ('wheel_rr', -1, -1)):
        out.append('\n' + link(name, cyl, origin(rpy=(math.pi / 2, 0, 0)), BLACK))  # axis z -> y
        out.append(joint(f'{name}_joint', name, (sx * g['wheel_x'], sy * g['wheel_y'], g['wheel_z'])))

    out.append('\n  <!-- the D555 case, and camera0_link = its LEFT IR imager. realsense2_camera\n'
               '       publishes the optical + IMU frames below camera0_link itself. -->\n')
    out.append(link('camera_case', box(v['camera.case_depth'], v['camera.case_width'], v['camera.case_height']),
                    colour=GREY))
    out.append(joint('camera_case_joint', 'camera_case', g['cam_case']))
    out.append(link('camera0_link'))
    out.append(joint('camera_joint', 'camera0_link', g['cam'], (g['cam_roll'], g['cam_pitch'], g['cam_yaw'])))

    out.append('\n  <!-- the RPLidar C1 case, square to the body, and `laser`, the frame /scan is\n'
               '       stamped in: at the laser plane, yawed to where the zero beam points. -->\n')
    out.append(link('lidar_case', box(v['lidar.case_size'], v['lidar.case_size'], v['lidar.case_height']),
                    colour=BLACK))
    out.append(joint('lidar_case_joint', 'lidar_case', g['lidar_case']))
    out.append(link('laser'))
    out.append(joint('laser_joint', 'laser', g['lidar'], (0, 0, g['lidar_yaw'])))

    for c in p.get('components') or []:
        out.append(f'\n  <!-- {c["name"]}: {c.get("status", "")} {c.get("date", "")} -->\n')
        out.append(link(c['name'], box(*c['size']), colour=GREY))
        out.append(joint(f'{c["name"]}_joint', c['name'], c['centre']))

    out.append('\n</robot>\n')
    return ''.join(out)


# ── CAD ─────────────────────────────────────────────────────────────────────
def cad(p, g):
    """One solid per part, in millimetres. Returns the part names, or None."""
    try:
        from build123d import Box, Compound, Cylinder, Location, Rot, export_step, export_stl
    except ImportError:
        print('  cad: build123d missing -- run through description/build, which uses ~/.venvs/cad')
        return None
    v, mm = g['v'], 1000.0

    def at(xyz, solid):
        return Location(tuple(c * mm for c in xyz)) * solid

    parts = {
        'chassis': at((g['frame_cx'], 0, v['frame.bottom_z'] + v['frame.height'] / 2),
                      Box(v['frame.length'] * mm, v['frame.width'] * mm, v['frame.height'] * mm)),
        'camera_d555': at(g['cam_case'], Box(v['camera.case_depth'] * mm, v['camera.case_width'] * mm,
                                             v['camera.case_height'] * mm)),
        'lidar_c1': at(g['lidar_case'], Box(v['lidar.case_size'] * mm, v['lidar.case_size'] * mm,
                                            v['lidar.case_height'] * mm)),
    }
    for name, sx, sy in (('wheel_lf', 1, 1), ('wheel_lr', -1, 1), ('wheel_rf', 1, -1), ('wheel_rr', -1, -1)):
        parts[name] = at((sx * g['wheel_x'], sy * g['wheel_y'], g['wheel_z']),
                         Rot(90, 0, 0) * Cylinder(g['wheel_r'] * mm, v['wheels.width'] * mm))
    for c in p.get('components') or []:
        parts[c['name']] = at(c['centre'], Box(*(s * mm for s in c['size'])))

    d = HERE / 'cad'
    d.mkdir(exist_ok=True)
    for old in d.glob('*'):
        old.unlink()
    for name, s in parts.items():
        s.label = name
        export_stl(s, str(d / f'{name}.stl'))
    export_step(Compound(children=list(parts.values()), label='rover'), str(d / 'rover.step'))
    return list(parts)


# ── drift ───────────────────────────────────────────────────────────────────
def grab(rel, pattern, n=1):
    m = re.search(pattern, (REPO / rel).read_text(), re.M)
    if not m:
        return None
    return tuple(float(m.group(i + 1)) for i in range(n)) if n > 1 else float(m.group(1))


def drift(g):
    v = g['v']
    F, R, S = g['env_front'], g['env_rear'], g['env_side']
    num = r'([-\d.]+)'
    fp = grab('phase3/config/nav2.yaml',
              r'^\s*footprint:\s*"\[' + r',\s*'.join([rf'\[{num},\s*{num}\]'] * 4) + r'\]"', 8)
    checks = [
        ('vo_node CAM_X_DEFAULT', grab('phase1/nodes/vo_node.py', r'^CAM_X_DEFAULT = ([-\d.]+)'), g['cam'][0]),
        ('vo_node CAM_Y_DEFAULT', grab('phase1/nodes/vo_node.py', r'^CAM_Y_DEFAULT = ([-\d.]+)'), g['cam'][1]),
        ('vo_node CAM_Z_DEFAULT', grab('phase1/nodes/vo_node.py', r'^CAM_Z_DEFAULT = ([-\d.]+)'), g['cam'][2]),
        ('firmware WHEEL_BASE_M', grab('phase1/firmware/rover_firmware_v2.ino', r'#define WHEEL_BASE_M\s+([\d.]+)f'),
         v['wheels.track']),
        ('firmware WHEEL_DIAMETER_M', grab('phase1/firmware/rover_firmware_v2.ino',
                                           r'#define WHEEL_DIAMETER_M\s+([\d.]+)f'), v['odometry.wheel_diameter']),
        ('nav2 footprint', fp, (F, S, F, -S, -R, -S, -R, S)),
    ]
    corners = r'^CORNERS = np\.array\(\[' + r',\s*'.join([rf'\[{num},\s*{num}\]'] * 4) + r'\]\)'
    for rel in ('phase3/nodes/pivot_goto.py', 'lidar/pivot_test.py'):
        checks.append((f'{Path(rel).name} CORNERS', grab(rel, corners, 8), (F, S, F, -S, -R, -S, -R, S)))
    for rel in ('phase1/nodes/fusion.py', 'phase1/nodes/compare.py'):
        name = Path(rel).name
        checks.append((f'{name} WHEEL_BASE_M', grab(rel, r'^WHEEL_BASE_M = ([\d.]+)'), v['wheels.track']))
        checks.append((f'{name} WHEEL_BASE_ROT_M', grab(rel, r'^WHEEL_BASE_ROT_M = ([\d.]+)'),
                       v['odometry.effective_track']))
        checks.append((f'{name} METRES_PER_COUNT diameter',
                       grab(rel, r'^METRES_PER_COUNT = math\.pi \* ([\d.]+) /'), v['odometry.wheel_diameter']))

    bad = 0
    for name, got, want in checks:
        gt = got if isinstance(got, tuple) else (got,)
        wt = want if isinstance(want, tuple) else (want,)
        if got is None:
            print(f'  ?  {name}: not found (pattern out of date?)')
            bad += 1
        elif any(abs(a - b) > TOL for a, b in zip(gt, wt)):
            print(f'  ✗  {name}: {got}  should be {tuple(round(w, 4) for w in wt) if len(wt) > 1 else round(wt[0], 4)}')
            bad += 1
    print(f'  drift: {len(checks) - bad}/{len(checks)} copies agree with params.yaml')
    return bad


def summary(g):
    v = g['v']
    print(f'  base_link: nose {g["nose_x"]:+.4f}  tail {g["tail_x"]:+.4f}  wheelbase {g["wheelbase"]:.3f}')
    print(f'  wheels:    x ±{g["wheel_x"]:.4f}  y ±{g["wheel_y"]:.3f}  z {g["wheel_z"]:.3f}')
    print('  camera0_link: x {:+.4f} y {:+.4f} z {:.4f}'.format(*g['cam'])
          + '   (tape+datasheet: x {:+.4f} y {:+.4f})'.format(*g['cam_geom']))
    print('  laser:        x {:+.4f} y {:+.4f} z {:.4f}  yaw {:.4f}'.format(*g['lidar'], g['lidar_yaw']))
    print(f'  envelope:  front {g["env_front"]:.3f}  rear {g["env_rear"]:.3f}  side {g["env_side"]:.3f}'
          f'  = {(g["env_front"] + g["env_rear"]) * 100:.1f} x {g["env_side"] * 200:.1f} cm')


def rsp(p, path, lidar_yaw=None):
    """robot_state_publisher's params file, with the URDF inline."""
    g = derive(p)
    if lidar_yaw is not None:  # ./rover's LIDAR_YAW=... override, for trying a value
        g['lidar_yaw'] = float(lidar_yaw)
    body = ''.join('      ' + line + '\n' if line else '\n' for line in urdf(p, g).splitlines())
    Path(path).write_text('# GENERATED by description/build.py --rsp from params.yaml\n'
                          'robot_state_publisher:\n  ros__parameters:\n'
                          '    robot_description: |\n' + body)
    print(f'  base_link -> camera0_link {g["cam"]}, laser {g["lidar"]} yaw {g["lidar_yaw"]:.4f}')


def main():
    p = load()
    if '--rsp' in sys.argv:
        a = sys.argv
        yaw = a[a.index('--lidar-yaw') + 1] if '--lidar-yaw' in a else None
        rsp(p, a[a.index('--rsp') + 1], yaw)
        return
    g = derive(p)
    text = urdf(p, g)
    ok = True
    if '--check' in sys.argv:
        if (HERE / 'rover.urdf').read_text() != text:
            print('  ✗  rover.urdf is stale -- run description/build')
            ok = False
    else:
        (HERE / 'rover.urdf').write_text(text)
        print('  wrote description/rover.urdf')
        built = cad(p, g)
        if built:
            print(f'  cad: {len(built)} parts -> description/cad/rover.step + one .stl each')
    summary(g)
    todo = [k for k, e in entries(p) if e['status'] == 'unknown']
    soft = [k for k, e in entries(p) if e['status'] == 'described']
    assumed = [f'{k} = {e["value"]}' for k, e in entries(p) if e['status'] == 'assumed']
    if assumed:
        print(f'  ASSUMED, not measured ({len(assumed)}): ' + ', '.join(assumed))
    if todo:
        print(f'  to measure ({len(todo)}): ' + ', '.join(todo))
    if soft:
        print(f'  described, not measured ({len(soft)}): ' + ', '.join(soft))
    sys.exit(0 if drift(g) == 0 and ok else 1)


if __name__ == '__main__':
    main()
