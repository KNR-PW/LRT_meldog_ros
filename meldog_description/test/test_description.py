"""Checks of the Meldog robot description: xacro -> URDF (-> SDF for Gazebo).

Run:  python3 -m pytest test/test_description.py -v      (from the package directory)
 or:  colcon test --packages-select meldog_description

The source tree is tested (not the install space). Gazebo checks are skipped when
neither `ign` nor `gz` with the `sdf` command is available.
"""
import itertools
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
ENTRY = PKG / 'description' / 'meldog.urdf.xacro'
LEGACY_ENTRY = PKG / 'description' / 'meldog_core.urdf.xacro'
CONFIG = PKG / 'config'

SIMS = ('none', 'gazebo', 'mujoco')
COLLISIONS = ('simple', 'mesh')
VARIANTS = list(itertools.product(SIMS, COLLISIONS))
LEGS = ('LF', 'RF', 'LR', 'RR')
ACTUATED = {f'{leg}{j}_joint' for leg in LEGS for j in ('T', 'H', 'K')}
LINKS = {'trunk_link', 'imu_link'} | {f'{leg}{p}_link' for leg in LEGS for p in ('H', 'UL', 'LL', 'F')}

pytestmark = pytest.mark.skipif(shutil.which('xacro') is None, reason='xacro not found (source ROS 2)')


def load_yaml(*parts):
    with open(CONFIG.joinpath(*parts)) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='session')
def env(tmp_path_factory):
    """Environment where $(find meldog_description) resolves to this source tree."""
    prefix = tmp_path_factory.mktemp('ament_prefix')
    index = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    (index / 'meldog_description').touch()
    (prefix / 'share' / 'meldog_description').symlink_to(PKG)
    e = dict(os.environ)
    e['AMENT_PREFIX_PATH'] = os.pathsep.join(filter(None, [str(prefix), e.get('AMENT_PREFIX_PATH')]))
    return e


def run_xacro(env, entry=ENTRY, **args):
    cmd = ['xacro', str(entry)] + [f'{k}:={v}' for k, v in args.items()]
    return subprocess.run(cmd, env=env, check=True, capture_output=True, text=True).stdout


@pytest.fixture(scope='session')
def urdf_text(env):
    return {(sim, col): run_xacro(env, sim=sim, collision=col) for sim, col in VARIANTS}


@pytest.fixture(scope='session')
def urdf(urdf_text):
    return {key: ET.fromstring(text) for key, text in urdf_text.items()}


def canon(e):
    """Comparable form of an element tree (ignores comments and whitespace)."""
    return (e.tag, sorted(e.attrib.items()), (e.text or '').strip(), [canon(c) for c in e])


def link(root, name):
    return root.find(f"link[@name='{name}']")


# --------------------------------------------------------------------------- URDF

@pytest.mark.parametrize('sim,col', VARIANTS)
def test_check_urdf(sim, col, urdf_text, tmp_path):
    if shutil.which('check_urdf') is None:
        pytest.skip('check_urdf not found')
    f = tmp_path / f'meldog_{sim}_{col}.urdf'
    f.write_text(urdf_text[(sim, col)])
    subprocess.run(['check_urdf', str(f)], check=True, capture_output=True)


@pytest.mark.parametrize('entry', [ENTRY, LEGACY_ENTRY])
def test_urdf_is_yaml_parsable(env, entry, urdf_text):
    """ROS 2 XML launch files parse a <param> value without type="str" as YAML.

    A comment with a colon followed by a space in the generated URDF breaks it
    ("mapping values are not allowed here"). Keep comments out of macro bodies.
    """
    texts = list(urdf_text.values()) if entry == ENTRY else [run_xacro(env, entry=entry)]
    for text in texts:
        yaml.safe_load(text)


@pytest.mark.parametrize('args', [{'sim': 'foo'}, {'collision': 'foo'}])
def test_unknown_argument_value_is_rejected(env, args):
    with pytest.raises(subprocess.CalledProcessError):
        run_xacro(env, **args)


def test_defaults(env, urdf):
    """No arguments: sim:=none, collision:=simple."""
    assert canon(ET.fromstring(run_xacro(env))) == canon(urdf[('none', 'simple')])


def test_legacy_entry_point_equals_gazebo(env, urdf):
    assert canon(ET.fromstring(run_xacro(env, entry=LEGACY_ENTRY))) == canon(urdf[('gazebo', 'simple')])


@pytest.mark.parametrize('col', COLLISIONS)
def test_core_is_simulator_independent(col, urdf):
    tags = {child.tag for child in urdf[('none', col)]}
    assert tags <= {'link', 'joint', 'material'}, tags


@pytest.mark.parametrize('sim,col', VARIANTS)
def test_kinematic_tree(sim, col, urdf):
    root = urdf[(sim, col)]
    assert {lk.get('name') for lk in root.findall('link')} == LINKS
    joints = {j.get('name'): j.get('type') for j in root.findall('joint')}
    assert {n for n, t in joints.items() if t == 'revolute'} == ACTUATED
    assert {n for n, t in joints.items() if t == 'fixed'} == {'imu_joint'} | {f'{leg}F_joint' for leg in LEGS}


def test_joint_limits_match_config(urdf):
    groups = load_yaml('robot', 'actuators.yaml')['joints']
    for j in urdf[('none', 'simple')].findall("joint[@type='revolute']"):
        g = groups[{'T': 'trunk', 'H': 'hip', 'K': 'knee'}[j.get('name')[2]]]
        lim = j.find('limit')
        assert (float(lim.get('lower')), float(lim.get('upper'))) == (g['lower'], g['upper']), j.get('name')


def test_collision_models(urdf):
    geo = load_yaml('robot', 'geometry.yaml')['links']
    parts = {'trunk_link': 'trunk', **{f'{leg}{p}_link': name for leg in LEGS
                                       for p, name in (('H', 'hip'), ('UL', 'upper_limb'),
                                                       ('LL', 'lower_limb'), ('F', 'foot'))}}
    for col in COLLISIONS:
        root = urdf[('none', col)]
        assert link(root, 'imu_link').find('collision') is None
        for name, part in parts.items():
            shapes = geo[part]['collision'][col]
            found = [c.find('geometry')[0].tag for c in link(root, name).findall('collision')]
            assert found == [s['type'] for s in shapes], (col, name)
            assert all(c.get('name') is None for c in link(root, name).findall('collision')), \
                'named <collision> breaks Gazebo surface params of lumped links'


def test_primitive_collisions_are_mirrored(urdf):
    root = urdf[('none', 'simple')]
    x, y, z = load_yaml('robot', 'geometry.yaml')['links']['hip']['collision']['simple'][0]['xyz']
    for leg, (sx, sy) in zip(LEGS, ((1, 1), (1, -1), (-1, 1), (-1, -1))):
        xyz = [float(v) for v in link(root, f'{leg}H_link').find('collision/origin').get('xyz').split()]
        assert xyz == pytest.approx([sx * x, sy * y, z]), leg


def test_total_mass_matches_config(urdf):
    m = load_yaml('robot', 'inertial.yaml')
    expected = m['trunk']['mass'] + m['imu']['mass'] + 4 * sum(
        m[p]['mass'] for p in ('hip', 'upper_limb', 'lower_limb', 'foot'))
    total = sum(float(e.get('value')) for e in urdf[('none', 'simple')].iter('mass'))
    assert total == pytest.approx(expected)


def test_inertia_is_physically_valid(urdf):
    for lk in urdf[('none', 'simple')].findall('link'):
        if float(lk.find('inertial/mass').get('value')) == 0.0:
            continue
        i = {k: float(v) for k, v in lk.find('inertial/inertia').attrib.items()}
        xx, yy, zz, xy, xz, yz = i['ixx'], i['iyy'], i['izz'], i['ixy'], i['ixz'], i['iyz']
        det = xx * (yy * zz - yz ** 2) - xy * (xy * zz - yz * xz) + xz * (xy * yz - yy * xz)
        name = lk.get('name')
        assert xx > 0 and xx * yy - xy ** 2 > 0 and det > 0, f'{name}: inertia not positive definite'
        assert xx + yy >= zz and xx + zz >= yy and yy + zz >= xx, f'{name}: triangle inequality'


def test_ros2_control_matches_urdf_and_controllers(urdf):
    root = urdf[('gazebo', 'simple')]
    rc = root.find('ros2_control')
    rc_joints = {j.get('name'): j for j in rc.findall('joint')}
    assert set(rc_joints) == ACTUATED

    limits = {j.get('name'): j.find('limit') for j in root.findall("joint[@type='revolute']")}
    for name, j in rc_joints.items():
        params = {p.get('name'): float(p.text) for p in j.find("command_interface[@name='effort']")}
        effort = float(limits[name].get('effort'))
        assert params == {'min': -effort, 'max': effort}, name

    sensors = {s.get('name') for s in rc.findall('sensor')}
    with open(PKG / 'controllers' / 'meldog_joint_controllers.yaml') as f:
        ctrl = yaml.safe_load(f)
    jc = ctrl['joint_controller']['ros__parameters']
    assert set(jc['joint_names']) == ACTUATED
    for name, p in jc['joint_params'].items():
        assert (p['position_min'], p['position_max']) == pytest.approx(
            (float(limits[name].get('lower')), float(limits[name].get('upper')))), name
    assert ctrl['imu_sensor_broadcaster']['ros__parameters']['sensor_name'] in sensors
    assert set(ctrl['contact_sensors_broadcaster']['ros__parameters']['sensor_names']) <= sensors


@pytest.mark.parametrize('col', COLLISIONS)
def test_mujoco_meshes_are_absolute_and_exist(col, urdf):
    root = urdf[('mujoco', col)]
    assert root.find('mujoco/compiler') is not None
    for mesh in root.iter('mesh'):
        path = Path(mesh.get('filename'))
        assert path.is_absolute() and path.is_file(), path


# --------------------------------------------------------------------------- Gazebo (SDF)

def sdf_command():
    for exe in ('ign', 'gz'):
        if shutil.which(exe) and subprocess.run([exe, 'sdf', '--help'], capture_output=True).returncode == 0:
            return exe
    return None


@pytest.fixture(scope='session', params=COLLISIONS)
def sdf_model(request, urdf_text, tmp_path_factory):
    exe = sdf_command()
    if exe is None:
        pytest.skip('neither `ign sdf` nor `gz sdf` available')
    f = tmp_path_factory.mktemp('sdf') / f'meldog_{request.param}.urdf'
    f.write_text(urdf_text[('gazebo', request.param)])
    out = subprocess.run([exe, 'sdf', '-p', str(f)], check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out).find('model')


def test_gazebo_contact_sensors_reference_existing_collisions(sdf_model):
    cols = {c.get('name') for lk in sdf_model.findall('link') for c in lk.findall('collision')}
    contact = [s for s in sdf_model.iter('sensor') if s.get('type') == 'contact']
    assert {s.get('name') for s in contact} == {f'{leg}F_contact' for leg in LEGS}
    for s in contact:
        assert s.find('contact/collision').text in cols, s.get('name')


def test_gazebo_surface_friction(sdf_model):
    """Every collision gets the friction of its (possibly lumped) link."""
    surface = load_yaml('gazebo', 'gazebo.yaml')['surface']
    for lk in sdf_model.findall('link'):
        for c in lk.findall('collision'):
            name = c.get('name')
            mu = c.find('surface/friction/ode/mu')
            assert mu is not None, f'{name}: no friction in SDF (gazebo reference not applied)'
            foot = '_fixed_joint_lump__' in name and 'F_link_collision' in name
            assert float(mu.text) == surface['foot' if foot else 'default']['mu1'], name


def test_gazebo_sensors_match_ros2_control(urdf, sdf_model):
    sdf_sensors = {s.get('name') for s in sdf_model.iter('sensor')}
    rc_sensors = {s.get('name') for s in urdf[('gazebo', 'simple')].find('ros2_control').findall('sensor')}
    assert rc_sensors <= sdf_sensors
