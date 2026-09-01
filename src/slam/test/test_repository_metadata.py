"""Repository-level checks for reproducible ROS package metadata.

These assertions intentionally live with the ``slam`` engineering tests because that
package owns the public three-command startup workflow.  They catch placeholder owner
data and missing runtime packages before another computer reaches a launch-time error.
"""

import json
from pathlib import Path
import xml.etree.ElementTree as ET


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
EXPECTED_MAINTAINER = 'DONGJIN'
EXPECTED_EMAIL = '2724596499@qq.com'
EXPECTED_LICENSE = 'Apache-2.0'


def package_manifests():
    """Return every first-party package manifest in a deterministic order."""
    return sorted((REPOSITORY_ROOT / 'src').glob('*/package.xml'))


def test_all_package_manifests_use_real_owner_and_repository_license():
    assert package_manifests()
    for manifest in package_manifests():
        root = ET.parse(manifest).getroot()
        maintainer = root.find('maintainer')
        license_node = root.find('license')
        assert maintainer is not None, manifest
        assert maintainer.text == EXPECTED_MAINTAINER, manifest
        assert maintainer.attrib.get('email') == EXPECTED_EMAIL, manifest
        assert license_node is not None and license_node.text == EXPECTED_LICENSE, manifest

    license_text = (REPOSITORY_ROOT / 'LICENSE').read_text(encoding='utf-8')
    assert 'Apache License' in license_text
    assert 'Version 2.0, January 2004' in license_text


def test_python_package_metadata_matches_its_manifest():
    for setup_file in sorted((REPOSITORY_ROOT / 'src').glob('*/setup.py')):
        source = setup_file.read_text(encoding='utf-8')
        assert f'maintainer="{EXPECTED_MAINTAINER}"' in source, setup_file
        assert f'maintainer_email="{EXPECTED_EMAIL}"' in source, setup_file
        assert f'license="{EXPECTED_LICENSE}"' in source, setup_file

        # ``ament_python`` is a colcon build type, not a ROS package/rosdep key.  Keep it
        # only in ``export/build_type``; declaring it as buildtool_depend makes rosdep
        # fail on a clean Ubuntu 24.04 installation.
        manifest = ET.parse(setup_file.with_name('package.xml')).getroot()
        assert manifest.findtext('export/build_type') == 'ament_python', setup_file
        assert 'ament_python' not in {
            node.text for node in manifest.findall('buildtool_depend')
        }, setup_file


def test_slam_manifest_declares_direct_launch_and_plugin_dependencies():
    root = ET.parse(REPOSITORY_ROOT / 'src' / 'slam' / 'package.xml').getroot()
    dependencies = {
        node.text
        for tag in ('depend', 'exec_depend')
        for node in root.findall(tag)
    }
    expected = {
        'dwb_core',
        'dwb_critics',
        'nav2_behavior_tree',
        'nav2_behaviors',
        'nav2_bt_navigator',
        'nav2_controller',
        'nav2_costmap_2d',
        'nav2_lifecycle_manager',
        'nav2_navfn_planner',
        'nav2_planner',
        'nav2_velocity_smoother',
        'quadruped_perception',
        'ros2topic',
    }
    assert expected <= dependencies


def test_vscode_recommends_ros_and_yaml_without_turtlebot_side_effects():
    vscode = REPOSITORY_ROOT / '.vscode'
    extensions = json.loads((vscode / 'extensions.json').read_text(encoding='utf-8'))
    recommendations = set(extensions['recommendations'])
    assert {'ms-iot.vscode-ros', 'redhat.vscode-yaml'} <= recommendations

    settings = json.loads((vscode / 'settings.json').read_text(encoding='utf-8'))
    terminal_environment = settings['terminal.integrated.env.linux']
    assert 'TURTLEBOT3_MODEL' not in terminal_environment


def test_coverage_artifacts_are_ignored_without_ignoring_vscode_configuration():
    ignored = (REPOSITORY_ROOT / '.gitignore').read_text(encoding='utf-8').splitlines()
    assert {'.coverage', '.coverage.*', 'coverage.xml', 'htmlcov/'} <= set(ignored)
    assert '.vscode/' not in ignored


def test_bootstrap_checks_colcon_build_support_and_hides_no_rosdep_errors():
    bootstrap = (REPOSITORY_ROOT / 'scripts' / 'bootstrap.sh').read_text(
        encoding='utf-8'
    )
    assert 'colcon_ros.task.ament_python.build' in bootstrap
    assert 'ros2 pkg prefix ament_python' not in bootstrap
    commands = '\n'.join(
        line for line in bootstrap.splitlines()
        if not line.lstrip().startswith('#')
    )
    assert '--skip-keys' not in commands
    assert ' -r ' not in commands
