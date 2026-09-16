"""Check repository-level public ROS package metadata and documentation."""

from pathlib import Path
import re
from urllib.parse import unquote
import xml.etree.ElementTree as ET
import importlib.util

from launch.actions import DeclareLaunchArgument


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
EXPECTED_MAINTAINER = 'Mikael Westermann'
EXPECTED_EMAIL = 'mikael.westermann@onrobot.com'
PUBLIC_TEXT_SUFFIXES = {'.md', '.rst'}
RELEASE_TEXT_SUFFIXES = {
    '.c', '.cmake', '.cpp', '.h', '.hpp', '.launch', '.md', '.msg', '.py',
    '.rst', '.rviz', '.txt', '.urdf', '.usda', '.xacro', '.xml', '.yaml',
    '.yml',
}
GRIPPER_MODEL_PACKAGES = {
    'onrobot_2fg7',
    'onrobot_2fg14',
    'onrobot_rg2',
    'onrobot_rg6',
}
PROHIBITED_PUBLIC_PATTERNS = {
    'internal planning reference': re.compile(r'master.?plan', re.IGNORECASE),
    'internal phase shorthand': re.compile(
        r'\b[PSTG]\d+\b', re.IGNORECASE),
    'internal requirement identifier': re.compile(
        r'\b(?:A|API|CODE|DESC|DOC|HW|REL|ROS|RT|SIM|SYS|TEST)-\d+\b'
    ),
    'internal fixture address': re.compile(
        r'\b10\.(?:36|45)\.\d{1,3}\.\d{1,3}\b'
    ),
    'agent work history': re.compile(r'\bcodex\b', re.IGNORECASE),
}


def package_manifests():
    """Return all ROS package manifests in stable order."""
    return sorted(REPOSITORY_ROOT.glob('*/package.xml'))


def public_documents():
    """Yield release-facing Markdown and reStructuredText documents."""
    for path in sorted(REPOSITORY_ROOT.rglob('*')):
        relative = path.relative_to(REPOSITORY_ROOT)
        if any(part.startswith('.') or part in {'docs-internal', 'doc-internal'}
               for part in relative.parts):
            continue
        if path.is_file() and path.suffix.lower() in PUBLIC_TEXT_SUFFIXES:
            yield path


def release_facing_texts():
    """Yield documentation, source, configuration, and code samples."""
    this_test = Path(__file__).resolve()
    for path in sorted(REPOSITORY_ROOT.rglob('*')):
        relative = path.relative_to(REPOSITORY_ROOT)
        if any(part.startswith('.') or part in {'docs-internal', 'doc-internal'}
               for part in relative.parts):
            continue
        if path.resolve() == this_test:
            continue
        if (path.is_file() and
                (path.suffix.lower() in RELEASE_TEXT_SUFFIXES or
                 path.name == 'CMakeLists.txt')):
            yield path


def test_all_packages_have_release_maintainer():
    """Require one release maintainer with the agreed contact address."""
    manifests = package_manifests()
    assert manifests, 'no ROS package manifests found'

    for manifest in manifests:
        root = ET.parse(manifest).getroot()
        maintainers = root.findall('maintainer')
        assert len(maintainers) == 1, (
            f'{manifest}: expected exactly one maintainer'
        )
        maintainer = maintainers[0]
        assert maintainer.text == EXPECTED_MAINTAINER, manifest
        assert maintainer.attrib.get('email') == EXPECTED_EMAIL, manifest


def test_all_packages_use_the_repository_bsd_license():
    """Keep package metadata aligned with the repository's public license."""
    for manifest in package_manifests():
        root = ET.parse(manifest).getroot()
        licenses = root.findall('license')
        assert len(licenses) == 1, (
            f'{manifest}: expected exactly one license declaration'
        )
        assert licenses[0].text == 'BSD-3-Clause', manifest


def test_getting_started_matches_binary_tool_api_distribution():
    """Require handoff docs to install both Tool API packages explicitly."""
    for relative in ('README.md', 'GETTING_STARTED.md'):
        content = (REPOSITORY_ROOT / relative).read_text(encoding='utf-8')
        assert (
            'libonrobot-tool-api0_*.deb' in content or
            'libonrobot-tool-api0_*_amd64.deb' in content or
            'libonrobot-tool-api0_${TOOL_API_VERSION}_amd64.deb' in content
        ), relative
        assert (
            'libonrobot-tool-api-dev_*.deb' in content or
            'libonrobot-tool-api-dev_*_amd64.deb' in content or
            'libonrobot-tool-api-dev_${TOOL_API_VERSION}_amd64.deb' in content
        ), relative
        assert '--skip-keys onrobot_tool_api' in content, relative
        assert 'Tool API source is not required' in content or (
            'Tool API runtime and development packages' in content
        ), relative


def test_bsd_3_clause_license_is_present_and_linked():
    """Keep the public repository license and README link consistent."""
    license_path = REPOSITORY_ROOT / 'LICENSE'
    content = license_path.read_text(encoding='utf-8')
    normalized = ' '.join(content.split())
    assert content.startswith('BSD 3-Clause License')
    assert 'Copyright (c) 2026, OnRobot A/S' in content
    assert 'Redistribution and use in source and binary forms' in normalized
    assert 'AS IS' in normalized
    assert 'Prototype Evaluation License' not in content

    readme = (REPOSITORY_ROOT / 'README.md').read_text(encoding='utf-8')
    assert '[BSD 3-Clause License](LICENSE)' in readme


def test_public_docs_have_no_beta_or_prototype_release_framing():
    """Keep customer documentation aligned with the release offering."""
    obsolete_terms = re.compile(
        r'\b(?:beta|prototype)\b|prototype evaluation license|'
        r'not (?:for|intended for) (?:production|safety-critical use)',
        re.IGNORECASE)
    findings = []
    for document in public_documents():
        content = document.read_text(encoding='utf-8')
        if obsolete_terms.search(content):
            findings.append(str(document.relative_to(REPOSITORY_ROOT)))
    assert not findings, '\n'.join(findings)


def test_three_finger_ros_path_is_described_as_evaluation_only():
    """Keep the 3FG scope distinct from the four supported release models."""
    documents = (
        'README.md',
        'GETTING_STARTED.md',
        'docs/supported-devices.md',
        'onrobot_gripper_demos/README.md',
        'docs/hardware-commissioning.md',
        'docs/reference/launch-parameters.md',
        'onrobot_3fg15/README.md',
        'onrobot_3fg25/README.md',
        'onrobot_gripper_rviz_plugins/README.md',
        'docs/reference/ros-interfaces.md',
    )
    for relative in documents:
        content = (REPOSITORY_ROOT / relative).read_text(encoding='utf-8')
        assert ('evaluation only' in content.lower() or
                'evaluation-only' in content.lower()), relative


def test_2fg7_three_view_demo_is_linked_and_installed():
    """Keep the customer demonstration discoverable and distributable."""
    guide = (
        REPOSITORY_ROOT / 'onrobot_gripper_isaac' /
        'doc' / 'RVIZ_ISAAC_2FG7_DEMO.md'
    )
    content = guide.read_text(encoding='utf-8')
    assert 'model:=2fg7' in content
    assert 'control:=conventional' in content
    assert 'control:=realtime' in content
    assert '--ros-preflight-only' in content
    assert '--ready-output' in content
    assert 'read-only' in content

    root_readme = (REPOSITORY_ROOT / 'README.md').read_text(
        encoding='utf-8')
    assert 'RVIZ_ISAAC_2FG7_DEMO.md' in root_readme

    cmake = (
        REPOSITORY_ROOT / 'onrobot_gripper_isaac' / 'CMakeLists.txt'
    ).read_text(encoding='utf-8')
    assert 'DIRECTORY config doc resource' in cmake


def test_rg2_hil_guide_is_linked_and_installed():
    """Keep the RG2 hardware workflow discoverable and distributable."""
    guide = (
        REPOSITORY_ROOT / 'onrobot_gripper_isaac' / 'doc' /
        'RG2_HIL_GUIDE.md'
    )
    content = guide.read_text(encoding='utf-8')
    normalized = ' '.join(content.split())
    assert 'model:=rg2' in content
    assert 'control:=conventional' in content
    assert 'control:=realtime' in content
    assert 'shadow_mimic_dof_count' in content
    assert 'Use 50 Hz for this check' in normalized

    root_readme = (REPOSITORY_ROOT / 'README.md').read_text(
        encoding='utf-8')
    assert 'RG2_HIL_GUIDE.md' in root_readme

    cmake = (
        REPOSITORY_ROOT / 'onrobot_gripper_isaac' / 'CMakeLists.txt'
    ).read_text(encoding='utf-8')
    assert 'DIRECTORY config doc resource' in cmake


def test_payload_retention_fixture_is_documented_and_installed():
    """Keep both cross-model payload-retention scenarios distributable."""
    package = REPOSITORY_ROOT / 'onrobot_gripper_isaac'
    readme = (package / 'README.md').read_text(encoding='utf-8')
    verification = (package / 'doc' / 'ISAAC_SIM_VERIFICATION.md').read_text(
        encoding='utf-8')
    physical_motion = (
        package / 'doc' / 'PHYSICAL_MOTION_SHOWCASE.md'
    ).read_text(encoding='utf-8')
    cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')
    config = (package / 'config' /
              'payload_retention_profiles.json').read_text(encoding='utf-8')

    assert '[Verify OnRobot grippers in' in readme
    assert 'doc/ISAAC_SIM_VERIFICATION.md' in readme
    assert 'run_payload_retention_test.py' in verification
    assert 'run_payload_retention_matrix.py' in verification
    assert 'full-rated' in verification
    for executable in (
            'run_physical_motion_showcase.py',
            'run_physical_motion_showcase_matrix.py'):
        assert executable in physical_motion
    assert 'scripts/run_payload_retention_test.py' in cmake
    assert 'scripts/run_payload_retention_matrix.py' in cmake
    assert 'scripts/run_physical_motion_showcase.py' in cmake
    assert 'scripts/run_physical_motion_showcase_matrix.py' in cmake
    assert 'scripts/run_drive_step_response.py' in cmake
    assert 'scripts/validate_physical_motion_results.py' in cmake
    assert 'DIRECTORY config doc resource' in cmake
    for model in GRIPPER_MODEL_PACKAGES:
        assert f'"{model.removeprefix("onrobot_")}"' in config


def test_isaac_package_has_rosdoc2_sphinx_entrypoint():
    """Keep package documentation in the ROS-standard source layout."""
    package = REPOSITORY_ROOT / 'onrobot_gripper_isaac'
    conf = (package / 'doc' / 'conf.py').read_text(encoding='utf-8')
    index = (package / 'doc' / 'index.rst').read_text(encoding='utf-8')
    cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert "extensions = ['myst_parser']" in conf
    assert "'user_docs/**'" in conf
    for source in (
            'FIRST_ISAAC_SIMULATION.md',
            'ISAAC_SIM_VERIFICATION.md',
            'PHYSICAL_MOTION_SHOWCASE.md',
            'RG2_HIL_GUIDE.md',
            'RVIZ_ISAAC_2FG7_DEMO.md'):
        assert (package / 'doc' / source).is_file()
        assert source.removesuffix('.md') in index
    assert 'DIRECTORY config doc resource' in cmake


def test_user_guide_packages_have_rosdoc2_sphinx_entrypoints():
    """Keep substantive package guides ready for standard package docs."""
    for name in (
            'onrobot_gripper_bringup',
            'onrobot_gripper_demos',
            'onrobot_gripper_description',
            'onrobot_gripper_moveit_config'):
        package = REPOSITORY_ROOT / name
        conf = (package / 'doc' / 'conf.py').read_text(encoding='utf-8')
        index = (package / 'doc' / 'index.rst').read_text(encoding='utf-8')
        include = (package / 'doc' / 'readme_include.md').read_text(
            encoding='utf-8')
        cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')

        assert "extensions = ['myst_parser']" in conf
        assert "'user_docs/**'" in conf
        assert '.. include:: readme_include.md' in index
        assert ':parser: myst_parser.sphinx_' in index
        assert '```{include} standard_docs/original/README.md' in include
        assert ':relative-images:' in include
        assert re.search(r'install\s*\([^)]*\bdoc\b', cmake, re.DOTALL)


def test_every_package_installs_a_readme():
    """Require each package to provide and install a landing page."""
    missing = []
    not_installed = []
    for manifest in package_manifests():
        package_dir = manifest.parent
        readme = package_dir / 'README.md'
        if not readme.is_file():
            missing.append(str(package_dir.relative_to(REPOSITORY_ROOT)))
            continue

        cmake = package_dir / 'CMakeLists.txt'
        cmake_text = cmake.read_text(encoding='utf-8')
        if not re.search(
            r'install\s*\([^)]*README\.md', cmake_text, re.DOTALL
        ):
            not_installed.append(
                str(package_dir.relative_to(REPOSITORY_ROOT))
            )

    assert not missing, 'packages without README.md: ' + ', '.join(missing)
    assert not not_installed, (
        'packages that do not install README.md: ' + ', '.join(not_installed)
    )


def test_release_text_does_not_expose_internal_project_language():
    """Reject internal terms in docs, samples, configuration, and comments."""
    findings = []
    for document in release_facing_texts():
        relative = str(document.relative_to(REPOSITORY_ROOT))
        for label, pattern in PROHIBITED_PUBLIC_PATTERNS.items():
            if pattern.search(relative):
                findings.append(f'{relative}: filename contains {label}')
        content = document.read_text(encoding='utf-8')
        for label, pattern in PROHIBITED_PUBLIC_PATTERNS.items():
            for match in pattern.finditer(content):
                line = content.count('\n', 0, match.start()) + 1
                findings.append(
                    f'{relative}:{line}: {label}'
                )

    assert not findings, '\n'.join(findings)


def test_relative_markdown_links_resolve():
    """Require every local Markdown link to resolve in the source tree."""
    broken = []
    markdown_link = re.compile(r'(?<!!)\[[^]]+\]\(([^)]+)\)')
    for document in public_documents():
        if document.suffix.lower() != '.md':
            continue
        content = document.read_text(encoding='utf-8')
        for raw_target in markdown_link.findall(content):
            target = raw_target.split('#', 1)[0].strip()
            if not target or '://' in target or target.startswith('mailto:'):
                continue
            resolved = (document.parent / unquote(target)).resolve()
            if not resolved.exists():
                broken.append(
                    f'{document.relative_to(REPOSITORY_ROOT)} -> {raw_target}'
                )

    assert not broken, '\n'.join(broken)


def test_task_guide_launch_tables_match_executed_launch_declarations():
    """Contributed reference defaults must match the current launch API."""
    text = (REPOSITORY_ROOT / 'docs/reference/launch-parameters.md').read_text()
    for heading, relative in (
            ('Interactive showcase', 'onrobot_gripper_demos/launch/showcase.launch.py'),
            ('Headless parallel-gripper bringup',
             'onrobot_gripper_bringup/launch/gripper.launch.py')):
        spec = importlib.util.spec_from_file_location('documented_launch', REPOSITORY_ROOT / relative)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        defaults = {
            item.name: ''.join(value.text for value in item.default_value)
            for item in module.generate_launch_description().entities
            if isinstance(item, DeclareLaunchArgument)
        }
        section = text.split(f'## {heading}\n', 1)[1].split('\n## ', 1)[0]
        rows = re.findall(r'^\| `([^`]+)` \| (`[^`]*`|empty) \|', section, re.M)
        assert rows, heading
        for name, default in rows:
            assert name in defaults, (heading, name)
            assert defaults[name] == ('' if default == 'empty' else default.strip('`'))


def test_message_reference_lists_current_gripper_state_fields():
    """Prevent stale field tables after changes to the typed feedback API."""
    definition = (REPOSITORY_ROOT / 'onrobot_gripper_msgs/msg/GripperState.msg').read_text()
    reference = (REPOSITORY_ROOT / 'docs/reference/messages.md').read_text()
    for line in definition.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or '=' in line:
            continue
        _, name = line.split()
        assert f'`{name}`' in reference, name


def test_documented_launch_entry_points_exist():
    """Require every documented ros2 launch package/file pair to resolve."""
    missing = []
    launch_command = re.compile(
        r'ros2\s+launch\s+([a-z0-9_]+)\s+([A-Za-z0-9_.-]+)')
    for document in public_documents():
        content = document.read_text(encoding='utf-8')
        for package_name, launch_file in launch_command.findall(content):
            expected = (
                REPOSITORY_ROOT / package_name / 'launch' / launch_file)
            if not expected.is_file():
                relative = document.relative_to(REPOSITORY_ROOT)
                package_manifest = (
                    REPOSITORY_ROOT / relative.parts[0] / 'package.xml'
                    if len(relative.parts) > 1 else None
                )
                if package_manifest and package_manifest.is_file():
                    dependencies = {
                        item.text for item in
                        ET.parse(package_manifest).getroot().findall(
                            'exec_depend')
                    }
                    if package_name in dependencies:
                        continue
                missing.append(
                    f'{document.relative_to(REPOSITORY_ROOT)}: '
                    f'{package_name} {launch_file}')

    assert not missing, '\n'.join(missing)


def test_documented_run_entry_points_are_installed():
    """Require documented ros2 run executables in package install rules."""
    missing = []
    run_command = re.compile(
        r'ros2\s+run\s+([a-z0-9_]+)\s+([A-Za-z0-9_.-]+)')
    for document in public_documents():
        content = document.read_text(encoding='utf-8')
        for package_name, executable in run_command.findall(content):
            cmake = REPOSITORY_ROOT / package_name / 'CMakeLists.txt'
            if (not cmake.is_file() or
                    executable not in cmake.read_text(encoding='utf-8')):
                missing.append(
                    f'{document.relative_to(REPOSITORY_ROOT)}: '
                    f'{package_name} {executable}')

    assert not missing, '\n'.join(missing)


def test_directory_installs_exclude_python_cache_artifacts():
    """Generated interpreter caches must never enter installed packages."""
    missing_exclusions = []
    install_directory = re.compile(
        r'install\s*\(\s*DIRECTORY\b.*?\)', re.DOTALL)
    for cmake in sorted(REPOSITORY_ROOT.glob('*/CMakeLists.txt')):
        content = cmake.read_text(encoding='utf-8')
        for block in install_directory.findall(content):
            if ('PATTERN "__pycache__" EXCLUDE' not in block or
                    'PATTERN "*.pyc" EXCLUDE' not in block):
                missing_exclusions.append(
                    str(cmake.relative_to(REPOSITORY_ROOT)))

    assert not missing_exclusions, (
        'directory installs without Python-cache exclusions: ' +
        ', '.join(missing_exclusions)
    )


def test_customer_bundle_excludes_repository_and_ci_metadata():
    """Keep internal collaboration state out of the customer archive."""
    script = (
        REPOSITORY_ROOT / 'scripts' / 'create_customer_bundle.sh'
    ).read_text(encoding='utf-8')

    for excluded in (
            "--exclude='./.git'",
            "--exclude='./.gitignore'",
            "--exclude='./bitbucket-pipelines.yml'",
            "--exclude='*/__pycache__'",
            "--exclude='build_*'",
            "--exclude='install_*'",
            "--exclude='log_*'"):
        assert excluded in script


def test_customer_bundle_records_paired_package_and_source_identity():
    """Require a structured identity record in every customer archive."""
    script = (
        REPOSITORY_ROOT / 'scripts' / 'create_customer_bundle.sh'
    ).read_text(encoding='utf-8')

    assert 'BUNDLE_MANIFEST.json' in script
    assert "'ros_distribution': 'Jazzy'" in script
    assert "'operating_system': 'Ubuntu 24.04'" in script
    assert "'file_integrity_manifest': 'SHA256SUMS'" in script
    assert 'ONROBOT_BUNDLE_RUNTIME_SHA256' in script
    assert 'ONROBOT_BUNDLE_DEVELOPMENT_SHA256' in script
    assert 'git -C "$repository_root" rev-parse HEAD' in script
    assert 'git -C "$repository_root" status --porcelain' in script


def test_model_launch_files_only_reference_their_own_description_package():
    """Prevent a model display launch from loading another gripper model."""
    findings = []
    for package_name in sorted(GRIPPER_MODEL_PACKAGES):
        launch_dir = REPOSITORY_ROOT / package_name / 'launch'
        other_packages = GRIPPER_MODEL_PACKAGES - {package_name}
        for launch_file in sorted(launch_dir.glob('*.py')):
            content = launch_file.read_text(encoding='utf-8')
            for other_package in sorted(other_packages):
                if other_package in content:
                    findings.append(
                        f'{launch_file.relative_to(REPOSITORY_ROOT)}: '
                        f'references {other_package}'
                    )

    assert not findings, '\n'.join(findings)


def test_model_srdf_identity_matches_its_package():
    """Keep installed semantic descriptions bound to the intended model."""
    findings = []
    for package_name in sorted(GRIPPER_MODEL_PACKAGES):
        srdf_dir = REPOSITORY_ROOT / package_name / 'srdf'
        for srdf in sorted(srdf_dir.glob('*.srdf')):
            root = ET.parse(srdf).getroot()
            if root.tag != 'robot' or root.attrib.get('name') != package_name:
                findings.append(
                    f'{srdf.relative_to(REPOSITORY_ROOT)}: expected robot '
                    f'name {package_name!r}, got {root.attrib.get("name")!r}'
                )

    assert not findings, '\n'.join(findings)


def test_literal_launch_description_files_exist():
    """Require literal URDF/Xacro launch inputs to exist in their package."""
    findings = []
    description_name = re.compile(
        r"['\"]([A-Za-z0-9_.-]+\.(?:urdf|xacro))['\"]"
    )
    for package_name in sorted(GRIPPER_MODEL_PACKAGES):
        package_dir = REPOSITORY_ROOT / package_name
        for launch_file in sorted((package_dir / 'launch').glob('*.py')):
            content = launch_file.read_text(encoding='utf-8')
            for filename in description_name.findall(content):
                if not (package_dir / 'urdf' / filename).is_file():
                    findings.append(
                        f'{launch_file.relative_to(REPOSITORY_ROOT)}: '
                        f'missing urdf/{filename}'
                    )

    assert not findings, '\n'.join(findings)
