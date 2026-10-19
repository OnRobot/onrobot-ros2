"""Sphinx configuration for rosdoc2 package documentation."""

extensions = ['myst_parser']
myst_heading_anchors = 3
project = 'OnRobot gripper integration for Isaac Sim'
author = 'OnRobot'
html_logo = '_static/onrobot-logo.png'
html_static_path = ['_static']
html_css_files = ['onrobot.css']

# rosdoc2 wraps the custom Sphinx project and separately copies doc/ as
# generic user documentation. Exclude those generated duplicates and the
# standard README copy; this project provides its own package landing page.
exclude_patterns = [
    '__readme_include.rst',
    'standard_docs/**',
    'standards.rst',
    'user_docs/**',
    'user_docs.rst',
]


def _resolve_panel_reference(app, doctree):
    """Resolve links to pages outside this package's rosdoc2 project."""
    from docutils import nodes
    from sphinx import addnodes

    targets = {
        '../../onrobot_gripper_rviz_plugins/README.md#2fg-closing-force-grip':
            '../onrobot_gripper_rviz_plugins/standard_docs/README.html#fg-closing-force-grip',
    }
    # Repository guides are not installed as pages of this ROS package.
    public_docs = 'https://github.com/OnRobot/onrobot-ros2/blob/main/docs/'
    for page in (
        'supported-devices.md',
        'state-diagnostics-and-recovery.md',
        'reference/messages.md#realtimecommand',
    ):
        targets['../../docs/' + page] = public_docs + page
    for node in doctree.findall(addnodes.pending_xref):
        target = targets.get(node.get('reftarget'))
        if target:
            node.replace_self(nodes.reference(
                '', '', *node.children, refuri=target))


def setup(app):
    app.connect('doctree-read', _resolve_panel_reference)
