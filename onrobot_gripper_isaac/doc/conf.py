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
    """Translate the repository link to the sibling rosdoc2 package page."""
    from docutils import nodes
    from sphinx import addnodes

    target = '../../onrobot_gripper_rviz_plugins/README.md#2fg-closing-force-grip'
    for node in doctree.findall(addnodes.pending_xref):
        if node.get('reftarget') == target:
            node.replace_self(nodes.reference(
                '', '', *node.children,
                refuri='../onrobot_gripper_rviz_plugins/standard_docs/README.html#fg-closing-force-grip'))


def setup(app):
    app.connect('doctree-read', _resolve_panel_reference)
