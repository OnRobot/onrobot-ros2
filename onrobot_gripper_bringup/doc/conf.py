"""Sphinx configuration for rosdoc2 package documentation."""

extensions = ['myst_parser']
project = 'OnRobot gripper bringup'
author = 'OnRobot'
html_logo = '_static/onrobot-logo.png'
html_static_path = ['_static']
html_css_files = ['onrobot.css']

exclude_patterns = [
    '__readme_include.rst',
    'standard_docs/**',
    'standards.rst',
    'user_docs/**',
    'user_docs.rst',
]


def _resolve_readme_guide_link(app, doctree):
    """Resolve the repository-relative README link in rosdoc2's flat doc tree."""
    from sphinx import addnodes

    for node in doctree.findall(addnodes.pending_xref):
        if node.get('reftarget') in ('doc/USB_RTU.md', 'doc/REPORTING_ISSUES.md'):
            node['reftarget'] = node['reftarget'][4:-3]
            node['refdomain'] = 'std'
            node['reftype'] = 'doc'


def setup(app):
    """Keep one README usable both in the repository and the generated site."""
    app.connect('doctree-read', _resolve_readme_guide_link)
