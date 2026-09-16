"""Sphinx configuration for rosdoc2 package documentation."""

extensions = ['myst_parser']
project = 'OnRobot gripper demos'
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
