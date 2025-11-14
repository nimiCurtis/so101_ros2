import os
import sys

sys.path.insert(0, os.path.abspath('..'))

project = 'SO101 ROS2'
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme',
    'sphinxcontrib.video',
]
templates_path = ['_templates']
exclude_patterns = []

html_theme = 'sphinx_rtd_theme'
