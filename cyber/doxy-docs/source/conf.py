#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess

import sphinx_rtd_theme


on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

if on_rtd:
    subprocess.call('cd ..; doxygen', shell=True)


html_theme = "sphinx_rtd_theme"

html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]


def setup(app):
    app.add_stylesheet("main_stylesheet.css")


extensions = ['breathe', 'recommonmark']
breathe_projects = {'Cyber RT Documents': '../xml'}
templates_path = ['_templates']
html_static_path = ['_static']
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}
master_doc = 'index'
project = 'Cyber RT Documents'
copyright = '2019, Apollo'
author = 'Apollo Baidu'

#html_logo = 'quantstack-white.svg'

exclude_patterns = []
highlight_language = 'c++'
pygments_style = 'sphinx'
todo_include_todos = False
htmlhelp_basename = 'CyberRTdoc'
