# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import sys
import os
sys.path.insert(0, os.path.abspath('../'))

project = 'ME405-Romi'
copyright = '2023, Jack Miller & Casey Pickett'
author = 'Jack Miller & Casey Pickett'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration


# Add napoleon (numpy & google docstring support) to the extensions list
extensions = [
    'sphinx.ext.todo',
    'sphinx.ext.napoleon',
    'sphinx.ext.autodoc',
]

templates_path = ['_templates']
exclude_patterns = ['_build',
    'Thumbs.db',
    '.DS_Store',
    'boot.rst',
    'main_bno055_example.rst',
    'main_lab3.rst',
    'main_lab4.rst',
    'main_lab5.rst',
    'QTRSensors_ReadlineCalibrate.rst',
    'vl53l1x_example.rst',
]



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
