# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'NorthStarFleet2025'
copyright = '2020, NorthStarFleet2025'
author = 'NorthStarFleet2025'

# The full version, including alpha/beta/rc tags
release = '1.0.0'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
# NOTE: extensions are configured once, further down, alongside the breathe/exhale settings
# they belong with.

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# No custom static files yet. Re-add html_static_path = ['_static'] alongside a _static/
# directory if a stylesheet or logo is introduced; pointing at a missing directory warns on
# every build.


breathe_default_project = "NorthStarFleet2025"

extensions = [
    'breathe',   # pulls Doxygen XML into Sphinx
    'exhale',    # generates a page per C++ symbol from that XML
]

# Setup the breathe extension
breathe_projects = {
    "NorthStarFleet2025": "./doxyoutput/xml"
}

# Setup the exhale extension
exhale_args = {
    # These arguments are required
    "containmentFolder":     "./api",
    "rootFileName":          "library_root.rst",
    "rootFileTitle":         "Code Reference",
    "doxygenStripFromPath":  "..",
    # Suggested optional arguments
    "createTreeView":        True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    # Doxygen is run by the Makefile instead, so gen_subsystem_pages.py can read its XML
    # before sphinx-build starts.
    "exhaleExecutesDoxygen": False,
    "exhaleUseDoxyfile":     True
}

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'