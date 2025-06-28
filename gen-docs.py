#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-06-26
# 
# ---- Generating Project Documentation Using Sphinx ----
# 
# To install Sphinx dependencies::
# 
#     pip install sphinx sphinx-rtd-theme
# 
# then, to create the documentation in a 'docs' directory::
# 
#     cd your-project
#     sphinx-quickstart docs
# 
# Answer the prompts like::
# 
#     Separate source and build directories? → yes
#     Project name → your project name
#     Author → you
#     Project version → 0.1.0 (or whatever)
#     Use autodoc, viewcode → yes
#     Use Makefile → yes

import subprocess
from pathlib import Path
import shutil
import sys
import os

PROJECT_NAME = "Brushless Motor Controller"
AUTHOR_NAME = "Ichiro Furusato"
VERSION = "0.1.0"

DOCS_DIR = Path("docs")
SOURCE_DIR = DOCS_DIR / "source"
CONF_PY = SOURCE_DIR / "conf.py"
INDEX_RST = SOURCE_DIR / "index.rst"
MODULES_RST = SOURCE_DIR / "modules.rst"

# Remove old docs directory if it exists
if DOCS_DIR.exists():
    print("Removing existing '{0}' directory...".format(DOCS_DIR))
    shutil.rmtree(DOCS_DIR)

# Run sphinx-quickstart
DOCS_DIR.mkdir(parents=True, exist_ok=True)
cmd = [
    "sphinx-quickstart",
    str(DOCS_DIR),
    "--quiet",
    "--project", PROJECT_NAME,
    "--author", AUTHOR_NAME,
    "--release", VERSION,
    "--language", "en",
    "--sep",
    "--makefile",
    "--ext-autodoc",
    "--ext-viewcode"
]
print("Running sphinx-quickstart...")
subprocess.run(cmd, check=True)

# Patch conf.py to add napoleon and sys.path
conf_text = CONF_PY.read_text()

if "sphinx.ext.napoleon" not in conf_text:
    conf_text = conf_text.replace(
        "extensions = [",
        "extensions = [\n    'sphinx.ext.napoleon',"
    )

if "sys.path.insert" not in conf_text:
    conf_text = conf_text.replace(
        "import sys",
        "import sys\nimport os\nsys.path.insert(0, os.path.abspath('../..'))  # add project root"
    )

CONF_PY.write_text(conf_text)
print("Patched conf.py to add napoleon and sys.path")

# Auto-generate modules.rst
def find_modules(base_dir: Path):
    py_files = list(base_dir.rglob("*.py"))
    modules = []
    for f in py_files:
        if f.name == "__init__.py":
            continue
        if f.name.startswith("test_"):
            continue
        rel = f.relative_to(base_dir).with_suffix("")
        parts = rel.parts
        module_name = ".".join(parts)
        modules.append(module_name)
    return modules

project_root = Path.cwd()
modules = find_modules(project_root)

lines = [
    "Modules",
    "=======",
    "",
]

for mod in sorted(modules):
    lines.append(".. automodule:: {0}".format(mod))
    lines.append("    :members:")
    lines.append("    :undoc-members:")
    lines.append("    :show-inheritance:")
    lines.append("")

MODULES_RST.write_text("\n".join(lines))
print("Generated {0} with {1} modules".format(MODULES_RST, len(modules)))

# Update index.rst to include modules.rst
index_text = INDEX_RST.read_text()
if "modules" not in index_text:
    index_text += "\n.. toctree::\n   :maxdepth: 2\n\n   modules\n"
    INDEX_RST.write_text(index_text)
    print("Updated index.rst to include modules.rst")

# Build HTML docs automatically
print("Building HTML documentation by running 'make html' in docs/")
try:
    subprocess.run(["make", "html"], cwd=str(DOCS_DIR), check=True)
    print("HTML documentation built successfully!")
except subprocess.CalledProcessError as e:
    print("Failed to build HTML docs: {0}".format(e))
    sys.exit(1)

print("\nAll done! Documentation is ready in the docs directory.")

