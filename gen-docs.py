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
# Try sphinx-lint:
#
# pip3 install sphinx-lint
# sphinx-lint .
#
# Or use pydocstyle for more aggressive linting:
#
# pip3 install pydocstyle
# pydocstyle .
#

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

# Get absolute project root path (where the script is run)
project_root = Path.cwd().resolve()

# Patch conf.py to add napoleon, autodoc_mock_imports, sys.path with absolute path, and set sphinx_rtd_theme
conf_text = CONF_PY.read_text()

# Add napoleon extension if missing
if "sphinx.ext.napoleon" not in conf_text:
    conf_text = conf_text.replace(
        "extensions = [",
        "extensions = [\n    'sphinx.ext.napoleon',"
    )

# Add autodoc_mock_imports line if missing
mock_imports_line = "autodoc_mock_imports = ['pigpio', 'spidev', 'RPi', 'smbus', 'ioexpander']"
if mock_imports_line not in conf_text:
    # Insert after extensions block (after the closing bracket)
    insert_after = conf_text.find("extensions = [")
    if insert_after != -1:
        # Find end of extensions list (the closing ']')
        end_of_extensions = conf_text.find("]", insert_after)
        if end_of_extensions != -1:
            conf_text = (
                conf_text[:end_of_extensions+1] +
                "\n\n" + mock_imports_line +
                conf_text[end_of_extensions+1:]
            )
    else:
        # Fallback: add at start
        conf_text = mock_imports_line + "\n\n" + conf_text

# Ensure imports of sys and os exist, add sys.path insert with absolute path
if "import sys" not in conf_text:
    conf_text = "import sys\nimport os\n\n" + conf_text
elif "import os" not in conf_text:
    conf_text = conf_text.replace("import sys", "import sys\nimport os")

lines = conf_text.splitlines()
insert_idx = 0
for i, line in enumerate(lines):
    if line.startswith("import ") or line.startswith("from "):
        insert_idx = i + 1
sys_path_line = f"sys.path.insert(0, os.path.abspath(r'{project_root}'))  # add project root"
if sys_path_line not in conf_text:
    lines.insert(insert_idx, sys_path_line)
conf_text = "\n".join(lines)

# Set html_theme to sphinx_rtd_theme
import re
if "html_theme" in conf_text:
    conf_text = re.sub(r"html_theme\s*=\s*['\"].*?['\"]", "html_theme = 'sphinx_rtd_theme'", conf_text)
else:
    conf_text += "\nhtml_theme = 'sphinx_rtd_theme'\n"

# Add import sphinx_rtd_theme if missing
if "import sphinx_rtd_theme" not in conf_text:
    conf_text = "import sphinx_rtd_theme\n" + conf_text

# add newline at end
conf_text = conf_text.rstrip() + "\n"

CONF_PY.write_text(conf_text)
print("Patched conf.py to add napoleon, autodoc_mock_imports, sys.path, and set sphinx_rtd_theme")

# Auto-generate modules.rst
def find_modules(base_dir: Path):
    py_files = list(base_dir.rglob("*.py"))
    modules = []
    for f in py_files:
        if f.name == "__init__.py":
            continue
        if f.name.startswith("test_"):
            continue
        if f.name.endswith("_test.py"):
            continue
        if "upy" in f.parts:  # skip MicroPython code
            continue
        if f.name in ("gen-docs.py", "generate-docs.py"):  # skip docs generator scripts
            continue
        rel = f.relative_to(base_dir).with_suffix("")
        parts = rel.parts
        module_name = ".".join(parts)
        modules.append(module_name)
    return modules

modules = find_modules(project_root)

lines = [
    "Modules",
    "=======",
    "",
]

for mod in sorted(modules):
    lines.append(f".. automodule:: {mod}")
    lines.append("    :members:")
    lines.append("    :undoc-members:")
    lines.append("    :show-inheritance:")
    # Customize exclusions per module
    if mod.endswith("globals"):
        lines.append("    :exclude-members: has, get, put, init")
    else:
        lines.append("    :exclude-members: main, init")
    lines.append("")

MODULES_RST.write_text("\n".join(lines))
print(f"Generated {MODULES_RST} with {len(modules)} modules")

# Update index.rst to include modules.rst
index_text = INDEX_RST.read_text()
if "modules" not in index_text:
    index_text += "\n.. toctree::\n   :maxdepth: 2\n\n   modules\n"
    INDEX_RST.write_text(index_text)
    print("Updated index.rst to include modules.rst")

# Build HTML docs automatically
print("Building HTML documentation by running 'make html' in docs/")
try:
    PICKY = True
    if PICKY:
#       subprocess.run(["make", "html", "SPHINXOPTS=-n -W"], cwd=str(DOCS_DIR), check=True) # will fail on warnings
        subprocess.run(["make", "html", "SPHINXOPTS=-n"], cwd=str(DOCS_DIR), check=True)
    else:
        subprocess.run(["make", "html"], cwd=str(DOCS_DIR), check=True)
    print("HTML documentation built successfully!")
except subprocess.CalledProcessError as e:
    print(f"Failed to build HTML docs: {e}")
    sys.exit(1)

print("\nAll done! Documentation is ready in the docs directory.")

