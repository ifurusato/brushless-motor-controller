#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-06-12
# modified: 2025-07-01
#
# ---- Generating Project Documentation Using Sphinx ----
#
# To install Sphinx dependencies::
#
#     pip install sphinx sphinx-rtd-theme
#
# Try sphinx-lint:
#
# pip3 install sphinx-lint
# sphinx-lint .
#

import os, sys
import subprocess
from pathlib import Path
from datetime import datetime
import shutil
from colorama import init, Fore, Style
init()

from motor_table_renderer import MotorTableRenderer

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

REMOVE_BUILD_UPON_COMPLETE = True

PROJECT_NAME = "Brushless Motor Controller"
AUTHOR_NAME = "Ichiro Furusato"
VERSION = "0.1.0"

# Change this to the temp directory
DOCS_DIR = Path("temp")
SOURCE_DIR = DOCS_DIR / "source"
CONF_PY = SOURCE_DIR / "conf.py"
INDEX_RST = SOURCE_DIR / "index.rst"
MODULES_RST = SOURCE_DIR / "modules.rst"

# Remove old docs directory if it exists
if DOCS_DIR.exists():
    print(Fore.YELLOW + "Removing existing '{0}' directory…".format(DOCS_DIR) + Style.RESET_ALL)
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
print(Fore.YELLOW + "Running sphinx-quickstart…" + Style.RESET_ALL)
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
print(Fore.YELLOW + "Patched conf.py to add napoleon, autodoc_mock_imports, sys.path, and set sphinx_rtd_theme" + Style.RESET_ALL)

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
        if f.name in ("gen-docs.py", "gen-table.py", "motor_table_renderer.py"):  # skip docs generator scripts
            continue
        rel = f.relative_to(base_dir).with_suffix("")
        parts = rel.parts
        module_name = ".".join(parts)
        modules.append(module_name)
    return modules

modules = find_modules(project_root)

# sidebar ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

lines = []

# append lines from sphinx-outline.rst
outline_file = project_root / "sphinx-outline.rst"
if outline_file.exists():
    _lines = outline_file.read_text().splitlines()
    for _line in _lines:
        lines.append(_line)

# append lines from table-rendered motor_config.yaml
renderer = MotorTableRenderer("upy/motor_config.yaml")
#renderer.prepend_template("sphinx-pinout.rst")
rendered_table = renderer.render_table()

pwm_timer = str(renderer.get_pwm_timer())
enc_timer = str(renderer.get_enc_timer())

# append lines from sphinx-pinout.rst
pinout_file = project_root / "sphinx-pinout.rst"
if pinout_file.exists():
    _text = pinout_file.read_text()
    _text = _text.replace('{{PWM_TIMER}}', pwm_timer)
    _text = _text.replace('{{ENC_TIMER}}', enc_timer)
    _text = _text.replace('{{PINOUT}}', rendered_table)
    _lines = _text.splitlines()
    for _line in _lines:
#       print("line: '{}'".format(_line))
        lines.append(_line)

# append lines from sphinx-modules.rst
modules_file = project_root / "sphinx-modules.rst"
if modules_file.exists():
    _lines = modules_file.read_text().splitlines()
    for _line in _lines:
        lines.append(_line)

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
print(Fore.YELLOW + "Generated {} with {} modules".format(MODULES_RST, len(modules)) + Style.RESET_ALL)

# Optional external prefix file for index.rst
index_prefix_file = project_root / "sphinx-index.rst"
today = datetime.now().strftime("%d %b %Y")

# Start with prefix from file, fallback to empty string
if index_prefix_file.exists():
    index_lines = index_prefix_file.read_text().splitlines()
else:
    index_lines = []

# Replace placeholder {{date}} with today in all lines
index_lines = [line.replace("{{date}}", today) for line in index_lines]

# Ensure toctree is included
toctree_lines = [
    "",
    ".. toctree::",
    "   :maxdepth: 2",
    "   :caption: Features",
    "",
    "   modules",
]

# Combine all lines and write out
final_index = "\n".join(index_lines + toctree_lines) + "\n"
INDEX_RST.write_text(final_index)
print(Fore.YELLOW + "Generated index.rst with overview and toctree for modules" + Style.RESET_ALL)

# Build HTML docs automatically
print(Fore.YELLOW + "Building HTML documentation by running 'make html'…" + Style.RESET_ALL)
try:
    PICKY = True
    if PICKY:
#       subprocess.run(["make", "html", "SPHINXOPTS=-n -W"], cwd=str(DOCS_DIR), check=True) # will fail on warnings
        subprocess.run(["make", "html", "SPHINXOPTS=-n"], cwd=str(DOCS_DIR), check=True)
    else:
        subprocess.run(["make", "html"], cwd=str(DOCS_DIR), check=True)
    print(Fore.YELLOW + "HTML documentation built successfully!" + Style.RESET_ALL)
except subprocess.CalledProcessError as e:
    print(Fore.RED + "Failed to build HTML docs: {}".format(e) + Style.RESET_ALL)
    sys.exit(1)

# Copy the generated HTML docs from 'temp/build/html' to 'docs'
final_docs_dir = Path("docs")
if not final_docs_dir.exists():
    print(Fore.YELLOW + "Creating final docs/ directory…" + Style.RESET_ALL)
    final_docs_dir.mkdir(parents=True)

# Remove old content in docs/ if it exists
shutil.rmtree(final_docs_dir, ignore_errors=True)

# Copy new content from the build folder
shutil.copytree(DOCS_DIR / "build" / "html", final_docs_dir)

# copy .nojekyll to docs
shutil.copyfile(".nojekyll", os.path.join("docs", ".nojekyll"))

if REMOVE_BUILD_UPON_COMPLETE:
    # delete the temp directory
    shutil.rmtree(DOCS_DIR, ignore_errors=True)
else:
    print(Fore.YELLOW + "\nNote: The temporary build directory was not deleted." + Style.RESET_ALL)

print(Fore.GREEN + "\nAll done! Documentation is ready in the docs directory." + Style.RESET_ALL)

#EOF
