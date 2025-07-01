#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-01
# modified: 2025-07-01

import yaml
from upy.stringbuilder import StringBuilder

# Load YAML configuration
with open("upy/motor_config.yaml", "r") as f:
    config = yaml.safe_load(f)

# Fields to extract (and header labels)
fields = [
    ("id", "Id"),
    ("name", "Name"),
    ("pwm_timer", "PWM Timer"),
    ("pwm_channel", "PWM Ch"),
    ("pwm_pin_name", "PWM Pin"),
    ("direction_pin_name", "Dir Pin"),
    ("encoder_pin_name", "Enc Pin"),
    ("enc_timer", "Enc Timer"),
    ("enc_channel", "Enc Ch")
]

# Extract just the field keys and headers
field_keys = [f[0] for f in fields]
headers = [f[1] for f in fields]

# Collect motor data
motors = config["kros"]["motors"]
rows = []
for motor_name in sorted(motors):
    motor = motors[motor_name]
    row = [str(motor.get(field, "")) for field in field_keys]
    rows.append(row)

# Determine column widths
col_widths = []
for i in range(len(headers)):
    max_width = len(headers[i])
    for row in rows:
        max_width = max(max_width, len(row[i]))
    col_widths.append(max_width)

# Helpers
def format_row(row, sep="|"):
    cells = []
    for i, cell in enumerate(row):
        cell_fmt = " {{:<{}}} ".format(col_widths[i])
        cells.append(cell_fmt.format(cell))
    return sep + sep.join(cells) + sep

def separator(char="-"):
    return "+" + "+".join(char * (w + 2) for w in col_widths) + "+"

# Use StringBuilder for result collection
result = StringBuilder()

# Build the RST table
result.append(separator("=") + "\n")
result.append(format_row(headers) + "\n")
result.append(separator("=") + "\n")

for row in rows:
    result.append(format_row(row) + "\n")
    result.append(separator() + "\n")

# Final output string
output = result.to_string()

# Save or use as needed
with open("motors_table.rst", "w") as f:
    f.write(output)

