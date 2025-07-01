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

class MotorTableRenderer:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.fields = [
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
        self.motors = self._load_motors()
        self.result = StringBuilder()

    def _load_motors(self):
        with open(self.yaml_path, "r") as f:
            config = yaml.safe_load(f)
        return config["kros"]["motors"]

    def _compute_column_widths(self, headers, rows):
        col_widths = []
        for i in range(len(headers)):
            max_width = len(headers[i])
            for row in rows:
                max_width = max(max_width, len(row[i]))
            col_widths.append(max_width)
        return col_widths

    def _format_row(self, row, col_widths, sep="|"):
        cells = []
        for i, cell in enumerate(row):
            cell_fmt = " {{:<{}}} ".format(col_widths[i])
            cells.append(cell_fmt.format(cell))
        return sep + sep.join(cells) + sep

    def _separator(self, col_widths, char="-"):
        return "+" + "+".join(char * (w + 2) for w in col_widths) + "+"

    def render_table(self):
        headers = [f[1] for f in self.fields]
        field_keys = [f[0] for f in self.fields]

        rows = []
        for motor_name in sorted(self.motors):
            motor = self.motors[motor_name]
            row = [str(motor.get(field, "")) for field in field_keys]
            rows.append(row)

        col_widths = self._compute_column_widths(headers, rows)
        result = StringBuilder()

        result.append(self._separator(col_widths, "=") + "\n")
        result.append(self._format_row(headers, col_widths) + "\n")
        result.append(self._separator(col_widths, "=") + "\n")

        for row in rows:
            result.append(self._format_row(row, col_widths) + "\n")
            result.append(self._separator(col_widths) + "\n")

        return result.to_string()

    def prepend_template(self, filepath):
        '''
        Reads text from the given file and prepends it to the result buffer.
        '''
        with open(filepath, "r") as f:
            header_text = f.read()
        # Prepend by creating a new StringBuilder with header + existing result
        new_result = StringBuilder()
        new_result.append(header_text)
        new_result.append("\n")  # Ensure separation
        new_result.append(self.result.to_string())
        self.result = new_result

