#!/micropython
# -*- coding: utf-8 -*-
#
# Copyright 2020-2025 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2025-07-09
# modified: 2025-07-09

class ZeroCrossingStage:
    '''
    Pseudo-Enum for Zero Crossing Stages.
    '''
    _instances = []

    def __init__(self, index, code, description):
        self._index = index
        self._code = code
        self._description = description
        ZeroCrossingStage._instances.append(self)

    @property
    def index(self):
        return self._index

    @property
    def code(self):
        return self._code

    @property
    def description(self):
        return self._description

    def __str__(self):
        return self._code

    def __repr__(self):
        return 'ZeroCrossingStage({}, {})'.format(self._index, self._code)

    @classmethod
    def all(cls):
        return cls._instances

    @classmethod
    def from_index(cls, index):
        for inst in cls._instances:
            if inst.index == index:
                return inst
        raise ValueError('No ZeroCrossingStage with index "{}"'.format(index))

    @classmethod
    def from_code(cls, code):
        for inst in cls._instances:
            if inst.code == code:
                return inst
        raise ValueError('No ZeroCrossingStage with code "{}"'.format(code))

# Instantiate your ZC Stages
ZeroCrossingStage.IDLE            = ZeroCrossingStage(0, 'IDLE',            'zero-crossing handler is idle')
ZeroCrossingStage.DECELERATING    = ZeroCrossingStage(1, 'DECELERATING',    'decelerating to zero RPM')
ZeroCrossingStage.CONFIRMING_STOP = ZeroCrossingStage(2, 'CONFIRMING_STOP', 'confirming motor has stopped physically')
ZeroCrossingStage.ACCELERATING    = ZeroCrossingStage(3, 'ACCELERATING',    'accelerating to final target RPM')
ZeroCrossingStage.COMPLETE        = ZeroCrossingStage(4, 'COMPLETE',        'zero-crossing process completed')

#EOF
