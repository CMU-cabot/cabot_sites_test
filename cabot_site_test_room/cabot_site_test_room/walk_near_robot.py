# Copyright (c) 2024  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import ros
import math
from pedestrian import state

stopped = False


def onUpdate(**args):
    name = args['name']
    relative_x = args.get('relative_x', -1.0)  # [m]
    relative_y = args.get('relative_y', -1.0)  # [m]

    # plugin variables
    x = args['x']
    y = args['y']
    yaw = args['yaw']

    # wait until robot pose is available
    if 'robot' not in args:
        return args

    # plugin variables
    rx = args['robot']['x']
    ry = args['robot']['y']
    ryaw = args['robot']['yaw']

    x = rx + math.cos(ryaw) * relative_x
    y = ry + math.sin(ryaw) * relative_y
    yaw = ryaw

    # update actor variables
    args['x'] = x
    args['y'] = y
    args['yaw'] = yaw

    return args
