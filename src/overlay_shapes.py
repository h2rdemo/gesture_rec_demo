#!/usr/bin/env python

import roslib
from ros_overlay.msg import Point
from ros_overlay.msg import Shape

DEFAULT_THICKNESS = 1.00
DEFAULT_COLOR = [255, 0, 128]
DEFAULT_FILL = False

def overlay_circle(id, (x, y), radius, color=DEFAULT_COLOR, fill=DEFAULT_FILL, thickness=DEFAULT_THICKNESS):
	rv = Shape()
	rv.id = id
	rv.delete = False
	rv.type = Shape.SHP_CIRCLE
	rv.fill = fill
	rv.thickness = thickness
	rv.color = color
	rv.poly = [Point(x, y)]
	rv.param = [radius]

	return rv

def overlay_delete(id):
	rv = Shape()
	rv.id = id
	rv.delete = True

	return rv