#!/usr/bin/env python
###############################################################################
# $Id$
#
# Project:  GDAL Python samples
# Purpose: convert long,lat to X,Y meter (or feet) coordinates based on the
#          projection from a given image.
# Author: Trent Hare (USGS)
# Based on tolatlong by Andrey Kiselev, dron@remotesensing.org
#
###############################################################################
# Copyright (c) 2003, Andrey Kiselev <dron@remotesensing.org>
# 
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
###############################################################################

try:
    from osgeo import gdal
    from osgeo import osr
    from osgeo.gdalconst import *
except ImportError:
    import gdal
    from gdalconst import *

import sys

# =============================================================================
def Usage():
    print('')
    print('report long,lat from X,Y in meter (or feet) coordinates based on the')
    print('projection from given image.')
    print('')
    print('Usage: meters2longlat.py X Y infile')
    print('')
    sys.exit( 1 )

# =============================================================================

infile = None
X = None
Y = None

# =============================================================================
# Parse command line arguments.
# =============================================================================
i = 1
while i < len(sys.argv):
    arg = sys.argv[i]

    if X is None:
        X = float(arg)

    elif Y is None:
        Y = float(arg)

    elif infile is None:
        infile = arg

    else:
        Usage()

    i = i + 1

if infile is None:
    Usage()
if X is None:
    Usage()
if Y is None:
    Usage()

# Open input dataset
indataset = gdal.Open( infile, GA_ReadOnly )

# Read geotransform matrix and calculate ground coordinates
geomatrix = indataset.GetGeoTransform()
#X = geomatrix[0] + geomatrix[1] * pixel + geomatrix[2] * line
#Y = geomatrix[3] + geomatrix[4] * pixel + geomatrix[5] * line

# Shift to the center of the pixel
#X += geomatrix[1] / 2.0
#Y += geomatrix[5] / 2.0

# Build Spatial Reference object based on coordinate system, fetched from the
# opened dataset
srs = osr.SpatialReference()
srs.ImportFromWkt(indataset.GetProjection())

srsLatLong = srs.CloneGeogCS()
ct = osr.CoordinateTransformation(srs, srsLatLong)
(long, lat, height) = ct.TransformPoint(X, Y)

# Report results
print('X: %f\t\t\tY: %f' % (X, Y))
print('longitude: %.6f\t\t\tlatitude: %.6f' % (long, lat))
print('longitude: %s\t\tlatitude: %s' % (gdal.DecToDMS(long, 'Long', 2), gdal.DecToDMS(lat, 'Lat', 2)))

