#!/usr/bin/env python
###############################################################################
# $Id$
#
# Project:  GDAL Python samples
# Purpose: convert lon,lat bounds to to X,Y meter coordinates given an input image for 
#          the projection transformation to compute size.
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
    print('\nGiven a pair of bounding longs, lats, resolution(m), bittype (8,16,32), and number of bands')
    print('the script will report uncompressed file size. Currently, the projection from the input image.\n')
    print('Usage: gdalsize.py minlong minlat maxlong maxlat res bitType bands infile\n')
    sys.exit( 1 )

# =============================================================================

minlong = None
minlat = None
maxlong = None
maxlat = None
res = None
bitType = None
bands = None
infile = None

# =============================================================================
# Parse command line arguments.
# =============================================================================
i = 1
while i < len(sys.argv):
    arg = sys.argv[i]

    if minlong is None:
        minlong = float(arg)
    elif minlat is None:
        minlat = float(arg)
    elif maxlong is None:
        maxlong = float(arg)
    elif maxlat is None:
        maxlat = float(arg)
    elif res is None:
        res = float(arg)
    elif bitType is None:
        bitType = float(arg)
    elif bands is None:
        bands = float(arg)
    elif infile is None:
        infile = arg
    else:
        Usage()

    i = i + 1

if maxlat is None:
    Usage()
if maxlong is None:
    Usage()
if minlat is None:
    Usage()
if minlong is None:
    Usage()
if res is None:
    Usage()
if bitType is None:
    Usage()
if bands is None:
    Usage()
if infile is None:
    Usage()

# Open input dataset
indataset = gdal.Open( infile, GA_ReadOnly )

# Read geotransform matrix and calculate ground coordinates
#geomatrix = indataset.GetGeoTransform()
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
ct = osr.CoordinateTransformation(srsLatLong, srs)
(minX, minY, height) = ct.TransformPoint(minlong, minlat)
(maxX, maxY, height) = ct.TransformPoint(maxlong, maxlat)

lines = abs(maxX - minX) / res
samples = abs(maxY - minY) / res

if bitType == 8:
    imageSizeMB = lines * samples * bands
elif bitType == 16:
    imageSizeMB = lines * samples * bands * 2
elif bitType == 32:
    imageSizeMB = lines * samples * bands * 4
else:
    print('bitType of %f not supported' % (bitType))


# Report results
#print('minlong: %f\t\tminlat: %f' % (minlong, minlat))
#print('minX: %f\t\tminY: %f' % (minX, minY))
#print('maxlong: %f\t\tmaxlat: %f' % (maxlong, maxlat))
#print('maxX: %f\t\tmaxY: %f' % (maxX, maxY))
print('\n%.1f in Megabytes' % (imageSizeMB))
print('%.1f in Gigabytes\n' % round(imageSizeMB / 1073741824 ))


