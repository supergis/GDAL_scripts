#!/usr/bin/env python
###############################################################################
# $Id: gdal2AsciiLatLonBands.py 1 2014-10-17 
#
# Project:  GDAL
# Purpose:  Script to translate GDAL supported raster into Ascii (Lat, Lon, Band n)
#                 or (Y, X, Band 1, Band 2, Band n)
# Author:   thare@usgs.gov, Trent Hare
#           based on GDAL samples from GDAL and python samples
#           http://svn.osgeo.org/gdal/trunk/gdal/swig/python/samples/
#
###############################################################################
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

from osgeo import gdal, gdal_array
from osgeo.gdalconst import *
from osgeo import osr
gdal.TermProgress = gdal.TermProgress_nocb
import numpy as np
import sys


# =============================================================================
# If more than one band is sent, then last band will be sent to output
def Usage():
    print('Usage: gdal2AsciiLatLonBands.py [-srcwin xoff yoff width height]')
    print('   [-band 1] [-band 2] [-band n] [-addheader] [-printLatLon] [-printYX] srcfile [dstfile]')
    print('--defaults to band 1 if nothing is sent')
    print('--srcwin offsets, width, and height values should be sent in meters')
    print('')
    sys.exit( 1 )

# =============================================================================
#
# Program mainline.
#

if __name__ == '__main__':

    srcwin = None
    skip = 1    
    srcfile = None
    dstfile = None
    addheader=False
    band_nums = []
    LatLon = True
    printLatLon=False
    printYX=False

    gdal.AllRegister()
    argv = gdal.GeneralCmdLineProcessor( sys.argv )
    if argv is None:
        sys.exit( 0 )

    # Parse command line arguments.
    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == '-srcwin':
            srcwin = (int(argv[i+1]),int(argv[i+2]),
                      int(argv[i+3]),int(argv[i+4]))
            i = i + 4
        elif arg == '-band':
            band_nums.append( int(argv[i+1]) )
            i = i + 1
        elif arg == '-addheader':
            addheader = True
        elif arg == '-printLatLon':
            printLatLon = True
            LatLon = True
        elif arg == '-printYX':
            printYX = True
        elif arg[0] == '-':
            Usage()
        elif srcfile is None:
            srcfile = arg
        elif dstfile is None:
            dstfile = arg
        else:
            Usage()
        i = i + 1

    if srcfile is None:
        Usage()

    stcdata = gdal.Open( srcfile, GA_ReadOnly )

    if band_nums == []: band_nums = [1]

    # Open source file. 
    srcdata = gdal.Open( srcfile )
    if srcdata is None:
        print('Could not open %s.' % srcfile)
        sys.exit( 1 )

    bands = []
    for band_num in band_nums: 
        band = srcdata.GetRasterBand(band_num)
        if band is None:
            print('Could not get band %d' % band_num)
            sys.exit( 1 )
        bands.append(band)

    gt = srcdata.GetGeoTransform()
    # simple check if the input is LatLon
    #if abs(gt[0]) <= 360 and abs(gt[3]) <= 360 \
        #and abs(indataset.RasterXSize * gt[1]) <= 360 \
        #and abs(indataset.RasterYSize * gt[5]) <= 360:
        #LatLon = True

    geomatrix = srcdata.GetGeoTransform()
    # Build Spatial Reference object based on coordinate system, fetched from the
    # opened dataset
    srs = osr.SpatialReference()
    srs.ImportFromWkt(srcdata.GetProjection())
    srsLatLong = srs.CloneGeogCS()
    coordtransform = osr.CoordinateTransformation(srs, srsLatLong)

    # Collect information on all the source files.
    if srcwin is None:
        srcwin = (0,0,srcdata.RasterXSize,srcdata.RasterYSize)

    # Open the output file.
    if dstfile is not None:
        dst_fh = open(dstfile,'wt')
    else:
        dst_fh = sys.stdout

    if addheader:
        if printLatLon:
            dst_fh.write( "Lat,Lon," )
        if printYX:
            dst_fh.write( "Y,X," )
        cnt = 1
        bStr = ""
        for band in bands:
           bStr = bStr + ( "band_%d," % (cnt) )
           cnt = cnt + 1
        bStr = bStr.rstrip(',')
        dst_fh.write( "%s\n" % (bStr) )

    band_format = ("%g," * len(bands)).rstrip(',') + '\n'
    if (printLatLon or printYX):
       lformat = '%.6f,%.6f,%s'
    else:
       lformat = '%s'

    #loop over lines 
    for y in range(srcwin[1],srcwin[1]+srcwin[3],skip):

        #for each line, grab all bands requested into numpy array
        data = []
        for band in bands:
            band_data = band.ReadAsArray(srcwin[0], y, srcwin[2], skip)
            band_data = np.reshape( band_data, (srcwin[2],) )
            data.append(band_data)

        #Loop over samples (X)
        for x_i in range(0,srcwin[2],skip):
            x = x_i + srcwin[0]
            geo_x = geomatrix[0] + (x+0.5) * geomatrix[1] + (y+0.5) * geomatrix[2]
            geo_y = geomatrix[3] + (x+0.5) * geomatrix[4] + (y+0.5) * geomatrix[5]

            x_i_data = []
            for i in range(len(bands)):
                x_i_data.append(data[i][x_i])

            band_str = band_format % tuple(x_i_data)
            #band_str = ','.join(band_str)

            #convert Y/X meters from image projection to lat/on
            if printLatLon:
                (geo_x, geo_y, height) = coordtransform.TransformPoint(geo_x, geo_y)

            #write out line to output. 
            #The check here for large values can be removed.
            if (abs(x_i_data[0]) < 1.0E12):
               if (printLatLon or printYX):
                   line = lformat % (float(geo_y),float(geo_x), band_str)
               else:
                   line = lformat % (band_str)
               dst_fh.write( line )
