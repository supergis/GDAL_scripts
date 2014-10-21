#!/usr/bin/env python
###############################################################################
# $Id: gdal2xyz_geocentricSpace.py 2014-10-21
#
# Project:  GDAL
# Purpose:  Script to translate GDAL supported raster (specifically a elevation DEM)
#           into a geocentric (body-fixed) XYZ ASCII table.
#           Defaults to Moon radius and DEM elevation values should in meters
# Author:   Frank Warmerdam, warmerdam@pobox.com, original gdal2xyz version
# update:   Trent Hare, Jan 31, 2011 to convert to space coordinates - only use degs
# update:   Trent Hare, Feb 1, 2011 now supports any GDAL meter projection as input
# update:   Trent Hare, Oct 21, 2014 now supports Lat/Lon from Bands
#
###############################################################################
# Copyright (c) 2002, Frank Warmerdam <warmerdam@pobox.com>
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
    import osr
    from gdalconst import *

import sys
import math

try:
    import numpy as Numeric
except ImportError:
    import Numeric

# =============================================================================
def Usage():
    print 'Usage: gdal2xyz_geocentricSpace.py [-skip factor] [-printLatLon] [-addheader] [-srcwin xoff yoff width height]'
    print '     [-radius value_m or -radiusBand n] [-latBand n] [-lonBand n] [-band b] srcfile [dstfile]'
    print 'Note: Was written for digital elevation files (DEMs), thus band 1 or -band b, should be elevation in meters'
    print 'Note: if no radius is sent, the radius will default to the Moon = 1737400.0'
    print 'Note: if variable radius is available as a band, then you can send -radiusBand b'
    print
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
    band_nums = []
    addheader=False
    LatLon=False
    printLatLon=False
    latBand_num = None
    lonBand_num = None
    radiusBand_num = None
    
    #Moon's radius
    theRadius = 1737400.0 

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
        elif arg == '-skip':
            skip = int(argv[i+1])
            i = i + 1
        elif arg == '-radius':
            theRadius = float(argv[i+1])
            i = i + 1
        elif arg == '-latBand':
            latBand_num = int(argv[i+1])
            i = i + 1
        elif arg == '-lonBand':
            lonBand_num = int(argv[i+1])
            i = i + 1
        elif arg == '-radiusBand':
            radiusBand_num = int(argv[i+1])
            i = i + 1
        elif arg == '-band':
            band_nums.append( int(argv[i+1]) )
            i = i + 1
        elif arg == '-addheader':
            addheader = True
        elif arg == '-printLatLon':
            printLatLon = True
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

    if band_nums == []: band_nums = [1]
    # Open source file. 
    indataset = gdal.Open( srcfile )
    if indataset is None:
        print 'Could not open %s.' % srcfile
        sys.exit( 1 )

    bands = []
    for band_num in band_nums: 
        band = indataset.GetRasterBand(band_num)
        if band is None:
            print 'Could not get band %d' % band_num
            sys.exit( 1 )
        bands.append(band)

    if ((latBand_num is None) and (lonBand_num is not None)) or \
       ((latBand_num is not None) and (lonBand_num is None)):
        print '\nError: Only one Lat or one Lon Band sent. You should have bands for each.\n'
        Usage ()

    if latBand_num is not None:
        latBand = indataset.GetRasterBand(latBand_num)
        if latBand is None:
            print 'Could not get Latitude band %d' % latBand_num
            sys.exit( 1 )

    if lonBand_num is not None:
        lonBand = indataset.GetRasterBand(lonBand_num)
        if lonBand is None:
            print 'Could not get Longitude band %d' % lonBand_num
            sys.exit( 1 )

    if radiusBand_num is not None:
        radiusBand = indataset.GetRasterBand(radiusBand_num)
        if radiusBand is None:
            print 'Could not get Radius band %d' % radiusBand_num
            sys.exit( 1 )

    geomatrix = indataset.GetGeoTransform()
    # Build Spatial Reference object based on coordinate system, fetched from the
    # opened dataset
    srs = osr.SpatialReference()
    srs.ImportFromWkt(indataset.GetProjection())
    #print srs
    srsLatLong = srs.CloneGeogCS()
    coordtransform = osr.CoordinateTransformation(srs, srsLatLong)

    # Collect information on all the source files.
    if srcwin is None:
        srcwin = (0,0,indataset.RasterXSize,indataset.RasterYSize)

    # Open the output file.
    if dstfile is not None:
        dst_fh = open(dstfile,'wt')
    else:
        dst_fh = sys.stdout

    if addheader:
        if printLatLon:
            dst_fh.write( "Lon,Lat,Band\n" )
        else:
            dst_fh.write( "X,Y,Radius\n" )
            
    band_format = ("%g " * len(bands)).rstrip() + '\n'
    format = '%.3f,%.3f,%.3f\n'

    # double check if the input is LatLon
    if abs(geomatrix[0]) <= 360 and abs(geomatrix[3]) <= 360 \
        and abs(indataset.RasterXSize * geomatrix[1]) <= 360 \
        and abs(indataset.RasterYSize * geomatrix[5]) <= 360:
        format = '%.6f,%.6f,%.3f\n'
        LatLon = True
        
    # Loop emitting data.
    for y in range(srcwin[1],srcwin[1]+srcwin[3],skip):

        data = []
        for band in bands:
            band_data = band.ReadAsArray( srcwin[0], y, srcwin[2], 1 )    
            band_data = Numeric.reshape( band_data, (srcwin[2],) )
            data.append(band_data)

        latData = []
        if latBand_num is not None:
            band_data = latBand.ReadAsArray( srcwin[0], y, srcwin[2], 1 )    
            band_data = Numeric.reshape( band_data, (srcwin[2],) )
            latData = band_data

        lonData = []
        if lonBand_num is not None:
            band_data = lonBand.ReadAsArray( srcwin[0], y, srcwin[2], 1 )    
            band_data = Numeric.reshape( band_data, (srcwin[2],) )
            lonData = band_data

        radiusData = []
        if radiusBand_num is not None:
            band_data = radiusBand.ReadAsArray( srcwin[0], y, srcwin[2], 1 )    
            band_data = Numeric.reshape( band_data, (srcwin[2],) )
            radiusData = band_data

        for x_i in range(0,srcwin[2],skip):

            x = x_i + srcwin[0]

            geo_x = geomatrix[0] + (x+0.5) * geomatrix[1] + (y+0.5) * geomatrix[2]
            geo_y = geomatrix[3] + (x+0.5) * geomatrix[4] + (y+0.5) * geomatrix[5]

            x_i_data = []
            for i in range(len(bands)):
                x_i_data.append(data[i][x_i])
            band_str = band_format % tuple(x_i_data)

            #convert Y/X meters from image projection to lat/on
            if not LatLon:
                (geo_x, geo_y, height) = coordtransform.TransformPoint(geo_x, geo_y)

            #override - get lat from band
            if latBand_num is not None:
                geo_y = float(latData[x])

            #override - get lon from band
            if lonBand_num is not None:
                geo_x = float(lonData[x])
                
            #override - get radius from band
            if radiusBand_num is not None:
                theRadius = float(radiusData[x])
                
            #simple sphere method. Needs to be changed for ellipse
            if (abs(x_i_data[0]) < 1.0E12):
               if printLatLon: #only support a single band
                   line = format % (float(geo_x),float(geo_y), x_i_data[0])
               else:
                   #print body-fixed coordinates
                   if radiusBand_num is not None:
                       #just use radius as provided for in band
                       geoC_x = (theRadius) * math.cos(math.radians(geo_y)) * math.cos(math.radians(geo_x))
                       geoC_y = (theRadius) * math.cos(math.radians(geo_y)) * math.sin(math.radians(geo_x))
                       geoC_z = (theRadius) * math.sin(math.radians(geo_y))
                   else:
                       #radius plus elevation band
                       geoC_x = (theRadius + x_i_data[0]) * math.cos(math.radians(geo_y)) * math.cos(math.radians(geo_x))
                       geoC_y = (theRadius + x_i_data[0]) * math.cos(math.radians(geo_y)) * math.sin(math.radians(geo_x))
                       geoC_z = (theRadius + x_i_data[0]) * math.sin(math.radians(geo_y))
                   line = format % (float(geoC_x),float(geoC_y), float(geoC_z))
               dst_fh.write( line )


