#!/bin/env python
# AsterMeta2Shapefile.py
#
# Feb 2013, Trent Hare, USGS, thare@usgs.gov
#
# Description: convert a directory of Aster "meta" file into a polygon shapefile
#   *.meta files can be downloaded from the Glovis website.
# License: public domain

try:
    from osgeo import osr
    from osgeo import ogr
except ImportError:
    import osr
    import ogr
    
import os, sys, glob

# =============================================================================
#If missing args, print usage and exit
def Usage():
    print('')
    print('Usage: AsterMeta2Shapefile.py outfile.shp')
    print('')
    sys.exit( 1 )

# =============================================================================
# Parse command line arguments.
# =============================================================================
if len(sys.argv) > 1:
    outfile = sys.argv[1]
else:
    Usage()

if (os.path.isfile(outfile)):
    print "file: " + outfile + " already exists. Please remove first."
    sys.exit(0)

spatialReference = osr.SpatialReference()
spatialReference.SetWellKnownGeogCS('WGS84')

driver = ogr.GetDriverByName('ESRI Shapefile')

path="*.meta"
fileCnt = len(glob.glob(path))
#Check to see if there are any files found in directory
if (fileCnt > 0):
    new_shape = driver.CreateDataSource(outfile)
    layer = new_shape.CreateLayer('AsterPolys', spatialReference, ogr.wkbPolygon)
    field_defn = ogr.FieldDefn('Name', ogr.OFTString)
    layer.CreateField(field_defn)
    feature_def = layer.GetLayerDefn()
else:
    print "No *.meta files found in this directory"

#Loop over files
for file in glob.glob(path):
    fileCnt = fileCnt + 1
    
    #loop over lines in files
    with open(file,'r') as f:
        for line in f:
            if "ID" in line: asterID = line.split("=")[1]
            if "ULLat" in line: ULLat = float(line.split("=")[1])
            if "ULLong" in line: ULLong = float(line.split("=")[1])
            if "URLat" in line: URLat = float(line.split("=")[1])
            if "URLong" in line: URLong = float(line.split("=")[1])
            if "LRLat" in line: LRLat = float(line.split("=")[1])
            if "LRLong" in line: LRLong = float(line.split("=")[1])
            if "LLLat" in line: LLLat = float(line.split("=")[1])
            if "LLLong" in line:  LLLong = float(line.split("=")[1])

        #Create polygon as edge
        edge = ogr.Geometry(ogr.wkbLinearRing)
        edge.AddPoint(ULLong, ULLat)
        edge.AddPoint(URLong, URLat)
        edge.AddPoint(LRLong, LRLat)
        edge.AddPoint(LLLong, LLLat)
        edge.CloseRings()

        #add edge as polygon
        rectangle = ogr.Geometry(ogr.wkbPolygon)
        rectangle.AddGeometry(edge)

        #Create new feature, set geometry, set single attribute, assign to layer
        new_feature = ogr.Feature(feature_def)
        new_feature.SetGeometry(rectangle)
        new_feature.SetField('Name', asterID)
        new_feature.SetGeometry(rectangle)
        layer.CreateFeature(new_feature)

if (fileCnt > 0):
    rectangle.Destroy()
    new_feature.Destroy()
    new_shape.Destroy()
    print "complete. " + str(fileCnt) +  " files processed"


