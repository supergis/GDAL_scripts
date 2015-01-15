gdal2gores.py

Purpose:  Given a Simple Cylindrical map projected image remap to an image with n gores.
            for placing map or printing map on a sphere (e.g. tennis ball). 
Author:   Trent Hare, thare@usgs.gov
Credits:  Based on C# implementation by winski software or
          Michal Wisniewski,  http://winski.x10.mx/?page_id=2
          and python GDAL samples, 
          http://svn.osgeo.org/gdal/trunk/gdal/swig/python/samples/

 
 Usage: gdalsize.py -ng 8 infile outfile
   where -ng is number of gores in output file, based on "interrupted" sinusoidal projection. 