gdalSize.py

 Purpose: Given a pair of bounding longs, lats, resolution(m), bittype (8,16,32), and number of bands
          the script will report uncompressed file size. Currently, the projection is from the input 
          image. This was written for MAP2 to approximate the output file size (w/o compression).

 Author: Trent Hare (USGS)
 Based on tolatlong by Andrey Kiselev, dron@remotesensing.org
 
 Usage: gdalsize.py minlong minlat maxlong maxlat res bitType bands infile
 