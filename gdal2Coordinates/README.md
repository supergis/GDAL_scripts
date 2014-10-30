Simple GDAL coordinate to coordinate tranlsations given an image
============

Usage: longlat2meters.py long lat inImage
   * Given a long,lat report X,Y in meter (or feet) coordinates based on the projection from input image.
   
Usage: meters2longlat.py X Y inImage
   * Given X,Y in meter (or feet) report long,lat coordinates based on the projection from input image.
    
Usage: tolatlong.py sample line inImage
   * Given sample, line report latitude/longitude coordinates for the center of the specified pixel for the input image.
   
Usage: pixel2meters.py sample line inImage
   * Given sample, line report X,Y in meters (or feet) coordinates for the center of the specified pixel for the input image