Usage: gdal2AsciiLatLonBands.py [-srcwin xoff yoff width height] [-band 1] [-band 2] [-band n] [-addheader] [-printLatLon] [-printYX] srcfile [dstfile]

* brackets [ ] indicate optional parameter. If no output file, will write to stdout
* defaults to band 1 if nothing is sent
* sent band order will be pushed in output ascii file.
* -srcwin offsets, width, and height values should be sent in meters
* -addheader will add a one row with field names (although bands are just numbered).

Use one or none for
 * -printLatLon will use GDAL/map projection to calculate Lat/Lon for every pixel
 * -printYX will write out Y,X in meters for every pixel 

Here are some use cases:

% gdal2AsciiLatLonBands.py -addheader -printYX  input.cub
* creates "Y, X, Band1" to stdout with header line

% gdal2AsciiLatLonBands.py -addheader -printYX  input.cub out.csv
* creates "Y, X, Band1" to out.csv with header line

% gdal2AsciiLatLonBands.py input.cub
* creates "Band1" listing to stdout

% gdal2AsciiLatLonBands.py  -addheader -printLatLon  input.cub out.csv
* creates "Lat, Lon, Band1" to out.csv with header line

% gdal2AsciiLatLonBands.py -addheader -printYX -band 1 -band 2 -band 3 input.cub out.csv
* creates "Y, X, Band1, Band2, Band3" to out.csv with header line

% gdal2AsciiLatLonBands.py -addheader -band 4 -band 1 input.cub out.xyz
* creates "Y, X, Band4, Band1" to out.csv with header line
