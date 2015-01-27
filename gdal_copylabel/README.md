gdal_copylabel.py

Description: Copies label's projection from one image to another (allows new pixel offset also)

Usage: gdal_copylabel.py [-of format] [-shiftX pixels] [-shiftY pixels (neg)] infile copyfile outfile

Example:
> gdal_copylabel.py -of vrt -shiftX 1 -shiftY -1 adir_DEM_1m_InSightE08_E_isis2_02_02_ang.cub DEM_1m_InSightE08_E_isis3.cub adir_DEM_1m_InSightE08_E_isis2_02_02_ang.vrt
   note: A GDAL VRT is a virtual format which point to the original "infile". To burn this label into a new image use gdal_translate
   > gdal_translate  adir_DEM_1m_InSightE08_E_isis2_02_02_ang.vrt adir_DEM_1m_InSightE08_E_isis2_02_02_ang.tif
   
   
   