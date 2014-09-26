PROJECT NAME
  Atlas Crater

LROC NAC SOURCE IMAGES
  M170266335 L/R
  M170259544 L/R

TECHNICAL CONTACT INFORMATION
  NAME		ARIZONA STATE UNIVERSITY
  INSTITUTION	ARIZONA STATE UNIVERSITY

DATE CREATED
  January 30, 2012

- Products are tied to LOLA elevations with RMS error of 3.27m to 3 different LOLA orbit tracks. The RMS error
  is the root-mean-square of the elevation difference between the LOLA points and the DTM. This value is a
  combination of error from the LOLA points and the DTM, and may give a measure of the horizontal and vertical
  accuracy of the DTM.
- The reported precision error from SOCET SET is 1.19 m.
- SOCET SET NGATE v5.5 was used to extract terrain.
- Post spacing in DTM is 2.0 m.



GENERAL NOTES:

KNOWN ARTIFACTS
  These are known artifacts present in some, but not all, LROC NAC DTMs.
  These artifacts may not be present in this DTM! Look at the terrain shaded relief and the confidence map
  to detect these before using the DTM!
  - Boxes
	Some DTMs have rectangular areas that are usually about 0.5-1.0 m different in elevation from the
	surrounding areas. These are artifacts of the processing algorithms used in SOCET SET ((c) BAE Systems).
 	There may be groups of these boxes. They are almost impossible to edit out, so the user should look
 	for such artifacts in a terrain shaded relief map before using the DTM for analysis.
  - NACL and NACR seams
	There are two NAC cameras (NACL & NACR) on board LRO. Stereo images are collected by acquiring images
	on two different orbits so the total parallax angle is greater than 12 degrees. The overlap between
	the four images (two pairs of NACL and NACR) provides three or four stereo models from which to collect
	elevation data. These stereo models are stitched together into one DTM. In some DTMs, the seams between
	the stereo models may be visible as long lines, usually up to 1 m difference. These seams are difficult
	to remove from the whole DTM.
  - Faceted areas
 	Areas that were very bland (low contrast) or deeply shadowed with low contrast and low signal may have
 	a "faceted" look to them. This is due to the processing algorithm not able to correlate the images and
	must interpolate to fill in the pixel. The terrain data in these areas are not trustworthy. Use the
	confidence map to look for "suspicious" data points.
  - Noise
	DTMs have varying level of noise from low signal images, low incidence angle, and jitter. The amount
	of noise may be detected using the slope map or the confidence map.

PDS PRODUCT NAMING CONVENTION
  The PRODUCT_ID naming convention for LROC NAC DTMs is: NAC_DTM_xxxxxxxxxxx_AyyyBzzzz.IMG where
	- NAC_DTM = indicating it's a NAC DTM product
	- xxxxxxxxxx = words specifying the sitename (up to 11 characters)
	- A = projection type (E for equirectangular, P for polar stereographic)
	- yyy = center latitude (the center latitude is multiplied by 10 to retain the geographic position
		to the nearest 10th of a degree, then rounded to the nearest integer).
	- B = N for northern hemisphere, S for southern hemisphere
	- zzzz = center longitude (the actual center longitude is multiplied by 10 to retain the geographic
		position to the nearest 10th of a degree, then rounded to the nearest integer). The longitude
		domain for all NAC DTMs is 360 positive east.
  Example: 	NAC_DTM_LINNECRATER_E280N0120.IMG is a NAC DTM of Linne Crater located at latitude=28.0N,
		longitude=12.0E in equirectangular projection.

RELATED DTM PRODUCTS
  For each NAC DTM, we also produce several related products. The naming convention for these products is:
  <basename>_<description> where <basename> refers to NAC_DTM_xxxxxxxxxxx and <description> refers to the
  type of product (see details below).
  - Ortho-rectified images
	We provide 4 ortho-rectified images, a mosaic of the NACL_1 and NACR_1 and a mosaic of NACL_2 and NACR_2
	in two resolutions (DTM resolution and image resolution). The <description> part of the filename is the
	name of the NAC image and the resolution.
  - Colorshade and colorshade legend
	The colorshade image is the color-coded topography draped over the shaded relief. The colorshade legend
	explains what the color means. The <description> for the colorshade image is CLRSHADE and the colorshade
	legend is CLRSHADE_LEGEND.
  - Color slope map and slope map legend
	The color slope map is the color-coded slope (in discrete increments) draped over the shaded relief. The
	slope map legend explains what the color means. The <description> for the color slope map is SLOPE and
	the slope map legend is SLOPE_LEGEND.
  - Shaded Relief
	The shaded relief image is generated with the sun azimuth of 315 degrees (northwest) and sun altitude of
	45 degrees. The <description> for the shaded relief image is SHADE
  - Confidence map
	The confidence map gives users a little guidance on what elevation values to trust and not to trust.
	Refer to the CONFIDENCE MAP TABLE LOOKUP below for the description of the pixel values. The <description>
	for the confidence map is CONF.
  - DTM in Geotiff format
	We also provide the DTM in GeoTIFF format to allow users more flexibility in using the DTM. There are no
	<description> for this file, and the filename is simply <basename>.tif
  - Readme
	The readme file contains description of the DTM, precision and accuracy information, description of known
	artifacts, naming convention, related products, and confidence table lookup. The <description> for the
	readme file is README.

CONFIDENCE MAP TABLE LOOKUP
  Value	 Description
    0	 NoDATA, outside boundary (e.g. out of stereo pair overlap)
    1	 Shadowed
    2	 Saturated
    3	 Suspicious (edge, corner, did not correlate, etc.)
    4	 Interpolated / Extrapolated (e.g. from neighbor pixels)
  10-14	 Value range of successful correlations
    15	 Manually edited

