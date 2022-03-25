# qlidarview

Simple LiDAR .pcap file viewer based on libQGLViewer.


This is experimental code for parsing and visualization of LiDAR .pcap files.

It visualizes 3D lidar point cloud and some range images (distances, depths, heights, etc...). 

No any other functionality is implemented.

Formally supported LiDARs at moment are VLP16, VLP32C, HDL32E, HDL64 and VLS128,
but no extensive testing was made yet because I still waiting for actual datasets 
appropriate for testings purposes.

The default calibration parameters are collected from LiDARs user manuals and VeloView .xml files.

The datalink types curretly supported are only DLT_NULL=0 (BSD loopback encapsulation) 
and DLT_EN10MB=1 (Ethernet (10Mb))


The build depencencies are 

	tinyxml2
	tbb 
	libtiff 
	opencv
	glu 
	freeglut 
	glew 
	qt5-declarative 
	qt5-imageformats 
	qt5-multimedia 
	qt5-graphicaleffects

	libqglviewer (http://libqglviewer.com)


# Screenshot

![Screenshot.png](./Screenshot.png)
