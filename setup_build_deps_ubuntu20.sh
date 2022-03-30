#! /bin/bash
# setup_build_deps_ubuntu.sh
#
#  Simple script to setup build deps under ubuntu.
#  Not actually tested yet
#
#  Created on: March 25, 2022
#      Author: amyznikov
#

topdir=..

apt-get install -y \
	git \
	cmake \
	autoconf \
	automake \
	libtool \
	qt5-default \
	libtinyxml2-dev \
	libbtbb-dev \
	libpcap-dev \
	libtiff-dev \
	libopencv-dev \
	python3-opencv

(cd ${topdir} && \
	git clone -b v1.7.2 https://github.com/hyperrealm/libconfig.git && \
	cd libconfig && \
	autoreconf && \
	./configure && \
	make -j4 install && \
	make clean)

(cd ${topdir} && \
	wget http://www.libqglviewer.com/src/libQGLViewer-2.7.1.tar.gz && \
	tar -xzvf libQGLViewer-2.7.1.tar.gz && \
	cd libQGLViewer-2.7.1/QGLViewer && \
	qmake && \
	make && \
	sudo make install && \
	make clean)
