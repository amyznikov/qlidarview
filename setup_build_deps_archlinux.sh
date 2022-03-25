#! /bin/bash
# setup_build_deps_archlinux.sh
#
#  Simple script to setup build deps under archlinux
#
#  Created on: March 25, 2022
#      Author: amyznikov
#

pacman -S \
	cmake \
	extra-cmake-modules \
	tinyxml2 \
	tbb \
	libtiff \
	opencv \
	glu \
	freeglut \
	glew \
	qt5-declarative \
	qt5-imageformats \
	qt5-multimedia \
	qt5-graphicaleffects

pikaur -S libqglviewer
