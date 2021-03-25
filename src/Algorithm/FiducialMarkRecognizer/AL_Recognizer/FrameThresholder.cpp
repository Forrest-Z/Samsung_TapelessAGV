/*  reacTIVision fiducial tracking framework
FrameThresholder.cpp
Copyright (C) 2005-2008 Martin Kaltenbrunner <mkalten@iua.upf.edu>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <stdafx.h>
#include "FrameThresholder.h"
FrameThresholder::FrameThresholder(void)
{ 
	m_thresholder = new TiledBernsenThresholder();
	initialize_tiled_bernsen_thresholder(m_thresholder, IMG_WIDTH, IMG_HEIGHT , 20 ); 
	//Tile Size: 12(원본 파일)

};
FrameThresholder::~FrameThresholder(void) 
{
	terminate_tiled_bernsen_thresholder( m_thresholder );
	delete m_thresholder;
}
void FrameThresholder::process(unsigned char *src, unsigned char *dest) {
	tiled_bernsen_threshold( m_thresholder, dest, src, 1, IMG_WIDTH,  IMG_HEIGHT, 20, 0 );
	//simple_threshold( src, srcBytes,dest,width, height,128 );
	//simple_adaptive_threshold( src, srcBytes,dest,width, height, 16 );
	//overlapped_adaptive_threshold( src, srcBytes,dest,width, height, 8 );
	//overlapped_adaptive_threshold2( src, srcBytes,dest,width, height, 32, gradient );
}
