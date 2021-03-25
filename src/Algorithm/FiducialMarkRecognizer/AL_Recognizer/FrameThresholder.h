/*  reacTIVision fiducial tracking framework
FrameThresholder.h
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

#ifndef FRAMETHRESHOLDER_H
#define FRAMETHRESHOLDER_H

#include "tiled_bernsen_threshold.h"
#include "threshold.h"

class FrameThresholder
{
	static const int IMG_WIDTH = 640;//320; // 영상 가로 크기
	static const int IMG_HEIGHT = 480;//240; // 영상 세로 크기
public:	
	FrameThresholder(void);
	~FrameThresholder(void);
	void process(unsigned char *src, unsigned char *dest);


private:
	TiledBernsenThresholder *m_thresholder;
};

#endif
