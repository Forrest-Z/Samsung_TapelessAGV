/*  reacTIVision fiducial tracking framework
FidtrackFinder.h
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

#ifndef FIDTRACKFINDER
#define FIDTRACKFINDER

#include <list>
#include <vector>
#include <list>
#include <sstream>
#include <string>
//
#include "FiducialObject.h"
#include "floatpoint.h"
#include "segment.h"
#include "fidtrackX.h"
#include "../../../KUNSPose/KuPose.h"


#define MAX_FIDUCIAL_COUNT 512
#define FINGER_ID -10

class FidtrackFinder
{
	static const int IMG_WIDTH = 640;//320; // 영상 가로 크기
	static const int IMG_HEIGHT = 480;//240; // 영상 세로 크기
public:	
	FidtrackFinder();
	~FidtrackFinder();

	list<KuPose> process(unsigned char *src, unsigned char *dest);
	void reset();

private:
	Segmenter segmenter;
	ShortPoint* dmap;	
	FiducialX fiducials[ MAX_FIDUCIAL_COUNT ];
	RegionX regions[ MAX_FIDUCIAL_COUNT*4 ];
	TreeIdMap treeidmap;
	FidtrackerX fidtrackerx;


	std::list<FiducialObject> fiducialList;

	float average_leaf_size;
	float average_fiducial_size;
	int session_id;

	void sendCursorMessages();

};

#endif
