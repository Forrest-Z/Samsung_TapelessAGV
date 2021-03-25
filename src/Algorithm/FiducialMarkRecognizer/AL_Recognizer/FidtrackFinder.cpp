/*  reacTIVision fiducial tracking framework
FidtrackFinder.cpp
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
#include "FidtrackFinder.h"
#include <sstream>
FidtrackFinder::FidtrackFinder()
{
	dmap = new ShortPoint[IMG_WIDTH*IMG_HEIGHT];
	initialize_treeidmap( &treeidmap );
	initialize_fidtrackerX( &fidtrackerx, &treeidmap, dmap);
	initialize_segmenter( &segmenter, IMG_WIDTH, IMG_HEIGHT, treeidmap.max_adjacencies );

	average_leaf_size = 4.0f;
	average_fiducial_size = 48.0f;
	session_id = 0;

	reset();
};

FidtrackFinder::~FidtrackFinder() 
{
	terminate_segmenter(&segmenter);
	terminate_treeidmap(&treeidmap);
	terminate_fidtrackerX(&fidtrackerx);
	delete[] dmap;
};

void FidtrackFinder::reset() 
{
	fiducialList.clear();

	for (int y=0;y<IMG_HEIGHT;y++) {
		for (int x=0;x<IMG_WIDTH;x++) {
			dmap[y*IMG_WIDTH+x].x = x;
			dmap[y*IMG_WIDTH+x].y = y;
		}
	}
}
list <KuPose> FidtrackFinder::process(unsigned char *src, unsigned char *dest) 
{
	reset();
	KuPose FiducialLandMarkPos;
	list <KuPose> lFiducialLandMarkPos;
	// segmentation
	step_segmenter( &segmenter, dest );
	// fiducial recognition
	int fid_count = find_fiducialsX( fiducials, MAX_FIDUCIAL_COUNT,  &fidtrackerx, &segmenter, IMG_WIDTH,IMG_HEIGHT);

	float total_leaf_size = 0.0f;
	float total_fiducial_size = 0.0f;
	int valid_fiducial_count = 0;

	// print info
	if(fid_count > 0)
	{
		//printf("Number of fiducials: %d ( ID: ", fid_count);

		for(int k = 0; k < fid_count; k++)
		{
			//printf("%d ", fiducials[k].id);

			if(fiducials[k].id == -1)
			{
				//printf("######### ");
			}
			else
			{
				//printf("################### ");
			}
		}

		//printf(")\n");
	}

	// process found symbols
	for(int i=0;i< fid_count;i++) 
	{

		if ( fiducials[i].id >=0 ) {
			valid_fiducial_count ++;
			total_leaf_size += fiducials[i].leaf_size;
			total_fiducial_size += fiducials[i].root_size;
		}

		FiducialObject *existing_fiducial = NULL;
		// update objects we had in the last frame
		// also check if we have an ID/position conflict
		// or correct an INVALID_FIDUCIAL_ID if we had an ID in the last frame
		for (std::list<FiducialObject>::iterator fiducial = fiducialList.begin(); fiducial!=fiducialList.end(); fiducial++) 
		{

			float distance = fiducial->distance(fiducials[i].x,fiducials[i].y);

			if (fiducials[i].id==fiducial->fiducial_id)  
			{
				// find and match a fiducial we had last frame already ...
				if(!existing_fiducial)
				{
					existing_fiducial = &(*fiducial);

					for (int j=0;j<fid_count;j++) 
					{
						if ( (i!=j) && (fiducials[j].id==fiducial->fiducial_id) && (fiducial->distance(fiducials[j].x,fiducials[j].y)<distance)) {
							//check if there is another fiducial with the same id closer
							existing_fiducial = NULL;
							break;
						}
					}	


					if (existing_fiducial!=NULL) 
					{
						for (std::list<FiducialObject>::iterator test = fiducialList.begin(); test!=fiducialList.end(); test++) 
						{
							FiducialObject *test_fiducial = &(*test);
							if ( (test_fiducial!=existing_fiducial) && (test_fiducial->fiducial_id==existing_fiducial->fiducial_id) && (test_fiducial->distance(fiducials[i].x,fiducials[i].y)<distance)) 
							{
								//check if there is another fiducial with the same id closer
								existing_fiducial = NULL;
								break;
							}
						}
					}
				} /*else if (distance<existing_fiducial->distance(fiducials[i].x,fiducials[i].y)) {
				  existing_fiducial = &(*fiducial);
				  // is this still necessary?
				  } */

			} 
			else if ((distance<average_fiducial_size/1.2) && (abs(fiducials[i].node_count-fiducial->node_count)<=FUZZY_NODE_RANGE)) 
			{
				// do we have a different ID at the same place?
				// this should correct wrong or invalid fiducial IDs
				// assuming that between two frames
				// there can't be a rapid exchange of two symbols
				// at the same place

				for (int j=0;j<fid_count;j++)
				{
					if ( (i!=j) && (fiducial->distance(fiducials[j].x,fiducials[j].y)<distance)) 
						goto fiducialList_loop_end;
				}	

				if (fiducials[i].id==INVALID_FIDUCIAL_ID)
				{
					//printf("corrected invalid ID to %d (%ld)\n", fiducial->fiducial_id,fiducial->session_id);

					//two pixel threshold since missing/added leaf nodes result in a slightly different position
					float dx = abs(fiducial->getX() - fiducials[i].x);
					float dy = abs(fiducial->getY() - fiducials[i].y);
					if ((dx<2.0f) && (dy<2.0f)) 
					{
						fiducials[i].x = (short int)fiducial->getX();
						fiducials[i].y = (short int)fiducial->getY();
					}

					fiducials[i].angle=fiducial->getAngle();
					fiducials[i].id=fiducial->fiducial_id;
					fiducial->state = FIDUCIAL_INVALID;
				} 
				else /*if (fiducials[i].id!=fiducial->fiducial_id)*/ 
				{

					if (!fiducial->checkIdConflict(session_id,fiducials[i].id)) 
					{
						//printf("corrected wrong ID from %d to %d (%ld)\n", fiducials[i].id,fiducial->fiducial_id,fiducial->session_id);
						fiducials[i].id=fiducial->fiducial_id;
					} 
					else 
					{
						session_id++;
					}
				}

				existing_fiducial = &(*fiducial);
				break;
			}

fiducialList_loop_end:;
		}

		if  (existing_fiducial!=NULL) 
		{
			// just update the fiducial from last frame ...
			existing_fiducial->update(fiducials[i].x,fiducials[i].y,fiducials[i].angle,fiducials[i].root_size,fiducials[i].leaf_size);

		} 
		else if  (fiducials[i].id!=INVALID_FIDUCIAL_ID) 
		{
			// add the newly found object
			session_id++;
			FiducialObject addFiducial(session_id, fiducials[i].id, IMG_WIDTH,IMG_HEIGHT,fiducials[i].root_colour,fiducials[i].node_count);
			addFiducial.update(fiducials[i].x,fiducials[i].y,fiducials[i].angle,fiducials[i].root_size,fiducials[i].leaf_size);

			fiducialList.push_back(addFiducial);
		}
		//else drawObject(fiducials[i].id,(int)(fiducials[i].x),(int)(fiducials[i].y),display,0);
		//LDW add
		//exception of incorrected land mark
		if(fiducials[i].id != -1)
		{
			FiducialLandMarkPos.setX((int)(fiducials[i].x));
			FiducialLandMarkPos.setY((int)(fiducials[i].y));
			FiducialLandMarkPos.setThetaRad(fiducials[i].angle);
			FiducialLandMarkPos.setID(fiducials[i].id);
			lFiducialLandMarkPos.push_back(FiducialLandMarkPos);
		}

	}
	return lFiducialLandMarkPos;
}

