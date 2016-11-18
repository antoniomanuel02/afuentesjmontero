/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
  
  
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{


	innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	tag.init(innermodel);
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
  
   try
   {
    RoboCompDifferentialRobot::TBaseState TBstate;
    differentialrobot_proxy->getBaseState(TBstate);
    innermodel->updateTransformValues("base", TBstate.x,0,TBstate.z,0,TBstate.alpha,0);
   }
   catch(const Ice::Exception &ex)
   {
        std::cout << ex << std::endl;
   }
 
  
    switch(state) {
    
      case State::SEARCH:
	qDebug()<< "Case SEARCH";
      
	if(tag.getId() == current){
	 
	    
	  differentialrobot_proxy->stopBase();
	  gotopoint_proxy->go("",tag.getPose().x(),tag.getPose().z(),0);
	  state = State::WAIT;
	}
	differentialrobot_proxy->setSpeedBase(0, 0.3);
	break;
      
      case State::WAIT:
	qDebug()<< "Case WAIT";
      
	if(gotopoint_proxy->atTarget() == true) {
	  differentialrobot_proxy->stopBase();
	  state = State::SEARCH;
	  current = current ++ % 4;
	}  
	if(tag.changed()) 
	{
	  gotopoint_proxy->go("",tag.getPose().x(),tag.getPose().z(),0);
	}
	break;
    }
}

void SpecificWorker::newAprilTag(const tagsList &tags) {
  
  tag.copy(tags.front().tx, tags.front().tz, tags.front().id);
  
}

  

  
// 	try
// 	{
// 		camera_proxy->getYImage(0,img, cState, bState);
// 		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
// 		searchTags(image_gray);
// 	}
// 	catch(const Ice::Exception &e)
// 	{
// 		std::cout << "Error reading from Camera" << e << std::endl;
// 	}





