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


	inermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
  
   try
   {
    TBaseState TBstate;
    differentialrobot_proxy->getBaseState(TBstate);
    QVec vector3 = QVec::zeros(3);
 
  
    switch(state) {
    
      case State::SEARCH:
	qDebug()<< "Case SEARCH";
      
	if(tag.getId() == current){
	 
	    
	  differentialrobot_proxy->stopBase();
	 // gotopoint_proxy->go();
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
	break;
    }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
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
}
void SpecificWorker::newAprilTag(const tagsList &tags) {
  
  
  
}







