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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <math.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:
	void compute(); 	
	void newAprilTag(const tagsList &tags);
private:
  
  struct Tag
  {
    
    bool active = false;
    /* mutable */ QMutex m;
    QVec pose = QVec::zeros(3);
    InnerModel *inner;
    int id = 0;
    
    
    void init(InnerModel *innermodel)
    { 
      inner= innermodel;
    }
    
    void setActive(bool v)
    {
	QMutexLocker ml (&m);
	active = v;
    }
	  
   void copy (float x,float z, int id_)
    {
      QMutexLocker ml (&m);
      id = id_;
      pose = inner->transform("void",QVec::vec3(x,0,z),"base");
    }
    
    int getId()
    {
      return id;
    }
	    
   QVec getPose()
   {
    QMutexLocker ml (&m);
    return pose;
   }
   
 };
 
  Tag tag;
  InnerModel *inermodel;
  int current = 0;
  enum class State  {SEARCH, WAIT} ; 
	State state = State::SEARCH;
	
};

#endif

