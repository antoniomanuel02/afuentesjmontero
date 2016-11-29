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
//#include <simplifypath/simplifypath.h>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	void gotoTarjet(RoboCompLaser::TLaserData &laser);
	bool obstacle(RoboCompLaser::TLaserData laser);
	void bug(RoboCompLaser::TLaserData &laser, TBaseState &bState);
	void init_bug(RoboCompLaser::TLaserData &laser, TBaseState &bState);
	bool targetAtSight(RoboCompLaser::TLaserData laser);
	float obstacleLeft(const TLaserData& tlaser);
	float distanceToLine(const TBaseState& bState);
	

public slots:
	
  void compute(); 	
	//QPolygon QPointF();
	//float k;


private:
	struct Tarjet
	{
	  bool active = false;
	  /* mutable */ QMutex m;
	  QVec pose = QVec::zeros(3);
	  
	  void setActive(bool v)
	  {
	    QMutexLocker ml (&m);
	    active = v;
	  }
	  bool isActive()
	  {
	    return active;
	  }
	  
	  void copy (float x, float z)
	  {
	    QMutexLocker ml (&m);
	    pose[0]=x;
	    pose[1]=0;
	    pose[2]=z;
	  }
	  
	  QVec getPose()
	  {
	    QMutexLocker ml (&m);
	    return pose;
	  }
	};
	
	Tarjet tarjet;
	InnerModel *inermodel;
	QLine2D linea;
	float distanciaAnterior;
	

	enum class State  { INIT, GOTO, INIT_BUG, BUG} ; 
	State state = State::INIT;
	
		
		 void go(const string &nodo, const float x, const float y, const float alpha);
		 void turn(const float speed);
		 bool atTarget();
		 void stop();
		
	};
	
#endif
