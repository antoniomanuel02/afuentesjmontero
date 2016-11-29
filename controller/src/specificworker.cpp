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

  state = State::INIT;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
void SpecificWorker::setPick(const Pick &myPick) {//se ejecuta en el hilo de middelware
 //se ejecuta en el hilo del middleware
  
  tarjet.copy(myPick.x,myPick.z);
  tarjet.setActive(true);
  
  qDebug() << myPick.x << myPick.z; 
  
  
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	

	inermodel = new InnerModel("/home/jorge/robocomp/files/innermodel/simpleworld.xml");
	//inermodel = new InnerModel("/home/jorge/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{

    try
    {
	//RoboCompDifferentialRobot::TBaseState TBstate;
	TBaseState TBstate;
	differentialrobot_proxy->getBaseState(TBstate);
	//QVec vector3 = QVec::zeros(3);
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
	inermodel->updateTransformValues("base", TBstate.x,0,TBstate.z,0,TBstate.alpha,0);
	QVec ini;
		
	switch(state) {
	  
	  case State::INIT:
	      qDebug()<< "Case init";
	      if(tarjet.isActive()){
		  qDebug()<< "Estado inicializado, cambia a GOTO";
		  ini = QVec::vec3(TBstate.x, 0, TBstate.z);
		  linea = QLine2D( ini, tarjet.getPose() );
		  state = State::GOTO;
	      }
	    break;
	  case State::GOTO:
	      qDebug()<< "Case Goto";
	      gotoTarjet(ldata);
	    break;
	  case State::INIT_BUG:
	    qDebug()<< "Case Init_Bug";
	    init_bug(ldata, TBstate);
	    
	    
	    break;   
	  case State::BUG:
	    qDebug()<< "Case Bug";
	    bug(ldata, TBstate);
	    break;
	}

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
    
}

void SpecificWorker::init_bug(RoboCompLaser::TLaserData &laser, TBaseState &bState)//solo gira para un lado,dar algo de avance en el giro 
{  
  float rot = 0.5;
  QVec posi = QVec::vec3(bState.x, 0., bState.z);
  distanciaAnterior = fabs(linea.perpendicularDistanceToPoint(posi));
  
  if(!obstacle(laser))
  {
    qDebug()<< "Cambia a BUG";
    state = State::BUG; 
    return;
  }
  try
  {
    differentialrobot_proxy->setSpeedBase(40, rot);
  }
  catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
       
}

void SpecificWorker::bug(RoboCompLaser::TLaserData &laser, TBaseState &bState) //no dar avance en el giro dar en el else,volver al bug init comprobando con el obstacle
{
  
  const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte//////////////
  float dist = obstacleLeft(laser);////////////////////
  float diffToline = distanceToLine(bState);////////////////
  

  if(targetAtSight(laser)) {
    qDebug()<< "Cambia a GOTO";
    state = State::GOTO;
    return;
  }
  
  //Check if close to the line and distance is reducing
  if (distanciaAnterior < 100 and diffToline < 0)
  {
    state = State::GOTO;
    qDebug() << "Crossing the line: from BUG to GOTO";
    return;
  }
  
  if(obstacle(laser))
  {
    qDebug()<< "Cambia a INIT_BUG";
    state = State::INIT_BUG; 
    return;
  } 
  
  float k=0.1;  // pendiente de la sigmoide
  float vrot =  -((1./(1. + exp(-k*(dist - 450.))))-1./2.);		//sigmoide para meter vrot entre -0.5 y 0.5. La k ajusta la pendiente.
  float vadv = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); 		//gaussiana para amortiguar la vel. de avance en funcion de vrot
  qDebug() << vrot << vadv;
  //vrot *= 0.3;
  differentialrobot_proxy->setSpeedBase ( vadv ,vrot );
  
}


bool SpecificWorker::obstacle(RoboCompLaser::TLaserData laser) {
  float threshold = 350;
  qDebug()<< "----------------------------------------";
  RoboCompLaser::TLaserData ldata = laser;  //read laser data 
  std::sort( ldata.begin() +5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
  if( ldata[5].dist  < threshold)//ldata.front().dist < threshold)
    {
      return true;
    }
    else 
    {
      return false;
    }
      
  
}
bool SpecificWorker::targetAtSight(RoboCompLaser::TLaserData laser)
{
  QPolygon poly;
	for ( auto l: laser )
	{
		QVec r = inermodel->laserTo ( "world","laser",l.dist,l.angle );
		QPoint p ( r.x(),r.z() );
		poly << p;
	}
	QVec targetInRobot = inermodel->transform("base", tarjet.getPose(), "world");
	float dist = targetInRobot.norm2();
	int veces = int(dist / 200);  //number of times the robot semilength fits in the robot-to-target distance
	float landa = 1./veces;
	
	QList<QPoint> points;
	points << QPoint(tarjet.getPose().x(),tarjet.getPose().z());  //Add target
	
	//Add points along lateral lines of robot
	for (float i=landa; i<= 1.; i+=landa)
	{
		QVec point = targetInRobot*(T)landa;
		QVec pointW = inermodel->transform("world", point ,"base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = inermodel->transform("world", point - QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
		pointW = inermodel->transform("world", point + QVec::vec3(200,0,0), "base");
		points << QPoint(pointW.x(), pointW.z());
		
	}
	foreach( QPoint p, points)
	{
		if( poly.containsPoint(p , Qt::OddEvenFill) == false)
			return false;
	}
	return true;

}

void SpecificWorker::gotoTarjet(RoboCompLaser::TLaserData &laser)
{

  qDebug() << "Entra en gotoTarjet";
  
  QVec rt = inermodel->transform("base", tarjet.getPose(), "world");
  
  float ang = atan2(rt.x(),rt.z());
  float dist = rt.norm2();
  
  if( dist <= 100)//ha llegado al destino
  {
    tarjet.setActive(false);
    qDebug()<< "Cambia a INIT";
    state = State::INIT;
    differentialrobot_proxy->stopBase();
    return;
    //qDebug()<< "Ha llegado a su destino";  
  }
  if(obstacle(laser))
  {
    qDebug()<< "Cambia a INIT_BUG";
    state = State::INIT_BUG;
    return;
  }
  
    
    if( fabs(ang) > 0.05 )
      dist = 0;
    if(dist > 300) 
      dist = 300;
    
  try
  {
      differentialrobot_proxy->setSpeedBase(dist, ang);  
  }
  catch (const Ice::Exception &ex) {std::cout << ex << std::endl;}

}

float SpecificWorker::obstacleLeft(const TLaserData& tlaser)
{
	const int laserpos = 85;
	float min = tlaser[laserpos].dist;
	for(int i=laserpos-2; i<laserpos+2;i++)
	{
		if (tlaser[i].dist < min)
			min = tlaser[i].dist;
	}
	return min;
}

float SpecificWorker::distanceToLine(const TBaseState& bState)
{
	QVec posi = QVec::vec3(bState.x, 0., bState.z);
	float distanciaEnPunto = fabs(linea.perpendicularDistanceToPoint(posi));
	float diff = distanciaEnPunto - distanciaAnterior;
	distanciaAnterior = distanciaEnPunto;
	return diff;
} 

void SpecificWorker::go(const string &nodo, const float x, const float y, const float alpha) {

  qDebug() <<x << y; 
  tarjet.copy(x,y);
  tarjet.setActive(true);
  state = State::INIT;
  
  
}
bool SpecificWorker::atTarget()
{
  
  return !tarjet.isActive();
   
}

void SpecificWorker::turn(const float speed)
{
    differentialrobot_proxy->setSpeedBase(0, speed);
}
void SpecificWorker::stop() {
  
  differentialrobot_proxy->stopBase();
}