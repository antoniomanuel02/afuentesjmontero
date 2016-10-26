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
void SpecificWorker::setPick(const Pick &myPick) {//se ejecuta en el hilo de middelware
 //se ejecuta en el hilo del middleware
  
  tarjet.copy(myPick.x,myPick.z);
  tarjet.setActive(true);
  
  qDebug() << myPick.x << myPick.z; 
  
  
}
bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	

	//inermodel = new InnerModel("/home/jorge/robocomp/files/innermodel/simpleworld.xml");
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
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
	inermodel->updateTransformValues("base", TBstate.x,0,TBstate.z,0,TBstate.alpha,0);
		
	switch(state) {
	  
	  case State::INIT:
	    
	    if(tarjet.active == true)
	        state = State::GOTO;
	    break;
	  case State::GOTO:
	    
	    gotoTarjet();
	    break;
	  case State::BUG:
	    
	    bug();
	    break;   
	  case State::END:
	    
	    break;
	}

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
    
}

void SpecificWorker::bug() {
  
  float vr;
  RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
  float d = ldata[10].dist;
  const float m = 1.f/ 1000.f;
  const float n = -0.5;
  float k;
  float rot = 0.5;
  TBaseState TBstate;
  
  if(cruzarLinea()) {
    
    state = State::GOTO;
    return;
  }
  
   k = m* d +n;  
  if(d > 160) 
   
    vr = d*k;
  
  if(d < 130) 
    vr = -d*k;
  float alfa = log(0.1)/log(0.2);
  float vadv = exp((fabs(vr)*alfa)*250);
  differentialrobot_proxy->setSpeedBase(vadv, rot);
  
  
}
bool SpecificWorker::cruzarLinea() {
  return true;
}

bool SpecificWorker::obstacle() {
  float threshold = 100;
  RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
  if( ldata[5].dist  < threshold)//ldata.front().dist < threshold)
    {
      return true;
    }
    else 
    {
      return false;
    }
      
  
}
bool SpecificWorker::targetAtSight()

{
  
  QPolygonf polygon;
  RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
  for (auto l;; ldata)  
  {
    QVec lr = inermodel->laserTo("world", "laser", l.dist, l.angle);
    polygon << QPointF(lr.x(), lr.z());
  }
  
  QVec t = tarjet.getPose();
  return  polygon.contains(QPointF(t.x(), t.z()));

}

void SpecificWorker::gotoTarjet()
{

  qDebug() << "gotoTarjet";
  
  QVec rt = inermodel->transform("base", tarjet.getPose(), "world");
  
  float ang = atan2(rt.x(),rt.z());
  float dist = rt.norm2();
  
  if( dist < 100)
  {
    differentialrobot_proxy->stopBase();
    qDebug()<< "Ha llegado a su destino";
    
  }else if(obstacle())
  {
    state = State::BUG;
  }
  else
  {
    float adv = dist;
    float rot = ang;
    if( fabs(ang) > 0.05 )

      adv = 0;

    differentialrobot_proxy->setSpeedBase(adv, rot);  
  }
  
  
  /*
   RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
	std::sort( ldata.begin() +5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
        //RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
       // std::sort( ldata.begin() +5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	TBaseState TBstate;
	QVec diferencia = QVec::zeros(2);
	float modulo_diferencia;
	
        
    /* Para que no se coma las cajas   
    if( ldata[5].dist  < threshold)//ldata.front().dist < threshold)
    {
      if(ldata[5].angle > 0) 
      {
	//if(ldata[6].angle < 45){
	  std::cout << ldata.front().dist << std::endl;
	  differentialrobot_proxy->setSpeedBase(5, -rot);
	 // usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
	  usleep(30000); 
	//}
	
      }else {
	//if(ldata[6].angle > -46){
	  std::cout << ldata.front().dist << std::endl;
	  differentialrobot_proxy->setSpeedBase(5, rot);
	 // usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1se
	  usleep(30000); 
	//}
	  
      }
      
    }
    else
    {
        differentialrobot_proxy->setSpeedBase(600, 0); 
    }
    /////////////////////////////////////////////////////
    
    if (tarjet.active == true)
    {
     
      //En tarjet.QVec tenemos las coordenadas del punto a donde tiene que ir el robot
      //En TBstate tenemos la posici´on del robot
      // im -> trans("base",qvec,"world")
      
      differentialrobot_proxy->getBaseState(TBstate);
      
      QVec target = tarjet.getPose();
  
      diferencia[0] = target[0] - TBstate.x; 
      diferencia[1] = target[1] - TBstate.z;
      
      target.print("target");
      qDebug() << TBstate.x << TBstate.z;
      diferencia.print("dif");
      
      
      
      
      std::cout << "diferencia x: " << diferencia[0] << std::endl;
      std::cout << "diferencia z: " << diferencia[1] << std::endl;
      
      //sacamos el modulo del vector
      modulo_diferencia = (diferencia.x() * diferencia.x()) + (diferencia.z() * diferencia.z());
      modulo_diferencia = sqrt(modulo_diferencia);
      
      float matrizSenoidal[2][2];
      float matrizWorld[2][1];
      float matrizTrans[2][1];
      float matrizResult[2][1];
      
      matrizSenoidal[0][0] = cos (TBstate.alpha);
      matrizSenoidal[0][1] = -sin (TBstate.alpha);
      matrizSenoidal[1][0] = sin (TBstate.alpha);
      matrizSenoidal[1][1] = cos (TBstate.alpha);
      
      matrizWorld[0][0] = target[0];
      matrizWorld[1][0] = target[1];
      
      matrizTrans[0][0] = diferencia[0];
      matrizTrans[1][0] = diferencia[1];
      
      
      matrizResult[0][0] = ((matrizSenoidal[0][0]*matrizWorld[0][0]) + (matrizSenoidal[0][1]*matrizWorld[1][0])) + (matrizTrans[0][0]);
      matrizResult[1][0] = ((matrizSenoidal[1][0]*matrizWorld[0][0]) + (matrizSenoidal[1][1]*matrizWorld[1][0])) + (matrizTrans[1][0]);
      
    
      float angulo = atan2(matrizResult[0][0],matrizResult[1][0]);
      std::cout << "angulo: " << angulo << std::endl;
     
     // differentialrobot_proxy->setSpeedBase(0, 0.5);
      
      
      
      if (fabs(angulo) >= 0.05) //valor absuluto de angulo;
      {	
	  if(angulo>0)
	  {
	    differentialrobot_proxy->setSpeedBase(0, 0.5);
	  }
	  
	  if(angulo<0)
	  {
	    differentialrobot_proxy->setSpeedBase(0, -0.5);
	  }
      }
      
      else if (fabs(angulo) < 0.05) //valor absuluto de angulo;
      {
	//std::cout << "Tira para alante con modulo: " << modulo_diferencia << std::endl;
	if (fabs(modulo_diferencia) > 100)
	{
	  differentialrobot_proxy->setSpeedBase(300, 0);
	}
	else
	  differentialrobot_proxy->setSpeedBase(0, 0);
      }
      
      
      
      
      
      //ahora necesitamos el angulo del vector diferencia para que el robot apunte 
      //a la zona donde debe ir
      
      
      //necesito ang y mod
      
    }
    */

}