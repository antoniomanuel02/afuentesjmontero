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

	
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
    const float threshold = 400; //millimeters
    float rot = 0.6;  //rads per second
    
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
	std::sort( ldata.begin() +5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
        //RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
       // std::sort( ldata.begin() +5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	TBaseState TBstate;
	QVec diferencia;
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
    */
    
    if (tarjet.active == true)
    {
     
      //En tarjet.QVec tenemos las coordenadas del punto a donde tiene que ir el robot
      //En TBstate tenemos la posici´on del robot
      differentialrobot_proxy->getBaseState(TBstate);
      
      diferencia = tarjet.getPose();
      std::cout << "diferencia x: " << diferencia.x() << std::endl;
      std::cout << "diferencia z: " << diferencia.z() << std::endl;
      diferencia[0] = diferencia.x() - TBstate.x; 
      diferencia[1] = diferencia.z() - TBstate.z;
      
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
      
      matrizWorld[0][0] = tarjet.pose[0];
      matrizWorld[1][0] = tarjet.pose[1];
      
      matrizTrans[0][0] = diferencia.x();
      matrizTrans[1][0] = diferencia.z();
      
      
      matrizResult[0][0] = ((matrizSenoidal[0][0]*matrizWorld[0][0]) + (matrizSenoidal[0][1]*matrizWorld[1][0])) + (matrizTrans[0][0]);
      matrizResult[1][0] = ((matrizSenoidal[1][0]*matrizWorld[0][0]) + (matrizSenoidal[1][1]*matrizWorld[1][0])) + (matrizTrans[1][0]);
      
      
      float angulo = atan2(matrizResult[0][0],matrizResult[1][0]);
     // std::cout << "angulo: " << angulo << std::endl;
     
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
	if (fabs(modulo_diferencia) > 0.00001)
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
    
    
    
    
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}







