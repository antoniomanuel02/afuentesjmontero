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
  tarjet.active = true;
  
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
    float aux_modulo = 0;
    try
    {
        
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data 
        std::sort( ldata.begin() +5, ldata.end()-5, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	TBaseState TBstate;
	QVec diferencia;
	differentialrobot_proxy->getBaseState(TBstate);
        
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
      
      diferencia = tarjet.getPose();
      diferencia[0] = diferencia.x() - TBstate.x; 
      diferencia[1] = diferencia.z() - TBstate.z;
      
      //sacamos el modulo del vector
      aux_modulo = (diferencia.x() * diferencia.x()) + (diferencia.z() * diferencia.z());
      aux_modulo = sqrt(aux_modulo);
      
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







