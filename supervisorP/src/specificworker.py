#
# Copyright (C) 2016 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice, traceback, time
import networkx as nx
import matplotlib.pyplot as plt
from PySide import *
from genericworker import *



ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'genericworker.py: ROBOCOMP environment variable not set! Exiting.'
	sys.exit()


preStr = "-I"+ROBOCOMP+"/interfaces/ --all "+ROBOCOMP+"/interfaces/"
Ice.loadSlice(preStr+"GotoPoint.ice")
from RoboCompGotoPoint import *
Ice.loadSlice(preStr+"AprilTags.ice")
from RoboCompAprilTags import *
Ice.loadSlice(preStr+"DifferentialRobot.ice")
from RoboCompDifferentialRobot import *



class SpecificWorker(GenericWorker):
	
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)
		self.rellenarGrafo()
	def setParams(self, params):
		#try:
		#	par = params["InnerModelPath"]
		#	innermodel_path=par.value
		#	innermodel = InnerModel(innermodel_path)
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"		
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		
		
		
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e
		return True     
	def rellenarGrafo(self):
	  pos={}
	  file = open('puntos.txt', 'r')
	  with file as f:
	     g = nx.Graph()
	     for line in f:
	       print line
	       l=line.split()
	       if l[0] == "N":
		 g.add_node(l[1], x=l[2], z=l[3], tipo= l[4])
		 pos[l[1]]=(float(l[2]), float(l[3]))
	       elif line[0] == "E":
		 g.add_edge(l[1],l[2])
		   
	  file.close()
          print pos
          img = plt.imread("plano.png")
	  plt.imshow(img,extent=([-12284, 25600, -3840, 9023]))
	  nx.draw_networkx(g, pos)
	  
	  print "Haciendo camino minimo"
	  print nx.shortest_path(g,source="1", target="6") 
	  
	  #nx.draw_networkx(g, posiciones)
	  plt.show()
	     
	def nodoCercano(self):
	  bState = RoboCompDifferentialRobot.Bstate()
	  differentialrobot_proxy.getBaseState(bState)
	  r = (bState.x , bState.z)
	  dist = lambda r,n: (r[0]-n[0])**2+(r[1]-n[1])**2
	  #funcion que devuele el nodo mas cercano al robot
	  return  sorted(list (( n[0] ,dist(n[1],r)) for n in posiciones.items() ), key=lambda s: s[1])[0][0]
	
	
	     #nx.shortestpath(g,source="3", target = "5")
	      #source es el punto del grafo mas cercano al robot
	      #nx.draw_circular(g,node_size=3000,node_color='b')
	      
