/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 */

#include <iostream>
#include <sstream>
#include "Controller.h"

using namespace std;
using namespace ns3;

namespace ns3
{
  Controller::Controller()
  { 
	  T=-1.0; 
	  Plot=false;
  }
  Controller::Controller(Ptr<Highway> highway)
  {
    this->highway=highway;
  }

  void Controller::SetHighway(Ptr<Highway> highway)
  {
    this->highway=highway;
  }
  
  Ptr<Highway> Controller::GetHighway()
  {
    return this->highway;
  }
  
  bool Controller::InitVehicle(Ptr<Highway> highway, int& VID)
  {
  
      srand(time(NULL));
      Ptr<Obstacle> ap;
      int apDist = highway->GetApInitDist();
      
      //Adicionado por Thales (simula a posY do Ap no canteiro central)
      int lane = highway->GetNumberOfLanes();

      // Adicionado por Thales (criação de Aps no canteiro central)
      for(int i=VID; i <=highway->GetApQtd(); i++)
      {        
        ap = CreateObject<Obstacle>();
        // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
        //ap->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
        ap->SetVehicleId(VID++);
        ap->SetPosition(Vector(apDist, highway->GetYForLane(lane,1), 0));
        ap->SetLength(1);
        ap->SetWidth(1);
        // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
        //ap->SetReceiveCallback(highway->GetReceiveDataCallback());
        ap->SetDirection(1);	// Está correto
        //obstacle->SetLane(3); Não preciso mais setar a pista, pois o array de Aps é único (não se guia por pista)
                        
        // Adicionado por Thales (atributo criado em Vehicle para setar se um veículo é Ap.
        // ... Serve para controlar a inserção de Aps em seu array específico). 
        ap->SetIsAp(true);     
        
        // Adicionado por Thales (inicializa TCT no momento de criação dos Aps com todos os trechos)
        ap->SetTable(((highway->GetApQtd()-1)*2));
        // Fim da adição
                
        highway->AddVehicle(ap);
        // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
        //Simulator::Schedule(Seconds(0.0), &Controller::BroadcastWarning, this, ap);
        apDist += highway->GetApDist();
      }   
      
      Ptr<Obstacle> obstacle;
      int obDist = highway->GetObInitDist();
      bool asObstacle = highway->GetAsObstacle();
      
      /*Adicionado por Thales (se obstacle == true, a criação de obstáculos começa do ID do último AP...
        ... até a quantidade total de obstáculos na via.*/
      int UpBound = (highway->GetApQtd() + highway->GetObQtd());
      
      // Adicionado por Thales (criação de obstáculos, caso obstacle == true)
      if (asObstacle){
		  for(int i=VID; i <= UpBound; i++)
		  {
			if (highway->GetObAleat())
			  lane = rand() % highway->GetNumberOfLanes();
			else
			  lane = 0;
			  
			obstacle =CreateObject<Obstacle>();
			// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
			//obstacle->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
			obstacle->SetVehicleId(VID++);
			/* controle: se não tem criação de obstáculos dinâmicos, cria-os dentro da via nas posições pré-definidas.
			 * Caso tenha, cria-os fora da via.*/
			if(highway->GetObDinamico() == false)
				obstacle->SetPosition(Vector(obDist, highway->GetYForLane(lane,1), 0));
			else
				obstacle->SetPosition(Vector(highway->GetHighwayLength()+50, highway->GetYForLane(lane,1), 0));
			// fim
			obstacle->SetLength(1);
			obstacle->SetWidth(1);
			// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
			//obstacle->SetReceiveCallback(highway->GetReceiveDataCallback());
			obstacle->SetDirection(1);
			obstacle->SetLane(lane);
			highway->AddVehicle(obstacle);
			// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
			//Simulator::Schedule(Seconds(0.0), &Controller::BroadcastWarning, this, obstacle);
			obDist += highway->GetObDist();
		  }		  
		  
		  // Adicionado por Thales (se via bi-direcional, cria obstáculos na outra direção também)
		  if (highway->GetTwoDirectional()){
			  
			  // Cria a mesma "obQtd" obstáculos na direção -1, além da quantidade "obQtd" já colocada na direção 1
			  UpBound += highway->GetObQtd();
			  // Na direção -1, a distância do 1° obstáculo é "o tamanho total da via - a distância inicial do 1° obstáculo"
			  int obDist = (highway->GetHighwayLength() - highway->GetObInitDist());
			  		  
			  for(int i=VID; i <= UpBound; i++)
			  {
				// Comentado para forçar obstáculos aleatórios na direção -1
				if (highway->GetObAleat())
				  lane = rand() % highway->GetNumberOfLanes();
				else
				  lane = 0;
				  
				obstacle =CreateObject<Obstacle>();
				// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
				//obstacle->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
				obstacle->SetVehicleId(VID++);
				// controle: idem acima.
				if(highway->GetObDinamico() == false)
					obstacle->SetPosition(Vector(obDist, highway->GetYForLane(lane,-1), 0));
				else
					obstacle->SetPosition(Vector(-50, highway->GetYForLane(lane,-1), 0));
				// fim
				obstacle->SetLength(1);
				obstacle->SetWidth(1);
				// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
				//obstacle->SetReceiveCallback(highway->GetReceiveDataCallback());
				obstacle->SetDirection(-1);	
				obstacle->SetLane(lane);
				highway->AddVehicle(obstacle);
				// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
				//Simulator::Schedule(Seconds(0.0), &Controller::BroadcastWarning, this, obstacle);
				obDist -= highway->GetObDist();
			  }		  
		  }
		}
	
   // Guarda o ID do último objeto criado que não é um "vehicle Veículo"
   highway->SetFirstIdVeh (VID);
   highway->SetAutoInject(true);

      // Return true: a signal to highway that the lane lists (queues) in where obstacles and vehicles are being added
      // must be sorted based on their positions.
      // Return false: to ignore sorting (do not return false when vehicles are manually added to the highway).
      return true;
  }
	
  // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
  /*bool Controller::ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
  {
    // we aim to create outputs which are readable by gnuplot for visulization purpose
    // this can be happen at beginning of each simulation step here. 
    if(Plot==true)
    {
      bool newStep=false;
      double now=Simulator::Now().GetHighPrecision().GetDouble();
      if(now > T)
      {
        T = now;
        newStep=true;
      }
			
      if(newStep==true)
      {
        if(T!=0.0)
        {
          cout << "e" << endl;
          //cout << "pause " << dt << endl;
        }
        float xrange = highway->GetHighwayLength();
        float yrange = highway->GetLaneWidth()*highway->GetNumberOfLanes();
        if(highway->GetTwoDirectional()) yrange=2*yrange + highway->GetMedianGap();
        cout << "set xrange [0:"<< xrange <<"]" << endl;
        cout << "set yrange [0:"<< yrange <<"]" << endl;
        cout << "plot '-' w points" << endl;
        newStep=false;
      }
//      cout << " Posicao " << vehicle->GetPosition().x << " " << vehicle->GetPosition().y << "Velocidade " << vehicle->GetVelocity() << endl;
      if(newStep==false)
      {
        cout << vehicle->GetPosition().x << " " << vehicle->GetPosition().y << endl;
      }
    }

    // to decelerate and stop the police car reaching the obstacle 
    if(vehicle->GetVehicleId()==2 && vehicle->GetPosition().x >=400)
    {
      vehicle->SetAcceleration(-2.0);
      // return true: a signal to highway that we aim to manually control the vechile 
      return true;
    }
    */
    // return false: a signal to highway that lets the vehicle automatically be handled (using IDM/MOBIL rules)
    //return false;
  //}
  
  // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
  /*void Controller::BroadcastWarning(Ptr<Vehicle> veh)
  {
    stringstream msg;
    msg << "'Enviando mensagem: apID => " << veh->GetVehicleId()
	<< " Posicao do AP => " << veh->GetPosition().x << " m" 
	<< " Direcao => " << veh->GetDirection()
        << " Pista => " << veh->GetLane()
	<< "'";
		
    Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());

    veh->SendTo(veh->GetBroadcastAddress(), packet);
		
    Simulator::Schedule(Seconds(5.0),&Controller::BroadcastWarning, this, veh);
  }*/

  // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
  //void Controller::ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address)
  //{
   /*string data=string((char*)packet->PeekData());
    stringstream ss (stringstream::in | stringstream::out);
			
    //double obs_id, obs_x; 
    //ss << data;
    //ss >> obs_id;
    //ss >> obs_x;

    int vid=veh->GetVehicleId();
    double now=Simulator::Now().GetSeconds();

    if(!Plot)
      cout << "TEMPO => " << now << " Vehi_Id => " << vid << " Distancia =>" << veh->GetPosition().x << " , " << veh->GetPosition().y << " Velocidade => " << veh->GetVelocity()  << " Mensagem recebida => " << data <<  endl;
*/
    //the police reaction to the received message
/*    if(vid==2)
    {
      double diff=obs_x - veh->GetPosition().x;
      stringstream msg;
      if(diff>2)
        msg << "police is only " << obs_x - veh->GetPosition().x << "(m) away from you vehicle " << obs_id;  
      else
        msg << "police is at the location x~" << obs_x << " to observe the case with id=" << obs_id;

      Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
      veh->SendTo(address, packet);
    }*/
  //}
}
