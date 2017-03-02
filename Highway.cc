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
#include <math.h>
#include <string>
#include "Highway.h"
#include "ns3/simulator.h"

namespace ns3
{
  TypeId Highway::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Highway")
    .SetParent<Object> ()
    .AddConstructor<Highway> ()
    ;
    return tid;
  }

  void Highway::InitHighway()
  {
    SeedManager::SetSeed (3);
    if(m_laneChangeSedan == 0)
      {
        m_laneChangeSedan = CreateObject<LaneChange>();
        m_laneChangeSedan->SetPolitenessFactor(0.2);
        m_laneChangeSedan->SetDbThreshold(0.3);
        /* Alterado por Thales: retirei o atributo da distância de segurança do modelo de troca de pista e coloquei 
         * diretamente como atributo do veículo, onde o acesso nas funções necessarias e que antes
         * acessavam a variável do modelo. Necessário pois não estava sendo possível alterar o valor
         * da distância de segurança dinâmicamente, uma vez que como todos os veículos apontam para
         * o mesmo objeto (passados no InjectVehicles, e não havia problema nisso, uma vez que os 
         * valores não eram alterados), caso alterasse a distância de um veículo, alteraria também
         * para todos os que apontavam para o mesmo objeto, perdendo a especificidade da distância.*/
        // 2 metros
        //m_laneChangeSedan->SetGapMin(2.0);
        m_laneChangeSedan->SetMaxSafeBreakingDeceleration(12.0);
        m_laneChangeSedan->SetBiasRight(0.2);
      }
    if(m_laneChangeTruck == 0)
      {
        m_laneChangeTruck = CreateObject<LaneChange>();
        m_laneChangeTruck->SetPolitenessFactor(0.2);
        m_laneChangeTruck->SetDbThreshold(0.2);
        // Idem à explicação acima.
        // 2 metros
        //m_laneChangeTruck->SetGapMin(2.0);
        m_laneChangeTruck->SetMaxSafeBreakingDeceleration(12.0);
        m_laneChangeTruck->SetBiasRight(0.3);
      }
    if(m_sedan == 0)
      {
        m_sedan = CreateSedanModel();
      }
    if(m_truck == 0)
      {
        m_truck = CreateTruckModel();
      }

    m_vehicleId = 1;
    bool init = false;
    
	if(!m_initVehicle.IsNull())
	  {
        init = m_initVehicle(Ptr<Highway>(this), m_vehicleId);
	  }

    if(init==true)
      {
		/*Adicionado por Thales (não há a necessidade de ordenar, já que as posições são
		estáticas. Fiz apenas para manter o padrão.*/		
		m_vehiclesAp.sort(ns3::Vehicle::Compare);
		// Fim da adição
		
        for(int i=0;i<m_numberOfLanes;i++)
          {
            m_vehicles[i].sort(ns3::Vehicle::Compare);
            if(m_twoDirectioanl==true) 
              m_vehiclesOpp[i].sort(ns3::Vehicle::Compare);
          }
      }
	
    if(m_autoInject==true) 
	  InjectVehicles(m_injectionSafetyGap, (int)m_sedanTruckPerc);
  }

  Ptr<Model> Highway::CreateSedanModel()
  {
    Ptr<Model> model = CreateObject<Model>();
    model->SetName("Sedan");
    model->SetDesiredVelocity(this->GetSedanVelocity()); //25.0;
    model->SetDeltaV(4.0);
    model->SetAcceleration(0.5);  
    model->SetDeceleration(3.0);  
    // Idem à explicação contida na função InitHighway
    // 2 metros
    //model->SetMinimumGap(2.0);
    model->SetTimeHeadway(0.1); // 0.1, 1.5; 
    model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
    return model;
  }

  Ptr<Model> Highway::CreateTruckModel()
  {
    Ptr<Model> model = CreateObject<Model>();
    model->SetName("Truck");
    model->SetDesiredVelocity(this->GetTruckVelocity()); // 22.2
    model->SetDeltaV(4.0);
    model->SetAcceleration(0.2);
    model->SetDeceleration(4.0);
    // Idem à explicação contida na função InitHighway
    // 2 metros
    //model->SetMinimumGap(2.0);
    model->SetTimeHeadway(0.5);
    model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
    return model;
  }

  Ptr<Vehicle> Highway::GetVehicle(std::list<Ptr<Vehicle> > v, int index)
  {
	std::list<Ptr<Vehicle> >::iterator i=v.begin();
    advance(i,index);
    return *i;
  }

  double Highway::GetYForLane(int lane, int dir)
  {
    if(dir == 1)
	  {
		return m_laneWidth * (lane + 0.5);
	  }
    return ((2 * m_numberOfLanes * m_laneWidth) + m_medianGap - GetYForLane(lane, 1));
  }

  double Highway::GetMedianGap()
  {
    return m_medianGap;
  }

  void Highway::SetMedianGap(double value)
  {
    m_medianGap=value;
  }

  double Highway::GetInjectionGap()
  {
    return m_injectionSafetyGap;
  }

  void Highway::SetInjectionGap(double value)
  {
    if(value > 0 )
      m_injectionSafetyGap= value;
  }

  double Highway::GetInjectionMixValue()
  {
    return m_sedanTruckPerc;
  }

  void Highway::SetInjectionMixValue(double value)
  {
    if(value>=0 && value<=100) 
	  m_sedanTruckPerc=value;
  }

  bool Highway::GetChangeLane()
  {
    return m_changeLaneSet;
  }

  void Highway::SetChangeLane(bool value)
  {
    m_changeLaneSet=value;
  }
  
  void Highway::InjectVehicles(double minGap, int p)
  {
    UniformVariable uRnd(0,100);
    int last;
    double gap;
    double vel=0.0;
    //int cria [m_numberOfLanes];
    for (int i = 0; i < m_numberOfLanes; i++)
      {
		//cria[i] = uRnd.GetValue();
        last = m_vehicles[i].size()-1;
        if (last < 0) 
          { 
            gap = minGap + 1;
            vel = 10+2*i;
            //vel=0.0;
          }
        else
          {
            gap = GetVehicle(m_vehicles[i],last)->GetPosition().x;
            //vel = GetVehicle(m_vehicles[i],last)->GetVelocity()/2;
            if(GetVehicle(m_vehicles[i],last)->GetVelocity() == 0.0)
				vel = 10+2*i;
			else
				vel = GetVehicle(m_vehicles[i],last)->GetVelocity();
            //vel=0.0;
          }
        if (gap > minGap)
          {
            if (uRnd.GetValue() <= p)
            //if (cria[i] <= p)
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
                // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                //temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(1);
                temp->SetPosition(Vector(-4,GetYForLane(i,1),0));
                temp->SetLane(i);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
                temp->SetModel(m_sedan);
                temp->SetLaneChange(m_laneChangeSedan);
                temp->SetLength(4);
                temp->SetWidth(2);
                // Configura distância inicial de segurança para 10 m.
                temp->SetDistSegDin(10);
                // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                //temp->SetReceiveCallback(m_receiveData);
                //temp->SetDevTxTraceCallback(m_devTxTrace);
                //temp->SetDevRxTraceCallback(m_devRxTrace);
                //temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                //temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                //temp->SetPhyTxTraceCallback(m_phyTxTrace);
                //temp->SetPhyStateTraceCallback(m_phyStateTrace);
                // Adicionado por Thales (inicializa TCT no momento de criação dos veículos com todos os trechos)
				temp->SetTable(((m_apQtd-1)*2));
                // Fim da adição
                m_vehicles[i].push_back(temp);
                contCarros1 += 1;
              }
            else
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
                // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                //temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(1);
                temp->SetPosition(Vector(-8,GetYForLane(i,1),0));
                temp->SetLane(i);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
                temp->SetModel(m_truck);
                temp->SetLaneChange(m_laneChangeTruck);
                temp->SetLength(8);
                temp->SetWidth(2);
                // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                //temp->SetReceiveCallback(m_receiveData);
                //temp->SetDevTxTraceCallback(m_devTxTrace);
                //temp->SetDevRxTraceCallback(m_devRxTrace);
                //temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                //temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                //temp->SetPhyTxTraceCallback(m_phyTxTrace);
                //temp->SetPhyStateTraceCallback(m_phyStateTrace);
                // Idem à explicação acima
                temp->SetDistSegDin(10);
                // Adicionado por Thales (inicializa TCT no momento de criação dos veículos com todos os trechos)
				temp->SetTable(((m_apQtd-1)*2));
                // Fim da adição
                m_vehicles[i].push_back(temp);
                contCaminhoes1 += 1;
              }
          }
      }
    if(m_twoDirectioanl==true)
      {
        for (int i = 0; i < m_numberOfLanes; i++)
          {
            last = m_vehiclesOpp[i].size()-1;
            if (last < 0) 
              {
                gap = minGap + 1;
                vel = 10+2*i;
               //vel=0.0;
              }
            else
              {
                gap = m_highwayLength - GetVehicle(m_vehiclesOpp[i],last)->GetPosition().x;
                //vel = GetVehicle(m_vehiclesOpp[i],last)->GetVelocity()/2;
                 if(GetVehicle(m_vehiclesOpp[i],last)->GetVelocity() == 0.0)
					vel = 10+2*i;
				 else
					vel = GetVehicle(m_vehiclesOpp[i],last)->GetVelocity();
                //vel=0.0;
              }
            if (gap > minGap)
              {
                if (uRnd.GetValue() <= p)
                //if (cria[i] <= p)
                  {
                    Ptr<Vehicle> temp=CreateObject<Vehicle>();
                    // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                    //temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
                    temp->SetVehicleId(m_vehicleId++);
                    temp->SetDirection(-1);
                    temp->SetPosition(Vector(m_highwayLength+4,GetYForLane(i,-1),0));
                    temp->SetLane(i);
                    temp->SetVelocity(vel);
                    temp->SetAcceleration(0.0);
                    temp->SetModel(m_sedan);
                    temp->SetLaneChange(m_laneChangeSedan);
                    temp->SetLength(4);
                    temp->SetWidth(2);
                    //temp->SetReceiveCallback(m_receiveData);
                    //temp->SetDevTxTraceCallback(m_devTxTrace);
                    //temp->SetDevRxTraceCallback(m_devRxTrace);
                    //temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                    //temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                    //temp->SetPhyTxTraceCallback(m_phyTxTrace);
                    //temp->SetPhyStateTraceCallback(m_phyStateTrace);
                    // Adicionado por Thales
                    temp->SetIdApAnt(m_apQtd);
                    // Fim da adição                    
                    // Idem à explicação acima
					temp->SetDistSegDin(10);                
                    // Adicionado por Thales (inicializa TCT no momento de criação dos veículos com todos os trechos)
					temp->SetTable(((m_apQtd-1)*2));
					// Fim da adição
                    m_vehiclesOpp[i].push_back(temp);
                    contCarros2 += 1;
                  }
                else
                  {
                    Ptr<Vehicle> temp=CreateObject<Vehicle>();
                    // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                    //temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
                    temp->SetVehicleId(m_vehicleId++);
                    temp->SetDirection(-1);
                    temp->SetPosition(Vector(m_highwayLength+8,GetYForLane(i,-1),0));
                    temp->SetLane(i);
                    temp->SetVelocity(vel);
                    temp->SetAcceleration(0.0);
                    temp->SetModel(m_truck);
                    temp->SetLaneChange(m_laneChangeTruck);
                    temp->SetLength(8);
                    temp->SetWidth(2);
                    // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
                    //temp->SetReceiveCallback(m_receiveData);
                    //temp->SetDevTxTraceCallback(m_devTxTrace);
                    //temp->SetDevRxTraceCallback(m_devRxTrace);
                    //temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                    //temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                    //temp->SetPhyTxTraceCallback(m_phyTxTrace);
                    //temp->SetPhyStateTraceCallback(m_phyStateTrace);
                    // Adicionado por Thales
                    temp->SetIdApAnt(m_apQtd);
                    // Fim da adição
                    // Idem à explicação acima
					temp->SetDistSegDin(10);
					// Adicionado por Thales (inicializa TCT no momento de criação dos veículos com todos os trechos)
					temp->SetTable(((m_apQtd-1)*2));
					// Fim da adição
                    m_vehiclesOpp[i].push_back(temp);
                    contCaminhoes2 += 1;  
                  }           
              }
               
          }
      }
  }

  void Highway::TranslateVehicles()
  {
    if(m_stopped==true) return;
    static int loop=0;
    // NOTE: ORDER OF CALLING THIS FUNCTIONS IS VERY VERY IMPORTANT (EFFECT OF CURRENT SPEED, POSITION, DECICION)
    if(loop==10) loop=0;
    if(loop==0 && m_changeLaneSet==true) 
      {
        ChangeLane(m_vehicles);
        if(m_twoDirectioanl==true) 
          ChangeLane(m_vehiclesOpp);
      }
    TranslatePositionVelocity(m_vehicles, m_dt);
    if(m_twoDirectioanl==true) 
	  TranslatePositionVelocity(m_vehiclesOpp, m_dt); 
    
	Accelerate(m_vehicles, m_dt);    
	if(m_twoDirectioanl==true) 
	  Accelerate(m_vehiclesOpp, m_dt);
    
    if(m_autoInject==true) 
	  InjectVehicles(m_injectionSafetyGap, (int)m_sedanTruckPerc);
 
    loop++;
	Simulator::Schedule(Seconds(m_dt), &Highway::Step, Ptr<Highway>(this));    
  }

  void Highway::Accelerate(std::list<Ptr<Vehicle> > vehicles[], double dt)
  {
    for (int i = 0; i < m_numberOfLanes; i++)
      {
        for (uint j = 0; j < vehicles[i].size(); j++)
          {
            Ptr<Vehicle> veh=GetVehicle(vehicles[i],j);
            bool controled=false;
            if(!m_controlVehicle.IsNull()) 
		      controled=m_controlVehicle(Ptr<Highway>(this), veh, dt);
            if(controled==false)
              {
                if (j == 0)
                  {
                    veh->Accelerate(0);
                    continue;
                  }
                veh->Accelerate(GetVehicle(vehicles[i],j - 1));
              }
           }
      }
  }

  void Highway::TranslatePositionVelocity(std::list<Ptr<Vehicle> > vehicles[], double dt)
  {
    std::list<Ptr<Vehicle> > reachedEnd;
    for (int i = 0; i < m_numberOfLanes; i++)
      {
        for (uint j = 0; j < vehicles[i].size(); j++)
          {
            Ptr<Vehicle> veh=GetVehicle(vehicles[i],j);
            veh->TranslatePosition(dt);
            veh->TranslateVelocity(dt);
            /* testeaviso: À todo tempo, verifica se o veículo fez uma troca de pista forçada devido à um obstáculo, 
             * e caso sim, verifica se o mesmo já passou pela posição do obstáculo, podendo, desta forma, voltar a
             * trocar de pista se necessário, apagando as informações do obstáculo ultrapassado da base de dados
             * particular do veículo.*/
            // controle: só entra nesta condição se o aviso de obstáculos estiver habilitado.
            if (GetAvisoObst())
            {
				if(veh->GetDirection() == 1)
				{
					if ((veh->GetTrocaForcada() == true ) and (veh->GetPosition().x >= veh->GetValObst().pos))
					{
						veh->SetTrocaForcada(false);
						veh->SetValObst(0.0, 10);
					}
				}
				else
				{
					if ((veh->GetTrocaForcada() == true ) and (veh->GetPosition().x <= veh->GetValObst().pos))
					{
						veh->SetTrocaForcada(false);
						veh->SetValObst(0.0, 10);
					}
				}
			}
			// fim
            /* À cada vez que atualiza a velocidade do veículo, verifica qual sua distância
             * de segurança proporcional.*/   
            // 2 metros
            
			double vel = veh->GetVelocity();
            double DistSegDin;
			if (vel >= 25.0)
				DistSegDin =  52.0;
			else if (vel >= 23.6)
				DistSegDin = 47.0;
			else if (vel >= 22.2)
				DistSegDin = 42.0;
			else if (vel >= 20.8)
				DistSegDin = 37.0;
			else if (vel >= 19.4)
				DistSegDin = 33.0;
			else if (vel >= 18.0)
				DistSegDin = 30.0;
			else if (vel >= 16.6)
				DistSegDin = 25.0;
			else if (vel >= 15.2)
				DistSegDin = 22.0;
			else if (vel >= 13.8)
				DistSegDin = 18.0;
			else if (vel >= 12.5)
				DistSegDin = 15.0;
			else if (vel >= 11.1)
				DistSegDin = 12.0;
			else if (vel >= 8.3)
				DistSegDin = 9.0;
			else if (vel >= 5.5)
				DistSegDin = 6.0;
			else if (vel >= 2.7)
				DistSegDin = 4.0;
			else
				DistSegDin = 2.0;
			// fim
			
			veh->SetDistSegDin(DistSegDin);
			// fim 2 metros
            //if(veh->GetPosition().x > m_highwayLength && veh->GetDirection()==1) 
            // Alterado por Thales (para não remover os obstáculos colocados fora da pista)
            if(veh->GetPosition().x > m_highwayLength && veh->GetDirection()==1 && veh->GetVehicleId() >= m_firstIdVeh) 
			  reachedEnd.push_back(veh);
            else if(veh->GetPosition().x <0 && veh->GetDirection()==-1 && veh->GetVehicleId() >= m_firstIdVeh) 
			  reachedEnd.push_back(veh);
          }

        for(uint r=0; r<reachedEnd.size(); r++)
          {
            Ptr<Vehicle> rm=GetVehicle(reachedEnd, r);
            vehicles[i].remove(rm);
            // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
            //rm->GetReceiveCallback().Nullify();
            // to put vehicle's node far away from the highway
            // we cannot dispose the vehicle here because its node may still be involved in send and receive process
            rm->SetPosition(Vector(10000, 10000, 10000)); 
            rm=0;
          }

        reachedEnd.clear();
      }
  }

  void Highway::ChangeLane(std::list<Ptr<Vehicle> > vehicles[])
  {
    if (m_numberOfLanes <= 1)
      {
        return;
      }

    for (int i = 0; i < m_numberOfLanes; i++)
      {
        if (i < 1)
          {
            DoChangeLaneIfPossible(vehicles,i, i + 1);
            continue;
          }
        if (i + 1 >= m_numberOfLanes)
          {
            DoChangeLaneIfPossible(vehicles,i, i - 1);
            continue;
          }
        DoChangeLaneIfPossible(vehicles, i, i + 1);
        DoChangeLaneIfPossible(vehicles, i, i - 1);
      }     
  }

  void Highway::DoChangeLaneIfPossible(std::list<Ptr<Vehicle> > vehicles[], int curLane, int desLane)
  {
	std::list<Ptr<Vehicle> > canChange;
    canChange.clear();
    
	for (uint j = 0; j < vehicles[curLane].size(); j++)
      {
        Ptr<Vehicle> fOld = 0;
        if (j > 0)
          {
            fOld = GetVehicle(vehicles[curLane],j - 1);
          }

        FindSideVehicles(vehicles, GetVehicle(vehicles[curLane],j), desLane);
        /* testeaviso: só pode trocar de pista os veículos que não possuem uma troca forçada ativa, se não tem obstáculo
         * no trecho que o veículo está ou se tem obstáculo, mas o mesmo está na mesma pista do veículo. Necessário para
         * evitar que o veículo retorne para a mesma pista que desviara antes de um obstáculo, ou que troque de pista
         * e acabe indo parar em uma pista que justamente possui um obstáculo*/
        //if (GetVehicle(vehicles[curLane],j)->CheckLaneChange(fOld, m_tempVehicles[0], m_tempVehicles[1], (curLane < desLane) ? true : false))
        /* controle: só entra nesta condição se o aviso de obstáculos estiver habilitado. Se não estiver, verifica a
         * possibilidade de trocar de pista normalmente.*/        
        if (GetAvisoObst())
        {
			if (GetVehicle(vehicles[curLane],j)->CheckLaneChange(fOld, m_tempVehicles[0], m_tempVehicles[1], (curLane < desLane) ? true : false) 
				and (GetVehicle(vehicles[curLane],j)->GetTrocaForcada() == false) 
				and ((GetVehicle(vehicles[curLane],j)->GetValObst().pista == 10) 
					or ((GetVehicle(vehicles[curLane],j)->GetValObst().pista) == (GetVehicle(vehicles[curLane],j)->GetLane()))))
			{
				canChange.push_back(GetVehicle(vehicles[curLane],j));
			}  
		}
		else
		{
			if (GetVehicle(vehicles[curLane],j)->CheckLaneChange(fOld, m_tempVehicles[0], m_tempVehicles[1], (curLane < desLane) ? true : false))
			{
				canChange.push_back(GetVehicle(vehicles[curLane],j));
			}
		}   
      }

    for (uint j = 0; j < canChange.size(); j++)
      {
		/* testeaviso: quando o veículo que está indo de encontro à um obstáculo conseguir trocar de pista, 
		 * deve-se setar a troca forçada, para que o mesmo não corra o risco de retornar à pista que possui obstáculo*/
		if(GetAvisoObst())
		{
			if (GetVehicle(canChange,j)->GetValObst().pos != 0.0)
				GetVehicle(canChange,j)->SetTrocaForcada(true);
		}
		// fim
        Vector position=GetVehicle(canChange,j)->GetPosition();
        position.y=GetYForLane(desLane, GetVehicle(canChange,j)->GetDirection());
        GetVehicle(canChange,j)->SetLane(desLane);
        GetVehicle(canChange,j)->SetPosition(position);
        vehicles[curLane].remove(GetVehicle(canChange,j));
        vehicles[desLane].push_back(GetVehicle(canChange,j));
      }

	m_vehicles[desLane].sort(ns3::Vehicle::Compare);
	/* Adicionado por Thales (não havia no código original, o que gerava uma quantidade muito maior
	... de veículos na direção -1*/
	m_vehiclesOpp[desLane].sort(ns3::Vehicle::Compare);
  }  
  

  void Highway::FindSideVehicles(std::list<Ptr<Vehicle> > vehicles[], Ptr<Vehicle> veh, int sideLane)
  {
    int front=-1, back=-1;
    m_tempVehicles[0] = 0;
	m_tempVehicles[1] = 0;
    for (uint i = 0; i < vehicles[sideLane].size(); i++)
      {
        if(veh->GetDirection() == 1)
          {	
            if (GetVehicle(vehicles[sideLane],i)->GetPosition().x <= veh->GetPosition().x)
              {
                back = i;
                front = back - 1;
                break;
              }
          }
        else
          {
            if (GetVehicle(vehicles[sideLane],i)->GetPosition().x >= veh->GetPosition().x)
            {
              back = i;
              front = back - 1;
              break;
            }
          }
      }

    if (back < 0)
      {
        front = vehicles[sideLane].size()-1;
      }
    if (back > -1)
      {
        m_tempVehicles[1] = GetVehicle(vehicles[sideLane], back);
      }
    if (front > -1)
      {
        m_tempVehicles[0] = GetVehicle(vehicles[sideLane], front);
      }
  }

  Highway::Highway()
  {
    m_dt=0.1;
    m_vehicleId=1;
    m_numberOfLanes=1;
    m_highwayLength=1000;
    m_laneWidth=5;
    m_laneChangeSedan=0;
    m_laneChangeTruck=0;
    m_sedan=0;
    m_truck=0;
    m_autoInject=false;
    m_twoDirectioanl=false;
    m_medianGap=5;
    m_injectionSafetyGap=50;
    m_sedanTruckPerc=80;
    m_changeLaneSet=false;
    // Adicionado por Thales
    m_obDinamico = false;
    m_avisoObst = false;

    // Setup Wifi
    m_wifiHelper = WifiHelper::Default();	
    m_wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
    m_wifiMacHelper = NqosWifiMacHelper::Default();
    m_wifiPhyHelper = YansWifiPhyHelper::Default();
    m_wifiChannelHelper = YansWifiChannelHelper::Default ();
    m_wifiMacHelper.SetType ("ns3::AdhocWifiMac");
    m_wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue ("wifia-6mbs"));
    //m_wifiChannelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
	m_wifiChannel = m_wifiChannelHelper.Create();
    m_wifiPhyHelper.SetChannel (m_wifiChannel);
    m_wifiPhyHelper.Set("TxPowerStart",DoubleValue(21.5));	// 250-300 meter transmission range 
    m_wifiPhyHelper.Set("TxPowerEnd",DoubleValue(21.5));      // 250-300 meter transmission range 			
    m_wifiPhyHelper.Set("TxPowerLevels",UintegerValue(1)); 
    m_wifiPhyHelper.Set("TxGain",DoubleValue(2)); 
    m_wifiPhyHelper.Set("RxGain",DoubleValue(2));  
    m_wifiPhyHelper.Set("EnergyDetectionThreshold", DoubleValue(-101.0));
  }

  Highway::~Highway()
  {
    m_laneChangeSedan=0;
    m_laneChangeTruck=0;
    m_sedan=0;
    m_truck=0;
    m_tempVehicles[0]=0; 
	m_tempVehicles[1]=0;
	
	// Adicionado por Thales (destrutor do array criado para Aps)
	m_vehiclesAp.erase(m_vehiclesAp.begin(), m_vehiclesAp.end());
	// Fim da adição
	
    for(int i=0;i<m_numberOfLanes;i++)
      {
        m_vehicles[i].erase(m_vehicles[i].begin(), m_vehicles[i].end());
      }
    if(m_twoDirectioanl==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
        {
          m_vehiclesOpp[i].erase(m_vehiclesOpp[i].begin(), m_vehiclesOpp[i].end());
        }
      }
  }
  void Highway::Step(Ptr<Highway> highway)
  {
    highway->TranslateVehicles();
  }
  void Highway::Start()
  {
    m_stopped=false;
    InitHighway();
    Simulator::Schedule(Seconds(0.0), &Step, Ptr<Highway>(this));
  }

  void Highway::Stop()
  {
    m_stopped=true;
  }
 
  double Highway::GetDeltaT()
  {
    return m_dt;
  }

  void Highway::SetDeltaT(double value)
  {
    if(value<=0) 
	  value=0.1;

    m_dt=value;
  }

  bool Highway::GetTwoDirectional()
  {
    return m_twoDirectioanl;
  }

  void Highway::SetTwoDirectional(bool value)
  {
    m_twoDirectioanl=value;
  }

  int Highway::GetNumberOfLanes()
  {
    return m_numberOfLanes;
  }

  void Highway::SetNumberOfLanes(int value)
  {
    if(value<1) 
	  value=1;
    else if(value>5) 
	  value=5;

    m_numberOfLanes=value;
  }

  double Highway::GetHighwayLength()
  {
    return m_highwayLength;
  }

  void Highway::SetHighwayLength(double value)
  {
    if(value<0) 
	  value=10000;

    m_highwayLength=value;
  }

  double Highway::GetLaneWidth()
  {
   return m_laneWidth;
  }

  void Highway::SetLaneWidth(double value)
  {
    if(value<0) 
	  value=5;

    m_laneWidth=value;
  }

  void Highway::PrintVehicles()
  {
    std::cout << "Lane 2----------------" << Simulator::Now()<< "--------" << std::endl;
    for(uint i=0; i<m_vehicles[1].size();i++)
      {
        Ptr<Vehicle> v=GetVehicle(m_vehicles[1],i);
		std::cout<< v->GetVehicleId() << ":" << v->GetPosition().x 
                 << ":" << v->GetPosition().y << ":" << v->GetVelocity() << std::endl;
      }
    std::cout << "----------------------" << std::endl;
  }

  Ptr<Model> Highway::GetSedanModel()
  {
    return m_sedan;
  }

  void Highway::SetSedanModel(Ptr<Model> value)
  {
    m_sedan=value;
  }

  Ptr<Model> Highway::GetTruckModel()
  {
    return m_truck;
  }

  void Highway::SetTruckModel(Ptr<Model> value)
  {
    m_truck=value;
  }

  Ptr<LaneChange> Highway::GetSedanLaneChange()
  {
    return m_laneChangeSedan;
  }

  void Highway::SetSedanLaneChange(Ptr<LaneChange> value)
  {
    m_laneChangeSedan=value;
  }

  Ptr<LaneChange> Highway::GetTruckLaneChange()
  {
    return m_laneChangeTruck;
  }

  void Highway::SetTruckLaneChange(Ptr<LaneChange> value)
  {
    m_laneChangeTruck=value;
  }

  bool Highway::GetAutoInject()
  {
    return m_autoInject;
  }

  void Highway::SetAutoInject(bool value)
  {
    m_autoInject=value; 
  }

  int Highway::GetLastVehicleId()
  {
    return m_vehicleId;
  }

  WifiHelper Highway::GetWifiHelper()
  {
    return m_wifiHelper;
  }

  NqosWifiMacHelper Highway::GetNqosWifiMacHelper()
  {
    return m_wifiMacHelper;
  }

  YansWifiPhyHelper Highway::GetYansWifiPhyHelper()
  {
	return m_wifiPhyHelper;
  }
  	  
  Ptr<YansWifiChannel> Highway::GetWifiChannel()
  {
    return m_wifiChannel;
  }

  void Highway::AddVehicle(Ptr<Vehicle> vehicle)
  {
    int lane=vehicle->GetLane();
    int dir=vehicle->GetDirection();
    
    // Adicionado por Thales (verifica se objeto vehicle é um AP)
    bool ap = vehicle->GetIsAp();
    
    if((lane < m_numberOfLanes && lane >= 0) or (ap))
      {
        if(!ap && dir==1) 
		  m_vehicles[lane].push_back(vehicle);
        else if(!ap && dir==-1) 
		  m_vehiclesOpp[lane].push_back(vehicle);
		else
		  // Adicionado por Thales (insere Aps em um array próprio, para ficar fora da via)
		  m_vehiclesAp.push_back(vehicle);
		  // Fim da adição
      }
  }

  Ptr<Vehicle> Highway::FindVehicle(int vid) // Alterado por Thales: Código original = "i<m_numberOfLanes"
  {
    Ptr<Vehicle> v=NULL;
    
    // Adicionado por Thales (retorna se o ID procurado é um ID de AP, quando i <= ApQtd)
	for(uint j=0;j<m_vehiclesAp.size();j++)
	{	
		v=GetVehicle(m_vehiclesAp,j);
		if(v->GetVehicleId()==vid)
		  return v;
	}
    // Fim da adição
    
    // Caso i > ApQtd, procura no array de veículos comuns (que não são Aps).  
    for(int i=0;i<m_numberOfLanes;i++)
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            if(v->GetVehicleId()==vid) 
			  return v;
          }
      }
      
    if(m_twoDirectioanl==true)
      {
        for(int i=0;i<m_numberOfLanes;i++) 
          {
            for(uint j=0;j<m_vehiclesOpp[i].size();j++)
              {	
                v=GetVehicle(m_vehiclesOpp[i],j);
                if(v->GetVehicleId()==vid) 
					return v;
              }
          }
      }
    //cout << "\n\nIREI RETORNAR UM VEICULO NULO para: " << vid <<endl;
    return v;
  }

  std::list<Ptr<Vehicle> > Highway::FindVehiclesInRange(Ptr<Vehicle> vehicle, double range)
  {
    std::list<Ptr<Vehicle> > neighbors;
    if(range<=0) 
	  return neighbors;

    Ptr<Vehicle> v=0;
    double diff=0;
    Vector pos, p;
    p=vehicle->GetPosition();
    for(int i=0;i<m_numberOfLanes;i++) 
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            pos=v->GetPosition();
            if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			  continue;
            
			diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
            if(diff < range) neighbors.push_back(v);
          }
      }
    if(m_twoDirectioanl==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
        {
          for(uint j=0;j<m_vehiclesOpp[i].size();j++)
            {	
              v=GetVehicle(m_vehiclesOpp[i],j);
              pos=v->GetPosition();
              if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			    continue;

              diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
              if(diff < range) neighbors.push_back(v);
            }
        }
      }

  return neighbors;
  }
  
  // Sobrecarga para retornar os veículos vizinhos de cada direção em uma lista distinta
  std::list<Ptr<Vehicle> > Highway::FindVehiclesInRange(Ptr<Vehicle> vehicle, double range, int dir)
  {
    std::list<Ptr<Vehicle> > neighbors;
    if(range<=0) 
	  return neighbors;

    Ptr<Vehicle> v=0;
    double diff=0;
    Vector pos, p;
    p=vehicle->GetPosition();
    
    if (dir == 1)
    {
    for(int i=0;i<m_numberOfLanes;i++) 
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            pos=v->GetPosition();
            if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			  continue;
            
			diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
            if(diff < range) neighbors.push_back(v);
          }
      }
	}
    else if(dir == -1)
      {
        for(int i=0;i<m_numberOfLanes;i++)
        {
          for(uint j=0;j<m_vehiclesOpp[i].size();j++)
            {	
              v=GetVehicle(m_vehiclesOpp[i],j);
              pos=v->GetPosition();
              if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			    continue;

              diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
              if(diff < range) neighbors.push_back(v);
            }
        }
      }

  return neighbors;
  }
  
  std::list<Ptr<Vehicle> > Highway::FindVehiclesInRange(int vid, double x, double y, double range)
  {
    std::list<Ptr<Vehicle> > neighbors;
    if(range<=0) 
	  return neighbors;

    Ptr<Vehicle> v=0;
    double diff=0;
    Vector pos;
    for(int i=0;i<m_numberOfLanes;i++)
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            pos=v->GetPosition();
            if(v->GetVehicleId()==vid) 
			  continue;
            
			diff= sqrt(pow(pos.x-x,2) + pow(pos.y-y,2));
            if(diff < range) neighbors.push_back(v);
          }
      }
    if(m_twoDirectioanl==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
        {
          for(uint j=0;j<m_vehiclesOpp[i].size();j++)
            {	
              v=GetVehicle(m_vehiclesOpp[i],j);
              pos=v->GetPosition();
              if(v->GetVehicleId()==vid) 
			    continue;

              diff= sqrt(pow(pos.x-x,2) + pow(pos.y-y,2));
              if(diff < range) neighbors.push_back(v);
            }
        }
      }

  return neighbors;
  }

  std::list<Ptr<Vehicle> > Highway::FindVehiclesInSegment(double x1, double x2, int lane, int dir)
  {
    std::list<Ptr<Vehicle> > segment;
    Ptr<Vehicle> v=0;
    Vector pos;
    if(dir==1 && lane< 5 && lane>=0)
      {
        for(uint j=0;j<m_vehicles[lane].size();j++)
          {	
            v=GetVehicle(m_vehicles[lane],j);
            pos=v->GetPosition();
            if(pos.x >= x1 && pos.x < x2) segment.push_back(v);
          }
      }
    else if(dir==-1 && m_twoDirectioanl==true && lane < 5 && lane >= 0)
      {
        for(uint j=0;j<m_vehiclesOpp[lane].size();j++)
          { 	
            v=GetVehicle(m_vehiclesOpp[lane],j);
            pos=v->GetPosition();
            if(pos.x >= x1 && pos.x < x2) 
			  segment.push_back(v);
          }
      }

    return segment;		
  }
  
  // Adicionado por Thales (sobrecarga do método FindVehiclesInSegment)
  // Alterado o tipo de x1 e x2 para receber as posições t_posX e t_posX+500
  
  std::list<Ptr<Vehicle> > Highway::FindVehiclesInSegment(int x1, int x2, int dir)
  {
    std::list<Ptr<Vehicle> > segment;
    Ptr<Vehicle> v=0;
    Vector pos;
    
    if(dir==1){
		// Adicionado mais um "For" para percorrer todos os veículos de todas as pistas
		for(int i=0;i<m_numberOfLanes;i++) 
		{
			for(uint j=0;j<m_vehicles[i].size();j++)
			  {	
				v=GetVehicle(m_vehicles[i],j);
				pos=v->GetPosition();
				// Só adiciona na lista se for um veículo de verdade
				if(v->GetVehicleId()>=m_firstIdVeh)
					if(pos.x >= x1 && pos.x < x2) segment.push_back(v);
			  }
		}
	}
    else if(dir==-1){
		// Adicionado mais um "For" para percorrer todos os veículos de todas as pistas
		for(int i=0;i<m_numberOfLanes;i++) 
		{
			for(uint j=0;j<m_vehiclesOpp[i].size();j++)
			  {	
				v=GetVehicle(m_vehiclesOpp[i],j);
				pos=v->GetPosition();
				// Só adiciona na lista se for um veículo de verdade
				if(v->GetVehicleId()>=m_firstIdVeh)
					if(pos.x > x1 && pos.x <= x2) segment.push_back(v);
			  }
		}	
	}	
	return segment;
  }
  
  // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
  /*Callback<void, Ptr<Vehicle>, Ptr<const Packet>, Address> Highway::GetReceiveDataCallback()
  {
    return m_receiveData;
  }

  void Highway::SetReceiveDataCallback(Callback<void, Ptr<Vehicle>, Ptr<const Packet>, Address> receiveData)
  {
    m_receiveData = receiveData;
  }

  DeviceTraceCallback Highway::GetDevTxTraceCallback()
  {
    return m_devTxTrace;
  }

  void Highway::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace)
  {
    m_devTxTrace = devTxTrace;
  }

  DeviceTraceCallback Highway::GetDevRxTraceCallback()
  {
    return m_devRxTrace;
  }
  
  void Highway::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace)
  {
    m_devRxTrace = devRxTrace;
  }

  PhyRxOkTraceCallback Highway::GetPhyRxOkTraceCallback()
  {
    return m_phyRxOkTrace;
  }

  void Highway::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace)
  {
    m_phyRxOkTrace = phyRxOkTrace;
  }

  PhyRxErrorTraceCallback Highway::GetPhyRxErrorTraceCallback()
  {
    return m_phyRxErrorTrace; 
  }

  void Highway::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace)
  {
    m_phyRxErrorTrace = phyRxErrorTrace;
  }

  PhyTxTraceCallback Highway::GetPhyTxTraceCallback()
  {
    return m_phyTxTrace;
  }

  void Highway::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace)
  {
    m_phyTxTrace = phyTxTrace;
  }

  PhyStateTraceCallback Highway::GetPhyStateTraceCallback()
  {
    return m_phyStateTrace;
  }

  void Highway::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace)
  {
    m_phyStateTrace = phyStateTrace;
  }

  Callback<bool, Ptr<Highway> ,Ptr<Vehicle> , double> Highway::GetControlVehicleCallback()
  {
    return m_controlVehicle;
  }

  void Highway::SetControlVehicleCallback(Callback<bool, Ptr<Highway> ,Ptr<Vehicle> , double> controlVehicle)
  {
    m_controlVehicle = controlVehicle;
  }

  Callback<bool, Ptr<Highway>, int&> Highway::GetInitVehicleCallback()
  {
    return m_initVehicle;
  }*/

  void Highway::SetInitVehicleCallback(Callback<bool, Ptr<Highway>, int&> initVehicle)
  {
    m_initVehicle = initVehicle;
  }
  
  // Adicionado por Igor Quintanilha
  void Highway::SetApDist(int apDist)
  {
    m_apDist = apDist;
  }
  
  int Highway::GetApDist(void)
  {
    return m_apDist;
  }
  
  void Highway::SetApQtd(int apQtd)
  {
    m_apQtd = apQtd;
  }
  
  int Highway::GetApQtd(void)
  {
    return m_apQtd;
  }
  
  void Highway::SetApInitDist(int apInitDist)
  {
    m_apInitDist = apInitDist;
  }
  
  int Highway::GetApInitDist(void)
  {
    return m_apInitDist;
  }
  
  /*void Highway::SetApAleat(bool apAleat)
  {
    m_apAleat = apAleat;
  }
  
  bool Highway::GetApAleat(void)
  {
    return m_apAleat;
  }*/
  
  float Highway::GetSedanVelocity(void)
  {
    return m_sedanVelocity;
  }
  
  void Highway::SetSedanVelocity(float sedanVelocity)
  {
    m_sedanVelocity = sedanVelocity;
  }
  
  float Highway::GetTruckVelocity(void)
  {
    return m_truckVelocity;
  }
  
  void Highway::SetTruckVelocity(float truckVelocity)
  {
    m_truckVelocity = truckVelocity;
  }
  
  // Adicionado por Thales
  void Highway::SetObDist(int obDist)
  {
    m_obDist = obDist;
  }
  
  int Highway::GetObDist(void)
  {
    return m_obDist;
  }
  
  void Highway::SetObQtd(int obQtd)
  {
    m_obQtd = obQtd;
  }
  
  int Highway::GetObQtd(void)
  {
    return m_obQtd;
  }
  
  void Highway::SetObInitDist(int obInitDist)
  {
    m_obInitDist = obInitDist;
  }
  
  int Highway::GetObInitDist(void)
  {
    return m_obInitDist;
  }
  
  void Highway::SetObAleat(bool obAleat)
  {
    m_obAleat = obAleat;
  }
  
  bool Highway::GetObAleat(void)
  {
    return m_obAleat;
  }
  
  void Highway::SetAsObstacle(bool asObstacle)
  {
    m_asObstacle = asObstacle;
  }
  
  bool Highway::GetAsObstacle(void)
  {
    return m_asObstacle;
  }
  
  void Highway::SetFirstIdVeh(int firstIdVeh)
  {
    m_firstIdVeh = firstIdVeh;
  }
  
  int Highway::GetFirstIdVeh(void)
  {
    return m_firstIdVeh;
  }
  
  // Adicionado por Thales (retorna a quantidade de carros e caminhões em cada direção)
  int Highway::getContCarros1(void){
	  return contCarros1;
  }
  
  void Highway::setContCarros1(int value){
	  contCarros1 = value;
  }
  
  int Highway::getContCarros2(void){
	  return contCarros2;
  }
  
  void Highway::setContCarros2(int value){
	  contCarros2 = value;
  }
   
    int Highway::getContCaminhoes1(void){
	  return contCaminhoes1;
  }
  
  void Highway::setContCaminhoes1(int value){
	  contCaminhoes1 = value;
  }
  
  int Highway::getContCaminhoes2(void){
	  return contCaminhoes2;
  }
  
  void Highway::setContCaminhoes2(int value){
	  contCaminhoes2 = value;
  }
  
  // controle
  void Highway::SetObDinamico(bool obDin)
  {
	  m_obDinamico = obDin;
  }
  
  bool Highway::GetObDinamico()
  {
	  return m_obDinamico;
  }
  
  void Highway::SetAvisoObst(bool avisoOb)
  {
	  m_avisoObst = avisoOb;
  }
  bool Highway::GetAvisoObst()
  {
	  return m_avisoObst;
  }      
  // fim
  
}
