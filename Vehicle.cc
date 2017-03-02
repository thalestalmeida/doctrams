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

#include "Vehicle.h"

namespace ns3
{	
  TypeId Vehicle::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Vehicle")
    .SetParent<Object> ()
    .AddConstructor<Vehicle> ()
    ;
    return tid;
  }

  Vehicle::Vehicle()
  {
    m_node=CreateObject<Node>();
	MobilityHelper mobility;
	mobility.Install(m_node);
    m_vehicleId = 1;
    m_lane = 0;
    m_direction= 0;
    m_velocity = 0.0;
    m_acceleration= 0.0;
    m_model = 0;
    m_laneChange = 0;
    m_length = 0;
    m_width = 0;
    // Adicionado por Thales
    m_isAp = false;
    m_IdApAnt = 1;
    m_time = 0;
    m_time1Ap = 0;
    // Atributo que recebe a distância de segurança proporcional à velocidade
    m_distSegDin = 0.0;
    // testeaviso: todo veículo inicializa com a troca forçada "false". 
    m_trocaForcada = false;
    // Inicializa o objeto com valores inválidos.
    m_valObst.pos = 0;
	m_valObst.pista = 10;
    // Fim da adição
    
    // Adicionado por Fabiano
    
    // Fim da adição
  }

  Vehicle::~Vehicle()
  {
  }

  // Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
  /*void Vehicle::SetupWifi(const WifiHelper &wifi, const YansWifiPhyHelper &phy, const NqosWifiMacHelper &mac)
  {
    NetDeviceContainer d;
	NodeContainer n(m_node);
    d = wifi.Install(phy, mac, n);
    
    std::ostringstream oss;

    oss << "/NodeList/" << m_node->GetId()<< "/DeviceList/0/Mac/MacTx";
    Config::Connect (oss.str(), MakeCallback (&Vehicle::DevTxTrace, this));
    oss.str("");
    oss << "/NodeList/" << m_node->GetId()<< "/DeviceList/0/Mac/MacRx";
    Config::Connect (oss.str(), MakeCallback (&Vehicle::DevRxTrace, this));
    oss.str("");
    oss << "/NodeList/" << m_node->GetId()<< "/DeviceList/0/Phy/State/RxOk";
    Config::Connect (oss.str(), MakeCallback (&Vehicle::PhyRxOkTrace, this));
    oss.str("");
    oss << "/NodeList/" << m_node->GetId()<< "/DeviceList/0/Phy/State/RxError";
    Config::Connect (oss.str(), MakeCallback (&Vehicle::PhyRxErrorTrace, this));
    oss.str("");
    oss << "/NodeList/" << m_node->GetId()<< "/DeviceList/0/Phy/State/Tx";
    Config::Connect (oss.str(), MakeCallback (&Vehicle::PhyTxTrace, this));
    oss.str("");
    oss << "/NodeList/" << m_node->GetId()<< "/DeviceList/0/Phy/State/State";
    Config::Connect (oss.str(), MakeCallback (&Vehicle::PhyStateTrace, this));

    m_device = d.Get(0);
    m_device->SetReceiveCallback(MakeCallback(&Vehicle::ReceivePacket, this));
  }*/

  void Vehicle::SetDirection(int value)
  {
    m_direction=value;
  }

  int Vehicle::GetDirection()
  {
    return m_direction;
  }

  int Vehicle::GetVehicleId()
  {
    return m_vehicleId;
  }

  void Vehicle::SetVehicleId(int value)
  {
    m_vehicleId=value;
  }

  Ptr<Model> Vehicle::GetModel()
  {
    return m_model;
  }

  void Vehicle::SetModel(Ptr<Model> value)
  {
    m_model=value;
  }

  Ptr<LaneChange> Vehicle::GetLaneChange()
  {
    return m_laneChange;
  }

  void Vehicle::SetLaneChange(Ptr<LaneChange> value)
  {
    m_laneChange=value;
  }

  Vector Vehicle::GetPosition()
  {
    return m_node->GetObject<MobilityModel>()->GetPosition();
  }

  void Vehicle::SetPosition(Vector value)
  {
    m_node->GetObject<MobilityModel>()->SetPosition(value);
  }

  double Vehicle::GetLength()
  {
    return m_length;
  }

  void Vehicle::SetLength(double value)
  {
    if(value < 0) 
	  value=0;
    m_length=value;
  }

  double Vehicle::GetWidth()
  {
    return m_width;
  }

  void Vehicle::SetWidth(double value)
  {
    if(value < 0)
	  value=0;
    m_width=value;
  }

  double Vehicle::GetVelocity()
  {
    return m_velocity;
  }

  void Vehicle::SetVelocity(double value)
  {
    m_velocity=value;
  }

  double Vehicle::GetAcceleration()
  {
    return m_acceleration;
  }

  void Vehicle::SetAcceleration(double acc)
  {
    m_acceleration=acc;
  }

  int Vehicle::GetLane()
  {
    return m_lane;
  }

  void Vehicle::SetLane(int value)
  {
    m_lane=value;
  }

  void Vehicle::Accelerate(Ptr<Vehicle> vwd)
  {	
    m_acceleration = Acceleration(vwd);
  }

  double Vehicle::Acceleration(Ptr<Vehicle> vwd)
  {
    return m_model->CalculateAcceleration(Ptr<Vehicle>(this), vwd);       
  }

  void Vehicle::TranslatePosition(double dt)
  {
    Vector v = this->GetPosition();
    v.x += dt * m_velocity * m_direction;
    this->SetPosition(v);
  }

  void Vehicle::TranslateVelocity(double dt)
  {
    m_velocity += (m_acceleration * dt);
    if(m_velocity<=0.0)
      {
	    m_velocity=0.0;
      }
  }

  bool Vehicle::CheckLaneChange(Ptr<Vehicle> frontOld, Ptr<Vehicle> frontNew, Ptr<Vehicle> backNew, bool toLeft)
  {
    return m_laneChange->CheckLaneChange(Ptr<Vehicle>(this), frontOld, frontNew, backNew, toLeft);
  }

  bool Vehicle::Compare(Ptr<Vehicle> v1, Ptr<Vehicle> v2)
  {
    if(v1->GetDirection() == 1)
      {
	    if (v1->GetPosition().x > v2->GetPosition().x) return true;
	    else return false;
      }
    else
      {
	    if (v1->GetPosition().x < v2->GetPosition().x) return true;
	    else return false;
      }
  }

// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
/*
  void Vehicle::DevTxTrace (std::string context, Ptr<const Packet> p)
  {
    if(!m_devTxTrace.IsNull()) 
	  m_devTxTrace(Ptr<Vehicle>(this), context, p);
  }

  void Vehicle::DevRxTrace (std::string context, Ptr<const Packet> p)
  {
    if(!m_devRxTrace.IsNull()) 
	  m_devRxTrace(Ptr<Vehicle>(this), context, p);
  }

  void Vehicle::PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
  {
    if(!m_phyRxOkTrace.IsNull()) 
      m_phyRxOkTrace(Ptr<Vehicle>(this), context, packet, snr, mode, preamble);
  }	

  void Vehicle::PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
  {
    if(!m_phyRxErrorTrace.IsNull()) 
	  m_phyRxErrorTrace(Ptr<Vehicle>(this), context, packet, snr);
  }

  void Vehicle::PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
  {
    if(!m_phyTxTrace.IsNull()) 
	  m_phyTxTrace(Ptr<Vehicle>(this), context, packet, mode, preamble, txPower);
  }

  void Vehicle::PhyStateTrace (std::string context, Time start, Time duration, enum WifiPhy::State state)
  {
    if(!m_phyStateTrace.IsNull()) 
	  m_phyStateTrace(Ptr<Vehicle>(this), context, start, duration, state);
  }

  bool Vehicle::ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet,uint16_t protocol,const Address& address)
  {
    if(!m_receive.IsNull()) 
	  m_receive(Ptr<Vehicle>(this), packet, address);
    return true;
  }

  Address Vehicle::GetAddress()
  {
    return m_device->GetAddress();
  }

  Address Vehicle::GetBroadcastAddress()
  {
    return m_device->GetBroadcast();
  }

  bool Vehicle::SendTo(Address address, Ptr<Packet> packet)
  {
    return m_device->Send(packet, address, 1);
  }

  VehicleReceiveCallback Vehicle::GetReceiveCallback()
  {
    return m_receive;
  }

  void Vehicle::SetReceiveCallback(VehicleReceiveCallback receive)
  {
    m_receive = receive;
  }

  DeviceTraceCallback Vehicle::GetDevTxTraceCallback()
  {
    return m_devTxTrace;
  }

  void Vehicle::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace)
  {
    m_devTxTrace = devTxTrace;
  }

  DeviceTraceCallback Vehicle::GetDevRxTraceCallback()
  {
    return m_devRxTrace;
  }
  
  void Vehicle::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace)
  {
    m_devRxTrace = devRxTrace;
  }

  PhyRxOkTraceCallback Vehicle::GetPhyRxOkTraceCallback()
  {
    return m_phyRxOkTrace;
  }

  void Vehicle::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace)
  {
    m_phyRxOkTrace = phyRxOkTrace;
  }

  PhyRxErrorTraceCallback Vehicle::GetPhyRxErrorTraceCallback()
  {
    return m_phyRxErrorTrace; 
  }

  void Vehicle::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace)
  {
    m_phyRxErrorTrace = phyRxErrorTrace;
  }

  PhyTxTraceCallback Vehicle::GetPhyTxTraceCallback()
  {
    return m_phyTxTrace;
  }

  void Vehicle::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace)
  {
    m_phyTxTrace = phyTxTrace;
  }

  PhyStateTraceCallback Vehicle::GetPhyStateTraceCallback()
  {
    return m_phyStateTrace;
  }

  void Vehicle::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace)
  {
    m_phyStateTrace = phyStateTrace;
  }*/
  
  //Adicionado por Thales
  void Vehicle::SetIsAp(bool isAp){
	m_isAp = isAp;
  }
  
  bool Vehicle::GetIsAp(){  
	return m_isAp;  
  }
  
  void Vehicle::SetIdApAnt(int idApAnt){
	  m_IdApAnt = idApAnt;
  }
  
  int Vehicle::GetIdApAnt(){
      return m_IdApAnt;
  }
  
  void Vehicle::SetTime(double time){
	  m_time = time;
  }
  
  double Vehicle::GetTime(){
      return m_time;
  }
  
  void Vehicle::SetTime1Ap(double time1Ap){
	  m_time1Ap = time1Ap;
  }
  
  double Vehicle::GetTime1Ap(){
      return m_time1Ap;
  }
  // Fim da adição
  
  
  //Adicionado por Fabiano
  // newttl
  //void Vehicle::UpdateTable(std::list<Ptr<Table> > table, int trecho, double decrem){
  void Vehicle::UpdateTable(std::list<Ptr<Table> > table, int trecho, int trechoProp, int trechoIniDir, int trechoFimDirCont, int treFimDir, double decrem, double ttlTreAtual){
	
	std::list<Ptr<Table> >::iterator it1, it2;
	it2 = table.begin();
	
	// variancia
	double condAnterior;
	
	// Verifica se é Ap para calcular a Média Harmônica antes de atualizar a tabela
	if(this->m_isAp)
	{
		// Percorre toda a tabela e atualiza onde o TTL for menor que o TTL da tabela recebida
		for(it1 = this->m_table.begin(); it1!= this->m_table.end(); it1++)
		{
			/* TTL: decrementa o TTL da tabela do veículo pelo tempo necessário desde a última atualização 
			 * antes de atualizar a tabela do AP (somente nos trechos onde o veículo não está). */
			if ((*it2)->GetTrecho() != trecho)
			{
				if (((*it2)->GetTtl() - decrem) > 0.0)
					(*it2)->SetTtl((*it2)->GetTtl() - decrem);
				else
					(*it2)->SetTtl(0.0);
					
			}	
			
			/* cronometro: Controla o tempo de validade da informação, para que informações com TTL muito alto não 
			 * se mantenham válidas por muito tempo sem serem atualizadas. Após o cronômetro chegar a zero, zera 
			 * o TTL. O tempo de validade da informação é decrementado de cada trecho pelo tempo necessário desde a 
			 * última atualização antes de atualizar a tabela do AP. Só decrementa nos trechos da direção do 
			 * veículo. Não mexe no tempo de validade de trechos de outra direção.  */
			if (((*it2)->GetTrecho() >= trechoIniDir) and ((*it2)->GetTrecho() <= treFimDir) and ((*it2)->GetTrecho() != trecho))
			{
				if (((*it2)->GetCronometro() - decrem) > 0.0)
					(*it2)->SetCronometro((*it2)->GetCronometro() - decrem);
				else
					(*it2)->SetCronometro(0.0);					
			}	
			
			/* newttl: Esta condição serve para, além de não deixar sobreescrever as informações recebidas pelo
			 * Ap por um veículo de uma determinada direção após receber as informações de um veículo de outra
			 * direção, já que o TTL dos trechos é zero (mesmo aqueles com condição já calculada), também permite
			 * que só atualize o Ap do trecho onde o veículo está para trás, pois é onde o veículo possui seguramente
			 * (exceto em caso de ser ultrapassado) informações mais recentes. Poderia gerar o questionamento: mas 
			 * corre o risco de deixar de atualizar trechos onde o TTL do Ap está zerado, devendo atualizar tudo. 
			 * Se nos trechos à frente do veículo, o TTL do Ap está zerado, seguramente no veículo também estará.*/
			if((*it1)->GetTtl() == 0.0)
			{
				
				if(((*it1)->GetTrecho() <= trecho) and ((*it1)->GetTrecho() >= trechoIniDir))
				{				  
					/* variancia: Caso seja a velocidade anterior do trecho seja -1, então asinda não houve nenhuma 
					 * atualização de condição neste trecho, e assim não há velocidade anterior. Deste modo, guarda 
					 * a velocidade enviada pelo veículo como anterior. Caso não seja, guarda a velocidade que está 
					 * no Ap como anterior.*/
					if(((*it1)->GetSpeedAnt() == -1))
						condAnterior = (*it2)->GetSpeed();
					else
						condAnterior = (*it1)->GetSpeed();
						
					// variancia // newttl
					/* Como o TTL do Ap é zero, o TTLMax será o próprio TTL definido pelo veículo, uma vez que, qualquer
					 * que seja seu valor, será suficiente para substituir a informação obsoleta sobre o trecho atual
					 * nos trechos anteriores, uma vez que se o TTL expirou aqui, certamente também expirou nos trechos 
					 * anteriores, não correndo o risco de deixar de atualizar devido ao TTL ser menor, mesmo com
					 * a informação sendo mais atualizada. É necessário enviar o TTL do trecho atual (ttlTreAtual) do veículo
					 * para definir o TTL máximo, pois se dermos um (*it2)->GetTtl() para esta finalidade, quando o TTL for 
					 * decrementado, o TTL máximo também será. A definição do TTL máximo é feita somente no trecho atual.*/
					if((*it1)->GetTrecho() == trecho)
						(*it1)->Update((*it2)->GetTrecho(), (*it2)->GetSpeed(), condAnterior, (*it2)->GetTtl(), ttlTreAtual, (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
					else	
						(*it1)->Update((*it2)->GetTrecho(), (*it2)->GetSpeed(), condAnterior, (*it2)->GetTtl(), (*it2)->GetTtlMax(), (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
				}
				else if (((*it1)->GetTrecho() >= trechoProp) and ((*it1)->GetTrecho() <= trechoFimDirCont))
				{
					// variancia // newttl
					(*it1)->Update((*it2)->GetTrecho(), (*it2)->GetSpeed(), (*it2)->GetSpeedAnt(), (*it2)->GetTtl(), (*it2)->GetTtlMax(), (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
				}
			}
			// fim
			
			// newttl
			/* Entra na condição ou se o TTL do Ap for menor que o enviado pelo veículo, ou mesmo se for maior, mas 
			 * estiver passando pelo trecho em que o veículo se encontra (caso o TTL do Ap seja maior que o do veículo
			 * em seu próprio trecho, o que manteria a condição obsoleta neste trecho, o que não é correto, já que 
			 * seguramente o veículo possui informações mais atuais em seu próprio trecho.*/
			else if(((*it1)->GetTtl() < (*it2)->GetTtl()) or ((*it1)->GetTrecho() == trecho))
			{	
				// variancia: guarda a velocidade atual como anterior antes de atualizar
				condAnterior = (*it1)->GetSpeed();
					
				// Caso seja o trecho onde o veículo está, calcula a média harmônica
				if ((*it1)->GetTrecho() == trecho)
				{						
					/* newttl: Caso TTL definido pelo veículo para o trecho em que está seja < que o maior 
					 * TTL já registrado, então seta o TTL como o maior já registrado. Necessário para que o TTL seja 
					 * grande o suficiente para que não corra o risco de informações obsoletas manterem-se sem atualizar
					 * devido à grandes TTLs calculados previamente.*/
					if(ttlTreAtual < (*it1)->GetTtlMax())
						(*it1)->Update((*it2)->GetTrecho(), (2/((1/(*it2)->GetSpeed())+(1/(*it1)->GetSpeed()))), condAnterior, (*it1)->GetTtlMax(), (*it1)->GetTtlMax(), (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
					/* newttl: Caso não, seta o TTL do trecho como o TTL definido pelo veículo com base nas velocidades
					 * médias dos trechos proporcionais + variância, e o salva também como o maior TTl já registrado.*/
					else
						(*it1)->Update((*it2)->GetTrecho(), (2/((1/(*it2)->GetSpeed())+(1/(*it1)->GetSpeed()))), condAnterior, ttlTreAtual, ttlTreAtual, (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
				}
				else
				{
					// variancia // newttl
					(*it1)->Update((*it2)->GetTrecho(), (*it2)->GetSpeed(), condAnterior, (*it2)->GetTtl(), (*it2)->GetTtlMax(), (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
				}
			}
			it2++;
		}
	}
	
	else
	{
		// Ap envia a TCT de volta para o veículo após o processo de atualização
		for(it1 = this->m_table.begin(); it1!= this->m_table.end(); it1++)
		{
			// variancia // newttl
			(*it1)->Update((*it2)->GetTrecho(), (*it2)->GetSpeed(), (*it2)->GetSpeedAnt(), (*it2)->GetTtl(), (*it2)->GetTtlMax(), (*it2)->GetObst(), (*it2)->GetCronometro(), (*it2)->GetHoraGrava());
			it2++;
		}
	}
  }
  
  void Vehicle::AddTable(Ptr<Table> table)
  {
	 std::list<Ptr<Table> >::iterator it; 
	 
	 // Percorre toda a tabela e verifica se o trecho já existe. Se sim, deleta e coloca o mais atualizado.
	 for(it = m_table.begin(); it!= m_table.end(); it++)
	 {
		 if((*it)->GetTrecho() == table->GetTrecho()){
			it = m_table.erase(it);
			m_table.insert(it, table);

			/* Necessário dar um break quando inserir a linha na posição correta da TCT para sair
			 * ... do loop, pois senão trava a simulação após 171 segundos.*/
			break;
		}
	 }
  }	  
 
  std::list<Ptr<Table> > Vehicle::GetTable()
  {	  
	  return m_table;	    
  }
  
  
  // Adicionado por Thales (cria TCTs inicial, com todos os trechos e valores zerados)
  /* Se não criar uma inicialização para as TCTs, o Ap não conseguirá realizar a comparação, 
   * ... uma vez que sua tabela estará vazia (sem nenhum valor para ser comparado).*/
  void Vehicle::SetTable(int qtdLinhas)
  {
	  // testeaviso: Aproveito o parâmetro "qtdLinhas" para inicializar o tamanho do array dinamicamente
	  this->m_obst = new Table::obstac[qtdLinhas];
	  Table::obstac obAux;
	  // testeaviso: Crio um objeto de aviso de obstáculo com valores inválidos para inicializar na TCT
	  obAux.pos = 0.0;
	  obAux.pista = 10;
	  for(int i = 1; i <= qtdLinhas; i++)
	  {	
		// variancia: inicia condição anterior com -1
		//this->m_table.push_back(CreateObject<Table>(i, 0.0, -1, 0.0, 0.0, obAux, 300.0, 0.0));
		/* newttl: Possui menos atributos (somente os que não possui valor padrão). Feito após a criação do 
		 * novo construtor com menos atributos, devido ao erro de não reconhecer novos atributos adicionados
		 * ao construtor padrão. */
		this->m_table.push_back(CreateObject<Table>(i, 0.0, 0.0, obAux, 0.0));
		// testeaviso: Inializando o array que armazena a posição e pista do obstáculo ultrapassado com valores inválidos
		this->m_obst[i-1].pos = 0.0;
		this->m_obst[i-1].pista = 10;
		// fim
	  }
  }
  
  // Fim da adição
  
  // Configura a distância de segurança proporcional à velocidade
  void Vehicle::SetDistSegDin(double value)
  {		
	  m_distSegDin = value;
  }
  
  // Retorna a distância de segurança proporcional à velocidade
  double Vehicle::GetDistSegDin()
  {		
	  return m_distSegDin;
  }
  
  // testeaviso: Usado para setar as informações do obstáculos recém descoberto
  void Vehicle::SetObstac(int trecho, double posic, int lane)
  {
	  m_obst[trecho-1].pos = posic;
	  m_obst[trecho-1].pista = lane;
  }
  
  // testeaviso: Retorna as informações do obstáculo recém descoberto (usado na hora de inserir a informação na TCT)
  Table::obstac Vehicle::GetObstac(int trecho)
  {		
	  return m_obst[trecho-1];
  }
  // fim
  
  // testeaviso: seta a troca forçada
  void Vehicle::SetTrocaForcada(bool value)
  {
	  m_trocaForcada = value;
  }
  
  // testeaviso: retorna se o veículo fez uma troca forçada
  bool Vehicle::GetTrocaForcada()
  {
	  return m_trocaForcada;
  }
  
  // testeaviso: Seta a posição e pista de um obstáculo que se encontra no trecho que o veículo vai entrar
  void Vehicle::SetValObst(Table::obstac value)
  {
	  m_valObst.pos = value.pos;
	  m_valObst.pista = value.pista;
  }
  
  // testeaviso: Sobrecarga para zerar os parâmetros quando o veículo ultrapassar a posição do obstáculo
  void Vehicle::SetValObst(double pos, int pista)
  {
	  m_valObst.pos = pos;
	  m_valObst.pista = pista;
  }
  
  // testeaviso: Retorna o objeto com a posição e pista do obstáculo no trecho em que o veículo vai entrar
  Table::obstac Vehicle::GetValObst()
  {
	  return m_valObst;
  }

}
