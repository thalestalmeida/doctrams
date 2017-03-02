/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 Tabela para atualizacao
 */

#include "Table.h"
  
  // Alterado por Thales (alterado o nome dos parâmetros pois estava dando conflito por ter o mesmo nome dos atributos)
  // newttl
  Table::Table(int trecho, double speed, double speedAnt, double ttl, double ttlMax, Table::obstac obst, double cronometro, double horaGrava)
  {
     this->t_trecho = trecho;
	 this->t_speed = speed;
	 // variancia
	 this->t_speedAnt = speedAnt;
	 this->t_ttl = ttl;
	 this->t_ttlMax = ttlMax;
	 this->t_obst.pos = obst.pos;
	 this->t_obst.pista = obst.pista;
	 this->t_cronometro = cronometro;
	 this->m_horaGrava = horaGrava;
  }
  
  Table::Table(int trecho, double speed, double ttl, Table::obstac obst, double horaGrava)
  {
     this->t_trecho = trecho;
	 this->t_speed = speed;
	 // variancia
	 this->t_speedAnt = -1;
	 this->t_ttl = ttl;
	 this->t_ttlMax = 0.0;
	 this->t_obst.pos = obst.pos;
	 this->t_obst.pista = obst.pista;
	 this->t_cronometro = 300.0;
	 this->m_horaGrava = horaGrava;
  }
  
  Table::Table()
  {
     this->t_trecho = 0;
	 this->t_speed = 0.0;
	 // variancia
	 this->t_speedAnt = 0.0;
	 this->t_ttl = 0.0;
	 this->t_ttlMax = 0.0;
	 this->t_obst.pos = 0.0;
	 this->t_obst.pista = 10;
	 this->t_cronometro = 300.0;
	 this->m_horaGrava = 0.0;
  } 
  
   Table::~Table()
  {
     this->t_trecho = 0;
	 this->t_speed = 0.0;
	 // variancia
	 this->t_speedAnt = 0.0;
	 this->t_ttl = 0.0;
	 this->t_ttlMax = 0.0;
	 this->t_obst.pos = 0.0;
	 this->t_obst.pista = 0;
	 this->t_cronometro = 0.0;
	 this->m_horaGrava = 0.0;
  }
	
  // newttl
  void Table::Update(int trecho, double speed, double speedAnt, double ttl, double ttlMax, Table::obstac obst, double cronometro, double horaGrava)
  {
	  this->t_trecho = trecho;
	  this->t_speed = speed;
	  // variancia
	  this->t_speedAnt = speedAnt;
	  this->t_ttl = ttl;
	  this->t_ttlMax = ttlMax;
	  this->t_obst.pos = obst.pos;
	  this->t_obst.pista = obst.pista;
	  this->t_cronometro = cronometro;
	  this->m_horaGrava = horaGrava;
  }
  
  int Table::GetTrecho()
  {
	  return t_trecho;
  }
  
  void Table::SetTrecho(int trecho)
  {
	  t_trecho = trecho;
  }
  
  double Table::GetSpeed()
  {
	  return t_speed;
  }
  
  void Table::SetSpeed(double speed)
  {
	  t_speed = speed;
  }
  
  // variancia
  double Table::GetSpeedAnt()
  {
	  return t_speedAnt;
  }
  
  void Table::SetSpeedAnt(double speedAnt)
  {
	  t_speedAnt = speedAnt;
  }
  
  double Table::GetTtl()
  {
	  return t_ttl;
  }	
  
  void Table::SetTtl(double ttl)
  {
	  t_ttl = ttl;
  }
  
  double Table::GetTtlMax()
  {
	  return t_ttlMax;
  }	
  
  void Table::SetTtlMax(double ttlMax)
  {
	  t_ttlMax = ttlMax;
  }
  
  Table::obstac Table::GetObst()
  {
	  return t_obst;
  }
  
  void Table::SetObst(Table::obstac obst)
  {
	  this->t_obst.pos = obst.pos;
	  this->t_obst.pista = obst.pista;
  }
  
  double Table::GetCronometro()
  {
	  return t_cronometro;
  }
  
  void Table::SetCronometro(double cronometro)
  {
	  t_cronometro = cronometro;
  }
  
  double Table::GetHoraGrava()
  {
	  return m_horaGrava;
  }
  
  void Table::SetHoraGrava(double value)
  {
	  m_horaGrava = value;
  }
