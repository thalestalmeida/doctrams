/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 Tabela para Atualizacao
 */

#ifndef CLASS_TABLE_
#define CLASS_TABLE_

  /**
  *
  * Tabela 
  */
#include "ns3/ptr.h"
#include "ns3/object.h"  

  class Table: public ns3::Object
  {
    private:
		int t_trecho; 
		double t_speed;
		// variancia
		double t_speedAnt;
		double t_ttl;
		double m_horaGrava;
		double t_ttlMax;
		double t_cronometro;
    
    public: 
		// Cria estrutura para armazenar a posição e pista de um possível obstáculo na TCT
		struct obstac
		{
			double pos;
			int pista;
		};     
		obstac t_obst;
 	    // newttl: construtor abaixo não funciona. Necessário criar outro construtor padrão recebendo menos atributos.
 	    Table(int trecho, double speed, double speedAnt, double ttl, double ttlMax, obstac obst, double cronometro, double horaGrava);
 	    Table(int trecho, double speed, double ttl, obstac obst, double horaGrava);
		Table();
	    ~Table();
		
		// newttl
		void Update(int trecho, double speed, double speedAnt, double ttl, double ttlMax, obstac obst, double cronometro, double horaGrava);
		int GetTrecho();
		void SetTrecho(int trecho);
		double GetSpeed();
		void SetSpeed(double speed);
		// variancia
		double GetSpeedAnt();
		void SetSpeedAnt(double speed);
		double GetTtl();
		void SetTtl(double ttl);
		double GetTtlMax();	
		void SetTtlMax(double cronometro);
		obstac GetObst();
		void SetObst (obstac obst);	
		double GetCronometro();
		void SetCronometro(double cronometro);	
		// variancia		
		double GetHoraGrava();
		void SetHoraGrava(double value);		
  };

#endif
