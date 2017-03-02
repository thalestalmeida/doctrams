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
 * GNU General Public License for 7more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 */

/*
		This the starting point of the simulation and experiments.
		The main function will parse the input and parameter settings.
		Creates a highway and set the highway parameters. then bind the events (callbacks)
		to the created controller and designed handlers. Sets the highway start and end time,
		and eventually runs the simulation which is basically running a highway with a controller.
		You can add your functions to controller to create various scenarios. 
 */

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <typeinfo>
#include <time.h>
#include <map>
#include <algorithm>
#include "math.h"
#include "limits.h"
#include "ns3/core-module.h"
#include "ns3/common-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/mobility-module.h"
#include "ns3/contrib-module.h"
#include "ns3/wifi-module.h"
#include "Highway.h"
#include "Controller.h"

NS_LOG_COMPONENT_DEFINE("HADI");

using namespace ns3;
using namespace std;


struct classcomp {

	bool operator() (pair<int, double > s1, pair<int, double > s2) const {
		return s1.first < s2.first;
	}
};

// Alterado por Thales (alterado o tipo do 2º parâmetro do par de int para double)
// Necessário para gravar o tempo que o veículo passou pelo AP com mais precisão em serverlog
std::map<int, std::set<pair<int, double >, classcomp > > mapVehicles;

static void Start(Ptr<Highway> highway) {
	highway->Start();
}

static void Stop(Ptr<Highway> highway) {
	highway->Stop();
}

std::list<std::string > vehListData;
stringstream dataString;
ofstream dataFile;
ifstream readDataFile, readDataFile2;
time_t start,end, serv, init_t;
struct tm * timeinfo;
int apInitDist = 500;
int apDist = 500;
int apQtd = 19;
int numOfLanes = 3;
int medianGap = 5;
int injectionGap = 52;
int injectionMixValue = 83;
bool setTwoDirectional = true;
int highwayLength = 10000;
float simTime = 7340.0;
float sedanVelocity = 25;
float truckVelocity = 20; 
int t_vid;
double t_posX;
// Adicionado por Thales
int obInitDist = 250;
int obDist = 2375;	
int obQtd = 1;
bool obAleat = false;
bool obstacle = true;
double raio = 5000;
int trechos = 2*(apQtd-1);    
// Adicionado por Thales
/* Ponteiro para variável double que armazena vel. média em tempo real em trechos Leste e Oeste
 * ... (usada na comparação da TCT). É preciso ser um ponteiro pois não é possível definir o tamanho
 * ... como "2*(apQtd-1)" em arrays comuns, pois o valor tem que ser constante e conhecido antes de
 * ... compilação. É necessário ser um vetor para que a velocidade em um trecho não seja sobreposta
 * ... pelo trecho seguinte, antes que seja assinalada no arquivo, como estava acontecendo anteriormente.
 * O tamanho de "2*(apQtd-1)" faz com que nas posições de 0 a 8 sejam as velocidades de trechos Leste, 
 * ... e de 9 a 17 sejam as velocidades de trechos Oeste.*/
// Fim da adição
double *VMRT  = new double [trechos];
double *VMRT2  = new double [trechos];
double contErroGlobal = 0;
double qtdTrePercGlobal = 0;
double contErroTrecho = 0;
double qtdTrePercTrecho = 0;
double contErroTreVizinho = 0;
double qtdTrePercTreVizinho = 0;
double contErroMesmaDir = 0;
double qtdTrePercMesmaDir = 0;
double qtdTrePercMesmaDirFrente1 = 0;
double contErroMesmaDirFrente1 = 0;
double qtdTrePercMesmaDirFrente2 = 0;
double contErroMesmaDirFrente2 = 0;
double qtdTrePercMesmaDirFrente3 = 0;
double contErroMesmaDirFrente3 = 0;  
double *pcontErroGlobal  = new double [trechos];
double *pqtdTrePercGlobal = new double [trechos];
double *pcontErroTrecho  = new double [trechos];
double *pqtdTrePercTrecho = new double [trechos];
double *pcontErroTreVizinho  = new double [trechos];
double *pqtdTrePercTreVizinho = new double [trechos];
double *pcontErroMesmaDir  = new double [trechos];
double *pqtdTrePercMesmaDir = new double [trechos];
double *pqtdTrePercMesmaDirFrente1  = new double [trechos];
double *pcontErroMesmaDirFrente1 = new double [trechos];
double *pqtdTrePercMesmaDirFrente2  = new double [trechos];
double *pcontErroMesmaDirFrente2 = new double [trechos];
double *pqtdTrePercMesmaDirFrente3  = new double [trechos];
double *pcontErroMesmaDirFrente3 = new double [trechos];
//double tempVehCruzouVia = simTime;
double *ptempMedCruzTre  = new double [trechos];
double *pcontVehCruzTre  = new double [trechos];
//double *pTtl  = new double [trechos];
double contErroMesmaDirFrente = 0;
double qtdTrePercMesmaDirFrente = 0;
double *pcontErroMesmaDirFrente  = new double [trechos];
double *pqtdTrePercMesmaDirFrente = new double [trechos];
double contErroTreVizDist = 0;
double qtdTrePercTreVizDist = 0;
double *pcontErroTreVizDist  = new double [trechos];
double *pqtdTrePercTreVizDist = new double [trechos];
double *freqInsObs = new double [obQtd];
double freqRemObs;
int y = 0;
int z = 0;
int *contObst = new int [2*apQtd];
double *maiorVelTre  = new double [trechos];
double *menorVelTre  = new double [trechos];
double *maiorVelTre2  = new double [trechos];
double *menorVelTre2  = new double [trechos];
double contErroFant = 0;
double qtdTrePercFant = 0;
double *pqtdTrePercFant  = new double [trechos];
double *pcontErroFant = new double [trechos];
// controle
bool obDinamico = true;
bool avaliaPosEstab = true;
double horaAvaliar = 0;
bool snapshotPeriod = false;
bool avisoObst = true;
// 1obdin
double tempoInsObs = 2900;
double tempoRemObs = 3080;
// newestab
double tempoViaEstab = simTime;
// propInativo
bool baixaDensidade = true;
// Fim da adição




// Retorna o trecho referente à posição X de um obstáculo
int GetTrechoObst (int posOb, int dirOb)
{
	int trechoOb;
	
	// Se direção do obstáculos == 1
	if (dirOb == 1)
	{
		// Calcula desta maneira
		trechoOb = floor((posOb - apInitDist) / apDist) + 1;
		if (trechoOb < 0 || trechoOb >= apQtd)
			trechoOb = 0;
	}
	// Se direção == -1
	else
	{
		// Calcula desta maneira
		trechoOb = floor((posOb - apInitDist) / apDist) + 1;
		if (trechoOb < 0 || trechoOb >= apQtd)
			trechoOb = 0;
		else
			trechoOb = ((apQtd-1) + (apQtd - trechoOb));
	}
	
	return trechoOb;
}



// Função que inicializa os arrays "taxa de erro Global" e "linhas válidas Global" com zero
void initPointArray (double *qtdErro[], double *qtdLinhasVal[], int tam)
{
	for (int i = 0; i < tam; i++)
	{
		(*qtdErro)[i] = 0;
		(*qtdLinhasVal)[i] = 0;
	}
}





// Sobrecarga da função que inicializa o array de inteiros que guarda a quantidade de obstáculos em cada trecho
void initPointArray (int *qtdObTre[], int tam)
{
	for (int i = 0; i < tam; i++)
		(*qtdObTre)[i] = 0;
}



// Função que inicializa o array de inteiros que guarda a maior e menor velocidade do trecho
void initPointArrayVarVel (double *maiorVelTre[], double *menorVelTre[], int tam)
{
	for (int i = 0; i < tam; i++)
	{
		(*maiorVelTre)[i] = -1;
		(*menorVelTre)[i] = -1;
		//cout <<"\nPasseui aqui" <<endl;
	}
}





// Função que calcula a taxa de acerto para cada trecho
void compTaxaAcerto (double *qtdErro[], double *qtdLinhasVal[], int tam, string comp)
{
	stringstream aux5;
	ofstream aux6;
	aux5 << "data/compTaxaAceTre/" << comp;
	aux6.open(aux5.str().c_str(),ios::app);
	aux6 << "Trecho	" << "Acerto" <<endl;
	for (int i = 0; i < tam; i++)
	{
		(*qtdErro)[i] = 100 - (((*qtdErro)[i]*100) / (*qtdLinhasVal)[i]);
		aux6 << i+1 << "	" << (*qtdErro)[i] << endl;
	}
	aux5.str("");
	aux6.close();
}




int flag1 = 0;
// Função que calcula a taxa de acerto
void compTaxaAcerto (double qtdErro, double qtdLinhasVal, string comp)
{
	ofstream acerto;
	acerto.open("data/servidor/acerto.log",ios::app);
	double percAcerto =  100 - ((qtdErro*100) / qtdLinhasVal);
	cout << "Percentual de acerto " << comp << ": " << fixed << setprecision(2) << percAcerto << "%" << endl;
	// Cria cabeçalho apenas 1x
	if(flag1 == 0)
	{
		acerto << "Taxa	" << "Acerto	" <<endl;
		flag1 = 1;
	}
	acerto << comp << "	" << percAcerto <<endl;
	acerto.close();
}





// Chama a função que gera a taxa de acerto da descentralizada
//void verTaxaAcerto(Ptr<Highway> highway, double tempo)
void verTaxaAcerto(Ptr<Highway> highway)
{
	//cout << "\nTaxa de Acertos no tempo " << tempo << ":" << endl;
	cout << "\nTaxa de Acertos: " << endl;
	compTaxaAcerto(contErroGlobal, qtdTrePercGlobal, "Global");
	compTaxaAcerto(contErroTrecho, qtdTrePercTrecho, "\"Trecho Atual\"");
	compTaxaAcerto(contErroTreVizinho, qtdTrePercTreVizinho, "Vizinhos");
	compTaxaAcerto(contErroTreVizDist, qtdTrePercTreVizDist, "Distantes");
	compTaxaAcerto(contErroMesmaDir, qtdTrePercMesmaDir, "\"Mesma Direção\"");
	compTaxaAcerto(contErroMesmaDirFrente, qtdTrePercMesmaDirFrente, "\"Mesma Direção (Frente)\"");
	compTaxaAcerto(contErroMesmaDirFrente1, qtdTrePercMesmaDirFrente1, "\"Mesma Direção (5 km à frente)\"");
	compTaxaAcerto(contErroMesmaDirFrente2, qtdTrePercMesmaDirFrente2, "\"Mesma Direção (2.5 km à frente)\"");
	compTaxaAcerto(contErroMesmaDirFrente3, qtdTrePercMesmaDirFrente3, "\"Mesma Direção (1.25 km à frente)\"");
	compTaxaAcerto(contErroFant, qtdTrePercFant, "Fantasma");
	cout << "Quantidade de veículos que passaram pela via: " << highway->GetLastVehicleId() - highway->GetFirstIdVeh() <<endl;        
	compTaxaAcerto (&pcontErroGlobal, &pqtdTrePercGlobal, trechos, "GlobalTrecho");
	compTaxaAcerto (&pcontErroTrecho, &pqtdTrePercTrecho, trechos, "TrechoTrecho");
	compTaxaAcerto (&pcontErroTreVizinho, &pqtdTrePercTreVizinho, trechos, "VizinhosTrecho");
	compTaxaAcerto (&pcontErroTreVizDist, &pqtdTrePercTreVizDist, trechos, "VizinhosDistantes");
	compTaxaAcerto (&pcontErroMesmaDir, &pqtdTrePercMesmaDir, trechos, "MesmaDirTrecho");
	compTaxaAcerto (&pcontErroMesmaDirFrente, &pqtdTrePercMesmaDirFrente, trechos, "MesmaDirFrenteTrecho");
	compTaxaAcerto (&pcontErroMesmaDirFrente1, &pqtdTrePercMesmaDirFrente1, trechos, "MesmaDir5Trecho");
	compTaxaAcerto (&pcontErroMesmaDirFrente2, &pqtdTrePercMesmaDirFrente2, trechos, "MesmaDir2Trecho");
	compTaxaAcerto (&pcontErroMesmaDirFrente3, &pqtdTrePercMesmaDirFrente3, trechos, "MesmaDir1Trecho");
	compTaxaAcerto (&pcontErroFant, &pqtdTrePercFant, trechos, "Fantasma");
	if(snapshotPeriod)
	{
		// Zera as variáveis (necessário quando a divlgação ocorrer à cada X minutos)
		contErroGlobal = qtdTrePercGlobal = contErroTrecho = qtdTrePercTrecho =	contErroTreVizinho = qtdTrePercTreVizinho =
		contErroMesmaDir = qtdTrePercMesmaDir = qtdTrePercMesmaDirFrente1 = contErroMesmaDirFrente1 = qtdTrePercMesmaDirFrente2 = 
		contErroMesmaDirFrente2 = qtdTrePercMesmaDirFrente3 = contErroMesmaDirFrente3 = 0;
		initPointArray (&pcontErroGlobal, &pqtdTrePercGlobal, trechos);
		initPointArray (&pcontErroTrecho, &pqtdTrePercTrecho, trechos);
		initPointArray (&pcontErroTreVizinho, &pqtdTrePercTreVizinho, trechos);
		initPointArray (&pcontErroMesmaDir, &pqtdTrePercMesmaDir, trechos);
		initPointArray (&pcontErroMesmaDirFrente1, &pqtdTrePercMesmaDirFrente1, trechos);
		initPointArray (&pcontErroMesmaDirFrente2, &pqtdTrePercMesmaDirFrente2, trechos);
		initPointArray (&pcontErroMesmaDirFrente3, &pqtdTrePercMesmaDirFrente3, trechos);
		initPointArray (&pcontErroFant, &pqtdTrePercFant, trechos);
	}
}





static void ServerData(Ptr<Highway> highway) {

	// Gera a hora que terminou a simulação
	time(&end);
	timeinfo = localtime(&end);
	cout << "\nFim da Simulacao: " << asctime(timeinfo);	
	// Calcula o tempo de simulação
	int horas, minutos, segundos;	
	segundos = difftime(end, start);
	horas = segundos/3600;
	minutos = (segundos - horas*3600)/60;
	segundos = (segundos - horas*3600 - minutos*60);	
	cout << "Tempo de simulacao: " << setw(2) << horas << cout.fill('0') << ":" << setw(2) << minutos << cout.fill(0) 
	<<  ":" << setw(2) <<  segundos << cout.fill(0) << endl;
	
	
	// Chama a função que verifica as taxas de acerto ao final da simulação
	verTaxaAcerto(highway);
	

	// Imprime o total de veículos em circulação na via 
	std::list<Ptr<Vehicle> > lista1, lista2;
	lista1 = highway->FindVehiclesInSegment(0, highwayLength, 1);
	lista2 = highway->FindVehiclesInSegment(0, highwayLength, -1);
	cout << "\nTotal de veículos na via: " << (lista1.size() + lista2.size()) <<endl;
	
	
	// Exporta um arquivo com a variação de velocidade (menor e maior) de cada trecho
	ofstream maiorMenorVel;
	maiorMenorVel.open("data/MaiorMenorVelTre/maiorMenorVel.log", ios::app);
	for(int i = 0; i < trechos; i++)
	{
		// Controle para criar o cabeçalho somente 1x
		if(i==0)
		{
			maiorMenorVel  	<< "Trecho	"
							<< "Máxima	"
							<< "Mínima	"
							<< endl;
		}
		maiorMenorVel  	<< i+1 <<"	"
						<< fixed << setprecision(2) << maiorVelTre[i] * 3.6 <<"	"
						<< fixed << setprecision(2) << menorVelTre[i] * 3.6 
						<< endl;
	}
	maiorMenorVel.close();
	
	
	// Geração de gráficos via gnuplot
	stringstream aux, aux2;
	ifstream ReadFile;
	std::string aux4;
	int aux3;
	// Exporta para um arquivo o resultado do comando que coleta a quantidade de arquivos no diretório
	system("ls graficos/*.txt | wc -l > graficos/Diversos/numArquivos");
	// Lê esse arquivo
	ReadFile.open("graficos/Diversos/numArquivos");
	if (ReadFile.is_open())
	{
	  // Coleta o valor gravado no arquivo e grava na string
	  getline(ReadFile, aux4);
	  if (aux4 != "")
	  {
		// A stringstream recebe o valor da string
		aux2 << aux4;
		// Passa o valor para a variável inteira a ser usada no FOR
		aux2 >> aux3;
	  }
	}
	ReadFile.close();
	// O for tem que ter uma iteração para cada arquivo que carrega um gráfico
	for (int i = 1; i <= aux3; i++)
	{
		aux << "gnuplot graficos/load_grafico" <<i <<".txt 2>/dev/null";
		/* Gera gráficos de: 1-Taxa de acerto GlobalTre; 2-Taxa de acerto TrechoTre; 3-Taxa de acerto VizinhosTre;
		 * 4-Maior e menor velocidade de cada trecho; 5-Taxas de acerto diversas.*/
		system(aux.str().c_str());
		aux.str("");
		aux2.str("");
		
	}

	
	// Geração do server.log
	stringstream s;	
	ofstream file;
	file.open("data/servidor/server.log");
	// Adicionado por Thales (cria cabeçalho para arquivo de saída que grava o instante que o veículo passou pelo Ap)
	file <<"AP	"
		 <<"Veículo	"
		 <<"Tempo"
		 << endl;
	// Fim da adição
	for (std::map<int, set<pair<int, double>, classcomp > >::iterator i = mapVehicles.begin(); i != mapVehicles.end(); ++i) {
		for (std::set<pair<int, double >, classcomp >::iterator j = i->second.begin(); j != i->second.end(); ++j) {
			file << i->first 
				 << "	" << j->first 
				 << "	" << j->second 
				 << endl;
		}
	}       
	
		
	file.close();
	char output[30];
	time_t rawtime;
	struct tm *timeinfo;
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(output, 30, "%H:%M:%S.%Y-%m-%d", timeinfo);
	
	dataString  << "Data=" << output << "_SimTime=" << simTime << "_QtdAP=" << apQtd << "_DistIniAp=" << apInitDist 
				<< "_DistAp=" << apDist << "_HighwayLen=" << highwayLength << "_NumLanes=" << numOfLanes << "_InjGap=" 
				<< injectionGap << "_InjMixVal=" << injectionMixValue << "_Obst=" << obstacle; 
	dataFile.open("last_sim.opt");
	dataFile << dataString.str().c_str() << endl;
	dataFile.close();
	system(s.str().c_str());
}



// Snapshot da taxa de acerto global à cada X minutos
double contErroGlobal2 = 0;
double qtdTrePercGlobal2 = 0;
int contveic = 0;

void snapshot (Ptr<Highway> highway, double tempo)
{
	Ptr<Vehicle> tempVeh;
	std::list<Ptr<Table> > tabela;
	std::list<Ptr<Table> >::iterator it2;
	/* Exporta à cada X segundos todos os veículos em circulação na via, mostrando sua velocidade 
	 * e a respectiva distância de segurança proporcional à esta velocidade*/
	ofstream teste;
	teste.open("data/servidor/teste.log",ios::app );
		
	// Conta até -1 pois ainda não deu tempo de inserir o último veículo criado no array, retornando NULL
	for(int i = highway->GetFirstIdVeh(); i <= highway->GetLastVehicleId()-1; i++)
	{		
		tempVeh = highway->FindVehicle(i);			
		//if (tempVeh != NULL and ((tempVeh->GetTime() >= tempVehCruzouVia) and (avaliaPosEstab)))
		// newestab
		//if (tempVeh != NULL and ((tempVeh->GetTime() >= tempoViaEstab) and (avaliaPosEstab)))
		if (tempVeh != NULL)
		{
			// Conta a quantidade total de veículos em circulação
			contveic += 1;
			tabela = tempVeh->GetTable();
			for(it2 = tabela.begin(); it2!= tabela.end(); it2++)
			{				
				// Verifica se a linha da tabela está com condição de erro
				if ((*it2)->GetTtl() != 0.0)
				{				
					if ((*it2)->GetSpeed() < 5.0)
					{
						if (VMRT[(*it2)->GetTrecho()-1] * 3.6 >= 5.0)
							contErroGlobal2+=1;
					}
					
					// Mudar para 30 se velocidades máximas forem 70 e 60	
					else if ((*it2)->GetSpeed() < 40.0)
					{
						if ((VMRT[(*it2)->GetTrecho()-1] * 3.6 < 5.0) || (VMRT[(*it2)->GetTrecho()-1] * 3.6 >= 40.0))
							contErroGlobal2+=1;
					}
					
					// Mudar para 60 se velocidades máximas forem 70 e 60	
					else if ((*it2)->GetSpeed() < 80.0)
					{
						if ((VMRT[(*it2)->GetTrecho()-1] * 3.6 < 40.0) || (VMRT[(*it2)->GetTrecho()-1] * 3.6 >= 80.0))
							contErroGlobal2+=1;
					}
					else 
					{	
						if (VMRT[(*it2)->GetTrecho()-1] * 3.6 < 80.0)
							contErroGlobal2+=1;
					}
					qtdTrePercGlobal2+=1;
				}
				
				// Grava em arquivo a velocidade e distância de segurança de cada veículo
				teste << fixed << setprecision(2) << tempVeh->GetVelocity()   <<"	"
												  << tempVeh->GetDistSegDin() <<"	"
												  <<endl;
			}
					
		}
	}
	teste.close();
	cout << "\nQuantidade de veículos: " << contveic <<endl;
	cout << "Quantidade de carros +1: " <<highway->getContCarros1() <<endl;
	cout << "Quantidade de carros -1: " <<highway->getContCarros2() <<endl;
	cout << "Quantidade de caminhões +1: " <<highway->getContCaminhoes1() <<endl;
	cout << "Quantidade de caminhões -1: " <<highway->getContCaminhoes2() <<endl;
	double percSnapshot = 100 - (contErroGlobal2*100 / qtdTrePercGlobal2);
	cout << "\nTaxa de Acertos Snapshot no tempo " << tempo << ": " << percSnapshot << "%" << endl;
	/* Necessário zerar as variáveis para que o snapshot sempre mostre a taxa de acerto com base nas 
	 * condições do momento em que foi chamado, nunca acumulando as estatísticas.*/
	contErroGlobal2 = 0;
	qtdTrePercGlobal2 = 0;
	contveic = 0;
}



// Calcula a velocidade média em tempo real
int flag = 0;

void calcVelMedRT (std::list<Ptr<Vehicle> > lista, int idTre)
{
	ofstream velMedTrechoRT, dadosTrecho, maiorMenorVelTre;
	
	stringstream temp;
	temp << "data/DadosDosTrechos/dadosTrecho" << idTre;
	dadosTrecho.open(temp.str().c_str(), ios::app);
	temp.str("");
    temp << "data/VelMediaTempoReal/velMedTrechoRT" << idTre;
    velMedTrechoRT.open(temp.str().c_str(), ios::app);
    temp.str("");
    // maiormenor
    temp << "data/MaiorMenorVelTre/maiorMenorVelTre" << idTre;
    maiorMenorVelTre.open(temp.str().c_str(), ios::app);
    temp.str("");
	if(Simulator::Now().GetSeconds() == 0.0)
	{
		dadosTrecho 	<< "Tempo	"
						<< "IdVeh	"
						<< "VMK	"
						<< "PosX	"
						<< "Pista"
						<< endl;
				
		velMedTrechoRT 	<< "Tempo	"
						<< "QtdVeh	"
						<< "VMKMED	"
						<< "VMKMA"
						<< endl;
		// maiormenor
		/*maiorMenorVelTre	<< "Minuto	"
							<< "Máxima	"
							<< "Mínima"
							<< endl;*/
	}
	
	/* Contador que controla a impressão de "velMedia" somente no final da soma da 
		... velocidade de todos os veículos no trecho.*/
	int count = 1;
	double velMediaRT = 0;
	std::vector<int> vmedRT;
	// Captura a quantidade de veículos de verdade neste trecho no momento
	int NumVehiclesTrecho = lista.size();
	 
	for (std::list<Ptr<Vehicle> >::iterator j = lista.begin(); j != lista.end(); j++) 
	{
		// Vai somando a velocidade de todos os veículos que estão no trecho
		velMediaRT += (*j)->GetVelocity();
		vmedRT.push_back((*j)->GetVelocity());
			
		// Gravação em arquivo da velocidade média do trecho em tempo real
		// MÉDIA ARITMÉTICA
		if (count == NumVehiclesTrecho)
		{
			// Armazena a vel. média no trecho Leste X na posição X-1
			VMRT2[idTre-1] = velMediaRT / NumVehiclesTrecho;
			//velMedTrechoRT 	<< fixed << setprecision(0) << Simulator::Now().GetSeconds() << "	"
							//<< NumVehiclesTrecho << "	"
							//<< fixed << setprecision(2) << (velMediaRT / NumVehiclesTrecho) << "	"
							//<< fixed << setprecision(2) << (velMediaRT / NumVehiclesTrecho) * 3.6 << "	"
							//<< endl;
		}
		
		// MEDIANA
		if (count == NumVehiclesTrecho)
		{	
			std::sort (vmedRT.begin(), vmedRT.end());
			int median = vmedRT.size()/2;
			if ((vmedRT.size() % 2) == 0)
			{
				VMRT[idTre-1] = ((vmedRT[median] + vmedRT[median-1]) / 2);
				// maiormenor: Inicializa o array que guarda a maior e menor velocidade
				if(flag == 0)
				{
					// maiormenor: Chama a função que inicializa o array que guarda a maior e menor velocidade em cada trecho
					initPointArrayVarVel(&maiorVelTre, &menorVelTre, trechos);
					initPointArrayVarVel(&maiorVelTre2, &menorVelTre2, trechos);
					flag = 1;
				}
			}
			else
			{
				VMRT[idTre-1] = (vmedRT[median]);
				// maiormenor: Inicializa o array que guarda a maior e menor velocidade
				if(flag == 0)
				{
					// maiormenor: Chama a função que inicializa o array que guarda a maior e menor velocidade em cada trecho
					initPointArrayVarVel(&maiorVelTre, &menorVelTre, trechos);
					initPointArrayVarVel(&maiorVelTre2, &menorVelTre2, trechos);
					flag = 1;
				}
			}
			velMedTrechoRT 	<< fixed << setprecision(0) << Simulator::Now().GetSeconds() << "	"
							<< NumVehiclesTrecho << "	"
							<< fixed << setprecision(2) << VMRT[idTre-1] * 3.6 << "	"
							<< fixed << setprecision(2) << VMRT2[idTre-1] * 3.6 << "	"
							<< endl;
							
			// maiormenor: Guarda a variação de velocidade (maior e menor) 							
			if ((maiorVelTre[idTre-1] == -1) and (menorVelTre[idTre-1] == -1))
			{
					maiorVelTre[idTre-1] = VMRT[idTre-1];
					menorVelTre[idTre-1] = VMRT[idTre-1];
			}
			else if (VMRT[idTre-1] > maiorVelTre[idTre-1])
					maiorVelTre[idTre-1] = VMRT[idTre-1];
			else if (VMRT[idTre-1] < menorVelTre[idTre-1])
					menorVelTre[idTre-1] = VMRT[idTre-1];	
					
			// maiormenor: Guarda a variação de velocidade (maior e menor, a cada 10 minutos) 		
			if ((maiorVelTre2[idTre-1] == -1) and (menorVelTre2[idTre-1] == -1))
			{
					maiorVelTre2[idTre-1] = VMRT[idTre-1];
					menorVelTre2[idTre-1] = VMRT[idTre-1];
			}
			else if (VMRT[idTre-1] > maiorVelTre2[idTre-1])
					maiorVelTre2[idTre-1] = VMRT[idTre-1];
			else if (VMRT[idTre-1] < menorVelTre2[idTre-1])
					menorVelTre2[idTre-1] = VMRT[idTre-1];	
					
			// maiormenor: Gera a variação de velocidade (maior e menor) à cada 10 minutos segundos
			if((int(Simulator::Now().GetSeconds()) % 600 == 0))
			{				
				maiorMenorVelTre	<< fixed << setprecision(0) << (Simulator::Now().GetSeconds() / 60) << "	"
									<< fixed << setprecision(2) << maiorVelTre2[idTre-1] * 3.6 << "	"
									<< fixed << setprecision(2) << menorVelTre2[idTre-1] * 3.6 << "	";
									// comentado para ficar no formato da geração de gráficos do IEEE << endl;
				/* maiormenor: zera a posição do array que guarda a maior e menor velocidade do respectivo
				 * trecho. Deve ser zerada somente a posição do respectivo trecho, e não todas as posições do
				 * array, pois as velocidades dos outros trechos ainda não foi gravado em arquivo, e caso zeradas, 
				 * farão com que os valores sejam perdidos, caindo no primeiro if (que verifica sem ambos são -1), 
				 * e atribui a velocidade média RT no trecho como maior e menor velocidade.*/
				maiorVelTre2[idTre-1] = -1;
				maiorVelTre2[idTre-1] = -1;
			}
		
		}
			
		// Gravação em arquivo das estatísticas dos veículos no trecho
		// Descomente para gravar dados dos trechos
		dadosTrecho << fixed << setprecision(0) << Simulator::Now().GetSeconds()
					  << "	" << (*j)->GetVehicleId()
					  << "	" << fixed << setprecision(2) << ((*j)->GetVelocity() * 3.6)
					  << "	" << fixed << setprecision(2) << (*j)->GetPosition().x
					  << "	" << (*j)->GetLane()
					  << endl;		
							  
		count++;  
	}
	
	vmedRT.clear();
	
	// Retira do buffer e salva no arquivo 
	// Descomente para gravar dados dos trechos	
	dadosTrecho.close();
	velMedTrechoRT.close();
	maiorMenorVelTre.close();
}




// testeaviso: Verifica se um veículo passou por um obstáculo. Caso sim, anota a posição e pista que o mesmo se encontra
void passouOb (std::list<Ptr<Vehicle> > lista, Ptr<Vehicle> tempVehicle2, int idFirstVeh)
{
	bool passouOb;
	int trechoOb;
	double posOb;
	int pistaOb;
	
	for (std::list<Ptr<Vehicle> >::iterator j = lista.begin(); j != lista.end(); j++) 
	{
		// Caso seja um veículo de verdade na direção 1 e o obstáculo também esteja na direção 1
		if (((*j)->GetDirection() == 1) and (tempVehicle2->GetDirection() == 1) and ((*j)->GetVehicleId() > idFirstVeh))
		{
			// Verifica se o veículo passou pelo obstáculo
			passouOb = ((*j)->GetPosition().x >= tempVehicle2->GetPosition().x);
			trechoOb = GetTrechoObst(tempVehicle2->GetPosition().x, tempVehicle2->GetDirection());
		}
		// Caso seja um veículo de verdade na direção -1 e o obstáculo também esteja na direção -1
		else if (((*j)->GetDirection() == -1) and (tempVehicle2->GetDirection() == -1) and ((*j)->GetVehicleId() > idFirstVeh))
		{
			// Verifica se o veículo passou pelo obstáculo
			passouOb = ((*j)->GetPosition().x <= tempVehicle2->GetPosition().x);
			trechoOb = GetTrechoObst(tempVehicle2->GetPosition().x, tempVehicle2->GetDirection());
		}
		/* Guarda informações somente de trechos > 0 e < 36 (estes não importam pois não tem como avisar os APs) e também
		* só entra na condição se a informação não foi guardada antes (acontece pois veículo permanecer no range por um tempo)*/
		if ((passouOb) and (trechoOb > 0) and (trechoOb <= 36) and ((*j)->GetObstac(trechoOb).pos != posOb))
		{
			posOb = tempVehicle2->GetPosition().x;
			pistaOb = tempVehicle2->GetLane();
			// Guarda a informação que passou por um obstáculo no seu array (na posição respectiva ao trecho)
			(*j)->SetObstac(trechoOb, posOb, pistaOb);
		}
	}
}




// Simula a perda de 5% de pacotes na rotina de troca de TCT
int contPackets = 0;
int flag2 = 0;
// espalhainfo
int flag4 = 0;
int flag5 = 0;
int flag6 = 0;
int flag7 = 0;
int trechoObLes;
int trechoObOes;

void trocaTCT (std::list<Ptr<Vehicle> > lista, Ptr<Vehicle> tempVehicle, int idFirstVeh)
{
	// Armazena o ID do carro inativo e fantasma
	int vehSemOBU = 0, vehFantasma = 0;
	ofstream velMedTrecho;
	ofstream TCTVeh, vehFant;
	// Ponteiro para linha da TCT
	Ptr<Table> linha;                    
	// Variável somente para armazenar as tabelas de veículo e Ap, para não passar o retorno de "Get" para o método
	std::list<Ptr<Table> > tabelaVeh, tabelaAp;
	bool passouAp;
	bool horaTrocar;
	int IdTrecho;
	double distVehAteFimVia;
	double velMedia;
	stringstream temp;
	// Armazena a condição da via de acordo com as métricas do Google Maps
	string condicao;
	// Armazena a taxa de acerto da condição da TCT com a condição do trecho em tempo real
	double taxaAcerto;
	bool mesmadir;
	bool dircont;
	bool vizinho;
	bool vizinho1;
	bool vizinho2;
	bool vizinho3;
	int trechoMax;
	int treIniDir;
	int treFimDirCont;
	int treFimDir;
	
	// Grava a velocidade média em cada trecho
	velMedTrecho.open("data/servidor/velMedTrecho.log", ios::app);			  
	/* Cria cabeçalho somente quando o tempo de simulação for 0. Não é possível criar fora
	... da função "vehData" pois é preciso gravar valores somente acessíveis de dentro da função.*/             
	if(flag2 == 0)
	{									
		velMedTrecho 	<<"Veículo	"
						<<"Trecho	"
						<<"Pista	"
						<<"VMK"
						<< endl;
		flag2 = 1;
	}
	
	for (std::list<Ptr<Vehicle> >::iterator j = lista.begin(); j != lista.end(); j++) 
	{
		if ((*j)->GetDirection() == 1)
			passouAp = ((*j)->GetPosition().x >= t_posX);									  
		else 
			passouAp = ((*j)->GetPosition().x <= t_posX);	
			
		// Se é um veículo e passou por um AP
		if (((*j)->GetVehicleId() >= idFirstVeh) and (passouAp))
		{
			// Verifica se está na hora de trocar tabela (se passou por um AP maior que o inicial)
			if ((*j)->GetDirection() == 1)								  
				horaTrocar = ((*j)->GetIdApAnt() < t_vid);
			else 
				horaTrocar = ((*j)->GetIdApAnt() > t_vid);
			// Pega o tempo exato que passa pelo 1° AP. Após isso, nunca mais entra nesta condição.
			if((*j)->GetTime() == 0)
			{
				// testeaviso: Coleta os valores dos trechos iniciais pois serão usados para ver se tem obstáculo ao entrar neles
				if((*j)->GetDirection() == 1)
					IdTrecho = 1;
				else
					IdTrecho = apQtd;
				// fim
				(*j)->SetTime (Simulator::Now().GetSeconds());          
				(*j)->SetTime1Ap (Simulator::Now().GetSeconds());               

				/* Necessário receber a TCT do primeiro Ap ao entrar na via*/
				tabelaAp = tempVehicle->GetTable();
				// Envia TCT com as condições dos trechos atualizada para o veículo
				// newttl
				(*j)->UpdateTable(tabelaAp, 0, 0, 0, 0, 0, 0, 0); 
				/* testeaviso: Verifica se há obstáculo no trecho em que vai entrar, com base na TCT recebida.
				 * Se sim, anota a posição e pista em que o mesmo se encontra para cuidar de não colidir com o mesmo.*/
				/* controle: só registra a posição e pista do possível obstáculo localizado no trecho em que vai entrar
				 * se o aviso de obstáculos estiver habilitado.*/
				if(avisoObst)
				{
					tabelaVeh = (*j)->GetTable();    
					std::list<Ptr<Table> >::iterator it4;							
					for(it4 = tabelaVeh.begin(); it4!= tabelaVeh.end(); it4++)
					{                 
						if(((*it4)->GetTrecho() == IdTrecho) and ((*it4)->GetObst().pos != 0.0))
						{				
							(*j)->SetValObst((*it4)->GetObst());
						}
						
						
						/* espalhainfo: Pega o momento em que aparece pela primeira vez nos Aps iniciais
						 * de ambas as direções as informações sobre obstáculos inseridos nos trechos.*/
						if(((*it4)->GetObst().pos != 0.0) and ((*j)->GetDirection() == 1) 
							and ((*it4)->GetTrecho() < apQtd) and (flag4 == 0))
						{
							cout <<"\nTempo para informação sobre inserção de obstáculo chegar ao início da direção leste: " <<Simulator::Now().GetSeconds() - tempoInsObs <<" segundos" <<endl;
							flag4 = 1;
							trechoObLes = (*it4)->GetTrecho();
						}
						else if(((*it4)->GetObst().pos != 0.0) and ((*j)->GetDirection() == -1) 
							and ((*it4)->GetTrecho() >= apQtd) and (flag5 == 0))
						{
							cout <<"\nTempo para informação sobre inserção de obstáculo chegar ao início da direção oeste: " <<Simulator::Now().GetSeconds() - tempoInsObs <<" segundos" <<endl;
							flag5 = 1;
							trechoObOes = (*it4)->GetTrecho();
						}
					
						
						/* espalhainfo: Pega o momento em que aparece pela primeira vez nos Aps iniciais
						 * de ambas as direções as informações sobre obstáculos removidos nos trechos. Só analisa
						 * se o trecho no qual o obstáculo foi inserido já foi zerado, porém somente se o tempo
						 * de gravação da informação for maior que o momento da remoção (para evitar problemas com
						 * veículo retardatários carregando informação desatualizada sobre os trechos).*/
						if(((*it4)->GetTrecho() == trechoObLes) and ((*it4)->GetHoraGrava() > tempoRemObs) 
							and ((*it4)->GetObst().pos == 0.0) and (flag6 == 0))
						{
							cout <<"\nTempo para informação sobre remoção de obstáculo chegar ao início da direção leste: " <<Simulator::Now().GetSeconds() - tempoRemObs <<" segundos" <<endl;
							flag6 = 1;
						}
						else if(((*it4)->GetTrecho() == trechoObOes) and ((*it4)->GetHoraGrava() > tempoRemObs) 
							and ((*it4)->GetObst().pos == 0.0) and (flag7 == 0))
						{
							cout <<"\nTempo para informação sobre remoção de obstáculo chegar ao início da direção oeste: " <<Simulator::Now().GetSeconds() - tempoRemObs <<" segundos" <<endl;
							flag7 = 1;
						}
						// fim espalhainfo
					}
				}
				// fim		
			}
			
			// Se está na hora de trocar tabela	  						 
			if (horaTrocar)
			{				
				if ((*j)->GetDirection() == 1)
				{
					IdTrecho = (*j)->GetIdApAnt();
					// Usada para controlar o trecho máximo que interessa o veículo saber se tem obstáculo ou não 
					trechoMax = (apQtd-1);
					distVehAteFimVia = (highwayLength - (*j)->GetPosition().x);
					// newttl
					treIniDir = 1;
					treFimDirCont = (2 * (apQtd-1));
					treFimDir = apQtd-1;
				}
				else 
				{
					IdTrecho = ((apQtd-1) + (apQtd - t_vid));
					// Usada para controlar o trecho máximo que interessa o veículo saber se tem obstáculo ou não 
					trechoMax = (2*(apQtd-1));
					distVehAteFimVia = (*j)->GetPosition().x;
					// newttl
					treIniDir = apQtd;
					treFimDirCont = apQtd-1;
					treFimDir = (2*(apQtd-1));
				}
				/* Pega o momento que o primeiro veículo criado cruza a via (taxa de acerto a partir 
				 * que já temos TCTs completas)*/
				/* newestab: Não precisa mais calcular esta informação, uma vez que o momento considerado
				 * para via estabilizada mudou após a mudança no cálculo do TTL.*/
				/*if(((*j)->GetVehicleId() == idFirstVeh) and (t_vid == apQtd))
				{
					tempVehCruzouVia = 2 * Simulator::Now().GetSeconds();
					// Inicializa as variáveis que controlam o tempo de criação e remoção do primeiro obstáculo
					//freqInsObs[y] = 333.0 + tempVehCruzouVia;
					//freqRemObs = (499.5 + freqInsObs[z]);
				}*/	
				
				
				// newfantasma
				// Guarda o ID do veículo fantasma, que será sempre criado à cada 100 veículos.
				if((*j)->GetVehicleId() % 100 == 0)
				{
					vehFantasma = (*j)->GetVehicleId();
				}				
				
				/* propInativos: Escolhe o veículo como carro inativo sempre que estiver habilitada a baixa
				 * densidade de OBU e o resultado do módulo do ID por 5 for diferente de zero, o que dará
				 * uma taxa de 80% de inativos.*/			      
				else if((baixaDensidade) and ((*j)->GetVehicleId() % 5 != 0))
				{
				    // Guarda o ID do veículo como carro inativo
				    vehSemOBU = (*j)->GetVehicleId();
				}
			      
				// Calcula a velocidade média no trecho
				velMedia = 3.6 * (apDist  / (Simulator::Now().GetSeconds() - (*j)->GetTime()));
					
				velMedTrecho 	<< (*j)->GetVehicleId() 
								<< "	" <<(*j)->GetIdApAnt()
								<< "	" <<(*j)->GetLane() 
								<< "	" <<fixed <<setprecision(2) <<velMedia
								<< endl;
				
		
				/// Início da troca de TCTs
								   
				/* Cria arquivos individuais para os veículos mostrando as condições (TCT) nos respectivos trechos
					(somente os últimos 2000 segundos de simulação, para não criar muitos arquivos.*/
				// newtll
				//if (Simulator::Now().GetSeconds() >= (simTime - 2000))
				if (Simulator::Now().GetSeconds() >= (0))
				{
					temp << "data/TCTVeiculoTrecho/TCTVeh" << (*j)->GetVehicleId() <<"Trecho" <<IdTrecho;
					TCTVeh.open(temp.str().c_str(), ios::app);
					TCTVeh 	<< "Trecho	"
							<< "MHT	"
							<< "MHTAnt	"
							<< "TTL	"
							<< "TTLMax	"
							<< "PosOB	"
							<< "PistaOB	"
							<< "Cronom "
							<< "Tempo "
							<< "VMKRT	"
							<< "Cond	"
							<< "%Acerto	"
							<< endl;
					temp.str("");
				}
				
				/* newttl: Antes de enviar a TCT para o Ap, o veículo deve consultar em sua TCT local a velocidade 
				 * média para percorrer do trecho proporcional ao trecho que está, até o trecho inicial de sua direção,
				 * para usar este valor como TTL da informação do trecho em que se encontra.*/
				double valTTL = 0.0;
				// Verifica qual é o trecho proporcional ao trecho em que se encontra
				int treProp = ((apQtd-1) + (apQtd - IdTrecho));
				tabelaVeh = (*j)->GetTable();
				/* Total de iterações necessárias para realizar o somatório das velocidades médias do 
				 * trecho proporcional até o trecho inicial da direção;*/
				std::list<Ptr<Table> >::iterator it;
				// variancia
				double variancia;
				
				// Percorre a TCT local para consultar a velocidade média do trecho proporcional até o inicial da direção
				for(it = tabelaVeh.begin(); it!= tabelaVeh.end(); it++)
				{
					// Executa até que percorra todos os trechos (do proporcional ao início da direção)
					if(((*it)->GetTrecho() >= treProp) and ((*it)->GetTrecho() <= treFimDirCont))
					{
						// String usada para comparar quando o resultado do cálculo do TTL é "inf" (valor nulo)
						std::string inf ("inf");
						/* variancia: Cálculo da variação de velocidade. Caso a velocidade atual seja menor que
						 * a anterior, então o tempo para atravessar o trecho será maior. Assim, para não definirmos
						 * simplesmente o TTL como o tempo para atravessar o trecho baseado na velocidade atual
						 * (já que a mesma vem diminuindo, o que poderia gerar um TTL não satisfatório se a velocidade
						 * continuar a cair), calculamos a variância e acrescentamos este resultado ao valor do TTL.*/							
						if(((*it)->GetSpeed() < (*it)->GetSpeedAnt()) and ((*it)->GetSpeed() >= 5))
							variancia = abs((((*it)->GetSpeed() - (*it)->GetSpeedAnt())/(*it)->GetSpeedAnt()));
						/* variancia: Caso a velocidade atual não seja menor que a anterior, não fazemos nada. Deixamos o TTL
						 * ser definido exatamente como o tempo para atravessar o trecho, baseado na velocidade atual
						 * do(s) trecho(s) proporcional.*/
						else 
							variancia = 0;
						//double velMedTreProp = (apDist / ((*it)->GetSpeed()/3.6));
						// variancia
						double velMedTreProp = (apDist / (((*it)->GetSpeed()-((*it)->GetSpeed() * variancia)) / 3.6));
						/* Início da conversão do valor obtido acima para string (necessário para verificar se
						 * o valor obtido acima foi um valor nulo (inf). Caso sim, o mesmo é transformado em 0.0.*/							
						std::ostringstream velMedTrePropEmStr;
						velMedTrePropEmStr << velMedTreProp;
						std::string velMedTrePropConStr = velMedTrePropEmStr.str();
						if (velMedTrePropConStr.compare(inf) == 0)
							velMedTreProp = 0.0;
						// Fim do processo de conversão e verificação do valor obtido
						// Incrementa o TTL com o valor obtido
						valTTL += velMedTreProp;
					}
				}
				// fim
					
			  
				/* Verifica a condição para trocar a TCT. Caso haja perda de pacotes (todos os veículos são ativos),
				 * então à cada 14 veículos que trocam, 1 passa direto sem trocar (índice de 7% de pacotes perdidos).
				 * Caso não haja perda de pacotes (80% de veículos inativos), então troca caso o veículo não seja um
				 * veículo inativo.*/
				 
				 bool condParaTrocarTCT;
				 
				 if(baixaDensidade == false)
					condParaTrocarTCT = ((contPackets < 14) and ((*j)->GetVehicleId() != vehSemOBU) 
					and ((*j)->GetVehicleId() != vehFantasma));
				 else
				 	condParaTrocarTCT = (((*j)->GetVehicleId() != vehSemOBU) and ((*j)->GetVehicleId() != vehFantasma));
				 
				// controle
				if (condParaTrocarTCT)
				{                        
					// Ao passar pelo Ap, cria a linha da TCT com os dados referente ao trecho que passou
					// variancia
					//linha = CreateObject<Table>(IdTrecho, velMedia, -1, valTTL, 0.0, (*j)->GetObstac(IdTrecho), 300.0, Simulator::Now().GetSeconds());
					// newttl
					linha = CreateObject<Table>(IdTrecho, velMedia, valTTL, (*j)->GetObstac(IdTrecho), Simulator::Now().GetSeconds());
					// Adiciona a linha na TCT					                
					(*j)->AddTable(linha);
					// Variável que serve somente para receber o retorno da tabela do veículo
					tabelaVeh = (*j)->GetTable();
					/* Envia a TCT local atualizada para o Ap. Envio para o AP, além da tabela do veículo, o tempo que 
					 * ele demorou para passar pelo trecho, no qual usarei para decrementar o TTL de todos os trechos de 
					 * sua tabela.*/
					tempVehicle->UpdateTable(tabelaVeh, IdTrecho, treProp, treIniDir, treFimDirCont, treFimDir, trunc(Simulator::Now().GetSeconds() - (*j)->GetTime()), valTTL);
					// Variável que serve somente para receber o retorno da tabela do Ap
					tabelaAp = tempVehicle->GetTable();
					// Envia TCT com as condições dos trechos atualizada para o veículo
					(*j)->UpdateTable(tabelaAp, 0, 0, 0, 0, 0, 0, 0);
					// Variável que serve somente para receber o retorno da tabela do veículo
					contPackets +=1;
				}				
				if ((contPackets >= 14))
					contPackets = 0;
					
					/* Necessário dar um get na tabela pois o único momento em que isso é feito é
					 * na rotina de troca de TCT (acima), e como caso o veículo seja um carro inativo
					 * ele não entra na rotina (e ele precisa deste get para percorrer a condição da TCT inicial)
					 * para comparar com a condição real dos trechos.*/
					tabelaVeh = (*j)->GetTable();              
					
					/* controle: caso avaliaPosEstab esteja habilitado, a avaliação da taxa de acerto será somente
					 * após a via estabilizada. Caso não esteja, a avaliação é feita desde o começo da simulação.*/					
					if(avaliaPosEstab == true)
						//horaAvaliar = tempVehCruzouVia;
						// newestab
						horaAvaliar = tempoViaEstab;
					else
						horaAvaliar = 0;
					// fim
								
					// Verifica a taxa de acerto da proposta após a via estar estabilizada 
					/* controle: verifica se o veículo foi criado após o tempo definido para avaliação da taxa de 
					 * acerto. Este tempo pode ser 0 ou após a via estar estabilizada.*/
					//if ((*j)->GetTime1Ap() > horaAvaliar)
					if ((*j)->GetTime1Ap() > 0)
					{
						std::list<Ptr<Table> >::iterator it1;							
						// Somatório dos Trechos percorridos por todos os veículos
						qtdTrePercTrecho+=1;
						// Guarda na posição específica do array quantas vezes cada trecho foi percorrido
						pqtdTrePercTrecho[IdTrecho-1] += 1;
						for(it1 = tabelaVeh.begin(); it1!= tabelaVeh.end(); it1++)
						{
							/* Só analisa as taxas de acerto se o veículo não for um carro inativo (pois o mesmo não
							 * possui uma OBU e assim não participa do sistema. Ou se não for um carro fantasma (pois 
							 * a TCT do mesmo nunca é atualizada, já que este não troca TCT)*/
							if (((*j)->GetVehicleId() != vehSemOBU) and ((*j)->GetVehicleId() != vehFantasma))
							{			
								// Coloca a condição do trecho com base nas métricas do Google Maps
								if ((*it1)->GetTtl() == 0.0)
									condicao = " - ";
								else
								{
									if((*j)->GetDirection() == 1)
									{
										mesmadir = ((*it1)->GetTrecho() < apQtd);
										dircont = ((abs((*j)->GetPosition().x - (highwayLength-((((*it1)->GetTrecho()-(apQtd-1))*apDist)+apInitDist)))) <= raio);
										vizinho = ((abs((*j)->GetPosition().x - (((*it1)->GetTrecho()*apDist)+apInitDist))) <= raio);
										vizinho1 = ((abs((*j)->GetPosition().x - (((*it1)->GetTrecho()*apDist)+apInitDist)) <= (raio)) and ((raio) <= distVehAteFimVia));
										vizinho2 = ((abs((*j)->GetPosition().x - (((*it1)->GetTrecho()*apDist)+apInitDist)) <= (raio/2)) and ((raio/2) <= distVehAteFimVia));
										vizinho3 = ((abs((*j)->GetPosition().x - (((*it1)->GetTrecho()*apDist)+apInitDist)) <= (raio/4)) and ((raio/4) <= distVehAteFimVia));																		
									}
									else 
									{
										mesmadir = ((*it1)->GetTrecho() >= apQtd);
										dircont = ((abs((*j)->GetPosition().x - (((*it1)->GetTrecho()*apDist)+apInitDist))) <= raio);
										vizinho = ((abs((*j)->GetPosition().x - (highwayLength-((((*it1)->GetTrecho()-(apQtd-1))*apDist)+apInitDist)))) <= raio);
										vizinho1 = (((abs((*j)->GetPosition().x - (highwayLength-((((*it1)->GetTrecho()-(apQtd-1))*apDist)+apInitDist)))) <= (raio)) and ((raio) <= distVehAteFimVia));
										vizinho2 = (((abs((*j)->GetPosition().x - (highwayLength-((((*it1)->GetTrecho()-(apQtd-1))*apDist)+apInitDist)))) <= (raio/2)) and ((raio/2) <= distVehAteFimVia));
										vizinho3 = (((abs((*j)->GetPosition().x - (highwayLength-((((*it1)->GetTrecho()-(apQtd-1))*apDist)+apInitDist)))) <= (raio/4)) and ((raio/4) <= distVehAteFimVia));
									}
									// Mudar para >= 60 se velocidades máximas forem 70 e 60	
									if (((*it1)->GetSpeed() >= 80) and (VMRT[(*it1)->GetTrecho()-1] * 3.6 >= 80))
											condicao = "Rápido";
									// Mudar para >= 30 e <= 60 se velocidades máximas forem 70 e 60	
									else if ((((*it1)->GetSpeed() >= 40) and ((*it1)->GetSpeed() < 80)) and (((VMRT[(*it1)->GetTrecho()-1] * 3.6) >= 40) and ((VMRT[(*it1)->GetTrecho()-1] * 3.6) < 80)))
												condicao = "Médio";
									// Mudar para >= 5 e <= 30 se velocidades máximas forem 70 e 60	
									else if ((((*it1)->GetSpeed() >= 5) and ((*it1)->GetSpeed() < 40)) and (((VMRT[(*it1)->GetTrecho()-1] * 3.6) >= 5) and ((VMRT[(*it1)->GetTrecho()-1] * 3.6) < 40)))
											condicao = "Lento";
									else if ((((*it1)->GetSpeed() >= 0) and ((*it1)->GetSpeed() < 5)) and (((VMRT[(*it1)->GetTrecho()-1] * 3.6) >= 0) and ((VMRT[(*it1)->GetTrecho()-1] * 3.6) < 5)))
											condicao = "Parado";
									else
									{
										condicao = "Erro";
										// Conta a quantidade de erros
										contErroGlobal+=1;
										// Conta erro global por trecho
										pcontErroGlobal[(*it1)->GetTrecho()-1]+=1;
										// Conta a quantidade de erros no trecho em que o veículo está
										if ((*it1)->GetTrecho() == IdTrecho)
										{
												contErroTrecho+=1;
												pcontErroTrecho[(*it1)->GetTrecho()-1] += 1;
										}
										// Conta a quantidade de erros somente nos trechos vizinhos
										if (mesmadir)
										{
											// Conta a quantidade de erros na mesma direção
											contErroMesmaDir+=1;
											pcontErroMesmaDir[(*it1)->GetTrecho()-1] +=1;
											// Trechos vizinhos
											if(vizinho)
											{
												contErroTreVizinho+=1;
												pcontErroTreVizinho[(*it1)->GetTrecho()-1] +=1;
											}
											// Trechos distantes
											else
											{
												contErroTreVizDist+=1;
												pcontErroTreVizDist[(*it1)->GetTrecho()-1] +=1;
											}
											// Trechos à frente
											if ((*it1)->GetTrecho() > IdTrecho)
											{
												// Todos à frente
												contErroMesmaDirFrente+=1;
												pcontErroMesmaDirFrente[(*it1)->GetTrecho()-1] +=1;
												// 5km
												if(vizinho1)
												{
													contErroMesmaDirFrente1+=1;
													pcontErroMesmaDirFrente1[(*it1)->GetTrecho()-1] +=1;
												}
												// 2.5 km
												if (vizinho2)
												{
													contErroMesmaDirFrente2+=1;
													pcontErroMesmaDirFrente2[(*it1)->GetTrecho()-1] +=1;
												}
												// 1.25 km
												if (vizinho3)
												{
													contErroMesmaDirFrente3+=1;
													pcontErroMesmaDirFrente3[(*it1)->GetTrecho()-1] +=1;
												}
											}										
										}
										else
										{
											// Agrega os trechos vizinhos da direção contrária
											if (dircont)
											{
												contErroTreVizinho+=1;
												pcontErroTreVizinho[(*it1)->GetTrecho()-1] +=1;
											}
											// Agrega os trechos distantes da direção contrária 						
											else
											{
												contErroTreVizDist+=1;
												pcontErroTreVizDist[(*it1)->GetTrecho()-1] +=1;
											}	
										}
									}
									// Conta a quantidade de linhas válidas na TCT (Global)
									qtdTrePercGlobal+=1;
									// Conta linhas válidas por trecho
									pqtdTrePercGlobal[(*it1)->GetTrecho()-1]+=1;
									// Conta a quantidade de linhas válidas na TCT (Mesma direção e Trechos vizinhos)
									if (mesmadir)
									{
										// Mesma direção
										qtdTrePercMesmaDir+=1;
										pqtdTrePercMesmaDir[(*it1)->GetTrecho()-1] +=1;
										// Trechos vizinhos
										if(vizinho)
										{
											qtdTrePercTreVizinho+=1;
											pqtdTrePercTreVizinho[(*it1)->GetTrecho()-1] +=1;
										}
										// Trechos distantes
										else
										{
											qtdTrePercTreVizDist+=1;
											pqtdTrePercTreVizDist[(*it1)->GetTrecho()-1] +=1;
										}
										// Trechos à frente
										if ((*it1)->GetTrecho() > IdTrecho)
										{
											// Todos à frente
											qtdTrePercMesmaDirFrente+=1;
											pqtdTrePercMesmaDirFrente[(*it1)->GetTrecho()-1] +=1;
											// 5 km								
											if(vizinho1)
											{
												qtdTrePercMesmaDirFrente1+=1;
												pqtdTrePercMesmaDirFrente1[(*it1)->GetTrecho()-1] += 1;
											}
											// 2.5 km
											if(vizinho2)
											{
												qtdTrePercMesmaDirFrente2+=1;
												pqtdTrePercMesmaDirFrente2[(*it1)->GetTrecho()-1] +=1;
											}
											// 1.25 km
											if(vizinho3)
											{
												qtdTrePercMesmaDirFrente3+=1;
												pqtdTrePercMesmaDirFrente3[(*it1)->GetTrecho()-1] +=1;
											}
										}
									}
									else
									{
										// Agrega os trechos vizinhos da direção contrária
										if(dircont)
										{
											qtdTrePercTreVizinho+=1;
											pqtdTrePercTreVizinho[(*it1)->GetTrecho()-1] +=1;
										}
										// Agrega os trechos distantes da direção contrária
										else
										{
											qtdTrePercTreVizDist+=1;
											pqtdTrePercTreVizDist[(*it1)->GetTrecho()-1] +=1;
										}
									}
								}
									
								// Define a taxa de acerto entre a condição do trecho na TCT e do trecho em tempo real	
								if ((*it1)->GetSpeed() > (VMRT[(*it1)->GetTrecho()-1] * 3.6))
									taxaAcerto = 100 - fmod(((*it1)->GetSpeed()*100)/(VMRT[(*it1)->GetTrecho()-1] * 3.6), 100);
								else
									if((*it1)->GetSpeed() < (VMRT[(*it1)->GetTrecho()-1] * 3.6))
									taxaAcerto = ((*it1)->GetSpeed()*100)/(VMRT[(*it1)->GetTrecho()-1] * 3.6);
																
								// Imprime as TCTs dos veículos à cada trecho após a via estar estabilizada			
								//if (Simulator::Now().GetSeconds() >= (simTime - 2000))
								if (Simulator::Now().GetSeconds() >= (0))
								{
									TCTVeh  << (*it1)->GetTrecho() <<"	"
											<< fixed <<setprecision(2) << (*it1)->GetSpeed() << "	"
											<< fixed <<setprecision(2) << (*it1)->GetSpeedAnt() << "	"
											<< fixed <<setprecision(2) << (*it1)->GetTtl() << "	"
											<< fixed <<setprecision(2) << (*it1)->GetTtlMax() << "	"
											<< fixed <<setprecision(2) << (*it1)->GetObst().pos << "	"
											<< fixed <<setprecision(2) << (*it1)->GetObst().pista << "	"
											<< fixed <<setprecision(0) << (*it1)->GetCronometro() << "	"
											<< fixed <<setprecision(0) << (*it1)->GetHoraGrava() << "	"
											<< fixed <<setprecision(2) << VMRT[(*it1)->GetTrecho()-1] * 3.6 << "	"
											<< condicao <<"	"
											<< taxaAcerto << "	"
											<< endl;
								}
							}
							/* Caso seja um carro genuninamente fantasma (e não somente um carro inativo), grava ambas 
							 * as condições (da TCT inicial e a real) do respectivo trecho em que o mesmo se encontra, 
							 * caso o fantasma tenha sido criado após a via estar estabilizada (Ap com informações 
							 * sobre todos os trechos)*/						
							else if((*j)->GetVehicleId() == vehFantasma)
							{
								temp << "data/VehFantasma/vehFant" << (*j)->GetVehicleId();                   
								vehFant.open(temp.str().c_str(), ios::app);
								// Condição para criar o cabeçalho somente 1x
								if(IdTrecho == 1 or IdTrecho == apQtd)
								{
								vehFant << "Trecho    "
										<< "CondTCT    "
										<< "CondAoChegar"
										<< endl;
								}
								temp.str("");
								
								if(((*it1)->GetTrecho() == IdTrecho) and ((*j)->GetTime1Ap() > tempoViaEstab))
								{
									// Calcula a taxa de acerto de veículos fantasmas
									if ((*it1)->GetSpeed() < 5.0)
									{
										if (VMRT[(*it1)->GetTrecho()-1] * 3.6 >= 5.0)
										{
											contErroFant+=1;
											pcontErroFant[(*it1)->GetTrecho()-1] += 1;
										}
									}									
									// Mudar para 30 se velocidades máximas forem 70 e 60	
									else if ((*it1)->GetSpeed() < 40.0)
									{
										if ((VMRT[(*it1)->GetTrecho()-1] * 3.6 < 5.0) || (VMRT[(*it1)->GetTrecho()-1] * 3.6 >= 40.0))
										{
											contErroFant+=1;
											pcontErroFant[(*it1)->GetTrecho()-1] += 1;
										}
									}									
									// Mudar para 60 se velocidades máximas forem 70 e 60	
									else if ((*it1)->GetSpeed() < 80.0)
									{
										if ((VMRT[(*it1)->GetTrecho()-1] * 3.6 < 40.0) || (VMRT[(*it1)->GetTrecho()-1] * 3.6 >= 80.0))
										{
											contErroFant+=1;
											pcontErroFant[(*it1)->GetTrecho()-1] += 1;
										}
									}
									else 
									{	
										if (VMRT[(*it1)->GetTrecho()-1] * 3.6 < 80.0)
										{
											contErroFant+=1;
											pcontErroFant[(*it1)->GetTrecho()-1] += 1;
										}
									}
									// Grava em arquivo as estatísticas do veículo fantasma
									qtdTrePercFant+=1;
									pqtdTrePercFant[(*it1)->GetTrecho()-1] += 1;
									vehFant	<< (*it1)->GetTrecho() << "	"
											<< fixed <<setprecision(2) << (*it1)->GetSpeed() << "	"
											<< fixed <<setprecision(2) << VMRT[(*it1)->GetTrecho()-1] * 3.6
											<< endl;
								}
							}
							
							// testeaviso: Se o veículo vai entrar em um trecho que tem obstáculo, guarda as informações do mesmo
							/* controle: só registra a posição e pista do possível obstáculo localizado no trecho em que vai entrar
							 * se o aviso de obstáculos estiver habilitado.*/
							if (avisoObst)
							{
								if(((*it1)->GetTrecho() == (IdTrecho+1)) and ((IdTrecho+1) <= trechoMax)  and ((*it1)->GetObst().pos != 0.0))
								{
									(*j)->SetValObst((*it1)->GetObst());
								}
							}
							// fim
									
						}
					// Comente aqui (if que habilita taxa de acerto somente após via estabilizada)
					}
					
					/// FIM DA TROCA DE TCTs (VEÍCULOS NA DIREÇÃO LESTE)
					
					// Seta a hora que passou pelo AP
					(*j)->SetTime (Simulator::Now().GetSeconds());
					// Seta o AP pelo qual passou como AP anterior 
					(*j)->SetIdApAnt (t_vid); 
					
				  }				  
								
				  // Adiciona ID do AP, ID de vehicle e momento que passou pelo AP no map
				  mapVehicles[t_vid].insert(std::make_pair((*j)->GetVehicleId(), Simulator::Now().GetSeconds()));
				}
	}
	velMedTrecho.close();
	TCTVeh.close();
	vehFant.close();
}



// Insere os obstáculos na via à cada X minutos
int firstObLes = apQtd+1;
int firstObOes = firstObLes + obQtd;
int obDistLes = obInitDist;
int obDistOes = highwayLength - obInitDist;
int contOb = 1;

void colocaObst(Ptr<Highway> highway)
{
	int posAleOb;
	Ptr<Vehicle> tempOb;
	int trechoOb;	
	// Controla a criação de somente 1 obstáculo por pista
	int flag1 = 0;	
	// Gerador de números aleatórios para a criação de obstáculos aleatórios
	srand(time(NULL));
	
	// Controle para impedir a criação de mais obstáculos do que o especificado em obQtd
	if (contOb <= obQtd)
	{
		tempOb = highway->FindVehicle(firstObLes);
		// Se obAleat, muda a posição (antes fixas) dos obstáculos da direção leste para posições aleatórias
		if (obAleat)
		{
			while (flag1 == 0)
			{
				// O obstáculo pode ter qualquer posição na via, de 0 até a extensão máxima da pista
				posAleOb = rand() % highwayLength;
				// Identifica o trecho correspondente à posição sorteada
				trechoOb = GetTrechoObst(posAleOb, tempOb->GetDirection());	
				// Caso não tenha obstáculo neste trecho
				if(contObst[trechoOb] == 0)
				{
					// Confirma a posição que receberá o obstáculo
					obDistLes = posAleOb;
					// Seta 1 para que não crie mais obstáculos neste trecho
					contObst[trechoOb] = 1;
					// Pode sair do loop. Caso não entre no if, executa até sortear uma posição válida.
					flag1 = 1;
				}
			}			
			
			cout <<"\nObstáculo de número " << contOb << " colocado no trecho " << trechoOb <<" da pista " << floor(tempOb->GetPosition().y / medianGap) << " da direção Leste!!!";
		}
		else
		{
			// 1obdin: Para criar mais obstáculos dinâmicos, mudar o valor fixo abaixo para obDistLes
			tempOb->SetPosition(Vector(4750, tempOb->GetPosition().y, tempOb->GetPosition().z));
			cout <<"\nObstáculo de número " << contOb << " colocado na posição " << tempOb->GetPosition().x <<" da pista " << floor(tempOb->GetPosition().y / medianGap) << " da direção Leste!!!";
		}
		
		
		flag1 = 0;
		tempOb = highway->FindVehicle(firstObOes);
		// Se obAleat, muda a posição (antes fixas) dos obstáculos da direção oeste para posições aleatórias
		if (obAleat)
		{	
			// teste	
			while (flag1 == 0)
			{
				posAleOb = rand() % highwayLength;
				trechoOb = GetTrechoObst(posAleOb, tempOb->GetDirection());
				if(contObst[trechoOb] == 0)
				{
					obDistOes = posAleOb;
					contObst[trechoOb] = 1;
					flag1 = 1;
				}
			}
			cout <<"\nObstáculo de número " << contOb << " colocado no trecho " << trechoOb <<" da pista " << floor(((((numOfLanes*2)*5)+medianGap) - tempOb->GetPosition().y) / 5) << " da direção Oeste!!!" <<endl;
		}
		else
		{
			// 1obdin: Para criar mais obstáculos dinâmicos, mudar o valor fixo abaixo para obDistOes
			tempOb->SetPosition(Vector(5250, tempOb->GetPosition().y, tempOb->GetPosition().z));		
			cout <<"\nObstáculo de número " << contOb << " colocado na posição " << tempOb->GetPosition().x <<" da pista " << floor(((((numOfLanes*2)*5)+medianGap) - tempOb->GetPosition().y) / 5) << " da direção Oeste!!!" <<endl;
		}
		
		/* Pode alterar o valor das variáveis obDistLes e obDistOes que, caso os obstáculos sejam aleatórios, 
		 * os mesmos receberão nos valores baseados na semente, não fazendo diferença esta alteração abaixo.*/
		obDistLes += obDist;
		obDistOes -= obDist;
		firstObLes += 1;
		firstObOes += 1;
		contOb ++;
	}
}




// Retira os obstáculos da via depois de X minutos de criados
int firstObLes2 = apQtd+1;
int firstObOes2 = firstObLes2 + obQtd;
int contOb2 = 1;

void removeObst(Ptr<Highway> highway)
{
	Ptr<Vehicle> tempOb;
	int trechoOb;
	
	// Controle para impedir a remoção de mais obstáculos do que o especificado em obQtd
	if (contOb2 <= obQtd)
	{
		tempOb = highway->FindVehicle(firstObLes2);
		// Coleta o trecho correspondente à posição do obstáculo a ser removido
		trechoOb = GetTrechoObst(tempOb->GetPosition().x, tempOb->GetDirection());
		// Remove do array para que seja possível criar novo obstáculo nesre trecho, caso sorteado
		contObst[trechoOb] = 0;
		tempOb->SetPosition(Vector((highwayLength+50), tempOb->GetPosition().y, tempOb->GetPosition().z));
		cout <<"\nObstáculo de número " << contOb2 << " removido do trecho " <<trechoOb <<" da pista " << floor(tempOb->GetPosition().y / medianGap) << " da direção Leste!!!";
		
		tempOb = highway->FindVehicle(firstObOes2);
		trechoOb = GetTrechoObst(tempOb->GetPosition().x, tempOb->GetDirection());
		contObst[trechoOb] = 0;
		tempOb->SetPosition(Vector(-50, tempOb->GetPosition().y, tempOb->GetPosition().z));
		cout <<"\nObstáculo de número " << contOb2 << " removido do trecho " <<trechoOb <<" da pista " << floor(((((numOfLanes*2)*5)+medianGap) - tempOb->GetPosition().y) / 5)<< " da direção Oeste!!!" <<endl;
		
		firstObLes2 += 1;
		firstObOes2 += 1;
		contOb2 ++;
	}
}





double freqAtualiz = 0.0;
// newestab
int flag3 = 0;

static void VehData(Ptr<Highway> highway) 
{
	Ptr<Vehicle> tempVehicle, tempVehicle2;
	std::list<Ptr<Vehicle> > lista1, lista2, lista3, lista4, lista5, lista6;
	ofstream file, vehFile, pathFile;
	file.open("data/vehData.log", ios::app);
	stringstream temp;  
	
	// newestab
	int contAux1 = 0;
	int contAux2 = 0;
		
	/* Para gerar um arquivo com as estatísticas individuais de cada veículo, descomente
	   ...a linha abaixo e comente a seguinte. OBS.: deixa a simulação mais lenta.*/
	//for (int i = 1; i <= highway->GetLastVehicleId(); i++)
	for (int i = 1; i <= apQtd; i++)
	{ 
		// Verifica se o veículo "i" ainda existe. Se já não cruzou a via. 
		tempVehicle = highway->FindVehicle(i);	
		if (tempVehicle != NULL) 
		{     
			/* Para gerar um arquivo com as estatísticas individuais de cada veículo, descomente
			...a condição IF abaixo. OBS.: deixa a simulação mais lenta.*/
		   //if(i <= highway->GetApQtd())
			//{
				lista1 = highway->FindVehiclesInRange(tempVehicle, 30, 1);
				lista2 = highway->FindVehiclesInRange(tempVehicle, 30, -1);
				t_vid = tempVehicle->GetVehicleId();
				t_posX = tempVehicle->GetPosition().x;                  
			   			   
			   /// INICÍO DO CÁLCULO DA VELOCIDADE MÉDIA EM TEMPO REAL DE CADA TRECHO                             
							  			  
			  /* Necessária a verificação se "i < apQtd" para não pegar os veículos no intervalo
				 ... a partir de i == apQtd, já que este trecho  não existe. Além disso, é usada a 
				 ... função "trunc" para só gravar em arquivo de 1 em 1 s, e não a cada 0.1 s. */
			  if((i < apQtd) and (Simulator::Now().GetSeconds() == trunc(Simulator::Now().GetSeconds())))
			  {	  
				  // Criado duas listas para ter como contar a quantidade de veículos em ambas direções usando .size()
				  lista3 = highway->FindVehiclesInSegment(t_posX, (t_posX+apDist), 1);
				  lista4 = highway->FindVehiclesInSegment(t_posX, (t_posX+apDist), -1);
				  
				  // Calcula a velocidade média em tempo real após a via estar estabilizada
				  /* controle: irá inciar o cálculo da velocidade média em tempo real ou desde o início
				   * da simulação (0), ou após o momento em que a via está estabilizada.*/				  
				  //if (Simulator::Now().GetSeconds() >= tempVehCruzouVia)
				  // newestab
				  //if (Simulator::Now().GetSeconds() >= tempoViaEstab)
				  if (Simulator::Now().GetSeconds() >= 0)
				  {
					  calcVelMedRT (lista3, i);
					  calcVelMedRT (lista4, ((apQtd-1) + (apQtd - i)));  
				  }   
				  // fim
				
			  }			  
			  /// FIM DO CÁLCULO DA VELOCIDADE MÉDIA EM TEMPO REAL DE CADA TRECHO             
			  
			  
			  if(Simulator::Now().GetSeconds() == trunc(Simulator::Now().GetSeconds()))
			  {
				  // TTL: decremento do TTL do AP à cada 1 segundo  
				  std::list<Ptr<Table> >::iterator it3;
				  std::list<Ptr<Table> > tabelaAp; 
				  tabelaAp =  tempVehicle->GetTable();
				  for(it3 = tabelaAp.begin(); it3!= tabelaAp.end(); it3++)
				  {		
					  if((*it3)->GetTtl() != 0.0)
						 (*it3)->SetTtl((*it3)->GetTtl() - 1.0);
					
					/* newestab: À cada 0.1 segundo, varre todos os trechos da TCT que estão na mesma direção 
					 * do respectivo Ap inicial da direção e verifica se há ainda algum trecho com TTl zero.
					 * Se sim, muda o valor da variável auxiliar para 1. */
					  if (tempVehicle->GetVehicleId() == 1)
						if(((*it3)->GetTrecho() < apQtd) and ((*it3)->GetTtl() == 0.0))
						 contAux1 = 1;
						 
					  if (tempVehicle->GetVehicleId() == apQtd)
						if(((*it3)->GetTrecho() >= apQtd) and ((*it3)->GetTtl() == 0.0))
						 contAux2 = 1;
					// fim
					
					 if((*it3)->GetCronometro() != 0.0)
						(*it3)->SetCronometro((*it3)->GetCronometro() - 1.0);
					 else
						(*it3)->SetTtl(0.0);
				  }				  
				  
				  /* newestab: Caso o valor da variável auxiliar de cada direção não tenha sido mudado para 1, 
				   * então não há mais trechos com TTL 0 na mesma direção dos respectivos Aps. Neste momento, 
				   * onde os motoristas que entram em ambas as direções já podem se orientar seguramente sobre 
				   * as condições dos trechos à frente, consideramos como via estabilizada (no antigo modo de
				   * cálculo do TTL, já tinhamos esta condição à partir do momento em que os Aps inicias tinham
				   * conhecimento sobre todas as condições de trânsito- hoje temos esta informação no mesmo tempo,
				   * porém com alguns trechos tendo TTL inválido - onde era preciso calcular somente o tempo do
				   * veículo cruzar a via * 2).*/
				  if((contAux1 == 0) and (contAux2 == 0) and (flag3 == 0))
				  {
					tempoViaEstab = Simulator::Now().GetSeconds();
					cout <<"\nVia estabilizada no tempo " <<tempoViaEstab <<"." <<endl;
					//freqInsObs[y] = 333.0 + tempoViaEstab;
					//freqRemObs = (499.5 + freqInsObs[z]);
					flag3 = 1;
				  }
			  }
			  
			
			  
			  /// INÍCIO DA VERIFICAÇÃO DA VELOCIDADE MÉDIA EM CADA TRECHO E TROCA DE TCT  
						trocaTCT(lista1, tempVehicle, highway->GetFirstIdVeh());
						trocaTCT(lista2, tempVehicle, highway->GetFirstIdVeh());
			  /// FIM DA VERIFICAÇÃO DA VELOCIDADE MÉDIA EM CADA TRECHO (AMBAS AS DIREÇÕES)
			  			  
			  // Controla a taxa de digulgação da taxa de acerto para à cada X segundos pós via estabilizada			  
			  if ((Simulator::Now().GetSeconds() >= (horaAvaliar + freqAtualiz)) and (snapshotPeriod))
			  {
				  //verTaxaAcerto(highway, (tempVehCruzouVia+freqAtualiz));
				  // newestab
				  //verTaxaAcerto(highway, (tempoViaEstab+freqAtualiz));
				  //snapshot(highway, (tempVehCruzouVia+freqAtualiz));
				  // newestab
				  snapshot(highway, (tempoViaEstab+freqAtualiz));
				  freqAtualiz += 60;
			  }
			  
			//}
			
			// Descomente o ELSE para gerar arquivo com estatísticas individuais dos veículos
			/* Adicionado por Thales (só executa este trecho se for veículo, ou seja, se não for AP nem Obstáculo)
			   ... Verificação necessária para não pegar veículo que, "senão for AP", pode ser obstáculo.*/
			/*else if (i >= highway->GetFirstIdVeh())
			{                 
			 // Arquivo geral com estatísticas de cada veículo (com modelo)
			 file << ""  
				  << Simulator::Now().GetSeconds()
				  << " " << tempVehicle->GetVehicleId() //Id do veiculo
				  << " " << tempVehicle->GetVelocity() //Velocidade
				  << " " << tempVehicle->GetPosition().x //Posicao x
				  << " " << tempVehicle->GetPosition().y //Posicao y
				  << " " << tempVehicle->GetDirection() //Direcao
				  << " " << tempVehicle->GetModel()->GetName() //Modelo
				  << endl;
				  
			 // Arquivo individual com estatísticas de cada veículo
			  temp << "data/veiculo/Veiculo" << tempVehicle->GetVehicleId();                  
			 vehFile.open(temp.str().c_str(), ios::app);                 
			 vehFile << ""
					 << Simulator::Now().GetSeconds()
					 << " " << tempVehicle->GetVelocity()
					 << " " << tempVehicle->GetVehicleId()
					 << " " << tempVehicle->GetPosition().x
					 << " " << tempVehicle->GetPosition().y
					 << " " << tempVehicle->GetDirection()
					 << endl;                        
			 vehFile.close();
			 temp.str("");
			 
			 // Arquivo individual com estatísticas de cada veículo dividido por tempo
			 temp << "data/veiculoTempo/Veiculo" << tempVehicle->GetVehicleId() << " " << time1 << "-" << time2;
			 vehFile.open(temp.str().c_str(), ios::app);                 
			 vehFile << ""
					 << Simulator::Now().GetSeconds()
					 << " " << tempVehicle->GetVelocity()
					 << " " << tempVehicle->GetVehicleId()
					 << " " << tempVehicle->GetPosition().x
					 << " " << tempVehicle->GetPosition().y
					 << " " << tempVehicle->GetDirection()
					 << endl;                         
			 vehFile.close();
			 temp.str("");
		}*/
	}
		cout <<'\r' << setw(5) << Simulator::Now().GetSeconds() << "/" << simTime;
  }
  
  // Insere obstáculos em ambas as direções à cada 30 minutos de simulação
  /// LEMBRAR QUE É PRECISO DESCOMENTAR A CRIAÇÃO DE OBSTÁCULOS EM POSIÇÕES INVÁLIDAS NO CONTROLLER.CC  
  if (obstacle and obDinamico)
  {
	  // 1obdin
	  //if ((Simulator::Now().GetSeconds() == freqInsObs[y]))
	  if ((Simulator::Now().GetSeconds() == tempoInsObs))
	  {
		  colocaObst(highway);
		  //y += 1;
		  //freqInsObs[y] = freqInsObs[y-1] + 333.0;	// 2000 seg
		  //freqInsObs[y] = freqInsObs[y-1] + 83.0;	// 500 seg
	  }
	  
	  // 1obdin
	  // Remove obstáculos após X minutos de sua criação
	  //if (Simulator::Now().GetSeconds() == freqRemObs)
	  if (Simulator::Now().GetSeconds() == tempoRemObs)
	  {
		  removeObst(highway);
		  //z += 1;
		  //freqRemObs = (freqInsObs[z] + 499.5);
		  //freqRemObs = freqInsObs[z] + 125.0; // 500 seg
	  }
  }
  
	/* testeaviso: Coleta todos os veículos que estão no range de um obstáculo e verifica se os mesmos
	 * ultrapassaram um obstáculo enviando para a função passouOb*/	
	if (avisoObst)
	{
		for (int i = (apQtd+1); i <= (apQtd+(2*obQtd)); i++)
		{ 
			tempVehicle2 = highway->FindVehicle(i);			
			if (tempVehicle2 != NULL) 
			{
				lista5 = highway->FindVehiclesInRange(tempVehicle2, 30, 1);
				lista6 = highway->FindVehiclesInRange(tempVehicle2, 30, -1);
				passouOb (lista5, tempVehicle2, highway->GetFirstIdVeh());
				passouOb (lista6, tempVehicle2, highway->GetFirstIdVeh());
			}
		}
	}
	// fim
		   
  file.close();
  Simulator::Schedule(Seconds(highway->GetDeltaT()), &VehData, highway);
}





int main(int argc, char *argv[]) {
	
	// process command-line args
	CommandLine cmd;
	cmd.AddValue("time", "simulation time", simTime);
	cmd.AddValue("apInitDist", "Initial position of the obstacle", apInitDist);
	cmd.AddValue("apDist", "Distance between obstacle", apDist);
	cmd.AddValue("apQtd", "Number of obstacles", apQtd);
	cmd.AddValue("hlength", "Highway length", highwayLength);
	cmd.AddValue("numOfLanes", "Number of lanes", numOfLanes);
	cmd.AddValue("medianGap", "Set median gap", medianGap);
	cmd.AddValue("injectionGap", "Minimum distance between two vehicles", injectionGap);
	cmd.AddValue("injectMixValue", "Set the ratio of car to truck", injectionMixValue);
	cmd.AddValue("obstacle", "Access Points as obstacle", obstacle);
	cmd.AddValue("apAleat", "", obAleat);
	cmd.AddValue("sedanVelocity", "Set max sedan velocity", sedanVelocity);
	cmd.AddValue("truckVelocity", "Set max truck velocity", truckVelocity);
	cmd.Parse(argc, argv);
	
	// Alterado por Thales (transforma verificação de corretude para vias uni e bi-direcionais)
	if (highwayLength < ((apQtd - 1) * apDist + (2*apInitDist)))
	{
		cout << "Invalid Highway Length" << endl;
		exit(1);
	}
	// Fim da alteração	
	if(apQtd < 2)
	{
	  cout<< "Invalid number of AP" << endl;
	  exit(1);
	}
	if(apInitDist < 0)
	{
	  cout << "Invalid initial distance" << endl;
	  exit(1);
	}
	if(obstacle == false && /*apAleat*/ obAleat == true)
	{
	  cout << "Invalid arguments for \"obstacle\" and \"obAleat\"" << endl;
	  exit(1);
	}
	
	// controle: dá erro caso não tenha obstáculos, mas a criação de obstáculo dinâmico esteja habilitada.
	if(obstacle == false && obDinamico == true)
	{
	  cout << "Invalid arguments for \"obstacle\" and \"obDinamico\"" << endl;
	  exit(1);
	}
	// fim

	Ptr<Highway> highway = CreateObject<Highway > ();			
	Ptr<Controller> controller = CreateObject<Controller > ();  
	controller->SetHighway(highway);

	highway->SetHighwayLength(highwayLength);
	highway->SetLaneWidth(5);
	highway->SetNumberOfLanes(numOfLanes);
	highway->SetChangeLane(true);
	highway->SetTwoDirectional(setTwoDirectional);
	highway->SetMedianGap(medianGap);
	highway->SetInjectionGap(injectionGap);
	highway->SetInjectionMixValue(injectionMixValue);
	highway->SetAutoInject(false);
	highway->SetDeltaT(0.1);
	highway->SetSedanVelocity(sedanVelocity);
	highway->SetTruckVelocity(truckVelocity);
	highway->SetApInitDist(apInitDist);
	highway->SetApDist(apDist);
	highway->SetApQtd(apQtd);	
	// Adicionado por Thales
	highway->SetObInitDist(obInitDist);
	highway->SetObDist(obDist);
	highway->SetObQtd(obQtd);
	highway->SetObAleat(obAleat);
	highway->SetAsObstacle(obstacle);
	// Inicializa variáveis contadoras da quantidade de carros e caminhões por direção
	highway->setContCarros1(0);
	highway->setContCarros2(0);
	highway->setContCaminhoes1(0);
	highway->setContCaminhoes2(0);
	/* controle: envia para highway se vai ter ou não aviso de obstáculos (usado na função DoLaneChangeIfPossible)
	 * e criação de obstáculo dinâmico (controller irá obter esta informação através do retorno do método GetObDinamico).*/
	highway->SetAvisoObst(avisoObst);
	highway->SetObDinamico(obDinamico);
	// fim
	

	// Change the transmission range of wifi shared in the Highway.
	highway->GetYansWifiPhyHelper().Set("TxPowerStart", DoubleValue(21.5)); // 250-300 meter transmission range 
	highway->GetYansWifiPhyHelper().Set("TxPowerEnd", DoubleValue(21.5)); // 250-300 meter transmission range 
	
	// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
	// Bind the Highway/Vehicle events to the event handlers. Controller's will catch them.  
	//highway->SetControlVehicleCallback(MakeCallback(&Controller::ControlVehicle, controller));
	highway->SetInitVehicleCallback(MakeCallback(&Controller::InitVehicle, controller));
	//highway->SetReceiveDataCallback(MakeCallback(&Controller::ReceiveData, controller));

	// Comentado pois a chamada de funções "Callback" estão gerando falhas de segmentação após algum tempo de simulação
	/*ns3::PacketMetadata::Enable();
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));*/

	
	Simulator::Schedule(Seconds(0.0), &Start, highway);	
	Simulator::Schedule(Seconds(simTime), &Stop, highway);
	
	ofstream vehDataFile;
	std::string line;
	
	//Adicionado por: Igor Quintanilha
	system("killall -9 vanet-highway 2>/dev/null");
	system("mkdir simulacoesAntigas 2>/dev/null");
	readDataFile.open("last_sim.opt");	
	//std::string line;
	stringstream op;
	if (readDataFile.is_open())
	{
	  getline(readDataFile, line);
	  if (line != "")
	  {
		op << "mv data " << "simulacoesAntigas/" << line << "";
		system(op.str().c_str());
	  }
	}	
	readDataFile.close();		
	
	system("mkdir data data/servidor data/veiculo data/veiculoTempo 2>/dev/null");
	// Adicionado por Thales
	system("mkdir data/TCTVeiculoTrecho data/DadosDosTrechos data/VelMediaTempoReal data/compTaxaAceTre data/VehFantasma data/MaiorMenorVelTre 2>/dev/null");
	// Fim da adição	
	system("rm -f data/veiculoTempo/* data/veiculo/Veiculo* data/vehData.log data/servidor/* 2>/dev/null");	
	// Adicionado por Thales
	system("rm -f data/TCTVeiculoTrecho/* data/DadosDosTrechos/* data/VelMediaTempoReal/* data/compTaxaAceTre/* data/VehFantasma/* data/MaiorMenorVelTre/* 2>/dev/null");
	// Fim da adição
		
	// Chama a função para inicializar com zero os array da taxa de acerto global e linhas válidas
	initPointArray (&pcontErroGlobal, &pqtdTrePercGlobal, trechos);
	initPointArray (&pcontErroTrecho, &pqtdTrePercTrecho, trechos);
	initPointArray (&pcontErroTreVizinho, &pqtdTrePercTreVizinho, trechos);
	initPointArray (&pcontErroTreVizDist, &pqtdTrePercTreVizDist, trechos);
	initPointArray (&pcontErroMesmaDir, &pqtdTrePercMesmaDir, trechos);
	initPointArray (&pcontErroMesmaDirFrente, &pqtdTrePercMesmaDirFrente, trechos);
	initPointArray (&pcontErroMesmaDirFrente1, &pqtdTrePercMesmaDirFrente1, trechos);
	initPointArray (&pcontErroMesmaDirFrente2, &pqtdTrePercMesmaDirFrente2, trechos);
	initPointArray (&pcontErroMesmaDirFrente3, &pqtdTrePercMesmaDirFrente3, trechos);
	initPointArray (&pcontErroFant, &pqtdTrePercFant, trechos);
	initPointArray (&contObst, (2*apQtd));
	
	// Inicializa as variáveis com simTime para que só se crie e remova obstáculos após via estabilizada
	//freqInsObs[y] = simTime; // via estabilizada  (3000 seg)
	//freqInsObs[y] = 333.0; // 5.3 minutos (2000 seg)
	//freqInsObs[y] = 83.0; // teste de 500 segundos 
	//freqRemObs = simTime; // via estabilizada  (3000 seg)
	//freqRemObs = (499.5 + freqInsObs[z]); // 2000 seg
	//freqRemObs = 125.0 + freqInsObs[z]; // 500 seg
		  
	time(&start);
	timeinfo = localtime(&start);
	cout << "Iniciando a simulacao: " << asctime(timeinfo);
	Simulator::Schedule(Seconds(0.0), &VehData, highway);
	Simulator::Schedule(Seconds(simTime), &ServerData, highway);    
	
	Simulator::Stop(Seconds(simTime));
	Simulator::Run();
	Simulator::Destroy();

	return 0;
}
