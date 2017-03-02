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

#include "LaneChange.h"

namespace ns3
{
  TypeId LaneChange::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::LaneChange")
      .SetParent<Object> ()
      .AddConstructor<LaneChange> ()
      ;
    return tid;
  }
	
  LaneChange::LaneChange(){;}

  double LaneChange::GetPolitenessFactor()
  {
    return m_politenessFactor;
  }

  void LaneChange::SetPolitenessFactor(double value)
  {
    m_politenessFactor=value;
  }
	
  double LaneChange::GetDbThreshold()
  {
    return m_dbThreshold;
  }

  void LaneChange::SetDbThreshold(double value)
  {
    m_dbThreshold=value;
  }
  // 2 metros
  /* Retirado os métodos que manipulam o atributo da distância de segurança que ficava no modelo 
   * de troca de pista, uma vez que o mesmo não é mais usado após transferir o atributo para o veículo.*/
  /*double LaneChange::GetGapMin()
  {
    return m_gapMin;
  }
	
  void LaneChange::SetGapMin(double value)
  {
    m_gapMin=value;
  }*/

  double LaneChange::GetMaxSafeBreakingDeceleration()
  {
    return m_maxSafeBreakingDeceleration;
  }
	
  void LaneChange::SetMaxSafeBreakingDeceleration(double value)
  {
    m_maxSafeBreakingDeceleration=value;
  }
 
  double LaneChange::GetBiasRight()
  {
    return m_biasRight;
  }
  
  void LaneChange::SetBiasRight(double value)
  {
    m_biasRight=value;
  }

  bool LaneChange::CheckLaneChange(Ptr<Vehicle> me, Ptr<Vehicle> fOld, Ptr<Vehicle> fNew, Ptr<Vehicle> bNew, bool toLeft)
  {
    double fNewPosition,bNewPosition, bNewLength;
    double bNew_acc;
    double others_disadv=0;
    /* Criação de duas variáveis para receber a distância de segurança do veículo que quer trocar de pista,
     * e o veículo que estará imediatamente atrás do primeiro quando este trocar de pista */
    double distSegDinMe, distSegDinBNew;
    // Pego a distância de segurança proporcional à velocidade do veículo que quer mudar de pista
    distSegDinMe = me->GetDistSegDin();

    if (fNew == 0/*null*/)
      {
        fNewPosition = (me->GetDirection()) * 100000000000000.0;//Double.MaxValue;               
      }
    else
      {
        fNewPosition = fNew->GetPosition().x;
      }
 
    if (bNew == 0/*null*/)
      {
        bNewPosition = -100000000000000.0 * (me->GetDirection());//Double.MinValue;
        bNewLength = me->GetLength();
        bNew_acc = -m_maxSafeBreakingDeceleration + 1;
        /* Caso não tenha nenhum veículo que fica atrás do veículo que quer mudar de pista após o mesmo
         * mudar, não é possível fazer um get em sua distância proporcional, devendo então colocar-se um valor alegórico.*/
        distSegDinBNew = 10.0;
      }
    else
      {
        bNewPosition = bNew->GetPosition().x;
        bNewLength = bNew->GetLength();
        bNew_acc = bNew->Acceleration(me);
        /* Caso haja um veículo na outra pista que ficará atrás do veículo que quer mudar após o mesmo
         * trocar de pista, a distância de segurança deste para o veículo que quer trocar de pista deverá ser proporcional
         * à sua velocidade */
        distSegDinBNew = bNew->GetDistSegDin();
      }

    if (fNew != 0/*null*/ && bNew != 0 /*null*/)
      {
        others_disadv = bNew->Acceleration(fNew) - bNew_acc;
      }

    double gapFront = (me->GetDirection())*(fNewPosition - me->GetPosition().x) - me->GetLength();
    double gapBack = (me->GetDirection())*(me->GetPosition().x - bNewPosition) - bNewLength;

	/* Substituição do valor estático da distância de segurança, presente no modelo de troca de pista, 
	 * para o valor específico proporcional à velocidade de cada um dos veículos envolvidos na condição. */
    // 2 metros
    //if ((gapFront > m_gapMin) && (gapBack > m_gapMin) && (bNew_acc > - m_maxSafeBreakingDeceleration))
    if ((gapFront > distSegDinMe) && (gapBack > distSegDinBNew) && (bNew_acc > - m_maxSafeBreakingDeceleration))
      {
        double my_adv = me->Acceleration(fNew) - me->Acceleration(fOld) + ((toLeft) ? -1 : 1) * m_biasRight;
            
        if (others_disadv < 0)
          {
            others_disadv = 0;
          }
          
        /* avisocorrigido: facilita a troca de pista caso tenha obstáculo no trecho em que o veículo vai entrar, 
         * onde troca de pista sem avaliar se vai ter vantagem de aceleração ou se causa desvantagem na aceleração
         * do novo de trás. */        
        if (me->GetValObst().pista == me->GetLane())
          {
            return true;
          }
        // fim
        
        if (my_adv - m_politenessFactor * others_disadv > m_dbThreshold)
          {
            return true;
          }
        else
          {
            return false;
          }
      }
    else
      {
        return false;
      }
  }
}
