package refinedastaralgorithm;
import student.*;
import java.util.ArrayList;

import utils.Node;
import utils.NodeState;

import es.upv.dsic.gti_ia.jgomas.CTerrainMap;
import es.upv.dsic.gti_ia.jgomas.Vector3D;
import java.lang.Math;
public class RefinedPathAStarAlgorithm extends PathFinder {
	
	private CTerrainMap m_Map=null;
	
	@Override
	public void setMap(CTerrainMap map) {
		this.m_Map=map;	
	}
	
	@Override
	public Vector3D[] getPath(Vector3D source, Vector3D target) {
		int dRefined = 1;
		int dDiagonalStep = 2;
		
		boolean bFindBestPath = false;
		
		RefinedNode m_Goal = new RefinedNode((int)target.x,(int) target.z);
		m_Goal.setState(NodeState.End);
		
		RefinedNode m_Start;
		
		m_Start = new RefinedNode((int)source.x, (int)source.z);
		
		m_Start.setdCostAct(0);
		m_Start.setState(NodeState.Start);
		RefinedNodeArrayList m_Open = new RefinedNodeArrayList();
		m_Open.addWithRNHeu(m_Start, m_Goal);
	
		RefinedNodeArrayList m_Closed = new RefinedNodeArrayList();
		
		while (m_Open.hasElement() && !bFindBestPath)
		{
			int iPosCurrent = m_Open.getPosLowestCostTot();
			RefinedNode m_Current = m_Open.get(iPosCurrent);
			m_Open.remove(iPosCurrent);
			if(m_Current.NodeEqualByCostMap(m_Goal))
			{
				//Encuentra el camino hasta el cuadrado de coste final.
				m_Closed.add(m_Current);
				bFindBestPath = true;
				System.out.println("Encontrado el camino optimo");
				// punto de terminacion del algoritmo
			}
			else
			{
				RefinedNodeArrayList m_Succesors = m_Current.generateRefinedSuccesor(m_Map, dRefined, dDiagonalStep);
				while(m_Succesors.hasElement())
				{
					
					RefinedNode m_Succesor = m_Succesors.get(0);
					m_Succesors.remove(0);
					
					int iPosSucOp = m_Open.searchNodeByPos(m_Succesor);
					int iPosSucClosed = m_Closed.searchNodeByPos(m_Succesor);
					
					if(iPosSucOp >= 0)
						if((m_Open.get(iPosSucOp).getdCostAct() <= m_Succesor.getdCostAct()) )
							m_Succesor = null;
					
					if(iPosSucClosed >= 0 && m_Succesor != null) 
						if((m_Closed.get(iPosSucClosed).getdCostAct() <= m_Succesor.getdCostAct())) // En closed)
							m_Succesor = null;
							
					if(m_Succesor != null)
					{
						if(iPosSucOp >= 0) m_Open.remove(iPosSucOp);
						if(iPosSucClosed >= 0) m_Closed.remove(iPosSucClosed);
						m_Open.addWithRNHeu(m_Succesor, m_Goal);
					}
				}
			} // Ya se han analizado todos los sucesores
			m_Closed.add(m_Current);
		}
		if(bFindBestPath){ return calcularPath(m_Closed.get(m_Closed.size()-1), target, (int)source.x, (int)source.z); }
		else {return null;}
	}
	
	public Vector3D[] calcularPath(RefinedNode m_Goal, Vector3D m_Target, int iSx, int iSz)
	{
		System.out.println("Calculamos el camino:");
		
		RefinedNode m_Actual = m_Goal;
		ArrayList<Vector3D> path = new ArrayList<Vector3D>();
		
		path.add(new Vector3D(m_Target.x, m_Map.GetTargetY() ,m_Target.y));	
		m_Actual = m_Actual.getAntecesor();
		
		while(m_Actual.getState() != NodeState.Start)
		{
			path.add(new Vector3D(m_Actual.getiPosActx(), m_Map.GetTargetY() ,m_Actual.getiPosActz()));	
			m_Actual = m_Actual.getAntecesor();
		}
		
		path.add(new Vector3D(iSx, m_Map.GetTargetY() ,iSz));
		
		Vector3D[] m_AStarPath = new Vector3D[path.size()];		
		int iPos = path.size()-1;	
		int iP=0;
		while(iPos >= 0 )
		{
			m_AStarPath[iP] = path.get(iPos);
			iP++;
			iPos -= 1;
		}
		System.out.println("Tamano: " + m_AStarPath.length);
		return m_AStarPath;
	}
	
}


