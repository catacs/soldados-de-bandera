package student;

import java.util.ArrayList;

import utils.Node;
import utils.NodeState;

import es.upv.dsic.gti_ia.jgomas.CTerrainMap;
import es.upv.dsic.gti_ia.jgomas.Vector3D;

public class PathAStarAlgorithm extends PathFinder {
	
	private CTerrainMap m_Map=null;
	
	
	
	@Override
	public void setMap(CTerrainMap map) {
		this.m_Map=map;
		
	}
	@Override
	public Vector3D[] getPath(Vector3D source, Vector3D target) {
		double dStep = 8.0;
		Node m_Goal = new Node(target.x, target.z, 0.0f, 0.0f);
		m_Goal.setState(NodeState.End);
		Node m_Start = new Node(source.x, source.z, 0.0f);
		m_Start.setState(NodeState.Start);
		m_Start.set_Heuristica(source,target);
		
		ArrayList <Node> m_open = new ArrayList <Node>();
		m_open.add(m_Start);
		
		ArrayList <Node> m_Closed = new ArrayList <Node>();
		
		while (!m_open.isEmpty())
		{
			int iPosCurrent = getLowestCostAct(m_open);
			Node m_Current = m_open.get(iPosCurrent);
			m_open.remove(iPosCurrent);
			
			if(m_Current.NodeEqualByCostMap(m_Goal))
			{
				m_Closed.add(m_Current);
				m_Goal.setAntecesor(m_Current);
				m_Closed.add(m_Goal);
				
				/**********Una vez completo todo calculo el camino con los nodos************/
				
				System.out.println("El path es valido");
				
				int iPath = 0;
				
				Node m_Actual = m_Goal;
				
				if(m_Actual.getState() != NodeState.End) System.out.println("No es el nodo final!!");
				
				ArrayList<Vector3D> path= new ArrayList<Vector3D>();
				
				while(m_Actual.getState() != NodeState.Start)
				{
					path.add(new Vector3D(m_Actual.getdPosActx(), m_Map.GetTargetY() ,m_Actual.getdPosActz()));
					
					m_Actual = m_Actual.getAntecesor();
					
				}
				
				Vector3D[] m_AStarPath = new Vector3D[path.size()];
							
				int iPos = path.size()-1;	
				int iP=0;
				while(iPos >= 0 )
				{
					m_AStarPath[iP] = path.get(iPos);
					iP++;
					iPos -= 1;
				}
				
				
				/*********************/
				return m_AStarPath;			
			}
			else
			{
				ArrayList <Node> m_Succesors = m_Current.generateSuccesor(m_Goal, m_Map, dStep);
				
				while(!m_Succesors.isEmpty())
				{
					Node m_Succesor = m_Succesors.get(0);
					m_Succesors.remove(0);
					
					int iPosSucOp = m_Succesor.searchNodeInArrayList(m_open);
					int iPosSucClosed = m_Succesor.searchNodeInArrayList(m_Closed);
					
					if(iPosSucOp >= 0)
					{
						if((m_open.get(iPosSucOp).getCostAct() <= m_Succesor.getCostAct()) )
						{// Se ha encontrado el nodo en open
							m_Succesor = null;
						}
					}
					
					if(iPosSucClosed >= 0) 
						{
							if((m_Closed.get(iPosSucClosed).getCostAct() <= m_Succesor.getCostAct())) // En closed)
							{
								m_Succesor = null;
							}
					}
					if(m_Succesor != null)
					{
						if(iPosSucOp >= 0) m_open.remove(iPosSucOp);
						if(iPosSucClosed >= 0) m_Closed.remove(iPosSucClosed);
						m_open.add(m_Succesor);
					}
				}
			} // Ya se han analizado todos los sucesores
			m_Closed.add(m_Current);
		}
	return null;
}
	
	private int getLowestCostTot(ArrayList<Node> m_list)
	{
		if (m_list.isEmpty())
			return -1;
		
		Node m_menor = m_list.get(0);
		int pos = 0;
		
		for(int i = 1; i < m_list.size(); i++ )
		{
			if(m_menor.getCostTot() > m_list.get(i).getCostTot())
			{
				m_menor = m_list.get(i);
				pos = i;
			}
		}
		return pos;
	}
	
	private int getLowestCostAct(ArrayList<Node> m_list)
	{
		if (m_list.isEmpty())
			return -1;
		
		Node m_menor = m_list.get(0);
		int pos = 0;
		
		for(int i = 1; i < m_list.size(); i++ )
		{
			if(m_menor.getCostAct() > m_list.get(i).getCostAct())
			{
				m_menor = m_list.get(i);
				pos = i;
			}
		}
		return pos;
	}
	
}


