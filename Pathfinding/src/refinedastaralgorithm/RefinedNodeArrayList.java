package refinedastaralgorithm;

import java.util.ArrayList;

import utils.Node;
import es.upv.dsic.gti_ia.jgomas.Vector3D;


public class RefinedNodeArrayList {

	/*PRIVATE VARIABLES*/
	private ArrayList <RefinedNode> m_List = new ArrayList <RefinedNode>();
	private Heuristic m_Heuristic = new EuclideanDistanceHeuristic();
	
	
	/*PUBLIC METHODS FROM ARRAYLIST*/
	public void add(RefinedNode e){ m_List.add(e);}
	public RefinedNode get(int index){ return m_List.get(index);}
	public void remove(int index){m_List.remove(index);}
	public int size(){return m_List.size();}
	public boolean isEmpty(){return m_List.isEmpty();}
	
	/*OUR PUBLIC METHODS*/
	public boolean hasElement(){return !m_List.isEmpty();}
	
	public void addWithRNHeu(RefinedNode e, RefinedNode goal)
	{
		e.setdCostHeu(m_Heuristic.calcHeuristicByRefinedNode(e, goal));
		e.autoSetCostTot(); m_List.add(e);
	}
	
	public void addWithV3DHeu(RefinedNode e, Vector3D goal)
	{
		Vector3D eNode = new Vector3D();
		eNode.x=e.getiPosActx(); eNode.y=0.0; eNode.z=e.getiPosActz(); 
		e.setdCostHeu(m_Heuristic.calcHeuristicByVector3D(eNode, goal));
		e.autoSetCostTot(); m_List.add(e);
	}	
	
	public int searchNodeByCostMap(RefinedNode e)
	{
		for(int iPos = 0; iPos < m_List.size(); iPos++)
			if(e.NodeEqualByCostMap(m_List.get(iPos)))
				return iPos;
		return -1;
	}
	
	public int searchNodeByPos(RefinedNode e)
	{
		for(int iPos = 0; iPos < m_List.size(); iPos++)
			if(e.NodeEqualByPos(m_List.get(iPos)))
				return iPos;
		return -1;
	}
	
	public int getPosLowestCostTot()
	{
		if (m_List.isEmpty()) return -1;
		
		RefinedNode m_Menor = m_List.get(0);
		int pos = 0;
		
		for(int i = 1; i < m_List.size(); i++ )
			if(m_Menor.getdCostTot() > m_List.get(i).getdCostTot())
				{ m_Menor = m_List.get(i); pos = i; }
		
		return pos;
	}
	
	public double getLowestCostTot()
	{
		int pos = getPosLowestCostTot();
		if(pos == -1) {System.out.println("Error grave al hallar el menos coste total en lista"); return -1;}
		else return this.get(pos).getdCostTot();
	}
	
	public int getPosLowestCostAct()
	{
		if (m_List.isEmpty()) return -1;
		
		RefinedNode m_menor = m_List.get(0);
		int pos = 0;
		
		for(int i = 1; i < m_List.size(); i++ )
			if(m_menor.getdCostAct() > m_List.get(i).getdCostAct())
				{m_menor = m_List.get(i); pos = i;}
		
		return pos;
	}


}
