package utils;
import java.util.ArrayList;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Random;


import es.upv.dsic.gti_ia.jgomas.CBasicTroop;
import es.upv.dsic.gti_ia.jgomas.CMedic;
import es.upv.dsic.gti_ia.jgomas.CMobile;
import es.upv.dsic.gti_ia.jgomas.CPack;
import es.upv.dsic.gti_ia.jgomas.CSight;
import es.upv.dsic.gti_ia.jgomas.CTask;
import es.upv.dsic.gti_ia.jgomas.CTerrainMap;
import es.upv.dsic.gti_ia.jgomas.Vector3D;

public class Node {
	
	
	private double dPosActx; // de 0 a 255
	private double dPosActz;
	
	private double dCostAct; 
	private double dCostHeu;
	private double dCostTot;
	
	private Node m_Antecesor;
		
	private NodeState m_Estado;
	
	public Node(double dx, double dz, double dca)
	{
		dPosActx = dx;
		dPosActz = dz;
		dCostAct = dca;
	}
	public Node(double dx, double dz, double dca, double dch)
	{
		dPosActx = dx;
		dPosActz = dz;
		dCostAct = dca;
		dCostHeu = dch;
		dCostTot = dca + dch;
	}
	
	public void setCostAct(double d)
	{
		dCostAct = d;
	}
	
	public Node getAntecesor()
	{
		return m_Antecesor;
	}
	
	public void setAntecesor(Node m_Temp)
	{
		this.m_Antecesor = m_Temp;
	}
	
	public double getCostAct()
	{
		return dCostAct;
	}
		
	public double getdPosActx()
	{
		return dPosActx;
	}
	
	public double getdPosActz()
	{
		return dPosActz;
	}
	
	public double getCostTot()
	{
		return dCostTot;
	}
	
	public void setState(NodeState estado)
	{
		m_Estado = estado;
	}
	
	public NodeState getState()
	{
		return this.m_Estado;
	}
	
	
	public void set_Heuristica(Vector3D m_PosAct, Vector3D m_Posfin ){
		Vector3D m_dist = new Vector3D();
		m_dist.x = m_PosAct.x - m_Posfin.x;
		m_dist.z = m_PosAct.z - m_Posfin.z;
		m_dist.y = 0.0f;
		dCostHeu = m_dist.x + m_dist.z;
		dCostTot = dCostHeu + dCostAct;
	}
	
	public void set_HeuristicaConPosActual(Vector3D m_PosFin)
	{	
		Vector3D m_dist = new Vector3D();
		m_dist.x = dPosActx - m_PosFin.x;
		m_dist.z = dPosActz - m_PosFin.z;
		m_dist.y = 0.0f;
		dCostHeu = (m_dist.x  + m_dist.z);
		dCostTot = dCostHeu + dCostAct;
	}
	
	public boolean NodeEqualByPos(Node m_b)
	{
		boolean breturn = true;
		
		if(this.getdPosActx() != m_b.getdPosActx()) breturn = false;
		if(this.getdPosActz() != m_b.getdPosActz()) breturn = false;
		
		return breturn;
	}
	
	public boolean NodeEqualByCostMap(Node m_b)
	{
		boolean breturn = true;
		
		if((int)(this.getdPosActx()/8) != (int)(m_b.getdPosActx()/8)) breturn = false;
		if((int)(this.getdPosActz()/8) != (int)(m_b.getdPosActz()/8)) breturn = false;
		
		return breturn;
	}
	
	public double absoluteDouble(double dx)
	{
		if( dx < 0) return -dx;
		else return dx;
	}
	
	public Node CloneNode(double dx, double dz, Vector3D m_Dest)
	{
		// Le das un nodo y te suma a x y z lo que le des
		Node m_devolver = new Node(this.getdPosActx()+dx, this.getdPosActz()+dz, this.getCostAct()+ absoluteDouble(dx) + absoluteDouble(dz));
		m_devolver.set_HeuristicaConPosActual(m_Dest);
		m_devolver.setAntecesor(this);
		m_devolver.setState(NodeState.Path);
		return m_devolver;
	}
	
	public int searchNodeInArrayList(ArrayList <Node> m_List)
	{
		for(int iPos = 0; iPos < m_List.size(); iPos++)
		{
			if(this.NodeEqualByCostMap(m_List.get(iPos)))
			{
				return iPos;
			}
		}
		
		return -1;
	}
	
	public int searchNodeInArrayListByPos(ArrayList <Node> m_List)
	{
		for(int iPos = 0; iPos < m_List.size(); iPos++)
		{
			if(this.NodeEqualByPos(m_List.get(iPos)))
			{
				return iPos;
			}
		}
		
		return -1;
	}
	
	public ArrayList <Node> generateSuccesor(Node Dest, CTerrainMap m_Mapa, double dStep)
	{
		ArrayList <Node> m_Succesors = new ArrayList <Node>();
		
		Vector3D m_Objective = new Vector3D();
		m_Objective.x = Dest.dPosActx;
		m_Objective.z = Dest.dPosActz;
		
	    Node m_Arriba = this.CloneNode(0.0f, dStep, m_Objective);
	    if(m_Mapa.CanWalk((int) (m_Arriba.getdPosActx()/8), (int) (m_Arriba.getdPosActz()/8)))
	    	m_Succesors.add(m_Arriba);
		
	    Node m_Abajo = this.CloneNode(0.0f, -dStep, m_Objective);
	    if(m_Mapa.CanWalk((int) (m_Abajo.getdPosActx()/8), (int) (m_Abajo.getdPosActz()/8)))
	    	m_Succesors.add(m_Abajo);
		
	    Node m_Izquierda = this.CloneNode(dStep, 0.0f, m_Objective);
	    if(m_Mapa.CanWalk((int) (m_Izquierda.getdPosActx()/8), (int) (m_Izquierda.getdPosActz()/8)))
	    	m_Succesors.add(m_Izquierda);
		
	    Node m_Derecha = this.CloneNode(-dStep, 0.0f, m_Objective);
	    if(m_Mapa.CanWalk((int) (m_Derecha.getdPosActx()/8), (int) (m_Derecha.getdPosActz()/8)))
	    	m_Succesors.add(m_Derecha);
		
		return m_Succesors;
	}
	
	public void ShowNode()
	{
		System.out.println("X: " + this.dPosActx + " Z: " + this.dPosActz);
	}
	
}
