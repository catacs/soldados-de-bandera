package refinedastaralgorithm;

import java.util.ArrayList;
import utils.NodeState;

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
import java.lang.Math;
public class RefinedNode {
	
	/*PRIVATE VARIABLES*/
	private double dPosActx; // de 0 a 255
	private double dPosActz;
	
	private double dCostAct; 
	private double dCostHeu;
	private double dCostTot;
	
	private RefinedNode m_Antecesor;	
	private NodeState m_Estado;
	
	/*CONSTRUCTORES*/
	
	public RefinedNode( double x, double z){ dPosActx=x; dPosActz=z;}
	public RefinedNode( double x, double z, double ca){ dPosActx=x; dPosActz=z; dCostAct=ca;}
	public RefinedNode( double x, double z, RefinedNode ca){ dPosActx=x; dPosActz=z; m_Antecesor = ca;}
	
	/*SETTERS AND GETTERS*/
	
	public void   setdPosActx(double x){ this.dPosActx = x;}
	public double getdPosActx(){ return this.dPosActx;}
	public void   setdPosActz(double z){ this.dPosActx = z;}
	public double getdPosActz(){ return this.dPosActz;}
	
	public void   setdCostAct(double x){ this.dCostAct = x;}
	public double getdCostAct(){ return this.dCostAct;}
	public void   setdCostHeu(double x){ this.dCostHeu = x;}
	public double getdCostHeu(){ return this.dCostHeu;}
	public void   setdCostTot(double x){ this.dCostTot = x;}
	public double getdCostTot(){ return this.dCostTot;}
	public void   autoSetCostTot(){ this.dCostTot = this.dCostAct + this.dCostHeu;}
	
	public RefinedNode getAntecesor(){return this.m_Antecesor;}
	public void setAntecesor(RefinedNode x){this.m_Antecesor = x;}
	public NodeState getState(){return this.m_Estado;}
	public void setState(NodeState e){this.m_Estado = e;}
	
	/*PUBLIC METHODS*/
	
	public boolean NodeEqualByPos(RefinedNode m_b)
	{
		boolean breturn = true;
		if(this.getdPosActx() != m_b.getdPosActx()) breturn = false;
		if(this.getdPosActz() != m_b.getdPosActz()) breturn = false;
		return breturn;
	}

	public boolean NodeEqualByCostMap(RefinedNode m_b)
	{
		boolean breturn = true;
		if((int)(this.getdPosActx()/8) != (int)(m_b.getdPosActx()/8)) breturn = false;
		if((int)(this.getdPosActz()/8) != (int)(m_b.getdPosActz()/8)) breturn = false;		
		return breturn;
	}
	
	public RefinedNode CloneNode(double dx, double dz)
	{
		RefinedNode m_Devolver = new 
			RefinedNode(this.getdPosActx()+dx, this.getdPosActz()+dz, this);
		m_Devolver.setdCostAct(this.dCostAct+ Math.abs(dx)+Math.abs(dz));
		return m_Devolver;
	}
	
	public RefinedNodeArrayList generateSuccesor(RefinedNode Dest, CTerrainMap m_Mapa)
	{
		RefinedNodeArrayList m_Succesors = new RefinedNodeArrayList();
		
		Vector3D m_Objective = new Vector3D();
		m_Objective.x = Dest.dPosActx;
		m_Objective.z = Dest.dPosActz;
		
	    RefinedNode m_Arriba = this.CloneNode(0.0f, 8.0);
	    if(m_Mapa.CanWalk((int) (m_Arriba.getdPosActx()/8), (int) (m_Arriba.getdPosActz()/8)))
	    	m_Succesors.add(m_Arriba);
		
	    RefinedNode m_Abajo = this.CloneNode(0.0f, -8.0);
	    if(m_Mapa.CanWalk((int) (m_Abajo.getdPosActx()/8), (int) (m_Abajo.getdPosActz()/8)))
	    	m_Succesors.add(m_Abajo);
		
	    RefinedNode m_Izquierda = this.CloneNode(8.0, 0.0f);
	    if(m_Mapa.CanWalk((int) (m_Izquierda.getdPosActx()/8), (int) (m_Izquierda.getdPosActz()/8)))
	    	m_Succesors.add(m_Izquierda);
		
	    RefinedNode m_Derecha = this.CloneNode(-8.0, 0.0f);
	    if(m_Mapa.CanWalk((int) (m_Derecha.getdPosActx()/8), (int) (m_Derecha.getdPosActz()/8)))
	    	m_Succesors.add(m_Derecha);
		
		return m_Succesors;
	}

	public RefinedNodeArrayList generateRefinedSuccesor(RefinedNode Dest, CTerrainMap m_Mapa, double dRefined)
	{
		RefinedNodeArrayList m_Succesors = new RefinedNodeArrayList();
		
		Vector3D m_Objective = new Vector3D();
		m_Objective.x = Dest.dPosActx;
		m_Objective.z = Dest.dPosActz;
		
		int iSectorx = (int) (this.getdPosActx()/8.0);
		int iSectorz = (int) (this.getdPosActz()/8.0);
		
	    RefinedNode m_Arriba = this.CloneNode(0.0f, iSectorz*8 - getdPosActz() - dRefined );
	    if(m_Mapa.CanWalk((int) (m_Arriba.getdPosActx()/8), (int) (m_Arriba.getdPosActz()/8)))
	    	m_Succesors.add(m_Arriba);
		
	    RefinedNode m_Abajo = this.CloneNode(0.0f, (iSectorz+1)*8 - getdPosActz() + dRefined);
	    if(m_Mapa.CanWalk((int) (m_Abajo.getdPosActx()/8), (int) (m_Abajo.getdPosActz()/8)))
	    	m_Succesors.add(m_Abajo);
		
	    RefinedNode m_Izquierda = this.CloneNode(iSectorx*8 - getdPosActx() - dRefined, 0.0f);
	    if(m_Mapa.CanWalk((int) (m_Izquierda.getdPosActx()/8), (int) (m_Izquierda.getdPosActz()/8)))
	    	m_Succesors.add(m_Izquierda);
		
	    RefinedNode m_Derecha = this.CloneNode((iSectorx+1)*8 - getdPosActx() + dRefined, 0.0f);
	    if(m_Mapa.CanWalk((int) (m_Derecha.getdPosActx()/8), (int) (m_Derecha.getdPosActz()/8)))
	    	m_Succesors.add(m_Derecha);
		
		return m_Succesors;
	}

	
	
	
	public void ShowRefinedNode(){System.out.println("X: " + this.dPosActx + " Z: " + this.dPosActz);}
	
}
