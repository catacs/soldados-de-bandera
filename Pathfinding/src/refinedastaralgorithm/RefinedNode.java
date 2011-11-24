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

	// Hay que cambiar los << por >> sino multiplica, no divide
	
	/*PRIVATE VARIABLES*/
	private int iPosActx; // de 0 a 255
	private int iPosActz;
	
	private double dCostAct; 
	private double dCostHeu;
	private double dCostTot;
	
	private RefinedNode m_Antecesor;	
	private NodeState m_Estado;
	
	/*CONSTRUCTORES*/
	
	public RefinedNode( int x, int z){ iPosActx=x; iPosActz=z;}
	public RefinedNode( int x, int z, int ca){ iPosActx=x; iPosActz=z; dCostAct=ca;}
	public RefinedNode( int x, int z, RefinedNode ca){ iPosActx=x; iPosActz=z; m_Antecesor = ca;}
	
	/*SETTERS AND GETTERS*/
	
	public void   setiPosActx(int x){ this.iPosActx = x;}
	public int getiPosActx(){ return this.iPosActx;}
	public void   setiPosActz(int z){ this.iPosActx = z;}
	public int getiPosActz(){ return this.iPosActz;}
	
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
		if(this.getiPosActx() != m_b.getiPosActx()) breturn = false;
		if(this.getiPosActz() != m_b.getiPosActz()) breturn = false;
		return breturn;
	}

	public boolean NodeEqualByCostMap(RefinedNode m_b)
	{
		boolean breturn = true;
		if((int)(this.getiPosActx()>>3) != (int)(m_b.getiPosActx()>>3)) breturn = false;
		if((int)(this.getiPosActz()>>3) != (int)(m_b.getiPosActz()>>3)) breturn = false;		
		return breturn;
	}
	
	public RefinedNode CloneNodeSquare(int ix, int iz)
	{
		RefinedNode m_Devolver = new 
			RefinedNode(this.getiPosActx()+ix, this.getiPosActz()+iz, this);
		m_Devolver.setdCostAct(this.dCostAct + Math.abs(ix) + Math.abs(iz));
		return m_Devolver;
	}
	
	public RefinedNode CloneNodeDiag(int dx, int dz)
	{
		RefinedNode m_Devolver = new 
			RefinedNode(this.getiPosActx()+dx, this.getiPosActz()+dz, this);
		double cost = Math.sqrt( Math.abs(dx*dx) + Math.abs(dz*dz));
		m_Devolver.setdCostAct(this.dCostAct + cost);
		return m_Devolver;
	}
	
	public RefinedNodeArrayList generateSuccesor(Vector3D Dest, CTerrainMap m_Mapa)
	{
		RefinedNodeArrayList m_Succesors = new RefinedNodeArrayList();
		
	    RefinedNode m_Arriba = this.CloneNodeSquare(0, 8);
	    if(m_Mapa.CanWalk(m_Arriba.getiPosActx()>>3, m_Arriba.getiPosActz()>>3))
	    	m_Succesors.add(m_Arriba);
		
	    RefinedNode m_Abajo = this.CloneNodeSquare(0, -8);
	    if(m_Mapa.CanWalk(m_Abajo.getiPosActx()>>3, m_Abajo.getiPosActz()>>3))
	    	m_Succesors.add(m_Abajo);
		
	    RefinedNode m_Izquierda = this.CloneNodeSquare(8, 0);
	    if(m_Mapa.CanWalk(m_Izquierda.getiPosActx()>>3, m_Izquierda.getiPosActz()>>3))
	    	m_Succesors.add(m_Izquierda);
		
	    RefinedNode m_Derecha = this.CloneNodeSquare(-8, 0);
	    if(m_Mapa.CanWalk(m_Derecha.getiPosActx()>>3, m_Derecha.getiPosActz()>>3))
	    	m_Succesors.add(m_Derecha);
		
		return m_Succesors;
	}

	public RefinedNodeArrayList generateRefinedSuccesor( CTerrainMap m_Mapa, int iRefined, int dDiagStep)
	{
		RefinedNodeArrayList m_Succesors = new RefinedNodeArrayList();
		
		int iSectorx = (this.getiPosActx())>>3;
		int iSectorz = (this.getiPosActz())>>3;
		
		//Sucesores Cardinales
		
	    RefinedNode m_Arriba = this.CloneNodeSquare(0, (iSectorz<<3) - getiPosActz() - iRefined );
	    if(m_Mapa.CanWalk((m_Arriba.getiPosActx())>>3, (m_Arriba.getiPosActz())>>3))
	    	m_Succesors.add(m_Arriba);
	    
	    RefinedNode m_Abajo = this.CloneNodeSquare(0, ((iSectorz+1)<<3) - getiPosActz() + iRefined);
	    if(m_Mapa.CanWalk((m_Abajo.getiPosActx())>>3, (m_Abajo.getiPosActz())>>3))
	    	m_Succesors.add(m_Abajo);
	    
	    RefinedNode m_Izquierda = this.CloneNodeSquare((iSectorx<<3) - getiPosActx() - iRefined, 0);
	    if(m_Mapa.CanWalk((m_Izquierda.getiPosActx())>>3, (m_Izquierda.getiPosActz())>>3))
	    	m_Succesors.add(m_Izquierda);
	    
	    RefinedNode m_Derecha = this.CloneNodeSquare(((iSectorx+1)<<3) - getiPosActx() + iRefined, 0);
	    if(m_Mapa.CanWalk((m_Derecha.getiPosActx())>>3, (m_Derecha.getiPosActz())>>3))
	    	m_Succesors.add(m_Derecha);
	    
	    //Sucesores diagonales
  
	    RefinedNode m_ArrIzq = this.CloneNodeDiag((iSectorx<<3) - getiPosActx() - iRefined, (iSectorz<<3) - getiPosActz() - iRefined );
	    boolean bAdd = false;
	    
	    if(m_Mapa.CanWalk((m_ArrIzq.getiPosActx())>>3, (m_ArrIzq.getiPosActz())>>3))
	    	if(m_Mapa.CanWalk(((m_ArrIzq.getiPosActx())>>3)+1, (m_ArrIzq.getiPosActz())>>3))
	    		if(m_Mapa.CanWalk((m_ArrIzq.getiPosActx())>>3, ((m_ArrIzq.getiPosActz())>>3)+1))
	    			if(m_Mapa.CanWalk(((m_ArrIzq.getiPosActx())>>3)+1, ((m_ArrIzq.getiPosActz())>>3)+1))
		    			bAdd = true;
	    
	    if(bAdd) m_Succesors.add(m_ArrIzq);
	
	    RefinedNode m_ArrDer = this.CloneNodeDiag(((iSectorx+1)<<3) - getiPosActx() + iRefined,(iSectorz<<3) - getiPosActz() - iRefined);
	    bAdd = false;
	    if(m_Mapa.CanWalk((m_ArrDer.getiPosActx())>>3, (m_ArrDer.getiPosActz())>>3))
	    	if(m_Mapa.CanWalk(((m_ArrDer.getiPosActx())>>3)-1, (m_ArrDer.getiPosActz())>>3))
	    		if(m_Mapa.CanWalk((m_ArrDer.getiPosActx())>>3, ((m_ArrDer.getiPosActz())>>3)+1))
	    			if(m_Mapa.CanWalk(((m_ArrDer.getiPosActx())>>3)-1, ((m_ArrDer.getiPosActz())>>3)+1))
	    			bAdd = true;
	    
	    if(bAdd) m_Succesors.add(m_ArrDer);

	    bAdd = false;
	    RefinedNode m_AbaDer = this.CloneNodeDiag(((iSectorx+1)<<3) - getiPosActx() + iRefined,((iSectorz+1)<<3) -getiPosActz() + iRefined);
	    if(m_Mapa.CanWalk((m_AbaDer.getiPosActx())>>3, (m_AbaDer.getiPosActz())>>3))
	    	if(m_Mapa.CanWalk(((m_AbaDer.getiPosActx())>>3)-1, (m_AbaDer.getiPosActz())>>3))
	    		if(m_Mapa.CanWalk((m_AbaDer.getiPosActx())>>3, ((m_AbaDer.getiPosActz())>>3)-1))
	    			if(m_Mapa.CanWalk(((m_AbaDer.getiPosActx())>>3)-1, ((m_AbaDer.getiPosActz())>>3)-1))
	    			bAdd = true;
	    if(bAdd) m_Succesors.add(m_AbaDer);

	    bAdd = false;
	    RefinedNode m_AbaIzq = this.CloneNodeDiag((iSectorx<<3) - getiPosActx() - iRefined,((iSectorz+1)<<3) - getiPosActz() + iRefined);
	    if(m_Mapa.CanWalk((m_AbaIzq.getiPosActx())>>3, (m_AbaIzq.getiPosActz())>>3))
	    	if(m_Mapa.CanWalk((m_AbaIzq.getiPosActx())>>3, ((m_AbaIzq.getiPosActz())>>3)-1))
	    		if(m_Mapa.CanWalk(((m_AbaIzq.getiPosActx())>>3)+1, (m_AbaIzq.getiPosActz())>>3))
	    			if(m_Mapa.CanWalk(((m_AbaIzq.getiPosActx())>>3)+1, ((m_AbaIzq.getiPosActz())>>3)-1))
	    			bAdd = true;
	    if(bAdd) m_Succesors.add(m_AbaIzq);
	 
		return m_Succesors;
	}
	public void ShowRefinedNode(){System.out.println("X: " + this.iPosActx + " Z: " + this.iPosActz);}
}
