package student;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;

import es.upv.dsic.gti_ia.jgomas.CTerrainMap;
import es.upv.dsic.gti_ia.jgomas.Vector3D;

public class PathDijkstra extends PathFinder {
	
	private CTerrainMap m_Map=null;
	private int[][] m_iNodeMatrix= null;
	private int m_iXSize,m_iZSize,m_iNodes;

	@Override
	public void setMap(CTerrainMap map) {

		m_Map=map;
		
		m_iXSize=m_Map.GetSizeX();//tamaño en X
		m_iZSize=m_Map.GetSizeZ();//tamaño en Z
		m_iNodes=m_iXSize*m_iZSize;//numero de nodos -> cada punto es un nodo
		
		generateNodeMatrix();
	}

	private void generateNodeMatrix() {
		
		m_iNodeMatrix= new int[m_iNodes][m_iNodes];
		
		if(m_Map==null) return;

		int iX,iZ,jX,jZ;
		
		for(int i=0;i<m_iNodes;i++)
		{
			//pasamos de i a x y z para "facilitar" los calculos
			iX=i%m_iXSize;
			iZ=i/m_iZSize;
			if(m_Map.CanWalk(iX, iZ))
			{
				for(int j=0;j<m_iNodes;j++)
				{
					//pasamos de j a x y z para "facilitar" los calculos
					jX=j%m_iXSize;
					jZ=j/m_iZSize;
					
					if(iX-1==jX && iZ+1==jZ)////arriba-derecha
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX-1==jX && iZ-1==jZ)//arriba-izquierda
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX+1==jX && iZ+1==jZ)//abajo-derecha
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX+1==jX && iZ-1==jZ)//abajo-izquierda
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX+1==jX && iZ==jZ)//derecha
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX-1==jX && iZ==jZ)//izquierda
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX==jX && iZ+1==jZ)//abajo
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX==jX && iZ-1==jZ)//arriba
					{
						m_iNodeMatrix[i][j]=1;
					}
					else if(iX==jX && iZ==jZ)//la misma
					{
						m_iNodeMatrix[i][j]=1;
					}
					else
					{
						m_iNodeMatrix[i][j]=0;
					}

				}
					
			}
			else
			{
				//si me encuentro en una posicion no pisable no puedo ir a ninguna otra porque no puedo
				//encontrarme nunca en esa posicion
				for(int j=0;j<m_iNodes;j++)
				{
					 m_iNodeMatrix[i][j]=0;
				}
				
			}
		}
		
		
	}

	@Override
	public Vector3D[] getPath(Vector3D source, Vector3D target) {
		
		//algoritmo

		int s=((int)(source.x/8)+((int)(source.z/8))*m_iZSize);
		int t= ((int)(target.x/8)+((int)(target.z/8)*m_iZSize));
		
		
		int[] Distancia= new int[m_iNodes]; 
		Boolean[] Fijados= new Boolean[m_iNodes];
		int[] Predecesor= new int[m_iNodes];
		int v=0;
		
		
		
		for (int u = 0; u < m_iNodes; u++) 
		{
				 Distancia[u]=Integer.MAX_VALUE; 
				 Fijados[u]=false; 
				 Predecesor[u]=s;
				 
		}
		
		
		System.out.println("Searching path using: Dijkstra ");
		
		//la distancia al source es 0
		Distancia[s] = 0;
		
		while (!Fijados[t]) 
		{
			int min = Integer.MAX_VALUE;

			for (int u = 0; u < m_iNodes; u++) 
			{
				if (!Fijados[u] && Distancia[u] < min) 
				{
					min = Distancia[u];
					v = u;
				}
			}
			
			Fijados[v] = true;
			
			for (int w = 0; w < m_iNodes; w++) {
				if (m_iNodeMatrix[v][w] != 0 && !Fijados[w] && Distancia[w] > Distancia[v] + m_iNodeMatrix[v][w]) 
				{
					Distancia[w] = Distancia[v] + m_iNodeMatrix[v][w];
					Predecesor[w] = v;
				}
			}

		} 
		
		System.out.println("Path found!!");
		
		
		
		int actual=t;
		int iter=0;
		ArrayList<Vector3D> path=new ArrayList<Vector3D>();
		while(actual!=Predecesor[actual])
		{
			path.add(new Vector3D(((actual%m_iXSize)*8)+4,0.00,((actual/m_iXSize)*8)+4));
			actual=Predecesor[actual];
			iter++;
		}
		
		//ponemos el camino en el orden correcto de principio a fin.
		Vector3D[] dijkstraPath= new Vector3D[iter];
		int pAs=0;
		for(int p=path.size()-1;p>=0;p--)
		{
			dijkstraPath[pAs]=path.get(p);
			pAs++;
			
		}
		
		
		return dijkstraPath;

		
	}

}
