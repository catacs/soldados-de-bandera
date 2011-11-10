package student;

import es.upv.dsic.gti_ia.jgomas.CTerrainMap;
import es.upv.dsic.gti_ia.jgomas.Vector3D;

public abstract class PathFinder {
	
	public abstract void setMap(CTerrainMap map);
	public abstract Vector3D[] getPath(Vector3D source,Vector3D target);
	
	

}
