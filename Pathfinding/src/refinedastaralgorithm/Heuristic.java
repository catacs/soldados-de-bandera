package refinedastaralgorithm;
import es.upv.dsic.gti_ia.jgomas.Vector3D;

public abstract class Heuristic {

	public abstract double calcHeuristicByVector3D(Vector3D m_Start, Vector3D m_Dest);
	public abstract double calcHeuristicByRefinedNode(RefinedNode m_Start, RefinedNode m_Dest);
	
	
}
