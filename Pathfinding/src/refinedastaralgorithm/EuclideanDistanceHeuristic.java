package refinedastaralgorithm;

import es.upv.dsic.gti_ia.jgomas.Vector3D;
import java.lang.Math;

public class EuclideanDistanceHeuristic extends Heuristic {
	
	public double calcHeuristicByVector3D(Vector3D m_Start, Vector3D m_Dest)
	{
		double distance = 0.0;
		distance += Math.abs(m_Dest.x - m_Start.x);
		distance += Math.abs(m_Dest.z - m_Start.z);
		return distance;
	}
	public double calcHeuristicByRefinedNode(RefinedNode m_Start, RefinedNode m_Dest)
	{
		double distance = 0.0;
		distance += Math.abs(m_Dest.getiPosActx() - m_Start.getiPosActx());
		distance += Math.abs(m_Dest.getiPosActz() - m_Start.getiPosActz());
		return distance;
	}	
}
