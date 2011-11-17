package refinedastaralgorithm;

import es.upv.dsic.gti_ia.jgomas.Vector3D;

public class ManhattanDistance extends Heuristic {

	@Override
	public double calcHeuristicByRefinedNode(RefinedNode start, RefinedNode dest) {
		int dist = 0;
		dist += Math.abs(dest.getiPosActx()>> 3 - start.getiPosActx()>>3);
		dist += Math.abs(dest.getiPosActz()>> 3 - start.getiPosActz()>>3);
		return dist;
	}

	@Override
	public double calcHeuristicByVector3D(Vector3D start, Vector3D dest) {
		int dist = 0;
		dist += Math.abs(((int)dest.x)>>3 -((int)start.x)>>3);
		dist += Math.abs(((int)dest.z)>>3 -((int)start.z)>>3);
		return dist;
	}

}
