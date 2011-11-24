package defenders;

import es.upv.dsic.gti_ia.jgomas.Vector3D;
import student.MyFieldOps;

public class def_FieldOps extends MyFieldOps {
	int iRadiusCP = 5;
	
	
	protected void CreateControlPoints() {
		int iMaxCP = 4;
		m_ControlPoints = new Vector3D [iMaxCP];
		for (int i = 0; i < iMaxCP; i++ ) {
			Vector3D ControlPoints = new Vector3D();
			while (true) {
				double x = m_Map.GetTargetX() + ((iRadiusCP-Math.random()*2*iRadiusCP));
				double z = m_Map.GetTargetZ() + ((iRadiusCP-Math.random()*2*iRadiusCP));
				if ( CheckStaticPosition(x, z) == true ) {
					ControlPoints.x = x;
					ControlPoints.z = z;
					m_ControlPoints[i] = ControlPoints;
					break;
				}
			}
		}
	}

}
