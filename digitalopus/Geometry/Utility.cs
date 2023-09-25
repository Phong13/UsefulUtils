using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;


namespace digitalopus.geometry
{
    public class G_Utility
    {
        public const float TWO_PI = Mathf.PI * 2f; //180 degrees
        public const float HALF_PI = Mathf.PI * 2f; //90 degrees

        //	public struct GContactPoint{
        //		public Vector3 normal;
        //		public Vector3 point;
        //		public Collider thisCollider;
        //		public Collider otherCollider;
        //	}

        static public void Destroy(UnityEngine.Object obj)
        {
            if (Application.isPlaying)
            {
                GameObject.Destroy(obj);
            }
            else if (Application.isEditor)
            {
                GameObject.DestroyImmediate(obj);
            }
            else
            {
                Debug.LogError("Could not destroy.");
            }
        }

        static public Vector3 RotateVectorUsingAngleAxisRodriguesFormula(Vector3 v, Vector3 axisUnit, float angle_rad)
        {
            Debug.Assert(IsVector3ApproxOne(axisUnit));
            float cosTheta = Mathf.Cos(angle_rad);
            return cosTheta * v +
                   Mathf.Sin(angle_rad) * Vector3.Cross(axisUnit, v) +
                   ((1f - cosTheta) * Vector3.Dot(axisUnit, v)) * axisUnit;
        }

        static public float d_NormalizeAngle_0_360(float x_deg)
        {
            x_deg = x_deg % 360f;
            if (x_deg < 0f)
                x_deg += 360f;
            return x_deg;
        }

        static public float d_NormalizeAngle_0_180(float x_deg)
        {
            x_deg = x_deg % 360f;
            if (x_deg <= -180f)
                x_deg += 360f;
            if (x_deg > 180f)
                x_deg -= 360f;
            return x_deg;
        }

        static public float r_NormalizeAngle_0_TwoPI(float x_rad)
        {
            x_rad = x_rad % TWO_PI;
            if (x_rad < 0f)
                x_rad += TWO_PI;
            return x_rad;
        }

        static public T GetComponentInAncestor<T>(Transform tt) where T : Component
        {
            Transform t = tt;
            while (t != null && t.parent != t)
            {
                ;
                T bb = t.GetComponent<T>();
                if (bb != null) return bb;
                t = t.parent;
            }
            return null;
        }

        //gets the -180 .. 180 angle in degrees from the z axis
        static public bool d_GetAngleAboutY(Vector3 forward, out float angle)
        {
            if (forward.z == 0f && forward.x == 0f)
            {
                //Debug.LogError("Frame " + Time.frameCount + " Can't get angle with vector lined up with y. forward vector is colinear with y axis.");
                angle = 0f;
                return false;
            }
            forward.y = 0f;
            forward.Normalize();
            float targetAngle = Mathf.Acos(forward.z); //in x,y plane
            if (forward.x < 0f) targetAngle = 2f * Mathf.PI - targetAngle;
            angle = targetAngle * Mathf.Rad2Deg;
            return true;
        }

        /// <summary>
        /// Rotates v by oldNoraml to newNormal
        /// </summary>
        static public Vector3 RotateOntoPlane(Vector3 v, Vector3 oldNormal, Vector3 newNormal)
        {
            Quaternion rot = Quaternion.FromToRotation(oldNormal, newNormal);
            return rot * v;
        }

        //y is up z polar is from z eleation is up from =0 plane
        static public Vector3 d_Spherical2Cartesian(float polar, float elevation, float radius)
        {
            Vector3 v;
            float a = radius * Mathf.Cos(elevation * Mathf.Deg2Rad);
            v.x = a * Mathf.Sin(polar * Mathf.Deg2Rad);
            v.y = radius * Mathf.Sin(elevation * Mathf.Deg2Rad);
            v.z = a * Mathf.Cos(polar * Mathf.Deg2Rad);
            return v;
        }

        //y is up z polar is from z eleation is up from =0 plane
        static public Vector3 d_Cartesian2Spherical(Vector3 r)
        {
            float outRadius = r.magnitude;
            float outPolar = Mathf.Atan2(r.x, r.z);
            float outElevation = Mathf.Asin(r.y / outRadius);
            Vector3 v = new Vector3();
            v[0] = outPolar * Mathf.Rad2Deg;
            v[1] = outElevation * Mathf.Rad2Deg;
            v[2] = outRadius;
            return v;
        }

        static bool Quadratic(float A, float B, float C, out float t0, out float t1)
        {
            float discrim = B * B - 4.0f * A * C;
            if (discrim <= 0.0f)
            {
                t0 = 0f;
                t1 = 0f;
                return false;
            }
            float rootDiscrim = Mathf.Sqrt(discrim);
            // Compute quadratic _t_ values
            float q;
            if (B < 0) q = -.5f * (B - rootDiscrim);
            else q = -.5f * (B + rootDiscrim);
            t0 = q / A;
            t1 = C / q;
            if (t0 > t1)
            {
                float temp = t0;
                t0 = t1;
                t1 = temp;
            }
            return true;
        }

        public static bool IntersectRaySphere(Vector3 rayOrigin, Vector3 rayDir, Vector3 sphereCenter, float sphereRad, out float tSph0, out float tSph1)
        {
            // Clip against the sphere cap
            Vector3 oc = rayOrigin - sphereCenter;
            float A = Vector3.Dot(rayDir, rayDir);
            float B = 2.0f * Vector3.Dot(oc, rayDir);
            float C = Vector3.Dot(oc, oc) - sphereRad * sphereRad;
            if (!Quadratic(A, B, C, out tSph0, out tSph1))
            {
                // no intersection
                return false;
            } else
            {
                return true;
            }
        }

        public static bool IsInsideSphere(Vector3 center, float radius, Vector3 pnt)
        {
            return (Vector3.Distance(center, pnt) <= radius);
        }

        public static float RayCastAgainstSphere(Vector3 center, float radius, Ray r, out Vector3 pnt)
        {
            Vector3 oc = r.origin - center;
            float a = Vector3.Dot(r.direction, r.direction);
            float b = 2.0f * Vector3.Dot(oc, r.direction);
            float c = Vector3.Dot(oc, oc) - radius * radius;
            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0f)
            {
                // Ray misses sphere.
                pnt = r.origin;
                return -1.0f;
            }

            // There will be two t's (+/- discriminant)
            // if both t's complex then ray missed sphere
            // if discriminant = 0 then ray skimmed surface of sphere (one solution)
            // if both t's positive then ray passed through sphere
            // if both t's neg then backwards ray would hit sphere
            // if on t neg then ray was cast from inside sphere.
            float t = (-b - Mathf.Sqrt(discriminant)) / (2.0f * a);
            if (t > 0f)
            {
                pnt = r.GetPoint(t);
                return t;
            }

            t = (-b + Mathf.Sqrt(discriminant)) / (2.0f * a);
            if (t > 0f)
            {
                pnt = r.GetPoint(t);
                return t;
            }

            pnt = r.origin;
            return -1f;
        }

        static public float GetDistPointToLine(Vector3 origin, Vector3 direction, Vector3 point)
        {
            Vector3 point2origin = origin - point;
            Vector3 point2closestPointOnLine = point2origin - Vector3.Dot(point2origin, direction) * direction;
            return point2closestPointOnLine.magnitude;
        }

        static public float GetDistPointToRay(Vector3 origin, Vector3 v2w, Vector3 p, out float t)
        {
            float l2 = v2w.sqrMagnitude;  // i.e. |w-v|^2 -  avoid a sqrt
            if (l2 == 0.0f)
            {
                Vector3 p2v = p - origin;
                t = 0f;
                return p2v.magnitude;   // v == w case
            }

            // Consider the line extending the segment, parameterized as v + t (w - v).
            // We find projection of point p onto the line. 
            // It falls where t = [(p-v) . (w-v)] / |w-v|^2
            t = Vector3.Dot(p - origin, v2w) / l2;
            Vector3 projection = origin + t * (v2w);  // Projection falls on the segment
            return (p - projection).magnitude;
        }


        static public float GetDistPointToLineSegment3D(Vector3 v, Vector3 w, Vector3 p, out float tt)
        {
            // Return minimum distance between line segment vw and point p
            Vector3 v2w = w - v;
            float l2 = v2w.sqrMagnitude;  // i.e. |w-v|^2 -  avoid a sqrt
            if (l2 == 0.0f)
            {
                Vector3 p2v = p - v;
                tt = 0f;
                return p2v.magnitude;   // v == w case
            }
            // Consider the line extending the segment, parameterized as v + t (w - v).
            // We find projection of point p onto the line. 
            // It falls where t = [(p-v) . (w-v)] / |w-v|^2
            float t = Vector3.Dot(p - v, w - v) / l2;
            float d = 0f;
            if (t < 0.0)
            {
                d = (p - v).magnitude;       // Beyond the 'v' end of the segment
                tt = 0f;
            }
            else if (t > 1.0)
            {
                d = (p - w).magnitude;  // Beyond the 'w' end of the segment
                tt = 1f;
            }
            else
            {
                Vector3 projection = v + t * (w - v);  // Projection falls on the segment
                d = (p - projection).magnitude;
                tt = t;
            }

            return d;
        }

        static public Vector3 GetClosestPointOnLine(Vector3 origin, Vector3 direction, Vector3 point)
        {
            Vector3 origin2point = point - origin;
            return origin + Vector3.Dot(origin2point, direction) * direction;
        }

        static public Vector3 GetVectorFromPointToClosestOnLine(Vector3 origin, Vector3 direction, Vector3 point)
        {
            Vector3 pointOnLine = GetClosestPointOnLine(origin, direction, point);
            return pointOnLine - point;
        }

        static public Vector3 GetVectorFromPointToClosestOnSegment(Vector3 P, Vector3 SP0, Vector3 SP1)
        {
            Vector3 v = SP1 - SP0;
            Vector3 w = P - SP0;

            float c1 = Vector3.Dot(w, v);
            if (c1 <= 0f)
                return SP0 - P;

            float c2 = Vector3.Dot(v, v);
            if (c2 <= c1)
                return SP1 - P;

            float b = c1 / c2;
            Vector3 Pb = SP0 + b * v;
            return Pb - P;
        }

        // dist3D_Line_to_Line(): get the 3D minimum distance between 2 lines
        //    Input:  two 3D lines L1 and L2
        //    Return: the shortest distance between L1 and L2
        public static Vector3 dist3D_Line_to_Line(Vector3 S1start, Vector3 S1end,
                                Vector3 S2start, Vector3 S2end)
        {
            Vector3 u = S1end - S1start;
            Vector3 v = S2end - S2start;
            Vector3 w = S1start - S2start;
            float a = Vector3.Dot(u, u);         // always >= 0
            float b = Vector3.Dot(u, v);
            float c = Vector3.Dot(v, v);         // always >= 0
            float d = Vector3.Dot(u, w);
            float e = Vector3.Dot(v, w);
            float D = a * c - b * b;        // always >= 0
            float sc, tc;

            // compute the line parameters of the two closest points
            if (D < 10e-10f)
            {          // the lines are almost parallel
                sc = 0.0f;
                tc = (b > c ? d / b : e / c);    // use the largest denominator
            }
            else
            {
                sc = (b * e - c * d) / D;
                tc = (a * e - b * d) / D;
            }

            // get the difference of the two closest points
            Vector3 dP = w + (sc * u) - (tc * v);  // =  L1(sc) - L2(tc)

            return dP;   // return the closest distance
        }
        //===================================================================


        // dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
        //    Input:  two 3D line segments S1 and S2
        //    Return: the shortest distance between S1 and S2
        public static Vector3 DistSegment2Segment(Vector3 S1start, Vector3 S1end,
                                           Vector3 S2start, Vector3 S2end, out float t1, out float t2)
        {
            Vector3 u = S1end - S1start;
            Vector3 v = S2end - S2start;
            Vector3 w = S1start - S2start;
            float a = Vector3.Dot(u, u);         // always >= 0
            float b = Vector3.Dot(u, v);
            float c = Vector3.Dot(v, v);         // always >= 0
            float d = Vector3.Dot(u, w);
            float e = Vector3.Dot(v, w);
            float D = a * c - b * b;        // always >= 0
            float sN, sD = D;       // sc = sN / sD, default sD = D >= 0
            float tN, tD = D;       // tc = tN / tD, default tD = D >= 0

            // compute the line parameters of the two closest points
            if (D < 10e-10f)
            { // the lines are almost parallel
                sN = 0.0f;         // force using point P0 on segment S1
                sD = 1.0f;         // to prevent possible division by 0.0 later
                tN = e;
                tD = c;
            }
            else
            {                 // get the closest points on the infinite lines
                sN = (b * e - c * d);
                tN = (a * e - b * d);
                if (sN < 0.0f)
                {        // sc < 0 => the s=0 edge is visible
                    sN = 0.0f;
                    tN = e;
                    tD = c;
                }
                else if (sN > sD)
                {  // sc > 1  => the s=1 edge is visible
                    sN = sD;
                    tN = e + b;
                    tD = c;
                }
            }

            if (tN < 0.0f)
            {            // tc < 0 => the t=0 edge is visible
                tN = 0.0f;
                // recompute sc for this edge
                if (-d < 0.0f)
                    sN = 0.0f;
                else if (-d > a)
                    sN = sD;
                else
                {
                    sN = -d;
                    sD = a;
                }
            }
            else if (tN > tD)
            {      // tc > 1  => the t=1 edge is visible
                tN = tD;
                // recompute sc for this edge
                if ((-d + b) < 0.0f)
                    sN = 0f;
                else if ((-d + b) > a)
                    sN = sD;
                else
                {
                    sN = (-d + b);
                    sD = a;
                }
            }
            // finally do the division to get sc and tc
            t1 = (Mathf.Abs(sN) < 10e-10f ? 0.0f : sN / sD);
            t2 = (Mathf.Abs(tN) < 10e-10f ? 0.0f : tN / tD);

            // get the difference of the two closest points
            Vector3 dP = w + (t1 * u) - (t2 * v);  // =  S1(sc) - S2(tc)
            return dP;   // return the closest distance
        }

        // cpa_time(): compute the time of CPA for two tracks
        //    Input:  two tracks Tr1 and Tr2 ray direction is velocity vector
        //    Return: the time at which the two tracks are closest
        static public float ClosestApproachTime(Ray Tr1, Ray Tr2)
        {
            Vector3 dv = Tr1.direction - Tr2.direction;

            float dv2 = Vector3.Dot(dv, dv);
            if (dv2 < 10e-10f)      // the  tracks are almost parallel
                return 0.0f;             // any time is ok.  Use time 0.

            Vector3 w0 = Tr1.origin - Tr2.origin;
            float cpatime = -Vector3.Dot(w0, dv) / dv2;

            return cpatime;             // time of CPA
        }
        //===================================================================


        // cpa_distance(): compute the distance at CPA for two tracks
        //    Input:  two tracks Tr1 and Tr2
        //    Return: the distance for which the two tracks are closest
        static public void ClosestApproachPoints(Ray Tr1, Ray Tr2, out Vector3 P1, out Vector3 P2)
        {
            float ctime = ClosestApproachTime(Tr1, Tr2);
            P1 = Tr1.origin + (ctime * Tr1.direction);
            P2 = Tr2.origin + (ctime * Tr2.direction);

            //return Vector3.Distance(P1,P2);            // distance at CPA
        }

        //return x is distance, y is t
        static public Vector2 GetDistPointToLineSegment2D(Vector2 v, Vector2 w, Vector2 p)
        {
            // Return minimum distance between line segment vw and point p
            Vector2 v2w = w - v;
            float l2 = v2w.sqrMagnitude;  // i.e. |w-v|^2 -  avoid a sqrt
            if (l2 == 0.0)
            {
                Vector2 p2v = p - v;
                return new Vector2(p2v.magnitude, 0f);   // v == w case
            }
            // Consider the line extending the segment, parameterized as v + t (w - v).
            // We find projection of point p onto the line. 
            // It falls where t = [(p-v) . (w-v)] / |w-v|^2
            float t = Vector2.Dot(p - v, w - v) / l2;
            float d = 0f;
            if (t < 0.0) d = (p - v).magnitude;       // Beyond the 'v' end of the segment
            else if (t > 1.0) d = (p - w).magnitude;  // Beyond the 'w' end of the segment
            Vector2 projection = v + t * (w - v);  // Projection falls on the segment
            d = (p - projection).magnitude;
            return new Vector2(d, t);
        }

        //returns -2 perfect overalp, -1or0 no overlap or num residual
        //else returns number of residual segments
        static public int FindOverlapResidualsOnAofColinearSegmentB(Vector3 a1, Vector3 a2, Vector3 b1, Vector3 b2, float threshold,
                                                                    ref Vector3 ra1, ref Vector3 ra2, ref Vector3 rb1, ref Vector3 rb2)
        {
            //first check if colinear
            Vector3 an = (a2 - a1).normalized;
            Vector3 bn = (b2 - b1).normalized;
            if (1f - Mathf.Abs(Vector3.Dot(an, bn)) > .000001f) //align to 1/4 degree
            {
                return 0; //don't point in same direction
            }

            Vector3 abn1 = (b1 - a1).normalized;
            Vector3 abn2 = (b2 - a1).normalized;
            int numResidual = 0;
            if (Vector3.Cross(an, abn1).magnitude < .000001f &&
                Vector3.Cross(an, abn2).magnitude < .000001f)
            {
                //colinear
                Vector3 dir = a2 - a1;
                float t1 = 0f;
                float t2 = 1f;
                float mm = Vector3.Dot(dir, dir);
                float t3 = Vector3.Dot(b1 - a1, dir) / mm;
                float t4 = Vector3.Dot(b2 - a1, dir) / mm;
                if (t3 > t4)
                {
                    float tmp = t3;
                    t3 = t4;
                    t4 = tmp;
                }


                // If the intervals [t1, t2] and [t3, t4] overlap, intervals_overlap()
                // should return 'true' and store the overlap endpoints in 'start' and 'end':
                if (Mathf.Abs(t3 - t1) < threshold && Mathf.Abs(t4 - t2) < threshold)
                {
                    return -2; //perfect overlap
                }
                else if (t3 > t2 - threshold / 2f || t4 < t1 + threshold / 2f)
                {
                    return -1; //no overlap
                }
                else
                {
                    if (t3 - t1 > threshold)
                    { //residual segment at start
                        ra1 = a1;
                        ra2 = a1 + dir * t3;
                        numResidual = 1;
                    }
                    if (t2 - t4 > threshold)
                    { //residual segment at end
                        if (numResidual == 1)
                        {
                            rb1 = a1 + dir * t4;
                            rb2 = a2;
                        }
                        else
                        {
                            ra1 = a1 + dir * t4;
                            ra2 = a2;
                        }
                        numResidual++;
                    }
                }
            }
            return numResidual; //not colinear
        }

        static public bool PointInPolygon2D(Vector2 point, List<Vector2> points)
        {
            int i, j, nvert = points.Count;
            bool c = false;

            for (i = 0, j = nvert - 1; i < nvert; j = i++)
            {
                if (((points[i].y) >= point.y != (points[j].y >= point.y)) &&
                    (point.x <= (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
                  )
                    c = !c;
            }
            return c;
        }
        /*
            public static Vector3 ClosestPointOnCollider(Collider c, Vector3 p){
                //tests bounding box, not so good for spheres and capsules
                Bounds b = c.bounds;
                float rad = Mathf.Max(b.
        //		Vector3 pt = Vector3.zero;
        //		if (c is BoxCollider){
        //			 pt = c.ClosestPointOnBounds(p);
        //		} else if (c is SphereCollider){
        //			SphereCollider sc = (SphereCollider) c;
        //			Matrix4x4 w2l = sc.transform.worldToLocalMatrix;
        //			Vector3 pl = w2l.MultiplyPoint3x4(p);
        //			//move in a tiny amount to avoid rounding errors
        //			Vector3 ptOnSurfaceLocal = (pl - sc.center).normalized * sc.radius * .9999f + sc.center;
        //			Matrix4x4 l2w = sc.transform.localToWorldMatrix;
        //			pt = l2w.MultiplyPoint3x4(ptOnSurfaceLocal);
        //			if (Vector3.Distance(sc.center,ptOnSurfaceLocal) > sc.radius) Debug.LogError("pt is not inside collider pos dist " + (Vector3.Distance(sc.center,ptOnSurfaceLocal) - sc.radius));
        //			
        //		} else if (c is CapsuleCollider){
        //			pt = Vector3.zero;
        //			Debug.LogError("CapsuleCollider not supported");
        //		} else if (c is MeshCollider){
        //			pt = Vector3.zero;
        //			Debug.LogError("MeshCollider not supported");
        //		}

                //nudge point toward center because we raycast to pt and that often misses which should be impossible
                pt = pt + (c.bounds.center - pt).normalized * .0001f;
                return pt;
            }	
        */
        public static Vector3 ClosestPointOnBox(Vector3 center, Vector3 extents, Vector3 pl)
        {
            //push points in along each axis
            bool isOutside = false;
            Vector3 bMin = center - extents;
            Vector3 bMax = center + extents;
            if (pl.x < bMin.x) { pl.x = bMin.x; isOutside = true; }
            else if (pl.x > bMax.x) { pl.x = bMax.x; isOutside = true; }
            if (pl.y < bMin.y) { pl.y = bMin.y; isOutside = true; }
            else if (pl.y > bMax.y) { pl.y = bMax.y; isOutside = true; }
            if (pl.z < bMin.z) { pl.z = bMin.z; isOutside = true; }
            else if (pl.z > bMax.z) { pl.z = bMax.z; isOutside = true; }

            if (!isOutside)
            { //point is inside need to find closest wall and move to it
                int closestIdx = 0;
                float minDistFound = Mathf.Infinity;
                float amtToMove = 0f;

                float toBottom = pl[0] - bMin[0];
                float toTop = bMax[0] - pl[0];
                if (toTop < minDistFound || toBottom < minDistFound)
                {
                    closestIdx = 0;
                    if (toTop < toBottom)
                    {
                        minDistFound = toTop;
                        amtToMove = toTop;
                    }
                    else
                    {
                        minDistFound = toBottom;
                        amtToMove = -toBottom;
                    }
                }

                toBottom = pl[1] - bMin[1];
                toTop = bMax[1] - pl[1];
                if (toTop < minDistFound || toBottom < minDistFound)
                {
                    closestIdx = 1;
                    if (toTop < toBottom)
                    {
                        minDistFound = toTop;
                        amtToMove = toTop;
                    }
                    else
                    {
                        minDistFound = toBottom;
                        amtToMove = -toBottom;
                    }
                }

                toBottom = pl[2] - bMin[2];
                toTop = bMax[2] - pl[2];
                if (toTop < minDistFound || toBottom < minDistFound)
                {
                    closestIdx = 2;
                    if (toTop < toBottom)
                    {
                        minDistFound = toTop;
                        amtToMove = toTop;
                    }
                    else
                    {
                        minDistFound = toBottom;
                        amtToMove = -toBottom;
                    }
                }
                pl[closestIdx] += amtToMove;
            }
            return pl;
        }

        //does more than just checks AABB bounds
        public static Vector3 ClosestPointOnCollider(Collider c, Vector3 p)
        {
            //tests bounding box, not so good for spheres and capsules
            Vector3 pt = Vector3.zero;
            if (c is BoxCollider)
            {
                BoxCollider bc = (BoxCollider)c;

                Vector3 pl = bc.transform.InverseTransformPoint(p);
                //push points in along each axis
                pl = ClosestPointOnBox(bc.center, bc.size / 2f, pl);
                pt = bc.transform.TransformPoint(pl);
            }
            else if (c is SphereCollider)
            {
                SphereCollider sc = (SphereCollider)c;
                Matrix4x4 w2l = sc.transform.worldToLocalMatrix;
                Vector3 pl = w2l.MultiplyPoint3x4(p);
                //move in a tiny amount to avoid rounding errors
                Vector3 ptOnSurfaceLocal = (pl - sc.center).normalized * sc.radius * .9999f + sc.center;
                Matrix4x4 l2w = sc.transform.localToWorldMatrix;
                pt = l2w.MultiplyPoint3x4(ptOnSurfaceLocal);
                if (Vector3.Distance(sc.center, ptOnSurfaceLocal) > sc.radius) Debug.LogError("pt is not inside collider pos dist " + (Vector3.Distance(sc.center, ptOnSurfaceLocal) - sc.radius));
            }
            else if (c is CapsuleCollider)
            {
                pt = Vector3.zero;
                Debug.LogError("CapsuleCollider not supported");
            }
            else if (c is MeshCollider)
            {
                //Do two raycasts one from current position, one from closest on bounds
                MeshCollider mc = (MeshCollider)c;
                Vector3 worldMeshCenter = c.bounds.center;
                Ray r = new Ray(p, worldMeshCenter - p);
                RaycastHit hi;
                Vector3 h1 = worldMeshCenter;
                if (mc.Raycast(r, out hi, 1000f))
                {
                    h1 = hi.point;
                }
                Vector3 h2 = worldMeshCenter;
                Vector3 closestOnBounds = mc.ClosestPointOnBounds(p);
                r.origin = closestOnBounds;
                r.direction = worldMeshCenter - closestOnBounds;
                if (mc.Raycast(r, out hi, 1000f))
                {
                    h2 = hi.point;
                }
                if ((p - h1).sqrMagnitude < (p - h2).sqrMagnitude)
                {
                    pt = h1;
                }
                else
                {
                    pt = h2;
                }
            }

            //nudge point toward center because we raycast to pt and that often misses which should be impossible
            pt = pt + (c.bounds.center - pt).normalized * .0001f;
            return pt;
        }

        public static bool SphereRayIntersection(Vector3 center, float radius,  Vector3 rayOrigin, Vector3 rayDir, out float t)
        {
            Vector3 v = rayOrigin - center;
            float b = 2 * Vector3.Dot(rayDir, v);
            float c = Vector3.Dot(v, v) - radius * radius;
            float d = b * b - 4 * c;
            if (d > 0)
            {
                float x1 = (-b - Mathf.Sqrt(d)) / 2f;
                float x2 = (-b + Mathf.Sqrt(d)) / 2f;
                if (x1 >= 0 && x2 >= 0)
                {
                    t = x1;
                    return true;
                }
                if (x1 < 0 && x2 >= 0)
                {
                    t = x2;
                    return true;
                }
            }

            t = 0;
            return false;
        }

        public static bool ColliderContains(Collider c, Vector3 p)
        {
            if (c is BoxCollider)
            {
                BoxCollider bc = (BoxCollider)c;
                Matrix4x4 w2l = bc.transform.worldToLocalMatrix;
                Vector3 pl = w2l.MultiplyPoint3x4(p);
                pl -= bc.center;
                Vector3 extents = bc.size / 2f;
                if (Mathf.Abs(pl.x) <= extents.x &&
                    Mathf.Abs(pl.y) <= extents.y &&
                    Mathf.Abs(pl.z) <= extents.z)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else if (c is SphereCollider)
            {
                SphereCollider sc = (SphereCollider)c;
                Matrix4x4 w2l = sc.transform.worldToLocalMatrix;
                Vector3 pl = w2l.MultiplyPoint3x4(p);
                pl -= sc.center;
                if (pl.sqrMagnitude <= sc.radius * sc.radius) return true;
                else return false;
            }
            else if (c is CapsuleCollider)
            {
                Debug.LogError("CapsuleCollider not supported");
                return false;
            }
            else if (c is MeshCollider)
            {
                Debug.LogError("MeshCollider not supported");
                return false;
            }
            else
            {
                Debug.LogError("Not Supported");
                return false;
            }
        }

        public static void KeepAtYZero(Rigidbody r)
        {
            float dist = r.transform.position.y;
            if (dist > 10e-5)
            {
                float v = -dist / Time.deltaTime;
                Vector3 vv = r.velocity;
                vv.y = v;
                r.velocity = vv;
                //			Debug.LogError("CORRECTING p=" + r.transform.position + " v=" + r.velocity + " f=" + Time.frameCount);
            }
            else
            {
                Vector3 vv = r.velocity;
                vv.y = 0f;
                r.velocity = vv;
            }
        }

        //	static Texture2D GetTextureWithCircle(int size){
        //		Texture2D t = new Texture2D(size,size,TextureFormat.ARGB32,true);
        //		Color32[] cs = t.GetPixels32();
        //		for (int i = 0; i < size; i++){
        //			for (int j = 0; j < size; j++){
        //					
        //			}
        //		}
        //	}

        public static bool CatmullRomSpline(Vector3[] inKeypoints, Vector3[] results, int samplesPerCurve)
        {
            if (inKeypoints.Length < 4)
            {
                Debug.LogError("Not enough control points");
            }

            if (results.Length != (inKeypoints.Length - 3) * samplesPerCurve + 1)
            {
                Debug.LogError("outCoordinates is the wrong size");
            }

            results[0] = inKeypoints[0];
            for (int n = 1; n < inKeypoints.Length - 2; n++)
            {
                for (int i = 0; i < samplesPerCurve; i++)
                {
                    results[(n - 1) * samplesPerCurve + i + 1] = CatmullRomSpline_PointOnCurve(inKeypoints[n - 1], inKeypoints[n], inKeypoints[n + 1], inKeypoints[n + 2], (1f / samplesPerCurve) * i);
                }
            }
            results[results.Length - 1] = inKeypoints[inKeypoints.Length - 1];
            return true;
        }

        /// <summary>
        /// Return a point on the curve between P1 and P2 with P0 and P4 describing curvature, at
        /// the normalized distance t.
        /// </summary>

        public static Vector3 CatmullRomSpline_PointOnCurve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            Vector3 result = new Vector3();

            float t0 = ((-t + 2f) * t - 1f) * t * 0.5f;
            float t1 = (((3f * t - 5f) * t) * t + 2f) * 0.5f;
            float t2 = ((-3f * t + 4f) * t + 1f) * t * 0.5f;
            float t3 = ((t - 1f) * t * t) * 0.5f;

            result.x = p0.x * t0 + p1.x * t1 + p2.x * t2 + p3.x * t3;
            result.y = p0.y * t0 + p1.y * t1 + p2.y * t2 + p3.y * t3;
            result.z = p0.z * t0 + p1.z * t1 + p2.z * t2 + p3.z * t3;

            return result;
        }

        public static Rect Bounds2ScreenRect(Camera c, Bounds b)
        {
            Vector3 cen = b.center;
            Vector3 ext = b.extents;
            Vector2[] extentPoints = new Vector2[8]
            {
            c.WorldToScreenPoint(new Vector3(cen.x-ext.x, cen.y-ext.y, cen.z-ext.z)),
            c.WorldToScreenPoint(new Vector3(cen.x+ext.x, cen.y-ext.y, cen.z-ext.z)),
            c.WorldToScreenPoint(new Vector3(cen.x-ext.x, cen.y-ext.y, cen.z+ext.z)),
            c.WorldToScreenPoint(new Vector3(cen.x+ext.x, cen.y-ext.y, cen.z+ext.z)),

            c.WorldToScreenPoint(new Vector3(cen.x-ext.x, cen.y+ext.y, cen.z-ext.z)),
            c.WorldToScreenPoint(new Vector3(cen.x+ext.x, cen.y+ext.y, cen.z-ext.z)),
            c.WorldToScreenPoint(new Vector3(cen.x-ext.x, cen.y+ext.y, cen.z+ext.z)),
            c.WorldToScreenPoint(new Vector3(cen.x+ext.x, cen.y+ext.y, cen.z+ext.z))
            };

            Vector2 min = extentPoints[0];
            Vector2 max = extentPoints[0];

            foreach (Vector2 v in extentPoints)
            {
                min = Vector2.Min(min, v);
                max = Vector2.Max(max, v);
            }

            return new Rect(min.x, min.y, max.x - min.x, max.y - min.y);
        }

        public static bool ElipseIsOutside(float elipseHalfDimX, float elipseHalfDimY, float x, float y)
        {
            return (x * x / (elipseHalfDimX * elipseHalfDimX) + y * y / (elipseHalfDimY * elipseHalfDimY)) > 1f;
        }

        public static Vector2 ElipseClampPointTo(float elipseHalfDimX, float elipseHalfDimY, float x, float y)
        {
            Vector2 B;
            B.x = x / elipseHalfDimX;
            B.y = y / elipseHalfDimY;
            B.Normalize();
            B.x = B.x * elipseHalfDimX;
            B.y = B.y * elipseHalfDimY;
            return B;
        }

        public static bool IsVector3ApproxOne(Vector3 v)
        {
            float sqrMag = v.sqrMagnitude;
            if (sqrMag > .9999f && sqrMag < 1.0001f) return true;
            return false;
        }

        public static bool EqualsApproximate(Vector3 a, Vector3 b)
        {
            Vector3 delta = a - b;
            return delta.magnitude < 1e-4f;
       }
    }
}
