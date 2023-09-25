using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using digitalopus.tranform;

namespace digitalopus.geometry
{
    public class ConeGeometryUtils
    {
        public enum ConeVsLineHitResult
        {
            noIntersection,
            intersectLineSegment,
            intersectLineButNotSegment,
        }

        [System.Serializable]
        public struct ConeHitResult
        {
            public bool startIsInsideCone;
            public float tHitInfLine0;
            public float tHitInfLine1;
            public int numClipped;
            public float tClipped0;
            public float tClipped1;
            public bool hit0wasCone;
            public bool hit1wasCone;
        }

        public struct ConeHit2 : System.IComparable<ConeHit2>
        {
            public float t;
            public Vector3 pnt_loc;
            /// <summary>
            /// Cone or sphere cap?
            /// </summary>
            public bool hitCone;

            public ConeHit2(float tt, Vector3 pnt_locc, bool wasConeHit)
            {
                t = tt;
                pnt_loc = pnt_locc;
                hitCone = wasConeHit;
            }

            public int CompareTo(ConeHit2 other)
            {
                float diff = t - other.t;
                if (diff > 0) return 1;
                if (diff == 0) return 0;
                return -1;
            }
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

        public static bool IsPointInsideCone(Vector3 point_wld, Vector3 coneOrigin, Vector3 coneAxis01Dir, float coneAngle_rad)
        {
            Debug.Assert(G_Utility.IsVector3ApproxOne(coneAxis01Dir));
            if (point_wld == coneOrigin) return true;
            Vector3 origin2pointDir = (point_wld - coneOrigin).normalized;
            float ang_rad = Mathf.Acos(Mathf.Clamp(Vector3.Dot(coneAxis01Dir, origin2pointDir), -1f, 1f));
            if (ang_rad <= coneAngle_rad) return true;
            return false;
        }

        public static bool IsPointInsideCone(Vector3 point_wld, Vector3 coneOrigin, Quaternion coneRot, float coneAngle_deg)
        {
            Vector3 p_coneFrm = TransformUtils.a_InverseTransformPoint_b(coneOrigin, coneRot, point_wld);
            if (Vector3.Angle(p_coneFrm.normalized, Vector3.forward) <= coneAngle_deg)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public static Vector3 SnapToCone(Vector3 p_wld, Vector3 coneOrigin, Vector3 coneAxis01Dir, float coneAngle_rad)
        {
            Debug.Assert(G_Utility.IsVector3ApproxOne(coneAxis01Dir));
            if (p_wld == coneOrigin)
            {
                return coneOrigin;
            }

            Vector3 origin2pointDir = (p_wld - coneOrigin);
            Debug.Assert(coneAxis01Dir != origin2pointDir.normalized, "degenerate case: p is on axis of cone.");

            // Construct a special frame aligned with coneOrigin where coneAxis is z and p is in the xz plane.
            Vector3 up_wld = Vector3.Cross(coneAxis01Dir, origin2pointDir);
            Quaternion wld_Rot_xz = Quaternion.LookRotation(coneAxis01Dir, up_wld);

            Vector3 p_xz = TransformUtils.a_InverseTransformPoint_b(coneOrigin, wld_Rot_xz, p_wld);

            // now we have a 2D problem of snapping a 2D point to a line.
            float coneLineX, coneLineZ;
            if (p_xz.x >= 0f)
            {
                coneLineX = Mathf.Sin(coneAngle_rad);
                coneLineZ = Mathf.Cos(coneAngle_rad);
            } else
            {
                coneLineX = - Mathf.Sin(coneAngle_rad);
                coneLineZ = Mathf.Cos(coneAngle_rad);
            }

            // closest point on line = origin + Vector3.Dot(origin2point, direction) * direction;
            // but in xz plane and origin = zero and direction is coneLineX, coneLineZ
            float dot = p_xz.x * coneLineX + p_xz.z * coneLineZ;
            Vector3 snappedP_xz = new Vector3(dot * coneLineX, 0f, dot * coneLineZ);
            if (snappedP_xz.z < 0)
            {
                return coneOrigin;
            }

            p_wld = TransformUtils.b_TransformPoint_a(coneOrigin, wld_Rot_xz, snappedP_xz);
            return p_wld;
        }


        public static bool IsPointInsideConeSphereCapBase(Vector3 point_wld, Vector3 coneOrigin, Quaternion coneRot, float coneAngle_deg, float coneHeightSphereRad)
        {
            Vector3 p_coneFrm = TransformUtils.a_InverseTransformPoint_b(coneOrigin, coneRot, point_wld);
            if (Vector3.Angle(p_coneFrm.normalized, Vector3.forward) <= coneAngle_deg &&
                p_coneFrm.sqrMagnitude <= coneHeightSphereRad * coneHeightSphereRad)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        /// <summary>
        /// Projects a point onto a cone. The path to the cone follows an arc centered on the cone origin.
        /// </summary>
        public static Vector3 ProjectPointOnCone_ArcPathToCone(Vector3 point_wld, Vector3 coneOrigin, Quaternion coneRot, float coneAngle_deg)
        {
            Vector3 p_coneFrm = TransformUtils.a_InverseTransformPoint_b(coneOrigin, coneRot, point_wld);
            float r = p_coneFrm.magnitude; // dist to cone origin.
                                           // project p onto plane that is defined by coneAxis and r from origin
            Vector3 axis2projOnPlane = new Vector3(p_coneFrm.x, p_coneFrm.y, 0);
            // snap proj point to cone surface
            float radAtPlane = r * Mathf.Tan(coneAngle_deg * Mathf.Deg2Rad);
            Vector3 p_onCone_coneFrm = new Vector3(0, 0, r) + axis2projOnPlane.normalized * radAtPlane; // p is a bit too far out
            p_onCone_coneFrm = p_onCone_coneFrm.normalized * r; // adjust distance so it is correct.
            Vector3 p_onCone_wld = TransformUtils.b_TransformPoint_a(coneOrigin, coneRot, p_onCone_coneFrm);
            return p_onCone_wld;
        }

        public static ConeVsLineHitResult IntersectConeOnly(Vector3 segStart, Vector3 segEnd, Vector3 coneOrigin, Quaternion coneRot, float coneRadius, float coneHeightSphereRad,
            out ConeHitResult outResults)
        {
            outResults.hit0wasCone = true;
            outResults.hit1wasCone = true;
            int numHits;

            Vector3 rayOrigin_cone = TransformUtils.a_InverseTransformPoint_b(coneOrigin, coneRot, segStart); //cone is at the origin
            Vector3 rayDir_cone = TransformUtils.a_InverseTransformDirection_b(coneRot, segEnd - segStart);

            float coneAngle_deg = Mathf.Asin(coneRadius / coneHeightSphereRad) * Mathf.Rad2Deg;
            if (!IntersectConeOnly(coneAngle_deg, coneRadius, coneHeightSphereRad,
                rayOrigin_cone, rayDir_cone,
                out outResults.tHitInfLine0, out outResults.tHitInfLine1, out numHits))
            {
                // no intersection between line segment and clipped
                outResults.numClipped = 0;
                outResults.tClipped0 = 0;
                outResults.tClipped1 = 1f;
                outResults.startIsInsideCone = false;
                return ConeVsLineHitResult.noIntersection;
            }

            ConeVsLineHitResult hitResult = ConeVsLineHitResult.noIntersection;
            if (numHits == 1)
            {
                bool startInside = (rayOrigin_cone.z >= 0 && Vector3.Angle(rayOrigin_cone, Vector3.forward) <= coneAngle_deg);
                outResults.startIsInsideCone = startInside;
                Vector3 rayEnd = rayOrigin_cone + rayDir_cone;
                bool endInside = (rayEnd.z >= 0 && Vector3.Angle(rayEnd, Vector3.forward) <= coneAngle_deg);

                if (0f <= outResults.tHitInfLine1 &&
                    outResults.tHitInfLine1 <= 1f)
                {
                    hitResult = ConeVsLineHitResult.intersectLineSegment;
                    if (startInside)
                    {
                        outResults.numClipped = 1;
                        outResults.tClipped0 = 0f;
                        outResults.tClipped1 = outResults.tHitInfLine0;
                    } else if (endInside)
                    {
                        outResults.numClipped = 1;
                        outResults.tClipped0 = outResults.tHitInfLine0;
                        outResults.tClipped1 = 0f;
                    } else
                    {
                        outResults.numClipped = 0;
                        outResults.tClipped0 = 0f;
                        outResults.tClipped1 = 1f;
                        Debug.LogError("Something is wrong. 0 <= t <= 1 should mean that start and end straddle the cone skin.");
                    }
                } else
                {
                    hitResult = ConeVsLineHitResult.intersectLineButNotSegment;
                    outResults.numClipped = 0;
                    outResults.tClipped0 = 0f;
                    outResults.tClipped1 = 1f;
                }
            }
            else if (numHits == 2)
            {
                outResults.startIsInsideCone = outResults.tHitInfLine0 <= 0f && outResults.tHitInfLine1 >= 0f;
                bool t0Clipped = 0f < outResults.tHitInfLine0 && outResults.tHitInfLine0 <= 1f;
                bool t1Clipped = 0f <= outResults.tHitInfLine1 && outResults.tHitInfLine1 < 1f;
                if (t0Clipped && t1Clipped)
                {
                    hitResult = ConeVsLineHitResult.intersectLineSegment;
                    outResults.numClipped = 2;
                    outResults.tClipped0 = outResults.tHitInfLine0;
                    outResults.tClipped1 = outResults.tHitInfLine1;
                }
                else if (t0Clipped)
                {
                    hitResult = ConeVsLineHitResult.intersectLineSegment;
                    outResults.numClipped = 1;
                    outResults.tClipped0 = outResults.tHitInfLine0;
                    outResults.tClipped1 = 1f;
                }
                else if (t1Clipped)
                {
                    hitResult = ConeVsLineHitResult.intersectLineSegment;
                    outResults.numClipped = 1;
                    outResults.tClipped0 = 0f;
                    outResults.tClipped1 = outResults.tHitInfLine1;
                }
                else
                {
                    hitResult = ConeVsLineHitResult.intersectLineButNotSegment;
                    outResults.numClipped = 0;
                    outResults.tClipped0 = 0f;
                    outResults.tClipped1 = 1f;
                }
            } else
            {
                Debug.LogError("How did we et more than 2 hits?");
                outResults.numClipped = 0;
                outResults.tClipped0 = 0;
                outResults.tClipped1 = 1f;
                outResults.startIsInsideCone = false;
            }

            return hitResult;
        }

        // returns 
        // infinite line intersects
        // line segment intersect
        // infinite intersect ts
        // segment intersect numTs
        // segment ts
        public static ConeVsLineHitResult IntersectConeVsLineSegmentSphereCapBase(Vector3 segStart, Vector3 segEnd, Vector3 coneOrigin, Quaternion coneRot, float coneRadius, float coneHeightSphereRad,
            out ConeHitResult outResults)
        {
            if (!IntersectConeVsInfiniteLineSphereCapBase(segStart, segEnd, coneOrigin, coneRot, coneRadius, coneHeightSphereRad,
                out outResults.tHitInfLine0, out outResults.tHitInfLine1, out outResults.hit0wasCone, out outResults.hit1wasCone))
            {
                // no intersection between line segment and clipped
                outResults.numClipped = 0;
                outResults.tClipped0 = 0;
                outResults.tClipped1 = 1f;
                outResults.startIsInsideCone = false;
                return ConeVsLineHitResult.noIntersection;
            }

            outResults.startIsInsideCone = outResults.tHitInfLine0 <= 0f && outResults.tHitInfLine1 >= 0f;
            bool t0Clipped = outResults.tHitInfLine0 >= 0f && outResults.tHitInfLine0 <= 1f;
            bool t1Clipped = outResults.tHitInfLine1 >= 0f && outResults.tHitInfLine1 <= 1f;
            if (t0Clipped && t1Clipped)
            {
                outResults.numClipped = 2;
                outResults.tClipped0 = outResults.tHitInfLine0;
                outResults.tClipped1 = outResults.tHitInfLine1;
            }
            else if (t0Clipped)
            {
                outResults.numClipped = 1;
                outResults.tClipped0 = outResults.tHitInfLine0;
                outResults.tClipped1 = 1f;
            }
            else if (t1Clipped)
            {
                outResults.numClipped = 1;
                outResults.tClipped0 = 0f;
                outResults.tClipped1 = outResults.tHitInfLine1;
            }
            else
            {
                outResults.numClipped = 0;
                outResults.tClipped0 = 0f;
                outResults.tClipped1 = 1f;
            }

            if (outResults.numClipped > 0)
            {
                return ConeVsLineHitResult.intersectLineSegment;
            }
            else
            {
                return ConeVsLineHitResult.intersectLineButNotSegment;
            }
        }

        public static bool IntersectConeOnly(float coneAngle_deg, float coneRadius, float coneHeightSphereRad,
            Vector3 ro, Vector3 rd,
            out float tHitInfLine0, out float tHitInfLine1, out int numHits)
        {
            // TODO we are allocating memory every call with this list. We should pass this in as a parameter or make the method non-static and store this buffer in a class instance.
            List<ConeHit2> hits = new List<ConeHit2>(6);

            // Transform _Ray_ so that cone axis is z is forward cone origin is at 0,0,0
            //insideCone = false;

            // Compute quadratic cone coefficients
            {
                float coneHeight = Mathf.Sqrt(coneHeightSphereRad * coneHeightSphereRad - coneRadius * coneRadius);
                float k = coneRadius / coneHeight;
                k = k * k;
                float A = rd.x * rd.x + rd.y * rd.y - k * rd.z * rd.z;
                float B = 2f * (rd.x * ro.x + rd.y * ro.y - k * rd.z * (ro.z));
                float C = ro.x * ro.x + ro.y * ro.y - k * (ro.z) * (ro.z);

                // Solve quadratic equation for _t_ values for infinite cone intersecting infinite line.
                if (!Quadratic(A, B, C, out tHitInfLine0, out tHitInfLine1))
                {
                    numHits = 0;
                    return false;
                }

                // Cones are complicated. There can be no hits, one hit, two hits.
                // Also this quadratic solution produces false positives cone is actually an hourglass.
                // Eliminate negative cone hits
                Vector3 pHit0_loc = ro + rd * tHitInfLine0;
                Vector3 pHit1_loc = ro + rd * tHitInfLine1;
                {
                    if (pHit0_loc.z < 0 && pHit1_loc.z < 0)
                    {
                        // all intersections on negative non-existant cone. no real intersections.
                        numHits = 0;
                        return false;
                    }
                    else if (pHit0_loc.z < 0 && pHit1_loc.z >= 0)
                    {
                        hits.Add(new ConeHit2(tHitInfLine1, pHit1_loc, true));
                        tHitInfLine0 = tHitInfLine1;
                        numHits = 1;
                        return true;
                    }
                    else if (pHit0_loc.z >= 0 && pHit1_loc.z < 0)
                    {
                        hits.Add(new ConeHit2(tHitInfLine0, pHit0_loc, true));
                        tHitInfLine1 = tHitInfLine0;
                        numHits = 1;
                        return true;
                    }
                    else
                    {
                        hits.Add(new ConeHit2(tHitInfLine0, pHit0_loc, true));
                        hits.Add(new ConeHit2(tHitInfLine1, pHit1_loc, true));
                        hits.Sort();
                        {
                            Vector3 a = hits[1].pnt_loc;
                            Vector3 b = hits[0].pnt_loc;
                            Vector3 ave = (a + b) * .5f;
                            if (ave.z >= 0 &&
                                Vector3.Angle(ave, Vector3.forward) <= coneAngle_deg)
                            {
                                tHitInfLine0 = hits[0].t;
                                tHitInfLine1 = hits[1].t;
                                numHits = 2;
                                return true;
                            }
                        }
                    }
                }
            }

            // This is full infinite line, not worrying about segment yet.
            numHits = hits.Count;
            if (hits.Count == 1)
            {
                Vector3 p = hits[0].pnt_loc;
                if (p.z >= 0 &&
                    Vector3.Angle(p, Vector3.forward) <= coneAngle_deg)
                {
                    tHitInfLine0 = hits[0].t;
                    tHitInfLine1 = hits[0].t;
                    return true;
                }
            }

            numHits = 0;
            return false;
        }

        public static bool IntersectConeVsInfiniteLineSphereCapBase(Vector3 segStart, Vector3 segEnd, Vector3 coneOrigin, Quaternion coneRot, float coneRadius, float coneHeightSphereRad,
            out float tHitInfLine0, out float tHitInfLine1, out bool hit0isCone, out bool hit1isCone)
        {
            Debug.Assert(coneHeightSphereRad > coneRadius, "ConeRadius must be smaller than coneHeightSphere radius. " +
                                                            "Cone radius is the radius of the circle at the intersection of the cone and sphere." +
                                                            "coneHeightSphereRad is the radius of the sphere. ");
            // TODO we are allocating memory every call with this list. We should pass this in as a parameter or make the method non-static and store this buffer in a class instance.
            List<ConeHit2> hits = new List<ConeHit2>(6);

            // Transform _Ray_ so that cone axis is z is forward cone origin is at 0,0,0
            //insideCone = false;
            Vector3 ro = TransformUtils.a_InverseTransformPoint_b(coneOrigin, coneRot, segStart); //cone is at the origin
            Vector3 rd = TransformUtils.a_InverseTransformDirection_b(coneRot, segEnd - segStart);
            float coneAngle_deg = Mathf.Asin(coneRadius / coneHeightSphereRad) * Mathf.Rad2Deg;

            // Compute quadratic cone coefficients
            {
                float coneHeight = Mathf.Sqrt(coneHeightSphereRad * coneHeightSphereRad - coneRadius * coneRadius);
                Debug.Assert(!float.IsNaN(coneHeight), "ConeHeight was NAN");
                float k = coneRadius / coneHeight;
                k = k * k;
                float A = rd.x * rd.x + rd.y * rd.y - k * rd.z * rd.z;
                float B = 2f * (rd.x * ro.x + rd.y * ro.y - k * rd.z * (ro.z));
                float C = ro.x * ro.x + ro.y * ro.y - k * (ro.z) * (ro.z);

                // Solve quadratic equation for _t_ values for infinite cone intersecting infinite line.
                if (!Quadratic(A, B, C, out tHitInfLine0, out tHitInfLine1))
                {
                    hit0isCone = false;
                    hit1isCone = false;
                    return false;
                }

                // Cones are complicated. There can be no hits, one hit, two hits.
                // Also this quadratic solution produces false positives cone is actually an hourglass.
                // Eliminate negative cone hits
                Vector3 pHit0_loc = ro + rd * tHitInfLine0;
                Vector3 pHit1_loc = ro + rd * tHitInfLine1;
                {
                    if (pHit0_loc.z < 0 && pHit1_loc.z < 0)
                    {
                        // all intersections on negative non-existant cone. no real intersections.
                        hit0isCone = false;
                        hit1isCone = false;
                        return false;
                    }
                    else if (pHit0_loc.z < 0 && pHit1_loc.z >= 0)
                    {
                        hits.Add(new ConeHit2(tHitInfLine1, pHit1_loc, true));
                    }
                    else if (pHit0_loc.z >= 0 && pHit1_loc.z < 0)
                    {
                        hits.Add(new ConeHit2(tHitInfLine0, pHit0_loc, true));
                    }
                    else
                    {
                        hits.Add(new ConeHit2(tHitInfLine0, pHit0_loc, true));
                        hits.Add(new ConeHit2(tHitInfLine1, pHit1_loc, true));
                    }
                }
            }

            // Now check in the sphere cap.
            float tSph0, tSph1;
            {
                // Clip against the sphere cap
                Vector3 oc = ro; // ray origin - sphere center
                float A = Vector3.Dot(rd, rd);
                float B = 2.0f * Vector3.Dot(oc, rd);
                float C = Vector3.Dot(oc, oc) - coneHeightSphereRad * coneHeightSphereRad;
                if (!Quadratic(A, B, C, out tSph0, out tSph1))
                {
                    // No intersection with sphere
                    hit0isCone = false;
                    hit1isCone = false;
                    return false;
                }

                Vector3 pSphHit0_loc = ro + rd * tSph0;
                Vector3 pSphHit1_loc = ro + rd * tSph1;
                {
                    if (pSphHit0_loc.z < 0 && pSphHit1_loc.z < 0)
                    {
                        // all intersections on negative cone. no real intersections.
                        hit0isCone = false;
                        hit1isCone = false;
                        return false;
                    }
                    else if (pSphHit0_loc.z < 0 && pSphHit1_loc.z >= 0)
                    {
                        hits.Add(new ConeHit2(tSph1, pSphHit1_loc, false));
                    }
                    else if (pSphHit0_loc.z >= 0 && pSphHit1_loc.z < 0)
                    {
                        hits.Add(new ConeHit2(tSph0, pSphHit0_loc, false));
                    }
                    else
                    {
                        hits.Add(new ConeHit2(tSph0, pSphHit0_loc, false));
                        hits.Add(new ConeHit2(tSph1, pSphHit1_loc, false));
                    }
                }
            }

            // Find the interval of the on the line that is inside the cone and inside the sphere
            Debug.Assert(hits.Count > 1, "How did we get here without some hits?");
            hits.Sort();

            /*
            {
                string s = "Found: " + hits.Count;
                for (int i = 0; i < hits.Count; i++)
                {
                    s += "," + hits[i].t;
                }

                Debug.Log(s);
            }
            */

            // This is full infinite line, not worrying about segment yet.
            for (int i = 1; i < hits.Count; i++)
            {
                Vector3 a = hits[i].pnt_loc;
                Vector3 b = hits[i - 1].pnt_loc;
                Vector3 ave = (a + b) * .5f;
                if (ave.z >= 0 &&
                    Vector3.Angle(ave, Vector3.forward) <= coneAngle_deg && // is inside cone
                    ave.sqrMagnitude < coneHeightSphereRad * coneHeightSphereRad)
                {
                    tHitInfLine0 = hits[i - 1].t;
                    tHitInfLine1 = hits[i].t;
                    hit0isCone = hits[i - 1].hitCone;
                    hit1isCone = hits[i].hitCone;
                    return true;
                }
            }

            hit0isCone = false;
            hit1isCone = false;
            return false;
        }
    }
}
