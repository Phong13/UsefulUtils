using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using digitalopus.core;
using digitalopus.util;

namespace digitalopus.tranform
{
    public class TransformUtils
    {
        public Vector3 testPoint;
        public Vector3 testDir;

        /// <summary>
        /// Transforms from child frame 'a' to parent frame 'b'
        /// </summary>
        [StructLayout(LayoutKind.Sequential), System.Serializable]
        public struct GTransform
        {
            public const int SIZE_BYTES = 28;

            public static GTransform identity = new GTransform()
            {
                b_rot_a = Quaternion.identity,
                pivot_b = Vector3.zero,

            };

            /// <summary>
            /// Rotation from a to b in the b frame
            /// </summary>
            public Quaternion b_rot_a;
            public Vector3 pivot_b;

            public static GTransform CreateTransform(Transform tt)
            {
                GTransform t;
                t.b_rot_a = tt.rotation;
                t.pivot_b = tt.position;
                return t;
            }

            public static GTransform CreateTransform(Quaternion b_Rot_a, Vector3 pivot_b)
            {
                GTransform t;
                t.b_rot_a = b_Rot_a;
                t.pivot_b = pivot_b;
                return t;
            }

            public static GTransform CreateIdentity()
            {
                GTransform t;
                t.b_rot_a = Quaternion.identity;
                t.pivot_b = Vector3.zero;
                return t;
            }

            public void NormalizeQuatSmallest()
            {
                b_rot_a.Normalize();
                b_rot_a = QuaternionUtils.ConvertToSmallestPathAroundSphere(b_rot_a);
            }

            /// <summary>
            /// If a_inc_a translates by X and rotates by R radians
            /// then the output is a smaller translation-rotation by a factor of t.
            /// </summary>
            public static GTransform Interpolate(GTransform a_inc_a, float t)
            {
                GTransform a_PartialInc_a;
                a_PartialInc_a.pivot_b = a_inc_a.pivot_b * t;
                a_PartialInc_a.b_rot_a = QuaternionUtils.Lerp(a_inc_a.b_rot_a, t);
                return a_PartialInc_a;
            }

            public static GTransform Lerp(GTransform a, GTransform b, float t)
            {
                GTransform c;
                c.pivot_b = Vector3.Lerp(a.pivot_b, b.pivot_b, t);
                c.b_rot_a = Quaternion.Slerp(a.b_rot_a, b.b_rot_a, t);
                return c;
            }

            public static GTransform Multiply(GTransform a, GTransform b)
            {
                GTransform t;
                t.b_rot_a = a.b_rot_a * b.b_rot_a;
                t.pivot_b = a.pivot_b + a.b_rot_a * b.pivot_b;
                return t;
            }

            // Have transform from A to B in A
            // What is transform from B to A in B
            // Tested pretty sure this works
            public static GTransform Inverse(GTransform a)
            {
                GTransform t;
                t.b_rot_a = Quaternion.Inverse(a.b_rot_a);
                t.pivot_b = t.b_rot_a * (-a.pivot_b);
                return t;
            }

            public GTransform Inverse()
            {
                return Inverse(this);
            }

            public override bool Equals(object obj)
            {
                return pivot_b.Equals(((GTransform)obj).pivot_b) &&
                       b_rot_a.Equals(((GTransform)obj).b_rot_a);
            }

            public bool EqualsApproximate(GTransform obj)
            {
                Vector3 delta = pivot_b - obj.pivot_b;
                bool aa = delta.magnitude < 1e-4f;
                bool bb = QuaternionUtils.CompareIgnoreDoubleCoverage(b_rot_a, obj.b_rot_a);

                // if (!(aa && bb)) UnityEngine.Debug.LogError( "failed " + aa + "  " + bb + "     "  + PrintDifference(ref obj));

                return aa && bb;
            }

            public string PrintDifference(ref GTransform b)
            {
                string s = string.Format("posDiff:{0}  rotDiff(deg):{1}", Vector3.Distance(pivot_b, b.pivot_b), Quaternion.Angle(b_rot_a, b.b_rot_a).ToString("f5"));
                return s;
            }

            public override int GetHashCode()
            {
                return pivot_b.GetHashCode() ^ b_rot_a.GetHashCode();
            }

            public GTransform __Multiply__(GTransform other)
            {
                return Multiply(this, other);
            }

            public Vector3 b_TransformPoint_a(Vector3 p_a)
            {
                return TransformUtils.b_TransformPoint_a(pivot_b, b_rot_a, p_a);
            }

            public Vector3 a_InverseTransformPoint_b(Vector3 p_b)
            {
                return TransformUtils.a_InverseTransformPoint_b(pivot_b, b_rot_a, p_b);
            }

            public Vector3 b_TransformDirection_a(Vector3 dir_a)
            {
                return TransformUtils.b_TransformDirection_a(b_rot_a, dir_a);
            }

            public Vector3 a_InverseTransformDirection_b(Vector3 dir_b)
            {
                return TransformUtils.a_InverseTransformDirection_b(b_rot_a, dir_b);
            }

            public static GTransform operator *(GTransform a, GTransform b) { return Multiply(a, b); }

            public override string ToString()
            {
                return "(p:" + pivot_b.ToString("f8") + " r:" + b_rot_a.ToString("f8") + ")";
            }

            public void DrawTransformGizmos()
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(pivot_b, .1f);
                GizmoUtils.DrawTransform(pivot_b, b_rot_a, .75f);
            }
        }

        /// <summary> Local to world. like Transform.TransformPoint pivot is Transform.position, rot is Transform.rotation</summary>
        public static Vector3 b_TransformPoint_a(Vector3 pivot_b, Quaternion b_Rot_a, Vector3 p_a)
        {
            return b_Rot_a * p_a + pivot_b;
        }

        /// <summary>
        /// Child should be a hierarchy child of parent. The _loc rotations are relative to parent.
        /// </summary>
        public static Vector3 GetPosParentGivenChildPosAndRot(Vector3 childPivot_wld, Quaternion childRot_wld, Vector3 childPos_loc, Quaternion childRot_loc)
        {
            // childPivot_wld = childRot_loc * childPos_loc + posParent_wld;
            Quaternion parentRot_wld = childRot_wld * Quaternion.Inverse(childRot_loc);
            return childPivot_wld - parentRot_wld * childPos_loc;
        }

        /// <summary> World to local. like Transform.TransformPoint pivot is Transform.position, rot is Transform.rotation </summary>
        public static Vector3 a_InverseTransformPoint_b(Vector3 pivot_b, Quaternion b_Rot_a, Vector3 p_b)
        {
            Quaternion a_Rot_b = Quaternion.Inverse(b_Rot_a);
            return a_Rot_b * (p_b - pivot_b);
        }

        public static Vector3 b_TransformDirection_a(Quaternion b_Rot_a, Vector3 dir_a)
        {
            return b_Rot_a * dir_a;
        }

        public static Vector3 a_InverseTransformDirection_b(Quaternion b_Rot_a, Vector3 dir_b)
        {
            Quaternion a_Rot_b = Quaternion.Inverse(b_Rot_a);
            return a_Rot_b * dir_b;
        }

        public static Quaternion b_GetALocalRotationInB_a(Quaternion wld_Rot_a, Quaternion wld_Rot_b)
        {
            return Quaternion.Inverse(wld_Rot_b) * wld_Rot_a;
        }

        public static bool IsUnitVector(Vector3 v)
        {
            return Mathf.Abs(v.sqrMagnitude - 1) < 1e-5f;
        }

        public static bool IsFinite(Vector3 v)
        {
            return float.IsFinite(v.x) && float.IsFinite(v.y) && float.IsFinite(v.z);
        }

        /// <summary>
        /// x = Yaw is a rotation (-Pi, PI) in the x,z plane assuming Z is forward.
        /// y = Pitch is tilt (-Pi, Pi) from that plane
        /// z = Rad is dist
        /// </summary>
        public static Vector3 Cartesian2Polar_YawPitchDist_radians(Vector3 p)
        {
            float r = p.magnitude;
            if (r < Mathf.Epsilon) return Vector3.zero;
            float yaw_rad = Mathf.Atan2(p.x, p.z);
            float r_xz = Mathf.Sqrt(p.x * p.x + p.z * p.z);
            float pitch_rad = Mathf.Atan2(p.y, r_xz);
            return new Vector3(yaw_rad, pitch_rad, r);
        }

        public static Vector3 Polar2Cartesian_YawPitchDist_radians(Vector3 polar_rad)
        {
            float yaw_rad = polar_rad.x;
            float pitch_rad = polar_rad.y;
            float r_xz = Mathf.Cos(pitch_rad) * polar_rad.z;
            float x = Mathf.Sin(yaw_rad) * r_xz;
            float z = Mathf.Cos(yaw_rad) * r_xz;
            float y = Mathf.Sin(pitch_rad) * polar_rad.z;
            return new Vector3(x, y, z);
        }

        public static Vector3 Polar_Rad2Deg(Vector3 polarRad)
        {
            return new Vector3(polarRad.x * Mathf.Rad2Deg, polarRad.y * Mathf.Rad2Deg, polarRad.z);
        }

        public static void TestPolarConversion()
        {
            Debug.Log("5, 0, 5 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(5, 0, 5))));
            Debug.Log("-5, 0, 5 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(-5, 0, 5))));
            Debug.Log("0, 5, 5 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(0, 5, 5))));
            Debug.Log("0, 5, 0 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(0, 5, 0))));
            Debug.Log("0, 0, 0 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(0, 0, 0))));
            Debug.Log("0, 0, -5 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(0, 0, -5))));
            Debug.Log("0, -5, 5 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(0, -5, 5))));
            Debug.Log("5, 5, 5 =>  " + Polar_Rad2Deg(TransformUtils.Cartesian2Polar_YawPitchDist_radians(new Vector3(5, 5, 5))));
        }

        public static void TestPolarConversionToFrom()
        {
            for (int i = 0; i < 1000; i++)
            {
                Vector3 p = Random.onUnitSphere * Random.Range(0, 100);
                Vector3 pp = TransformUtils.Polar2Cartesian_YawPitchDist_radians(TransformUtils.Cartesian2Polar_YawPitchDist_radians(p));
                Debug.Assert(Vector3.Distance(p, pp) < 1e-4f, "Bad dist: " + Vector3.Distance(p, pp));
            }
        }

        public void TestTransformUtils(Transform tmpTA)
        {
            Quaternion wld_Rot_loc = Random.rotation;
            Vector3 p_wld = Random.onUnitSphere * Random.Range(1f, 10f);
            TransformUtils.GTransform wld_X_loc = TransformUtils.GTransform.CreateTransform(wld_Rot_loc, p_wld);
            tmpTA.position = p_wld;
            tmpTA.rotation = wld_Rot_loc;

            // Relative to frame A generate a random force acting at a random point.
            // Calc spatial force in frame A
            Vector3 dirA = Random.onUnitSphere;
            Vector3 pA = Random.onUnitSphere * Random.Range(1f, 10f);

            Vector3 pB = wld_X_loc.b_TransformPoint_a(pA);
            Vector3 pBB = tmpTA.TransformPoint(pA);

            Vector3 dirB = wld_X_loc.b_TransformDirection_a(dirA);
            Vector3 dirBB = tmpTA.TransformDirection(dirA);

            Vector3 dirAA = wld_X_loc.a_InverseTransformDirection_b(dirB);
            Vector3 dirAAA = tmpTA.InverseTransformDirection(dirBB);

            Vector3 pAA = wld_X_loc.a_InverseTransformPoint_b(pB);
            Vector3 pAAA = tmpTA.InverseTransformPoint(pBB);

            TransformUtils.GTransform loc_X_wld = wld_X_loc.Inverse();

            Vector3 pAAAA = loc_X_wld.b_TransformPoint_a(pB);
            Vector3 dirAAAA = loc_X_wld.b_TransformDirection_a(dirB);

            Debug.Log(dirA.ToString("f5") + "\n" + dirAA.ToString("f5") + "\n" + dirAAA.ToString("f5") + "\n" + dirAAAA.ToString("f5"));
            Debug.Log(dirB.ToString("f5") + "\n" + dirBB.ToString("f5"));
            Debug.Log(pA.ToString("f5") + "\n" + pAA.ToString("f5") + "\n" + pAAA.ToString("f5") + "\n" + pAAAA.ToString("f5"));
            Debug.Log(pB.ToString("f5") + "\n" + pBB.ToString("f5"));

            Vector3 rA = Random.onUnitSphere * Random.Range(1f, 10f);
            Vector3 rB = wld_X_loc.b_TransformPoint_a(rA);
            Debug.Assert(loc_X_wld.b_TransformPoint_a(rB) == rA, loc_X_wld.b_TransformPoint_a(rB) + "  " + rA);
        }
    }
}
