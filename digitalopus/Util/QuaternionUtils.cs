using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace digitalopus.util
{
    public class QuaternionUtils
    {
        /**
           Decompose the rotation on to 2 parts.
           1. Twist - rotation around the "direction" vector
           2. Swing - rotation around axis that is perpendicular to "direction" vector
           The rotation can be composed back by 
           rotation = swing * twist

           has singularity in case of swing_rotation close to 180 degrees rotation.
           if the input quaternion is of non-unit length, the outputs are non-unit as well
           otherwise, outputs are both unit
        */
        public static void SwingTwistDecomposition(Quaternion rotation,
                                           Vector3 direction,
                                           out Quaternion swing,
                                           out Quaternion twist)
        {
            Vector3 ra = new Vector3(rotation.x, rotation.y, rotation.z); // rotation axis
            Vector3 p = Vector3.Project(ra, direction); // return projection v1 on to v2  (parallel component)
            twist.x = p.x; twist.y = p.y; twist.z = p.z; twist.w = rotation.w;
            twist.Normalize();
            swing = rotation * Quaternion.Inverse(twist);
        }

        /// <summary>
        /// If b_rot_a rotates X degrees about axis A.
        /// 
        /// returns rotation that rotates t01 * X degrees about axis A.
        /// </summary>
        public static Quaternion Lerp(Quaternion b_rot_a, float t01)
        {
            if (b_rot_a.w >= 1f) return b_rot_a;
            if (b_rot_a.w < -1f)
            {
                Vector3 axisDir = Vector3.right;
                float angHalfTheta_rad = Mathf.PI * t01;
                Quaternion q;
                q.x = Mathf.Sin(angHalfTheta_rad);
                q.y = 0f;
                q.z = 0f;
                q.w = Mathf.Cos(angHalfTheta_rad);
                return q;
            } else
            {
                float angHalfTheta_rad = Mathf.Acos(b_rot_a.w);
                angHalfTheta_rad *= t01;
                float sinHalfTheta = Mathf.Sin(angHalfTheta_rad);
                float magInv = 1f / Mathf.Sqrt(b_rot_a.x * b_rot_a.x + b_rot_a.y * b_rot_a.y + b_rot_a.z * b_rot_a.z);
                Quaternion q;
                q.x = b_rot_a.x * sinHalfTheta * magInv;
                q.y = b_rot_a.y * sinHalfTheta * magInv;
                q.z = b_rot_a.z * sinHalfTheta * magInv;
                q.w = Mathf.Cos(angHalfTheta_rad);
                return q;
            }
        }

        /// <summary>
        /// From Unity's AnimationRigging package. Not sure why we need it.
        /// Calculates a Quaternion rotation of one vector to another.
        /// </summary>
        /// <param name="from">Starting vector.</param>
        /// <param name="to">Destination vector.</param>
        /// <returns>Quaternion rotation.</returns>
        public static Quaternion FromToRotation_AnimRigging(Vector3 from, Vector3 to)
        {
            float theta = Vector3.Dot(from.normalized, to.normalized);
            if (theta >= 1f)
                return Quaternion.identity;

            if (theta <= -1f)
            {
                Vector3 axis = Vector3.Cross(from, Vector3.right);
                if (axis.sqrMagnitude == 0f)
                    axis = Vector3.Cross(from, Vector3.up);

                return Quaternion.AngleAxis(180f, axis);
            }

            return Quaternion.AngleAxis(Mathf.Acos(theta) * Mathf.Rad2Deg, Vector3.Cross(from, to).normalized);
        }

        public static bool IsValidQuat(Quaternion inertiaRotationOverride)
        {
            if (Mathf.Abs(Magnitude(inertiaRotationOverride) - 1f) < 1e-5f) return true;
            return false;
        }

        /// <summary>
        /// There are two equivalent quaternions that describe every rotation, a short path and a long path round the sphere.
        /// </summary>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Quaternion ConvertToSmallestPathAroundSphere(Quaternion q)
        {
            if (q.w < 0f)
            {
                return new Quaternion(-q.x, -q.y, -q.z, -q.w);
            }
            else
            {
                return q;
            }
        }

        public static void ConvertToSmallestPathAroundSphere(ref Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }
        }

        public static void ConvertToClosestToTarget(in Quaternion a_RotTarget_b, ref Quaternion a_RotQ_b)
        {
            Quaternion a_RotDelta_a = a_RotTarget_b * Quaternion.Inverse(a_RotQ_b);
            if (a_RotDelta_a.w < 0f)
            {
                ConvertToRotateOtherDirectionAroundSphere(ref a_RotQ_b);
            }
        }

        public static void ConvertToRotateOtherDirectionAroundSphere(ref Quaternion q)
        {
            q.x = -q.x;
            q.y = -q.y;
            q.z = -q.z;
            q.w = -q.w;
        }

        public static bool CompareIgnoreDoubleCoverage(Quaternion aa, Quaternion bb)
        {
            Quaternion a = ConvertToSmallestPathAroundSphere(aa);
            Quaternion b = ConvertToSmallestPathAroundSphere(bb);

            if (Mathf.Abs(aa.w) < 10e-8f && Mathf.Abs(bb.w) < 10e-8f)
            {
                // We still might need to flip one of the quats
                Vector3 aaa = new Vector3(a.x, a.y, a.z);
                Vector3 bbb = new Vector3(b.x, b.y, b.z);
                if (Vector3.Dot(aaa, bbb) < 0f)
                {
                    b.x = -b.x;
                    b.y = -b.y;
                    b.z = -b.z;
                    b.w = -b.w;
                }
            }

            Vector4 deltaQ;
            deltaQ.x = a.x - b.x;
            deltaQ.y = a.y - b.y;
            deltaQ.z = a.z - b.z;
            deltaQ.w = a.w - b.w;
            return deltaQ.magnitude < 10e-6f;
        }

        public static Quaternion FindQuaternionTwist(Quaternion q, Vector3 axis, Vector3 normalToAxis)
        {
            Debug.Assert(Mathf.Abs(Vector3.Dot(axis, normalToAxis)) < 10e-8f);

            Vector3 transformed = q * normalToAxis;

            // Project transformed vector onto plane
            Vector3 flattened = Vector3.ProjectOnPlane(transformed, axis);
            flattened.Normalize();

            // Get angle between original vector and projected transform to get angle around normal
            return Quaternion.FromToRotation(normalToAxis, flattened);
        }

        public static Quaternion FindQuaternionDifference_0_360(Quaternion final, Quaternion initial)
        {
            Quaternion diff = final * Quaternion.Inverse(initial);
            return diff;
        }

        public static Quaternion FindQuaternionDifference_0_180(Quaternion final, Quaternion initial)
        {
            Quaternion diff = final * Quaternion.Inverse(initial);
            if (diff.w < 0f)
            {
                diff.x = -diff.x; diff.y = -diff.y; diff.z = -diff.z; diff.w = -diff.w;
                diff.Set(-diff.x, -diff.y, -diff.z, -diff.w);
                //diff.Normalize();
            }
            return diff;
        }

        public static float Magnitude(Quaternion q)
        {
            return Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        }

        public static float GetAngle_deg(Quaternion q)
        {
            Debug.Assert(q.w > -1.0001f && q.w < 1.0001f, " quaternion.w should be between 1 and -1. was: " + q.w);
            return 2f * Mathf.Rad2Deg * Mathf.Acos(Mathf.Clamp(q.w,-1f, 1f));
        }

        public static Vector3 GetAngleAxis_PseudoVector(Quaternion q)
        {
            float a;
            Vector3 ax;
            q.ToAngleAxis(out a, out ax);
            return ax.normalized * (a * Mathf.Deg2Rad);
        }

        public static Vector3 ToAngularVelocity(Quaternion deltaRot, float dt)
        {
            float angleInDegrees;
            Vector3 rotationAxis;
            Debug.Log("AngVelo raw: " + deltaRot.ToString("F5"));
            if (deltaRot.w < 0f)
            {
                deltaRot.x = -deltaRot.x;
                deltaRot.y = -deltaRot.y;
                deltaRot.z = -deltaRot.z;
                deltaRot.w = -deltaRot.w;
            }

            deltaRot.ToAngleAxis(out angleInDegrees, out rotationAxis);
            Debug.Log("AngVelo In Deg per sec: " + (angleInDegrees / dt).ToString("F5") + "  " + (angleInDegrees).ToString("F5"));
            return rotationAxis * (angleInDegrees * Mathf.Deg2Rad / dt);
        }

        public static string ToStringAngAxis(Quaternion q)
        {
            Vector3 axis;
            float angle_deg;
            q.ToAngleAxis(out angle_deg, out axis);
            return string.Format("(axis:{0} angle:{1})", axis, angle_deg);
        }

        public static Vector3 ToAngularVelocity(Quaternion from, Quaternion to, float dt)
        {
            Quaternion deltaRot = to * Quaternion.Inverse(from);
            return ToAngularVelocity(deltaRot, dt);
        }

        public static Quaternion AngleAxisToQuaternion(Vector3 angAxisRotation_rad)
        {
            float angle_rad = angAxisRotation_rad.magnitude;
            float s = Mathf.Sin(angle_rad / 2f);

            if (angle_rad > 1e-8f)
            {
                angAxisRotation_rad /= angle_rad; // normalize
                Quaternion q;
                q.x = angAxisRotation_rad.x * s;
                q.y = angAxisRotation_rad.y * s;
                q.z = angAxisRotation_rad.z * s;
                q.w = Mathf.Cos(angle_rad / 2f);
                return q;
            } else
            {
                return Quaternion.identity;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        public static bool EqualsExact(Quaternion a, Quaternion b)
        {
            return  a.x == b.x && 
                    a.y == b.y && 
                    a.z == b.z && 
                    a.w == b.w;
        }

        /// <summary>
        /// 
        /// </summary>
        public static bool EqualsApprox(Quaternion a, Quaternion b)
        {
            float angle_deg = Quaternion.Angle(a, b);
            return Mathf.Abs(angle_deg) < .5f;
        }

        internal static Vector3 CalcPerpToQuatAxis(Quaternion q, Vector3 hintA, Vector3 hintB)
        {
            Vector3 axis = new Vector3(q.x, q.y, q.z);
            float mag = axis.magnitude;
            if (mag < 1e-10f) return hintA; // no rotation, any axis will do.
            Vector3 axisN = axis / mag;
            hintA.Normalize();
            Vector3 hint;
            if (Vector3.Dot(hintA, axisN) > (1f - 1e-3f))
            {
                hint = hintB;
            }
            else
            {
                hint = hintA;
            }

            Vector3 perp = Vector3.ProjectOnPlane(hint, axisN);
            perp.Normalize();
            return perp;
        }
    }
}
