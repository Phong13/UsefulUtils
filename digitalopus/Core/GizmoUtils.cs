using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using digitalopus.tranform;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace digitalopus.core
{
    public class GizmoUtils
    {
        public static void DrawOrientedPlane(Vector3 planePosition, Vector3 planeNormal)
        {
            DrawOrientedPlane(planePosition, planeNormal, 1f);
        }

        public static void DrawOrientedPlane(Vector3 planePosition, Vector3 planeNormal, float scale)
        {
            Quaternion rotation = Quaternion.LookRotation(planeNormal);
            Matrix4x4 trs = Matrix4x4.TRS(planePosition, rotation, Vector3.one);
            Matrix4x4 tmp = Gizmos.matrix;
            Gizmos.matrix = trs;
            Gizmos.DrawWireCube(Vector3.zero, new Vector3(scale, scale, 0.0001f));
            Gizmos.matrix = tmp;
        }

        public static void DrawTransform(Transform t, float scale)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = Color.red;
            Gizmos.DrawRay(t.position, t.right * scale);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(t.position, t.up * scale);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(t.position, t.forward * scale);
            Gizmos.color = oldColor;
        }

        public static void DrawTransform(Transform t, float scale, Color tintColor)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = Color.Lerp(Color.red, tintColor, .5f);
            Gizmos.DrawRay(t.position, t.right * scale);
            Gizmos.color = Color.Lerp(Color.green, tintColor, .5f);
            Gizmos.DrawRay(t.position, t.up * scale);
            Gizmos.color = Color.Lerp(Color.blue, tintColor, .5f);
            Gizmos.DrawRay(t.position, t.forward * scale);
            Gizmos.color = oldColor;
        }

        public static void DrawTransform(digitalopus.tranform.TransformUtils.GTransform t, float scale)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = Color.red;
            Gizmos.DrawRay(t.pivot_b, t.b_rot_a * Vector3.right * scale);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(t.pivot_b, t.b_rot_a * Vector3.up * scale);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(t.pivot_b, t.b_rot_a * Vector3.forward * scale);
            Gizmos.color = oldColor;
        }

        public static void DrawTransform(Vector3 position, Quaternion rotation, float scale)
        {
            Color oldColor = Gizmos.color;
            Gizmos.color = Color.red;
            Gizmos.DrawRay(position, rotation * Vector3.right * scale);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(position, rotation * Vector3.up * scale);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(position, rotation * Vector3.forward * scale);
            Gizmos.color = oldColor;
        }

        /*
        public static void DrawConeGizmo(Vector3 tip, Vector3 axis, float height, float radius)
        {
            Vector3 ax = axis.normalized;
            Vector3 basePnt = tip + axis.normalized * height;
            Vector3 perp;
            {
                Vector3 offset = Vector3.up;
                if (Vector3.Angle(offset, ax) < 5f) offset = Vector3.forward;
                perp = Vector3.ProjectOnPlane(ax, offset).normalized;
            }
            DrawWireCircle(basePnt, axis, basePnt + perp, radius);
            Gizmos.DrawLine(basePnt, tip);
        }
        */
        public static void DrawConeRoundBottom(float ang_deg, Vector3 tip, Vector3 axis, float coneRadius)
        {
            float bottomRad = Mathf.Tan(ang_deg * Mathf.Deg2Rad) * coneRadius;
            DrawConeRoundBottom(tip, axis, bottomRad, coneRadius);
        }

        public static void DrawConeRoundBottom(Vector3 tip, Vector3 axis, float bottomSphereRad, float coneRadius)
        {
            Debug.Assert(bottomSphereRad > coneRadius, "bottomSphere must be larger than coneRadius.");
            float ht = Mathf.Sqrt(bottomSphereRad * bottomSphereRad - coneRadius * coneRadius);
            Vector3 basePnt = tip + axis.normalized * ht;
            axis.Normalize();
            Vector3 approxStartInPlaneHint;
            {
                Vector3 offset = Vector3.up;
                if (Vector3.Angle(offset, axis) < 5f) offset = Vector3.forward;
                approxStartInPlaneHint = basePnt + Vector3.ProjectOnPlane(axis, offset).normalized;
            }

            {
                float corners = 16; // How many corners the circle should have
                Vector3 spoke = Vector3.ProjectOnPlane(approxStartInPlaneHint, axis).normalized * coneRadius; // Where the first point of the circle starts
                Quaternion segmentRot = Quaternion.AngleAxis(360f / corners, axis);
                Vector3 lastPosition = basePnt + spoke;
                float angle = 0f;
                int spokeIdx = 0;
                while (angle <= 360f)
                {
                    angle += 360f / corners;
                    Vector3 nextSpoke = segmentRot * spoke;
                    Vector3 nextPosition = basePnt + nextSpoke;
                    Gizmos.DrawLine(lastPosition, nextPosition);
                    if (spokeIdx % 3 == 0)
                    {
                        Gizmos.DrawLine(tip, nextPosition);
                    }
                    spoke = nextSpoke;
                    lastPosition = nextPosition;
                    spokeIdx++;
                }
            }

            Gizmos.color = Gizmos.color * .5f;
            {
                float innerRad = coneRadius * .3f;
                float corners = 16; // How many corners the circle should have
                Vector3 spoke = Vector3.ProjectOnPlane(approxStartInPlaneHint, axis).normalized * innerRad; // Where the first point of the circle starts
                Quaternion segmentRot = Quaternion.AngleAxis(360f / corners, axis);
                ht = Mathf.Sqrt(bottomSphereRad * bottomSphereRad - innerRad * innerRad);
                basePnt = tip + axis.normalized * ht;
                Vector3 lastPosition = basePnt + spoke;
                float angle = 0f;
                int spokeIdx = 0;
                while (angle <= 360f)
                {
                    angle += 360f / corners;
                    Vector3 nextSpoke = segmentRot * spoke;
                    Vector3 nextPosition = basePnt + nextSpoke;
                    Gizmos.DrawLine(lastPosition, nextPosition);
                    spoke = nextSpoke;
                    lastPosition = nextPosition;
                    spokeIdx++;
                }
            }

            {
                float innerRad = coneRadius * .6f;
                float corners = 12; // How many corners the circle should have
                Vector3 spoke = Vector3.ProjectOnPlane(approxStartInPlaneHint, axis).normalized * innerRad; // Where the first point of the circle starts
                Quaternion segmentRot = Quaternion.AngleAxis(360f / corners, axis);
                ht = Mathf.Sqrt(bottomSphereRad * bottomSphereRad - innerRad * innerRad);
                basePnt = tip + axis.normalized * ht;
                Vector3 lastPosition = basePnt + spoke;
                float angle = 0f;
                int spokeIdx = 0;
                while (angle <= 360f)
                {
                    angle += 360f / corners;
                    Vector3 nextSpoke = segmentRot * spoke;
                    Vector3 nextPosition = basePnt + nextSpoke;
                    Gizmos.DrawLine(lastPosition, nextPosition);
                    spoke = nextSpoke;
                    lastPosition = nextPosition;
                    spokeIdx++;
                }
            }
        }


        public static void DrawWireCircle(Vector3 origin, Vector3 normal, Vector3 approxStartInPlaneHint, float radius)
        {
            float corners = 11; // How many corners the circle should have
            Vector3 spoke = Vector3.ProjectOnPlane(approxStartInPlaneHint, normal).normalized * radius; // Where the first point of the circle starts
            Quaternion segmentRot = Quaternion.AngleAxis(360f / corners, normal);
            Vector3 lastPosition = origin + spoke;
            float angle = 0f;
            while (angle <= 360f)
            {
                angle += 360f / corners;
                Vector3 nextSpoke = segmentRot * spoke;
                Vector3 nextPosition = origin + nextSpoke;
                Gizmos.DrawLine(lastPosition, nextPosition);
                spoke = nextSpoke;
                lastPosition = nextPosition;
            }
        }

        public static void DrawQuat(Vector3 p, Quaternion qq)
        {
            DrawQuat(p, qq, Vector3.one);
        }

        /// <summary>
        /// TODO there are some cases where this draws backwards depending on approxStartInPlaneHint
        /// </summary>
        public static void DrawQuat(Vector3 p, Quaternion qq, Vector3 approxStartInPlaneHint)
        {
            Vector3 ax = new Vector3(qq.x, qq.y, qq.z);
            Vector3 axNorm = ax.normalized;
            float ang = 2f * Mathf.Acos(qq.w);
            Color cc = Gizmos.color;
            Color fadedC = Color.Lerp(cc, Color.clear, .5f);
            Gizmos.color = fadedC;
            Gizmos.DrawLine(p, p + axNorm * .5f);
            GizmoUtils.DrawWireArc(p, ax, .5f, approxStartInPlaneHint, ang * Mathf.Rad2Deg);
        }

        public static void DrawWireArc(Vector3 origin, Vector3 axis, float radius, Vector3 approxStartInPlaneHint, float arcLength_deg)
        {
            Color cc = Gizmos.color;
            Color fadedC = Color.Lerp(cc, Color.clear, .5f);
            DrawWireArc(origin, axis, radius, approxStartInPlaneHint, arcLength_deg, fadedC, cc);
        }

        public static void DrawWireArc(Vector3 origin, Vector3 axis, float radius, Vector3 approxStartInPlaneHint, float arcLength_deg, Color fromC, Color toC)
        {
            float corners = 21; // How many corners the circle should have
            Vector3 spoke = Vector3.ProjectOnPlane(approxStartInPlaneHint, axis).normalized * radius; // Where the first point of the circle starts
            Quaternion segmentRot = Quaternion.AngleAxis(arcLength_deg / corners, axis);
            Vector3 lastPosition = origin + spoke;
            float angle = 0f;
            float incr = arcLength_deg / corners;
            Gizmos.color = fromC;
            Gizmos.DrawRay(origin, spoke);
            Gizmos.DrawWireSphere(origin + spoke, .01f);
            int numSpoke = 0;
            if (Mathf.Abs(arcLength_deg) > 1e-6f)
            {
                while (Mathf.Abs(angle) <= Mathf.Abs(arcLength_deg) * 1.0001f)
                {
                    angle += arcLength_deg / corners;
                    Vector3 nextSpoke = segmentRot * spoke;
                    Vector3 nextPosition = origin + nextSpoke;
                    Gizmos.DrawLine(lastPosition, nextPosition);
                    spoke = nextSpoke;
                    numSpoke++;
                    lastPosition = nextPosition;
                }
            }

            Gizmos.color = toC;
            Gizmos.DrawRay(origin, spoke);
            Gizmos.DrawSphere(origin + spoke, .01f);
        }

        public static void DrawWireCapsule(Vector3 _pos, Vector3 _pos2, float _radius, Color _color)
        {
#if UNITY_EDITOR
            if (_color != default) Handles.color = _color;

            var forward = _pos2 - _pos;
            var _rot = Quaternion.LookRotation(forward);
            var pointOffset = _radius / 2f;
            var length = forward.magnitude;
            var center2 = new Vector3(0f, 0, length);

            Matrix4x4 angleMatrix = Matrix4x4.TRS(_pos, _rot, Handles.matrix.lossyScale);

            using (new Handles.DrawingScope(angleMatrix))
            {
                Handles.DrawWireDisc(Vector3.zero, Vector3.forward, _radius);
                Handles.DrawWireArc(Vector3.zero, Vector3.up, Vector3.left * pointOffset, -180f, _radius);
                Handles.DrawWireArc(Vector3.zero, Vector3.left, Vector3.down * pointOffset, -180f, _radius);
                Handles.DrawWireDisc(center2, Vector3.forward, _radius);
                Handles.DrawWireArc(center2, Vector3.up, Vector3.right * pointOffset, -180f, _radius);
                Handles.DrawWireArc(center2, Vector3.left, Vector3.up * pointOffset, -180f, _radius);

                DrawLine(_radius, 0f, length);
                DrawLine(-_radius, 0f, length);
                DrawLine(0f, _radius, length);
                DrawLine(0f, -_radius, length);
            }
#endif
        }

        public static void DrawWireCapsule(Vector3 _pos, Quaternion _rot, Vector3 _center_loc, float _radius, float _height, Color _color = default(Color))
        {
#if UNITY_EDITOR
            if (_color != default(Color)) Handles.color = _color;
            _pos += digitalopus.tranform.TransformUtils.b_TransformDirection_a(_rot, _center_loc);
            Matrix4x4 angleMatrix = Matrix4x4.TRS(_pos, _rot, Handles.matrix.lossyScale);
            using (new Handles.DrawingScope(angleMatrix))
            {
                var pointOffset = (_height - (_radius * 2)) / 2;

                //draw sideways
                Handles.DrawWireArc(Vector3.up * pointOffset, Vector3.left, Vector3.back, -180, _radius);
                Handles.DrawLine(new Vector3(0, pointOffset, -_radius), new Vector3(0, -pointOffset, -_radius));
                Handles.DrawLine(new Vector3(0, pointOffset, _radius), new Vector3(0, -pointOffset, _radius));
                Handles.DrawWireArc(Vector3.down * pointOffset, Vector3.left, Vector3.back, 180, _radius);
                //draw frontways
                Handles.DrawWireArc(Vector3.up * pointOffset, Vector3.back, Vector3.left, 180, _radius);
                Handles.DrawLine(new Vector3(-_radius, pointOffset, 0), new Vector3(-_radius, -pointOffset, 0));
                Handles.DrawLine(new Vector3(_radius, pointOffset, 0), new Vector3(_radius, -pointOffset, 0));
                Handles.DrawWireArc(Vector3.down * pointOffset, Vector3.back, Vector3.left, -180, _radius);
                //draw center
                Handles.DrawWireDisc(Vector3.up * pointOffset, Vector3.up, _radius);
                Handles.DrawWireDisc(Vector3.down * pointOffset, Vector3.up, _radius);

            }
#endif
        }

        /*
        public static void DrawWireCapsule(Vector3 p, Quaternion r, CapsuleCollider cc, Color _color)
        {
    #if UNITY_EDITOR
            if (_color != default) Handles.color = _color;

            Vector3 unit_loc = Vector3.zero;
            if (cc.direction == 0) unit_loc = new Vector3(1, 0, 0);
            if (cc.direction == 1) unit_loc = new Vector3(0, 1, 0);
            if(cc.direction == 2) unit_loc = new Vector3(0, 0, 2);

            float _radius = cc.radius;

            float halfHeight = cc.height / 2f - _radius;
            Vector3 _pos_wld = TransformUtils.TransformPointLocal2Wld(p, r, cc.center -  unit_loc * halfHeight);

            //var forward = _pos2 - _pos;
            //var _rot = Quaternion.LookRotation(forward);
            var pointOffset = _radius / 2f;
            var length = halfHeight * 2f;
            var center2_wld = _pos_wld + TransformUtils.TransformDirection(r, unit_loc * length);

            Matrix4x4 angleMatrix = Matrix4x4.TRS(_pos_wld, r, Handles.matrix.lossyScale);

            using (new Handles.DrawingScope(angleMatrix))
            {
                Handles.DrawWireDisc(Vector3.zero, Vector3.forward, _radius);
                Handles.DrawWireArc(Vector3.zero, Vector3.up, Vector3.left * pointOffset, -180f, _radius);
                Handles.DrawWireArc(Vector3.zero, Vector3.left, Vector3.down * pointOffset, -180f, _radius);
                Handles.DrawWireDisc(center2_wld, Vector3.forward, _radius);
                Handles.DrawWireArc(center2_wld, Vector3.up, Vector3.right * pointOffset, -180f, _radius);
                Handles.DrawWireArc(center2_wld, Vector3.left, Vector3.up * pointOffset, -180f, _radius);

                DrawLine(_radius, 0f, length);
                DrawLine(-_radius, 0f, length);
                DrawLine(0f, _radius, length);
                DrawLine(0f, -_radius, length);
            }
    #endif
        }
        */

        private static void DrawLine(float arg1, float arg2, float forward)
        {
#if UNITY_EDITOR
            Handles.DrawLine(new Vector3(arg1, arg2, 0f), new Vector3(arg1, arg2, forward));
#endif
        }
    }
}
