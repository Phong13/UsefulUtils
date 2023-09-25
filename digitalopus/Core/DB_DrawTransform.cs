using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace digitalopus.core
{
    public class DB_DrawTransform : MonoBehaviour
    {
        public float scale = .5f;
        public Color gizmoColor = Color.gray;

        private void OnDrawGizmos()
        {
            GizmoUtils.DrawTransform(transform, scale, gizmoColor);
            Gizmos.color = gizmoColor;
            Gizmos.DrawSphere(transform.position, scale * .03f);
        }
    }
}
