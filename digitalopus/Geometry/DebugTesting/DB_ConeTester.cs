using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using digitalopus.core;
using digitalopus.tranform;
using digitalopus.geometry;

public class DB_ConeTester : MonoBehaviour
{
    public float height = 2f;
    public float radius = 1f;

    public Transform endCone;

    public Transform testPntStart;
    public Transform testPntEnd;

    public bool db_isStartInsideA;
    public bool db_isStartInsideB;


    /*
    public int db_numHits;
    public bool db_startInsideCone;
    public Vector3 db_hitPnt0;
    public Vector3 db_hitPnt1;

    public Vector3 db_sph0;
    public Vector3 db_sph1;

    public float db_t0;
    public float db_t1;
    */
    public ConeGeometryUtils.ConeHitResult coneHit;

    int frmCnt;
    Vector3 center;
    Vector3 r0;
    Vector3 r1;
    Quaternion tmp_v1;
    Quaternion tmp_v2;
    Quaternion tmp_v3;
    float dr0;
    float dr1;


    public void Start()
    {

        frmCnt = 0;
    }

    public void FixedUpdate()
    {
        frmCnt++;
        center = transform.position + transform.forward * height * .5f;
        if (frmCnt % 100 == 0)
        {
            r0 = Random.onUnitSphere;
            r1 = -r0;
            //testPntStart.position = center + Random.onUnitSphere * Random.Range(1f, 4f);
            //testPntEnd.position = center + (center - testPntStart.position) * 2f;
            tmp_v1 = Quaternion.AngleAxis(2f, Random.onUnitSphere);
            tmp_v2 = Quaternion.AngleAxis(2f, Random.onUnitSphere);
            tmp_v3 = Quaternion.AngleAxis(2f, Random.onUnitSphere);
            dr0 = Random.Range(-.003f, .003f);
            dr1 = Random.Range(-.003f, .003f);
        }

        r0 = r0 + dr0 * r1.normalized;
        r1 = r1 + dr1 * r1.normalized;
        r0 = tmp_v1 * r0;
        r1 = -r0.normalized; // tmp_v2 * r1;
        testPntStart.position = center + r0;
        testPntEnd.position = center + r1;
        transform.rotation = tmp_v3 * transform.rotation;
    }

    //ClimbStepPath stepPathHelper;

    [Range(0,5)]
    public float distAlongPath;

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.cyan;
        GizmoUtils.DrawConeRoundBottom(transform.position, transform.forward, height, radius);

        // GizmoUtils.DrawConeRoundBottom(endCone.position, endCone.forward, height, radius);

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(testPntStart.position, .05f);
        Gizmos.DrawSphere(testPntEnd.position, .02f);
        Gizmos.color = DebugColors.redTr;
        Gizmos.DrawLine(testPntStart.position, testPntEnd.position);

        

        float coneAngle_deg = Mathf.Asin(radius / height) * Mathf.Rad2Deg;
        Transform rootPivot = transform;

        db_isStartInsideA = ConeGeometryUtils.IsPointInsideCone(testPntStart.position, transform.position, transform.rotation, coneAngle_deg);
        db_isStartInsideB = ConeGeometryUtils.IsPointInsideCone(testPntStart.position, transform.position, transform.forward, coneAngle_deg * Mathf.Deg2Rad);

        Vector3 snapStartToCone = ConeGeometryUtils.SnapToCone(testPntStart.position, transform.position, transform.forward, coneAngle_deg * Mathf.Deg2Rad);

        Gizmos.color = DebugColors.limeGreen;
        Gizmos.DrawSphere(snapStartToCone, .03f);
        {
            /*
            if (stepPathHelper == null) stepPathHelper = new ClimbStepPath(name);
            Vector3 p;
            Quaternion r;
            Vector3 vv = stepPathHelper.AdvanceAlongPathV3(testPntStart.position, testPntEnd.position, 
                        rootPivot.position, rootPivot.rotation,
                        endCone.position, endCone.rotation, coneAngle_deg, distAlongPath, out p, out r);
            stepPathHelper.DoDrawGizmos(rootPivot.position, rootPivot.rotation, height, radius);

            Gizmos.color = Color.green;
            Gizmos.DrawSphere(vv, .08f);
            */

            /*
            {
                float distPre = Vector3.Distance(stepPathHelper.db_coneSnapPnt_wld, testPntStart.position);
                Vector3 s_cone = TransformUtils.a_InverseTransformPoint_b(stepPathHelper.db_rootPivotPosSnapPnt, stepPathHelper.db_rootPivotRotSnapPnt, stepPathHelper.db_coneSnapPnt_wld);
                Vector3 e_cone = TransformUtils.a_InverseTransformPoint_b(endCone.position, endCone.rotation, testPntEnd.position);
                float numSeg = 5;
                Vector3 pPrev_wld = Vector3.zero;
                Gizmos.color = Color.gray;
                float dist = 0;
                for (int i = 0; i < numSeg; i++)
                {
                    float t = ((float)i) / (numSeg - 1f);
                    Vector3 rp = Vector3.Lerp(stepPathHelper.db_rootPivotPosSnapPnt, endCone.position, t);
                    Quaternion rr = Quaternion.Lerp(stepPathHelper.db_rootPivotRotSnapPnt, endCone.rotation, t);
                    Vector3 p_cone = Vector3.Lerp(s_cone, e_cone, t);
                    p_cone = p_cone.normalized * Mathf.Lerp(s_cone.magnitude, e_cone.magnitude, t);
                    Vector3 p_wld = TransformUtils.b_TransformPoint_a(rp, rr, p_cone);
                    if (i > 0)
                    {
                        Gizmos.color = Color.gray;
                        Gizmos.DrawLine(p_wld, pPrev_wld);
                        Gizmos.DrawSphere(p_wld, .02f);
                        Gizmos.color = DebugColors.greyTr;
                        Gizmos.DrawLine(rp, p_wld);
                        float d = Vector3.Distance(p_wld, pPrev_wld);
                        if (distPre + dist <= distAlongPath && distPre + dist + d >= distAlongPath)
                        {
                            Vector3 pp = Vector3.Lerp(pPrev_wld, p_wld, (distAlongPath - (distPre + dist)) / d);
                            Gizmos.color = Color.yellow;
                            Gizmos.DrawSphere(pp, .03f);
                        }

                        dist += d;
                    }
                    else
                    {
                        Gizmos.color = Color.gray;
                        Gizmos.DrawSphere(p_wld, .04f);
                    }
                    pPrev_wld = p_wld;
                }
            }
            */




            /*
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawWireSphere(segStart, .06f);
                Gizmos.color = Color.blue;
                Gizmos.DrawWireSphere(segEnd, .06f);
            }

            Gizmos.color = DebugColors.orange;
            if (db_mustPassThroughCone && db_footMoverDir_doArc)
            {
                Gizmos.DrawRay(rootPivot.position, db_footMoveDir_pivot2Current);
                Gizmos.DrawRay(rootPivot.position, db_footMoveDir_pivot2End);
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(rootPivot.position, db_footMoveDir_axis);
                Gizmos.color = DebugColors.orangeTr;
                Vector3 pivot2current = (segStart - transform.position);
                GizmoUtils.DrawWireArc(rootPivot.position, -db_footMoveDir_axis, pivot2current.magnitude, pivot2current, db_footMoveDir_angRad * Mathf.Rad2Deg);
                Gizmos.DrawLine(testPntStart.position, segStart);
                Gizmos.DrawLine(testPntEnd.position, segEnd);
            }
            else
            {
                Gizmos.color = DebugColors.orangeTr;
                Gizmos.DrawLine(testPntStart.position, segStart);
                Gizmos.DrawLine(segStart, segEnd);
                Gizmos.DrawLine(testPntEnd.position, segEnd);
            }

            if (!db_mustPassThroughCone)
            {
                Gizmos.color = DebugColors.purple;
                Gizmos.DrawSphere(db_midPntOnCone, .04f);
            }
            */
        }


        /*
        {
            Vector3 rayOrigin = testPntStart.position;
            Vector3 rayDir = (testPntEnd.position - testPntStart.position).normalized;
            float tHit0, tHit1;
            //Vector3 pHit0, pHit1;
            ConeGeometryUtils.IntersectConeVsLineSegmentSphereCapBase(testPntStart.position, testPntEnd.position, transform.position, transform.rotation, radius, height, out db_startInsideCone, out tHit0, out tHit1, out db_numHits, out db_t0, out db_t1);
            Vector3 dir = testPntEnd.position - testPntStart.position;
            db_hitPnt0 = testPntStart.position + dir * db_t0;
            db_hitPnt1 = testPntStart.position + dir * db_t1;

            Gizmos.color = DebugColors.redTr;
            Gizmos.DrawLine(testPntStart.position, testPntEnd.position);
            Gizmos.color = Color.red;
            Gizmos.DrawLine(db_hitPnt0, db_hitPnt1);

            if (db_startInsideCone)
            {
                Gizmos.color = DebugColors.orange;
            }
            else
            {
                Gizmos.color = Color.red;
            }

            Gizmos.DrawSphere(testPntStart.position, .05f);
            Gizmos.DrawSphere(testPntEnd.position, .02f);

            Gizmos.color = Color.cyan * .75f;
            if (db_numHits > 0 && db_t0 > 0f) Gizmos.DrawSphere(db_hitPnt0, .08f);
            Gizmos.color = Color.cyan;
            if (db_numHits > 0 && db_t1 < 1f) Gizmos.DrawSphere(db_hitPnt1, .08f);
        }
        */

        
        {
            Vector3 rayOrigin = testPntStart.position;
            Vector3 rayDir = (testPntEnd.position - testPntStart.position).normalized;
            ConeGeometryUtils.IntersectConeOnly(testPntStart.position, testPntEnd.position, transform.position, transform.rotation, radius, height, out coneHit);
            Vector3 dir = testPntEnd.position - testPntStart.position;
            Vector3 db_hitPnt0 = testPntStart.position + dir * coneHit.tClipped0;
            Vector3 db_hitPnt1 = testPntStart.position + dir * coneHit.tClipped1;

            Gizmos.color = DebugColors.redTr;
            Gizmos.DrawLine(testPntStart.position, testPntEnd.position);
            Gizmos.color = Color.red;
            Gizmos.DrawLine(db_hitPnt0, db_hitPnt1);

            if (coneHit.startIsInsideCone)
            {
                Gizmos.color = DebugColors.orange;
            }
            else
            {
                Gizmos.color = Color.red;
            }

            Gizmos.DrawSphere(testPntStart.position, .05f);
            Gizmos.DrawSphere(testPntEnd.position, .02f);

            Gizmos.color = Color.cyan * .75f;
            if (coneHit.numClipped > 0 && coneHit.tClipped0 > 0f) Gizmos.DrawSphere(db_hitPnt0, .08f);
            Gizmos.color = Color.cyan;
            if (coneHit.numClipped > 0 && coneHit.tClipped1 < 1f) Gizmos.DrawSphere(db_hitPnt1, .08f);
        }
        

    }
}
