using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using digitalopus.tranform;

public class DB_TestTransformUtils : MonoBehaviour
{
    public Transform[] animRig;

    public Rigidbody tmpA;
    public Rigidbody tmpB;

    public Vector3 tmpXX, tmpYY, tmpZZ, tmpPos;

    public Transform tmpTA;
    public Transform tmpTB;

    public Transform cube;
    public Transform capsule;

    [ContextMenu("Test Transform")]
    public void TestTransform()
    {
        TransformUtils.GTransform capsLocal = TransformUtils.GTransform.CreateTransform(capsule.localRotation, capsule.localPosition);
        TransformUtils.GTransform invCapsLocal = capsLocal.Inverse();
        Debug.Log(capsLocal.b_rot_a.eulerAngles + "   inv:  " + invCapsLocal.b_rot_a.eulerAngles);
    }

    [ContextMenu("Test Inverse")]
    public void TestInverse()
    {
        for (int i = 0; i < 1000; i++)
        {
            TransformUtils.GTransform a = TransformUtils.GTransform.CreateTransform(Random.rotation, Random.onUnitSphere * Random.Range(0, 100));
            TransformUtils.GTransform aInv = a.Inverse();
            TransformUtils.GTransform aInvInv = aInv.Inverse();
            Debug.Assert(a.EqualsApproximate(aInvInv), a + "   " + aInvInv);
        }
    }


    [ContextMenu("Test transform utils")]
    public void TestTransformUtils()
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

    [ContextMenu("Test Trans Utils")]
    public void TestTransUtils()
    {
        TransformUtils.GTransform a = TransformUtils.GTransform.CreateTransform(animRig[0].rotation, animRig[0].position);
        TransformUtils.GTransform b = TransformUtils.GTransform.CreateTransform(animRig[1].localRotation, animRig[1].localPosition);
        TransformUtils.GTransform c = TransformUtils.GTransform.Multiply(a, b);
        TransformUtils.GTransform cInv = TransformUtils.GTransform.Inverse(c);

        // c should take stuff from b local to world
        Vector3 d = Random.onUnitSphere;
        Vector3 p = Random.onUnitSphere * Random.Range(3, 15);
        Debug.Assert(animRig[1].TransformPoint(p) == c.b_TransformPoint_a(p), animRig[1].TransformPoint(p).ToString("f4") + "  " + c.b_TransformPoint_a(p).ToString("f4"));
        Debug.Assert(animRig[1].TransformDirection(d) == c.b_TransformDirection_a(d));
        Debug.Assert(animRig[1].InverseTransformPoint(p) == c.a_InverseTransformPoint_b(p));
        Debug.Assert(animRig[1].InverseTransformDirection(d) == c.a_InverseTransformDirection_b(d));

        // cInv should take stuff from world to b local
        Debug.Assert(animRig[1].InverseTransformPoint(p) == cInv.b_TransformPoint_a(p));
        Debug.Assert(animRig[1].InverseTransformDirection(d) == cInv.b_TransformDirection_a(d));
    }
}
