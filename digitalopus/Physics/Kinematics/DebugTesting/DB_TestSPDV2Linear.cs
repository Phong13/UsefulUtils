using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace digitalopus.physics.kinematics
{
    public class DB_TestSPDV2Linear : MonoBehaviour
    {
        public float mass;
        public float dt;
        public float kp;
        public float kd;

        public Vector3 objToMoveVel;
        public Rigidbody rbToMove;
        public Transform objToMove;
        public Transform target;
        public Transform rbTarget;

        SPDVector3CalculatorV2 spdCalculator;

        private float t;

        // Start is called before the first frame update
        void Start()
        {
            spdCalculator.mass = mass;
            spdCalculator.dt = dt;
            spdCalculator.kp = kp;
            spdCalculator.kd = kd;
            t = 0f;
        }

        [ContextMenu("Do Step")]
        void DoStep()
        {
            Vector3 targetV;
            {
                // update the target
                t += Time.fixedDeltaTime;
                targetV = new Vector3(Mathf.Sin(t * Mathf.Deg2Rad * 180f) * 25f, 0f, 0f);
                Vector3 targetDeltaP = targetV * dt;
                target.position += targetDeltaP;
                rbTarget.position += targetDeltaP;
            }

            spdCalculator.mass = mass;
            spdCalculator.dt = dt;
            spdCalculator.kp = kp;
            spdCalculator.kd = kd;
            Vector3 a = spdCalculator.ComputSPDAccelerationOnly(objToMove.position, objToMoveVel,
                                                    target.position, targetV);
            objToMoveVel += a * spdCalculator.dt;
            Vector3 p = objToMove.position;
            p = p + objToMoveVel * dt;
            objToMove.position = p;

            //--------------------
            Vector3 f = spdCalculator.ComputSPDForce(rbToMove.position, rbToMove.velocity,
                                                    rbTarget.position, targetV);
            rbToMove.AddForce(f);
        }

        [ContextMenu("Test Ang Diff")]
        public void TestAngDifference()
        {
            for (int i = 0; i < 30; i++)
            {
                float a = 180f * Mathf.Deg2Rad;
                float b = (-500f + i * 30f) * Mathf.Deg2Rad;
                float dif = SPDRotation1DCalculatorV2.AngleDifferenceAminusB_rad(a, b);
                Debug.Log("  a " + a * Mathf.Rad2Deg + "  b " + b * Mathf.Rad2Deg + "   dif (a - b):" + dif * Mathf.Rad2Deg);
            }
        }

        private void FixedUpdate()
        {
            if (Time.frameCount > 20)
            {
                DoStep();
            }
        }
    }
}
