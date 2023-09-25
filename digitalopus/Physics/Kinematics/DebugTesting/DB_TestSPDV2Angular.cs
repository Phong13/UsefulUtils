using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions.Must;

namespace digitalopus.physics.kinematics
{
    public class DB_TestSPDV2Angular : MonoBehaviour
    {
        public float dt;
        public float kp;
        public float kd;

        public Vector3 objToMoveAngVel;
        public Rigidbody rbToMove;
        public Transform objToMove;
        public Transform target;
        public Transform rbTarget;

        SPDQuaternionCalculatorV2 spdCalculator;

        List<Vector3> db_angAccel = new List<Vector3>(20);
        List<Vector3> db_torques = new List<Vector3>(20);

        float t;

        [ContextMenu("Do Step")]
        void DoStep()
        {
            Vector3 targetV;
            {
                // update the target
                t += Time.fixedDeltaTime;
                float omega = Mathf.Sin(t * Mathf.Deg2Rad * 180f);
                Vector3 axis = new Vector3(1f, 0f, 0f);
                targetV = axis * omega;
                Quaternion targetDeltaP = Quaternion.AngleAxis(omega * Mathf.Rad2Deg * dt, axis);
                target.rotation = targetDeltaP * target.rotation;
            }

            spdCalculator.inertiaTensor = rbToMove.inertiaTensor;
            spdCalculator.body_intertiaTensorRotation_dyn = rbToMove.inertiaTensorRotation;
            spdCalculator.dt = dt;
            spdCalculator.kp = kp;
            spdCalculator.kd = kd;

            Vector3 a;
            {
                // Kinematic rotation
                a = spdCalculator.ComputSPDAccelerationOnly(objToMove.rotation, objToMoveAngVel,
                                                        target.rotation, targetV);
                

                    db_angAccel.Add(a);
                
                
                objToMoveAngVel += a * spdCalculator.dt;

                Quaternion pTarg1;  // =  pTarg0 + dt * vTarg0
                {
                    float omegaTarg = objToMoveAngVel.magnitude;
                    if (omegaTarg < 10e-8f)
                    {
                        pTarg1 = objToMove.rotation;
                    }
                    else
                    {
                        Quaternion delta = Quaternion.AngleAxis(omegaTarg * dt * Mathf.Rad2Deg,
                                                                objToMoveAngVel / omegaTarg);
                        pTarg1 = delta * objToMove.rotation;
                    }
                }

                objToMove.rotation = pTarg1;
            }

            {
                
                {
                    Vector3 torque2;
                    torque2 = spdCalculator.ComputSPDTorque(rbToMove.rotation, rbToMove.angularVelocity,
                                                            target.rotation, targetV);

                        db_torques.Add(torque2);
 
                    rbToMove.AddTorque(torque2);
                }
                
                /*
                {
                    Vector3 torque2;
                    torque2 = spdCalculator.ComputSPDAccelerationOnly(rbToMove.rotation, rbToMove.angularVelocity,
                                                            target.rotation, Vector3.zero);

                    db_torques.PushBack(torque2);

                    rbToMove.AddTorque(torque2 * 100f);
                }
                */
            }
        }

        private void FixedUpdate()
        {
            if (Time.frameCount > 20)
            {
                DoStep();
            }

            if (Time.frameCount == 20)
            {
                Debug.Break();
                Debug.LogError("BREAK ");
            }
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.magenta;
            for (int i = 0; i < db_angAccel.Count; i++)
            {
                float t = ((float)i) / db_angAccel.Count;
                Gizmos.color = Color.Lerp(Color.gray, Color.magenta, t);
                Vector3 p = objToMove.position;
                p.x += i * .04f; 
                Gizmos.DrawRay(p, db_angAccel[i]);
            }

            Gizmos.color = Color.cyan;
            for (int i = 0; i < db_torques.Count; i++)
            {
                float t = ((float)i) / db_angAccel.Count;
                Gizmos.color = Color.Lerp(Color.gray, Color.cyan, t);
                Vector3 p = rbToMove.position;
                p.x += i * .04f;
                Gizmos.DrawRay(p, db_torques[i]);
            }
        }
    }
}
