using UnityEngine;
using System.Collections;
using digitalopus.geometry;
using digitalopus.util;
using System.Text;


//  Based on paper:  Jei Tam, Stable Proportional-Derivative Controllers
//
//  p1, v0, pTarg0, vTarg0  are postions and velocity this time step.
//  p1, v1, pTart1, vTarg1  are positions and velocity next time step. We will approximate these.
//
//  (1) acc = -kp(p1 - pTarg1) - kd(v1 - vTarg1)  // this looks like normal PD but we are using next-time-step quantities.
//
//  To approximate next time step do this:
// 
//	  (2) p1 = p0 + dt * v0
//	  (3) v1 = v0 + dt * a0
//	  (4) a0 * m = f
//
//  Each of these calculators compute estimates for accelerations and/or forces.
//
//  The accelerations should be used if trying to estimate.
//
//  The forces can be torques can be applied to a rigidbody and should match the computed accelerations.
//
namespace digitalopus.physics.kinematics
{
    /// <summary>
    /// This version is 1 dimensional.
    /// </summary>
    public struct SPDFloatCalculatorV2
    {
        [SerializeField]
        public float kp;

        [SerializeField]
        public float mass;

        [SerializeField]
        public float dt;

        [SerializeField]
        public float kd;

        public float ComputSPDAccelerationOnly(float p0, float v0,
                                    float pTarg0, float vTarg0)
        {
            float pTarg1 = pTarg0 + dt * vTarg0;
            float vTarg1 = vTarg0;
            float p1 = p0 + dt * v0;
            float a = -kp * (p1 - pTarg1) - kd * (v0 - vTarg1);
            return a;
        }

        public float ComputSPDForce(float p0, float v0,
                                    float pTarg0, float vTarg0)
        {
            float a = ComputSPDAccelerationOnly(p0, v0, pTarg0, vTarg0);
            float f = mass * a;
            return f;
        }
    }

    /// <summary>
    /// This version is 3 dimensional.
    /// </summary>
    public struct SPDVector3CalculatorV2
    {
        [SerializeField]
        public float kp;

        [SerializeField]
        public float mass;

        [SerializeField]
        public float dt;

        [SerializeField]
        public float kd;

        public Vector3 ComputSPDAccelerationOnly(Vector3 p0_wld, Vector3 v0_wld,
                                    Vector3 pTarg0_wld, Vector3 vTarg0_wld)
        {
            Vector3 pTarg1_wld = pTarg0_wld + dt * vTarg0_wld;
            Vector3 vTarg1_wld = vTarg0_wld;
            Vector3 p1_wld = p0_wld + dt * v0_wld;
            // Idealy we would want to use v1_wld instead of v0_wld. But we would need to know the acceleration
            // for that. Assume v1_wld == v0_wld.
            Vector3 a_wld = -kp * (p1_wld - pTarg1_wld) - kd * (v0_wld - vTarg1_wld);
            return a_wld;
        }

        public Vector3 ComputSPDForce(Vector3 p0_wld, Vector3 v0_wld,
                                    Vector3 pTarg0_wld, Vector3 vTarg0_wld)
        {
            Vector3 a_wld = ComputSPDAccelerationOnly(p0_wld, v0_wld, pTarg0_wld, vTarg0_wld);
            Vector3 f_wld = mass * a_wld;
            return f_wld;
        }
    }

    /// <summary>
    /// This version is 1 dimensional angular.
    /// 
    /// Use it for 1-D rotations like hinge joints.
    /// </summary>
    public struct SPDRotation1DCalculatorV2
    {
        [SerializeField]
        public float kp;

        [SerializeField]
        public float momentOfInertia;

        [SerializeField]
        public float dt;

        [SerializeField]
        public float kd;

        public static float AngleDifferenceAminusB_rad(float a_rad, float b_rad)
        {
            float diff_rad = (b_rad - a_rad + Mathf.PI) % (Mathf.PI * 2f) - Mathf.PI;
            return diff_rad < -Mathf.PI ? -(diff_rad + (Mathf.PI * 2f)) : -diff_rad;
        }

        public float ComputSPDAccelerationOnly(float ang0_rad, float v0_radPerSec,
                                    float angTarg0_rad, float vTarg0_radPerSec)
        {
            float angTarg1_rad = angTarg0_rad + dt * vTarg0_radPerSec;
            float vTarg1_rad = vTarg0_radPerSec;
            float ang1_rad = ang0_rad + dt * v0_radPerSec;

            float deltaAng_rad = ang1_rad - angTarg1_rad;
            if (Mathf.Abs(deltaAng_rad) > Mathf.PI)
            {
                // The difference between current angle and target angle is greater than 180
                // There is a shorter route to the target.
                deltaAng_rad = AngleDifferenceAminusB_rad(ang1_rad, angTarg1_rad);
            }

            float angAcc_radPerSecSec = -kp * (deltaAng_rad) - kd * (v0_radPerSec - vTarg1_rad);
            return angAcc_radPerSecSec;
        }

        public float ComputSPDForce(float p0, float v0,
                                    float pTarg0, float vTarg0)
        {
            float a = ComputSPDAccelerationOnly(p0, v0, pTarg0, vTarg0);
            float f = momentOfInertia * a;
            return f;
        }
    }

    public struct SPDQuaternionCalculatorV2
    {
        /// <summary>
        /// ------------------------
        /// 
        /// Inertia Tensor stuff
        /// One complication with the Quaternion version of SPD is the inertia tensor.
        /// 
        ///     f = M * a
        /// 
        /// f is torque, a is angular acceleration.
        /// M is the inertia tensor and it is not a simple scalar like in the float and Vector3 version of SPD
        /// It is a matrix. However there exists a coordinate system in which the inertia tensor is a diagonal matrix.
        /// This coordinate system is called the principal axes coordinate system.
        ///     obj_inertiaTensorRotation_dyn
        /// rotates from inertia space to object local space. The axes of inertia-dynamics-space are called the principal axes coordinate system.
        ///     inertiaTensor
        /// is the diagonal entries of the matrix in the principal axes coordinate system. Basically the axes are scaled by these numbers.
        /// 
        /// Think of the inertia tensor as a coordinate transformation matrix:
        ///     - it transforms object-local space to principal-axis-space, com-is-pivot, scaled-by-inertiaTensor 
        ///     - the centerOfMass location is the pivot
        ///     - the body_intertiaTensorRotation_paxis is the rotation of the coordinate system where the matrix is diagonal
        ///     - the inertiaTensor is the scaling
        /// 
        /// 
        /// Note that because this calculator only deals with angular quantities we don't need to consider the
        /// pivot when converting between inertia space and object local space.
        /// 
        /// Now consider:
        ///     kinematics  <=>          dynamics
        ///         a  *  M    =           f
        /// 
        /// Kinematics quantities:
        ///    postition, velocity, angularVelocity, acceleration
        ///    
        /// Dynamics qunatities:
        ///    force, torque, momentum, angularMomentum
        ///    
        /// The inertia matrix converts quantites to and from objLocalSpace <=> inertiaDynamicsSpace
        ///    
        /// The SPD controller computes an accelereration in kinematic space. Then we need to use the intertiaTensor to 
        /// convert these quantities into dynamics quantities.
        /// </summary>
        [SerializeField]
        public Vector3 inertiaTensor;   // rigidbody.inertiaTensor

        [SerializeField]
        public Quaternion body_intertiaTensorRotation_dyn;    // rigidbody.intertiaTensorRotation

        // ----------------------
        [SerializeField]
        public float dt;

        [SerializeField]
        public float kp;

        [SerializeField]
        public float kd;

        /// <summary>
        /// V0 and vTarg0 are angular velocities. 
        /// They obey the left hand thumb rule in Unity.
        /// 
        /// </summary>
        public Vector3 ComputSPDAccelerationOnly(Quaternion wld_Rot_p0, Vector3 v0_wld,
                                    Quaternion wld_RotTarg_p0, Vector3 vTarg0_wld)
        {
            Quaternion wld_RotTarg_p1;  // =  pTarg0 + dt * vTarg0   expected body rotation one frame in the future.
            {
                float omegaTarg_rad = vTarg0_wld.magnitude;
                if (omegaTarg_rad < 10e-8f)
                {
                    wld_RotTarg_p1 = wld_RotTarg_p0;
                }
                else
                {
                    Quaternion wld_delta_wld = Quaternion.AngleAxis(omegaTarg_rad * dt * Mathf.Rad2Deg,
                                                            vTarg0_wld / omegaTarg_rad);
                    wld_RotTarg_p1 = wld_delta_wld * wld_RotTarg_p0; 
                }
            }

            Quaternion wld_Rot_p1; // = p0 + _dt * v0;   expected target rotation one frame in the future.
            {
                float omega = v0_wld.magnitude;
                if (omega < 10e-8f)
                {
                    wld_Rot_p1 = wld_Rot_p0;
                }
                else
                {
                    Quaternion wld_delta_wld = Quaternion.AngleAxis(omega * dt * Mathf.Rad2Deg,
                                                            v0_wld / omega);
                    wld_Rot_p1 = wld_delta_wld * wld_Rot_p0; 
                }
            }

            float p_deltaMag; // p1 - pTarg1    difference between current angle and target angle
            Vector3 p_deltaDir_wld;
            {
                Quaternion wld_deltaP_wld = wld_Rot_p1 * Quaternion.Inverse(wld_RotTarg_p1);

                if (wld_deltaP_wld.w < 0f)
                {
                    // The rotation is greater than 180 degrees (long way around the sphere). Convert to shortest path.
                    wld_deltaP_wld.x = -wld_deltaP_wld.x;
                    wld_deltaP_wld.y = -wld_deltaP_wld.y;
                    wld_deltaP_wld.z = -wld_deltaP_wld.z;
                    wld_deltaP_wld.w = -wld_deltaP_wld.w;
                }

                wld_deltaP_wld.ToAngleAxis(out p_deltaMag, out p_deltaDir_wld);
                p_deltaDir_wld.Normalize();
                p_deltaMag *= Mathf.Deg2Rad;
            }

            Vector3 a = -kp * p_deltaDir_wld * p_deltaMag - kd * (v0_wld - vTarg0_wld);
            return a;
        }

        public Vector3 ComputSPDTorque(Quaternion wld_Rot_p0, Vector3 v0_wld,
                                    Quaternion wld_RotTarg_p0, Vector3 vTarg0_wld
                                    )
        {
            // acc_wld is the kinematic angular acceleration in wld frame. Calculated using SPD
            Vector3 acc_wld = ComputSPDAccelerationOnly(wld_Rot_p0, v0_wld,
                                                        wld_RotTarg_p0, vTarg0_wld);

            // Calc transform from world frame to objects principal axis frame
            Quaternion  wld_Rot_paxis = wld_Rot_p0 * body_intertiaTensorRotation_dyn;
            
            // We need to convert the acceleration to torque in the principal axis frame.
            // In this frame the inertia matrix is diagonal. The inertiaTensor vector is the
            // non-zero diagonal entries of this matrix.
            Vector3 acc_paxis = Quaternion.Inverse(wld_Rot_paxis) * acc_wld;

            // Now that we are in dynamics principal axis frame we can compute torque using:
            //      torque = intertiaTensor * angularAcceleration
            Vector3 torque_dyn;
            torque_dyn.x = inertiaTensor.x * acc_paxis.x;
            torque_dyn.y = inertiaTensor.y * acc_paxis.y;
            torque_dyn.z = inertiaTensor.z * acc_paxis.z;

            // Now transform the torque from principal axis frame back to world frame.
            Vector3 torque_wld = wld_Rot_paxis * torque_dyn;

            return torque_wld;
        }
    }
}
