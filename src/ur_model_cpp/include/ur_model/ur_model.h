/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef UR_MODEL_UR_MODEL
#define UR_MODEL_UR_MODEL

#include <ur_model/model_base.h>
#include <control_core/types.h>
#include <tf/transform_broadcaster.h>

namespace ur
{
  /**
   * @brief URModel Class 
   * 
   * Implementation of Universal Robot kinematic and dynamic model
   * 
   * @note the robot base frame (first frame is denoted as '0')
   * the world frame is denoted as 'b'
   *
   */
  class URModel : public model_interface::ModelBase
  {
  public:
    /* types */

    typedef model_interface::ModelBase Base;
    typedef cc::MatrixX Regressor;
    typedef cc::VectorX Parameters;
    typedef cc::MatrixX IKSolutions;

  private:
    /* members */

    // dynamic model
    cc::MatrixDof M_;
    cc::MatrixDof C_;
    cc::VectorDof g_;

    // robot regressor
    Parameters theta_;
    Regressor Yr_;

    /* parameters */

    // link length
    cc::Scalar L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12;

    // link mass
    cc::Scalar m1, m2, m3, m4, m5, m6;

    // inertia tensors
    cc::Scalar I111, I112, I113, I122, I123, I133;
    cc::Scalar I211, I212, I213, I222, I223, I233;
    cc::Scalar I311, I312, I313, I322, I323, I333;
    cc::Scalar I411, I412, I413, I422, I423, I433;
    cc::Scalar I511, I512, I513, I522, I523, I533;
    cc::Scalar I611, I612, I613, I622, I623, I633;

    // gravity components
    cc::Scalar gx, gy, gz;

    // joint limits
    cc::Scalar lo_jl1, lo_jl2, lo_jl3, lo_jl4, lo_jl5, lo_jl6;
    cc::Scalar hi_jl1, hi_jl2, hi_jl3, hi_jl4, hi_jl5, hi_jl6;

    // gravity vectors
    cc::LinearPosition g_b_, g_0_;

    // base to world transformation
    cc::HomogeneousTransformation T_0_b_;
    cc::HomogeneousTransformation T_b_0_;

    // tool to ef transformation
    cc::HomogeneousTransformation T_tool_ef_;

    // frames
    std::string robot_0_frame_; // 0 frame tf name
    std::string base_frame_;    // base frame tf name
    std::string tool_frame_;    // tool frame tf name

    // broadcast frames
    tf::TransformBroadcaster br_;
    std::vector<tf::StampedTransform> tf_stamped_transform_; // stack of tf transformations

  public:
    /**
     * @brief Construct a new URModel object
     * 
     * @param name name space for the ros parameter service
     */
    URModel(const std::string &name = "ur_model");

    /**
     * @brief Destroy the object
     */
    virtual ~URModel();

    /** 
    * @brief inertia matrix computation
    *
    * @param inertia Matrix
    * @param joint position
    */
    const cc::MatrixDof &inertiaMatrix(const cc::JointPosition &q);

    /** 
    * @brief coriolis centripetal matrix computation
    *
    * @param coriolis centripetal matrix
    * @param joint position
    * @param joint velocity
    */
    const cc::MatrixDof &centripetalMatrix(const cc::JointPosition &q, const cc::JointVelocity &qP);

    /** 
    * @brief gravitational vector computation
    *
    * @param gravitational vector
    * @param joint position
    */
    const cc::VectorDof &gravityVector(const cc::JointPosition &q);

    /** 
    * @brief regressor matrix computation
    *
    * @param regressor matrix
    * @param joint position
    * @param joint velocity
    * @param reference joint velocity
    * @param reference joint acceleration
    */
    const Regressor &regressor(const cc::JointPosition &q, const cc::JointVelocity &qP, const cc::JointVelocity &qrP, const cc::JointAcceleration &qrPP);

    /**
     * @brief inital guess based on loaded parameters
     * Call this to initalize parameter vector theta
     * 
     * @return const Parameters& 
     */
    const Parameters &parameterInitalGuess();

    /** 
    * @brief robot base wrt fixed base
    */
    cc::HomogeneousTransformation T_0_B() const;

    /** 
    * @brief fixed base wrt to robot base
    */
    cc::HomogeneousTransformation T_B_0() const;

    /**
     * @brief Tool wrt ef
     */
    cc::HomogeneousTransformation T_Tool_Ef() const;

    /** 
    * @brief gravity vector wrt word
    */
    cc::LinearPosition g_B() const;

    /** 
    * @brief gravity vector wrt base
    */
    cc::LinearPosition g_0() const;

    /** 
    * @brief endeffector transformation matrix
    */
    cc::HomogeneousTransformation T_ef_0(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector transformation matrix wrt to robot base frame
    */
    cc::HomogeneousTransformation T_ef_B(const cc::JointPosition &q) const;

    /** 
    * @brief tool transformation matrix
    */
    cc::HomogeneousTransformation T_tool_0(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector transformation matrix wrt to robot base frame
    */
    cc::HomogeneousTransformation T_tool_B(const cc::JointPosition &q) const;

    /** 
    * @brief transformation matrix j-th joint wrt to robot base frame
    * 
    * Joint index j in range: 0-5
    */
    cc::HomogeneousTransformation T_j_0(const cc::JointPosition &q, int j) const;

    /** 
    * @brief transformation matrix j-th center of mass frame wrt to robot base frame
    * 
    * Joint index j in range: 0-5
    */
    cc::HomogeneousTransformation T_cm_j_0(const cc::JointPosition &q, int j) const;

    /** 
    * @brief endeffector jacobian matrix wrt to robot base frame
    */
    cc::Jacobian J_ef_0(const cc::JointPosition &q) const;

    /** 
    * @brief endeffector jacobian matrix wrt to robot base frame
    */
    cc::Jacobian J_tool_0(const cc::JointPosition &q) const;

    /** 
    * @brief jacobian matrix at j-th joint wrt to robot base frame
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian J_j_0(const cc::JointPosition &q, int j) const;

    /** 
    * @brief jacobian matrix at j-th joint offseted by length tj wrt 0 frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jt_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, int j) const;

    /** 
    * @brief jacobian matrix at j-th center of mass wrt to robot base frame
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jcm_j_0(const cc::JointPosition &q, int j) const;

    /** 
    * @brief jacobian matrix at j-th center of mass offseted by length tj wrt robot base frame
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jtcm_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, int j) const;

    /** 
    * @brief endeffector jacobian derivative wrt robot base frame 
    */
    cc::Jacobian Jp_ef_0(const cc::JointPosition &q, const cc::JointVelocity &qP) const;

    /** 
    * @brief tool jacobian derivative wrt robot base frame 
    */
    cc::Jacobian Jp_tool_0(const cc::JointPosition &q, const cc::JointVelocity &qP) const;

    /** 
    * @brief jacobian derivative at j-th joint wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jp_j_0(const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /** 
    * @brief jacobian derivative at j-th joint offseted by length tj wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jtp_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /** 
    * @brief jacobian derivative at j-th center of mass wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jpcm_j_0(const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /** 
    * @brief jacobian derivative at j-th center of mass offseted by length tj wrt robot base frame 
    * 
    * Joint index j in range: 0-5
    */
    cc::Jacobian Jtpcm_j_0(const cc::LinearPosition &tj, const cc::JointPosition &q, const cc::JointVelocity &qP, int j) const;

    /**
     * @brief return lower joint limits of j-th joint
     * 
     */
    cc::Scalar lowerJointLimits_j(int j) const;

    /**
     * @brief return upper joint limits of j-th joint
     * 
     */
    cc::Scalar upperJointLimits_j(int j) const;

    /** 
    * @brief inverse kinematics, return all posible 8 ik solutions
    * 
    * @note Simon: I think there is a bug here! Need to check ...
    */
    void inverseKinematics(IKSolutions &Qs, const cc::HomogeneousTransformation &Tef, cc::Scalar q6_d) const;

    /**
     * @brief Get base Frame name
     */
    std::string getBaseFrame() const;

    /**
     * @brief Get base Frame name
     */
    std::string get0Frame() const;

    /**
     * @brief Get the Tool Frame name
     */
    std::string getToolFrame() const;

    /**
     * @brief Get the DH Frame name
     */
    std::string getDHFrame_j(int j) const;

    /**
     * @brief broad cast frames in tf tree
     */
    void broadcastFrames(const cc::JointPosition &q, const ros::Time &time);

  protected:
    /**
     * @brief initalize the model
     */
    virtual bool init(ros::NodeHandle &nh);

  private:
    /* matlab generated functions */

    /* inertia matrix */
    void matrix_M(cc::MatrixDof &M,
                  const cc::JointPosition &q) const;

    /* coriolis centripetal matrix */
    void matrix_C(cc::MatrixDof &C,
                  const cc::JointPosition &q,
                  const cc::JointVelocity &qP) const;

    /* gravitational vector */
    void matrix_G(cc::VectorDof &G,
                  const cc::JointPosition &q) const;

    /* regressor matrix */
    void matrix_Y(Regressor &Y,
                  const cc::JointPosition &q,
                  const cc::JointVelocity &qP,
                  const cc::JointVelocity &qrP,
                  const cc::JointAcceleration &qrPP) const;

    /* parameter vector theta */
    void matrix_th(Parameters &th) const;

    /* inverse kinematics */
    void matrix_IK(IKSolutions &Qs, const cc::HomogeneousTransformation &Tef, cc::Scalar q6_d) const;

    /* transformations */
    void matrix_T1_0(cc::HomogeneousTransformation &T1_0,
                     const cc::JointPosition &q) const;
    void matrix_T2_0(cc::HomogeneousTransformation &T2_0,
                     const cc::JointPosition &q) const;
    void matrix_T3_0(cc::HomogeneousTransformation &T3_0,
                     const cc::JointPosition &q) const;
    void matrix_T4_0(cc::HomogeneousTransformation &T4_0,
                     const cc::JointPosition &q) const;
    void matrix_T5_0(cc::HomogeneousTransformation &T5_0,
                     const cc::JointPosition &q) const;
    void matrix_T6_0(cc::HomogeneousTransformation &T6_0,
                     const cc::JointPosition &q) const;

    void matrix_Tcm1_0(cc::HomogeneousTransformation &Tcm1_0,
                       const cc::JointPosition &q) const;
    void matrix_Tcm2_0(cc::HomogeneousTransformation &Tcm2_0,
                       const cc::JointPosition &q) const;
    void matrix_Tcm3_0(cc::HomogeneousTransformation &Tcm3_0,
                       const cc::JointPosition &q) const;
    void matrix_Tcm4_0(cc::HomogeneousTransformation &Tcm4_0,
                       const cc::JointPosition &q) const;
    void matrix_Tcm5_0(cc::HomogeneousTransformation &Tcm5_0,
                       const cc::JointPosition &q) const;
    void matrix_Tcm6_0(cc::HomogeneousTransformation &Tcm6_0,
                       const cc::JointPosition &q) const;

    /* jacobians */
    void matrix_J1_0(cc::Jacobian &J1_0,
                     const cc::JointPosition &q) const;
    void matrix_J2_0(cc::Jacobian &J2_0,
                     const cc::JointPosition &q) const;
    void matrix_J3_0(cc::Jacobian &J3_0,
                     const cc::JointPosition &q) const;
    void matrix_J4_0(cc::Jacobian &J4_0,
                     const cc::JointPosition &q) const;
    void matrix_J5_0(cc::Jacobian &J5_0,
                     const cc::JointPosition &q) const;
    void matrix_J6_0(cc::Jacobian &J6_0,
                     const cc::JointPosition &q) const;

    void matrix_Jcm1_0(cc::Jacobian &Jcm1_0,
                       const cc::JointPosition &q) const;
    void matrix_Jcm2_0(cc::Jacobian &Jcm2_0,
                       const cc::JointPosition &q) const;
    void matrix_Jcm3_0(cc::Jacobian &Jcm3_0,
                       const cc::JointPosition &q) const;
    void matrix_Jcm4_0(cc::Jacobian &Jcm4_0,
                       const cc::JointPosition &q) const;
    void matrix_Jcm5_0(cc::Jacobian &Jcm5_0,
                       const cc::JointPosition &q) const;
    void matrix_Jcm6_0(cc::Jacobian &Jcm6_0,
                       const cc::JointPosition &q) const;

    void matrix_Jt1_0(cc::Jacobian &Jt1_0,
                      const cc::LinearPosition &t,
                      const cc::JointPosition &q) const;
    void matrix_Jt2_0(cc::Jacobian &Jt2_0,
                      const cc::LinearPosition &t,
                      const cc::JointPosition &q) const;
    void matrix_Jt3_0(cc::Jacobian &Jt3_0,
                      const cc::LinearPosition &t,
                      const cc::JointPosition &q) const;
    void matrix_Jt4_0(cc::Jacobian &Jt4_0,
                      const cc::LinearPosition &t,
                      const cc::JointPosition &q) const;
    void matrix_Jt5_0(cc::Jacobian &Jt5_0,
                      const cc::LinearPosition &t,
                      const cc::JointPosition &q) const;
    void matrix_Jt6_0(cc::Jacobian &Jt6_0,
                      const cc::LinearPosition &t,
                      const cc::JointPosition &q) const;

    void matrix_Jtcm1_0(cc::Jacobian &Jtcm1_0,
                        const cc::LinearPosition &t,
                        const cc::JointPosition &q) const;
    void matrix_Jtcm2_0(cc::Jacobian &Jtcm2_0,
                        const cc::LinearPosition &t,
                        const cc::JointPosition &q) const;
    void matrix_Jtcm3_0(cc::Jacobian &Jtcm3_0,
                        const cc::LinearPosition &t,
                        const cc::JointPosition &q) const;
    void matrix_Jtcm4_0(cc::Jacobian &Jtcm4_0,
                        const cc::LinearPosition &t,
                        const cc::JointPosition &q) const;
    void matrix_Jtcm5_0(cc::Jacobian &Jtcm5_0,
                        const cc::LinearPosition &t,
                        const cc::JointPosition &q) const;
    void matrix_Jtcm6_0(cc::Jacobian &Jtcm6_0,
                        const cc::LinearPosition &t,
                        const cc::JointPosition &q) const;

    /* jacobian derivatives */
    void matrix_J1_0p(cc::Jacobian &J1_0p,
                      const cc::JointPosition &q,
                      const cc::JointVelocity &qP) const;
    void matrix_J2_0p(cc::Jacobian &J2_0p,
                      const cc::JointPosition &q,
                      const cc::JointVelocity &qP) const;
    void matrix_J3_0p(cc::Jacobian &J3_0p,
                      const cc::JointPosition &q,
                      const cc::JointVelocity &qP) const;
    void matrix_J4_0p(cc::Jacobian &J4_0p,
                      const cc::JointPosition &q,
                      const cc::JointVelocity &qP) const;
    void matrix_J5_0p(cc::Jacobian &J5_0p,
                      const cc::JointPosition &q,
                      const cc::JointVelocity &qP) const;
    void matrix_J6_0p(cc::Jacobian &J6_0p,
                      const cc::JointPosition &q,
                      const cc::JointVelocity &qP) const;

    void matrix_Jcm1_0p(cc::Jacobian &Jcm1_0p,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP) const;
    void matrix_Jcm2_0p(cc::Jacobian &Jcm2_0p,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP) const;
    void matrix_Jcm3_0p(cc::Jacobian &Jcm3_0p,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP) const;
    void matrix_Jcm4_0p(cc::Jacobian &Jcm4_0p,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP) const;
    void matrix_Jcm5_0p(cc::Jacobian &Jcm5_0p,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP) const;
    void matrix_Jcm6_0p(cc::Jacobian &Jcm6_0p,
                        const cc::JointPosition &q,
                        const cc::JointVelocity &qP) const;

    void matrix_Jt1_0p(cc::Jacobian &Jt1_0p,
                       const cc::LinearPosition &t,
                       const cc::JointPosition &q,
                       const cc::JointVelocity &qP) const;
    void matrix_Jt2_0p(cc::Jacobian &Jt2_0p,
                       const cc::LinearPosition &t,
                       const cc::JointPosition &q,
                       const cc::JointVelocity &qP) const;
    void matrix_Jt3_0p(cc::Jacobian &Jt3_0p,
                       const cc::LinearPosition &t,
                       const cc::JointPosition &q,
                       const cc::JointVelocity &qP) const;
    void matrix_Jt4_0p(cc::Jacobian &Jt4_0p,
                       const cc::LinearPosition &t,
                       const cc::JointPosition &q,
                       const cc::JointVelocity &qP) const;
    void matrix_Jt5_0p(cc::Jacobian &Jt5_0p,
                       const cc::LinearPosition &t,
                       const cc::JointPosition &q,
                       const cc::JointVelocity &qP) const;
    void matrix_Jt6_0p(cc::Jacobian &Jt6_0p,
                       const cc::LinearPosition &t,
                       const cc::JointPosition &q,
                       const cc::JointVelocity &qP) const;

    void matrix_Jtcm1_0p(cc::Jacobian &Jtcm1_0p,
                         const cc::LinearPosition &t,
                         const cc::JointPosition &q,
                         const cc::JointVelocity &qP) const;
    void matrix_Jtcm2_0p(cc::Jacobian &Jtcm2_0p,
                         const cc::LinearPosition &t,
                         const cc::JointPosition &q,
                         const cc::JointVelocity &qP) const;
    void matrix_Jtcm3_0p(cc::Jacobian &Jtcm3_0p,
                         const cc::LinearPosition &t,
                         const cc::JointPosition &q,
                         const cc::JointVelocity &qP) const;
    void matrix_Jtcm4_0p(cc::Jacobian &Jtcm4_0p,
                         const cc::LinearPosition &t,
                         const cc::JointPosition &q,
                         const cc::JointVelocity &qP) const;
    void matrix_Jtcm5_0p(cc::Jacobian &Jtcm5_0p,
                         const cc::LinearPosition &t,
                         const cc::JointPosition &q,
                         const cc::JointVelocity &qP) const;
    void matrix_Jtcm6_0p(cc::Jacobian &Jtcm6_0p,
                         const cc::LinearPosition &t,
                         const cc::JointPosition &q,
                         const cc::JointVelocity &qP) const;
  };

} // namespace ur

#endif
