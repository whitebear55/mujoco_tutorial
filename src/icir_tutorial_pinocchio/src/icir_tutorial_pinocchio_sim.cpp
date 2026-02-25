#include "icir_tutorial_pinocchio/icir_tutorial_pinocchio_sim.hpp"

using namespace std;
using namespace Eigen;
using namespace pinocchio;

void keyboard_event();
void setMode(int mode, const std::string& msg); 
void simCommandCallback(const std_msgs::StringConstPtr &msg);
void check_motion_finished();
void GuiJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

int main(int argc, char **argv)
{
    /////////////////////////// Setting ////////////////////////////////////////////////    
    ros::init(argc, argv, "icir_gen3_pinocchio_sim");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(SAMPLING_RATE);

    // Mujoco Subs
    jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));

    // Mujoco Pubs
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);
    mujoco_run_pub_ = n_node.advertise<std_msgs::Bool>("mujoco_ros/mujoco_ros_interface/sim_run", 5);

    //topic echo
    pos_des_pub_ = n_node.advertise<std_msgs::Float64MultiArray>("pos_des_topic", 10);
    pos_cur_pub_ = n_node.advertise<std_msgs::Float64MultiArray>("pos_cur_topic", 10);

    // Mujoco Msg
    robot_command_msg_.position.resize(GEN3_DOF); 
    robot_command_msg_.torque.resize(GEN3_DOF); 

    // Ros Param
    string urdf_name, urdf_path;
    n_node.getParam("urdf_path", urdf_path);
    n_node.getParam("urdf_name", urdf_name);    

    // Pinocchio
    vector<string> package_dirs;
    package_dirs.push_back(urdf_path);
    std::string urdfFileName = package_dirs[0] + "/" + urdf_name; // urdf file의 전체 경로 생성
    
    // Pinocchio의 기능을 사용하기 쉽게 감싸놓은 Wrapper 클래스의 Instance인 RobotWrapper 생성
    robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, false);  

    // 로봇의 고정된 물리정보(링크,관절한계)
    model_ = robot_->model();

    // 로봇의 현재 값(관절각도, 속도, 질량행렬 등 매 순간 변하는 값)
    Data data(model_);
    data_ = data;

    // q,v,a의 dimension 출력
    cout << "nq : " << robot_->nq() << endl;
    cout << "nv : " << robot_->nv() << endl;
    cout << "na : " << robot_->na() << endl;

    // Control Variable
    state_.J.resize(6, GEN3_DOF);   

    state_.q_des.setZero();
    state_.q_des_pre.setZero();   
    state_.v_des.setZero();
    state_.ddq_des.setZero();
    state_.tau_des.setZero();
    target_offset_.setZero();
    target_rotation_offset_.setZero();

    m_p_.resize(12); // EE의 위치(3개의 위치, 9개의 회전행렬)
    m_v.resize(6); // 3개의 선속도, 각속도
    m_a.resize(6);
    m_v_ref = Motion(m_v.setZero());
    m_a_ref = Motion(m_a.setZero());

    // Control Variable
    step_size_ = 0.05;
    axis_idx_=0;

    // control 대상 지정(6DoF / 7DOF)
    m_frame_id = model_.getFrameId("Actuator6");    
    m_joint_id = model_.getJointId("Actuator6");    
    cout << "m_frame_id : " << m_frame_id << endl;
    cout << "m_joint_id : " << m_joint_id << endl;
    
    cout << "nframes : " << model_.nframes << endl;
    for (pinocchio::FrameIndex i = 0; i < model_.nframes; ++i)
    {
    const auto & f = model_.frames[i];
    std::cout << "Frame ID: " << i
              << ", Name: " << f.name
              << ", Type: " << f.type
              << ", Parent Joint: " << f.parent
              << std::endl;
    }
    ////////////////////////////////////////////////////////////////////////////////////////      

    Home << 0.00, 45.0, 90.0, 0.0, 45.0, -90.0;
    Home2 << 45.00, 45.0, 90.0, 30.0, 50.0, 0.0;    
    ////////////////////////////////////////////////////////////////////////////////////////
    
    // Joint space Gain
    posture_Kp << 40000., 40000., 40000., 40000., 40000., 40000.; // desired pos에서 벗어났을 때 되돌아오려는 강성값
    posture_Kd << 2.0*posture_Kp.cwiseSqrt(); // Damping 
    
    // Task space Gain
    // ee_Kp << 1000., 1000., 1000., 2000., 2000., 2000.;
    ee_Kp << 5000., 5000., 5000., 50000., 50000., 50000.;
    ee_Kd << 2.0*ee_Kp.cwiseSqrt();
    ////////////////////////////////////////////////////////////////////////////////////////

    while (ros::ok()){
        keyboard_event();    
        
        std_msgs::Bool sim_run_msg_;
        sim_run_msg_.data = true;
        mujoco_run_pub_.publish(sim_run_msg_);
        
        robot_->computeAllTerms(data_, state_.q, state_.v);        

        if (ctrl_mode_== 0){
            state_.q_des.setZero();
            state_.tau_des.setZero();    
        }

        if (ctrl_mode_ == 1){ // joint task //h home
            if (chg_flag_){
                cubic_.stime = time_;
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                

                for (int i = 0; i<GEN3_DOF; i++)
                {
                    q_target_(i) = Home(i) * M_PI / 180.;
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                // state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), 0.0, 0.0);                            

                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -posture_Kp(i)*(state_.q(i) - state_.q_des(i)) -posture_Kd(i)*(state_.v(i) - state_.v_des(i));
            }                 
        }

        if (ctrl_mode_ == 2){ // joint task //a home2
            if (chg_flag_){
                cubic_.stime = time_;       
                cubic_.ftime = time_+ 2.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v;                

                for (int i = 0; i<GEN3_DOF; i++)
                {                    
                    q_target_(i) = Home2(i) * M_PI / 180.;
                }               

                chg_flag_ = false;
            }            
            for (int i=0; i<GEN3_DOF; i++)
            {
                // state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), cubic_.v0(i), 0.0);                            
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), 0.0, 0.0);                            

                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                state_.ddq_des(i) = -posture_Kp(i)*(state_.q(i) - state_.q_des(i)) -posture_Kd(i)*(state_.v(i) - state_.v_des(i));
            }            
        }

        if (ctrl_mode_ == 3){
            if (chg_flag_){
                ROS_INFO("time_ at mode3 start: %f", time_);  // 추가
                
                SE3Cubic_.stime = time_;
                SE3Cubic_.duration = 2.0;                

                //set init ee pose
                H_ee_ = robot_->position(data_, m_joint_id); // EE position get
                SE3Cubic_.m_init = H_ee_;                  

                //set desired ee pose
                H_ee_ref_ = H_ee_;
                H_ee_ref_.translation() += target_offset_;

                // 2. 회전 추가 (현재 회전에 5도 회전 행렬을 곱함)
                Matrix3d delta_R;
                delta_R = AngleAxisd(target_rotation_offset_(0), Vector3d::UnitX())
                        * AngleAxisd(target_rotation_offset_(1), Vector3d::UnitY())
                        * AngleAxisd(target_rotation_offset_(2), Vector3d::UnitZ());
                
                H_ee_ref_.rotation() = H_ee_.rotation() * delta_R; // 현재 자세 기준 회전

                chg_flag_ = false;
            }
            // ROS_INFO_THROTTLE(0.5, "Target Offset: x: %.3f, y: %.3f, z: %.3f", target_offset_(0), target_offset_(1), target_offset_(2));
            
            // ROS_INFO_THROTTLE(0.5, "Target Rotation Offset: x: %.3f, y: %.3f, z: %.3f", target_rotation_offset_(0), target_rotation_offset_(1), target_rotation_offset_(2));

            //desired trajectory [VectorXd];
            // SampleEE -> Control Loop가 돌때마다 목표치가 실시간으로 업데이트 되는 값 (위치 + 회전행렬)
            sampleEE_ = SE3Cubic(time_, SE3Cubic_.stime, SE3Cubic_.duration, SE3Cubic_.m_init, H_ee_ref_);
            vectorToSE3(sampleEE_, m_M_ref);  //desired trajectory in SE3 [pinocchio::SE3]
            
            // 현재 상태와 Jacobian을 실시간으로 Update
            SE3 oMi; // Current Pos in World Frame
            Motion v_frame; // World Frmae과 동일한 Origin / 각 축의 방향만 다름                                                           
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame
            SE3ToVector(oMi, m_p_);                        // 벡터 형태로 변환 (m_p_에 저장됨)
            
            // 0.5초에 한 번만 출력하도록 설정
            // ROS_INFO_THROTTLE(0.5, "Current EE Position: X: %.3f, Y: %.3f, Z: %.3f", m_p_(0), m_p_(1), m_p_(2));

            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            // m_drift : 현재 움직임으로 인해 발생하는 가짜 가속도(코리올리 힘 a = jq'' + j'q')
            // 로봇이 현재 움직이고 있는 상태에서, 추가적인 관절 가속도(q'')없이도 발생하는 EE의 코리올리 성분
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot            
            
            // q'이 변할 때, 전역좌표계 방향 기준으로 EE가 어떻게 움직이는지를 나타냄
            robot_->jacobianWorld(data_, m_joint_id, state_.J);           //jacobian in global frame     
            
            // q'이 변할 때, EE가 바라보고 있는 축 방향으로 EE가 어떻게 움직이는지를 나타냄
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame

            // cout << "robot_->position(data_, m_joint_id)" << robot_->position(data_, m_joint_id) << endl;
            // cout << "robot_->framePosition(data_, m_frame_id, oMi)" << oMi << endl;

            SE3ToVector(oMi, m_p_);                                       // current pos in vector form            
            //oMi_inv : Robot 손끝에서 봤을 때, World는 어디에 있는가?
            // oMi_inv*m_M_ref : m_M_ref(글로벌에서 표현된 목표 위치)를 현재 EE의 로컬로 가져오려면 oMi의 역행렬을 통해 로컬로 가져와야함
            // oMi를 그대로 곱하면 로컬을 글로벌로 가져오는거
            // TODO : local frame에서의 위치 오차계산
            errorInSE3(oMi, m_M_ref, m_p_error);                          // pos erorr represented in local frame, oMi_inv*m_M_ref                                    

            // Transformation from local to world
            // m_wMl을 통해 inv 연산을 수행하기 위한 도구
            m_wMl.translation(oMi.translation());                               // use rotation only for vel&acc transformation
            m_wMl.rotation(oMi.rotation());                               // use rotation only for vel&acc transformation

            m_p_error_vec = m_p_error.toVector();                         // pos err vector in local frame            
            // TODO : local frame에서 속도 오차 계산
            m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;                 // vel err vector in local frame                         

            // TODO : desired acc in local frame -> Task Space에서의 PD Control + FeedForward : Local이 다음 순간에 가져야할 목표 가속도
            m_a_des =   ee_Kp.cwiseProduct(m_p_error_vec) // 위치 복원력
                        + ee_Kd.cwiseProduct(m_v_error.toVector()) // 속도 감쇠력
                        + m_wMl.actInv(m_a_ref).toVector(); // 목표 가속도
            
            // ROS_INFO("J condition number: %f",m_J_local_.jacobiSvd().singularValues()(0) / m_J_local_.jacobiSvd().singularValues()(5));

            //transformation from ee to joint            
            state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                            

            //publish
            pos_des_pub();            
            pos_cur_pub();
        }

        if (ctrl_mode_ == 4) { 
            if (chg_flag_) {
                cubic_.stime = time_;
                cubic_.ftime = time_ + 3.0;
                cubic_.q0 = state_.q;
                cubic_.v0 = state_.v; // 시작 속도 저장
                
                q_target_ << 0.0, 45.0, 45.0, 0.0, 90.0, 0.0; // 7자유도 예시
                q_target_ = q_target_ * M_PI / 180.0;

                chg_flag_ = false;
            }
            
            for (int i=0; i<GEN3_DOF; i++) {
                // 1. 목표 위치 계산
                state_.q_des(i) = cubic(time_, cubic_.stime, cubic_.ftime, cubic_.q0(i), q_target_(i), 0.0, 0.0);
                
                // 2. 목표 속도 계산 (효인님이 말씀하신 부분!)
                state_.v_des(i) = (state_.q_des(i) - state_.q_des_pre(i)) * SAMPLING_RATE;
                state_.q_des_pre(i) = state_.q_des(i);
                
                // 3. 완전한 PD 제어 수식 적용
                state_.ddq_des(i) = -posture_Kp(i)*(state_.q(i) - state_.q_des(i)) - posture_Kd(i)*(state_.v(i) - state_.v_des(i));
            }
        }

        if (ctrl_mode_ == 5) { 
            if (chg_flag_){
                ROS_INFO("time_ at mode3 start: %f", time_);  // 추가
                
                H_ee_ = robot_->position(data_, m_joint_id); // EE ref위치 GET
                sine_start_time_ = time_;
                chg_flag_ = false;
            }   
            // Sine 파라미터 설정
            double amplitude = 0.05; // 5cm 왕복
            double frequency = 0.2;  // 0.5Hz (2초에 한 번 왕복)
            
            // 시간 흐름에 따른 목표 위치 계산
            double elapsed_time = time_ - sine_start_time_;
            double offset = amplitude * sin(2.0 * M_PI * frequency * elapsed_time);             

            // 기준 위치는 그대로 두고 하나의 변수를 추가로 생성
            m_M_ref = H_ee_;
            m_M_ref.translation()(axis_idx_) += offset;

            
            // 현재 상태와 Jacobian을 실시간으로 Update
            SE3 oMi; // Current Pos in World Frame
            Motion v_frame; // World Frmae과 동일한 Origin / 각 축의 방향만 다름                                                           
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame            
            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot            
            robot_->jacobianWorld(data_, m_joint_id, state_.J);           //jacobian in global frame                 
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame        
            SE3ToVector(oMi, m_p_);

            // TODO : local frame에서의 위치 오차계산
            errorInSE3(oMi, m_M_ref, m_p_error);

            m_wMl.rotation(oMi.rotation());
            m_p_error_vec = m_p_error.toVector();
            
            // Feedforward 속도 성분 추가 (Sine 미분값)
            Vector6d v_feedforward;
            v_feedforward.setZero();
            v_feedforward(axis_idx_) = amplitude * 2.0 * M_PI * frequency * cos(2.0 * M_PI * frequency * elapsed_time);
            
            m_v_error = m_wMl.actInv(Motion(v_feedforward)) - v_frame;

            m_a_des = ee_Kp.cwiseProduct(m_p_error_vec) 
                    + ee_Kd.cwiseProduct(m_v_error.toVector());

            state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                           

            //publish
            pos_des_pub();            
            pos_cur_pub();
        }

        if (ctrl_mode_ == 6) { // GUI Control Mode
            for (int i = 0; i < GEN3_DOF; i++) {
                state_.q_des(i) = q_target_(i); // 슬라이더 값을 즉시 목표치로 설정
                
                // 속도 목표는 0으로 두어 슬라이더 위치에서 멈추도록 유도
                state_.v_des(i) = 0.0;
                
                // 강력한 PD 제어로 슬라이더 위치를 추종
                state_.ddq_des(i) = -posture_Kp(i)*0.5 * (state_.q(i) - state_.q_des(i)) 
                                - posture_Kd(i) * (state_.v(i) - state_.v_des(i));
            }
        }




        if (ctrl_mode_ == 10){ // impedance control //v
            if (chg_flag_){
                SE3Cubic_.stime = time_;
                SE3Cubic_.duration = 2.0;                

                //set init ee pose
                H_ee_ = robot_->position(data_, m_joint_id);
                SE3Cubic_.m_init = H_ee_;                  

                //set desired ee pose
                H_ee_ref_ = H_ee_;                

                chg_flag_ = false;                
            }

            //to make K(x-xd)=0, put xd=x 
            H_ee_ref_.translation() = robot_->position(data_, m_joint_id).translation();
            m_M_ref = H_ee_ref_;
            
            // SE3ToVector(H_ee_ref_, sampleEE_);
            // vectorToSE3(sampleEE_, m_M_ref);  //desired trajectory in SE3 [pinocchio::SE3]                                 

            SE3 oMi;
            Motion v_frame;                                                            
            robot_->framePosition(data_, m_frame_id, oMi);                //frame position in global frame
            robot_->frameVelocity(data_, m_frame_id, v_frame);            //frame velocity in local frame
            robot_->frameClassicAcceleration(data_, m_frame_id, m_drift); //m_drift in local frame which is identical to J_dot * q_dot                        
            robot_->frameJacobianLocal(data_, m_frame_id, m_J_local_);    //frame jacobian in local frame
            
            SE3ToVector(oMi, m_p_);                                       // current pos in vector form            
            errorInSE3(oMi, m_M_ref, m_p_error);                          // pos erorr represented in local frame, oMi_inv*m_M_ref                                    

            // Transformation from local to world
            m_wMl.rotation(oMi.rotation());                               // use rotation only for vel&acc transformation

            m_p_error_vec = m_p_error.toVector();                         // pos err vector in local frame            
            m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;                 // vel err vector in local frame                         

            cout << m_p_error_vec.transpose() << endl;

            // desired acc in local frame
            m_a_des =   ee_Kp.cwiseProduct(m_p_error_vec)
                        + ee_Kd.cwiseProduct(m_v_error.toVector())
                        + m_wMl.actInv(m_a_ref).toVector();                             

            //transformation from ee to joint            
            state_.ddq_des = m_J_local_.completeOrthogonalDecomposition().pseudoInverse() * (m_a_des - m_drift.toVector());                                            

            //publish
            pos_des_pub();            
            pos_cur_pub();
        }
    
        ////////////////////////////////////////////////////////////////////////////////////////
        state_.tau_des = data_.M * state_.ddq_des + data_.nle;
        robot_command(); //send torque command state_.tau_des to mujoco

        ros::spinOnce();
        check_motion_finished();
        loop_rate.sleep();
    }

    return 0;
}
// 새로운 모드를 설정하고 메시지를 출력하는 도우미 함수
void setMode(int mode, const std::string& msg) {
    ctrl_mode_ = mode;
    chg_flag_ = true;
    cout << "\n----------------------------" << endl;
    cout << msg << endl;
    cout << "----------------------------\n" << endl;
    ROS_INFO("Command Received -> Pos Offset: [%.2f, %.2f, %.2f] | Rot Offset: [%.2f, %.2f, %.2f]",
         target_offset_(0), target_offset_(1), target_offset_(2),
         target_rotation_offset_(0), target_rotation_offset_(1), target_rotation_offset_(2));
}
void keyboard_event(){
    if (_kbhit()){
        int key;
        key = getchar();
        switch (key){
            case 'h': //home joint
                setMode(1, "Move to Home Position");
                log_printed_ = false;
                break;
            case 'g': //gravity
                setMode(0, "Gravity Mode");
                log_printed_ = false;
                break;
            case 'a': //a joint
                setMode(2, "Move to Home2 Position");
                log_printed_ = false;
                break;
                
            case 'z': // Z축 하강
                target_offset_ << 0, 0, -step_size_; 
                target_rotation_offset_.setZero();
                setMode(3, "Move EE -step_size_ Z");
                log_printed_ = false;
                break;

            case 'q': // Z축 상승
                target_offset_ << 0, 0, step_size_;
                target_rotation_offset_.setZero();
                setMode(3, "Move EE +step_size_ Z");
                log_printed_ = false;
                break;

            case 'x': // X축 후진
                target_offset_ << -step_size_, 0, 0;
                target_rotation_offset_.setZero();
                setMode(3, "Move EE -step_size_ X");
                log_printed_ = false;
                break;

            case 's': // X축 전진
                target_offset_ << +step_size_, 0, 0;
                target_rotation_offset_.setZero();
                setMode(3, "Move EE +step_size_ X");
                log_printed_ = false;
                break;  

            case 'e': // y축 전진
                target_offset_ << 0, +step_size_, 0;
                target_rotation_offset_.setZero();
                setMode(3, "Move EE +step_size_ Y");
                log_printed_ = false;
                break;
            case 'c': // y축 후진
                target_offset_ << 0, -step_size_, 0;
                target_rotation_offset_.setZero();
                setMode(3, "Move EE -step_size_ Y");
                log_printed_ = false;
                break;

            case 'p': // Z-axis Sine (기존)
                axis_idx_ = 2; 
                setMode(5, "Sine Motion: Z-axis");
                break;
            case 'o': // X-axis Sine (추가)
                axis_idx_ = 0;
                setMode(5, "Sine Motion: X-axis");
                break;
            case 'i': // Y-axis Sine (추가)
                axis_idx_ = 1;
                setMode(5, "Sine Motion: Y-axis");
                break;


            case '1': // 예: x축(Roll) +5도 회전
                // Rotation Jog를 위한 플래그나 목표 변환값 설정
                target_rotation_offset_ << 5.0 * M_PI / 180.0, 0, 0; 
                target_offset_.setZero();
                setMode(3, "Rotate EE +5deg X-axis");
                log_printed_ = false;
                break;

            case '2': // 예: y축(Roll) +5도 회전
                // Rotation Jog를 위한 플래그나 목표 변환값 설정
                target_rotation_offset_ << 0,5.0 * M_PI / 180.0, 0; 
                target_offset_.setZero();
                setMode(3, "Rotate EE +5deg Y-axis");
                log_printed_ = false;
                break;
            
            case '3': // 예: z축(Roll) +5도 회전
                // Rotation Jog를 위한 플래그나 목표 변환값 설정
                target_rotation_offset_ << 0, 0, 5.0 * M_PI / 180.0; 
                target_offset_.setZero();
                setMode(3, "Rotate EE +5deg Z-axis");
                log_printed_ = false;
                break;
            
            case '6': // GUI 모드 진입
                setMode(6, "GUI진입");
                log_printed_ = false;
                break;
            
            case 'v': // impedance control
                setMode(10, "Impedance Control Mode");
                log_printed_ = false;
                break;

            case 'j': // 'j'를 누르면 새로운 관절 동작 실행
                setMode(4, "New Joint Action: Greeting");
                log_printed_ = false;
                break;  
        }
    }
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);

        // 추가: 시간 및 제어 변수 초기화 -> 시뮬레이터만 초기화되고, ROS의 controller 노드는 초기화가 되지않음
        mujoco_time_ = 0.0;
        time_ = 0.0;
        ctrl_mode_ = 0;
        chg_flag_ = false;
        cubic_.stime = 0.0;
        cubic_.ftime = 0.0;
        SE3Cubic_.stime = 0.0;
        SE3Cubic_.duration = 0.0;
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;

        // 추가
        mujoco_time_ = 0.0;
        time_ = 0.0;
        ctrl_mode_ = 0;
        chg_flag_ = false;
        cubic_.stime = 0.0;
        cubic_.ftime = 0.0;
        SE3Cubic_.stime = 0.0;
        SE3Cubic_.duration = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){

    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
  for (int i=0; i<GEN3_DOF; i++){
    state_.q(i) = msg->position[i];
    state_.v(i) = msg->velocity[i];
  }
}

void robot_command()
{
    robot_command_msg_.MODE = 1;
        robot_command_msg_.header.stamp = ros::Time::now();
    robot_command_msg_.time = time_;
    for (int i=0; i<GEN3_DOF; i++)
    {
        //robot_command_msg_.position[i] = state_.q_des(i);
        robot_command_msg_.torque[i] = state_.tau_des(i);
    }
                      
    robot_command_pub_.publish(robot_command_msg_); 
}

void pos_des_pub()
{
    std_msgs::Float64MultiArray msg;
    VectorXd des_vec(12); // 위치 3 + 회전 9
    
    // m_M_ref(SE3)를 벡터로 변환하여 담기
    SE3ToVector(m_M_ref, des_vec); 
    
    msg.data.resize(des_vec.size());
    for(int i=0; i<des_vec.size(); i++){
        msg.data[i] = des_vec[i];
    }
    pos_des_pub_.publish(msg);
}

void pos_cur_pub()
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(m_p_.size());
    for(int i=0; i<m_p_.size(); i++){
        msg.data[i] = m_p_[i];
    }
    pos_cur_pub_.publish(msg);
}

void check_motion_finished() {
    // Task Space 제어(모드 3)이고, 설정된 시간이 지났을 때
    if (ctrl_mode_ == 3 && !log_printed_) {
        if (time_ >= SE3Cubic_.stime + SE3Cubic_.duration) {
            
            // 현재 EE 위치 계산
            SE3 current_ee;
            robot_->framePosition(data_, m_frame_id, current_ee);
            
            cout << "\n========== Motion Finished ==========" << endl;
            ROS_INFO("Final EE Pos: X: %.4f, Y: %.4f, Z: %.4f", 
                     current_ee.translation()(0), 
                     current_ee.translation()(1), 
                     current_ee.translation()(2));
            cout << "=====================================\n" << endl;
            
            log_printed_ = true; // 한 번만 찍도록 설정
        }
    }
}
