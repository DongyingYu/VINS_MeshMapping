#include "pose_graph.h"
extern float RESOLUTION;
PoseGraph::PoseGraph()
{
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    posegraph_visualization->setScale(0.1);
    posegraph_visualization->setLineWidth(0.01);
    //开一个线程,执行优化
	t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;

}

PoseGraph::~PoseGraph()
{
	t_optimization.join();
}

void PoseGraph::registerPub(ros::NodeHandle &n)
{
    pub_pg_path = n.advertise<nav_msgs::Path>("pose_graph_path", 1000);
    pub_base_path = n.advertise<nav_msgs::Path>("base_path", 1000);
    pub_pose_graph = n.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
    pub_pointcloud2 = n.advertise<sensor_msgs::PointCloud2>("pointcloud2",1000);
    // pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("odometry_input_topic",1000);
    //去掉八叉树的显示,无用信息
    //pub_octree = n.advertise<sensor_msgs::PointCloud2>("octree", 1000);
    for (int i = 1; i < 10; i++)
        pub_path[i] = n.advertise<nav_msgs::Path>("path_" + to_string(i), 1000);
}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    // sequence = 1, sequence_cnt = 0 at init, sequence_cnt++
    // then sequence_cnt remains 1 (no new sequence in my case)
    if (sequence_cnt != cur_kf->sequence)
    {
        //run once
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
	// always true
    if (flag_detect_loop)
    {
        TicToc tmp_t;
        // get loop_index here
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
	if (loop_index != -1)
	{
        //printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);

        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)
                {
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio *  vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
	}
	m_keyframelist.lock();
    
    Vector3d P;
    Matrix3d R;
    //获取VIO当前帧的位姿P、R，根据偏移量得到实际位姿
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    //更新当前帧的位姿P、R
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();

    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    // 八叉树地图融合,需要结合运动位姿进行融合,组合过后发布八叉树地图信息
    //--------------------------------------------------------------------
    // sensor_msgs::PointCloud2 tmp_pcl;
    // m_octree.lock();

    // int pcl_count_temp = 0;
    // for (unsigned int i = 0; i < cur_kf->point_3d_depth.size(); i++)
    // {
    //     //对传入octree的地图点进行了一系列操作
    //     cv::Point3f pcl = cur_kf->point_3d_depth[i];
    //     Vector3d pts_i(pcl.x , pcl.y, pcl.z);
    //     Vector3d w_pts_i = R * (qi_d * pts_i + ti_d) + P;
    //     pcl::PointXYZ searchPoint;
    //     searchPoint.x = w_pts_i(0);
    //     searchPoint.y = w_pts_i(1);
    //     searchPoint.z = w_pts_i(2);
    //     if (octree->getVoxelDensityAtPoint(searchPoint) < 5)
    //     {
    //         cur_kf->point_3d_depth[pcl_count_temp] = pcl;
    //         octree->addPointToCloud(searchPoint, cloud);
    //         // Uncomment this to get pointcloud
    //         //save_cloud->points.push_back(searchPoint);
    //         ++pcl_count_temp;
    //     }
    // }
    // cur_kf->point_3d_depth.resize(pcl_count_temp);
    // pcl::toROSMsg(*(octree->getInputCloud()), tmp_pcl);
    // m_octree.unlock();
    //-----------------------------------------------------------------

    // not used
    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);
        loop_path_file.precision(0);
        loop_path_file << cur_kf->time_stamp * 1e9 << ",";
        loop_path_file.precision(5);
        loop_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        loop_path_file.close();
    }
    // not used
    // draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    // show connections between loop frames, not needed
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)
        {
            //printf("has loop \n");
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P,P0;
            Matrix3d connected_R,R0;
            connected_KF->getPose(connected_P, connected_R);
            //cur_kf->getVioPose(P0, R0);
            cur_kf->getPose(P0, R0);
            if(cur_kf->sequence > 0)
            {
                //printf("add loop into visual \n");
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }

        }
    }
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
    //----------------------------------------------
    //tmp_pcl.header = pose_stamped.header;
    // add frame to key frame list
	keyframelist.push_back(cur_kf);
    publish();
    // -----------------------
    // pub_octree.publish(tmp_pcl);
	m_keyframelist.unlock();

}


void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    if (flag_detect_loop)
       loop_index = detectLoop(cur_kf, cur_kf->index);
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    if (loop_index != -1)
    {
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    //draw local connection
    if (SHOW_S_EDGE)
    {
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())
                break;
            Vector3d conncected_P;
            Matrix3d connected_R;
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;
        }
    }
    /*
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */

    keyframelist.push_back(cur_kf);
    //publish();
    m_keyframelist.unlock();
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    // set false, not used
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    //------------------------------------------------------------------------------
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    //------------------------------------------------------------------------------
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
    }
/*


    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        m_optimize_buf.lock();
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            optimize_buf.pop();
        }
        m_optimize_buf.unlock();
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j],
                                            t_array[i-j],
                                            euler_array[i],
                                            t_array[i]);
                  }
                }

                //add loop edge

                if((*it)->has_loop)
                {
                    assert((*it)->loop_index >= first_looped_index);
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index],
                                                                  t_array[connected_index],
                                                                  euler_array[i],
                                                                  t_array[i]);

                }

                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            //printf("pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            m_keyframelist.unlock();
            updatePath();
        }

        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
}

void PoseGraph::updatePath()
{
    TicToc t_update;
    // store info for updating dense pcl without locking keyframe list
    // which may take much time
    vector<vector<cv::Point3f>> tmp_keyframelist;
    queue<pair<Matrix3d, Vector3d>> tmp_RTlist;
    std_msgs::Header tmp_header;


    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();
    posegraph_visualization->reset();
    // not used
    if (SAVE_LOOP_PATH)
    {
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();
    }

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);

        tmp_RTlist.push(make_pair(R, P));

        Quaterniond Q;
        Q = R;
//        printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time((*it)->time_stamp);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();

        tmp_keyframelist.push_back((*it)->point_3d_depth);



        if((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
            tmp_header = pose_stamped.header;
        }
        //not used
        if (SAVE_LOOP_PATH)
        {
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);
            loop_path_file.precision(0);
            loop_path_file << (*it)->time_stamp * 1e9 << ",";
            loop_path_file.precision(5);
            loop_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            loop_path_file.close();
        }
        //draw local connection
        // not used
        if (SHOW_S_EDGE)
        {
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;
            for (; rit != keyframelist.rend(); rit++)
            {
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;
                    lrit++;
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())
                            break;
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;
                    }
                    break;
                }
            }
        }
        if (SHOW_L_EDGE)
        {
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {

                KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);
                //(*it)->getVioPose(P, R);
                (*it)->getPose(P, R);
                if((*it)->sequence > 0)
                {
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }

    }
    publish();
    m_keyframelist.unlock();
    
    //-----------------------------------------------------------
    // 八叉树操作部分
    // throw the costy part beyond m_keyframelist
    // m_octree.lock();
    // //some clean up
    // octree->deleteTree();
    // cloud->clear();
    // //save_cloud->clear();
    // octree->setInputCloud(cloud);
    // octree->addPointsFromInputCloud();
    // octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
    // int update_count = 0;
    // for (auto &pcl_vect : tmp_keyframelist)
    // {
    //     Vector3d P;
    //     Matrix3d R;
    //     R = tmp_RTlist.front().first;
    //     P = tmp_RTlist.front().second;
    //     for (auto &pcl : pcl_vect)
    //     {
            
    //         Vector3d pts_i(pcl.x , pcl.y, pcl.z);
    //         Vector3d w_pts_i = R * (qi_d * pts_i + ti_d) + P;
    //         pcl::PointXYZ searchPoint;
    //         searchPoint.x = w_pts_i(0);
    //         searchPoint.y = w_pts_i(1);
    //         searchPoint.z = w_pts_i(2);
    //         ++update_count;
    //         octree->addPointToCloud(searchPoint, cloud);
    //         // Uncomment this to get pointcloud
    //         //save_cloud->points.push_back(searchPoint);
    //     }
    //     tmp_RTlist.pop();
    // }
    // pclFilter(true);
    // m_octree.unlock();
    // ROS_INFO("Update done! Time cost: %f   total points: %d", t_update.toc(), update_count);
    //--------------------------------------------------------------- 

}

//---------没用到-----------------
//sensor_msgs::PointCloud2 PointsToPointCloud (const cv::Mat img, const cv::Mat depth)
/*
void PoseGraph::PointsToPointCloud(const cv::Mat& color, const cv::Mat& depth, const double& cx, 
									   const double& cy, const double& fx, const double& fy,const int ROW, const int& COL )
{
    PointCloud::Ptr tmp(new PointCloud );
    //sensor_msgs::PointCloud2ConstPtr pointcloud；
    vector<cv::Point3f> point_depth;
    //
    cv::Mat _depth, _color;
    _depth = depth.clone();
    _color = color.clone();
    for ( int m=0; m< 480; m+=3 )
    {
        for ( int n=0; n< 640; n+=3 )
        {
            //取第m行 第n列的深度值; ptr函数会返回指向该图像第m行数据的头指针,然后加上位移n.
            float d = (float)_depth.ptr<unsigned short>(m)[n] / 1000.0;
            //float d = ((float)depth.at<unsigned short>(n, m)) / 1000.0;
            //std::cout << "---------d:      " << d << std::endl;
            // if (d < 0.01 || d>5)
            //     continue;

            if (d > 0.02 && d < 6)
            {
                PointT p;
                p.z = d;
                p.x = ( n - cx) * p.z / fx;
                p.y = ( m - cy) * p.z / fy;
                
                // std::cout << "cx:   " << cx << "cy:   " << cy << std::endl;
                // std::cout << "fx:   " << fx << "fy:   " << fy << std::endl;

                p.r = _color.ptr<uchar>(m)[n*3];
                p.g = _color.ptr<uchar>(m)[n*3+1];
                p.b = _color.ptr<uchar>(m)[n*3+2];
                //得到带RGB信息的点云信息
                tmp->points.push_back(p);
                //cloud->data.push_back(p);
            }

        }
    }
    //cv::imwrite("/home/ipsg/study/vins/VinsRGBD_debug/color.png",color);
    //报错:输入点云没有数据
    // std::cout << "------点云数量:      " << tmp->points.size() << std::endl;
    // std::cout << "------save pcdfile....------" << std::endl;
    pcl::io::savePCDFileBinary("/home/ipsg/study/vins/VinsRGBD_debug/map_viewer.pcd", *tmp);
    // std::cout << "------save pcdfile has done...------" << std::endl;

    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*tmp, cloud);

    // const int num_channels = 3; // x y z
    //cloud.header.seq = 
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
    //一般无序点云的高设置为1
    // cloud.height = 1;
    // cloud.width = tmp->points.size();
    // cloud.is_bigendian = false;
    // cloud.is_dense = true;
    // cloud.point_step = num_channels * sizeof(float);
    // cloud.row_step = cloud.point_step * cloud.width;
    // cloud.fields.resize(num_channels);

    // std::string channel_id[] = { "x", "y", "z"};
    // for (int i = 0; i<num_channels; i++) {
    //     cloud.fields[i].name = channel_id[i];
    //     cloud.fields[i].offset = i * sizeof(float);
    //     cloud.fields[i].count = 1;
    //     cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    // }
    //pcl::fromROSMsg(tmp,*cloud);
    pub_pointcloud2.publish(cloud);
    // point cloud is null ptr

    // cloud.data.resize(cloud.row_step * cloud.height);

	// unsigned char *cloud_data_ptr = &(cloud.data[0]);

    // float data_array[num_channels];
    // for (unsigned int i=0; i<cloud.width; i++) {
    //     if (map_points.at(i)->nObs >= min_observations_per_point_) {
    //         data_array[0] = map_points.at(i)->GetWorldPos().at<float> (2); //x. Do the transformation by just reading at the position of z instead of x
    //         data_array[1] = -1.0* map_points.at(i)->GetWorldPos().at<float> (0); //y. Do the transformation by just reading at the position of x instead of y
    //         data_array[2] = -1.0* map_points.at(i)->GetWorldPos().at<float> (1); //z. Do the transformation by just reading at the position of y instead of z
    //          //TODO dont hack the transformation but have a central conversion function for MapPointsToPointCloud and TransformFromMat

    //         memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
    //     }
    // }


}
*/

// 注意点云格式问题  https://github.com/felixchenfy/open3d_ros_pointcloud_conversion
void PoseGraph::PointsToPointCloud( const pcl::PointCloud<PointT>::Ptr &Point, const std_msgs::Header &header)
{
    //std::cout << "--------------point test  1---------------" << std::endl;
    //pcl::io::savePCDFileBinary("/home/ipsg/dataset/map_viewer.pcd", *Point);

    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*Point, cloud);
    //std::cout << "--------------point test  2---------------" << std::endl;
    cloud.header = header;
    //loud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "odom";

    cloud.fields.resize(4);
    std::string channel_id[] = { "x", "y", "z", "rgb"};

    cloud.fields[0].name = channel_id[0];
    cloud.fields[0].offset = 0 * sizeof(float);
    cloud.fields[0].count = 1;
    cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;

    cloud.fields[1].name = channel_id[1];
    cloud.fields[1].offset = 1 * sizeof(float);
    cloud.fields[1].count = 1;
    cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;

    cloud.fields[2].name = channel_id[2];
    cloud.fields[2].offset = 2 * sizeof(float);
    cloud.fields[2].count = 1;
    cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;

    cloud.fields[3].name = channel_id[3];
    cloud.fields[3].offset = 4 * sizeof(float);
    cloud.fields[3].count = 1;
    cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;



    pub_pointcloud2.publish(cloud);
}

void PoseGraph::pubKeyframePose(const nav_msgs::Odometry::ConstPtr &pose_msg, const std_msgs::Header &header){
    std::cout << "[INFO]: Enter into the pose_graph pubKeyframe function. " << std::endl;
    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "imu";

    odometry.pose.pose.position.x = pose_msg->pose.pose.position.x;
    odometry.pose.pose.position.y = pose_msg->pose.pose.position.y;
    odometry.pose.pose.position.z = pose_msg->pose.pose.position.z;
    odometry.pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    odometry.pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    odometry.pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    odometry.pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;

    // 设置速度值,对应于子节点,可以不需要发布下面的信息
    // odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    // odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    // odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    //"odometry"
    pub_keyframe_pose.publish(odometry);    

}

// ---------------------------------------------------------------------------------------------
// 不使用八叉树点云处理,不需用到
/*
void PoseGraph::pclFilter(bool flag)
{
    //滤波
    pcl::PointCloud<pcl::PointXYZ> cloud_copy(*(octree->getInputCloud()));
    pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>* temp_octree = new pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>(RESOLUTION);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    cout<<"Size of Cloud before filter:"<<cloud_copy.size()<<endl;
    pcl::PointCloud<pcl::PointXYZ>::iterator pcl_iter= cloud_copy.begin();
    while(pcl_iter != cloud_copy.end())
    {
        if (octree->getVoxelDensityAtPoint(*pcl_iter) >= 3)
        {
            temp_cloud->push_back(*pcl_iter);
        }
        ++pcl_iter;
    }
    temp_octree->defineBoundingBox(-100, -100, -100, 100, 100, 100);
    temp_octree->setInputCloud(temp_cloud);
    temp_octree->addPointsFromInputCloud();
    cout<<"Size of Cloud after filter:"<<temp_cloud->size()<<endl;
    // filter octree after loop
    if (flag)
    {
        std::cout << "------test2-------" << std::endl;
        //lock octree and cloud
        delete octree;
        (*cloud).clear();
        octree = temp_octree;
        cloud = temp_cloud;
        //sensor_msgs::PointCloud2 tmp_pcl;
        //pcl::toROSMsg(*temp_cloud, tmp_pcl);
        //tmp_pcl.header.stamp =  ros::Time::now();
        //tmp_pcl.header.frame_id = "world";
        //pub_octree.publish(tmp_pcl);
    }
    // use 'p' to filter octree at any time
    else
    {
        std::cout << "------test1-------" << std::endl;
        sensor_msgs::PointCloud2 tmp_pcl;
        pcl::toROSMsg(*temp_cloud, tmp_pcl);
        tmp_pcl.header.stamp =  ros::Time::now();
        tmp_pcl.header.frame_id = "world";
        pub_octree.publish(tmp_pcl);
    }


}
*/
//--------------------------------------------------------------------------------------------

void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile,*pFile_shan_pg,*pFile_shan_vio;
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
//    string file_path_shan_pg = POSE_GRAPH_SAVE_PATH + "stamped_traj_estimate_mono_pg.txt";
//    string file_path_shan_vio = POSE_GRAPH_SAVE_PATH + "stamped_traj_estimate_mono_vio.txt";
    string file_path_shan_pg = POSE_GRAPH_SAVE_PATH + "stamped_traj_estimate.txt";
    string file_path_shan_vio = POSE_GRAPH_SAVE_PATH+ "vio_stamped_traj_estimate.txt";
    pFile = fopen (file_path.c_str(),"w");
    pFile_shan_pg = fopen(file_path_shan_pg.c_str(),"w");
    pFile_shan_vio = fopen(file_path_shan_vio.c_str(),"w");
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    std::cout << "[INFO]: The size of Keyframe list is :  " << keyframelist.size() << std::endl;
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, depth_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            // image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            image_path = POSE_GRAPH_SAVE_PATH + "/rgb/" + to_string((*it)->index + 1) + ".png";
            depth_path = POSE_GRAPH_SAVE_PATH + "/depth/" + to_string((*it)->index + 1) + ".png";
            imwrite(image_path.c_str(), (*it)->image_color);
            imwrite(depth_path.c_str(), (*it)->depth);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Quaterniond rVIO_tmp_Q{(*it)->vio_R_w_i.transpose()};
        Quaterniond rPG_tmp_Q{(*it)->R_w_i.transpose()};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;

        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",(*it)->index, (*it)->time_stamp,
                 VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
                 PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                 VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),
                 PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),
                 (*it)->loop_index,
                 (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                 (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                 (int)(*it)->keypoints.size());
        // 位姿保存的格式符合要求，为TUM位姿格式；
        // fprintf (pFile_shan_pg, "%f %f %f %f %f %f %f %f\n",(*it)->time_stamp,
        //          PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
        //          PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),PG_tmp_Q.w());
        // fprintf (pFile_shan_vio, "%f %f %f %f %f %f %f %f\n",(*it)->time_stamp,
        //          VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
        //          VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),VIO_tmp_Q.w());
        fprintf (pFile_shan_pg, "%f %f %f %f %f %f %f\n",
                 PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                 PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),PG_tmp_Q.w());
        fprintf (pFile_shan_vio, "%f %f %f %f %f %f %f\n",
                 VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
                 VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),VIO_tmp_Q.w());

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;

        /*
        // 按需要进行保存
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y,
                    (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);
        */
    }
    fclose(pFile_shan_vio);
    fclose(pFile_shan_pg);
    fclose(pFile);


    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}

void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp,
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz,
                                    &PG_Tx, &PG_Ty, &PG_Tz,
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF)
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                    VIO_Tx, VIO_Ty, VIO_Tz,
                                    PG_Tx, PG_Ty, PG_Tz,
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void PoseGraph::publish()
{
    for (int i = 1; i <= sequence_cnt; i++)
    {
        //if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            pub_pg_path.publish(path[i]);
            pub_path[i].publish(path[i]);
            //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }

    pub_base_path.publish(base_path);

    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

void PoseGraph::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1 > &_loop_info)
{
    KeyFrame* kf = getKeyFrame(index);
    kf->updateLoop(_loop_info);
    if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
    {
        if (FAST_RELOCALIZATION)
        {
            KeyFrame* old_kf = getKeyFrame(kf->loop_index);
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getPose(w_P_old, w_R_old);
            kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = kf->getLoopRelativeT();
            relative_q = (kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;

            m_drift.lock();
            yaw_drift = shift_yaw;
            r_drift = shift_r;
            t_drift = shift_t;
            m_drift.unlock();
        }
    }
}
