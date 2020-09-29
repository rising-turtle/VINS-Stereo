#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
// queue<sensor_msgs::ImageConstPtr> img_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<sensor_msgs::ImageConstPtr> img2_buf;
std::mutex m_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

FeatureTracker trackerData; // [NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

void sync_process(); 
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg); 

void img_callback1(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}
void img_callback2(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    img2_buf.push(img_msg);
    m_buf.unlock(); 
}


// void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
void handle_stereo_image(cv::Mat& img1, cv::Mat& img2, double msg_timestamp)
{
    static int cnt = 0; 
    // ROS_WARN("received img_msg timestamp: %lf", msg_timestamp);
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = msg_timestamp; // img_msg->header.stamp.toSec();
        last_image_time = msg_timestamp; // img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    // if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    if(msg_timestamp - last_image_time > 1.0 || msg_timestamp < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = msg_timestamp; // img_msg->header.stamp.toSec();
    // frequency control
    // if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    if (round(1.0 * pub_count / (msg_timestamp - first_image_time)) <= FREQ)
    {
       // ROS_INFO("feature_node: first_image_time: %lf current_time: %lf now publish", first_image_time, msg_timestamp);
        PUB_THIS_FRAME = true;
        // reset the frequency control
        // if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        if (abs(1.0 * pub_count / (msg_timestamp - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = msg_timestamp; //  img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    if(EQUALIZE){
         static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
         clahe->apply(img1, img1); 
         clahe->apply(img2, img2); 
    }

    TicToc t_r;
    // map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;    

    trackerData.trackImage(msg_timestamp, img1, img2); 

    if(SHOW_TRACK){
        cv::Mat imgTrack = trackerData.getTrackImage(); 
        // pubTrackImage(imgTrack, msg_timestamp); 

        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(msg_timestamp);
        sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
        // pub_image_track.publish(imgTrackMsg);
        pub_match.publish(imgTrackMsg);
    }

   if (PUB_THIS_FRAME)
   {
        // ofstream ouf("feature_sent.txt");
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header.stamp = ros::Time(msg_timestamp); //  = img_msg->header;
        feature_points->header.frame_id = "world";

        // vector<set<int>> hash_ids(NUM_OF_CAM);
        int camera_id = 0;
        int feature_id; 
        geometry_msgs::Point32 p;

        // features on the left image 
        set<int> hash_ids; 
        for(size_t j=0; j < trackerData.ids.size(); j++){
            if(trackerData.track_cnt[j] > 1){
                feature_id = trackerData.ids[j];
                p.x = trackerData.cur_un_pts[j].x;
                p.y = trackerData.cur_un_pts[j].y;
                p.z = 1;

                hash_ids.insert(feature_id); 
                feature_points->points.push_back(p); 
                id_of_point.values.push_back(feature_id * NUM_OF_CAM + camera_id); 
                u_of_point.values.push_back(trackerData.cur_pts[j].x);
                v_of_point.values.push_back(trackerData.cur_pts[j].y);
                velocity_x_of_point.values.push_back(trackerData.pts_velocity[j].x);
                velocity_y_of_point.values.push_back(trackerData.pts_velocity[j].y);
            }
            // ouf << feature_id<<" p0: "<<p.x<<" "<<p.y<<" "<<endl; 
        }
        // cout <<" send feature_points before right: "<<feature_points->points.size()<<" at "<<std::fixed<<feature_points->header.stamp.toSec()<<endl; 

        // features on the right image 
        camera_id = 1; 
        for(size_t j=0; j < trackerData.ids_right.size(); j++){
            feature_id = trackerData.ids_right[j];
            if(hash_ids.find(feature_id) != hash_ids.end()){
                p.x = trackerData.cur_un_right_pts[j].x;
                p.y = trackerData.cur_un_right_pts[j].y;
                p.z = 1;
                feature_points->points.push_back(p); 
                id_of_point.values.push_back(feature_id * NUM_OF_CAM + camera_id); 
                u_of_point.values.push_back(trackerData.cur_right_pts[j].x);
                v_of_point.values.push_back(trackerData.cur_right_pts[j].y);
                velocity_x_of_point.values.push_back(trackerData.right_pts_velocity[j].x);
                velocity_y_of_point.values.push_back(trackerData.right_pts_velocity[j].y);
            }
        }

        // cout <<" send feature_points after right: "<<feature_points->points.size()<<" at "<<std::fixed<<feature_points->header.stamp.toSec()<<endl; 
        // ROS_WARN("has published %d feature points", pub_count); 
        
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
     
        // static ofstream ouf2("history1.txt"); 
        ROS_INFO("cnt = %d publish %f, at %f with %d features ", ++cnt, feature_points->header.stamp.toSec(), ros::Time::now().toSec(),   feature_points->points.size());
        // ouf2<<"cnt = "<<cnt<<" publish "<<feature_points->header.stamp.toSec()<<" with "<<feature_points->points.size()<<" features "<<endl; 

        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points);
    }
    // ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker_stereo");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); // Debug Info
    readParameters(n);

    // setup cameras 
    // for (int i = 0; i < NUM_OF_CAM; i++){
    //     trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);
    //     camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    //     m_camera.push_back(camera);
    // }
    trackerData.readIntrinsicParameter(CAM_NAMES); 

    if(FISHEYE)
    {
        //for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData.fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData.fisheye_mask.data)
            {
                ROS_INFO("load mask fail FISHEYE_MASK: %s", FISHEYE_MASK.c_str());
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img_callback1);
    ros::Subscriber sub_img2 = n.subscribe(IMAGE2_TOPIC, 100, img_callback2); 

    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    std::thread sync_thread{sync_process};
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image1, image2;
            std_msgs::Header header;
            double msg_timestamp = 0;

            m_buf.lock();
            if (!img1_buf.empty() && !img2_buf.empty())
            {
                double time1 = img1_buf.front()->header.stamp.toSec();
                double time2 = img2_buf.front()->header.stamp.toSec();
                msg_timestamp = time1; 
                // 0.003s sync tolerance
                if(time1 < time2 - 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else if(time1 > time2 + 0.003)
                {
                    img2_buf.pop();
                    printf("throw img2\n");
                }
                else
                {
                    msg_timestamp = img1_buf.front()->header.stamp.toSec();
                    header = img1_buf.front()->header;
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    image2 = getImageFromMsg(img2_buf.front());
                    img2_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            
            // if(!image0.empty())
                // estimator.inputImage(time, image0, image1);
            if(!image1.empty())
                handle_stereo_image(image1, image2, msg_timestamp); 
        }
        else
        {
            ROS_ERROR("feature_tracker_node.cpp: should not arrive here!"); 
            // cv::Mat image;
            // std_msgs::Header header;
            // double time = 0;
            // m_buf.lock();
            // if(!img0_buf.empty())
            // {
            //     time = img0_buf.front()->header.stamp.toSec();
            //     header = img0_buf.front()->header;
            //     image = getImageFromMsg(img0_buf.front());
            //     img0_buf.pop();
            // }
            // m_buf.unlock();
            // if(!image.empty())
                // estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat ret_img;
    if (img_msg->encoding == "8UC1")
    {
        ROS_DEBUG("feature_tracker_node.cpp: image type: gray 8UC1 "); 
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image.clone();
    }
    else if(img_msg->encoding == "8UC3"){

        ROS_DEBUG("feature_tracker_node.cpp: image type: RGB 8UC3"); 

        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "bgr8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        ret_img = ptr->image.clone(); 
        cv::cvtColor(ret_img, ret_img, cv::COLOR_BGR2GRAY);
    }
    else{
        // ROS_DEBUG("feature_tracker_node.cpp: image type: unknown"); 
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image.clone();
    }
    
    return ret_img;
}

cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}