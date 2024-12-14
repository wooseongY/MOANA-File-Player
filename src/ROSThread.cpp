#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{
  // std::cout<<"ros thread create start"<<std::endl;
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  save_flag_ = false;
  process_flag_ = false;
  stop_skip_flag_ = true;


  minimum = 0;
  search_bound_ = 10;
  // search_bound_ = 1000000;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  prev_clock_stamp_ = 0;
  // std::cout<<"ros thread create end"<<std::endl;
}

ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;

  xband_thread_.active_ = false;
  lidar_thread_.active_ = false;
  wband_thread_.active_ = false;
  stereo_left_thread_.active_ = false;
  stereo_right_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  xband_thread_.cv_.notify_all();
  if(xband_thread_.thread_.joinable()) xband_thread_.thread_.join();

  lidar_thread_.cv_.notify_all();
  if(lidar_thread_.thread_.joinable()) lidar_thread_.thread_.join();
  

  wband_thread_.cv_.notify_all(); 
  if(wband_thread_.thread_.joinable()) wband_thread_.thread_.join();

  stereo_left_thread_.cv_.notify_all();
  if(stereo_left_thread_.thread_.joinable()) stereo_left_thread_.thread_.join();

  stereo_right_thread_.cv_.notify_all();
  if(stereo_right_thread_.thread_.joinable()) stereo_right_thread_.thread_.join();
}

void ROSThread::ros_initialize(rclcpp::Node::SharedPtr node)
{
    node_ = node;  // Use a shared pointer for the node

    rclcpp::Time pre_time = rclcpp::Clock().now();
    pre_timer_stamp_ = pre_time.nanoseconds();

    timer_ = node_->create_wall_timer(std::chrono::nanoseconds(100), std::bind(&ROSThread::TimerCallback, this));  // 100ns timer

    // Subscribers
    start_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/file_player_start", 1, std::bind(&ROSThread::FilePlayerStart, this, std::placeholders::_1));

    stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/file_player_stop", 1, std::bind(&ROSThread::FilePlayerStop, this, std::placeholders::_1));

    // Publishers
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);

    lidar_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar", 10);
    xband_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/xband_radar", 10);
    wband_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/wband_radar", 10);
    stereo_left_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera_left", 10); // camera_0
    stereo_right_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera_right", 10); // camera_1

    // Clock Publisher
    clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
}


void ROSThread::run()
{
    // Create an executor (multi-threaded executor in this case)
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add this node to the executor
    executor.add_node(node_);

    // Spin the executor to handle callbacks
    executor.spin();
}

void ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  gps_thread_.active_ = false;
  gps_thread_.cv_.notify_all();
  if(gps_thread_.thread_.joinable()) gps_thread_.thread_.join();

  xband_thread_.active_ = false;
  xband_thread_.cv_.notify_all();
  if(xband_thread_.thread_.joinable()) xband_thread_.thread_.join();

  lidar_thread_.active_ = false;
  lidar_thread_.cv_.notify_all();
  if(lidar_thread_.thread_.joinable()) lidar_thread_.thread_.join();

  wband_thread_.active_ = false;
  wband_thread_.cv_.notify_all();
  if(wband_thread_.thread_.joinable()) wband_thread_.thread_.join();

  stereo_left_thread_.active_ = false;
  stereo_left_thread_.cv_.notify_all();
  if(stereo_left_thread_.thread_.joinable()) stereo_left_thread_.thread_.join();

  stereo_right_thread_.active_ = false;
  stereo_right_thread_.cv_.notify_all();
  if(stereo_right_thread_.thread_.joinable()) stereo_right_thread_.thread_.join();


  //check path is right or not
  ifstream f((data_folder_path_+"/data_stamp.csv").c_str());
  if(!f.good()){
     cout << "data_stamp.csv does not exist." << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;

  std::cout << "read data stamp" << std::endl;
  fp = fopen((data_folder_path_+"/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
    // std::cout<<stamp<<","<<data_name<<std::endl;
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;

  cout << "Data stamps are loaded" << endl;
  fclose(fp);


//Read gps data
  // std::cout << "read gps data" << std::endl;
  // fp = fopen((data_folder_path_+"/gt_pose.csv").c_str(),"r");
  // double latitude, longitude, altitude, altitude_orthometric;
  // double cov[9];
  // sensor_msgs::NavSatFix gps_data;
  // gps_data_.clear();
  // while( fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
  //               &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8])
  //        == 13
  //        )
  // {
  //   gps_data.header.stamp = rclcpp::Time(stamp);
  //   gps_data.header.frame_id = "gps";
  //   gps_data.latitude = latitude;
  //   gps_data.longitude = longitude;
  //   gps_data.altitude = altitude;
  //   for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
  //   gps_data_[stamp] = gps_data;
  // }
  // cout << "Gps data are loaded" << endl;

  // fclose(fp);


  xband_file_list_.clear();
  lidar_file_list_.clear();
  wband_file_list_.clear();
  stereo_left_file_list_.clear();
  stereo_right_file_list_.clear();

  GetDirList(data_folder_path_ + "/sensor_data/LiDAR",lidar_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/X_band_radar",xband_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/W_band_radar", wband_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/Camera_left",stereo_left_file_list_);
  GetDirList(data_folder_path_ + "/sensor_data/Camera_right",stereo_right_file_list_);


  data_stamp_thread_.active_ = true;
  xband_thread_.active_ = true;
  lidar_thread_.active_ = true;
  wband_thread_.active_ = true;
  stereo_left_thread_.active_ = true;
  stereo_right_thread_.active_ = true;

  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  xband_thread_.thread_ = std::thread(&ROSThread::XbandThread,this);
  lidar_thread_.thread_ = std::thread(&ROSThread::LiDARThread,this);
  wband_thread_.thread_ = std::thread(&ROSThread::WbandThread,this);
  stereo_left_thread_.thread_ = std::thread(&ROSThread::StereoLeftThread,this);
  stereo_right_thread_.thread_ = std::thread(&ROSThread::StereoRightThread,this);
}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }


    //check whether stop region or not
    if(stamp == stop_region_iter->first){
      if(stop_skip_flag_ == true){
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true){
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false) return;
    if(iter->second.compare("X_band_radar") == 0){
      xband_thread_.push(stamp);
      xband_thread_.cv_.notify_all();
    }else if(iter->second.compare("LiDAR") == 0){
      lidar_thread_.push(stamp);
      lidar_thread_.cv_.notify_all();
    }else if(iter->second.compare("W_band_radar") == 0){
      wband_thread_.push(stamp);
      wband_thread_.cv_.notify_all();
    }else if(iter->second.compare("Camera_left") == 0){
      stereo_left_thread_.push(stamp);
      stereo_left_thread_.cv_.notify_all();
    }else if(iter->second.compare("Camera_right") == 0){
      stereo_right_thread_.push(stamp);
      stereo_right_thread_.cv_.notify_all();
    }

    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::msg::Clock clock;
        clock.clock = rclcpp::Time(stamp);
        clock_pub_->publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }
    if(save_flag_ == true && process_flag_ == false){

      storage_options.uri = rcpputils::fs::path(data_folder_path_ + "/" + to_string(bag_idx_)).string();
      bag_.open(storage_options);
      process_flag_ = true;
    }
    else if(save_flag_ == false && process_flag_ == true){
      process_flag_ = false;
      bag_.close();
      bag_idx_++;
    }
  }
  cout << "Data publish complete" << endl;
}

// void ROSThread::TimerCallback(const rclcpp::TimerEvent& event)
void ROSThread::TimerCallback()
{
    rclcpp::Time current_time = rclcpp::Clock().now();
    int64_t current_stamp = current_time.nanoseconds();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}


void ROSThread::LiDARThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(lidar_thread_.mutex_);
    lidar_thread_.cv_.wait(ul);
    if(lidar_thread_.active_ == false) return;
    ul.unlock();

    std::cout.precision(20);
    while(!lidar_thread_.data_queue_.empty()){
      auto data = lidar_thread_.pop();
      //publish data
      if(to_string(data) + ".bin" == lidar_next_.first){
        lidar_next_.second.header.stamp = rclcpp::Time(data) ;
        lidar_next_.second.header.frame_id = "lidar";
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          bag_.write(lidar_next_.second, "/lidar", rclcpp::Time(data));
        }
        lidar_pub_->publish(lidar_next_.second);

      }else{
        //load current data
        pcl::PointCloud<pc_type> cloud;
        cloud.clear();
        sensor_msgs::msg::PointCloud2 publish_cloud;
        string current_file_name = data_folder_path_ + "/sensor_data/LiDAR" +"/"+ to_string(data) + ".bin";
        if(find(next(lidar_file_list_.begin(),max(0,previous_file_index-search_bound_)),lidar_file_list_.end(),to_string(data)+".bin") != lidar_file_list_.end()){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            int k = 0;
            while(!file.eof()){
                pc_type point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                point.ring = (k%32) + 1 ;
                k = k+1 ; // 32 beams

                // if(data > 1691936557946849179)
                //     file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                cloud.points.push_back (point);
            }
            file.close();

            pcl::toROSMsg(cloud, publish_cloud);
            publish_cloud.header.stamp = rclcpp::Time(data); 
            publish_cloud.header.frame_id = "lidar";          
            lidar_pub_->publish(publish_cloud);

        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<pc_type> cloud;
      cloud.clear();
      sensor_msgs::msg::PointCloud2 publish_cloud;
      current_file_index = find(next(lidar_file_list_.begin(),max(0,previous_file_index-search_bound_)),lidar_file_list_.end(),to_string(data)+".bin") - lidar_file_list_.begin();
      if(find(next(lidar_file_list_.begin(),max(0,previous_file_index-search_bound_)),lidar_file_list_.end(),lidar_file_list_[current_file_index+1]) != lidar_file_list_.end()){
          string next_file_name = data_folder_path_ + "/sensor_data/LiDAR" +"/"+ lidar_file_list_[current_file_index+1];

          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          int k = 0;
          while(!file.eof()){
              pc_type point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              point.ring = (k%64) + 1 ;
              k = k+1 ;
              // if(data > 1691936557946849179)
              //     file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
              cloud.points.push_back (point);
          }
          file.close();
          pcl::toROSMsg(cloud, publish_cloud);
          lidar_next_ = make_pair(lidar_file_list_[current_file_index+1], publish_cloud);
      }

      previous_file_index = current_file_index;
    }
    if(lidar_thread_.active_ == false) return;
  }
}

void 
ROSThread::WbandThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(wband_thread_.mutex_);
    wband_thread_.cv_.wait(ul);
    if(wband_thread_.active_ == false)
      return;
    ul.unlock();

    while(!wband_thread_.data_queue_.empty())
    {
      auto data = wband_thread_.pop();
      //process
      if(wband_file_list_.size() == 0) continue;

      //publish
      if( to_string(data)+".png" == wband_next_.first && !wband_next_.second.empty() )
      {
        cv_bridge::CvImage wband_out_msg;
        wband_out_msg.header.stamp = rclcpp::Time(data);
        wband_out_msg.header.frame_id = "wband_radar";
        wband_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        wband_out_msg.image    = wband_next_.second;
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          std::shared_ptr<sensor_msgs::msg::Image> image_msg = wband_out_msg.toImageMsg();
          bag_.write(*image_msg, "/wband_radar", rclcpp::Time(wband_out_msg.header.stamp));
        }
        wband_pub_->publish(*wband_out_msg.toImageMsg());
      }
      else
      {
        string current_wband_name = data_folder_path_ + "/sensor_data/W_band_radar/" + to_string(data) + ".png";
        cv::Mat wband_image = imread(current_wband_name, cv::IMREAD_GRAYSCALE);
        if(!wband_image.empty())
        {
          cv_bridge::CvImage wband_out_msg;
          wband_out_msg.header.stamp = rclcpp::Time(data);
          wband_out_msg.header.frame_id = "wband_radar";
          wband_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
          wband_out_msg.image    = wband_image;
          wband_pub_->publish(*wband_out_msg.toImageMsg());
        }
        previous_img_index = 0;
      }

      //load next image
      current_img_index = (int)(find(next(wband_file_list_.begin(), max(0, previous_img_index - search_bound_)),
                                      wband_file_list_.end(), to_string(data)+".png")
                                - wband_file_list_.begin());

      if(current_img_index < (int)wband_file_list_.size()-2)
      {
        string next_wband_name = data_folder_path_ + "/sensor_data/W_band_radar/" + wband_file_list_[current_img_index+1];
        cv::Mat wband_image = imread(next_wband_name, cv::IMREAD_GRAYSCALE);
        if(!wband_image.empty())
        {
          wband_next_ = make_pair(wband_file_list_[current_img_index+1], wband_image);
        }
      }
      previous_img_index = current_img_index;
    }
    
    if(wband_thread_.active_ == false) return;
  }
}


void ROSThread::XbandThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(xband_thread_.mutex_);
    xband_thread_.cv_.wait(ul);
    if(xband_thread_.active_ == false) return;
    ul.unlock();

    while(!xband_thread_.data_queue_.empty()){
      auto data = xband_thread_.pop();
      //process
      if(xband_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".png" == xband_next_.first && !xband_next_.second.empty()){
        cv_bridge::CvImage xband_out_msg;
        xband_out_msg.header.stamp = rclcpp::Time(data);
        xband_out_msg.header.frame_id = "xband_radar";
        xband_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        xband_out_msg.image    = xband_next_.second;
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          std::shared_ptr<sensor_msgs::msg::Image> image_msg = xband_out_msg.toImageMsg();
          bag_.write(*image_msg, "/xband_radar", rclcpp::Time(xband_out_msg.header.stamp));
        }
        xband_pub_->publish(*xband_out_msg.toImageMsg());

      }else{
      //  cout << "Re-load xband image from image path" << endl;

        string current_xband_name = data_folder_path_ + "/sensor_data/X_band_radar" +"/"+ to_string(data)+".png";
        cv::Mat current_xband_image;
        current_xband_image = imread(current_xband_name, cv::IMREAD_ANYDEPTH);

        if(!current_xband_image.empty()){

            cv_bridge::CvImage xband_out_msg;
            xband_out_msg.header.stamp = rclcpp::Time(data);
            xband_out_msg.header.frame_id = "xband_radar";
            xband_out_msg.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            xband_out_msg.image    = current_xband_image;

            xband_pub_->publish(*xband_out_msg.toImageMsg());
        }
        previous_img_index = 0;

      }

      //load next image
      current_img_index = find(next(xband_file_list_.begin(), max(0,previous_img_index - search_bound_)),xband_file_list_.end(),to_string(data)+".png") - xband_file_list_.begin();
      if(current_img_index < xband_file_list_.size()-2){

          string next_xband_name = data_folder_path_ + "/sensor_data/X_band_radar" +"/"+ xband_file_list_[current_img_index+1];
          cv::Mat next_xband_image;
          next_xband_image = imread(next_xband_name, cv::IMREAD_ANYDEPTH);
          if(!next_xband_image.empty()){
              xband_next_ = make_pair(xband_file_list_[current_img_index+1], next_xband_image);
          }
      }
      previous_img_index = current_img_index;
    }
    if(xband_thread_.active_ == false) return;
  }
}

void ROSThread::StereoLeftThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(stereo_left_thread_.mutex_);
    stereo_left_thread_.cv_.wait(ul);
    if(stereo_left_thread_.active_ == false) return;
    ul.unlock();

    while(!stereo_left_thread_.data_queue_.empty()){
      auto data = stereo_left_thread_.pop();
      //process
      if(stereo_left_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".jpg" == stereo_left_next_img_.first && !stereo_left_next_img_.second.empty()){

        cv_bridge::CvImage left_out_msg;
        left_out_msg.header.stamp = rclcpp::Time(data);
        left_out_msg.header.frame_id = "camera_left";
        left_out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        left_out_msg.image    = stereo_left_next_img_.second;

        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          std::shared_ptr<sensor_msgs::msg::Image> image_msg_left = left_out_msg.toImageMsg();
          bag_.write(*image_msg_left, "/camera_left", rclcpp::Time(left_out_msg.header.stamp));
        }

        stereo_left_pub_->publish(*left_out_msg.toImageMsg());

      }else{
//        cout << "Re-load stereo image from image path" << endl;

        string current_stereo_left_name = data_folder_path_ + "/sensor_data/Camera_left" +"/"+ to_string(data)+".jpg";
        cv::Mat current_left_image;
        current_left_image = imread(current_stereo_left_name, cv::IMREAD_ANYDEPTH);

        if(!current_left_image.empty()){
            cv_bridge::CvImage left_out_msg;
            left_out_msg.header.stamp = rclcpp::Time(data);
            left_out_msg.header.frame_id = "camera_left";
            left_out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            left_out_msg.image    = current_left_image;

            stereo_left_pub_->publish(*left_out_msg.toImageMsg());
        }

        previous_img_index = 0;

      }

      //load next image
      current_img_index = find(next(stereo_left_file_list_.begin(), max(0,previous_img_index - search_bound_)),stereo_left_file_list_.end(),to_string(data)+".jpg") - stereo_left_file_list_.begin();
      if(current_img_index < stereo_left_file_list_.size()-2){
          string next_stereo_left_name = data_folder_path_ + "/sensor_data/Camera_left" +"/"+ stereo_left_file_list_[current_img_index+1];
          cv::Mat next_left_image;
          next_left_image = imread(next_stereo_left_name, cv::IMREAD_COLOR);
          if(!next_left_image.empty()){
              stereo_left_next_img_ = make_pair(stereo_left_file_list_[current_img_index+1], next_left_image);
          }

      }
      previous_img_index = current_img_index;
    }
    if(stereo_left_thread_.active_ == false) return;
  }
}

void ROSThread::StereoRightThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(stereo_right_thread_.mutex_);
    stereo_right_thread_.cv_.wait(ul);
    if(stereo_right_thread_.active_ == false) return;
    ul.unlock();

    while(!stereo_right_thread_.data_queue_.empty()){
      auto data = stereo_right_thread_.pop();
      //process
      if(stereo_right_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".jpg" == stereo_right_next_img_.first && !stereo_right_next_img_.second.empty()){

        cv_bridge::CvImage right_out_msg;
        right_out_msg.header.stamp = rclcpp::Time(data);
        right_out_msg.header.frame_id = "camera_right";
        right_out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        right_out_msg.image    = stereo_right_next_img_.second;
        if(save_flag_ == true && process_flag_ == true){
          std::lock_guard<std::mutex> lock(bag_mutex_);
          std::shared_ptr<sensor_msgs::msg::Image> image_msg_right = right_out_msg.toImageMsg();

          bag_.write(*image_msg_right, "/camera_right", rclcpp::Time(right_out_msg.header.stamp) );
        }
        stereo_right_pub_->publish(*right_out_msg.toImageMsg());

      }else{
//        cout << "Re-load stereo image from image path" << endl;

        string current_stereo_right_name = data_folder_path_ + "/sensor_data/Camera_right" +"/"+ to_string(data)+".jpg";
        cv::Mat current_right_image;
        current_right_image = imread(current_stereo_right_name, cv::IMREAD_ANYDEPTH);

        if(!current_right_image.empty()){
            cv_bridge::CvImage right_out_msg;
            right_out_msg.header.stamp = rclcpp::Time(data);
            right_out_msg.header.frame_id = "camera_right";
            right_out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            right_out_msg.image    = current_right_image;
            stereo_right_pub_->publish(*right_out_msg.toImageMsg());
        }

        previous_img_index = 0;

      }

      //load next image
      current_img_index = find(next(stereo_right_file_list_.begin(), max(0,previous_img_index - search_bound_)),stereo_right_file_list_.end(),to_string(data)+".jpg") - stereo_right_file_list_.begin();
      if(current_img_index < stereo_right_file_list_.size()-2){
          string next_stereo_right_name = data_folder_path_ + "/sensor_data/Camera_right" +"/"+ stereo_right_file_list_[current_img_index+1];
          cv::Mat next_right_image;
          next_right_image = imread(next_stereo_right_name, cv::IMREAD_COLOR);
          if(!next_right_image.empty()){
              stereo_right_next_img_ = make_pair(stereo_right_file_list_[current_img_index+1], next_right_image);
          }

      }
      previous_img_index = current_img_index;
    }
    if(stereo_right_thread_.active_ == false) return;
  }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{
  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
  cout<< "Get file list from " << dir << " : " << files.size() << " files" << endl;
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::msg::Bool::SharedPtr msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::msg::Bool::SharedPtr msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }
}
