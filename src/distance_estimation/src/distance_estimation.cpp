#define BOOST_BIND_NO_PLACEHOLDERS

#include "distance_estimation/distance_estimation_node.hpp"

Distance_estimation::Distance_estimation(const rclcpp::NodeOptions& options) 
: Node("distance_estimation",options)
{  
 /*related topics */
  detection_sub_ =
    std::make_unique<message_filters::Subscriber<yolov8_msgs::msg::DetectionArray>>(
      this, "/yolo/tracking");
  center_sub_ =
    std::make_unique<message_filters::Subscriber<cluster_msgs::msg::ClusterArray>>(
      this, "/comparison/final");
  sync_ =
    std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(10), *detection_sub_, *center_sub_);

  sync_->registerCallback(&Distance_estimation::distance_estimation_cb, this);

  RCLCPP_INFO(this->get_logger(), "\n"
  "Node:       Distance_estimation\n");
}

float Distance_estimation::most_common_search(std::vector<float> prev_frame_track_id_array)
{
  std::unordered_map<int, float> frequency; // 숫자 등장 횟수를 저장할 unordered_map

  // 각 숫자의 등장 횟수를 세기
  for (const auto& num : prev_frame_track_id_array) {
    // std::cout<<"check3"<<std::endl;
      frequency[num]++;
  }

  // 최댓값 찾기
  int maxFrequency = 0;
  float mostCommonNumber = 0;
  for (const auto& pair : frequency) {
      if (pair.second > maxFrequency) {
          maxFrequency = pair.second;
          mostCommonNumber = pair.first;
      }
  }

  // 결과 출력
  // std::cout << "가장 많이 등장하는 숫자: " << mostCommonNumber << " (등장 횟수: " << maxFrequency << "회)" << std::endl;

  return mostCommonNumber;
}

float Distance_estimation::distance_estimation(float most_common_id, std::vector<std::vector<float>> prev_frame_total_array)
{
  std::vector<float> assumption_target_height;
  std::vector<float> assumption_target_x;
  std::vector<float> assumption_target_y;

  for(int i=0; i<prev_frame_total_array.size(); i++)
  {
    if(prev_frame_total_array[i][0]==most_common_id)
    {
      // std::cout<<"check4"<<std::endl;
      assumption_target_height.push_back(prev_frame_total_array[i][1]);
      assumption_target_x.push_back(prev_frame_total_array[i][2]);
      assumption_target_y.push_back(prev_frame_total_array[i][3]);
    }
  }
  std::cout<<"target_appear_times: "<<assumption_target_height.size()<<std::endl;
  std::cout<<"target_id: "<<most_common_id<<std::endl;
  for(int j=0; j<assumption_target_height.size()-1; j++)
  {
    if(assumption_target_height[j+1]>assumption_target_height[j])
    {
      float m = 0.7; //30km/h speed && 0.1 second
      float distance = m / (1-(assumption_target_height[j]/assumption_target_height[j+1]));

      // std::cout<<"distance: "<<distance<<std::endl;
      // std::cout<<"center_x: "<<assumption_target_x[j]<<std::endl;
      // std::cout<<"center_y: "<<assumption_target_y[j]<<std::endl;
      if((int)assumption_target_y[j]>=188 &&(int)assumption_target_y[j]<=200){
        std::cout<<"[ distance | center_y ] "<<(int)distance<<"|"<<(int)assumption_target_y[j]<<std::endl;
      }
    }
    else{
      std::cout<<"j+1<j"<<std::endl;
    }
  }
}

void Distance_estimation::distance_estimation_cb(
  const yolov8_msgs::msg::DetectionArray detection_msg, 
  const cluster_msgs::msg::ClusterArray center_msg) 
{
  // std::cout<<"||       callBack        ||"<<std::endl;
  if(frame_i==10)
  {
    float most_common_id=most_common_search(prev_frame_track_id_array);
    float distance=distance_estimation(most_common_id, prev_frame_total_array);

    prev_frame_track_id_array.clear();
    prev_frame_total_array.clear();

    frame_i=0;
  }


  for(int i=0; i<=detection_msg.detections.size(); i++)
  {
    
    double left_top_x=round(detection_msg.detections[i].bbox.center.position.x-detection_msg.detections[i].bbox.size.x / 2.0);
    double left_top_y=round(detection_msg.detections[i].bbox.center.position.y-detection_msg.detections[i].bbox.size.y / 2.0);
    double right_bottom_x=round(detection_msg.detections[i].bbox.center.position.x+detection_msg.detections[i].bbox.size.x / 2.0);
    double right_bottom_y=round(detection_msg.detections[i].bbox.center.position.y+detection_msg.detections[i].bbox.size.y / 2.0);

    if(right_bottom_x-left_top_x<500 && right_bottom_x-left_top_x>0
      && right_bottom_y-left_top_y<500 && right_bottom_y-left_top_y>0
      && detection_msg.detections[i].id.empty()==false) //exception cover   *edit later
    {
      double bbox_width_x=right_bottom_x-left_top_x;
      double bbox_height_y=right_bottom_y-left_top_y;
      std::string track_id= detection_msg.detections[i].id;

      prev_frame_track_id_array.push_back(std::stof(track_id));
      prev_frame_total_array.push_back({std::stof(track_id), (float)bbox_height_y, detection_msg.detections[i].bbox.center.position.x, detection_msg.detections[i].bbox.center.position.y});
    }
  }
  frame_i++;
}