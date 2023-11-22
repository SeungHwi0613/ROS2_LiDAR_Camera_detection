// Using cones to calculate distance
#define NUM_OF_CONES 11
//2018-07-25 Narrow
int CONES_COOR_Y[NUM_OF_CONES] = { 1080, 774, 646, 578, 548, 526, 512, 495, 484, 477, 473};
float CONES_LONGIT_POS[NUM_OF_CONES] = { 0, 5, 10, 15, 20, 25, 30, 40, 50, 60, 70 };

//-------------------------- Compute parameter of VD objects --------------------------
static float V_Compute_Longi_Dist (const cv::Rect& _objBoundBox){
	
	#if 1	
  int yCoorDiff[NUM_OF_CONES];
  float factor[NUM_OF_CONES];
  
  // Calculate y_coor difference compared to bonnet line
  int yCoorDiff_value = (_objBoundBox.y + _objBoundBox.height) - vpCoorY;
  
  for(int i = 0; i < NUM_OF_CONES; i++){
    yCoorDiff[i] = CONES_COOR_Y[i] - vpCoorY;
  }
  
  // Calculate distance
  float distance = -1;
  for(int i = 0; i < NUM_OF_CONES - 1; i++){
    if((yCoorDiff_value <= yCoorDiff[i]) && (yCoorDiff_value >= yCoorDiff[i+1])){
      // linear interpolation
      float a = float(CONES_LONGIT_POS[i] - CONES_LONGIT_POS[i+1])/ (yCoorDiff[i] - yCoorDiff[i+1]);
      float b = CONES_LONGIT_POS[i] - a*yCoorDiff[i];
      
      distance = a*yCoorDiff_value + b;
    }
    else if(yCoorDiff_value <= yCoorDiff[NUM_OF_CONES - 1]){
      distance = 100.0f;
    }
  }
  
  return distance;
	
	#else
		
	float dist = -1;

	int delta_y     = _objBoundBox.y + _objBoundBox.height - vpCoorY;
	int delta_y_max = bonnetCoorY - vpCoorY;

	// Check valid of delta_y
	if (delta_y >= 1 || delta_y <= delta_y_max){
		// Calculate pixel/centimate ratio 
		float camera_to_5m_line   = calibr_camera_2_bump*0.01f + 5.0f;
		float cam_height_in_meter = calibr_camera_height*0.01f;
		
		float rad_per_pixel = std::atan(cam_height_in_meter/camera_to_5m_line) / (NearY - vpCoorY);	
		
		dist = (cam_height_in_meter / std::tan(delta_y * rad_per_pixel) - calibr_camera_2_bump*0.01f);
	}
		
	return dist;
	
	#endif
	
}

static float V_Compute_Lat_Dist (const cv::Rect& _objBoundBox){
	
	float lateral = 0;
	int coorY = _objBoundBox.y + _objBoundBox.height;
	
	if(coorY <= bonnetCoorY && coorY >= vpCoorY){
		// Calculate lateral (meter) / pixel
		int intersect_x_l = NearLeft;
		if(NearLeft != FarLeft){
			
			float a = float(NearY - FarY) / (NearLeft - FarLeft);
			float b = NearY - a*NearLeft;
			
			intersect_x_l = int((coorY - b) / a);
		}
		
		int intersect_x_r = NearRight;
		if(NearRight != FarRight){
			
			float a = float(NearY - FarY) / (NearRight - FarRight);
			float b = NearY - a*NearRight;
			
			intersect_x_r = int((coorY - b) / a);
		}

		float centimPixelRatio = calibr_vehicle_width / float(intersect_x_r - intersect_x_l);
		
		int targetCarCenter = _objBoundBox.x + _objBoundBox.width/2;
		int egoCarCenter    = (intersect_x_l + intersect_x_r)/2;
		
		lateral = (targetCarCenter - egoCarCenter) * centimPixelRatio / 100.0f; // meter
	}
	
	return lateral;
}