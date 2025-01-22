https://www.notion.so/depth_reader-py-17ec1ea8cb1c80cda1eef52d34cf17c9


//depth_reader.py 
depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') # ROS 메세지를 openCV로 이미지로 변환
depth_image는 리스트,배열 형태임 2차원 배열, 
center_depth = depth_image[height // 2, width //2]
