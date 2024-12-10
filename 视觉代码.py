# #------------------------引入停车线和斑马线和转向线并向下位机发送数据--------------------------------------
# #--------------------------------更正滑动弹窗和巡线中点贴合赛道线-------------------------------------------
# import time
#
# from self import self
# from sklearn.cluster import KMeans
# import numpy as np
# import cv2
# import struct  # 用于数据打包
# from enum import Enum
# from SerialComm import SerialComm
#
# class State(Enum):
#     left = 1
#     right = 2
#     center = 3
#     left_to_center = 4
#     right_to_center = 5
#
#
# class BikeClass:
#     def __init__(self):
#     #    self.comm = SerialComm(port="/dev/ttyUSB0", baudrate=115200, timeout=1)
#     #    self.comm = SerialComm(port="COM3", baudrate=115200, timeout=1)
#
#         self.M = None
#         self.M_inverse = None
#         self.get_warp_M()
#
#         self.img_size = (640, 480)
#         self.img = None
#         self.warp_img = None
#         self.edges = None
#         self.cap = cv2.VideoCapture(0)
#         if not self.cap.isOpened():
#             print("Error: Could not open video device.")
#             self.cap = None  # 确保无法访问无效的 VideoCapture 对象
#
#         self.state = State.center
#         self.margin = 35          #滑动弹窗的宽度可调整初始值35可调整
#         self.minpix = 25
#         self.nwindows = 10       #滑动窗口的窗口数
#         self.window_height = np.int32(self.img_size[1] / self.nwindows)
#
#         self.left_fit = None  # 初始化为 None
#         self.right_fit = None  # 初始化为 None
#
#         self.leftx_mean = None
#         self.rightx_mean = None
#         self.line_points_x = []
#         self.left_line_x = []
#         self.right_line_x = []
#
#         self.dynamic_center_x = self.img_size[0] // 2
#         self.to_center_x = 38         #偏移量可微调减小降低右侧敏感度，增大降低左侧敏感度 38初始值
#         self.error_flag = 0
#         self.last_x_error = None
#
#         self.rightx_mean_list = []
#         self.leftx_mean_list = []
#         self.kmeans = KMeans(n_clusters=2)
#
#         self.bike_args = type('', (), {})()
#         self.bike_args.shifted_x = 150
#         self.bike_args.dynamic_shift_x = 0
#         self.bike_args.block_detect_y = 300  # 摄像头检测障碍物的前瞻距离
#         self.bike_args.block_detect_delta_y = 150  # 检测区域高度
#         self.bike_args.blue_low = np.array([100, 150, 0])  # 蓝色下限
#         self.bike_args.blue_upper = np.array([140, 255, 255])  # 蓝色上限
#         self.bike_args.yellow_low = np.array([20, 100, 100])  # 黄色下限
#         self.bike_args.yellow_upper = np.array([30, 255, 255])  # 黄色上限
#         self.bike_args.block_h_upper = 700  # 锥桶高度上限
#
#         self.catch_block_times = 0
#         self.last_block_state = State.center
#         self.block_direction_list = [State.center] * 5  # 记录最近的状态
#         self.skip_block_nums = 0  # 跳过的锥桶数量
#
#
#         self.wait_back_center_flag = 0
#         self.skip_frame_times = 0
#         self.initial_skip_frame_times = 10  # 设置回归中线的帧数
#         self.stop_line_times = 0  # 停车线计数
#         self.stop_line_flag = True  # 停车线标志位
#
#         self.x_error = None  # 初始化 x_error 属性
#         self.left_fit = np.array([0, 0, 0])  # 初始化 left_fit
#         self.right_fit = np.array([0, 0, 0])  # 初始化 right_fit
#
#         self.y = self.img_size[1] // 2  # 假设 y 是图像的中间高度
#         self.M_inverse_list = self.M_inverse.flatten()  # 将M_inverse转换为一维数组
#
#         # 初始化人行横道识别相关属性
#         self.crosswalk_detected = False
#         self.stop_flag = True  # 停车标志位，默认 True 表示继续行驶
#         self.crosswalk_distance_threshold = 30  # 人行横道距离阈值（单位：cm）
#
#         #左转道
#
#         self.left_turn_detected = False  # 左变道标志检测标记
#         self.left_lane_completed = False  # 变道完成标记
#
#     # 设置颜色阈值用于左变道标志检测
#         self.blue_lower = np.array([100, 150, 0])  # 蓝色背景HSV下限
#         self.blue_upper = np.array([140, 255, 255])  # 蓝色背景HSV上限
#
#     def show_log(self, message):
#         """打印日志信息"""
#         print(message)
#
#
#     def get_warp_M(self):
#         objdx = 200
#         objdy = 230
#         imgdx = 220
#         imgdy = 250
#         list_pst = [[172, 330], [461, 330], [75, 475], [546, 475]]  #参数修改------
#         pts1 = np.float32(list_pst)
#         pts2 = np.float32([[imgdx, imgdy], [imgdx + objdx, imgdy],
#                            [imgdx, imgdy + objdy], [imgdx + objdx, imgdy + objdy]])
#         self.M = cv2.getPerspectiveTransform(pts1, pts2)
#         self.M_inverse = cv2.getPerspectiveTransform(pts2, pts1)
#
#     def img_preprocess(self):
#         if self.cap is None:
#             return
#         ret, self.img = self.cap.read()
#         if not ret:
#             print("Error: Could not read frame.")
#             return
#         self.img = cv2.medianBlur(self.img, 9)
#         self.warp_img = cv2.warpPerspective(self.img, self.M, self.img_size)
#
#         edges = cv2.Canny(self.warp_img, 50, 40, apertureSize=3)
#         kernel = np.ones((3, 3), np.uint8)
#         edges = cv2.dilate(edges, kernel, iterations=2)
#
#         edges_mask = np.zeros((self.img_size[1], self.img_size[0]), dtype=np.uint8)
#         cv2.rectangle(edges_mask, (160, 0), (480, 480), 255, thickness=cv2.FILLED)
#         self.edges = cv2.bitwise_and(edges, edges, mask=edges_mask)
#
#     def img_HoughLines(self):
#         self.line_points_x.clear()
#         lines = cv2.HoughLines(self.edges, 1, np.pi / 180, threshold=260)
#         if lines is not None:
#             for line in lines:
#                 rho, theta = line[0]
#                 theta_degree = np.degrees(theta)
#                 if theta_degree > 90:
#                     theta_degree = 180 - theta_degree
#                 if np.abs(theta_degree) > 35:
#                     continue
#                 elif np.abs(theta) == 0:
#                     b = rho
#                     self.line_points_x.append(int(b))
#                 else:
#                     m = -1 / np.tan(theta)
#                     b = rho / np.sin(theta)
#                     self.line_points_x.append(int((self.img_size[1] - b) / m))
#
#     def sliding_window_tracking(self):
#         """滑动窗追踪实现"""
#         nonzero = self.edges.nonzero()
#         nonzeroy = np.array(nonzero[0])
#         nonzerox = np.array(nonzero[1])
#
#         left_lane_inds = []
#         right_lane_inds = []
#         last_good_left_inds_len = 0
#         last_good_right_inds_len = 0
#
#         # 初始化连续未检测到的次数计数器
#         if not hasattr(self, 'left_missing_count'):
#             self.left_missing_count = 0
#         if not hasattr(self, 'right_missing_count'):
#             self.right_missing_count = 0
#
#         # 改进的滑动窗追踪，增加了更加精细的动态计算
#         for window in range(self.nwindows):
#             win_y_low = self.img_size[1] - (window + 1) * self.window_height
#             win_y_high = self.img_size[1] - window * self.window_height
#
#             # 左车道
#             if self.state in (State.left, State.center, State.left_to_center) and self.leftx_mean is not None:
#                 win_xleft_low = self.leftx_mean - self.margin
#                 win_xleft_high = self.leftx_mean + self.margin
#                 cv2.rectangle(self.warp_img, (int(win_xleft_low), int(win_y_low)),
#                               (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 2)
#
#                 good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
#                                   (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
#                 left_lane_inds.append(good_left_inds)
#                 last_good_left_inds_len = len(good_left_inds)
#
#                 # 当找到足够的像素点时，更新 self.leftx_mean
#                 if last_good_left_inds_len > self.minpix:
#                     self.leftx_mean = np.int32(np.mean(nonzerox[good_left_inds]))
#
#             # 右车道
#             if self.state in (State.right, State.center, State.right_to_center) and self.rightx_mean is not None:
#                 win_xright_low = self.rightx_mean - self.margin
#                 win_xright_high = self.rightx_mean + self.margin
#                 cv2.rectangle(self.warp_img, (int(win_xright_low), int(win_y_low)),
#                               (int(win_xright_high), int(win_y_high)), (0, 255, 0), 2)
#
#                 good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
#                                    (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
#                 right_lane_inds.append(good_right_inds)
#                 last_good_right_inds_len = len(good_right_inds)
#
#                 # 当找到足够的像素点时，更新 self.rightx_mean
#                 if last_good_right_inds_len > self.minpix:
#                     self.rightx_mean = np.int32(np.mean(nonzerox[good_right_inds]))
#
#             # 单侧车道线的处理逻辑
#             if last_good_left_inds_len < self.minpix and last_good_right_inds_len > self.minpix:
#                 # 增加左侧未检测到车道线的计数
#                 self.left_missing_count += 1
#                 self.right_missing_count = 0  # 重置右侧计数
#
#                 # 如果左侧连续未检测到车道线，才认为仅右侧检测到车道线
#                 if self.left_missing_count >= 2:
#                     self.dynamic_center_x = self.rightx_mean - int(self.img_size[0] * 0.61)  # 动态调整中心点
#
#             elif last_good_right_inds_len < self.minpix and last_good_left_inds_len > self.minpix:
#                 # 增加右侧未检测到车道线的计数
#                 self.right_missing_count += 1
#                 self.left_missing_count = 0  # 重置左侧计数
#
#                 # 如果右侧连续未检测到车道线，才认为仅左侧检测到车道线
#                 if self.right_missing_count >= 2:
#                     self.dynamic_center_x = self.leftx_mean + int(self.img_size[0] * 0.61)
#             else:
#                 # 如果两侧都有足够的像素点，重置计数器
#                 self.left_missing_count = 0
#                 self.right_missing_count = 0
#
#         # 合并窗口内的索引
#         if self.state in (State.left, State.center, State.left_to_center):
#             if len(left_lane_inds) > 0:  # 检查列表是否非空
#                 left_lane_inds = np.concatenate(left_lane_inds)
#                 leftx = nonzerox[left_lane_inds]
#                 lefty = nonzeroy[left_lane_inds]
#
#         if self.state in (State.right, State.center, State.right_to_center):
#             if len(right_lane_inds) > 0:  # 检查列表是否非空
#                 right_lane_inds = np.concatenate(right_lane_inds)
#                 rightx = nonzerox[right_lane_inds]
#                 righty = nonzeroy[right_lane_inds]
#
#         # 车道线拟合
#         if self.state in (State.left, State.left_to_center) and len(left_lane_inds) > 0:
#             self.left_fit = np.polyfit(lefty, leftx, 2)
#
#         elif self.state in (State.right, State.right_to_center) and len(right_lane_inds) > 0:
#             self.right_fit = np.polyfit(righty, rightx, 2)
#
#         elif self.state == State.center and len(left_lane_inds) > 0 and len(right_lane_inds) > 0:
#             self.left_fit = np.polyfit(lefty, leftx, 2)
#             self.right_fit = np.polyfit(righty, rightx, 2)
#
#     # def detect_stop_line(self, serial_driver=None):
#     #     if self.state == State.center and self.error_flag != 1:
#     #         # 检查self.leftx_mean和self.rightx_mean是否为None
#     #         if self.leftx_mean is not None and self.rightx_mean is not None:
#     #             stop_line_detect_leftx = self.leftx_mean - 20
#     #             stop_line_detect_rightx = self.rightx_mean + 20
#     #
#     #             if stop_line_detect_rightx > stop_line_detect_leftx:
#     #                 stop_line_detect_image = self.img[250:480, stop_line_detect_leftx:stop_line_detect_rightx]
#     #                 stop_line_hsv_img = cv2.cvtColor(stop_line_detect_image, cv2.COLOR_BGR2HSV)
#     #
#     #                 yellow_mask = cv2.inRange(stop_line_hsv_img, self.bike_args.yellow_low, self.bike_args.yellow_upper)
#     #                 yellow_lines = cv2.HoughLinesP(yellow_mask, 1, np.pi / 180, 140, 115, 10)
#     #
#     #                 if yellow_lines is not None and len(yellow_lines) > 0:
#     #                     self.show_log("开始停车")
#     #                     self.stop_line_flag = False  # 设置停车线标志位
#     #
#     #                     serial_driver.send_data(self.bike_args.first_stop_line)
#     #
#     #                     self.show_image(yellow_mask, 1)
#     #
#     #                     # 向下位机发送停车信号，等待 30 秒后继续
#     #                 #    self.wait_30_seconds()
#     #                     self.wait_30_seconds(serial_driver)
#     #                     self.serial_send_start_flag(serial_driver)
#     #         else:
#     #             print("检测停止线时，左或右车道线的平均值尚未计算。")
#     #
#     # def wait_30_seconds(self, serial_driver):
#     #     start_time = time.time()
#     #     print("停车线检测到，开始暂停 30 秒")
#     #
#     #     while True:
#     #         elapsed_time = time.time() - start_time
#     #         if elapsed_time >= 30:
#     #             break
#     #         # 检查其他条件，确保程序不会因为等待而退出
#     #         time.sleep(0.1)  # 小的间隔，避免完全阻塞CPU
#     #
#     #     self.stop_line_flag = True  # 重置停车标志位
#     #     print("暂停 30 秒结束，重置停车线标志位")
#     #     self.serial_send_start_flag(serial_driver)
#     #
#     # def serial_send_stop_flag(self, serial_driver=None):
#     #     """发送停车标志到下位机"""
#     #     try:
#     #         stop_line_flag = False      # 停车标志，假设停车标志为0x00
#     #         # 使用 struct.pack 打包停车标志位，这里使用'>B'表示一个字节
#     #         data = struct.pack('>B', stop_line_flag)
#     #         # 调用 serial_driver 发送数据包
#     #         if serial_driver:
#     #             serial_driver.send_packet(0x00, data)
#     #             print("停车标志已发送")
#     #         else:
#     #             print("serial_driver 未初始化")
#     #     except Exception as e:
#     #         print(f"发送停车标志失败: {e}")
#     #
#     # def serial_send_start_flag(self, serial_driver=None):
#     #     """发送启动标志到下位机"""
#     #     try:
#     #         start_flag = True  # 启动标志，假设启动标志为0x01
#     #         # 使用 struct.pack 打包启动标志位，这里使用'>B'表示一个字节
#     #         data = struct.pack('>B', start_flag)
#     #         # 调用 serial_driver 发送数据包
#     #         if serial_driver:
#     #             serial_driver.send_packet(0x01, data)
#     #             print("启动电机标志已发送")
#     #         else:
#     #             print("serial_driver 未初始化")
#     #     except Exception as e:
#     #         print(f"发送启动标志失败: {e}")
#
#     def img_HoughLines_filter(self):
#         self.left_line_x.clear()
#         self.right_line_x.clear()
# #------------------增宽-----------------------
#         # 如果检测到的左右线段数量不足，可以增加筛选宽度
#         min_required_lines = 5  # 假设需要至少5条线段
#         adjusted_shifted_x = self.bike_args.shifted_x
#
#         # 动态调整筛选宽度
#         if len(self.left_line_x) < min_required_lines or len(self.right_line_x) < min_required_lines:
#             adjusted_shifted_x += 10  # 如果不足，增大筛选宽度
#         else:
#             adjusted_shifted_x = self.bike_args.shifted_x  # 否则保持默认宽度
# #-------------------结束------------------------------
#         if len(self.line_points_x) != 0:
#             for point_x in self.line_points_x:
#                 if point_x < self.dynamic_center_x and point_x > (self.dynamic_center_x - self.bike_args.shifted_x):#----------------------调整图像参数--------------------------
#                     self.left_line_x.append(point_x)
#                     cv2.circle(self.warp_img, (point_x, self.img_size[1]), radius=5, color=(255, 255, 255),thickness=-1)
#                 elif point_x > self.dynamic_center_x and point_x < (self.dynamic_center_x + self.bike_args.shifted_x):#----------------------调整图像参数--------------------------
#                     self.right_line_x.append(point_x)
#                     cv2.circle(self.warp_img, (point_x, self.img_size[1]), radius=5, color=(255, 255, 255),thickness=-1)
#
#             if self.state in (State.left, State.left_to_center, State.center):
#                 if len(self.left_line_x) != 0:
#                     self.leftx_mean = int(np.mean(self.left_line_x))
#                 else:
#                     self.leftx_mean = None
#
#             if self.state in (State.right, State.right_to_center, State.center):
#                 if len(self.right_line_x) != 0:
#                     self.rightx_mean = int(np.mean(self.right_line_x))
#                 else:
#                     self.rightx_mean = None
#         else:
#             self.leftx_mean = None
#             self.rightx_mean = None
#     #----------------------巡线----------------------------
#     def img_dynamic_center_task(self):
#         if self.state in (State.right, State.right_to_center):
#             if len(self.rightx_mean_list) < 10:  # 增加样本数以增强稳定性
#                 if self.error_flag != 1 and self.rightx_mean is not None:
#                     self.rightx_mean_list.append(self.rightx_mean)
#             else:
#                 rightx_mean_data = np.array(self.rightx_mean_list[5:]).reshape(-1, 1)
#                 self.kmeans.fit(rightx_mean_data)
#                 labels = self.kmeans.labels_
#                 count = np.bincount(labels)
#
#                 if len(count) == 2:
#                     kmeans_rightx_mean = (
#                         int(np.mean(rightx_mean_data[labels == 0])) if count[0] > count[1]
#                         else int(np.mean(rightx_mean_data[labels == 1]))
#                     )
#                     self.dynamic_center_x = kmeans_rightx_mean - self.to_center_x + 13    # 可调整偏移量以优化精度
#
#                 self.rightx_mean_list.clear()
#
#         elif self.state in (State.left, State.left_to_center):
#             if len(self.leftx_mean_list) < 10:   # 增加样本数以增强稳定性
#                 if self.error_flag != 1 and self.leftx_mean is not None:
#                     self.leftx_mean_list.append(self.leftx_mean)
#             else:
#                 leftx_mean_data = np.array(self.leftx_mean_list[5:]).reshape(-1, 1)
#                 self.kmeans.fit(leftx_mean_data)
#                 labels = self.kmeans.labels_
#                 count = np.bincount(labels)
#
#                 if len(count) == 2:
#                     kmeans_leftx_mean = (
#                         int(np.mean(leftx_mean_data[labels == 0])) if count[0] > count[1]
#                         else int(np.mean(leftx_mean_data[labels == 1]))
#                     )
#                     self.dynamic_center_x = kmeans_leftx_mean + self.to_center_x - 13      # 可调整偏移量以优化精度
#
#                 self.leftx_mean_list.clear()
#         else:
#             # 无左、右车道时将中心点设为图像中点
#             self.dynamic_center_x = self.img_size[ 0] // 2
#
#     def detect_lane(self):
#         self.img_HoughLines()
#         self.img_HoughLines_filter()
#         self.sliding_window_tracking()
#         self.img_dynamic_center_task()
#
#     def detect_blue_cone(self, serial_driver=None):
#         if self.state == State.center and self.error_flag != 1:
#             if self.leftx_mean is not None and self.rightx_mean is not None:  # 确保左右车道线均已计算
#                 block_detect_leftx = self.leftx_mean + 40
#                 block_detect_rightx = self.rightx_mean - 40
#                 if block_detect_rightx > block_detect_leftx:
#                     detect_center = int((block_detect_leftx + block_detect_rightx) / 2)
#                     block_detect_image = self.img[
#                                          self.bike_args.block_detect_y - self.bike_args.block_detect_delta_y:self.bike_args.block_detect_y,
#                                          int(block_detect_leftx):int(block_detect_rightx)
#                                          ]
#                     block_hsv_img = cv2.cvtColor(block_detect_image, cv2.COLOR_BGR2HSV)
#                     blue_mask = cv2.inRange(block_hsv_img, self.bike_args.blue_low, self.bike_args.blue_upper)
#
#                     # 找到蓝色锥桶的轮廓
#                     contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     block_list = []
#                     for item in contours:
#                         x, y, w, h = cv2.boundingRect(item)
#                         if h < self.bike_args.block_h_upper or w < 13:
#                             continue
#                         else:
#                             block_list.append((x, y, w, h))
#
#                     if len(block_list) == 1:
#                         x, y, w, h = block_list[0]
#                         block_meanx = int(x + w / 2)
#                         if block_detect_leftx + block_meanx < detect_center:
#                             block_state = State.left
#                         else:
#                             block_state = State.right
#
#                         if self.block_direction_list[self.skip_block_nums] != State.center:
#                             if block_state != self.block_direction_list[self.skip_block_nums]:
#                                 block_state = self.block_direction_list[self.skip_block_nums]
#
#                         if block_state == self.last_block_state:
#                             self.catch_block_times += 1
#                             if self.catch_block_times >= 3:
#                                 self.shift_to_center_x = int(
#                                     (self.rightx_mean - self.leftx_mean) / 2) - self.bike_args.shifted_x
#                                 self.to_center_x = self.shift_to_center_x + self.bike_args.shifted_x
#
#                                 # 根据障碍物位置调整避障线偏移
#                                 if abs(block_detect_leftx + block_meanx - detect_center) < 10:
#                                     self.bike_args.shifted_x = 22
#                                 else:
#                                     self.bike_args.shifted_x = 38
#
#                                 # 根据障碍物方向调整状态
#                                 if block_state == State.left:
#                                     self.state = State.right
#                                 elif block_state == State.right:
#                                     self.state = State.left
#
#                                 # 发送数据
#                                 serial_driver.send_data(self.bike_args.wait_back_center_data)
#                                 self.wait_back_center_flag = 1
#                                 self.catch_block_times = 0
#                                 self.show_log("障碍物方向:{0}\nto_center_x:{1}".format(block_state.name, str(self.to_center_x)))
#                                 self.show_image(blue_mask, 1)
#                         else:
#                             self.catch_block_times = 0
#                             self.last_block_state = block_state
#             else:
#                 print("左右车道线的平均值尚未计算，无法进行锥桶检测。")
#                 return
#
#     def detect_block(self):
#         # 通过颜色阈值检测障碍物（锥桶）
#         img = self.img
#         if img is None:
#             return False
#
#         hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#         mask = cv2.inRange(hsv, self.bike_args.blue_low, self.bike_args.blue_upper)
#         contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#
#         block_found = False
#         for contour in contours:
#             if cv2.contourArea(contour) > self.bike_args.block_h_upper:
#                 M = cv2.moments(contour)
#                 if M["m00"] > 0:
#                     cX = int(M["m10"] / M["m00"])
#                     cY = int(M["m01"] / M["m00"])
#
#                     if cY < self.bike_args.block_detect_y + self.bike_args.block_detect_delta_y:
#                         block_found = True
#                         self.catch_block_times += 1
#                         if cX < self.dynamic_center_x - 50:  # 左侧锥桶
#                             self.state = State.left
#                         elif cX > self.dynamic_center_x + 50:  # 右侧锥桶
#                             self.state = State.right
#                         break
#
#         # 检测到障碍物
#         if block_found:
#             self.last_block_state = self.state
#             self.skip_block_nums = 0
#             self.wait_back_center_flag = 0  # 如果在障碍物状态中，则清除回归状态
#         else:
#             if self.last_block_state != State.center:
#                 self.block_direction_list.append(self.last_block_state)
#                 if len(self.block_direction_list) > 5:
#                     self.block_direction_list.pop(0)
#                 if self.block_direction_list.count(self.block_direction_list[0]) == len(self.block_direction_list):
#                     self.skip_block_nums += 1
#                 else:
#                     self.skip_block_nums = 0
#
#                 if self.skip_block_nums >= 5:  # 经过5帧未检测到障碍物
#                     if self.last_block_state == State.left:
#                         self.state = State.left_to_center
#                     elif self.last_block_state == State.right:
#                         self.state = State.right_to_center
#                     self.last_block_state = State.center
#
#     def update_state_after_obstacle(self):
#         if self.state == State.left_to_center and self.dynamic_center_x >= 200:
#             self.state = State.center
#             self.dynamic_center_x = 320
#             self.skip_frame_times = self.initial_skip_frame_times  # 重新设置帧数
#
#         elif self.state == State.right_to_center and self.dynamic_center_x <= 440:
#             self.state = State.center
#             self.dynamic_center_x = 320
#             self.skip_frame_times = self.initial_skip_frame_times  # 重新设置帧数
#
#     def point_reverse_perspective(self, point):
#         x, y = point
#         denom = self.M_inverse_list[6] * x + self.M_inverse_list[7] * y + 1
#         if denom == 0:
#             raise ValueError("Denominator is zero, invalid transformation.")
#         x_transformed = (self.M_inverse_list[0] * x + self.M_inverse_list[1] * y + self.M_inverse_list[2]) / denom
#         y_transformed = (self.M_inverse_list[3] * x + self.M_inverse_list[4] * y + self.M_inverse_list[5]) / denom
#         return (int(x_transformed), int(y_transformed))
#
#     def img_get_error_x(self):
#         """计算图像中心与检测线路的偏差值"""
#         global e
#         if self.error_flag != 1:
#             try:
#                 if np.all(self.left_fit == 0) and np.all(self.right_fit == 0):
#                     # 如果左右车道线均未计算，则将 x_error 设置为 0
#                     self.x_error = -1
#                     print("车道线拟合尚未计算，x_error 设置为 -1")
#                 else:
#                     if self.state == State.left or self.state == State.left_to_center:
#                         shifted_left_fit = np.copy(self.left_fit)
#                         shifted_left_fit[2] = shifted_left_fit[2] + self.bike_args.shifted_x + self.dynamic_shift_x
#                         self.x_mean = shifted_left_fit[0] * self.y ** 2 + shifted_left_fit[1] * self.y + \
#                                       shifted_left_fit[2]
#                     elif self.state == State.right or self.state == State.right_to_center:
#                         shifted_right_fit = np.copy(self.right_fit)
#                         shifted_right_fit[2] = shifted_right_fit[2] - self.bike_args.shifted_x - self.dynamic_shift_x
#                         self.x_mean = shifted_right_fit[0] * self.y ** 2 + shifted_right_fit[1] * self.y + \
#                                       shifted_right_fit[2]
#                     else:
#                         mid_fit = (self.left_fit + self.right_fit) / 2
#                         self.x_mean = mid_fit[0] * self.y ** 2 + mid_fit[1] * self.y + mid_fit[2]
#
#                     print(f"x_mean calculated: {self.x_mean}")
#                     self.catch_point_source = self.point_reverse_perspective((self.x_mean, self.y))
#                     self.x_error = self.x_mean - 320
#                     if self.last_x_error is not None:
#                         self.x_error = int(self.last_x_error * 0.3 + self.x_error * 0.7)
#                     self.last_x_error = self.x_error
#
#             except Exception as e:
#                 print(f"Error in img_get_error_x: {e}")
#                 self.error_flag = 1
#
#     def serial_send_x(self):
#         """发送误差数据到下位机"""
#         if self.error_flag != 1:
#             try:
#                 type = 0x00  # 数据类型
#                 data = struct.pack('>h', self.x_error)  # 打包数据
#                 self.comm.send_packet(type, data)  # 调用这个发
#             except Exception as e:
#                 print(f"串口发送失败: {e}")
#                 import traceback
#                 traceback.print_exc()  # 输出详细的错误信息和堆栈
#
#     def detect_lane_and_send(self, serial_driver=None):
#         self.detect_lane()
#         self.detect_blue_cone()  # 添加锥桶检测
# #        self.detect_stop_line(serial_driver)  # 调用停车线检测
# #        self.serial_send_stop_flag(serial_driver)  # 发送停车标志
#         self.img_get_error_x()  # 计算 x_error
#         self.serial_send_x()  # 发送误差数据
#         if self.x_error is not None:
#             print(f"x_error: {self.x_error}")
#
#     def release_resources(self):
#         if self.cap:
#             self.cap.release()
#         cv2.destroyAllWindows()
#     # ====================== 人行横道识别功能开始 ======================
#     def detect_crosswalk(self, image):
#         """
#         检测人行横道，检测到后设置crosswalk_detected为True。
#         """
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         # 定义白色的HSV范围，用于识别人行横道
#         white_lower = np.array([0, 0, 200])
#         white_upper = np.array([180, 30, 255])
#
#         # 生成掩码，识别白色矩形框
#         white_mask = cv2.inRange(hsv_image, white_lower, white_upper)
#         contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
#         crosswalk_detected = False
#         crosswalk_rectangles = 0  # 计数检测到的矩形框数量
#         last_x = None  # 在此处初始化last_x
#         for contour in contours:
#             x, y, w, h = cv2.boundingRect(contour)
#             # 检测矩形框的尺寸和相邻间距，以符合人行横道的规格
#             if 25 < w < 35 and 8 < h < 12:  # 符合30cm x 10cm规格
#                 crosswalk_rectangles += 1
#                 # 判断多个矩形框是否间距合理
#                 if last_x is not None and 8 < abs(last_x - x) < 12:
#                     crosswalk_detected = True
#                 last_x = x
#
#         self.crosswalk_detected = crosswalk_detected
#         return white_mask if crosswalk_detected else None
#
#     def check_distance_and_stop(self, distance_to_crosswalk):
#         """
#         如果距离人行横道小于30cm，触发停车逻辑，向下位机发送停车标志。
#         """
#         if distance_to_crosswalk < self.crosswalk_distance_threshold:
#             print("检测到人行横道，距离小于30cm，开始停车")
#
#             # 设置停车标志位
#             self.stop_flag = False
#             self.send_stop_flag()
#
#             # 暂停10秒
#             self.wait_10_seconds()
#
#             # 停止标志位复位，并发送启动信号
#             self.stop_flag = True
#             self.send_start_flag()
#
#     def wait_10_seconds(self):
#         """模拟10秒等待"""
#         print("开始暂停10秒")
#         start_time = time.time()
#         while True:
#             elapsed_time = time.time() - start_time
#             if elapsed_time >= 10:
#                 break
#             # 确保在等待期间可以响应其他操作
#             time.sleep(0.1)  # 设置较小的时间间隔，避免完全阻塞CPU
#         print("暂停10秒结束")
#
#     def send_stop_flag(self):
#         """发送停车标志到下位机"""
#         try:
#             stop_flag = False  # 停车标志位为0x00
#             data = struct.pack('>B', stop_flag)
#             if self.serial_driver:
#                 self.serial_driver.send_packet(0x00, data)
#                 print("停车标志已发送到下位机")
#             else:
#                 print("serial_driver未初始化")
#         except Exception as e:
#             print(f"发送停车标志失败: {e}")
#
#     def send_start_flag(self):
#         """发送启动标志到下位机"""
#         try:
#             start_flag = True  # 启动标志位为0x01
#             data = struct.pack('>B', start_flag)
#             if self.serial_driver:
#                 self.serial_driver.send_packet(0x01, data)
#                 print("启动电机标志已发送到下位机")
#             else:
#                 print("serial_driver未初始化")
#         except Exception as e:
#             print(f"发送启动标志失败: {e}")
#
#     def process_frame(self, image, distance_to_crosswalk):
#         """
#         在给定图像帧中检测人行横道，如果检测到并距离小于阈值则停车。
#         """
#         crosswalk_mask = self.detect_crosswalk(image)
#         if crosswalk_mask is not None:
#             self.check_distance_and_stop(distance_to_crosswalk)
#             # 可视化人行横道检测结果
#             cv2.imshow("Crosswalk Detection", crosswalk_mask)
#         """
#                主循环中每帧的处理函数
#                """
#         if self.cap.isOpened():
#             ret, frame = self.cap.read()
#             if not ret:
#                 print("无法读取摄像头帧")
#                 return
#                 # 如果车道线还没有计算出误差，直接跳过误差计算
#             if np.all(self.left_fit == 0) and np.all(self.right_fit == 0):
#                     self.x_error = 0
#                     print("车道线还没有计算，x_error设置为0")
#
#             # 检测左变道标志
#             if self.detect_left_turn_sign(frame):
#                 self.execute_lane_change()
#             else:
#                 self.update_state_after_lane_change()
#
#             # 显示检测结果
#             cv2.imshow("Frame", frame)
#     # ====================== 左转弯 ======================
#     def detect_left_turn_sign(self, image):
#         """
#         检测左变道标志。
#         """
#         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#         blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
#
#         # 查找蓝色区域中的箭头形状
#         contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         for contour in contours:
#             area = cv2.contourArea(contour)
#             if area > 1000:  # 根据实际标志大小调整面积阈值
#                 x, y, w, h = cv2.boundingRect(contour)
#                 # 根据标志的宽高比和尺寸进行判断
#                 if 0.7 < h / w < 1.2 and 20 < w < 40 and 30 < h < 50:
#                     self.left_turn_detected = True
#                     cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
#                     print("检测到左变道标志！")
#                     return True
#         self.left_turn_detected = False
#         return False
#
#     def execute_lane_change(self):
#         """
#         执行左变道操作并切换状态。
#         """
#         if self.left_turn_detected and self.state == "center":
#             print("执行左变道操作")
#             self.state = "left_lane"  # 切换为左变道状态
#             self.send_left_turn_signal()  # 发送左转指令
#
#     def update_state_after_lane_change(self):
#         """
#         完成左变道后，将状态恢复到中心状态。
#         """
#         if self.state == "left_lane" and not self.left_turn_detected:
#             print("变道完成，恢复到中心状态")
#             self.state = "center"
#             self.left_lane_completed = True  # 记录已完成变道
#             self.send_straight_signal()  # 发送直行指令
#
#     def send_left_turn_signal(self):
#         """发送左转信号给下位机，x_error 为负值"""
#         type = 0x01  # 数据类型：假设0x01表示左转
#         x_error = -10  # 固定的左转误差值，可以根据实际需求调整
#         data = struct.pack('>h', x_error)  # 打包数据
#         self.comm.send_packet(type, data)  # 发送左转命令
#         print("发送左转指令，x_error:", x_error)
#
#     def send_straight_signal(self):
#         """发送直行信号给下位机，x_error 为0"""
#         type = 0x00  # 数据类型：假设0x00表示直行
#         x_error = 0  # 变道完成后，回正
#         data = struct.pack('>h', x_error)  # 打包数据
#         self.comm.send_packet(type, data)  # 发送直行命令
#         print("发送直行指令，x_error:", x_error)
#
# if __name__ == '__main__':
#     bike = BikeClass()
#
#     if bike.cap:  # 确保 VideoCapture 已打开成功
#         while True:
#             bike.img_preprocess()
#             bike.detect_lane_and_send()
#             bike.process_frame(bike.img, distance_to_crosswalk=25)  # 假设25cm为检测到人行横道的距离
#
#         #    cv2.imshow('img', bike.img)
#
#             cv2.imshow('edges', bike.edges)
#             cv2.imshow('warp_img', bike.warp_img)
#
#
#             # 按 'q' 键退出循环
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#
#     bike.release_resources()
#---------------------------------------------------------------更正-----------------------------------------------------------
#------------------------引入停车线和斑马线和转向线并向下位机发送数据--------------------------------------
#--------------------------------更正滑动弹窗和巡线中点贴合赛道线-------------------------------------------
import time

from self import self
from sklearn.cluster import KMeans
import numpy as np
import cv2
import struct  # 用于数据打包
from enum import Enum
from SerialComm import SerialComm

class State(Enum):
    left = 1
    right = 2
    center = 3
    left_to_center = 4
    right_to_center = 5


class BikeClass:
    def __init__(self):
    #    self.comm = SerialComm(port="/dev/ttyUSB0", baudrate=115200, timeout=1)
    #    self.comm = SerialComm(port="COM3", baudrate=115200, timeout=1)

        self.M = None
        self.M_inverse = None
        self.get_warp_M()

        self.img_size = (640, 480)
        self.img = None
        self.warp_img = None
        self.edges = None
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open video device.")
            self.cap = None  # 确保无法访问无效的 VideoCapture 对象

        self.state = State.center
        self.margin = 35          #滑动弹窗的宽度可调整初始值35可调整
        self.minpix = 25
        self.nwindows = 10       #滑动窗口的窗口数
        self.window_height = np.int32(self.img_size[1] / self.nwindows)

        self.left_fit = None  # 初始化为 None
        self.right_fit = None  # 初始化为 None

        self.leftx_mean = None
        self.rightx_mean = None
        self.line_points_x = []
        self.left_line_x = []
        self.right_line_x = []

        self.dynamic_center_x = self.img_size[0] // 2
        self.to_center_x = 38         #偏移量可微调减小降低右侧敏感度，增大降低左侧敏感度 38初始值
        self.error_flag = 0
        self.last_x_error = None

        self.rightx_mean_list = []
        self.leftx_mean_list = []
        self.kmeans = KMeans(n_clusters=2)

        self.bike_args = type('', (), {})()
        self.bike_args.shifted_x = 150
        self.bike_args.dynamic_shift_x = 0
        self.bike_args.block_detect_y = 300  # 摄像头检测障碍物的前瞻距离
        self.bike_args.block_detect_delta_y = 150  # 检测区域高度
        self.bike_args.blue_low = np.array([100, 150, 0])  # 蓝色下限
        self.bike_args.blue_upper = np.array([140, 255, 255])  # 蓝色上限
        self.bike_args.yellow_low = np.array([20, 100, 100])  # 黄色下限
        self.bike_args.yellow_upper = np.array([30, 255, 255])  # 黄色上限
        self.bike_args.block_h_upper = 700  # 锥桶高度上限

        self.catch_block_times = 0
        self.last_block_state = State.center
        self.block_direction_list = [State.center] * 5  # 记录最近的状态
        self.skip_block_nums = 0  # 跳过的锥桶数量


        self.wait_back_center_flag = 0
        self.skip_frame_times = 0
        self.initial_skip_frame_times = 10  # 设置回归中线的帧数

        self.x_error = None  # 初始化 x_error 属性
        self.left_fit = np.array([0, 0, 0])  # 初始化 left_fit
        self.right_fit = np.array([0, 0, 0])  # 初始化 right_fit

        self.y = self.img_size[1] // 2  # 假设 y 是图像的中间高度
        self.M_inverse_list = self.M_inverse.flatten()  # 将M_inverse转换为一维数组

        # 初始化人行横道识别相关属性
        self.crosswalk_detected = False
        self.stop_flag = True  # 停车标志位，默认 True 表示继续行驶
        self.crosswalk_distance_threshold = 30  # 人行横道距离阈值（单位：cm）

        #左转道

        self.left_turn_detected = False  # 左变道标志检测标记
        self.left_lane_completed = False  # 变道完成标记

    # 设置颜色阈值用于左变道标志检测
        self.blue_lower = np.array([100, 150, 0])  # 蓝色背景HSV下限
        self.blue_upper = np.array([140, 255, 255])  # 蓝色背景HSV上限

    def show_log(self, message):
        """打印日志信息"""
        print(message)


    def get_warp_M(self):
        objdx = 200
        objdy = 230
        imgdx = 220
        imgdy = 250
        list_pst = [[172, 330], [461, 330], [75, 475], [546, 475]]  #参数修改------
        pts1 = np.float32(list_pst)
        pts2 = np.float32([[imgdx, imgdy], [imgdx + objdx, imgdy],
                           [imgdx, imgdy + objdy], [imgdx + objdx, imgdy + objdy]])
        self.M = cv2.getPerspectiveTransform(pts1, pts2)
        self.M_inverse = cv2.getPerspectiveTransform(pts2, pts1)

    def img_preprocess(self):
        if self.cap is None:
            return
        ret, self.img = self.cap.read()
        if not ret:
            print("Error: Could not read frame.")
            return
        self.img = cv2.medianBlur(self.img, 9)
        self.warp_img = cv2.warpPerspective(self.img, self.M, self.img_size)

        edges = cv2.Canny(self.warp_img, 50, 40, apertureSize=3)
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=2)

        edges_mask = np.zeros((self.img_size[1], self.img_size[0]), dtype=np.uint8)
        cv2.rectangle(edges_mask, (160, 0), (480, 480), 255, thickness=cv2.FILLED)
        self.edges = cv2.bitwise_and(edges, edges, mask=edges_mask)

    def img_HoughLines(self):
        self.line_points_x.clear()
        lines = cv2.HoughLines(self.edges, 1, np.pi / 180, threshold=260)
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                theta_degree = np.degrees(theta)
                if theta_degree > 90:
                    theta_degree = 180 - theta_degree
                if np.abs(theta_degree) > 35:
                    continue
                elif np.abs(theta) == 0:
                    b = rho
                    self.line_points_x.append(int(b))
                else:
                    m = -1 / np.tan(theta)
                    b = rho / np.sin(theta)
                    self.line_points_x.append(int((self.img_size[1] - b) / m))

    def sliding_window_tracking(self):
        """滑动窗追踪实现"""
        nonzero = self.edges.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane_inds = []
        right_lane_inds = []
        last_good_left_inds_len = 0
        last_good_right_inds_len = 0

        # 初始化连续未检测到的次数计数器
        if not hasattr(self, 'left_missing_count'):
            self.left_missing_count = 0
        if not hasattr(self, 'right_missing_count'):
            self.right_missing_count = 0

        # 改进的滑动窗追踪，增加了更加精细的动态计算
        for window in range(self.nwindows):
            win_y_low = self.img_size[1] - (window + 1) * self.window_height
            win_y_high = self.img_size[1] - window * self.window_height

            # 左车道
            if self.state in (State.left, State.center, State.left_to_center) and self.leftx_mean is not None:
                win_xleft_low = self.leftx_mean - self.margin
                win_xleft_high = self.leftx_mean + self.margin
                cv2.rectangle(self.warp_img, (int(win_xleft_low), int(win_y_low)),
                              (int(win_xleft_high), int(win_y_high)), (0, 255, 0), 2)

                good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                  (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                left_lane_inds.append(good_left_inds)
                last_good_left_inds_len = len(good_left_inds)

                # 当找到足够的像素点时，更新 self.leftx_mean
                if last_good_left_inds_len > self.minpix:
                    self.leftx_mean = np.int32(np.mean(nonzerox[good_left_inds]))

            # 右车道
            if self.state in (State.right, State.center, State.right_to_center) and self.rightx_mean is not None:
                win_xright_low = self.rightx_mean - self.margin
                win_xright_high = self.rightx_mean + self.margin
                cv2.rectangle(self.warp_img, (int(win_xright_low), int(win_y_low)),
                              (int(win_xright_high), int(win_y_high)), (0, 255, 0), 2)

                good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                                   (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
                right_lane_inds.append(good_right_inds)
                last_good_right_inds_len = len(good_right_inds)

                # 当找到足够的像素点时，更新 self.rightx_mean
                if last_good_right_inds_len > self.minpix:
                    self.rightx_mean = np.int32(np.mean(nonzerox[good_right_inds]))

            # 单侧车道线的处理逻辑
            if last_good_left_inds_len < self.minpix and last_good_right_inds_len > self.minpix:
                # 增加左侧未检测到车道线的计数
                self.left_missing_count += 1
                self.right_missing_count = 0  # 重置右侧计数

                # 如果左侧连续未检测到车道线，才认为仅右侧检测到车道线
                if self.left_missing_count >= 2:
                    self.dynamic_center_x = self.rightx_mean - int(self.img_size[0] * 0.61)  # 动态调整中心点

            elif last_good_right_inds_len < self.minpix and last_good_left_inds_len > self.minpix:
                # 增加右侧未检测到车道线的计数
                self.right_missing_count += 1
                self.left_missing_count = 0  # 重置左侧计数

                # 如果右侧连续未检测到车道线，才认为仅左侧检测到车道线
                if self.right_missing_count >= 2:
                    self.dynamic_center_x = self.leftx_mean + int(self.img_size[0] * 0.61)
            else:
                # 如果两侧都有足够的像素点，重置计数器
                self.left_missing_count = 0
                self.right_missing_count = 0

        # 合并窗口内的索引
        if self.state in (State.left, State.center, State.left_to_center):
            if len(left_lane_inds) > 0:  # 检查列表是否非空
                left_lane_inds = np.concatenate(left_lane_inds)
                leftx = nonzerox[left_lane_inds]
                lefty = nonzeroy[left_lane_inds]

        if self.state in (State.right, State.center, State.right_to_center):
            if len(right_lane_inds) > 0:  # 检查列表是否非空
                right_lane_inds = np.concatenate(right_lane_inds)
                rightx = nonzerox[right_lane_inds]
                righty = nonzeroy[right_lane_inds]

        # 车道线拟合
        if self.state in (State.left, State.left_to_center) and len(left_lane_inds) > 0:
            self.left_fit = np.polyfit(lefty, leftx, 2)

        elif self.state in (State.right, State.right_to_center) and len(right_lane_inds) > 0:
            self.right_fit = np.polyfit(righty, rightx, 2)

        elif self.state == State.center and len(left_lane_inds) > 0 and len(right_lane_inds) > 0:
            self.left_fit = np.polyfit(lefty, leftx, 2)
            self.right_fit = np.polyfit(righty, rightx, 2)

    def img_HoughLines_filter(self):
        self.left_line_x.clear()
        self.right_line_x.clear()
#------------------增宽-----------------------
        # 如果检测到的左右线段数量不足，可以增加筛选宽度
        min_required_lines = 5  # 假设需要至少5条线段
        adjusted_shifted_x = self.bike_args.shifted_x

        # 动态调整筛选宽度
        if len(self.left_line_x) < min_required_lines or len(self.right_line_x) < min_required_lines:
            adjusted_shifted_x += 10  # 如果不足，增大筛选宽度
        else:
            adjusted_shifted_x = self.bike_args.shifted_x  # 否则保持默认宽度
#-------------------结束------------------------------
        if len(self.line_points_x) != 0:
            for point_x in self.line_points_x:
                if point_x < self.dynamic_center_x and point_x > (self.dynamic_center_x - self.bike_args.shifted_x):#----------------------调整图像参数--------------------------
                    self.left_line_x.append(point_x)
                    cv2.circle(self.warp_img, (point_x, self.img_size[1]), radius=5, color=(255, 255, 255),thickness=-1)
                elif point_x > self.dynamic_center_x and point_x < (self.dynamic_center_x + self.bike_args.shifted_x):#----------------------调整图像参数--------------------------
                    self.right_line_x.append(point_x)
                    cv2.circle(self.warp_img, (point_x, self.img_size[1]), radius=5, color=(255, 255, 255),thickness=-1)

            if self.state in (State.left, State.left_to_center, State.center):
                if len(self.left_line_x) != 0:
                    self.leftx_mean = int(np.mean(self.left_line_x))
                else:
                    self.leftx_mean = None

            if self.state in (State.right, State.right_to_center, State.center):
                if len(self.right_line_x) != 0:
                    self.rightx_mean = int(np.mean(self.right_line_x))
                else:
                    self.rightx_mean = None
        else:
            self.leftx_mean = None
            self.rightx_mean = None
    #----------------------巡线----------------------------
    def img_dynamic_center_task(self):
        if self.state in (State.right, State.right_to_center):
            if len(self.rightx_mean_list) < 10:  # 增加样本数以增强稳定性
                if self.error_flag != 1 and self.rightx_mean is not None:
                    self.rightx_mean_list.append(self.rightx_mean)
            else:
                rightx_mean_data = np.array(self.rightx_mean_list[5:]).reshape(-1, 1)
                self.kmeans.fit(rightx_mean_data)
                labels = self.kmeans.labels_
                count = np.bincount(labels)

                if len(count) == 2:
                    kmeans_rightx_mean = (
                        int(np.mean(rightx_mean_data[labels == 0])) if count[0] > count[1]
                        else int(np.mean(rightx_mean_data[labels == 1]))
                    )
                    self.dynamic_center_x = kmeans_rightx_mean - self.to_center_x + 13    # 可调整偏移量以优化精度

                self.rightx_mean_list.clear()

        elif self.state in (State.left, State.left_to_center):
            if len(self.leftx_mean_list) < 10:   # 增加样本数以增强稳定性
                if self.error_flag != 1 and self.leftx_mean is not None:
                    self.leftx_mean_list.append(self.leftx_mean)
            else:
                leftx_mean_data = np.array(self.leftx_mean_list[5:]).reshape(-1, 1)
                self.kmeans.fit(leftx_mean_data)
                labels = self.kmeans.labels_
                count = np.bincount(labels)

                if len(count) == 2:
                    kmeans_leftx_mean = (
                        int(np.mean(leftx_mean_data[labels == 0])) if count[0] > count[1]
                        else int(np.mean(leftx_mean_data[labels == 1]))
                    )
                    self.dynamic_center_x = kmeans_leftx_mean + self.to_center_x - 13      # 可调整偏移量以优化精度

                self.leftx_mean_list.clear()
        else:
            # 无左、右车道时将中心点设为图像中点
            self.dynamic_center_x = self.img_size[ 0] // 2

    def detect_lane(self):
        self.img_HoughLines()
        self.img_HoughLines_filter()
        self.sliding_window_tracking()
        self.img_dynamic_center_task()

    def detect_blue_cone(self, serial_driver=None):
        if self.state == State.center and self.error_flag != 1:
            # 确保左右车道线均已计算
            if self.leftx_mean is not None and self.rightx_mean is not None:
                block_detect_leftx = self.leftx_mean + 40
                block_detect_rightx = self.rightx_mean - 40

                # 验证左右边界是否合理
                if block_detect_rightx > block_detect_leftx:
                    detect_center = int((block_detect_leftx + block_detect_rightx) / 2)

                    # 截取检测区域图像
                    block_detect_image = self.img[
                                         self.bike_args.block_detect_y - self.bike_args.block_detect_delta_y:self.bike_args.block_detect_y,
                                         int(block_detect_leftx):int(block_detect_rightx)
                                         ]
                    block_hsv_img = cv2.cvtColor(block_detect_image, cv2.COLOR_BGR2HSV)

                    # 应用蓝色掩码并进行噪声过滤
                    blue_mask = cv2.inRange(block_hsv_img, self.bike_args.blue_low, self.bike_args.blue_upper)
                    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

                    # 找到蓝色锥桶的轮廓
                    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    block_list = []

                    # 过滤轮廓并按长宽比识别锥桶
                    for item in contours:
                        x, y, w, h = cv2.boundingRect(item)

                        # 确保轮廓符合预期尺寸和形状
                        if h < self.bike_args.block_h_upper or w < 13:
                            continue
                        if 100 < cv2.contourArea(item) < 5000 and 0.7 < h / w < 1.2:  # 假设锥桶的长宽比接近1
                            block_list.append((x, y, w, h))

                    # 检测到单个锥桶，确定其方向
                    if len(block_list) == 1:
                        x, y, w, h = block_list[0]
                        block_meanx = int(x + w / 2)
                        block_state = State.left if (block_detect_leftx + block_meanx < detect_center) else State.right

                        # 根据预设方向更新障碍物状态
                        if self.block_direction_list[self.skip_block_nums] != State.center:
                            if block_state != self.block_direction_list[self.skip_block_nums]:
                                block_state = self.block_direction_list[self.skip_block_nums]

                        # 如果检测结果与上次一致，累计捕获次数
                        if block_state == self.last_block_state:
                            self.catch_block_times += 1
                            if self.catch_block_times >= 3:
                                # 根据障碍物位置调整避障线偏移
                                self.shift_to_center_x = int(
                                    (self.rightx_mean - self.leftx_mean) / 2) - self.bike_args.shifted_x
                                self.to_center_x = self.shift_to_center_x + self.bike_args.shifted_x

                                if abs(block_detect_leftx + block_meanx - detect_center) < 10:
                                    self.bike_args.shifted_x = 22
                                else:
                                    self.bike_args.shifted_x = 38

                                # 更新状态并发送数据
                                self.state = State.right if block_state == State.left else State.left
                                serial_driver.send_data(self.bike_args.wait_back_center_data)
                                self.wait_back_center_flag = 1
                                self.catch_block_times = 0
                                self.show_log(
                                    "障碍物方向:{0}\nto_center_x:{1}".format(block_state.name, str(self.to_center_x)))
                                self.show_image(blue_mask, 1)
                        else:
                            # 如果方向不同，重置捕获次数
                            self.catch_block_times = 0
                            self.last_block_state = block_state
                else:
                    print("左右边界值不合理，无法进行锥桶检测。")
                    return
            else:
                print("左右车道线的平均值尚未计算，无法进行锥桶检测。")
                return

    def detect_block(self):
        # 通过颜色阈值检测障碍物（锥桶）
        img = self.img
        if img is None:
            return False

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.bike_args.blue_low, self.bike_args.blue_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        block_found = False
        for contour in contours:
            if cv2.contourArea(contour) > self.bike_args.block_h_upper:
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    if cY < self.bike_args.block_detect_y + self.bike_args.block_detect_delta_y:
                        block_found = True
                        self.catch_block_times += 1
                        if cX < self.dynamic_center_x - 50:  # 左侧锥桶
                            self.state = State.left
                        elif cX > self.dynamic_center_x + 50:  # 右侧锥桶
                            self.state = State.right
                        break

        # 检测到障碍物
        if block_found:
            self.last_block_state = self.state
            self.skip_block_nums = 0
            self.wait_back_center_flag = 0  # 如果在障碍物状态中，则清除回归状态
        else:
            if self.last_block_state != State.center:
                self.block_direction_list.append(self.last_block_state)
                if len(self.block_direction_list) > 5:
                    self.block_direction_list.pop(0)
                if self.block_direction_list.count(self.block_direction_list[0]) == len(self.block_direction_list):
                    self.skip_block_nums += 1
                else:
                    self.skip_block_nums = 0

                if self.skip_block_nums >= 5:  # 经过5帧未检测到障碍物
                    if self.last_block_state == State.left:
                        self.state = State.left_to_center
                    elif self.last_block_state == State.right:
                        self.state = State.right_to_center
                    self.last_block_state = State.center

    def update_state_after_obstacle(self):
        if self.state == State.left_to_center and self.dynamic_center_x >= 200:
            self.state = State.center
            self.dynamic_center_x = 320
            self.skip_frame_times = self.initial_skip_frame_times  # 重新设置帧数

        elif self.state == State.right_to_center and self.dynamic_center_x <= 440:
            self.state = State.center
            self.dynamic_center_x = 320
            self.skip_frame_times = self.initial_skip_frame_times  # 重新设置帧数

    def point_reverse_perspective(self, point):
        x, y = point
        denom = self.M_inverse_list[6] * x + self.M_inverse_list[7] * y + 1
        if denom == 0:
            raise ValueError("Denominator is zero, invalid transformation.")
        x_transformed = (self.M_inverse_list[0] * x + self.M_inverse_list[1] * y + self.M_inverse_list[2]) / denom
        y_transformed = (self.M_inverse_list[3] * x + self.M_inverse_list[4] * y + self.M_inverse_list[5]) / denom
        return (int(x_transformed), int(y_transformed))

    def img_get_error_x(self):
        """计算图像中心与检测线路的偏差值"""
        global e
        if self.error_flag != 1:
            try:
                if np.all(self.left_fit == 0) and np.all(self.right_fit == 0):
                    # 如果左右车道线均未计算，则将 x_error 设置为 0
                    self.x_error = -1
                    print("车道线拟合尚未计算，x_error 设置为 -1")
                else:
                    if self.state == State.left or self.state == State.left_to_center:
                        shifted_left_fit = np.copy(self.left_fit)
                        shifted_left_fit[2] = shifted_left_fit[2] + self.bike_args.shifted_x + self.dynamic_shift_x
                        self.x_mean = shifted_left_fit[0] * self.y ** 2 + shifted_left_fit[1] * self.y + \
                                      shifted_left_fit[2]
                    elif self.state == State.right or self.state == State.right_to_center:
                        shifted_right_fit = np.copy(self.right_fit)
                        shifted_right_fit[2] = shifted_right_fit[2] - self.bike_args.shifted_x - self.dynamic_shift_x
                        self.x_mean = shifted_right_fit[0] * self.y ** 2 + shifted_right_fit[1] * self.y + \
                                      shifted_right_fit[2]
                    else:
                        mid_fit = (self.left_fit + self.right_fit) / 2
                        self.x_mean = mid_fit[0] * self.y ** 2 + mid_fit[1] * self.y + mid_fit[2]

                    print(f"x_mean calculated: {self.x_mean}")
                    self.catch_point_source = self.point_reverse_perspective((self.x_mean, self.y))
                    self.x_error = self.x_mean - 320
                    if self.last_x_error is not None:
                        self.x_error = int(self.last_x_error * 0.3 + self.x_error * 0.7)
                    self.last_x_error = self.x_error

            except Exception as e:
                print(f"Error in img_get_error_x: {e}")
                self.error_flag = 1

    def serial_send_x(self):
        """发送误差数据到下位机"""
        if self.error_flag != 1:
            try:
                type = 0x00  # 数据类型
                data = struct.pack('>h', self.x_error)  # 打包数据
                self.comm.send_packet(type, data)  # 调用这个发
            except Exception as e:
                print(f"串口发送失败: {e}")
                import traceback
                traceback.print_exc()  # 输出详细的错误信息和堆栈

    def detect_lane_and_send(self, serial_driver=None):
        self.detect_lane()
        self.detect_blue_cone()  # 添加锥桶检测
        self.img_get_error_x()  # 计算 x_error
        self.serial_send_x()  # 发送误差数据
        if self.x_error is not None:
            print(f"x_error: {self.x_error}")

    def release_resources(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
    # ====================== 人行横道识别功能开始 ======================
    def detect_crosswalk(self, image):
        """
        检测人行横道，检测到后设置crosswalk_detected为True。
        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 定义白色的HSV范围，用于识别人行横道
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])

        # 生成掩码，识别白色矩形框
        white_mask = cv2.inRange(hsv_image, white_lower, white_upper)
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        crosswalk_detected = False
        crosswalk_rectangles = 0  # 计数检测到的矩形框数量
        last_x = None  # 在此处初始化last_x
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # 检测矩形框的尺寸和相邻间距，以符合人行横道的规格
            if 25 < w < 35 and 8 < h < 12:  # 符合30cm x 10cm规格
                crosswalk_rectangles += 1
                # 判断多个矩形框是否间距合理
                if last_x is not None and 8 < abs(last_x - x) < 12:
                    crosswalk_detected = True
                last_x = x

        # 假设3个及以上的相邻矩形才算人行横道
        if crosswalk_rectangles >= 3:
            crosswalk_detected = True

        self.crosswalk_detected = crosswalk_detected
        return white_mask if crosswalk_detected else None

    def check_distance_and_stop(self, distance_to_crosswalk):
        """
        如果距离人行横道小于30cm，触发停车逻辑，向下位机发送停车标志。
        """
        if distance_to_crosswalk < self.crosswalk_distance_threshold:
            print("检测到人行横道，距离小于30cm，开始停车")

            # 设置停车标志位
        if self.stop_flag == False:  # 确保仅在未停车时执行
            self.send_stop_flag()

            # 暂停10秒
            self.wait_10_seconds()

            # 停止标志位复位，并发送启动信号
            self.stop_flag = True
            self.send_start_flag()

    def wait_10_seconds(self):
        """模拟10秒等待"""
        print("开始暂停10秒")
        start_time = time.time()
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time >= 10:
                break
            # 确保在等待期间可以响应其他操作
            time.sleep(0.1)  # 设置较小的时间间隔，避免完全阻塞CPU
        print("暂停10秒结束")

    def send_stop_flag(self):
        """发送停车标志到下位机"""
        try:
            stop_flag = False  # 停车标志位为0x00
            data = struct.pack('>B', stop_flag)
            if self.serial_driver:
                self.serial_driver.send_packet(0x00, data)
                print("停车标志已发送到下位机")
            else:
                print("serial_driver未初始化")
        except Exception as e:
            print(f"发送停车标志失败: {e}")

    def send_start_flag(self):
        """发送启动标志到下位机"""
        try:
            start_flag = True  # 启动标志位为0x01
            data = struct.pack('>B', start_flag)
            if self.serial_driver:
                self.serial_driver.send_packet(0x01, data)
                print("启动电机标志已发送到下位机")
            else:
                print("serial_driver未初始化")
        except Exception as e:
            print(f"发送启动标志失败: {e}")

    def process_frame(self, image, distance_to_crosswalk):
        """
        在给定图像帧中检测人行横道，如果检测到并距离小于阈值则停车。
        """
        crosswalk_mask = self.detect_crosswalk(image)
        if crosswalk_mask is not None:
            self.check_distance_and_stop(distance_to_crosswalk)
            # 可视化人行横道检测结果
            cv2.imshow("Crosswalk Detection", crosswalk_mask)
        """
               主循环中每帧的处理函数
               """
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                print("无法读取摄像头帧")
                return
                # 如果车道线还没有计算出误差，直接跳过误差计算
            if np.all(self.left_fit == 0) and np.all(self.right_fit == 0):
                    self.x_error = 0
                    print("车道线还没有计算，x_error设置为0")

            # 检测左变道标志
            if self.detect_left_turn_sign(frame):
                self.execute_lane_change()
            else:
                self.update_state_after_lane_change()

            # 显示检测结果
            cv2.imshow("Frame", frame)
    # ====================== 左转弯 ======================
    def detect_left_turn_sign(self, image):
        """
        检测左变道标志，并增加更严格的面积和宽高比过滤条件。
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)

        contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1200:  # 更严格的面积阈值
                x, y, w, h = cv2.boundingRect(contour)
                if 0.75 < h / w < 1.1 and 25 < w < 35 and 35 < h < 45:  # 更严格的宽高比和尺寸范围
                    self.left_turn_detected = True
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    print("检测到左变道标志！")
                    return True
        self.left_turn_detected = False
        return False

    def execute_lane_change(self):
        """
        执行左变道操作并切换状态。
        """
        if self.left_turn_detected and self.state == "center":
            print("执行左变道操作")
            self.state = "left_lane"  # 切换为左变道状态
            self.send_left_turn_signal()  # 发送左转指令

    def update_state_after_lane_change(self):
        """
        完成左变道后，将状态恢复到中心状态。
        """
        if self.state == "left_lane" and not self.left_turn_detected:
            print("变道完成，恢复到中心状态")
            self.state = "center"
            self.left_lane_completed = True  # 记录已完成变道
            self.send_straight_signal()  # 发送直行指令

    def send_left_turn_signal(self):
        """发送左转信号给下位机，x_error 为负值"""
        type = 0x01  # 数据类型：假设0x01表示左转
        x_error = -10  # 固定的左转误差值，可以根据实际需求调整
        data = struct.pack('>h', x_error)  # 打包数据
        if self.serial_driver:
            self.serial_driver.send_packet(type, data)  # 发送左转命令
            print("发送左转指令，x_error:", x_error)
        else:
            print("serial_driver未初始化")

    def send_straight_signal(self):
        """发送直行信号给下位机，x_error 为0"""
        type = 0x00  # 数据类型：假设0x00表示直行
        x_error = 0  # 变道完成后，回正
        data = struct.pack('>h', x_error)  # 打包数据
        if self.serial_driver:
            self.serial_driver.send_packet(type, data)  # 发送直行命令
            print("发送直行指令，x_error:", x_error)
        else:
            print("serial_driver未初始化")

if __name__ == '__main__':
    bike = BikeClass()

    if bike.cap:  # 确保 VideoCapture 已打开成功
        while True:
            bike.img_preprocess()
            bike.detect_lane_and_send()
            bike.process_frame(bike.img, distance_to_crosswalk=25)  # 假设25cm为检测到人行横道的距离

        #    cv2.imshow('img', bike.img)

            cv2.imshow('edges', bike.edges)
            cv2.imshow('warp_img', bike.warp_img)


            # 按 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    bike.release_resources()