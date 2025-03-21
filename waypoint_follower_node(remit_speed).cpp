//2025_03_21작성한 코드입니다.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/qos.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;

// ANSI 이스케이프 코드
const std::string CLEAR_SCREEN = "\033[2J\033[H";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string RED = "\033[31m";
const std::string CYAN = "\033[36m";
const std::string RESET = "\033[0m";
const std::string BOLD = "\033[1m";

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt) 
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt), prev_error_(0), integral_(0) {}

    double compute(double setpoint, double current_value) {
        double error = setpoint - current_value;
        integral_ += error * dt_;
        double derivative = (error - prev_error_) / dt_;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        prev_error_ = 0;
        integral_ = 0;
    }

private:
    double kp_;     // 비례 게인
    double ki_;     // 적분 게인
    double kd_;     // 미분 게인
    double dt_;     // 제어 주기
    double prev_error_;
    double integral_;
};

struct Waypoint {
    double x;
    double y;
    double z;
};

// 드론 상태 정보 구조체
struct DroneStatus {
    double current_x, current_y, current_z;
    double target_x, target_y, target_z;
    double threshold_x, threshold_y, threshold_z;
    double x_correction, y_correction, z_correction;
    int current_waypoint;
    int total_waypoints;
    bool is_waiting;
    double waiting_time;
    double wait_duration;
    bool has_position;
    bool is_offboard;
    bool is_armed;
    // 라이다 데이터 추가
    float left_lidar;
    float right_lidar;
    float wall_distance;
    // 벽 접근 포인트 관련 정보 추가
    bool is_wall_point;         // 현재 벽 접근 포인트인지 여부
    int wall_point_index;       // 현재 벽 접근 포인트 인덱스 (1: WP1-1, 2: WP2-1, ...)
    std::string waypoint_name;  // 현재 웨이포인트 이름 (예: "WP1", "WP1-1")
};

// TUI 관리를 위한 클래스
class DroneStatusDisplay {
public:
    void update(const DroneStatus& status) {
        std::stringstream ss;
        ss << CLEAR_SCREEN;
        
        // 상단 테두리
        ss << "+" << std::string(78, '-') << "+\n";
        
        // 제목
        ss << "│" << CYAN << BOLD << std::string(30, ' ') << "드론 상태 모니터링" 
           << std::string(30, ' ') << RESET << "│\n";
        
        // 구분선
        ss << "├" << std::string(78, '─') << "┤\n";
        
        // 현재 위치 정보
        ss << "│  현재 위치:                                                                  │\n";
        print_position_info(ss, "X", status.current_x, status.target_x, status.threshold_x);
        print_position_info(ss, "Y", status.current_y, status.target_y, status.threshold_y);
        print_position_info(ss, "Z", status.current_z, status.target_z, status.threshold_z);
        
        // 구분선
        ss << "├" << std::string(78, '─') << "┤\n";
        
        // 라이다 데이터 섹션 추가
        ss << "│  " << CYAN << BOLD << "라이다 데이터:" << RESET << std::string(78 - 16, ' ') << "│\n";
        print_lidar_info(ss, "왼쪽", status.left_lidar);
        print_lidar_info(ss, "오른쪽", status.right_lidar);
        print_lidar_info(ss, "평균", status.wall_distance);
        
        // 구분선
        ss << "├" << std::string(78, '─') << "┤\n";
        
        // PID 보정값
        ss << "│  PID 보정값:                                                                │\n";
        print_correction_info(ss, "X", status.x_correction);
        print_correction_info(ss, "Y", status.y_correction);
        print_correction_info(ss, "Z", status.z_correction);
        
        // 구분선
        ss << "├" << std::string(78, '─') << "┤\n";
        
        // 웨이포인트 정보
        ss << "│  " << CYAN;
        ss << "웨이포인트 진행상황: ";
        
        // 웨이포인트 이름이 있으면 표시, 없으면 번호로 표시
        if (!status.waypoint_name.empty()) {
            ss << status.waypoint_name;
        } else {
            ss << status.current_waypoint + 1;
        }
        
        ss << "/" << status.total_waypoints;
        
        ss << std::string(78 - 35, ' ') << RESET << "│\n";
        
        if (status.is_waiting) {
            ss << "│  " << CYAN;
            ss << "대기 시간: " << std::fixed << std::setprecision(1) 
               << status.waiting_time << "/" << status.wait_duration << "초";
            ss << std::string(78 - 25, ' ') << RESET << "│\n";
        } else {
            ss << "│" << std::string(78, ' ') << "│\n";
        }
        
        // 구분선
        ss << "├" << std::string(78, '─') << "┤\n";
        
        // 시스템 상태
        print_status(ss, "위치 수신", status.has_position);
        print_status(ss, "오프보드 모드", status.is_offboard);
        print_status(ss, "기체 시동", status.is_armed);
        
        // 하단 테두리
        ss << "└" << std::string(78, '─') << "┘\n";
        
        // 출력
        std::cout << ss.str() << std::flush;
    }

private:
    void print_position_info(std::stringstream& ss, const char* axis, double current, double target, double threshold) {
        double diff = std::abs(target - current);
        std::string color = (diff < threshold) ? GREEN : (diff < threshold * 2 ? YELLOW : RED);
        
        ss << "│    " << color << std::left << std::setw(2) << axis << ": "
           << "현재(" << std::fixed << std::setprecision(2) << std::setw(6) << current << "m) → "
           << "목표(" << std::setw(6) << target << "m) "
           << "[차이: " << std::setw(6) << diff << "m]" << RESET
           << std::string(78 - 52, ' ') << "│\n";
    }

    void print_correction_info(std::stringstream& ss, const char* axis, double correction) {
        std::string color = (std::abs(correction) < 0.1) ? GREEN : 
                           (std::abs(correction) < 0.5 ? YELLOW : RED);
        
        ss << "│    " << color << std::left << std::setw(2) << axis << ": "
           << std::fixed << std::setprecision(2) << correction << RESET
           << std::string(78 - 15, ' ') << "│\n";
    }

    // 라이다 데이터 출력을 위한 함수 추가
    void print_lidar_info(std::stringstream& ss, const char* label, float distance) {
        std::string color;
        // 라이다 거리에 따른 색상 설정
        if (distance <= 0.1) {
            color = RED;    // 0.1m 이하는 빨간색 (유효하지 않은 데이터)
            
            ss << "│    " << color << std::left << std::setw(8) << label << ": "
               << std::fixed << std::setprecision(2) << distance << " m " << "(무효)" << RESET
               << std::string(78 - 28, ' ') << "│\n";
            return;
        } else if (distance <= 1.5) {
            color = GREEN;  // 1.5m 이하는 녹색 (목표 달성)
        } else if (distance <= 3.0) {
            color = YELLOW; // 1.5m~3m는 노란색 (가까움)
        } else {
            color = RED;    // 3m 초과는 빨간색 (멈)
        }
        
        ss << "│    " << color << std::left << std::setw(8) << label << ": "
           << std::fixed << std::setprecision(2) << distance << " m" << RESET
           << std::string(78 - 20, ' ') << "│\n";
    }

    void print_status(std::stringstream& ss, const char* label, bool status) {
        ss << "│  " << (status ? GREEN : RED) << BOLD
           << std::left << std::setw(15) << label << ": "
           << std::setw(10) << (status ? "활성" : "비활성") << RESET
           << std::string(78 - 30, ' ') << "│\n";
    }
};

class WaypointFollower : public rclcpp::Node {
public:
    WaypointFollower() : Node("waypoint_follower") {
        display_ = std::make_unique<DroneStatusDisplay>();
        // PID 제어기 초기화 (각 축별로 다른 게인 설정)
        pid_x_ = std::make_unique<PIDController>(1.0, 0.1, 0.2, 0.1);  // X축 PID
        pid_y_ = std::make_unique<PIDController>(1.0, 0.1, 0.2, 0.1);  // Y축 PID
        pid_z_ = std::make_unique<PIDController>(2.0, 0.5, 0.2, 0.1);  // Z축 PID (진동 감소를 위한 조정)

        // 벽 접근용 저속 PID 제어기 (낮은 게인값 사용)
        slow_pid_x_ = std::make_unique<PIDController>(0.3, 0.05, 0.05, 0.1);  // 저속 X축 PID
        slow_pid_y_ = std::make_unique<PIDController>(0.3, 0.05, 0.05, 0.1);  // 저속 Y축 PID
        slow_pid_z_ = std::make_unique<PIDController>(0.5, 0.1, 0.05, 0.1);   // 저속 Z축 PID

        // QoS 설정 - local_position_listener와 동일하게 설정
        auto pub_qos = rclcpp::QoS(1)
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            
        auto sub_qos = rclcpp::QoS(10)
            .best_effort()
            .durability_volatile();
        
        // Load waypoints
        loadWaypoints("");  // 파일 경로는 함수 내부에서 하드코딩됨

        // Publishers - 기존 QoS 유지
        trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", pub_qos);
        vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", pub_qos);
        offboard_control_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", pub_qos);

        // Subscriber - vehicle_odometry 토픽으로 변경
        local_position_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", sub_qos,
            std::bind(&WaypointFollower::localPositionCallback, this, std::placeholders::_1));

        // Yaw 제어값 구독 추가
        yaw_control_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/drone/yaw_control",
            10,
            std::bind(&WaypointFollower::yaw_control_callback, this, std::placeholders::_1));

        // 라이다 데이터 구독 추가
        lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/iris/downward_lidar_plugin/out", 10,
            std::bind(&WaypointFollower::lidar_callback, this, std::placeholders::_1));

        // 벽면 정렬 데이터 구독
        wall_alignment_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "wall_alignment_data", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 2) {
                    // 라이다 데이터가 0.1 이하인 경우 이전 값을 유지
                    float new_left_distance = msg->data[0];
                    float new_right_distance = msg->data[1];
                    
                    if (new_left_distance > 0.1) {
                        left_distance_ = new_left_distance;
                    }
                    
                    if (new_right_distance > 0.1) {
                        right_distance_ = new_right_distance;
                    }
                    
                    // 두 라이다 모두 정상 데이터인 경우에만 로그 출력
                    if (new_left_distance > 0.1 && new_right_distance > 0.1) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "라이다 데이터 수신: 왼쪽=%.2fm, 오른쪽=%.2fm, 평균=%.2fm",
                            left_distance_, right_distance_, (left_distance_ + right_distance_) / 2.0);
                    } else {
                        // 비정상 데이터가 있을 경우 로그 출력
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "비정상 라이다 데이터 감지: 왼쪽=%.2fm, 오른쪽=%.2fm (0.1m 이하의 값은 무시됨)",
                            new_left_distance, new_right_distance);
                    }
                }
            });

        // yaw_control_node에서 발행하는 라이다 데이터 구독 추가
        yaw_control_lidar_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "yaw_control_lidar_data", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 2) {
                    // 라이다 데이터가 0.1 이하인 경우 이전 값을 유지
                    float new_left_distance = msg->data[0];
                    float new_right_distance = msg->data[1];
                    
                    if (new_left_distance > 0.1) {
                        left_distance_ = new_left_distance;
                    }
                    
                    if (new_right_distance > 0.1) {
                        right_distance_ = new_right_distance;
                    }
                    
                    // 두 라이다 모두 정상 데이터인 경우에만 로그 출력
                    if (new_left_distance > 0.1 && new_right_distance > 0.1) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Yaw 컨트롤에서 라이다 데이터 수신: 왼쪽=%.2fm, 오른쪽=%.2fm, 평균=%.2fm",
                            left_distance_, right_distance_, (left_distance_ + right_distance_) / 2.0);
                    } else {
                        // 비정상 데이터가 있을 경우 로그 출력
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Yaw 컨트롤에서 비정상 라이다 데이터 감지: 왼쪽=%.2fm, 오른쪽=%.2fm (0.1m 이하의 값은 무시됨)",
                            new_left_distance, new_right_distance);
                    }
                }
            });

        // Timer for publishing control commands
        timer_ = create_wall_timer(100ms, std::bind(&WaypointFollower::timer_callback, this));

        current_waypoint_index_ = 0;
        offboard_setpoint_counter_ = 0;
        threshold_distance_x_ = 0.1;  // 10cm로 X축 임계값 설정
        threshold_distance_y_ = 0.1;  // 10cm로 Y축 임계값 설정
        threshold_distance_z_ = 0.1;  // 10cm로 z축 임계값 설정
        waypoint_wait_time_ = 10.0; // 10초 대기 시간
        waypoint_reached_time_ = 0.0;
        is_waiting_at_waypoint_ = false;
        arrival_count_ = 0;
        has_position_ = false;

        // 상태 변수 초기화
        drone_status_.is_offboard = false;
        drone_status_.is_armed = false;
        drone_status_.has_position = false;
        drone_status_.is_waiting = false;
        drone_status_.waiting_time = 0.0;
        drone_status_.wait_duration = waypoint_wait_time_;
        drone_status_.is_wall_point = false;
        drone_status_.wall_point_index = 0;
        drone_status_.waypoint_name = "";

        current_yaw_ = 0.0;  // 초기 yaw 값
        last_valid_yaw_ = 0.0;  // 마지막으로 수신된 유효한 yaw 값
        use_external_yaw_ = false;  // yaw_control 노드 사용 여부 플래그
        last_yaw_control_time_ = this->get_clock()->now();  // yaw 제어 메시지 마지막 수신 시간
        yaw_control_timeout_ = 2.0;  // yaw 제어 타임아웃 (초)

        left_distance_ = 0.0;
        right_distance_ = 0.0;
        is_moving_to_wall_ = false;
        wall_target_x_ = 0.0;
        wall_target_y_ = 0.0;
        is_aligned_with_wall_ = false;
        is_waiting_at_wall_ = false;
        
        // 벽면 접근 관련 변수 초기화
        wall_approach_step_ = 0;
        use_slow_pid_ = false;
        wall_target_final_x_ = 0.0;
        wall_target_final_y_ = 0.0;

        // 벽 접근 속도 제한 추가
        wall_approach_max_velocity_ = 0.7f;  // 벽 접근 최대 속도 설정 (0.7 m/s)
        wall_direction_unit_ = {0.0f, 0.0f, 0.0f};  // 벽 접근 방향 단위 벡터
        wall_approach_start_time_ = this->get_clock()->now();  // 벽 접근 시작 시간
        wall_approach_total_distance_ = 0.0f;  // 벽까지의 총 거리
        
        RCLCPP_INFO(this->get_logger(), "웨이포인트 팔로워 노드가 시작되었습니다. 총 %zu개의 웨이포인트가 로드되었습니다.", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "라이다 초기 상태: 왼쪽=%.2fm, 오른쪽=%.2fm", left_distance_, right_distance_);
    }

    ~WaypointFollower() {
        display_.reset();  // ncurses 정리
    }

private:
    void loadWaypoints(const std::string& filename) {
        std::string waypoints_file = "/home/jetson2/Documents/GPR_DRONE/ws_sensor_combined/waypoint/waypoints_3d.txt";
        std::ifstream file(waypoints_file);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "웨이포인트 파일을 열 수 없습니다: %s", waypoints_file.c_str());
            return;
        }

        waypoints_.clear();
        std::string line;
        
        // 첫 번째 줄(헤더) 건너뛰기
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            // 빈 줄이나 주석 건너뛰기
            if (line.empty() || line[0] == '#') {
                continue;
            }

            // WPxxx: x, y, z 형식에서 좌표만 추출
            size_t colon_pos = line.find(':');
            if (colon_pos != std::string::npos) {
                std::string coords = line.substr(colon_pos + 1);
                std::stringstream ss(coords);
                std::string x_str, y_str, z_str;
                
                // x 좌표
                std::getline(ss, x_str, ',');
                // y 좌표
                std::getline(ss, y_str, ',');
                // z 좌표
                std::getline(ss, z_str, ',');

                try {
                    Waypoint wp;
                    wp.x = std::stod(x_str);
                    wp.y = std::stod(y_str);
                    wp.z = std::stod(z_str);
                    waypoints_.push_back(wp);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "웨이포인트 파싱 오류: %s", line.c_str());
                    continue;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "총 %zu개의 웨이포인트를 로드했습니다.", waypoints_.size());
        
        // 총 웨이포인트 개수 계산 (각 웨이포인트마다 벽 접근 지점도 포함)
        int total_points = waypoints_.size() * 2;  // 각 웨이포인트마다 벽 접근 지점도 추가
        
        // DroneStatus 업데이트
        drone_status_.total_waypoints = total_points;
        drone_status_.current_waypoint = 0;
    }

    void localPositionCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        current_position_x_ = msg->position[0];
        current_position_y_ = msg->position[1];
        // Z 위치는 라이다 데이터로 업데이트되므로 여기서는 업데이트하지 않음
        has_position_ = true;
        updateDroneStatus();
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->data.size() >= 16) {  // PointCloud2의 첫 번째 포인트 값 추출
            current_position_z_ = -(*reinterpret_cast<const float*>(&msg->data[0]));  // NED 좌표계에 맞게 음수로 변환
            has_position_ = true;
            updateDroneStatus();
        } else {
            RCLCPP_WARN(this->get_logger(), "라이다 데이터가 충분하지 않습니다.");
        }
    }

    void updateDroneStatus() {
        drone_status_.current_x = current_position_x_;
        drone_status_.current_y = current_position_y_;
        drone_status_.current_z = current_position_z_;
        
        // 벽 접근 모드일 때
        if (is_moving_to_wall_ || is_waiting_at_wall_) {
            // 벽 접근 지점 정보 설정
            drone_status_.is_wall_point = true;
            drone_status_.wall_point_index = current_waypoint_index_ + 1;  // 1부터 시작
            
            // 목표 좌표를 벽 접근 지점으로 설정
            if (is_moving_to_wall_) {
                drone_status_.target_x = wall_target_x_;
                drone_status_.target_y = wall_target_y_;
                drone_status_.target_z = waypoints_[current_waypoint_index_].z;  // 현재 고도 유지가 아닌 원래 웨이포인트의 z 값으로 유지
            } else {
                // 벽에서 대기 중일 때는 최종 목표 위치 사용
                drone_status_.target_x = wall_target_final_x_;
                drone_status_.target_y = wall_target_final_y_;
                drone_status_.target_z = current_position_z_;
            }
            
            // 웨이포인트 이름 설정 (WP1-1, WP2-1 등)
            std::stringstream wp_name;
            wp_name << "WP" << current_waypoint_index_ + 1 << "-1";
            drone_status_.waypoint_name = wp_name.str();
            
            // 진행 상황 업데이트 (벽 접근 지점)
            drone_status_.current_waypoint = current_waypoint_index_ * 2 + 1;  // 벽 접근 지점은 홀수 인덱스
        } 
        // 일반 웨이포인트 모드일 때
        else if (current_waypoint_index_ < waypoints_.size()) {
            drone_status_.is_wall_point = false;
            const auto& wp = waypoints_[current_waypoint_index_];
            drone_status_.target_x = wp.x;
            drone_status_.target_y = wp.y;
            drone_status_.target_z = wp.z;
            
            // 웨이포인트 이름 설정 (WP1, WP2 등)
            std::stringstream wp_name;
            wp_name << "WP" << current_waypoint_index_ + 1;
            drone_status_.waypoint_name = wp_name.str();
            
            // 진행 상황 업데이트 (일반 웨이포인트)
            drone_status_.current_waypoint = current_waypoint_index_ * 2;  // 일반 웨이포인트는 짝수 인덱스
        }
        
        drone_status_.threshold_x = threshold_distance_x_;
        drone_status_.threshold_y = threshold_distance_y_;
        drone_status_.threshold_z = threshold_distance_z_;
        drone_status_.has_position = has_position_;
        
        if (is_waiting_at_waypoint_) {
            drone_status_.is_waiting = true;
            drone_status_.waiting_time = this->get_clock()->now().seconds() - waypoint_reached_time_;
        } else if (is_waiting_at_wall_) {
            // 라이다 데이터가 유효한지 확인
            bool valid_lidar_data = (left_distance_ > 0.1 && right_distance_ > 0.1);
            
            // 라이다 데이터가 유효하고 벽과 정렬된 경우에만 대기 시간 표시
            if (valid_lidar_data && is_aligned_with_wall_) {
                drone_status_.is_waiting = true;
                drone_status_.waiting_time = this->get_clock()->now().seconds() - waypoint_reached_time_;
            } else {
                drone_status_.is_waiting = false;
                drone_status_.waiting_time = 0.0;
            }
        } else {
            drone_status_.is_waiting = false;
            drone_status_.waiting_time = 0.0;
        }
        
        drone_status_.left_lidar = left_distance_;
        drone_status_.right_lidar = right_distance_;
        drone_status_.wall_distance = (left_distance_ + right_distance_) / 2.0;
        
        display_->update(drone_status_);
    }

    bool reachedWaypoint() {
        if (!has_position_ || current_waypoint_index_ >= waypoints_.size()) return false;
        
        const auto& wp = waypoints_[current_waypoint_index_];
        double dx = std::abs(wp.x - current_position_x_);
        double dy = std::abs(wp.y - current_position_y_);
        double dz = std::abs(wp.z - current_position_z_);
        
        if (dx < threshold_distance_x_ && 
            dy < threshold_distance_y_ && 
            dz < threshold_distance_z_) {
            return true;
        }
        return false;
    }

    void publishOffboardControlMode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publishTrajectorySetpoint() {
        // 변수 선언 추가
        double x_correction = 0.0;
        double y_correction = 0.0;
        double z_correction = 0.0;
        
        auto setpoint = px4_msgs::msg::TrajectorySetpoint();
        setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        
        if (is_moving_to_wall_) {
            // 벽 접근 시 속도 제한 적용
            
            // 경과 시간 계산 (초 단위)
            double elapsed_seconds = (this->get_clock()->now() - wall_approach_start_time_).seconds();
            
            // 현재까지 이동해야 할 거리 계산 (최대 속도로 제한)
            float current_distance = std::min(
                static_cast<float>(elapsed_seconds * wall_approach_max_velocity_), 
                wall_approach_total_distance_
            );
            
            // 현재 목표 위치 계산 (시작 위치 + 방향 * 현재 거리)
            float current_target_x = wall_approach_start_x_ + wall_direction_unit_[0] * current_distance;
            float current_target_y = wall_approach_start_y_ + wall_direction_unit_[1] * current_distance;
            float current_target_z = waypoints_[current_waypoint_index_].z;  // Z 값은 웨이포인트 값 유지
            
            // PID 보정값 계산 (slow_pid_ 사용)
            x_correction = slow_pid_x_->compute(current_target_x, current_position_x_);
            y_correction = slow_pid_y_->compute(current_target_y, current_position_y_);
            z_correction = slow_pid_z_->compute(current_target_z, current_position_z_);
            
            // 보정된 위치 설정
            setpoint.position = {
                static_cast<float>(current_target_x + x_correction),
                static_cast<float>(current_target_y + y_correction),
                static_cast<float>(current_target_z + z_correction)
            };
            
            // 속도 벡터 설정 (단위 방향 * 최대 속도)
            setpoint.velocity = {
                wall_direction_unit_[0] * wall_approach_max_velocity_,
                wall_direction_unit_[1] * wall_approach_max_velocity_,
                wall_direction_unit_[2] * wall_approach_max_velocity_
            };
            
            // 목표 지점 도달 확인
            float remaining_distance = wall_approach_total_distance_ - current_distance;
            
            RCLCPP_DEBUG(this->get_logger(), 
                "벽 접근 중 - 속도: %.2f m/s, 경과 시간: %.2f초, 남은 거리: %.2f m",
                wall_approach_max_velocity_, elapsed_seconds, remaining_distance);
                
            // 목표 지점에 도달했는지 확인 (거리가 0에 가까워지면)
            if (remaining_distance < 0.05) {  // 5cm 이내면 도착으로 간주
                // 벽 접근 완료 처리 (기존 코드와 동일)
                RCLCPP_INFO(this->get_logger(), "벽면 방향 이동 완료! 현재 위치: (%.2f, %.2f), 목표 위치: (%.2f, %.2f)",
                           current_position_x_, current_position_y_, wall_target_final_x_, wall_target_final_y_);
                
                // 상태 초기화 및 벽 대기 상태 설정
                is_moving_to_wall_ = false;
                is_waiting_at_waypoint_ = false;
                is_waiting_at_wall_ = true;
                use_slow_pid_ = false;
                
                // 벽 대기 시간 기록 시작
                waypoint_reached_time_ = this->get_clock()->now().seconds();
                
                RCLCPP_INFO(this->get_logger(), 
                    "[중요] 벽 접근 지점(WP%zu-1)에 도달. 대기 시작: 계획된 대기 시간 %.1f초",
                    current_waypoint_index_ + 1, waypoint_wait_time_);
                
                // 헤드 방향이 벽면과 정렬되었는지 확인
                float distance_diff = std::abs(left_distance_ - right_distance_);
                if (distance_diff < 0.1) {
                    is_aligned_with_wall_ = true;
                    RCLCPP_INFO(this->get_logger(), "헤드 방향이 벽면과 정렬되었습니다. 대기를 시작합니다.");
                } else {
                    is_aligned_with_wall_ = false;
                    RCLCPP_INFO(this->get_logger(), 
                        "헤드 방향이 벽면과 정렬되지 않았습니다. 왼쪽 거리: %.2f, 오른쪽 거리: %.2f, 차이: %.2f",
                        left_distance_, right_distance_, distance_diff);
                }
            }
        } 
        else {
            // 일반 웨이포인트 이동 시 기존 코드 사용
            double target_x, target_y, target_z;
            
            if (is_waiting_at_wall_) {
                // 벽 접근 지점에서 대기 중일 때는 최종 목표 위치 사용
                target_x = wall_target_final_x_;
                target_y = wall_target_final_y_;
                target_z = waypoints_[current_waypoint_index_].z;
            }
            else {
                // 일반적인 웨이포인트 추적 중일 때
                target_x = waypoints_[current_waypoint_index_].x;
                target_y = waypoints_[current_waypoint_index_].y;
                target_z = waypoints_[current_waypoint_index_].z;
            }

            // PID 보정값 계산
            if (use_slow_pid_) {
                // 저속 PID 사용 (필요한 경우)
                x_correction = slow_pid_x_->compute(target_x, current_position_x_);
                y_correction = slow_pid_y_->compute(target_y, current_position_y_);
                z_correction = slow_pid_z_->compute(target_z, current_position_z_);
                
                const double max_correction = 0.05;
                x_correction = std::max(-max_correction, std::min(x_correction, max_correction));
                y_correction = std::max(-max_correction, std::min(y_correction, max_correction));
                z_correction = std::max(-max_correction, std::min(z_correction, max_correction));
            } else {
                // 일반 PID 사용
                x_correction = pid_x_->compute(target_x, current_position_x_);
                y_correction = pid_y_->compute(target_y, current_position_y_);
                z_correction = pid_z_->compute(target_z, current_position_z_);
            }

            // 보정값 상태 업데이트
            drone_status_.x_correction = x_correction;
            drone_status_.y_correction = y_correction;
            drone_status_.z_correction = z_correction;

            // 최종 목표 위치 설정
            setpoint.position = {
                static_cast<float>(target_x + x_correction),
                static_cast<float>(target_y + y_correction),
                static_cast<float>(target_z + z_correction)
            };
            
            // 일반 이동시에는 속도 제한 없음
            setpoint.velocity[0] = NAN;
            setpoint.velocity[1] = NAN;
            setpoint.velocity[2] = NAN;
        }

        // Yaw 제어 - 기존 코드 유지
        rclcpp::Time current_time = this->get_clock()->now();
        double time_since_last_yaw = (current_time - last_yaw_control_time_).seconds();
        
        if (use_external_yaw_ && time_since_last_yaw < yaw_control_timeout_) {
            // 외부에서 받은 yaw 값 사용
            setpoint.yaw = current_yaw_;
            last_valid_yaw_ = current_yaw_;  // 유효한 yaw 값 저장
            RCLCPP_DEBUG(this->get_logger(), "외부 yaw 값 사용: %.2f", current_yaw_);
        } else {
            if (use_external_yaw_ && time_since_last_yaw >= yaw_control_timeout_) {
                RCLCPP_INFO(this->get_logger(), 
                    "yaw_control 메시지 %.1f초 동안 수신되지 않음. 마지막 유효한 yaw 값(%.2f)을 사용합니다.", 
                    time_since_last_yaw, last_valid_yaw_);
                use_external_yaw_ = false;
            }
            
            // 마지막으로 수신된 유효한 yaw 값 사용
            setpoint.yaw = last_valid_yaw_;
        }

        trajectory_setpoint_publisher_->publish(setpoint);

        // PID 보정값 로깅 (디버깅용)
        RCLCPP_DEBUG(this->get_logger(),
            "PID 보정값 - X: %.3f, Y: %.3f, Z: %.3f",
            x_correction, y_correction, z_correction);
    }

    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    void timer_callback() {
        if (offboard_setpoint_counter_ == 10) {
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
            drone_status_.is_offboard = true;
            drone_status_.is_armed = true;
        }

        // 웨이포인트 도달 확인
        bool reached = reachedWaypoint();
        
        // 웨이포인트에 처음 도달했을 때 (대기 시작)
        if (reached && !is_waiting_at_waypoint_ && !is_moving_to_wall_ && !is_waiting_at_wall_) {
            is_waiting_at_waypoint_ = true;
            waypoint_reached_time_ = this->get_clock()->now().seconds();
            arrival_count_++;
            
            const auto& wp = waypoints_[current_waypoint_index_];
            double dx = wp.x - current_position_x_;
            double dy = wp.y - current_position_y_;
            double dz = wp.z - current_position_z_;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            RCLCPP_INFO(this->get_logger(), 
                "도착 %d: 웨이포인트 %zu 도달, 대기 시작\n"
                "X좌표: 현재(%.2f m) -> 목표(%.2f m), 차이(%.2f m), 임계값(%.2f m)\n"
                "Y좌표: 현재(%.2f m) -> 목표(%.2f m), 차이(%.2f m), 임계값(%.2f m)\n"
                "Z좌표: 현재(%.2f m) -> 목표(%.2f m), 차이(%.2f m), 임계값(%.2f m)",
                arrival_count_, current_waypoint_index_ + 1,
                current_position_x_, wp.x, std::abs(dx), threshold_distance_x_,
                current_position_y_, wp.y, std::abs(dy), threshold_distance_y_,
                current_position_z_, wp.z, std::abs(dz), threshold_distance_z_);
        }
        // 목표 지점에서 벗어났을 때 대기 상태 초기화
        else if (!reached && is_waiting_at_waypoint_ && !is_moving_to_wall_ && !is_waiting_at_wall_) {
            is_waiting_at_waypoint_ = false;
            RCLCPP_WARN(this->get_logger(), 
                "웨이포인트 %zu에서 벗어남. 대기 상태를 초기화합니다.\n"
                "X좌표: 현재(%.2f m) -> 목표(%.2f m), 차이(%.2f m), 임계값(%.2f m)\n"
                "Y좌표: 현재(%.2f m) -> 목표(%.2f m), 차이(%.2f m), 임계값(%.2f m)\n"
                "Z좌표: 현재(%.2f m) -> 목표(%.2f m), 차이(%.2f m), 임계값(%.2f m)",
                current_waypoint_index_ + 1,
                current_position_x_, waypoints_[current_waypoint_index_].x, 
                std::abs(waypoints_[current_waypoint_index_].x - current_position_x_), threshold_distance_x_,
                current_position_y_, waypoints_[current_waypoint_index_].y, 
                std::abs(waypoints_[current_waypoint_index_].y - current_position_y_), threshold_distance_y_,
                current_position_z_, waypoints_[current_waypoint_index_].z, 
                std::abs(waypoints_[current_waypoint_index_].z - current_position_z_), threshold_distance_z_);
        }
        
        // 웨이포인트에서 대기 완료 후 벽면 방향으로 이동 시작
        if (is_waiting_at_waypoint_) {
            double current_time = this->get_clock()->now().seconds();
            double waiting_time = current_time - waypoint_reached_time_;
            
            if (waiting_time >= waypoint_wait_time_) {
                is_waiting_at_waypoint_ = false;
                
                // WP에서 대기 완료 후 항상 WP-1(벽 접근 지점)으로 이동하도록 수정
                // 이전: 라이다 유효성 및 wall_distance > 1.5 조건을 검사하여 조건부로 이동
                // 수정: 조건 없이 항상 벽 접근 지점으로 이동하도록 변경
                RCLCPP_INFO(this->get_logger(), "웨이포인트 %zu에서 %.1f초 대기 완료. 다음으로 이동합니다.",
                           current_waypoint_index_ + 1, waypoint_wait_time_);
                
                // 현재 라이다 데이터가 유효하지 않더라도, 기본값 (기본 거리 3m)을 사용하여 벽 접근 지점 계산
                float wall_distance = 3.0;  // 기본 거리 값 (라이다 유효하지 않을 때 사용)
                
                // 유효한 라이다 데이터가 있으면 실측 거리 사용
                if (left_distance_ > 0.1 && right_distance_ > 0.1) {
                    wall_distance = (left_distance_ + right_distance_) / 2.0;
                    RCLCPP_INFO(this->get_logger(), "측정된 벽면과의 거리: 왼쪽=%.2fm, 오른쪽=%.2fm, 평균=%.2fm", 
                               left_distance_, right_distance_, wall_distance);
                } else {
                    RCLCPP_WARN(this->get_logger(), "유효하지 않은 라이다 데이터. 기본 거리(%.2fm)를 사용합니다. 왼쪽=%.2fm, 오른쪽=%.2fm", 
                              wall_distance, left_distance_, right_distance_);
                }
                
                // 항상 벽 접근 지점 계산 및 이동 수행 (조건 확인하지 않음)
                RCLCPP_INFO(this->get_logger(), "벽면 방향으로 이동을 시작합니다. 현재 Yaw: %.2f도", 
                           current_yaw_ * 180.0 / M_PI);
                
                // 벽과의 거리가 1.5m가 되는 지점까지 이동해야 할 거리
                float target_distance = 1.5;  // 목표 거리: 1.5m
                float move_distance = wall_distance - target_distance;
                
                // move_distance가 음수인 경우 (이미 1.5m보다 가까운 경우) 최소값으로 조정
                move_distance = std::max(move_distance, 0.5f);  // 최소 0.5m는 이동하도록 설정
                
                // 드론의 헤드 방향을 기준으로 이동 방향 계산
                double delta_x = move_distance * std::cos(current_yaw_);
                double delta_y = move_distance * std::sin(current_yaw_);
                
                // 최종 목표 위치 설정
                wall_target_final_x_ = current_position_x_ + delta_x;
                wall_target_final_y_ = current_position_y_ + delta_y;
                
                // 속도 제한 이동을 위한 변수 설정
                wall_approach_start_x_ = current_position_x_;
                wall_approach_start_y_ = current_position_y_;
                wall_approach_total_distance_ = move_distance;
                wall_approach_start_time_ = this->get_clock()->now();
                
                // 단위 방향 벡터 계산
                double total_distance = std::sqrt(delta_x*delta_x + delta_y*delta_y);
                if (total_distance > 0.001) {
                    wall_direction_unit_ = {
                        static_cast<float>(delta_x / total_distance),
                        static_cast<float>(delta_y / total_distance),
                        0.0f  // Z 방향으로는 이동하지 않음
                    };
                }
                
                is_moving_to_wall_ = true;
                use_slow_pid_ = true;  // 저속 PID 사용 설정
                
                RCLCPP_INFO(this->get_logger(), 
                    "벽면 방향 이동 시작: (%.2f, %.2f) -> 최종 지점(%.2f, %.2f), 총 이동 거리: %.2fm, 최대 속도: %.2f m/s",
                    current_position_x_, current_position_y_, 
                    wall_target_final_x_, wall_target_final_y_,
                    total_distance, wall_approach_max_velocity_);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "웨이포인트 %zu에서 대기 중: %.1f초 / %.1f초",
                    current_waypoint_index_ + 1, waiting_time, waypoint_wait_time_);
            }
        }
        
        // 벽면 방향으로 이동 중인 경우 - 이제 publishTrajectorySetpoint()에서 처리
        if (is_moving_to_wall_) {
            // 경과 시간 계산 (초 단위)
            double elapsed_seconds = (this->get_clock()->now() - wall_approach_start_time_).seconds();
            
            // 현재까지 이동해야 할 거리 계산 (최대 속도로 제한)
            float current_distance = std::min(
                static_cast<float>(elapsed_seconds * wall_approach_max_velocity_), 
                wall_approach_total_distance_
            );
            
            // 남은 거리 계산
            float remaining_distance = wall_approach_total_distance_ - current_distance;
            
            // 로그 출력
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "벽면 방향으로 이동 중: 속도=%.2f m/s, 경과 시간=%.1f초, 남은 거리=%.2fm",
                wall_approach_max_velocity_, elapsed_seconds, remaining_distance);
        }

        publishOffboardControlMode();
        publishTrajectorySetpoint();

        offboard_setpoint_counter_++;
    }
    
    // 다음 웨이포인트로 이동하는 함수
    void moveToNextWaypoint() {
        // 벽 대기 상태에서 호출되었는지 확인 (timer_callback에서 이미 처리됨)
        if (is_waiting_at_wall_) {
            RCLCPP_WARN(this->get_logger(), 
                "경고: 벽 대기 상태에서 moveToNextWaypoint가 호출되었습니다. 이는 timer_callback에서 처리됩니다.");
            return;
        }
        
        // 일반 웨이포인트에서 대기 상태를 초기화
        if (is_waiting_at_waypoint_) {
            RCLCPP_INFO(this->get_logger(), "웨이포인트 %zu에서 %d초 대기 완료. 다음으로 이동합니다.", 
                       current_waypoint_index_ + 1, waypoint_wait_time_);
            is_waiting_at_waypoint_ = false;
            drone_status_.is_waiting = false;
        }
        
        // 현재 웨이포인트에서 다음 웨이포인트로 인덱스 증가
        // (timer_callback에서 벽 접근 지점 이동은 인덱스를 증가시키지 않음)
        RCLCPP_INFO(this->get_logger(), 
            "moveToNextWaypoint 함수 호출: 현재 인덱스=%zu -> 인덱스 증가 없음", 
            current_waypoint_index_);
    }
    
    // 모든 웨이포인트 완료 시 호출하는 함수 (RTL 수행)
    void completeAllWaypoints() {
        RCLCPP_INFO(this->get_logger(), "모든 웨이포인트 및 벽 접근 완료. RTL을 시작합니다.");
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
        RCLCPP_INFO(this->get_logger(), "RTL 명령을 성공적으로 발행했습니다. 드론이 시작 지점으로 돌아갑니다.");
        
        // 모든 작업 완료 후 노드 종료
        RCLCPP_INFO(this->get_logger(), "작업 완료. 10초 후 노드가 종료됩니다.");
        
        // 10초 후 종료 타이머 설정 (RTL이 시작되도록 충분한 시간을 줌)
        create_wall_timer(10000ms, [this]() {
            RCLCPP_INFO(this->get_logger(), "노드를 종료합니다.");
            rclcpp::shutdown();
        });
    }

    void yaw_control_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_ = msg->data;
        last_valid_yaw_ = current_yaw_;  // 수신된 yaw 값을 마지막 유효한 값으로 저장
        use_external_yaw_ = true;
        last_yaw_control_time_ = this->get_clock()->now();  // 마지막 수신 시간 업데이트
        RCLCPP_INFO(this->get_logger(), "Received yaw control value: %.2f degrees (%.2f rad)", 
                   current_yaw_ * 180.0 / M_PI, current_yaw_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr local_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_control_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;  // 라이다 구독자 추가
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wall_alignment_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yaw_control_lidar_sub_;  // yaw_control 라이다 데이터 구독자 추가

    std::vector<Waypoint> waypoints_;
    size_t current_waypoint_index_;
    int offboard_setpoint_counter_;
    double threshold_distance_x_;  // X축 임계값
    double threshold_distance_y_;  // Y축 임계값
    double threshold_distance_z_;  // Z축 임계값
    double current_position_x_;
    double current_position_y_;
    double current_position_z_;
    
    // 웨이포인트 대기 관련 변수들
    double waypoint_wait_time_;      // 각 웨이포인트에서 대기할 시간 (초)
    double waypoint_reached_time_;   // 웨이포인트에 도달한 시간
    bool is_waiting_at_waypoint_;    // 현재 웨이포인트에서 대기 중인지 여부
    int arrival_count_;              // 도착 횟수를 카운트
    bool has_position_;  // 위치 데이터 수신 여부

    // PID 제어기 추가
    std::unique_ptr<PIDController> pid_x_;
    std::unique_ptr<PIDController> pid_y_;
    std::unique_ptr<PIDController> pid_z_;

    // 벽 접근용 저속 PID 제어기
    std::unique_ptr<PIDController> slow_pid_x_;
    std::unique_ptr<PIDController> slow_pid_y_;
    std::unique_ptr<PIDController> slow_pid_z_;

    std::unique_ptr<DroneStatusDisplay> display_;
    DroneStatus drone_status_;

    double current_yaw_;
    double last_valid_yaw_;  // 마지막으로 수신된 유효한 yaw 값
    bool use_external_yaw_;
    rclcpp::Time last_yaw_control_time_;  // yaw 제어 메시지 마지막 수신 시간
    double yaw_control_timeout_;  // yaw 제어 타임아웃 (초)

    float left_distance_;
    float right_distance_;
    bool is_moving_to_wall_;  // 벽면으로 이동 중인지 여부
    double wall_target_x_;    // 벽면 방향 이동 목표 X 좌표
    double wall_target_y_;    // 벽면 방향 이동 목표 Y 좌표
    bool is_aligned_with_wall_;  // 헤드 방향이 벽면과 정렬되었는지 여부
    bool is_waiting_at_wall_;    // 벽면 이동 후 대기 중인지 여부

    int wall_approach_step_;  // 벽면 이동 단계
    bool use_slow_pid_;        // 저속 PID 사용 여부
    double wall_target_final_x_;  // 최종 목표 X 좌표
    double wall_target_final_y_;  // 최종 목표 Y 좌표

    // 추가 멤버 변수
    float wall_approach_max_velocity_;  // 벽 접근 최대 속도
    std::array<float, 3> wall_direction_unit_;  // 벽 접근 방향 단위 벡터
    rclcpp::Time wall_approach_start_time_;  // 벽 접근 시작 시간
    float wall_approach_total_distance_;  // 벽까지의 총 거리
    float wall_approach_start_x_;  // 벽 접근 시작 X 좌표
    float wall_approach_start_y_;  // 벽 접근 시작 Y 좌표
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollower>());
    rclcpp::shutdown();
    return 0;
} 
