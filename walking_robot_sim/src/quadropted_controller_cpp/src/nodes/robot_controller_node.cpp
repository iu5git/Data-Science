#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <quadropted_msgs/msg/robot_foot_contact.hpp>
#include <quadropted_msgs/msg/robot_mode_command.hpp>
#include <quadropted_msgs/msg/robot_velocity.hpp>
#include <quadropted_msgs/srv/robot_behavior_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "quadropted_controller_cpp/controllers/crawl_gait.hpp"
#include "quadropted_controller_cpp/controllers/rest_controller.hpp"
#include "quadropted_controller_cpp/controllers/stand_controller.hpp"
#include "quadropted_controller_cpp/controllers/trot_gait.hpp"
#include "quadropted_controller_cpp/kinematics/inverse_kinematics.hpp"
#include "quadropted_controller_cpp/states/state_command.hpp"
#include "quadropted_controller_cpp/utils/math_utils.hpp"

namespace quadropted {

class RobotControllerNode : public rclcpp::Node {
  public:
    RobotControllerNode() : Node("robot_controller_cpp"), rate_(60), state_(0.25) {
        declare_parameter("verbose", false);
        // debug_mode removed
        verbose_ = get_parameter("verbose").as_bool();
        // debug_mode removed

        // Геометрия робота
        double body[] = {0.3762, 0.0935};
        double legs[] = {0.0, 0.0955, 0.213, 0.213};

        // Default stance — асимметричный как в Python RobotController.py
        // Python: delta_x = body[0]*0.5, x_shift_front=0.02, x_shift_back=0.0
        // FR/FL: 0.1881 + 0.02 =  0.2081
        // RR/RL: -0.1881 + 0.0  = -0.1881  (2cm ближе к центру чем передние!)
        double dx_front = body[0] * 0.5 + 0.02;  // 0.2081 — передние лапы
        double dx_back = body[0] * 0.5 + 0.0;    // 0.1881 — задние лапы
        double dy = body[1] * 0.5 + legs[1];
        default_stance_.resize(3, 4);
        default_stance_ << dx_front, dx_front, -dx_back, -dx_back, -dy, dy, -dy, dy, 0, 0, 0, 0;

        state_.foot_locations = default_stance_;
        state_.behavior_state = BehaviorState::REST;

        // Контроллеры
        // FIX: вернуть оригинальные timing как в Python RobotController.py
        // stance_time=0.04, swing_time=0.18 → stance_ticks=2, swing_ticks=9
        trot_gait_ = std::make_unique<TrotGaitController>(0.04, 0.18, 0.02, false, default_stance_);
        crawl_gait_ = std::make_unique<CrawlGaitController>(0.55, 0.45, 0.02, default_stance_);
        rest_ctrl_ = std::make_unique<RestController>(default_stance_);
        stand_ctrl_ = std::make_unique<StandController>(default_stance_);
        use_trot_ = false;
        use_stand_ = false;

        // Начинаем с REST → TROT
        command_.trot_event = true;
        command_.rest_event = true;
        change_controller();

        // IK
        ik_ = std::make_unique<InverseKinematics>(body[0], body[1], legs[0], legs[1], legs[2], legs[3]);

        // Publishers
        joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("joint_group_controller/commands", 10);
        foot_contact_pub_ =
            create_publisher<quadropted_msgs::msg::RobotFootContact>("foot_contact", rclcpp::SensorDataQoS());

        // Subscriptions
        velocity_sub_ = create_subscription<quadropted_msgs::msg::RobotVelocity>(
            "robot_velocity", 10, [this](const quadropted_msgs::msg::RobotVelocity::SharedPtr msg) {
                if (msg->robot_id == 1) {
                    command_.velocity = {msg->cmd_vel.linear.x, msg->cmd_vel.linear.y, msg->cmd_vel.linear.z};
                    command_.yaw_rate = {msg->cmd_vel.angular.x, msg->cmd_vel.angular.y, msg->cmd_vel.angular.z};

                    // STAND mode — логируем каждую команду
                    if (state_.behavior_state == BehaviorState::STAND) {
                        RCLCPP_INFO(get_logger(), "[STAND VELOCITY] vx=%.4f vy=%.4f vz=%.4f | ax=%.4f ay=%.4f az=%.4f",
                                    command_.velocity[0], command_.velocity[1], command_.velocity[2],
                                    command_.yaw_rate[0], command_.yaw_rate[1], command_.yaw_rate[2]);
                    }

                    // Ограничение скорости для CRAWL режима (как в Python crawl_gait.py)
                    if (state_.behavior_state == BehaviorState::CRAWL) {
                        constexpr double crawl_max_vx = 0.011;
                        constexpr double crawl_max_yaw = 0.15;
                        command_.velocity[0] = std::clamp(command_.velocity[0], -crawl_max_vx, crawl_max_vx);
                        command_.velocity[1] =
                            std::clamp(command_.velocity[1], -crawl_max_vx * 0.5, crawl_max_vx * 0.5);
                        command_.yaw_rate[2] = std::clamp(command_.yaw_rate[2], -crawl_max_yaw, crawl_max_yaw);
                    }
                    if (false)
                        RCLCPP_DEBUG(get_logger(), "[DEBUG] Velocity: vx=%.4f vy=%.4f vz=%.4f yaw=%.4f",
                                     command_.velocity[0], command_.velocity[1], command_.velocity[2],
                                     command_.yaw_rate[2]);
                }
            });

        // IMU subscription — обновляем roll/pitch для компенсации
        imu_sub_ =
            create_subscription<sensor_msgs::msg::Imu>("imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                // Конвертируем quaternion в euler angles
                double w = msg->orientation.w;
                double x = msg->orientation.x;
                double y = msg->orientation.y;
                double z = msg->orientation.z;
                state_.imu_roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
                state_.imu_pitch = std::asin(2.0 * (w * y - z * x));
                if (false)
                    RCLCPP_DEBUG(get_logger(), "[DEBUG] IMU: roll=%.4f pitch=%.4f", state_.imu_roll, state_.imu_pitch);
            });

        mode_sub_ = create_subscription<quadropted_msgs::msg::RobotModeCommand>(
            "robot_mode", 10, [this](const quadropted_msgs::msg::RobotModeCommand::SharedPtr msg) {
                if (msg->robot_id == 1) {
                    if (msg->mode == "REST") {
                        command_.rest_event = true;
                        command_.trot_event = false;
                        command_.crawl_event = false;
                        command_.stand_event = false;
                    } else if (msg->mode == "TROT") {
                        command_.rest_event = false;
                        command_.trot_event = true;
                        command_.crawl_event = false;
                        command_.stand_event = false;
                    } else if (msg->mode == "CRAWL") {
                        command_.rest_event = false;
                        command_.trot_event = false;
                        command_.crawl_event = true;
                        command_.stand_event = false;
                    } else if (msg->mode == "STAND") {
                        command_.rest_event = false;
                        command_.trot_event = false;
                        command_.crawl_event = false;
                        command_.stand_event = true;
                    }
                    change_controller();
                    controller_change_needed_ = true;
                }
            });

        // Control loop timer — используем microseconds для точной частоты 60 Hz
        // Python: 1.0/60 = 0.01667s, C++: 1000000/60 = 16666.67µs ≈ 16667µs
        timer_ = create_wall_timer(std::chrono::microseconds(static_cast<long long>(1000000.0 / rate_)),
                                   std::bind(&RobotControllerNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "Robot Controller Node (C++) started at %d Hz", rate_);
        RCLCPP_INFO(get_logger(), "Startup grace period: 2 seconds (waiting for robot to land)");

        // Behavior service (sit/up/walk)
        behavior_srv_ = create_service<quadropted_msgs::srv::RobotBehaviorCommand>(
            "robot_behavior_command",
            [this](const std::shared_ptr<quadropted_msgs::srv::RobotBehaviorCommand::Request> request,
                   std::shared_ptr<quadropted_msgs::srv::RobotBehaviorCommand::Response> response) {
                std::string cmd = request->command;
                std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
                RCLCPP_INFO(get_logger(), "Received behavior command: %s", cmd.c_str());

                if (cmd == "sit") {
                    command_.stand_event = true;
                    command_.rest_event = false;
                    command_.trot_event = false;
                    command_.crawl_event = false;
                    change_controller();
                    controller_change_needed_ = true;
                    state_.body_local_position[2] = -0.15;
                    response->success = true;
                    response->message = "Robot sat down.";
                } else if (cmd == "up") {
                    command_.rest_event = true;
                    command_.stand_event = false;
                    command_.trot_event = false;
                    command_.crawl_event = false;
                    change_controller();
                    controller_change_needed_ = true;
                    state_.body_local_position[2] = 0.0;
                    response->success = true;
                    response->message = "Robot stood up.";
                } else if (cmd == "walk") {
                    command_.rest_event = true;
                    command_.trot_event = true;
                    command_.stand_event = false;
                    command_.crawl_event = false;
                    change_controller();
                    controller_change_needed_ = true;
                    state_.body_local_position[2] = 0.0;
                    response->success = true;
                    response->message = "Robot started walking.";
                } else {
                    response->success = false;
                    response->message = "Unknown command: " + request->command;
                }
            });
    }

  private:
    void change_controller() {
        if (command_.trot_event && command_.rest_event) {
            // REST first, then TROT
            state_.behavior_state = BehaviorState::REST;
            rest_ctrl_->pid().reset(this->now().seconds());
            command_.rest_event = false;

            state_.behavior_state = BehaviorState::TROT;
            use_crawl_ = false;
            use_trot_ = true;
            trot_gait_->pid_controller().reset(this->now().seconds());
            state_.ticks = 0;
            state_.body_local_position[2] = 0.0;  // поднять корпус
            command_.trot_event = false;
            RCLCPP_INFO(get_logger(), "Switched to TROT controller");
        } else if (command_.trot_event) {
            if (state_.behavior_state == BehaviorState::REST) {
                state_.behavior_state = BehaviorState::TROT;
                use_trot_ = true;
                use_crawl_ = false;
                use_stand_ = false;
                trot_gait_->pid_controller().reset(this->now().seconds());
                state_.ticks = 0;
                state_.body_local_position[2] = 0.0;
            } else if (state_.behavior_state == BehaviorState::CRAWL) {
                state_.behavior_state = BehaviorState::TROT;
                use_trot_ = true;
                use_crawl_ = false;
                use_stand_ = false;
                trot_gait_->pid_controller().reset(this->now().seconds());
                state_.ticks = 0;
                state_.body_local_position[2] = 0.0;
                RCLCPP_INFO(get_logger(), "Switched to TROT controller (from CRAWL)");
            } else if (state_.behavior_state == BehaviorState::STAND) {
                state_.behavior_state = BehaviorState::TROT;
                use_trot_ = true;
                use_crawl_ = false;
                use_stand_ = false;
                trot_gait_->pid_controller().reset(this->now().seconds());
                state_.ticks = 0;
                state_.body_local_position[2] = 0.0;
                RCLCPP_INFO(get_logger(), "Switched to TROT controller (from STAND)");
            }
            command_.trot_event = false;
        } else if (command_.rest_event) {
            state_.behavior_state = BehaviorState::REST;
            use_crawl_ = false;
            use_trot_ = false;
            use_stand_ = false;
            rest_ctrl_->pid().reset(this->now().seconds());
            state_.body_local_position[2] = -0.15;  // лечь на землю
            command_.rest_event = false;
            RCLCPP_INFO(get_logger(), "Switched to REST controller — lying down");
        } else if (command_.stand_event) {
            if (state_.behavior_state != BehaviorState::STAND) {
                state_.behavior_state = BehaviorState::STAND;
                use_stand_ = true;
                use_trot_ = false;
                use_crawl_ = false;
                state_.body_local_position[2] = 0.005;
                RCLCPP_INFO(get_logger(), "Switched to STAND controller");
            }
            command_.stand_event = false;
        } else if (command_.crawl_event) {
            state_.behavior_state = BehaviorState::CRAWL;
            use_crawl_ = true;
            use_trot_ = false;
            use_stand_ = false;
            crawl_gait_->reset();
            state_.ticks = 0;
            state_.body_local_position[2] = 0.0;  // поднять корпус из REST
            command_.crawl_event = false;
        }
    }

    Eigen::MatrixXd step_trot(State& state, const Command& cmd) {
        state.ticks++;  // Инкрементируем каждый тик
        // При нулевой скорости — стабильная стойка
        bool has_command =
            std::abs(cmd.velocity[0]) > 1e-4 || std::abs(cmd.velocity[1]) > 1e-4 || std::abs(cmd.yaw_rate[2]) > 1e-4;
        if (!has_command) {
            // Плавное возвращение к default_stance (как в Python autoRest)
            Eigen::MatrixXd result = default_stance_;
            result.row(2).setConstant(cmd.robot_height);
            // Lerp: 90% текущая позиция + 10% к целевой = плавный переход за ~20 шагов
            constexpr double alpha = 0.1;
            return state.foot_locations * (1.0 - alpha) + result * alpha;
        }

        // Use TrotGaitController's step method for unified stance/swing logic
        Eigen::MatrixXd new_foot_locations =
            trot_gait_->step(state.ticks, state.foot_locations,
                             Eigen::Vector3d{cmd.velocity[0], cmd.velocity[1], cmd.yaw_rate[2]}, cmd.robot_height);

        // DEBUG: показываем foot locations
        if (state.ticks % 60 == 0) {
            RCLCPP_INFO(
                get_logger(),
                "[DEBUG] foot_locs: FR=(%.4f,%.4f,%.4f) FL=(%.4f,%.4f,%.4f) RR=(%.4f,%.4f,%.4f) RL=(%.4f,%.4f,%.4f)",
                state.foot_locations(0, 0), state.foot_locations(1, 0), state.foot_locations(2, 0),
                state.foot_locations(0, 1), state.foot_locations(1, 1), state.foot_locations(2, 1),
                state.foot_locations(0, 2), state.foot_locations(1, 2), state.foot_locations(2, 2),
                state.foot_locations(0, 3), state.foot_locations(1, 3), state.foot_locations(2, 3));
        }

        // IMU compensation
        if (trot_gait_->use_imu()) {
            auto comp = trot_gait_->pid_controller().run(state.imu_roll, state.imu_pitch, this->now().seconds());
            Eigen::Matrix3d rot = rotxyz(-comp[0], -comp[1], 0);
            new_foot_locations = rot * new_foot_locations;
            if (false)
                RCLCPP_DEBUG(get_logger(), "[DEBUG] IMU comp: roll=%.3f pitch=%.3f comp_x=%.3f comp_y=%.3f",
                             state.imu_roll, state.imu_pitch, -comp[0], -comp[1]);
        }

        // DEBUG: каждые 60 тиков
        if (state.ticks % 60 == 0) {
            Eigen::VectorXi contacts = trot_gait_->contacts(state.ticks);
            RCLCPP_INFO(get_logger(), "[DEBUG] TROT step: ticks=%d contacts=[%d,%d,%d,%d]", state.ticks, contacts(0),
                        contacts(1), contacts(2), contacts(3));
        }

        return new_foot_locations;
    }

    Eigen::MatrixXd step_crawl(State& state, const Command& cmd) {
        state.ticks++;
        // При нулевой скорости — стабильная стойка
        bool has_command =
            std::abs(cmd.velocity[0]) > 1e-4 || std::abs(cmd.velocity[1]) > 1e-4 || std::abs(cmd.yaw_rate[2]) > 1e-4;
        if (!has_command) {
            // Плавное возвращение к default_stance
            Eigen::MatrixXd result = default_stance_;
            result.row(2).setConstant(cmd.robot_height);
            constexpr double alpha = 0.1;
            return state.foot_locations * (1.0 - alpha) + result * alpha;
        }

        Eigen::VectorXi contacts = crawl_gait_->contacts(state.ticks);
        int phase_idx = crawl_gait_->phase_index(state.ticks);
        Eigen::MatrixXd new_foot_locations = Eigen::MatrixXd::Zero(3, 4);

        for (int leg = 0; leg < 4; ++leg) {
            if (contacts(leg) == 1) {
                // Stance — CrawlStanceController с move_sideways
                // Python: move_sideways = (phase_index in (0,4)), move_left = (phase_index == 0)
                bool move_sideways = (phase_idx == 0 || phase_idx == 4);
                bool move_left = (phase_idx == 0);
                new_foot_locations.col(leg) = crawl_gait_->stance().next_foot_location(
                    leg, state.foot_locations, Eigen::Vector3d{cmd.velocity[0], cmd.velocity[1], cmd.yaw_rate[2]},
                    cmd.robot_height, crawl_gait_->is_first_cycle(), move_sideways, move_left);
            } else {
                // Swing — используем CRAWL swing controller (было: trot_gait_->swing_controller())
                int sub_ticks = crawl_gait_->subphase_ticks(state.ticks);
                double swing_prop = static_cast<double>(sub_ticks) / crawl_gait_->swing_ticks();

                // Python: shifted_left = (phase_index in (1,3))
                bool shifted_left = (phase_idx == 1 || phase_idx == 3);
                (void)shifted_left;  // CrawlSwing пока не использует (TODO в crawl_gait step)

                new_foot_locations.col(leg) = crawl_gait_->swing().next_foot_location(
                    swing_prop, leg, state.foot_locations,
                    Eigen::Vector3d{cmd.velocity[0], cmd.velocity[1], cmd.yaw_rate[2]}, cmd.robot_height);
            }
        }

        // DEBUG
        if (state.ticks % 60 == 0) {
            RCLCPP_INFO(get_logger(), "[DEBUG] CRAWL step: ticks=%d contacts=[%d,%d,%d,%d]", state.ticks, contacts(0),
                        contacts(1), contacts(2), contacts(3));
        }

        return new_foot_locations;
    }

    Eigen::MatrixXd step_rest(State& state, const Command& cmd) {
        state.ticks++;
        return rest_ctrl_->step(state, cmd);
    }

    Eigen::MatrixXd step_stand(State& state, Command& cmd) {
        // NOTE: state.ticks НЕ инкрементируется здесь (как в Python StandController)
        // ticks инкрементируется только в step_trot() и step_crawl()

        // DEBUG: логируем команды STAND (каждый тик т.к. ticks не меняется)
        static int stand_debug_counter = 0;
        stand_debug_counter++;
        if (stand_debug_counter % 30 == 0) {
            RCLCPP_INFO(get_logger(),
                        "[STAND DEBUG] cmd: vx=%.4f vy=%.4f vz=%.4f ax=%.4f ay=%.4f az=%.4f | "
                        "pos: x=%.4f y=%.4f z=%.4f | ori: r=%.4f p=%.4f y=%.4f",
                        cmd.velocity[0], cmd.velocity[1], cmd.velocity[2], cmd.yaw_rate[0], cmd.yaw_rate[1],
                        cmd.yaw_rate[2], state.body_local_position[0], state.body_local_position[1],
                        state.body_local_position[2], state.body_local_orientation[0], state.body_local_orientation[1],
                        state.body_local_orientation[2]);
        }

        return stand_ctrl_->run(state, cmd);
    }

    void publish_foot_contacts() {
        auto msg = std::make_unique<quadropted_msgs::msg::RobotFootContact>();
        if (state_.behavior_state == BehaviorState::REST || state_.behavior_state == BehaviorState::STAND) {
            // В режиме отдыха и стойки все лапы на земле (как в Python)
            msg->contacts = {true, true, true, true};
        } else if (use_trot_) {
            Eigen::VectorXi contacts = trot_gait_->contacts(state_.ticks);
            msg->contacts = {contacts(0) != 0, contacts(1) != 0, contacts(2) != 0, contacts(3) != 0};
        } else if (use_crawl_) {
            Eigen::VectorXi contacts = crawl_gait_->contacts(state_.ticks);
            msg->contacts = {contacts(0) != 0, contacts(1) != 0, contacts(2) != 0, contacts(3) != 0};
        } else {
            msg->contacts = {true, true, true, true};
        }
        foot_contact_pub_->publish(std::move(msg));
    }

    void control_loop() {
        // Grace period при старте — ждём пока робот приземлится
        if (startup_grace_ > 0) {
            startup_grace_--;
            if (startup_grace_ == 0) {
                RCLCPP_INFO(get_logger(), "Startup grace period complete, controller active");
            }
            return;
        }

        // Run controller
        Eigen::MatrixXd leg_positions;
        if (use_trot_) {
            leg_positions = step_trot(state_, command_);
        } else if (use_crawl_) {
            leg_positions = step_crawl(state_, command_);
        } else if (use_stand_) {
            leg_positions = step_stand(state_, command_);
        } else {
            leg_positions = step_rest(state_, command_);
        }

        state_.foot_locations = leg_positions;
        state_.robot_height = command_.robot_height;

        if (controller_change_needed_) {
            change_controller();
            controller_change_needed_ = false;
        }

        // Publish foot contacts for odometry
        publish_foot_contacts();

        // IK debug — проверяем размеры
        if (state_.ticks < 5) {
            RCLCPP_INFO(get_logger(), "[IK DEBUG] leg_positions: %dx%d, dx=%.3f dy=%.3f dz=%.3f",
                        (int)leg_positions.rows(), (int)leg_positions.cols(), state_.body_local_position[0],
                        state_.body_local_position[1], state_.robot_height);
        }

        try {
            auto joint_angles =
                ik_->inverse_kinematics(leg_positions, state_.body_local_position[0], state_.body_local_position[1],
                                        state_.body_local_position[2], state_.body_local_orientation[0],
                                        state_.body_local_orientation[1], state_.body_local_orientation[2]);

            auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
            msg->data.assign(joint_angles.begin(), joint_angles.end());
            joint_pub_->publish(std::move(msg));

            // DEBUG: выводим каждые 60 тиков (раз в секунду)
            if (state_.ticks % 60 == 0) {
                RCLCPP_INFO(get_logger(),
                            "[DEBUG] cmd: vx=%.4f vy=%.4f vz=%.4f yaw=%.4f | "
                            "pos: x=%.4f y=%.4f z=%.4f | "
                            "joints[0-2]: %.4f %.4f %.4f",
                            command_.velocity[0], command_.velocity[1], command_.velocity[2], command_.yaw_rate[2],
                            state_.body_local_position[0], state_.body_local_position[1], state_.body_local_position[2],
                            joint_angles[0], joint_angles[1], joint_angles[2]);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "IK error: %s", e.what());
        }
    }

    // Members
    int rate_;
    bool verbose_;
    bool debug_mode_ = false;  // removed
    bool controller_change_needed_ = false;
    bool use_trot_ = false;
    bool use_crawl_ = false;
    bool use_stand_ = false;
    int startup_grace_ = 120;  // 2 секунды задержки при старте

    Eigen::MatrixXd default_stance_;
    State state_;
    Command command_;

    std::unique_ptr<TrotGaitController> trot_gait_;
    std::unique_ptr<CrawlGaitController> crawl_gait_;
    std::unique_ptr<RestController> rest_ctrl_;
    std::unique_ptr<StandController> stand_ctrl_;
    std::unique_ptr<InverseKinematics> ik_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<quadropted_msgs::msg::RobotFootContact>::SharedPtr foot_contact_pub_;
    rclcpp::Subscription<quadropted_msgs::msg::RobotVelocity>::SharedPtr velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<quadropted_msgs::msg::RobotModeCommand>::SharedPtr mode_sub_;
    rclcpp::Service<quadropted_msgs::srv::RobotBehaviorCommand>::SharedPtr behavior_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace quadropted

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<quadropted::RobotControllerNode>());
    rclcpp::shutdown();
    return 0;
}
