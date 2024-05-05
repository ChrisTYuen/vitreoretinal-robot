"""
struct MoonshotDrillRobotConfiguration
{
    bool use_real_robot;

    bool vrep_readonly;
    bool vrep_send_initial_posture;
    std::string vrep_robot_name;
    std::string vrep_ip;
    int vrep_port;

    std::string robot_parameter_file_path;

    int thread_sampling_time_nsec;

    rosilo::RobotDriverDensoConfiguration robot_driver_denso_configuration;
};
"""


class MoonshotDrillRobotConfiguration:

    def __init__(self):
        self.use_real_robot = None

        self.vrep_readonly = None
        self.vrep_send_initial_posture = None
        self.vrep_robot_name = None
        self.vrep_ip = None
        self.vrep_port = None

        self.robot_parameter_file_path = None

        self.thread_sampling_time_nsec = None

        self.robot_driver_denso_configuration = None

        self.vrep_robot = None
