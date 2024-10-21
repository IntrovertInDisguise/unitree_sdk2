/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

enum test_mode
{
  /*---Basic motion---*/
  normal_stand,
  balance_stand,
  velocity_move,
  trajectory_follow,
  stand_down,
  stand_up,
  damp,
  recovery_stand,
  /*---Special motion ---*/
  sit,
  rise_sit,
  stretch,
  wallow,
  pose,
  scrape,
  front_flip,
  front_jump,
  front_pounce,
  stop_move = 99
};

// Modify this array to define the sequence of actions
const test_mode action_sequence[] = {
  normal_stand,
  rise_sit,
  pose,
  stand_down,
  stand_up,
  sit,
 normal_stand,
  wallow,
  rise_sit,
  stop_move
};

const int num_actions = sizeof(action_sequence) / sizeof(action_sequence[0]);

class Custom
{
public:
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  };

  void RobotControl()
  {
    ct += dt;
    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;

    // Switch action based on timer
    if (ct - action_time >= action_duration)
    {
      action_counter = (action_counter + 1) % num_actions; // Cycle through actions
      action_time = ct; // Reset timer
    }

    // Get the current action from the sequence
    test_mode current_action = action_sequence[action_counter];

    switch (current_action)
    {
    case normal_stand:            // 0. idle, default stand
      sport_client.SwitchGait(0); // 0:idle; 1:tort; 2:tort running; 3:climb stair; 4:tort obstacle
      sport_client.StandUp();
      break;

    case balance_stand:                  // 1. Balance stand (controlled by dBodyHeight + rpy)
      sport_client.Euler(0.1, 0.2, 0.3); // roll, pitch, yaw
      sport_client.BodyHeight(0.0);      // relative height [-0.18~0.03]
      sport_client.BalanceStand();
      break;

    case velocity_move: // 2. target velocity walking (controlled by velocity + yawSpeed)
      sport_client.Move(0.3, 0, 0.3);
      break;

    case trajectory_follow: // 3. path mode walking
      time_seg = 0.2;
      time_temp = ct - time_seg;
      for (int i = 0; i < 30; i++)
      {
        time_temp += time_seg;

        px_local = 0.5 * sin(0.5 * time_temp);
        py_local = 0;
        yaw_local = 0;
        vx_local = 0.5 * cos(0.5 * time_temp);
        vy_local = 0;
        vyaw_local = 0;

        path_point_tmp.timeFromStart = i * time_seg;
        path_point_tmp.x = px_local * cos(yaw0) - py_local * sin(yaw0) + px0;
        path_point_tmp.y = px_local * sin(yaw0) + py_local * cos(yaw0) + py0;
        path_point_tmp.yaw = yaw_local + yaw0;
        path_point_tmp.vx = vx_local * cos(yaw0) - vy_local * sin(yaw0);
        path_point_tmp.vy = vx_local * sin(yaw0) + vy_local * cos(yaw0);
        path_point_tmp.vyaw = vyaw_local;
        path.push_back(path_point_tmp);
      }
      sport_client.TrajectoryFollow(path);
      break;

    case stand_down: // 4. position stand down.
      sport_client.StandDown();
      break;

    case stand_up: // 5. position stand up
      sport_client.StandUp();
      break;

    case sit:
      sport_client.Sit();
      break;

    case rise_sit:
      sport_client.RiseSit();
      break;

    case stop_move: // stop move
      sport_client.StopMove();
      break;

    default:
      sport_client.StopMove();
    }
  };

  // Get initial position
  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
  };

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0; // Initial position and yaw
  double ct = 0;         // Runtime counter
  double action_time = 0; // Time when the current action started
  int action_counter = 0; // Current action index
  const double action_duration = 10.0; // Duration of each action in seconds
  float dt = 0.005;      // Control loop step size (0.001~0.01)
};

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;

  sleep(1); // Wait for 1 second to obtain a stable state

  custom.GetInitState(); // Get initial position
  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

  while (1)
  {
    sleep(10);
  }
  return 0;
}
