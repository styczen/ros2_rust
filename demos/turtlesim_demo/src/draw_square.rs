use anyhow::{Error, Result};
use geometry_msgs::msg::Twist as TwistMsg;
use std::sync::{Arc, Mutex};
use turtlesim::msg::Pose as TurtlePoseMsg;

const DIST_DIFF_EPS: f32 = 0.1;
const ANGLE_DIFF_EPS: f32 = 0.01;
const LINEAR_VEL_STOP: f32 = 0.0001;
const ANGULAR_VEL_STOP: f32 = 0.0001;

#[derive(Default, Debug)]
enum TurtleState {
    #[default]
    Idle,
    Forward,
    StopForward,
    Turn,
    StopTurn,
}

fn command_turtle(
    twist_pub: &rclrs::Publisher<TwistMsg>,
    linear: f64,
    angular: f64,
) -> Result<(), rclrs::RclrsError> {
    let mut twist = TwistMsg::default();
    twist.linear.x = linear;
    twist.angular.z = angular;
    twist_pub.publish(twist)
}

fn has_reached_goal(current_pose: &TurtlePoseMsg, goal_pose: &TurtlePoseMsg) -> bool {
    (goal_pose.x - current_pose.x).abs() < DIST_DIFF_EPS
        && (goal_pose.y - current_pose.y).abs() < DIST_DIFF_EPS
        && (goal_pose.theta - current_pose.theta).abs() < ANGLE_DIFF_EPS
}

fn has_stopped(current_pose: &TurtlePoseMsg) -> bool {
    current_pose.linear_velocity.abs() < LINEAR_VEL_STOP
        && current_pose.angular_velocity.abs() < ANGULAR_VEL_STOP
}

fn wrap_angle(angle: f32) -> f32 {
    if angle > std::f32::consts::PI {
        return angle - std::f32::consts::TAU;
    }
    return angle;
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(std::env::args())?;
    let mut node = rclrs::Node::new(&context, "draw_square")?;
    let mut state = TurtleState::default();

    let current_pose = Arc::new(Mutex::new(None));
    let current_pose_cb = Arc::clone(&current_pose);
    let _pose_sub = node.create_subscription(
        "turtle1/pose",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: TurtlePoseMsg| {
            *current_pose_cb.lock().unwrap() = Some(msg);
        },
    )?;
    let twist_pub =
        node.create_publisher::<TwistMsg>("turtle1/cmd_vel", rclrs::QOS_PROFILE_DEFAULT)?;

    let reset_done = Arc::new(Mutex::new(false));

    let reset_done_cb = Arc::clone(&reset_done);
    let reset_client = node.create_client::<std_srvs::srv::Empty>("reset")?;
    let empty = std_srvs::srv::Empty_Request::default();
    reset_client.async_send_request_with_callback(
        empty,
        move |_: std_srvs::srv::Empty_Response| {
            *reset_done_cb.lock().unwrap() = true;
        },
    )?;

    let mut goal_pose = TurtlePoseMsg::default();
    std::thread::spawn(move || -> Result<()> {
        loop {
            if !*reset_done.lock().unwrap() {
                continue;
            }

            {
                if let Some(ref curr_pose) = *current_pose.lock().unwrap() {
                    match state {
                        TurtleState::Idle => {
                            state = TurtleState::Forward;
                            goal_pose.x = curr_pose.x + curr_pose.theta.cos() * 2.0;
                            goal_pose.y = curr_pose.y + curr_pose.theta.sin() * 2.0;
                            goal_pose.theta = curr_pose.theta;
                        }
                        TurtleState::Forward => {
                            if has_reached_goal(&curr_pose, &goal_pose) {
                                println!("Reached goal ({:?})", goal_pose);
                                state = TurtleState::StopForward;
                                command_turtle(&twist_pub, 0.0, 0.0)?;
                            } else {
                                command_turtle(&twist_pub, 1.0, 0.0)?;
                            }
                        }
                        TurtleState::StopForward => {
                            if has_stopped(curr_pose) {
                                println!("Stopped");
                                state = TurtleState::Turn;
                                goal_pose.x = curr_pose.x;
                                goal_pose.y = curr_pose.y;
                                goal_pose.theta =
                                    wrap_angle(curr_pose.theta + std::f32::consts::FRAC_PI_2);
                            } else {
                                command_turtle(&twist_pub, 0.0, 0.0)?;
                            }
                        }
                        TurtleState::Turn => {
                            if has_reached_goal(&curr_pose, &goal_pose) {
                                println!("Reached goal ({:?})", goal_pose);
                                state = TurtleState::StopTurn;
                                command_turtle(&twist_pub, 0.0, 0.0)?;
                            } else {
                                command_turtle(&twist_pub, 0.0, 0.4)?;
                            }
                        }
                        TurtleState::StopTurn => {
                            if has_stopped(curr_pose) {
                                println!("Stopped");
                                state = TurtleState::Forward;
                                goal_pose.x += curr_pose.theta.cos() * 2.0;
                                goal_pose.y += curr_pose.theta.sin() * 2.0;
                                goal_pose.theta = curr_pose.theta;
                            } else {
                                command_turtle(&twist_pub, 0.0, 0.0)?;
                            }
                        }
                    }
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(16));
        }
    });

    rclrs::spin(&node)?;
    Ok(())
}
