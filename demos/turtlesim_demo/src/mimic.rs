use anyhow::{Error, Result};
use std::env;

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "mimic")?;

    let cmd_pub = node.create_publisher::<geometry_msgs::msg::Twist>(
        "output/cmd_vel",
        rclrs::QOS_PROFILE_DEFAULT,
    )?;
    let _pose_sub = node.create_subscription(
        "input/pose",
        rclrs::QOS_PROFILE_DEFAULT,
        move |msg: turtlesim::msg::Pose| {
            let mut twist = geometry_msgs::msg::Twist::default();
            twist.angular.z = msg.angular_velocity as f64;
            twist.linear.x = msg.linear_velocity as f64;
            match cmd_pub.publish(twist) {
                Err(_) => println!("Cannot publish twist message"),
                _ => (),
            }
        },
    )?;

    rclrs::spin(&node).map_err(|err| err.into())
}
