use crossterm::{
    event::{read, Event, KeyCode, KeyEvent},
    terminal::{disable_raw_mode, enable_raw_mode},
};

const INTRO: &str = "Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
'Q' to quit.";

struct TeleopTurtleNode {
    _node: rclrs::Node,
    twist_pub: rclrs::Publisher<geometry_msgs::msg::Twist>,
}

impl TeleopTurtleNode {
    fn new(ctx: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(ctx, "teleop_turtle")?;
        let twist_pub = node.create_publisher::<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel",
            rclrs::QOS_PROFILE_DEFAULT,
        )?;
        Ok(Self {
            _node: node,
            twist_pub,
        })
    }

    fn publish_twist(&self, linear: f64, angular: f64) -> Result<(), rclrs::RclrsError> {
        let mut twist_msg = geometry_msgs::msg::Twist::default();
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        self.twist_pub.publish(twist_msg)
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("{}", INTRO);
    let context = rclrs::Context::new(std::env::args())?;
    let node = TeleopTurtleNode::new(&context)?;

    enable_raw_mode()?;

    loop {
        let event = read()?;

        let mut linear = 0.0;
        let mut angular = 0.0;

        match event {
            Event::Key(KeyEvent {
                code: KeyCode::Left,
                ..
            }) => angular = 1.0,
            Event::Key(KeyEvent {
                code: KeyCode::Right,
                ..
            }) => angular = -1.0,
            Event::Key(KeyEvent {
                code: KeyCode::Up, ..
            }) => linear = 1.0,
            Event::Key(KeyEvent {
                code: KeyCode::Down,
                ..
            }) => linear = -1.0,
            Event::Key(KeyEvent {
                code: KeyCode::Char('q'),
                ..
            }) => break,
            _ => (),
        }

        if linear != 0.0 || angular != 0.0 {
            node.publish_twist(linear, angular)?;
        }
    }

    disable_raw_mode()?;
    Ok(())
}
