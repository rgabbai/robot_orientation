// ROS Client Lib 
use std::sync::Arc;
use rclrust::{qos::QoSProfile, rclrust_info};
use rclrust_msg::std_msgs::msg::String as String_;
//use rclrust_msg::sensor_msgs::msg::CompressedImage;
use rclrust_msg::sensor_msgs::msg::Imu as Imu_;
//use rclrust_msg::std_msgs::msg::String as String_;

//use std::sync::Mutex; // to pass shared data between threads
use nalgebra::{Quaternion, UnitQuaternion};
use serde_json;
use serde_derive::Serialize;
use serde_derive::Deserialize;

//use serde::{Serialize, Deserialize};
//use serde_json;

//#[derive(Serialize, Deserialize, Clone, Debug)]

const SUB_TOPIC_NAME: &str  = "imu/data"; 
const PUB_TOPIC_NAME: &str  = "robot_orientation/data"; 

#[derive(Serialize, Deserialize, Debug)]
struct DataOrien {
    yaw:f32,
    pitch:f32,
    roll:f32,
}

use anyhow::Result;

#[tokio::main]
async fn main() -> Result<()>  {
    // Initialize the ROS2 client library
    println!("Robot orientation node: Generate yaw,pitch and role data");
    let ctx = rclrust::init()?;
    let mut node = ctx.create_node("robot_orientation")?;
    let logger = node.logger();

    let publisher = node.create_publisher::<String_>(PUB_TOPIC_NAME, &QoSProfile::default())?;   // robot orientation publisher

    let _subscription = node.create_subscription(
        SUB_TOPIC_NAME,
        move |msg: Arc<Imu_>| {
            // Convert quaternion to Euler angles and publish
            let orient = process_imu_data(&*msg);
             // Print the processed orientation data
            //println!("Process IMU data: {:?}", orient);

            let serialized_data = match serde_json::to_string(&orient) {
                    Ok(data) => data,
                    Err(e) => {
                        eprintln!("Failed to serialize detected data: {}", e);
                        return;  // Don't proceed if serialization fails
                    }
            };
            
            let   message = String_ {
               data: serialized_data,
            };
            //println!("Publishing: '{}'", message.data);
            rclrust_info!(logger, "Publishing: '{}'", message.data);
            publisher.publish(&message).unwrap();
        },
        &QoSProfile::default(),
    )?;

    node.wait();
    Ok(())  
}

fn process_imu_data(msg: &Imu_) -> DataOrien {
    // Extract the quaternion from the IMU message
    let q = Quaternion::new(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);

    // Normalize the quaternion
    let norm_quat = UnitQuaternion::from_quaternion(q.normalize());

    // Convert quaternion to Euler angles
    let euler_angles = norm_quat.euler_angles();

    // Convert to degrees according to camera Axis
    let pitch_deg = euler_angles.0.to_degrees();      // X left
    let roll_deg = euler_angles.1.to_degrees();    // Z forward
    let yaw_deg = euler_angles.2.to_degrees();      // Y axis down

    // Create and return the Float64MultiArray message
      // Create and return the DataOrien instance
    DataOrien {
        yaw: yaw_deg as f32,
        pitch: pitch_deg as f32,
        roll: roll_deg as f32,
    }
}


