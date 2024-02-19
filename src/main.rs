use safe_drive::{
    context::Context, error::DynError, logger::Logger, msg::common_interfaces::geometry_msgs::msg::Twist, topic::{subscriber, publisher::Publisher},
};

use safe_drive::msg::common_interfaces::geometry_msgs::msg;
use drobo_interfaces::msg::MdLibMsg;
use std::f64::consts::PI;

const   MECANUNM_DIA:f64 = 0.152;
const   ROBOT_CENTER_TO_WHEEL_DISTANCE:f64 = 0.37; // ロボットの重心からメカナムホイールまでの距離


fn main() -> Result<(), DynError>{

    // for debug
    let _logger = Logger::new("omni_controll");


    let ctx = Context::new()?;
    let node = ctx.create_node("omni_control", None, Default::default())?;
    let subscriber = node.create_subscriber::<msg::Twist>("cmd_vel", None)?;
    let publisher = node.create_publisher::<drobo_interfaces::msg::MdLibMsg>("md_driver_topic", None)?;
    let mut selector = ctx.create_selector()?;
    
    selector.add_subscriber(
        subscriber,
        {
            Box::new(move |msg| {
                let topic_callback_data = topic_callback(msg);

                move_chassis(topic_callback_data[0],topic_callback_data[1],topic_callback_data[2],&publisher);
            })
        },
    );


    loop {
        selector.wait()?;
    }
}


fn topic_callback(msg: subscriber::TakenMsg<Twist>) -> [f64;3]{

        let yrpm:f64 = msg.linear.x / (2. * PI * MECANUNM_DIA) * 60.;
        let xrpm:f64 = msg.linear.y / (2. * PI * MECANUNM_DIA) * 60.;
        let yaw:f64 = msg.angular.z;


        [yrpm, -xrpm, -yaw]
}

fn move_chassis(_yrpm:f64,_xrpm:f64,_yaw:f64,publisher:&Publisher<MdLibMsg>){


    let mut wheels_rpm:[f64;4] = [0.;4];
    let speed_abs:f64 = (_xrpm.powf(2.) + _yrpm.powf(2.)).sqrt();
    let radwimps:f64 = _yrpm.atan2(_xrpm);

    wheels_rpm[0] = ((speed_abs * radwimps.sin()) + (speed_abs * radwimps.sin()) + (2.*2_f64.sqrt()*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4. * PI * MECANUNM_DIA;
    wheels_rpm[1] = ((speed_abs * radwimps.sin()) - (speed_abs * radwimps.sin()) - (2.*2_f64.sqrt()*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4. * PI * MECANUNM_DIA;
    wheels_rpm[2] = ((speed_abs * radwimps.sin()) + (speed_abs * radwimps.sin()) - (2.*2_f64.sqrt()*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4. * PI * MECANUNM_DIA;
    wheels_rpm[3] = ((speed_abs * radwimps.sin()) - (speed_abs * radwimps.sin()) + (2.*2_f64.sqrt()*_yaw*ROBOT_CENTER_TO_WHEEL_DISTANCE)) / 4. * PI * MECANUNM_DIA;


    for i in 0..4{
        _send_pwm(i as u8 ,0,wheels_rpm[i] > 0., wheels_rpm[i] as u16, publisher);

    }

}

fn _send_pwm(_address:u8, _semi_id:u8,_phase:bool,_power:u16,publisher:&Publisher<MdLibMsg>){
    let mut msg = drobo_interfaces::msg::MdLibMsg::new().unwrap();
    msg.address = _address as u8;
    msg.semi_id = _semi_id as u8;
    msg.mode = 2 as u8; //MotorLibのPWMモードに倣いました
    msg.phase = _phase as bool;
    msg.power = _power as u16;

    publisher.send(&msg).unwrap()

}