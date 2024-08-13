import React, { Component } from "react";
import { Joystick } from "react-joystick-component";
import Config from "../scripts/config";
import { Row, Col, Container, Card } from "react-bootstrap";
import * as Three from "three";
import "../Teleop.css";

class TeleopState extends Component {
  state = {
    ros: null,
    x: 0,
    y: 0,
    orientation: 0,
    linear_velocity: 0,
    angular_velocity: 0,
    connected: false,
  };

  constructor() {
    super();
    this.init_connection();
    this.handleMove = this.handleMove.bind(this);
    this.handleStop = this.handleStop.bind(this);
  }

  init_connection() {
    this.state.ros = new window.ROSLIB.Ros();
    console.log(this.state.ros);

    this.state.ros.on("connection", () => {
      console.log("connection established in Teleoperation Component!");
      console.log(this.state.ros);
      this.setState({ connected: true });
    });

    this.state.ros.on("close", () => {
      console.log("connection is closed!");
      this.setState({ connected: false });
      //try to reconnect every 3 seconds
      setTimeout(() => {
        try {
          this.state.ros.connect(
            "ws://" +
              Config.ROSBRIDGE_SERVER_IP +
              ":" +
              Config.ROSBRIDGE_SERVER_PORT +
              ""
          );
        } catch (error) {
          console.log("connection problem ");
        }
      }, Config.RECONNECTION_TIMER);
    });

    try {
      this.state.ros.connect(
        "ws://" +
          Config.ROSBRIDGE_SERVER_IP +
          ":" +
          Config.ROSBRIDGE_SERVER_PORT +
          ""
      );
    } catch (error) {
      console.log(
        "ws://" +
          Config.ROSBRIDGE_SERVER_IP +
          ":" +
          Config.ROSBRIDGE_SERVER_PORT +
          ""
      );
      console.log("connection problem ");
    }
  }

  handleMove(event) {
    console.log("handle move");
    //we need to create a ROS publisher on the topic cmd_vel
    var cmd_vel = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });
    //we need to create a twist message to be to published to rosbridge
    var twist = new window.ROSLIB.Message({
      linear: {
        x: event.y / 50,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: -event.x / 50,
      },
    });
    //we need to publish the message on the cmd_vel topic
    cmd_vel.publish(twist);
  }

  handleStop(event) {
    console.log("handle stop");
    //we need to create a ROS publisher on the topic cmd_vel
    var cmd_vel = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });
    //we need to create a twist message to be to published to rosbridge
    var twist = new window.ROSLIB.Message({
      linear: {
        x: 0,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: 0,
      },
    });
    //we need to publish the message on the cmd_vel topic
    cmd_vel.publish(twist);
  }

  componentDidMount() {
    this.getRobotState();
  }

  getRobotState() {
    //create a pose subscriber
    var pose_subscriber = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    //create a pose callback
    pose_subscriber.subscribe((message) => {
      this.setState({ x: message.pose.pose.position.x.toFixed(2) });
      this.setState({ y: message.pose.pose.position.y.toFixed(2) });
      this.setState({
        orientation: this.getOrientationFromQuaternion(
          message.pose.pose.orientation
        ).toFixed(2),
      });
    });

    //create a subscriber for the velocities in the odom topic
    var velocity_subscriber = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.ODOM_TOPIC,
      messageType: "nav_msgs/Odometry",
    });

    //callback function for the odom
    velocity_subscriber.subscribe((message) => {
      this.setState({
        linear_velocity: message.twist.twist.linear.x.toFixed(2),
      });
      this.setState({
        angular_velocity: message.twist.twist.angular.z.toFixed(2),
      });
    });
  }

  getOrientationFromQuaternion(ros_orientation_quaternion) {
    var q = new Three.Quaternion(
      ros_orientation_quaternion.x,
      ros_orientation_quaternion.y,
      ros_orientation_quaternion.z,
      ros_orientation_quaternion.w
    );
    //convert this quaternion into Roll, Pitch and Yaw
    var RPY = new Three.Euler().setFromQuaternion(q);

    return RPY["_z"] * (180 / Math.PI);
  }

  render() {
    return (
      <Container fluid className="teleop-card">
        <h1 className="teleop-header">Teleoperation and Robot State</h1>
        <Row>
          <Col md={4} className="d-flex align-items-center justify-content-center">
            <Joystick
              size={100}
              baseColor="#EEEEEE"
              stickColor="#BBBBBB"
              move={this.handleMove}
              stop={this.handleStop}
            />
          </Col>
          <Col md={4} className="teleop-features">
            <Card className="teleop-card same-size-card">
              <Card.Body>
                <h4 className="feature-title">Position</h4>
                <div className="feature-details">
                  <div className="feature-group">
                    <p className="feature">x: {this.state.x}</p>
                    <p className="feature">y: {this.state.y}</p>
                    <p className="feature">Orientation: {this.state.orientation}</p>
                  </div>
                </div>
              </Card.Body>
            </Card>
          </Col>
          <Col md={4} className="teleop-features">
            <Card className="teleop-card same-size-card">
              <Card.Body>
                <h4 className="feature-title">Velocities</h4>
                <div className="feature-details">
                  <div className="feature-group">
                    <p className="feature">Linear Velocity: {this.state.linear_velocity}</p>
                    <p className="feature">Angular Velocity: {this.state.angular_velocity}</p>
                  </div>
                </div>
              </Card.Body>
            </Card>
          </Col>
        </Row>
      </Container>
    );
  }
}

export default TeleopState;
