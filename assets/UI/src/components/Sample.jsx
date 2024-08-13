import React, { Component } from "react";
import Config from "../scripts/config";
import { Row, Col, Container, Card, Button } from "react-bootstrap";
import * as Three from "three";
import "../art.css";

class Sample extends Component {
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

  move(direction) {
    let linear_x = 0;
    let angular_z = 0;

    switch (direction) {
      case "up":
        linear_x = 0.5;
        break;
      case "down":
        linear_x = -0.5;
        break;
      case "left":
        angular_z = 0.5;
        break;
      case "right":
        angular_z = -0.5;
        break;
      default:
        break;
    }

    var cmd_vel = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });

    var twist = new window.ROSLIB.Message({
      linear: {
        x: linear_x,
        y: 0,
        z: 0,
      },
      angular: {
        x: 0,
        y: 0,
        z: angular_z,
      },
    });

    cmd_vel.publish(twist);
  }

  stop() {
    var cmd_vel = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });

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

    cmd_vel.publish(twist);
  }

  componentDidMount() {
    this.getRobotState();
  }

  getRobotState() {
    var pose_subscriber = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    pose_subscriber.subscribe((message) => {
      this.setState({ x: message.pose.pose.position.x.toFixed(2) });
      this.setState({ y: message.pose.pose.position.y.toFixed(2) });
      this.setState({
        orientation: this.getOrientationFromQuaternion(
          message.pose.pose.orientation
        ).toFixed(2),
      });
    });

    var velocity_subscriber = new window.ROSLIB.Topic({
      ros: this.state.ros,
      name: Config.ODOM_TOPIC,
      messageType: "nav_msgs/Odometry",
    });

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

    var RPY = new Three.Euler().setFromQuaternion(q);

    return RPY["_z"] * (180 / Math.PI);
  }

  render() {
    return (
      <Container fluid className="teleop-card">
        <h1 className="teleop-header">Teleoperation and Robot State</h1>
        <Row>
          <Col md={4} className="d-flex flex-column align-items-center justify-content-center">
            <div className="directional-buttons">
              <Button
                variant="dark"
                className="direction-button"
                onMouseDown={() => this.move("up")}
                onMouseUp={() => this.stop()}
              >
                &#9650;
              </Button>
              <div>
                <Button
                  variant="dark"
                  className="direction-button"
                  onMouseDown={() => this.move("left")}
                  onMouseUp={() => this.stop()}
                >
                  &#9664;
                </Button>
                <Button
                  variant="dark"
                  className="direction-button"
                  onMouseDown={() => this.move("right")}
                  onMouseUp={() => this.stop()}
                >
                  &#9654;
                </Button>
              </div>
              <Button
                variant="dark"
                className="direction-button"
                onMouseDown={() => this.move("down")}
                onMouseUp={() => this.stop()}
              >
                &#9660;
              </Button>
            </div>
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

export default Sample;
