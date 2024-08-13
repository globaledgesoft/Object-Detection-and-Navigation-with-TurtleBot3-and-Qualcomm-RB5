import React, { Component } from "react";
import Connection from "./Connection";
import RobotState from "./RobotState";
import Teleoperation from "./Teleoperation";
import TeleopState  from "./TeleopState";
import Map from "./Map";
import RobotState1 from "./RobotState1";
import { Row, Col, Container, Button } from "react-bootstrap";
class Home extends Component {
  state = {};

  render() {
    return (
      <div>
        <Container>
          <h1 className="text-center mt-3">Robot Control Page</h1>
          <Row>
            <Col>
              <Connection />
            </Col>
          </Row>
          <Row>
            <Col>
              
              <TeleopState />
              
            </Col>
          </Row>
          <Row>
            {" "}
            <Col>

              <RobotState1/>
            </Col>
            <Col>
              <h1>MAP</h1>
              <Map></Map>
              
            </Col>
          </Row>

        </Container>
      </div>
    );
  }
}

export default Home;
