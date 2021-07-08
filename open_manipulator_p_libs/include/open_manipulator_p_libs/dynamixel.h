/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#if defined(__OPENCR__)
  #include <RobotisManipulator.h>
  #include <DynamixelWorkbench.h>
#else
  #include <robotis_manipulator/robotis_manipulator.h>
  #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#endif

namespace dynamixel
{

#define SYNC_WRITE_HANDLER 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

//#define CONTROL_LOOP_TIME 10;    //ms

// Protocol 2.0 
// SEEDR: Modified to use the "Indirect Data" Addresses
#define ADDR_PRESENT_CURRENT_2  634 // shall be configured as "Ind Data 1" (2 bytes)
#define ADDR_PRESENT_VELOCITY_2 636 // shall be configured as "Ind Data 3" (4 bytes)
#define ADDR_PRESENT_POSITION_2 640 // shall be configured as "Ind Data 7" (4 bytes)

// Note SEEDR June 2021: Modified to use Indirect Data addresses for GOAL POSITION
// the other memory addresses don't seem to be used
#define ADDR_GOAL_POSITION_2 658
//#define ADDR_GOAL_POSITION_2 564 //116
#define ADDR_VELOCITY_TRAJECTORY_2 584 //136
#define ADDR_POSITION_TRAJECTORY_2 588 //140
#define ADDR_PROFILE_ACCELERATION_2 556 //108
#define ADDR_PROFILE_VELOCITY_2 560 //112


// SEEDR April 2021: hardcode indirect data address as they're not covered
// in the workbench tools
#define ADDR_PRO_INDIRECT_ADDR_1_LOW 	  168

/* To mirror goal position */
#define ADDR_PRO_INDIRECT_ADDR_25_LOW 	216



// SEEDR; extended support fo X series
#define ADDR_XC_XM_XH_INDIRECT_ADDR_29_LOW 	578 // in the XM ctrl table, Position 640 reads Ind Data 35, so we need to
												// map to that position accordingly
/* To mirror goal position */
#define ADDR_XC_XM_XH_INDIRECT_ADDR_53_LOW 	626


#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4
#define LENGTH_VELOCITY_TRAJECTORY_2 4
#define LENGTH_POSITION_TRAJECTORY_2 4
#define LENGTH_PROFILE_ACCELERATION_2 4
#define LENGTH_PROFILE_VELOCITY_2 4
#define LENGTH_GOAL_POSITION_2 4

// DO not support anymore
// Protocol 1.0
// #define ADDR_PRESENT_CURRENT_1 = 40;
// #define ADDR_PRESENT_VELOCITY_1 = 38;
// #define ADDR_PRESENT_POSITION_1 = 36;

// #define LENGTH_PRESENT_CURRENT_1 = 2;
// #define LENGTH_PRESENT_VELOCITY_1 = 2;
// #define LENGTH_PRESENT_POSITION_1 = 2;

typedef struct
{
  std::vector<uint8_t> id;
  uint8_t num;
} Joint;

class JointDynamixel : public robotis_manipulator::JointActuator
{
private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

public:
  JointDynamixel(){}
  virtual ~JointDynamixel(){}


  /*****************************************************************************
  ** Joint Dynamixel Control Functions
  *****************************************************************************/
  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);
  void configureCtrlTableIndirection(uint8_t id, const char* wb_toolbox_param_name, uint16_t* configuration_address); // SeedR


  /*****************************************************************************
  ** Functions called in Joint Dynamixel Control Functions
  *****************************************************************************/
  bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
  bool writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector);
  std::vector<robotis_manipulator::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
};

class JointDynamixelProfileControl : public robotis_manipulator::JointActuator
{
private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;
  float control_loop_time_; // unit: ms
  std::map<uint8_t, robotis_manipulator::ActuatorValue> previous_goal_value_;

public:
  JointDynamixelProfileControl(float control_loop_time = 0.010);
  virtual ~JointDynamixelProfileControl(){}


  /*****************************************************************************
  ** Joint Dynamixel Profile Control Functions
  *****************************************************************************/
  virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
  virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
  virtual std::vector<uint8_t> getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  virtual std::vector<robotis_manipulator::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);


  /*****************************************************************************
  ** Functions called in Joint Dynamixel Profile Control Functions
  *****************************************************************************/
  bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
  bool setSDKHandler(uint8_t actuator_id);
  bool writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value);
  bool writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector);
  std::vector<robotis_manipulator::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
};

class GripperDynamixel : public robotis_manipulator::ToolActuator
{
 private:
  DynamixelWorkbench *dynamixel_workbench_;
  Joint dynamixel_;

 public:
  GripperDynamixel() {}
  virtual ~GripperDynamixel() {}


  /*****************************************************************************
  ** Tool Dynamixel Control Functions
  *****************************************************************************/
  virtual void init(uint8_t actuator_id, const void *arg);
  virtual void setMode(const void *arg);
  virtual uint8_t getId();

  virtual void enable();
  virtual void disable();

  virtual bool sendToolActuatorValue(robotis_manipulator::ActuatorValue value);
  virtual robotis_manipulator::ActuatorValue receiveToolActuatorValue();


  /*****************************************************************************
  ** Functions called in Tool Dynamixel Profile Control Functions
  *****************************************************************************/
  bool initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
  bool setOperatingMode(STRING dynamixel_mode = "position_mode");
  bool writeProfileValue(STRING profile_mode, uint32_t value);
  bool setSDKHandler();
  bool writeGoalPosition(double radian);
  double receiveDynamixelValue();
};

} // namespace DYNAMIXEL
#endif // DYNAMIXEL_H_




