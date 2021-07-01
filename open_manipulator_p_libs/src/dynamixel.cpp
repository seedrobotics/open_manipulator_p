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

#include "../include/open_manipulator_p_libs/dynamixel.h"

using namespace dynamixel;
using namespace robotis_manipulator;
 
/*****************************************************************************
** Joint Dynamixel Control Functions
*****************************************************************************/
void JointDynamixel::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = JointDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void JointDynamixel::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  bool result = false;
  // const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = JointDynamixel::setOperatingMode(actuator_id, get_arg_[0]);
    if (result == false)
      return;

    result = JointDynamixel::setSDKHandler(actuator_id.at(0));
    if (result == false)
      return;
  }
  else
  {
    result = JointDynamixel::writeProfileValue(actuator_id, get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }
  return;
}

std::vector<uint8_t> JointDynamixel::getId()
{
  return dynamixel_.id;
}

void JointDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;
  
  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = true;
}

void JointDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;
  
  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = false;
}

bool JointDynamixel::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;

  std::vector<double> radian_vector;
  for(uint32_t index = 0; index < value_vector.size(); index++)
  {
    radian_vector.push_back(value_vector.at(index).position);
  }
  result = JointDynamixel::writeGoalPosition(actuator_id, radian_vector);
  if (result == false)
    return false;

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixel::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  return JointDynamixel::receiveAllDynamixelValue(actuator_id);
}


/*****************************************************************************
** Functions called in Joint Dynamixel Control Functions
*****************************************************************************/
bool JointDynamixel::initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  bool result = false;
  const char* log = NULL;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id = actuator_id;
  dynamixel_.num = actuator_id.size();

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error(log);
  }    

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    result = dynamixel_workbench_->ping(id, &get_model_number, &log);

    if (result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel ID");
    }
    else
    {
      char str[100];
      sprintf(str, "Joint Dynamixel ID : %d, Model Name : %s", id, dynamixel_workbench_->getModelName(id));
      log::println(str);

      //// PRO Plus DYLs do not use Velocity based profile.
      // result = dynamixel_workbench_->setVelocityBasedProfile(id, &log);
      // if(result == false)
      // {
      //   log::error(log);
      //   log::error("Please check your Dynamixel firmware version (v38~)");
      // }


	  /* SEED ROBOTICS: March 2021 *******
	  We need to find a way to match the control table of the PRO units with the one in SEED units
	  - In PRO units, Present Current, Velocity and position start at memory index 574 and take a length of 10 bytes
	  - SEED units' control table goes up to address 255 maximum. When an address above 255 is requested, it is truncated to the LOW BYTE
	  
	  Therefore, we need to find a match where the LOW byte of (Current, Velocity, POsition) triplet matches on both PRO and SEED units.
	  
	  We can achieve this with address 640 to 649.
	  - In PRO units: 640 to 649 are indirect memory addresses ("Ind. Data 7" to "Ind Data 16"). 
		These mirror the values of the memory addresses that we configure at positions 180 onwards.
		We can abstractly view this as positions 180 onwards holding the "pointer" to the value we want to read and positions 640 onwards as the "de-referenced pointers"
		holding the value that they point to.
	  - The LOWBYTE(640) = 128, so in SEED units, the triplet (Current, Velocity, POsition) starts at position 128 all the way to 137, because Seed units
	    at present only look at the LOW value of the Address (say a request to read position 640, would ignore the HIGH Byte, and the low byte of 640=128).
	   (these are also implemented as pointers in SEED units, but they're not user-configurable; they're hardcoded to allow integration into the Manipulator framework; actuator FW version >= 37 needed )
	  
	  - GOAL POSITION is added/mirrored after this triplet as a 4 byte value as well after the above, at positions 650 to 653
		In Seed units this stays at position 138 to 141. Bear in mind that while this position in Seed units is a 4 byte 
		position, the data range remains the same 0~4095. The use of 4 bytes is for compatibility purposes only.
	  
	  For PRO and XM units, setup the pointers to mirror the data to addresses 640 to 649. 
	  HEADS UP: bc memory addresses in PRO units are 2 byte (WORD), we configure the pointers in positions 180~199 (each pointer address is 2 bytes)
	  but the actual mirrored value is always 1 byte (we provide the address for each of the bytes and the address is a 2 byte word).
	  After configuration, the values are then shown in positions 640~649.

	  See Robotis e-Manual for more information.

	  *** 
	  We'll now remap sequence of addresses 574-583 which corresponds to the triplet (Current, Velocity, Position), to positions 640~649 ("Ind.Data7" ~ "Ind.Data16").
	  We shall NOT do this for SEED units bc this redirection is hardcoded in the fw
	  */
	  
	  // TORQUE must be off to be able to manipulate configurations of the actuator
	  // See e-Manual
	  //disable();
	  
	  const char *model_name_= dynamixel_workbench_->getModelName(id, &log);
	  uint16_t start_addr_ = 0;
	  uint16_t current_ind_cfg_addr_ = 0;
	  
	  /* SEEDR: this may be extended to include other families of actuators; */
	  if (strncmp(model_name_, "SEED", strlen("SEED")) == 0) {
		  // don't assign an address; seed units are already hard coded with this indirection
		  start_addr_ = 0;
		  
	  } else if ( 	strncmp(model_name_, "XC", strlen("XC")) == 0 ||
					strncmp(model_name_, "XM", strlen("XM")) == 0 ||
					strncmp(model_name_, "XH", strlen("XH")) == 0   ) {
		start_addr_ = ADDR_XC_XM_XH_INDIRECT_ADDR_35_LOW;
			
	  } else if ( 	strncmp(model_name_, "PRO", strlen("PRO")) == 0 ) {
		start_addr_ = ADDR_PRO_INDIRECT_ADDR_7_LOW;  
		
	  } else {
		STRING formatted_msg = "INDIRECT DATA SETUP: The type of actuator is unhandled in dynamixel.cpp. Inderect data won't be at addresses 640 onwards. Device ID " + std::to_string(id) + " model name " + model_name_;
		log::error(formatted_msg.c_str());
	  }	  
	  
	  if (start_addr_ != 0) {
		  current_ind_cfg_addr_ = start_addr_;
		  
		  STRING present_current_st = "Present_Current";		  
		  const char* present_current_chr = present_current_st.c_str();
		  configureCtrlTableIndirection(id, present_current_chr, &current_ind_cfg_addr_);		  
		  
		  STRING present_velocity_st = "Present_Velocity";	
		  const char* present_velocity_chr = present_velocity_st.c_str();
		  configureCtrlTableIndirection(id, present_velocity_chr, &current_ind_cfg_addr_);

		  STRING present_position_st = "Present_Position";		  
		  const char* present_position_chr = present_position_st.c_str();
		  configureCtrlTableIndirection(id, present_position_chr, &current_ind_cfg_addr_);
		  
		  // must redirect GOAL POSITION as well bc it is written with SYNC_WRITE which requires
		  // the same Ctrl address on all devices.
		  STRING goal_position_st = "Goal_Position";		  
		  const char* goal_position_chr = goal_position_st.c_str();
		  configureCtrlTableIndirection(id, goal_position_chr, &current_ind_cfg_addr_);		  
		  
		  // Based on PRO units, we assume 2 bytes for current and 4 bytes for velocity 
		  // and 4 bytes for position. Check that the end address has advanced 10 WORD positions (20 bytes)
		  // if not, then we will be misaligned (maybe the workbench abstraction provided
		  // a parameter with only 2 bytes ?? or more??). Be verbose just in case
		  if (current_ind_cfg_addr_ != start_addr_ + 28) {
			  log::error("SEED mods: Error setting up Data Indirection in the control table.");
			  STRING formatted_msg = "The length of addresses configured (data provided via dyamixel_toolbox abstraction) was expected to be 28 but was  " + std::to_string(current_ind_cfg_addr_ - start_addr_);
			  
			  log::error(formatted_msg.c_str());
		  }
	  }	  

      result = dynamixel_workbench_->writeRegister(id, return_delay_time_char, 0, &log);
      if (result == false)
      {
        log::error(log);
        log::error("Unable to set Return Delay time. Please check your Dynamixel firmware version");
      }
    }
  }
  return true;
}

bool JointDynamixel::setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t acceleration = 0;
  const uint32_t current = 0;

  if (dynamixel_mode == "position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->currentBasedPositionMode(actuator_id.at(num), current, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }

  return true;
}

bool JointDynamixel::setSDKHandler(uint8_t actuator_id)
{
  bool result = false;
  const char* log = NULL;

  /* SeedR, July 2021: modified writing t Goal position to use Indirect addressing as well
    so that uniformity can be achieved when using different series of servos */
  //result = dynamixel_workbench_->addSyncWriteHandler(actuator_id, "Goal_Position", &log);
  result = dynamixel_workbench_->addSyncWriteHandler(ADDR_GOAL_POSITION_2, LENGTH_GOAL_POSITION_2, &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(ADDR_PRESENT_CURRENT_2, 
                                                    (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2), 
                                                    &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool JointDynamixel::writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  for (uint8_t num = 0; num < actuator_id.size(); num++)
  {
    result = dynamixel_workbench_->writeRegister(actuator_id.at(num), char_profile_mode, value, &log);
    if (result == false)
    {
      log::error(log);
    }
  }

  return true;
}

bool JointDynamixel::writeGoalPosition(std::vector<uint8_t> actuator_id, std::vector<double> radian_vector)
{
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[actuator_id.size()];
  int32_t goal_position[actuator_id.size()];

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    id_array[index] = actuator_id.at(index);
    goal_position[index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), radian_vector.at(index));
  }

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER, id_array, actuator_id.size(), goal_position, 1, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixel::receiveAllDynamixelValue(std::vector<uint8_t> actuator_id)
{
  bool result = false;
  const char* log = NULL;

  std::vector<robotis_manipulator::ActuatorValue> all_actuator;

  uint8_t id_array[actuator_id.size()];
  for (uint8_t index = 0; index < actuator_id.size(); index++)
    id_array[index] = actuator_id.at(index);

  int32_t get_current[actuator_id.size()];
  int32_t get_velocity[actuator_id.size()];
  int32_t get_position[actuator_id.size()];

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          actuator_id.size(),
                                          &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                actuator_id.size(),
                                                ADDR_PRESENT_CURRENT_2,
                                                LENGTH_PRESENT_CURRENT_2,
                                                get_current,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_VELOCITY_2,
                                                LENGTH_PRESENT_VELOCITY_2,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_POSITION_2,
                                                LENGTH_PRESENT_POSITION_2,
                                                get_position,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    robotis_manipulator::ActuatorValue actuator;
    actuator.effort = dynamixel_workbench_->convertValue2Current(get_current[index]);
    actuator.velocity = dynamixel_workbench_->convertValue2Velocity(actuator_id.at(index), get_velocity[index]);
    actuator.position = dynamixel_workbench_->convertValue2Radian(actuator_id.at(index), get_position[index]);

    all_actuator.push_back(actuator);
  }

  return all_actuator;
}
/****************************************
 SEED code to assist in configuring indirection
 while still relying on the toolbox abstraction
 attempt to configure indirection by relying on the Dynamixel Wokrbench
 abstraction layer as much as possible
 
 id = ID of the servo
 wb_toolbox_param_name = name of the parameter for which we're configuring indirection
						the actual address of the parameter in the device ctrl table is
						pulled "by name" via the wrobench abstraction layer
indirection_cfg_address = 	the address where we configure the Indirection
							aka "Indirect Adddress X"
							(actual data is then shown at address of "Indirect Data X")
							
Indirection_cg_address may be passed by reference and this way we are
always up to date on the last address used as it's incremented in this function.							
*****************************************/
void JointDynamixel::configureCtrlTableIndirection(uint8_t id, const char* wb_toolbox_param_name, uint16_t* indirection_cfg_address) {		  
	const char* log = NULL;
	
	const ControlItem* current_ctrl_item_ = dynamixel_workbench_->getItemInfo(id, wb_toolbox_param_name, &log);
	if (current_ctrl_item_ == NULL)
	{
		log::error(log);

		STRING formated_error = "Unable to get control item \"" + STRING(wb_toolbox_param_name) + "\" for ID " + std::to_string(id) + ". File: " + __FILE__;
		log::error(formated_error.c_str());
	}	
	else {
	  uint16_t original_addr_to_mirror_ = current_ctrl_item_->address;
	  uint8_t len_to_mirror_   = current_ctrl_item_->data_length;
	  uint8_t data_[2];
	  
	  while (len_to_mirror_ > 0) {
		  data_[0] = (uint8_t) (original_addr_to_mirror_ % 256);
		  data_[1] = (uint8_t) (original_addr_to_mirror_ / 256);		  
		  
		  bool result = dynamixel_workbench_->writeRegister(id, *indirection_cfg_address, 2, data_, &log);
		  if (result == false)
		  {
			log::error(log);
			STRING formated_error = "Failure configuring Indirect Data address \"" + std::to_string(*indirection_cfg_address) + "\" for ID " + std::to_string(id) + ". File: " + __FILE__;
			log::error(formated_error.c_str());				
		  }
		  
		  // loop control variables
		  original_addr_to_mirror_++;
		  (*indirection_cfg_address) += 2; // each indrect Adress is configured as a 2 byte (WORD) address, bc Ctl Table addresses in DYN2 are always 2 bytes
		  len_to_mirror_--;
	  }
	}
}

/*****************************************************************************
** Joint Dynamixel Profile Control Functions
*****************************************************************************/
JointDynamixelProfileControl::JointDynamixelProfileControl(float control_loop_time)
{
  control_loop_time_ = control_loop_time;
}

void JointDynamixelProfileControl::init(std::vector<uint8_t> actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = JointDynamixelProfileControl::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void JointDynamixelProfileControl::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
  bool result = false;
  // const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = JointDynamixelProfileControl::setOperatingMode(actuator_id, get_arg_[0]);
    if (result == false)
      return;

    result = JointDynamixelProfileControl::setSDKHandler(actuator_id.at(0));
    if (result == false)
      return;
  }
  else
  {
    result = JointDynamixelProfileControl::writeProfileValue(actuator_id, get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }
  return;
}

std::vector<uint8_t> JointDynamixelProfileControl::getId()
{
  return dynamixel_.id;
}

void JointDynamixelProfileControl::enable()
{
  const char* log = NULL;
  bool result = false;

  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = true;
}

void JointDynamixelProfileControl::disable()
{
  const char* log = NULL;
  bool result = false;

  for (uint32_t index = 0; index < dynamixel_.num; index++)
  {
    result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(index), &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  enabled_state_ = false;
}

bool JointDynamixelProfileControl::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;

  result = JointDynamixelProfileControl::writeGoalProfilingControlValue(actuator_id, value_vector);
  if (result == false)
    return false;

  return true;
}

std::vector<robotis_manipulator::ActuatorValue> JointDynamixelProfileControl::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
  return JointDynamixelProfileControl::receiveAllDynamixelValue(actuator_id);
}


/*****************************************************************************
** Functions called in Joint Dynamixel Profile Control Functions
*****************************************************************************/
bool JointDynamixelProfileControl::initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  bool result = false;
  const char* log = NULL;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id = actuator_id;
  dynamixel_.num = actuator_id.size();

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error(log);
  }

  uint16_t get_model_number;
  for (uint8_t index = 0; index < dynamixel_.num; index++)
  {
    uint8_t id = dynamixel_.id.at(index);
    result = dynamixel_workbench_->ping(id, &get_model_number, &log);

    if (result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel ID");
    }
    else
    {
      char str[100];
      sprintf(str, "Joint Dynamixel ID : %d, Model Name : %s", id, dynamixel_workbench_->getModelName(id));
      log::println(str);

      result = dynamixel_workbench_->setTimeBasedProfile(id, &log);
      if(result == false)
      {
        log::error(log);
        log::error("Please check your Dynamixel firmware version (v38~)");
      }

      result = dynamixel_workbench_->writeRegister(id, return_delay_time_char, 0, &log);
      if (result == false)
      {
        log::error(log);
        log::error("Please check your Dynamixel firmware version");
      }
    }
  }
  return true;
}

bool JointDynamixelProfileControl::setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = uint32_t(control_loop_time_*1000) * 3;
  const uint32_t acceleration = uint32_t(control_loop_time_*1000);
  const uint32_t current = 0;

  if (dynamixel_mode == "position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->currentBasedPositionMode(actuator_id.at(num), current, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }
  else
  {
    for (uint8_t num = 0; num < actuator_id.size(); num++)
    {
      result = dynamixel_workbench_->jointMode(actuator_id.at(num), velocity, acceleration, &log);
      if (result == false)
      {
        log::error(log);
      }
    }
  }

  return true;
}

bool JointDynamixelProfileControl::setSDKHandler(uint8_t actuator_id)
{
  bool result = false;
  const char* log = NULL;
  
  /* SeedR, July 2021: modified writing t Goal position to use Indirect addressing as well	
    so that uniformity can be achieved when using different series of servos */
  //result = dynamixel_workbench_->addSyncWriteHandler(actuator_id, "Goal_Position", &log);
  result = dynamixel_workbench_->addSyncWriteHandler(ADDR_GOAL_POSITION_2, LENGTH_GOAL_POSITION_2, &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                                    (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                                    &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool JointDynamixelProfileControl::writeProfileValue(std::vector<uint8_t> actuator_id, STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  for (uint8_t num = 0; num < actuator_id.size(); num++)
  {
    result = dynamixel_workbench_->writeRegister(actuator_id.at(num), char_profile_mode, value, &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  return true;
}

bool JointDynamixelProfileControl::writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<robotis_manipulator::ActuatorValue> value_vector)
{
  bool result = false;
  const char* log = NULL;

  uint8_t id_array[actuator_id.size()];
  int32_t goal_value[actuator_id.size()];

  //add tarajectory eq.
  for(uint8_t index = 0; index < actuator_id.size(); index++)
  {
    float result_position;
    float time_control = control_loop_time_;       //ms

    if(previous_goal_value_.find(actuator_id.at(index)) == previous_goal_value_.end())
    {
      previous_goal_value_.insert(std::make_pair(actuator_id.at(index), value_vector.at(index)));
    }

    result_position = value_vector.at(index).position + 3*(value_vector.at(index).velocity * (time_control))/2;

    id_array[index] = actuator_id.at(index);
    goal_value[index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), result_position);

    previous_goal_value_[actuator_id.at(index)] = value_vector.at(index);
  }

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER, id_array, actuator_id.size(), goal_value, 1, &log);
  if (result == false)
  {
    log::error(log);
  }
  return true;
}


std::vector<robotis_manipulator::ActuatorValue> JointDynamixelProfileControl::receiveAllDynamixelValue(std::vector<uint8_t> actuator_id)
{
  bool result = false;
  const char* log = NULL;

  std::vector<robotis_manipulator::ActuatorValue> all_actuator;

  uint8_t id_array[actuator_id.size()];
  for (uint8_t index = 0; index < actuator_id.size(); index++)
    id_array[index] = actuator_id.at(index);

  int32_t get_current[actuator_id.size()];
  int32_t get_velocity[actuator_id.size()];
  int32_t get_position[actuator_id.size()];

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                          id_array,
                                          actuator_id.size(),
                                          &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                id_array,
                                                actuator_id.size(),
                                                ADDR_PRESENT_CURRENT_2,
                                                LENGTH_PRESENT_CURRENT_2,
                                                get_current,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_VELOCITY_2,
                                                LENGTH_PRESENT_VELOCITY_2,
                                                get_velocity,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                 id_array,
                                                 actuator_id.size(),
                                                ADDR_PRESENT_POSITION_2,
                                                LENGTH_PRESENT_POSITION_2,
                                                get_position,
                                                &log);
  if (result == false)
  {
    log::error(log);
  }

  for (uint8_t index = 0; index < actuator_id.size(); index++)
  {
    robotis_manipulator::ActuatorValue actuator;
    actuator.effort = dynamixel_workbench_->convertValue2Current(get_current[index]);
    actuator.velocity = dynamixel_workbench_->convertValue2Velocity(actuator_id.at(index), get_velocity[index]);
    actuator.position = dynamixel_workbench_->convertValue2Radian(actuator_id.at(index), get_position[index]);

    all_actuator.push_back(actuator);
  }

  return all_actuator;
}


/*****************************************************************************
** Tool Dynamixel Control Functions
*****************************************************************************/
void GripperDynamixel::init(uint8_t actuator_id, const void *arg)
{
  STRING *get_arg_ = (STRING *)arg;

  bool result = GripperDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

  if (result == false)
    return;
}

void GripperDynamixel::setMode(const void *arg)
{
  bool result = false;
// const char* log = NULL;

  STRING *get_arg_ = (STRING *)arg;

  if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
  {
    result = GripperDynamixel::setOperatingMode(get_arg_[0]);
    if (result == false)
      return;
  }
  else
  {
    result = GripperDynamixel::writeProfileValue(get_arg_[0], std::atoi(get_arg_[1].c_str()));
    if (result == false)
      return;
  }

  result = GripperDynamixel::setSDKHandler();
  if (result == false)
    return;
}

uint8_t GripperDynamixel::getId()
{
  return dynamixel_.id.at(0);
}

void GripperDynamixel::enable()
{
  const char* log = NULL;
  bool result = false;
  
  result = dynamixel_workbench_->torqueOn(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    log::error(log);
  }
  enabled_state_ = true;
}

void GripperDynamixel::disable()
{
  const char* log = NULL;
  bool result = false;
  
  result = dynamixel_workbench_->torqueOff(dynamixel_.id.at(0), &log);
  if (result == false)
  {
    log::error(log);
  }
  enabled_state_ = false;
}

bool GripperDynamixel::sendToolActuatorValue(robotis_manipulator::ActuatorValue value)
{
  return GripperDynamixel::writeGoalPosition(value.position);
}

robotis_manipulator::ActuatorValue GripperDynamixel::receiveToolActuatorValue()
{
  robotis_manipulator::ActuatorValue result;
  result.position = GripperDynamixel::receiveDynamixelValue();
  result.velocity = 0.0;
  result.acceleration = 0.0;
  result.effort = 0.0;
  return result;
}


/*****************************************************************************
** Functions called in Tool Dynamixel Profile Control Functions
*****************************************************************************/
bool GripperDynamixel::initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
  const char* log = NULL;
  bool result = false;

  STRING return_delay_time_st = "Return_Delay_Time";
  const char * return_delay_time_char = return_delay_time_st.c_str();

  dynamixel_.id.push_back(actuator_id);
  dynamixel_.num = 1;

  dynamixel_workbench_ = new DynamixelWorkbench;

  result = dynamixel_workbench_->init(dxl_device_name.c_str(), std::atoi(dxl_baud_rate.c_str()), &log);
  if (result == false)
  {
    log::error(log);
  }

  uint16_t get_model_number;
  result = dynamixel_workbench_->ping(dynamixel_.id.at(0), &get_model_number, &log);
  if (result == false)
  {
    log::error(log);
    log::error("Please check your Dynamixel ID");
  }
  else
  {
    char str[100];
    sprintf(str, "Gripper Dynamixel ID : %d, Model Name :", dynamixel_.id.at(0));
    strcat(str, dynamixel_workbench_->getModelName(dynamixel_.id.at(0)));
    log::println(str);

    // result = dynamixel_workbench_->setVelocityBasedProfile(dynamixel_.id.at(0), &log);
    // if(result == false)
    // {
    //   log::error(log);
    //   log::error("Please check your Dynamixel firmware version (v38~)");
    // }

    result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), return_delay_time_char, 0, &log);
    if (result == false)
    {
      log::error(log);
      log::error("Please check your Dynamixel firmware version");
    }
  }

  return true;
}

bool GripperDynamixel::setOperatingMode(STRING dynamixel_mode)
{
  const char* log = NULL;
  bool result = false;

  const uint32_t velocity = 0;
  const uint32_t acceleration = 0;
  const uint32_t current = 300;

  if (dynamixel_mode == "position_mode")
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, acceleration, &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  else if (dynamixel_mode == "current_based_position_mode")
  {
    result = dynamixel_workbench_->currentBasedPositionMode(dynamixel_.id.at(0), current, &log);
    if (result == false)
    {
      log::error(log);
    }
  }
  else
  {
    result = dynamixel_workbench_->jointMode(dynamixel_.id.at(0), velocity, acceleration, &log);
    if (result == false)
    {
      log::error(log);
    }
  }

  return true;
}

bool GripperDynamixel::writeProfileValue(STRING profile_mode, uint32_t value)
{
  const char* log = NULL;
  bool result = false;

  const char * char_profile_mode = profile_mode.c_str();

  result = dynamixel_workbench_->writeRegister(dynamixel_.id.at(0), char_profile_mode, value, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool GripperDynamixel::setSDKHandler()
{
  bool result = false;
  const char* log = NULL;

  result = dynamixel_workbench_->addSyncWriteHandler(dynamixel_.id.at(0), "Goal_Position", &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->addSyncReadHandler(dynamixel_.id.at(0),
                                                    "Present_Position", 
                                                    &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

bool GripperDynamixel::writeGoalPosition(double radian)
{
  bool result = false;
  const char* log = NULL;

  int32_t goal_position = 0;

  goal_position = dynamixel_workbench_->convertRadian2Value(dynamixel_.id.at(0), radian);

  result = dynamixel_workbench_->syncWrite(SYNC_WRITE_HANDLER, &goal_position, &log);
  if (result == false)
  {
    log::error(log);
  }

  return true;
}

double GripperDynamixel::receiveDynamixelValue()
{
  bool result = false;
  const char* log = NULL;

  int32_t get_value = 0;
  uint8_t id_array[1] = {dynamixel_.id.at(0)};

  result = dynamixel_workbench_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, 
                                          id_array,
                                          (uint8_t)1,
                                          &log);
  if (result == false)
  {
    log::error(log);
  }

  result = dynamixel_workbench_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, 
                                            id_array,
                                            (uint8_t)1,
                                            &get_value, 
                                            &log);
  if (result == false)
  {
    log::error(log);
  } 

  return dynamixel_workbench_->convertValue2Radian(dynamixel_.id.at(0), get_value);
}

