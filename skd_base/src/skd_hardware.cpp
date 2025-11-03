#include "skd_base/skd_hardware.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define USLEEP_DELAY 1000UL

namespace skd_base
{
    static std::string HW_NAME = "HW_NAME";

    hardware_interface::CallbackReturn SKDHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
        if (hardware_interface::SystemInterface::on_init(params) != 
        hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Set hardware name
        std::string HW_NAME = get_hardware_info().name;
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Hardware name: %s", info_.name.c_str());
        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Number of Joints %zu", info_.joints.size());

        // Check joint number
        if (JOINTS_NUMBER != info_.joints.size()) {
            RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "Joint size is invalid");
            return CallbackReturn::ERROR;
        }

        // Checks for each joint (from https://github.com/husky/husky)
        for (size_t idx = 0; idx < info_.joints.size(); ++idx) {
            const hardware_interface::ComponentInfo & joint = info_.joints[idx];
            // Only one command interface per joint
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return CallbackReturn::ERROR;
            }
            // Only one *velocity* command interface per joint
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }

            // Only two state interface per joint
            if (joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return CallbackReturn::ERROR;
            }
            // First state interface is for position
            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);
                return CallbackReturn::ERROR;
            }
            // Second state interface is for velocity
            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    rclcpp::get_logger(HW_NAME),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return CallbackReturn::ERROR;
            }

            // Check joint name and save the index
            size_t k = 0;
            for (; k <= JOINTS_NUMBER; ++k) {
                if (k == JOINTS_NUMBER) {
                    RCLCPP_FATAL(rclcpp::get_logger(HW_NAME), "'%s' is not a valid joint name", joint.name.c_str());
                    return CallbackReturn::ERROR;
                }
                else if (hw_joints[k].name == joint.name) {
                    hw_joints[k].joint_idx = idx;
                    RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joint '%s' is in (%ld) order", joint.name.c_str(), idx);
                    
                    /// @todo
                    // if (joint.parameters.find("PID_gains") != joint.parameters.end())  {
                    //     auto PID_gains_prop = joint.parameters.at("pid_gains");

                    //     RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "%s PID Gains: %s", joint.name.c_str(), PID_gains_prop.c_str()); 
                    // }
                    
                    break;
                }
            }
        }

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Interfaces created successfully"); 

        /** @todo: add the ability to configure the order of velocity command joint
         * https://github.com/husarion/rosbot_hardware_interfaces/blob/main/src/rosbot_system.cpp#L72-L100
         */

        serial_port_name = info_.hardware_parameters["serial_port"];
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "(Serial) Port: %s", serial_port_name.c_str());

        std::string _baud_rate = info_.hardware_parameters["baud_rate"];
        serial_baud_rate = std::stoi(_baud_rate);
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "(Serial) Baudrate: %d", serial_baud_rate);

        std::string _flow_control = info_.hardware_parameters["flow_control"];
        if (_flow_control == "none") {
            serial_flow_control = 0;
        } else {
            serial_flow_control = -1;
        }
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "(Serial) Flow-control: %d", serial_flow_control);

        std::string _parity = info_.hardware_parameters["parity"];
        if (_parity == "none" || _parity == "false" || _parity == "0") {
            serial_parity = false;
        } else {
            serial_parity = true;
        }
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "(Serial) Parity: %s", serial_parity ? "true" : "false");

        std::string _stop_bits = info_.hardware_parameters["stop_bits"];
        serial_stop_bits = std::stoi(_stop_bits);
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "(Serial) Stop-bits: %d", serial_stop_bits);

        // Create serial port object
        serial_port = std::make_shared<SerialPort>();

        if (serial_port->config_baudrate(serial_baud_rate) != SerialPort::ReturnType::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Failed to set serial buadrate");
        }
        if (serial_port->config_flow_control((bool)serial_flow_control) != SerialPort::ReturnType::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Failed to set serial flow control");
        }
        if (serial_port->config_parity(serial_parity) != SerialPort::ReturnType::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Failed to set serial parity bits");
        }
        if (serial_port->config_stop_bits(serial_stop_bits) != SerialPort::ReturnType::SUCCESS) {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Failed to set serial stop bits");
        }

        SerialPort::ReturnType _open_error = serial_port->open(serial_port_name);
        if (_open_error != SerialPort::ReturnType::SUCCESS) {
            switch (_open_error) {
                case SerialPort::ReturnType::FAILED_OPEN:
                    // RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to open serial port: %s (%d)", strerror(errno), errno);
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to open serial port");
                    break;
                case SerialPort::ReturnType::FAILED_GET_CONF:
                    // RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to get serial port configuration: %s (%d)", strerror(errno), errno);
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to get serial port configuration");
                    break;
                case SerialPort::ReturnType::FAILED_SET_SPEED:
                    // RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to set serial port speed: %s (%d)\n", strerror(errno), errno);
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to set serial port speed");
                    break;
                case SerialPort::ReturnType::FAILED_SET_CONF:
                    // RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to set serial port configuration: %s (%d)", strerror(errno), errno);
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to set serial port configuration");
                    break;
                default:
                    break;
            }

            return CallbackReturn::FAILURE;
        } else {
            usleep(50*1000UL);
        }

        // Start serial communication with ACK message
        startCommunication();

        // Reset odometry
        resetOdometry();

        /// @todo: get PID params (from each wheel?) and write to hardware
        // setPIDGains()

        std::string _hw_update_rate = info_.hardware_parameters["update_rate"];
        hw_update_rate = std::stof(_hw_update_rate);
        // Set update period (1 [s] / [Hz]) in nanoseconds
        _hw_update_period_nS = uint32_t(1E9 / hw_update_rate);
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Write to and read from hardware period: %d [mS]", uint32_t(_hw_update_period_nS / 1E6));

        return CallbackReturn::SUCCESS;

    }

    std::vector<StateInterface> SKDHardware::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < JOINTS_NUMBER; ++i) {
            state_interfaces.emplace_back(
                StateInterface(info_.joints[hw_joints[i].joint_idx].name, hardware_interface::HW_IF_POSITION, &hw_joints[i].state_position));
            state_interfaces.emplace_back(
                StateInterface(info_.joints[hw_joints[i].joint_idx].name, hardware_interface::HW_IF_VELOCITY, &hw_joints[i].state_velocity));
        }
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "State interfaces exported");
        return state_interfaces;
    }

    std::vector<CommandInterface> SKDHardware::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < JOINTS_NUMBER; ++i) {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(info_.joints[hw_joints[i].joint_idx].name, hardware_interface::HW_IF_VELOCITY, &hw_joints[i].command));
        }
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Command interfaces exported");
        return command_interfaces;
    }

    CallbackReturn SKDHardware::on_activate(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Activating");

        // set some default values
        for (size_t i = 0; i < JOINTS_NUMBER; i++)
        {
            if (std::isnan(hw_joints[i].state_position)) {
                hw_joints[i].state_position = 0.0f;
            }
            if (std::isnan(hw_joints[i].state_velocity)) {
                hw_joints[i].state_velocity = 0.0f;
            }
            if (std::isnan(hw_joints[i].command)) {
                hw_joints[i].command = 0.0f;
            }
        }

        // Reset the flags that indicate that this is the first pass
        _first_read_pass = _first_write_pass = true;

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SKDHardware::on_deactivate(const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Deactivating");
    
        if (serial_port->is_open()) {
            serial_port->close();
            serial_port.reset();
        }

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System successfully stopped!");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SKDHardware::read(const rclcpp::Time& time, const rclcpp::Duration&) {
        if (_first_read_pass || (time - _last_read_time).nanoseconds() > _hw_update_period_nS) {
            _first_read_pass = false;
            _last_read_time = time;

            // hardware comms and operations
            RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");

            // Make sure we are connected to the serial port
            if (!serial_port->is_open()) {
                RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Not connected to serial port");
                return hardware_interface::return_type::ERROR;
            }
            
            if (updateJointsFromHardware()) { // If error when update
                RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Can't update joints from hardware");
                ///////////////////////////////////////////////////////////////////////////////
                return hardware_interface::return_type::OK;
            }

            /// @todo: Check if the joints were read successfully
            // RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SKDHardware::write(const rclcpp::Time& time, const rclcpp::Duration&) {
        if (_first_write_pass || (time - _last_write_time).nanoseconds() > _hw_update_period_nS) {
            _first_write_pass = false;
            _last_write_time = time;
            RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");

            // Make sure we are connected to the serial port
            if (!serial_port->is_open()) {
                RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Not connected to serial port");
                return hardware_interface::return_type::ERROR;
            }
            
            if (writeCommandsToHardware()) { // If error when update
                RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Can't write joints to hardware");
                ///////////////////////////////////////////////////////////////////////////////
                return hardware_interface::return_type::OK;
            }

            /// @todo: Check if the joints were write successfully
            // RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");
        }

        return hardware_interface::return_type::OK;
    }

    void dump_frame(uint8_t frame[], size_t len) {
        char buff[128];
        int offset = 0;
        for (size_t l = 0; l < len; l++)
        {
            sprintf(&buff[offset], "%02X ", frame[l]);
            offset += 3;
        }

        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "[DUMP FRAME] %s", buff);
    }

    bool SKDHardware::startCommunication(void) {
        // Generate empty message
        const uint16_t command = 0x00AA;
        const uint16_t payload_length = 0;
        const uint16_t total_length = 4 + payload_length;
        uint8_t message[total_length];

        message[0] = (uint8_t)(command & 0xFF);
        message[1] = (uint8_t)((command >> 8) & 0xFF);
        message[2] = (uint8_t)(payload_length & 0xFF);
        message[3] = (uint8_t)((payload_length >> 8) & 0xFF);

        for (uint8_t i = 0; i < 10; ++i) {
            RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Trying to establish communication (%d)", i);

            // Send
            SerialPort::ReturnType _write_error = serial_port->write_frame(message, total_length);
            if (_write_error != SerialPort::ReturnType::SUCCESS) {
                switch (_write_error) {
                    case SerialPort::ReturnType::FAILED_TXFRAME_BUFFER_OVF:
                        RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to send frame. Buffer overflow");
                        break;
                    case SerialPort::ReturnType::FAILED_TO_WRITE:
                        RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write to serial port");
                        break;
                    case SerialPort::ReturnType::FAILED_TO_WRITE_ALL:
                        RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write all bytes to serial port");
                        break;
                    default:
                        RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Serial write unknown error");
                        break;
                }
                continue;
            }

            usleep(USLEEP_DELAY);

            // Now we should receive at least one frame
            // A) ACK message
            // B) NACK message
            std::vector<SerialHdlcFrame> frames;
            SerialPort::ReturnType _res = serial_port->read_frames(frames);
            RCLCPP_WARN_EXPRESSION(rclcpp::get_logger(HW_NAME), _res > 0, "Read result: %d", _res);
            // If one or more frame received, check for ACK, NACK or error
            if (frames.size() >= 1) {
                // dump_frame(frames[0].data, frames[0].length);

                if (frames[0].length == 1) {
                    if (frames[0].data[0] == 0xA0) { // Check for ACK
                        return (int)hardware_interface::return_type::OK;
                    }
                    else if (frames[0].data[0] == 0xA1) { // If it is NACK
                        RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "NACK received. Error: 0x%02X", frames[0].data[1]);
                        // return (int)hardware_interface::return_type::ERROR;
                        continue;
                    }
                    else { // error
                        RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received. HDLC error.");
                        // return (int)hardware_interface::return_type::ERROR;
                        continue;
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received.");
                    // return (int)hardware_interface::return_type::ERROR;
                    continue;
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Not response received.");
                // return (int)hardware_interface::return_type::ERROR;
                continue;
            }
        }

        return (int)hardware_interface::return_type::ERROR;
    }

    bool SKDHardware::updateJointsFromHardware(void) {        
        // Ask for the encoders state (position and speed)
        // Generate request message
        const uint16_t command = 0x0004;
        const uint16_t payload_length = 0;
        const uint16_t total_length = 4 + payload_length;
        uint8_t message[total_length];

        message[0] = (uint8_t)(command & 0xFF);
        message[1] = (uint8_t)((command >> 8) & 0xFF);
        message[2] = (uint8_t)(payload_length & 0xFF);
        message[3] = (uint8_t)((payload_length >> 8) & 0xFF);

        // Send the request command
        SerialPort::ReturnType _write_error = serial_port->write_frame(message, total_length);
        if (_write_error != SerialPort::ReturnType::SUCCESS) {
            switch (_write_error) {
                case SerialPort::ReturnType::FAILED_TXFRAME_BUFFER_OVF:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to send frame. Buffer overflow");
                    break;
                case SerialPort::ReturnType::FAILED_TO_WRITE:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write to serial port");
                    break;
                case SerialPort::ReturnType::FAILED_TO_WRITE_ALL:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write all bytes to serial port");
                    break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Serial write unknown error");
                    break;
            }
            return (int)hardware_interface::return_type::OK;
        }

        usleep(USLEEP_DELAY);

        // Now we should receive at least one frame plus 2 if success
        // A) ACK message
        //  1) Wheel position
        //  2) Wheel velocity
        // B) NACK message
        std::vector<SerialHdlcFrame> frames;
        SerialPort::ReturnType _res = serial_port->read_frames(frames);
        RCLCPP_WARN_EXPRESSION(rclcpp::get_logger(HW_NAME), _res > 0, "Read result: %d", _res);

        // Flag to check position and velocity update
        bool position_update = false, velocity_update = false;

        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "%lu frames received.", frames.size());
        // If one or more frame received, check for ACK, NACK or error
        if (frames.size() >= 1) {
            // dump_frame(frames[0].data, frames[0].length);
            if (frames[0].length == 1) {
                if (frames[0].data[0] == 0xA0) { // Check for ACK
                    // If only one frame received, try to read the other two
                    if (frames.size() < 3) {
                        SerialPort::ReturnType _res = serial_port->read_frames(frames);
                        RCLCPP_WARN_EXPRESSION(rclcpp::get_logger(HW_NAME), _res > 0, "Read result: %d", _res);
                        
                        RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "%lu frames received.", frames.size());
                    }
                }
                else if (frames[0].data[0] == 0xA1) { // If it is NACK
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "NACK received. Error: 0x%02X", frames[0].data[1]);
                    return (int)hardware_interface::return_type::ERROR;
                }
                else { // error
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received. HDLC error.");
                    return (int)hardware_interface::return_type::ERROR;
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received.");
                return (int)hardware_interface::return_type::ERROR;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Not response received.");
            return (int)hardware_interface::return_type::ERROR;
        }

        // If all three frames are received
        if (frames.size() >= 3) {
            for (size_t i = 1; i < frames.size(); i++) {
                // dump_frame(frames[i].data, frames[i].length);

                if (frames[i].length < 4) {
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Incomplete frame received.");
                }

                uint16_t rcvd_command = (((uint16_t)frames[i].data[0]) | (((uint16_t)frames[i].data[1]) << 8)); // ((int16_t)frames[i].data[1] << 8) | frames[i].data[0];
                uint16_t rcvd_payload = (((uint16_t)frames[i].data[2]) | (((uint16_t)frames[i].data[3]) << 8));

                RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Decoded command: %u - Len: %u", rcvd_command, rcvd_payload);

                double temp_fl, temp_fr, temp_rl, temp_rr;

                temp_fl = ((int16_t)((((uint16_t)frames[i].data[5]) << 8) | (((uint16_t)frames[i].data[4])))) / 100.0;
                temp_fr = ((int16_t)((((uint16_t)frames[i].data[7]) << 8) | (((uint16_t)frames[i].data[6])))) / 100.0;
                temp_rl = ((int16_t)((((uint16_t)frames[i].data[9]) << 8) | (((uint16_t)frames[i].data[8])))) / 100.0;
                temp_rr = ((int16_t)((((uint16_t)frames[i].data[11]) << 8) | (((uint16_t)frames[i].data[10])))) / 100.0;

                if (rcvd_command == 0x7004)
                {
                    hw_joints[0].state_position = temp_fl;
                    hw_joints[1].state_position = temp_fr;
                    hw_joints[2].state_position = temp_rl;
                    hw_joints[3].state_position = temp_rr;
                    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Recived pos: %f, %f, %f, %f",
                    temp_fl, temp_fr, temp_rl, temp_rr);
                    position_update = true;
                }

                if (rcvd_command == 0x7006)
                {
                    hw_joints[0].state_velocity = temp_fl;
                    hw_joints[1].state_velocity = temp_fr;
                    hw_joints[2].state_velocity = temp_rl;
                    hw_joints[3].state_velocity = temp_rr;
                    RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Recived vel: %f, %f, %f, %f",
                    temp_fl, temp_fr, temp_rl, temp_rr);
                    velocity_update = true;
                }            
            }
        }

        if (position_update && velocity_update) {
            return (int)hardware_interface::return_type::OK;
        }

        return (int)hardware_interface::return_type::ERROR;
    }

    bool SKDHardware::writeCommandsToHardware(void) {      
        // Generate set speed message
        const uint16_t command = 0x0005;
        const uint16_t payload_length = 8;
        const uint16_t total_length = 4 + payload_length;
        uint8_t message[total_length];

        message[0] = (uint8_t)(command & 0xFF);
        message[1] = (uint8_t)((command >> 8) & 0xFF);
        message[2] = (uint8_t)(payload_length & 0xFF);
        message[3] = (uint8_t)((payload_length >> 8) & 0xFF);

        int16_t temp_fl = (int16_t) (hw_joints[0].command * 100);
        int16_t temp_fr = (int16_t) (hw_joints[1].command * 100);
        int16_t temp_rl = (int16_t) (hw_joints[2].command * 100);
        int16_t temp_rr = (int16_t) (hw_joints[3].command * 100);
        
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Sending %d, %d, %d, %d",
            temp_fl, temp_fr, temp_rl, temp_rr);

        message[4] = (uint8_t)(temp_fl & 0xFF);
        message[5] = (uint8_t)((temp_fl >> 8) & 0xFF);
        message[6] = (uint8_t)(temp_fr & 0xFF);
        message[7] = (uint8_t)((temp_fr >> 8) & 0xFF);
        message[8] = (uint8_t)(temp_rl & 0xFF);
        message[9] = (uint8_t)((temp_rl >> 8) & 0xFF);
        message[10] = (uint8_t)(temp_rr & 0xFF);
        message[11] = (uint8_t)((temp_rr >> 8) & 0xFF);

        // Send the set command
        SerialPort::ReturnType _write_error = serial_port->write_frame(message, total_length);
        if (_write_error != SerialPort::ReturnType::SUCCESS) {
            switch (_write_error) {
                case SerialPort::ReturnType::FAILED_TXFRAME_BUFFER_OVF:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to send frame. Buffer overflow");
                    break;
                case SerialPort::ReturnType::FAILED_TO_WRITE:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write to serial port");
                    break;
                case SerialPort::ReturnType::FAILED_TO_WRITE_ALL:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write all bytes to serial port");
                    break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Serial write unknown error");
                    break;
            }
            return (int)hardware_interface::return_type::OK;
        }

        usleep(USLEEP_DELAY);

        // Now we should receive at least one frame
        // A) ACK message
        // B) NACK message
        std::vector<SerialHdlcFrame> frames;
        SerialPort::ReturnType _res = serial_port->read_frames(frames);
        RCLCPP_WARN_EXPRESSION(rclcpp::get_logger(HW_NAME), _res > 0, "Read result: %d", _res);
        // If one or more frame received, check for ACK, NACK or error
        if (frames.size() >= 1) {
            // dump_frame(frames[0].data, frames[0].length);

            if (frames[0].length == 1) {
                if (frames[0].data[0] == 0xA0) { // Check for ACK
                    return (int)hardware_interface::return_type::OK;
                }
                else if (frames[0].data[0] == 0xA1) { // If it is NACK
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "NACK received. Error: 0x%02X", frames[0].data[1]);
                    return (int)hardware_interface::return_type::ERROR;
                }
                else { // error
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received. HDLC error.");
                    return (int)hardware_interface::return_type::ERROR;
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received.");
                return (int)hardware_interface::return_type::ERROR;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Not response received.");
            return (int)hardware_interface::return_type::ERROR;
        }
    }

    bool SKDHardware::resetOdometry(void) {
        RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Resetting odometry");

        // Generate reset odometry message
        const uint16_t command = 0x0006;
        const uint16_t payload_length = 0;
        const uint16_t total_length = 4 + payload_length;
        uint8_t message[total_length];
        
        message[0] = (uint8_t)(command & 0xFF);
        message[1] = (uint8_t)((command >> 8) & 0xFF);
        message[2] = (uint8_t)(payload_length & 0xFF);
        message[3] = (uint8_t)((payload_length >> 8) & 0xFF);

        // Send reset odometry command
        SerialPort::ReturnType _write_error = serial_port->write_frame(message, total_length);
        if (_write_error != SerialPort::ReturnType::SUCCESS) {
            switch (_write_error) {
                case SerialPort::ReturnType::FAILED_TXFRAME_BUFFER_OVF:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to send frame. Buffer overflow");
                    break;
                case SerialPort::ReturnType::FAILED_TO_WRITE:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write to serial port");
                    break;
                case SerialPort::ReturnType::FAILED_TO_WRITE_ALL:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Failed to write all bytes to serial port");
                    break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger(HW_NAME), "Serial write unknown error");
                    break;
            }
            return (int)hardware_interface::return_type::ERROR;
        }

        usleep(USLEEP_DELAY);

        // Now we should receive at least one frame
        // A) ACK message
        // B) NACK message
        std::vector<SerialHdlcFrame> frames;
        SerialPort::ReturnType _res = serial_port->read_frames(frames);
        RCLCPP_WARN_EXPRESSION(rclcpp::get_logger(HW_NAME), _res > 0, "Read result: %d", _res);
        // If one or more frame received, check for ACK, NACK or error
        if (frames.size() >= 1) {
            // dump_frame(frames[0].data, frames[0].length);

            if (frames[0].length == 1) {
                if (frames[0].data[0] == 0xA0) { // Check for ACK
                    return (int)hardware_interface::return_type::OK;
                }
                else if (frames[0].data[0] == 0xA1) { // If it is NACK
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "NACK received. Error: 0x%02X", frames[0].data[1]);
                    return (int)hardware_interface::return_type::ERROR;
                }
                else { // error
                    RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received. HDLC error.");
                    return (int)hardware_interface::return_type::ERROR;
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Bad frame received.");
                return (int)hardware_interface::return_type::ERROR;
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger(HW_NAME), "Not response received.");
            return (int)hardware_interface::return_type::ERROR;
        }

        // return (int)hardware_interface::return_type::OK;
    }

}  // namespace skd_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    skd_base::SKDHardware, hardware_interface::SystemInterface)
