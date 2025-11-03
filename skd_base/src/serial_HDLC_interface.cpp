#include "skd_base/serial_HDLC_interface.hpp"

#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
// #include <errno.h>

#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"

#define  HDLC_FRAME_BOUNDRY_FLAG     0x7E
#define  HDLC_ESCAPE_FLAG            0x7D
#define  HDLC_ESCAPE_XOR             0x20
#define  HDLC_CRC_INIT_VALUE         0xFFFF

namespace skd_base {
    using ReturnType = SerialPort::ReturnType; 

    SerialPort::SerialPort() :
        serial_port_(-1),
        rx_frame_length_(0),
        rx_frame_crc_(HDLC_CRC_INIT_VALUE),
        rx_frame_escape_(false),
        tx_frame_length_(0),
        tx_frame_crc_(HDLC_CRC_INIT_VALUE) {
            _baudrate = B9600;
            _parity = false;
        }

    SerialPort::~SerialPort() {
        close();
    }

    ReturnType SerialPort::config_baudrate(const uint32_t baudrate) {
        if (is_open()) {
            return ReturnType::FAILED_CONFIG_BUSSY;
        }
        switch (baudrate) {
            case 9600:
                _baudrate = B9600;
                break;
            case 19200:
                _baudrate = B19200;
                break;
            case 57600:
                _baudrate = B57600;
                break;
            case 115200:
                _baudrate = B115200;
                break;
            default:
                return ReturnType::INVALID_BAUDRATE;
        }
        return ReturnType::SUCCESS;
    }

    ReturnType SerialPort::config_flow_control(const bool flow_control) {
        if (is_open()) {
            return ReturnType::FAILED_CONFIG_BUSSY;
        }
        _flow_control = flow_control;
        return ReturnType::SUCCESS;
    }

    ReturnType SerialPort::config_parity(const bool parity) {
        if (is_open()) {
            return ReturnType::FAILED_CONFIG_BUSSY;
        }
        _parity = parity;
        return ReturnType::SUCCESS;
    }

    ReturnType SerialPort::config_stop_bits(const uint8_t stop_bits) {
        if (is_open()) {
            return ReturnType::FAILED_CONFIG_BUSSY;
        }
        switch (stop_bits) {
            case 1:
            case 2:
                _stop_bits = stop_bits;
                break;
            default:
                return ReturnType::INVALID_STOPBITS;
        }
        return ReturnType::SUCCESS;
    }


    ReturnType SerialPort::open(const std::string & port_name) {
        /// @todo Check permissions

        serial_port_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY);
        usleep(10000);

        if (serial_port_ < 0) {
            return ReturnType::FAILED_OPEN;
        }

        struct termios tty_config{};
        if (::tcgetattr(serial_port_, &tty_config) != 0) {
            close();
            return ReturnType::FAILED_GET_CONF;
        }

        memset(&tty_config, 0, sizeof(termios));
        
        tty_config.c_cflag = _baudrate | CRTSCTS | CS8 | CLOCAL | CREAD;

        if (_parity) {
            tty_config.c_cflag |= PARENB;   // Clear parity bit, disabling parity (most common)
        } else {
            tty_config.c_cflag &= ~PARENB;   // Clear parity bit, disabling parity (most common)
        }

        if (_stop_bits == 2) {
            tty_config.c_cflag |= CSTOPB;
        } else {
            tty_config.c_cflag &= ~CSTOPB;   // Clear stop field, only one stop bit used in communication (most common)
        }
        
        tty_config.c_cflag &= ~CSIZE;    // Clear bits per byte
        tty_config.c_cflag |=  CS8;      // 8 bit per byte
        tty_config.c_iflag = IGNPAR;
        tty_config.c_oflag = OPOST;
        tty_config.c_lflag = 0;
        tty_config.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty_config.c_cc[VMIN] = 1;
        tcflush(serial_port_, TCIFLUSH);

        if (::tcsetattr(serial_port_, TCSANOW, &tty_config) != 0) {
            close();
            return ReturnType::FAILED_SET_CONF;
        }

        return ReturnType::SUCCESS;
    }

    bool SerialPort::is_open() const {
        return serial_port_ >= 0;
    }            

    ReturnType SerialPort::close() {
        if (is_open()) {
            ::close(serial_port_);
            serial_port_ = -1;
        }
        
        return ReturnType::SUCCESS;
    }

    ReturnType SerialPort::read_frames(std::vector<SerialHdlcFrame>& frames) {
        if (!is_open()) {
            return ReturnType::FAILED_PORT_CLOSED;
        }

        ssize_t num_bytes = 0;
        num_bytes = ::read(serial_port_, rx_buffer_, SERIAL_RX_BUFFER_MAX_SIZE);  // Leo el primer byte

        for (ssize_t i = 0; i < num_bytes; i++) {
            decode_byte(rx_buffer_[i], frames);
        }

        return ReturnType::SUCCESS;
    }

    void SerialPort::decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames) {
        if (data == HDLC_FRAME_BOUNDRY_FLAG) {
            if (rx_frame_escape_) {
                rx_frame_escape_ = false;
            }
            else if (rx_frame_length_ >= 2 && rx_frame_crc_ == ((uint16_t(rx_frame_buffer_[rx_frame_length_ - 1]) << 8) | rx_frame_buffer_[rx_frame_length_ - 2])) {
                SerialHdlcFrame frame;
                memcpy(frame.data, rx_frame_buffer_, rx_frame_length_ - 2);
                frame.length = rx_frame_length_ - 2;
                frames.push_back(frame);
            }
            rx_frame_length_ = 0;
            rx_frame_crc_ = HDLC_CRC_INIT_VALUE;
            return;
        }

        if (data == HDLC_ESCAPE_FLAG) {
            rx_frame_escape_ = true;
            return;
        }

        if (rx_frame_escape_) {
            rx_frame_escape_ = false;
            data ^= HDLC_ESCAPE_XOR;
        }

        rx_frame_buffer_[rx_frame_length_] = data;
        if (rx_frame_length_ >= 2) {
            rx_frame_crc_ = crc_update(rx_frame_crc_, rx_frame_buffer_[rx_frame_length_ - 2]);
        }
        rx_frame_length_++;

        if (rx_frame_length_ == SERIAL_HDLC_FRAME_MAX_SIZE) {
            rx_frame_length_ = 0;
            rx_frame_crc_ = HDLC_CRC_INIT_VALUE;
        }
    }

    ReturnType SerialPort::write_frame(const uint8_t* data, size_t size) {
        if (!is_open()) {
            return ReturnType::FAILED_PORT_CLOSED;
        }
        
        // Generate the fame
        tx_frame_length_ = 0;
        tx_frame_crc_  = HDLC_CRC_INIT_VALUE;
        
        // Write the frame boundry flag
        tx_frame_buffer_[tx_frame_length_++] = HDLC_FRAME_BOUNDRY_FLAG;
        // Write the frame
        for (size_t i = 0; i < size; i++) {
            // Check if there is space left in buffer
            // Reserve 4 bytes: 1 boundary flag + 2 CRC bytes and
            // one potential escape flag
            if ((tx_frame_length_ + 5) >= SERIAL_HDLC_FRAME_MAX_SIZE) {
                return ReturnType::FAILED_TXFRAME_BUFFER_OVF;
            } else {
                tx_frame_crc_ = crc_update(tx_frame_crc_, data[i]);
                encode_byte(data[i]);
            }
        }

        // Reserve 5 bytes: 1 boundary flag + 2 CRC bytes and
        // two potential escape flag
        if ((tx_frame_length_ + 6) >= SERIAL_HDLC_FRAME_MAX_SIZE) {
            return ReturnType::FAILED_TXFRAME_BUFFER_OVF;
        }
        
        // Write the checksum
        encode_byte((tx_frame_crc_ & 0xFF));
        encode_byte(((tx_frame_crc_ >> 8) & 0xFF));
        // Write the frame boundry flag
        tx_frame_buffer_[tx_frame_length_++] = HDLC_FRAME_BOUNDRY_FLAG;

        // Write all bytes and check
        ssize_t _written = ::write(serial_port_, tx_frame_buffer_, tx_frame_length_);
        if (_written == -1) {
            return ReturnType::FAILED_TO_WRITE;
        } else if ((size_t)_written != tx_frame_length_) {
            return ReturnType::FAILED_TO_WRITE_ALL;
        }

        return ReturnType::SUCCESS;
    }

    void SerialPort::encode_byte(uint8_t data) {
        if (data == HDLC_ESCAPE_FLAG || data == HDLC_FRAME_BOUNDRY_FLAG) {
            tx_frame_buffer_[tx_frame_length_++] = HDLC_ESCAPE_FLAG;
            data ^= HDLC_ESCAPE_XOR;
        }
        tx_frame_buffer_[tx_frame_length_++] = data;
    }

    uint16_t SerialPort::crc_update(uint16_t crc, uint8_t data) {
        data ^= (uint8_t)(crc & 0xFF);
        data ^= (data << 4);

        return ((((uint16_t)data << 8) | ((uint8_t)(crc >> 8) & 0xFF)) 
            ^ (uint8_t)(data >> 4) 
            ^ ((uint16_t)data << 3));
    }

} // namespace skd_base

