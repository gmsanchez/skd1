#ifndef __SKD_BASE__SERIAL_HDLC_INTERFACE_HPP__
#define __SKD_BASE__SERIAL_HDLC_INTERFACE_HPP__

#include <vector>
#include <string>
#include <cstdint>

#define SERIAL_RX_BUFFER_MAX_SIZE           128
#define SERIAL_TX_BUFFER_MAX_SIZE           128

// Serial HDLC frame: 1 byte boundary flag + N data bytes + 1 byte boundary flag
// Message frame: 2 message type bytes + 2 length bytes + N payload bytes (64 MAX)
// Use two times MAX_PAYLOAD_SIZE for possible worst-case escape flags
#define SERIAL_HDLC_PAYLOAD_MAX_SIZE        32
#define SERIAL_HDLC_FRAME_MAX_SIZE          (SERIAL_HDLC_PAYLOAD_MAX_SIZE * 2 + 2 + 4)

namespace skd_base
{
    struct SerialHdlcFrame
    {
        uint8_t data[SERIAL_HDLC_PAYLOAD_MAX_SIZE];
        size_t length;
    };

    class SerialPort
    {
    public:
        enum ReturnType : uint8_t {
            SUCCESS = 0,
            ERROR = 1,
            FAILED_OPEN,                    // Failed to open serial port
            FAILED_GET_CONF,                // Failed to get serial port configuration
            FAILED_SET_SPEED,               // Failed to set serial port speed
            FAILED_SET_CONF,                // Failed to set serial port configuration
            FAILED_TO_READ,                 // Failed to read serial port data
            FAILED_TO_WRITE,                // Failed to write serial port data
            FAILED_TO_WRITE_ALL,            // Failed to write all data to serial port
            INVALID_BAUDRATE,               // Error invalid baudrate
            INVALID_STOPBITS,               // Error invalid stop bits
            FAILED_CONFIG_BUSSY,            // Can't configure when port is open
            FAILED_PORT_CLOSED,             // Failed, the port is closed
            FAILED_TXFRAME_BUFFER_OVF = 20,      // Failed to send frame, buffer overflow 
        };

        SerialPort();
        ~SerialPort();
        
        ReturnType open(const std::string & port_name);
        ReturnType close();
        ReturnType read_frames(std::vector<SerialHdlcFrame>& frames);
        ReturnType write_frame(const uint8_t* data, size_t size);
        bool is_open() const;

        ReturnType config_baudrate(const uint32_t baudrate);
        ReturnType config_flow_control(const bool flow_control);
        ReturnType config_parity(const bool parity);
        ReturnType config_stop_bits(const uint8_t stop_bits);

    protected:
        void encode_byte(uint8_t data);
        void decode_byte(uint8_t data, std::vector<SerialHdlcFrame>& frames);
        uint16_t crc_update(uint16_t crc, uint8_t data);

    private:
        int serial_port_;
        uint8_t rx_buffer_[SERIAL_RX_BUFFER_MAX_SIZE];
        uint8_t rx_frame_buffer_[SERIAL_HDLC_FRAME_MAX_SIZE];
        size_t rx_frame_length_;
        uint16_t rx_frame_crc_;
        bool rx_frame_escape_;
        uint8_t tx_frame_buffer_[SERIAL_HDLC_FRAME_MAX_SIZE];
        size_t tx_frame_length_;
        uint16_t tx_frame_crc_;

        uint32_t _baudrate;
        bool _flow_control;
        bool _parity;
        uint8_t _stop_bits;
    };
} // namespace skd_base

#endif  // !__SKD_BASE__SERIAL_HDLC_INTERFACE_HPP__