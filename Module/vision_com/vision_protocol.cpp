#include "vision_protocol.hpp"

#include <cstring>

uint16_t VisionProtocol::crc16Modbus(const uint8_t *data, size_t len,
                                     uint16_t crc)
{
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
        {
            crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                                 : static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

bool VisionProtocol::processByte(uint8_t byte, Frame &out)
{
    switch (state_)
    {
    case State::HEAD1:
        if (byte == kHeader1)
        {
            rx_buf_[0] = byte;
            rx_len_ = 1;
            state_ = State::HEAD2;
        }
        break;

    case State::HEAD2:
        if (byte == kHeader2)
        {
            rx_buf_[rx_len_++] = byte;
            state_ = State::LEN_H;
        }
        else if (byte == kHeader1)
        {
            rx_buf_[0] = byte; // AA AA ... 视为新帧头重新开始
            rx_len_ = 1;
        }
        else
        {
            reset();
        }
        break;

    case State::LEN_H:
        frame_len_ = static_cast<uint16_t>(byte) << 8;
        rx_buf_[rx_len_++] = byte;
        state_ = State::LEN_L;
        break;

    case State::LEN_L:
        frame_len_ |= byte;
        // 长度合法性: 至少是无payload帧(10字节), 不超出接收缓冲
        if (frame_len_ < kFrameOverhead || frame_len_ > kMaxFrameLen)
        {
            reset();
            break;
        }
        rx_buf_[rx_len_++] = byte;
        state_ = State::CODE_H;
        break;

    case State::CODE_H:
        rx_buf_[rx_len_++] = byte;
        state_ = State::CODE_L;
        break;

    case State::CODE_L:
        rx_buf_[rx_len_++] = byte;
        state_ = State::CRC_H;
        break;

    case State::CRC_H:
        crc_rx_ = static_cast<uint16_t>(byte) << 8;
        rx_buf_[rx_len_++] = byte;
        state_ = State::CRC_L;
        break;

    case State::CRC_L:
        crc_rx_ |= byte;
        rx_buf_[rx_len_++] = byte;
        // 无payload则直接等帧尾
        state_ = (frame_len_ > kFrameOverhead) ? State::PAYLOAD : State::TAIL1;
        break;

    case State::PAYLOAD:
        rx_buf_[rx_len_++] = byte;
        if (rx_len_ >= frame_len_ - 2) // 剩余2字节为帧尾
            state_ = State::TAIL1;
        break;

    case State::TAIL1:
        if (byte != kTail1)
        {
            reset();
            break;
        }
        rx_buf_[rx_len_++] = byte;
        state_ = State::TAIL2;
        break;

    case State::TAIL2:
    {
        if (byte != kTail2)
        {
            reset();
            break;
        }
        rx_buf_[rx_len_++] = byte;
        // CRC范围 = 除CRC字段(buf[6],buf[7])外的全部字节(含帧头帧尾), 分两段续算
        uint16_t crc = crc16Modbus(rx_buf_, 6);
        crc = crc16Modbus(rx_buf_ + 8, rx_len_ - 8, crc);
        if (crc != crc_rx_)
        {
            reset();
            break;
        }
        out.code = static_cast<uint16_t>((rx_buf_[4] << 8) | rx_buf_[5]);
        out.payload_len = frame_len_ - kFrameOverhead;
        if (out.payload_len > 0)
            std::memcpy(out.payload, rx_buf_ + 8, out.payload_len);
        reset();
        return true;
    }
    }
    return false;
}

uint16_t VisionProtocol::packFrame(uint16_t code, const uint8_t *payload,
                                   uint16_t payload_len, uint8_t *out_buf)
{
    if (payload_len > kMaxPayload)
        return 0;
    const uint16_t total = kFrameOverhead + payload_len;
    out_buf[0] = kHeader1;
    out_buf[1] = kHeader2;
    out_buf[2] = static_cast<uint8_t>(total >> 8); // LEN 大端
    out_buf[3] = static_cast<uint8_t>(total & 0xFF);
    out_buf[4] = static_cast<uint8_t>(code >> 8); // CODE 大端
    out_buf[5] = static_cast<uint8_t>(code & 0xFF);
    // out_buf[6],[7] 留给CRC, 最后回填
    if (payload_len > 0 && payload != nullptr)
        std::memcpy(out_buf + 8, payload, payload_len);
    out_buf[8 + payload_len] = kTail1;
    out_buf[9 + payload_len] = kTail2;
    // CRC: 帧头~业务码 + payload~帧尾(跳过CRC字段)
    uint16_t crc = crc16Modbus(out_buf, 6);
    crc = crc16Modbus(out_buf + 8, payload_len + 2, crc);
    out_buf[6] = static_cast<uint8_t>(crc >> 8); // CRC 大端
    out_buf[7] = static_cast<uint8_t>(crc & 0xFF);
    return total;
}