#include "vision_com.hpp"

bool VisionCom::init(UartPort *port)
{
    port_ = port;
    return port_ != nullptr;
}

bool VisionCom::sendEvent(VisionProtocol::UpCode code)
{
    if (port_ == nullptr)
        return false;
    const uint16_t len =
        VisionProtocol::packFrame(code, nullptr, 0, tx_buf_);
    if (len == 0)
        return false;
    // HAL_BUSY 表示TX双缓冲已满, 调用方自行决定是否重试
    return port_->writeDma(tx_buf_, len) == HAL_OK;
}

bool VisionCom::sendPathRequestNextIndexed(uint16_t index)
{
    if (port_ == nullptr)
        return false;
    const uint8_t payload[2] = {
        static_cast<uint8_t>(index & 0xFF), // 小端低字节在前
        static_cast<uint8_t>((index >> 8) & 0xFF),
    };
    const uint16_t len =
        VisionProtocol::packFrame(VisionProtocol::UP_REQ_NEXT_INDEXED, payload,
                                  sizeof(payload), tx_buf_);
    if (len == 0)
        return false;
    return port_->writeDma(tx_buf_, len) == HAL_OK;
}