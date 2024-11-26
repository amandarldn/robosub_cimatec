#include <thruster_enitech_driver/SDO.hpp>
#include <thruster_enitech_driver/Protocol.hpp>

using namespace thruster_enitech::SDO;

Request::Request(canbus::Message const& msg, thruster_enitech::Protocol& protocol, uint16_t object, uint8_t subindex)
    : thruster_enitech::Request(msg)
    , protocol(protocol)
    , object(object)
    , subindex(subindex) {}

bool Request::update(canbus::Message const& message)
{
    if ((message.can_id & 0xF80) != 0x580)
    {
        protocol.update(message);
        return false;
    }
    else if ((message.can_id & 0x7f) != protocol.getNodeID())
        throw UnexpectedSDOReply("received a SDO reply for a different node than this one");
    else if (message.size != 8)
        throw UnexpectedSDOReply("received a SDO reply of size different than 8");
    else if (message.data[0] == 0x80)
    {
        parseSDOError(message);
        throw std::logic_error("received a SDO error message and parseSDOError returned. It should not have happened");
    }

    uint16_t msg_object   = read16(message, 1);
    uint8_t  msg_subindex = message.data[3];
    if (msg_object != object)
        throw UnexpectedSDOReply("received a SDO reply for a different object than the expected one");
    if (msg_subindex != subindex)
        throw UnexpectedSDOReply("received a SDO reply for a different subindex than the expected one");

    return true;
}

namespace
{
    struct SDOErrorDescription
    {
        uint16_t w0;
        uint16_t w1;
        char const* msg;
    };
}



void Request::parseSDOError(canbus::Message const& msg)
{
    static const SDOErrorDescription errors[] = {
        { 0x0504, 0x0001, "client/server command unknown or invalid" },
        { 0x0601, 0x0000, "unsupported access to this object" },
        { 0x0601, 0x0001, "read access not support on this object" },
        { 0x0601, 0x0002, "write access not support on this object" },
        { 0x0602, 0x0000, "object does not exist" },
        { 0x0609, 0x0011, "sub-index does not exist" },
        { 0x0800, 0x0000, "General error" },
        { 0x0800, 0x0020, "data cannot be transferred or stored" },
        { 0x0800, 0x0021, "data cannot be transferred or stored because of local control" },
        { 0x0800, 0x0022, "data cannot be transferred or stored because the current device state" },
        { 0x0800, 0x0024, "no data available" },
        { 0, 0, NULL }
    };

    SDOErrorDescription const* error = errors;


    while (error->msg)
    {
        if ((read16(msg, 4) == error->w0) && (read16(msg, 6) == error->w1))
            throw SDOError(error->msg);
        error++;
    }

    throw SDOError("unknown SDO error");
}

Read8::Read8(Protocol& protocol, uint16_t object, uint8_t subindex)
    : Request(protocol.makeSDORead(object, subindex), protocol, object, subindex) {}

bool Read8::update(canbus::Message const& message)
{
    if (!Request::update(message))
        return false;
    else if (message.data[0] != 0x42)
        throw UnexpectedSDOReply("expected a reply for a SDO read and got something else");

    time = message.time;
    value = message.data[4];
    return true;
}

Read16::Read16(Protocol& protocol, uint16_t object, uint8_t subindex)
    : Request(protocol.makeSDORead(object, subindex), protocol, object, subindex) {}

bool Read16::update(canbus::Message const& message)
{
    if (!Request::update(message))
        return false;
    else if (message.data[0] != 0x42)
        throw UnexpectedSDOReply("expected a reply for a SDO read and got something else");

    time = message.time;
    value = thruster_enitech::read16(message, 4);
    return true;
}

ReadString::ReadString(Protocol& protocol, uint16_t object, uint8_t subindex)
    : Request(protocol.makeSDORead(object, subindex), protocol, object, subindex) {}

bool ReadString::update(canbus::Message const& message)
{
    if (!Request::update(message))
        return false;
    else if (message.data[0] != 0x42)
        throw UnexpectedSDOReply("expected a reply for a SDO read and got something else");

    time  = message.time;
    char const* data = reinterpret_cast<char const*>(message.data + 4);
    value = std::string(data, data + 4);
    return true;
}

ReadBG149Temperature::ReadBG149Temperature(Protocol &protocol, uint8_t probe_number)
    : Read16(protocol, 0x3010, probe_number) {}

Write::Write(Protocol& protocol, uint16_t object, uint8_t subindex, uint8_t value)
    : Request(protocol.makeSDOWrite(object, subindex, value), protocol, object, subindex) {}
Write::Write(Protocol& protocol, uint16_t object, uint8_t subindex, uint16_t value)
    : Request(protocol.makeSDOWrite(object, subindex, value), protocol, object, subindex) {}

bool Write::update(canbus::Message const& message)
{
    if (!Request::update(message))
        return false;
    else if (message.data[0] != 0x60)
        throw UnexpectedSDOReply("expected a reply for a SDO write and got something else");
    return true;
}

WriteHeartbeatPeriod::WriteHeartbeatPeriod(Protocol& protocol, base::Time const& period)
    : Write(protocol, 0x1017, 0x00, static_cast<uint16_t>(period.toMilliseconds())) {}

WriteUpdatePeriod::WriteUpdatePeriod(Protocol& protocol, base::Time const& period)
    : Write(protocol, 0x2200, 0x00, static_cast<uint16_t>(period.toMilliseconds())) {}

