#include <cstring>
#include <thruster_enitech_driver/Protocol.hpp>

using namespace thruster_enitech;

Protocol::Protocol(uint8_t node_id)
    : mNodeID(node_id)
    , mLastKnownState(UNKNOWN) {}

uint8_t Protocol::getNodeID() const
{
    return mNodeID;
}

base::Time Protocol::getLastHeartbeat() const
{
    return mLastHeartBeat;
}

bool Protocol::hasLastStatus() const
{
    return !mLastKnownStatus.time.isNull();
}

Status Protocol::getLastStatus() const
{
    if (mLastKnownStatus.time.isNull())
        throw StatusUnknown("never received any status package");
    return mLastKnownStatus;
}

Emergency Protocol::getLastEmergency() const
{
    return mLastReceivedEmergency;
}

bool Protocol::hasKnownState() const
{
    return mLastKnownState != UNKNOWN;
}

Protocol::State Protocol::getLastKnownState() const
{
    if (hasKnownState())
        return mLastKnownState;
    throw StateUnknown("did not yet receive a state update from the node");
}

void Protocol::setLastKnownState(State state)
{
    mLastKnownState = state;
}

MSG_TYPE Protocol::update(canbus::Message const& msg)
{
    if (msg.can_id == static_cast<uint32_t>(0x700 + mNodeID))
    {
        parseHeartbeat(msg);
        return MSG_HEARTBEAT;
    }
    else if (msg.can_id == static_cast<uint32_t>(0x080 + mNodeID))
    {
        if (msg.size == 3)
        {
            parseEmergencyMessage(msg);
            return MSG_EMERGENCY;
        }
        else if (msg.size == 0)
        {
            parseInitializedMessage(msg);
            return MSG_INITIALIZED;
        }
        else
            throw InvalidMessage("message with CAN ID 0x080 + node ID received, but with a message length that is neither 3 (emergency message) nor 0 (initialized message)");
    }
    else if (msg.can_id == static_cast<uint32_t>(0x180 + mNodeID))
    {
        if (msg.size == 6)
        {
            parsePDO(msg);
            return MSG_STATUS;
        }
        else
            throw InvalidMessage("message with CAN ID 0x180 + node ID received, but with a message length that is not 6 as expected");
    }
    return MSG_NONE;
}

canbus::Message Protocol::start() const               { return makeSMT(0x01); }
canbus::Message Protocol::stop() const                { return makeSMT(0x02); }
canbus::Message Protocol::enterPreOperational() const { return makeSMT(0x80); }
canbus::Message Protocol::reset() const               { return makeSMT(0x81); }
canbus::Message Protocol::resetCommunication() const  { return makeSMT(0x82); }

canbus::Message Protocol::makeCommand(base::JointState const& command) const
{
    // Set start and control-in-water bits
    uint8_t command_byte = 1 | (1 << 4);

    base::JointState::MODE control_mode = command.getMode();
    double value = command.getField(control_mode);

    // Get the CW/CCW command based on the sign of 'value' and make value
    // absolute
    //
    // CW is positive ! See Protocol for an explanation
    if (value < 0)
        command_byte |= 1 << 1;

    // Convert value into a proper command
    uint16_t setvalue;
    if (control_mode == base::JointState::RAW)
    {
        setvalue = static_cast<uint16_t>(fabs(value));
        command_byte |= 1 << 3;
    }
    else if (control_mode == base::JointState::SPEED)
    {
        setvalue = static_cast<uint16_t>(round(fabs(value) / (2 * M_PI)));
    }

    canbus::Message msg;
    msg.can_id = 0x200 + mNodeID;
    msg.size = 3;
    write16(msg, 0, setvalue);
    msg.data[2] = command_byte;
    return msg;
}

canbus::Message Protocol::makeSMT(uint8_t command) const
{
    canbus::Message msg;
    msg.time = base::Time::now();
    msg.can_id = 0;
    msg.data[0] = command;
    msg.data[1] = mNodeID;
    msg.size = 2;
    return msg;
}

canbus::Message Protocol::makeSDO(uint8_t cmd, uint16_t object, uint8_t subindex) const
{
    canbus::Message msg;
    msg.time = base::Time::now();
    memset(msg.data, 0, 8);
    msg.can_id = 0x600 + mNodeID;
    msg.size = 8;
    msg.data[0] = cmd;
    write16(msg, 1, object);
    msg.data[3] = subindex;
    return msg;
}
canbus::Message Protocol::makeSDORead(uint16_t object, uint8_t subindex) const
{
    return makeSDO(0x40, object, subindex);
}

canbus::Message Protocol::makeSDOWrite(uint16_t object, uint8_t subindex, uint8_t value) const
{
    canbus::Message msg = makeSDO(0x22, object, subindex);
    msg.data[4] = value;
    return msg;
}
canbus::Message Protocol::makeSDOWrite(uint16_t object, uint8_t subindex, uint16_t value) const
{
    canbus::Message msg = makeSDO(0x22, object, subindex);
    write16(msg, 4, value);
    return msg;
}

void Protocol::parseInitializedMessage(canbus::Message const& msg)
{
    // We interpret an initialized message (or registration message) as a
    // heartbeat with a state of PRE_OPERATIONAL
    mLastHeartBeat = msg.time;
    mLastKnownState = PRE_OPERATIONAL;
}

void Protocol::parseHeartbeat(canbus::Message const& msg)
{
    if (msg.size != 1)
        throw InvalidMessage("received a heartbeat message with a size that is not 1");

    // Heartbeat
    switch(msg.data[0]) {
        case 0x00:
            mLastKnownState = BOOTUP;
            break;
        case 0x04:
            mLastKnownState = STOPPED;
            break;
        case 0x05:
            mLastKnownState = RUNNING;
            break;
        case 0x7F:
            mLastKnownState = PRE_OPERATIONAL;
            break;
        default:
            throw InvalidMessage("unexpected node state in heartbeat message");
    };
    mLastHeartBeat = msg.time;
}

void Protocol::parseEmergencyMessage(canbus::Message const& msg)
{
    Emergency emergency;

    // reads the error register which stores information about overtemperature
    // on thruster motors and thruster electronics
    uint8_t error_register = msg.data[2];
    emergency.overtemp_motor = error_register & (1 << 0);
    emergency.overtemp_bg149 = error_register & (1 << 1);

    uint16_t error_code = read16(msg, 0);
    switch(error_code)
    {
        case 0x0000:
            emergency.fault_free = true;
            break;
        case 0x5000:
            emergency.hardware_error = true;
            break;
        case 0x5010:
            emergency.sensor_error = true;
            break;
        case 0x6300:
            emergency.data_error = true;
            break;
        default:
            throw InvalidMessage("received emergency message with unexpected error code");
    }

    mLastReceivedEmergency = emergency;
}

void Protocol::parsePDO(canbus::Message const& msg)
{
    if (msg.size != 6)
        throw InvalidMessage("expected the PDO message to have a length of 6");

    uint8_t status_byte = msg.data[5];
    bool start_gain      = status_byte & (1 << 0);
    bool air_parameters  = !(status_byte & (1 << 1));
    bool overtemp_bg149  = status_byte & (1 << 2);
    bool overtemp_motor  = status_byte & (1 << 3);
    bool current_control = status_byte & (1 << 4);
    // 5 not used
    bool ccw             = status_byte & (1 << 6);
    // 7 not used

    Status parsed;
    parsed.time = msg.time;

    double direction = ccw ? -1 : 1;
    parsed.overtemp_bg149 = overtemp_bg149;
    parsed.overtemp_motor = overtemp_motor;
    parsed.speed   = direction * static_cast<double>(read16(msg, 0)) * M_PI * 2;
    parsed.current = direction * static_cast<double>(read16(msg, 2));

    parsed.start_gain = start_gain;
    parsed.air_parameters = air_parameters;
    parsed.control_mode = current_control ? base::JointState::RAW : base::JointState::SPEED;
    mLastKnownStatus = parsed;
    mLastKnownState = RUNNING;
}

