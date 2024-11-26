#include <thruster_enitech_driver/NMT.hpp>

using namespace thruster_enitech::NMT;

Request::Request(Protocol& protocol, canbus::Message const& msg, Protocol::State expected_state)
    : thruster_enitech::Request(msg)
    , mProtocol(protocol)
    , mExpectedState(expected_state) {}

bool Request::update(canbus::Message const& msg)
{
    mProtocol.update(msg);
    return mProtocol.hasKnownState() && mProtocol.getLastKnownState() == mExpectedState;
}

Start::Start(Protocol& protocol)
    : Request(protocol, protocol.start(), Protocol::RUNNING) {}

Stop::Stop(Protocol& protocol)
    : Request(protocol, protocol.stop(), Protocol::STOPPED) {}

EnterPreOperational::EnterPreOperational(Protocol& protocol)
    : Request(protocol, protocol.enterPreOperational(), Protocol::PRE_OPERATIONAL) {}

Reset::Reset(Protocol& protocol)
    : Request(protocol, protocol.reset(), Protocol::PRE_OPERATIONAL) {}


