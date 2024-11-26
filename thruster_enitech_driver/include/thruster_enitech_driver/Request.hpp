#ifndef THRUSTER_ENITECH_REQUEST_HPP
#define THRUSTER_ENITECH_REQUEST_HPP

#include <ros_canbus/Driver.hpp>
#include <ros_canbus/DriverHico.hpp>
#include <ros_canbus/DriverHicoPCI.hpp>
#include <ros_canbus/Driver2Web.hpp>
#include <ros_canbus/DriverEasySYNC.hpp>
#include <ros_canbus/DriverSocket.hpp>
#include <ros_canbus/DriverNetGateway.hpp>
#include <ros_canbus/Message.hpp>
#include <ros_canbus/time.hpp>

namespace thruster_enitech {
    /* Handling of a request/reply scheme
     *
     * The protocol used by enitech is actually a request/reply protocol.
     * Protocol implements the request generation as well as some of the parsing
     * (PDO and hearbeat but not SDOs). However, there is no encapsulation of a
     * request/reply scheme.
     *
     * This class is the base class for the request/reply handling, both for
     * register read/write (SDOs in SDO::Request and subclasses) and for NMT
     * (network management, that is start/stop/... in NMT::Request and in NMT
     * functions)
     *
     * All the subclasses are meant to be used as follows (pseudocode !):
     *
     * @code
     * Request& request = SomeRequestGenerator; // e.g. NMT::Start(protocol)
     * writeCANMessage(request.message);
     * while (!request.update(readCANMessage()));
     * @endcode
     */
    struct Request {
        /** The message that should be sent to perform this request */
        canbus::Message message;

        Request(canbus::Message const& message)
            : message(message) {}
        virtual ~Request() {}
        virtual bool update(canbus::Message const& message) = 0;
    };
}

#endif

