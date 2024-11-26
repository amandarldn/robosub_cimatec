#ifndef THRUSTER_ENITECH_NMT_HPP
#define THRUSTER_ENITECH_NMT_HPP

#include <thruster_enitech_driver/Request.hpp>
#include <thruster_enitech_driver/Protocol.hpp>

#include <ros_canbus/Driver.hpp>
#include <ros_canbus/DriverHico.hpp>
#include <ros_canbus/DriverHicoPCI.hpp>
#include <ros_canbus/Driver2Web.hpp>
#include <ros_canbus/DriverEasySYNC.hpp>
#include <ros_canbus/DriverSocket.hpp>
#include <ros_canbus/DriverNetGateway.hpp>
namespace thruster_enitech
{
    /** Network management requests (start/stop/...)
     *
     * These objects encapsulate both the request (in the form of a CAN
     * message) as well as the logic necessary to wait for an acknowledgment
     * of the request.
     *
     * In the case of NMT requests, the "acknowledgment" is done by waiting
     * for a heartbeat message from the node that announces the state change
     * expected by the request.
     *
     * For an equivalent interface geared towards the SDOs, check the SDO
     * namespace.
     *
     * It is meant to be used in the following way (pseudocode !)
     *
     * @code
     * Request request(NMT::Start(protocol));
     * writeCANMessage(request.message);
     * while (!request.update(readCANMessage()));
     * @endcode
     */
    namespace NMT
    {
        struct Request : public thruster_enitech::Request
        {
            Request(Protocol& protocol,
                    canbus::Message const& message,
                    Protocol::State expected_state);

            /** Update the request object with a received message
             *
             * @return true if the node reached the expected state, and false
             *   otherwise.
             */
            bool update(canbus::Message const& message);

        protected:
            static canbus::Message makeMessage(uint8_t msg, uint8_t node_id);
            Protocol& mProtocol;
            Protocol::State mExpectedState;
        };

        /** Start the node
         *
         * The post-condition state is RUNNING
         */
        struct Start : public Request
        {
            Start(Protocol& protocol);
        };

        /** Stop the node
         *
         * The post-condition state is STOPPED
         */
        struct Stop : public Request
        {
            Stop(Protocol& protocol);
        };

        /** Make the node go into preoperational mode
         *
         * The post-condition state is PRE_OPERATIONAL
         */
        struct EnterPreOperational : public Request
        {
            EnterPreOperational(Protocol& protocol);
        };

        /** Reset the node
         *
         * This applies any pending parameter change (cycle time, node ID and
         * heartbeat cycle time)
         */
        struct Reset : public Request
        {
            Reset(Protocol& protocol);
        };
    }
}

#endif

