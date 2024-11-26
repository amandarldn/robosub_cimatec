#ifndef _ENITECH_DRIVER_HPP_
#define _ENITECH_DRIVER_HPP_

#include <stdexcept>
#include <stdint.h>
#include <thruster_enitech_driver/Status.hpp>
#include <thruster_enitech_driver/Emergency.hpp>
#include <ros_canbus/Driver.hpp>
#include <ros_canbus/DriverHico.hpp>
#include <ros_canbus/DriverHicoPCI.hpp>
#include <ros_canbus/Driver2Web.hpp>
#include <ros_canbus/DriverEasySYNC.hpp>
#include <ros_canbus/DriverSocket.hpp>
#include <ros_canbus/DriverNetGateway.hpp>
#include <ros_canbus/Message.hpp>
#include <ros_canbus/time.hpp>
/** CAN-based protocol for the Enitech thrusters
 *
 * The main class is the Protocol class, which handles most of the low-level
 * work. Higher-level interfaces for NMT and SDOs are provided in the form of
 * the Request subclasses. See the documentation of the thruster_enitech::SDO
 * and thruster_enited::NMT namespaces for more details
 */
namespace thruster_enitech
{
    enum MSG_TYPE
    {
        MSG_NONE,
        MSG_HEARTBEAT,
        MSG_EMERGENCY,
        MSG_STATUS,
        MSG_INITIALIZED
    };

    /** Exception thrown by driver methods that require the knowledge of the
     * state but does not have it
     */
    struct StateUnknown : public std::runtime_error
    {
        StateUnknown(std::string const& msg)
            : std::runtime_error(msg) {}
    };

    /** Exception thrown by driver methods that require the knowledge of the
     * state but does not have it
     */
    struct StatusUnknown : public std::runtime_error
    {
        StatusUnknown(std::string const& msg)
            : std::runtime_error(msg) {}
    };

    /** Exception thrown by driver methods that parse an incoming message that
     * has an invalid node ID
     */
    struct InvalidNodeID : public std::runtime_error
    {
        InvalidNodeID(std::string const& msg)
            : std::runtime_error(msg) {}
    };

    /** Exception thrown when a driver method receives a message whose format
     * does not match the protocol description
     */
    struct InvalidMessage : public std::runtime_error
    {
        InvalidMessage(std::string const& msg)
            : std::runtime_error(msg) {}
    };

    /** Helper method to read a 16 bits value in the CAN message data field */
    inline uint16_t read16(canbus::Message const& msg, size_t i)
    {
        return static_cast<uint16_t>(msg.data[i+1]) << 8 | msg.data[i];
    }

    /** Helper method to write a 16 bits value in the CAN message data field */
    inline void write16(canbus::Message& msg, size_t i, uint16_t value)
    {
        msg.data[i+1] = value >> 8;
        msg.data[i] = value & 0xFF;
    }


    /** Representation of the low-level CAN protocol for a given thruster
     *
     * It handles core messages (PDO, emergency and hearbeat), and maintains the
     * known state of the thruster (as reported by the heartbeats).
     *
     * It also provides methods to generate SDO and NMT messages but does not
     * handle the request/reply nature of the SDO/NMT protocol. This is
     * delegated to the subclasses of the Request class. See the documentation
     * of the NMT and SDO namespaces for more details.
     *
     * About positive and negative directions. The "positive" direction is
     * defined as the thruster rotation that will make the thruster go towards
     * the positive vector as defined by the screw rule (i.e. the thrust vector,
     * on the other hand, is negative w.r.t. the screw rule). On the enitech
     * thruster we have, this means that clockwise is positive, but that could
     * be different on other thrusters.
     */
    class Protocol
    {
    public:
        Protocol(uint8_t node_id = 0);

        /** The node ID for which this protocol object handles communication */
        uint8_t getNodeID() const;

        enum State {
            UNKNOWN,
                BOOTUP,
                PRE_OPERATIONAL,
                RUNNING,
                STOPPED
            };

            /** Time of the last received heartbeat */
            base::Time getLastHeartbeat() const;

            /** Whether we received a state update since the last state change */
            bool hasKnownState() const;

            /** Returns the last known thruster state
             *
             * @throws StateUnknown if the state is not known. Check hastKnownState() to test
             *   for the state availability beforehand
             */
            State getLastKnownState() const;

        /** Whether we ever received a status update */
        bool hasLastStatus() const;

        /** The last received status
         *
         * @throws StatusUnknown if we never received a status before. Check
         *   hasLastStatus() to test for its availability beforehand
         */
        Status getLastStatus() const;

        /** The last received emergency message
         */
        Emergency getLastEmergency() const;

        /** Sets the node's known state
         *
         * This is used by some SDO replies that have information about the node
         * state
         */
        void setLastKnownState(State state);

        /* Notify the driver of an incoming CAN message
         *
         * It updates the internal representation of the thruster state
         *
         * @return true if we received a status update, and false otherwise
         */
        MSG_TYPE update(canbus::Message const& message);

        /** Generate a NMT message to start the node
         *
         * This is a low-level method usually not called directly, but through
         * the request object returned by NMT::start.
         */
        canbus::Message start() const;

        /** Generate a NMT message to stop the node
         *
         * This is a low-level method usually not called directly, but through
         * the request object returned by NMT::stop.
         */
        canbus::Message stop() const;

        /** Generate a NMT message to put the node into PRE_OPERATIONAL state
         *
         * This is a low-level method usually not called directly, but through
         * the request object returned by NMT::enterPreOperational.
         */
        canbus::Message enterPreOperational() const;

        /** Generate a NMT message to reset the node
         *
         * This is a low-level method usually not called directly, but through
         * the request object returned by NMT::reset.
         */
        canbus::Message reset() const;

        /** Reset the node's communication
         *
         * This applies any pending parameter change (cycle time, node ID and
         * heartbeat cycle time)
         */
        canbus::Message resetCommunication() const;

        /** Generate a command PDO message
         *
         * @arg value the command value. If in speed mode, it is in
         *   radians/second, in current mode it is in amps
         * @arg current_control whether the control should be done in current or
         *   speed
         * @return the CAN message that should be sent to the thruster
         */
        canbus::Message makeCommand(base::JointState const& command) const;

        /** Generate a CAN message to request reading a SDO */
        canbus::Message makeSDORead(uint16_t index, uint8_t subindex) const;
        /** Generate a CAN message to request writing a 8 bit value on a SDO */
        canbus::Message makeSDOWrite(uint16_t index, uint8_t subindex, uint8_t value) const;
        /** Generate a CAN message to request writing a 16 bit value on a SDO */
        canbus::Message makeSDOWrite(uint16_t index, uint8_t subindex, uint16_t value) const;

    private:
        uint8_t mNodeID;
        base::Time mLastHeartBeat;
        State mLastKnownState;
        Status mLastKnownStatus;
        Emergency mLastReceivedEmergency;

        canbus::Message makeSMT(uint8_t command) const;
        canbus::Message makeSDO(uint8_t cmd, uint16_t object, uint8_t subindex) const;
        void parseHeartbeat(canbus::Message const& msg);
        void parseEmergencyMessage(canbus::Message const& msg);
        void parseInitializedMessage(canbus::Message const& msg);
        void parsePDO(canbus::Message const& msg);
    };

} // end namespace enitech

#endif // _ENITECH_DRIVER_HPP_
