#ifndef thruster_enitech_SDO_HPP
#define thruster_enitech_SDO_HPP

#include <thruster_enitech_driver/Request.hpp>

namespace thruster_enitech {
    class Protocol;

    /** Machinery to handle SDOs
     *
     * The classes in the SDO namespace represent both a SDO request (i.e. the
     * generation of the CAN message that makes the request) as well as the
     * logic that waits for the reply from the node, and (for read requests),
     * its parsing
     *
     * For an equivalent interface geared towards the NMTs, check the NMT
     * namespace.
     *
     * It is meant to be used in the following way (pseudocode !)
     *
     * @code
     * ReadString request(...);
     * writeCANMessage(request.message);
     * while (!request.update(readCANMessage()));
     * @endcode
     */
    namespace SDO {
        /** Exception thrown by the update method of Request and its subclasses
         * when a SDO reply is received that does not match the request
         */
        struct UnexpectedSDOReply : public std::runtime_error
        {
            UnexpectedSDOReply(std::string const& msg)
                : std::runtime_error(msg) {}
        };

        /** Exception thrown for the SDO error messages */
        struct SDOError : public std::runtime_error
        {
            SDOError(std::string const& msg)
                : std::runtime_error(msg) {}
        };

        /** Base class for handling a SDO request/reply (either read or write)
         *
         * Objects of this type are returned by the protocol class. The caller
         * must send the CAN message stored in the message attribute and then
         * call update() repeatedly until the method returns true
         *
         * Any non-SDO message is forwarded to the underlying Protocol object
         *
         * The assumption is that one does not do SDO requests in parallel, so
         * any SDO packet passed to update() that do not match what the request
         * expects will cause an UnexpectedSDOReply exception
         */
        struct Request : public thruster_enitech::Request
        {
            Request(canbus::Message const& msg,
                    Protocol& protocol,
                    uint16_t object, uint8_t subindex);

            /** Update the request object with a received message
             *
             * @return true if the expected reply message is received, and false
             *   otherwise. In the case of a SDO read, where the information
             *   stored in the message will be decoded is specific to the
             *   corresponding Request subclass.
             */
            bool update(canbus::Message const& message);

        private:
            /** The protocol object
             *
             * The request object will forward any message that is not relevant
             * for itself
             */
            Protocol& protocol;

            /** The SDO object */
            const uint16_t object;

            /** The SDO subindex */
            const uint8_t subindex;

            /** Parses a SDO error message and throws the corresponding
             * exception (a subclass of SDOError)
             */
            void parseSDOError(canbus::Message const& msg);
        };

        struct Read8 : public Request
        {
            /** Time at which the message got received */
            base::Time time;
            /** Value in the message */
            uint8_t value;

            Read8(Protocol& protocol, uint16_t object, uint8_t subindex);

            bool update(canbus::Message const& message);
        };

        struct Read16 : public Request
        {
            /** Time at which the message got received */
            base::Time time;
            /** Value in the message */
            uint16_t value;

            Read16(Protocol& protocol, uint16_t object, uint8_t subindex);

            bool update(canbus::Message const& message);
        };

        struct ReadString : public Request
        {
            /** Time at which the message got received */
            base::Time time;
            /** Value in the message */
            std::string value;

            ReadString(Protocol& protocol, uint16_t object, uint8_t subindex);

            bool update(canbus::Message const& message);
        };

        struct ReadBG149Temperature : public Read16
        {
            ReadBG149Temperature(Protocol& protocol, uint8_t probe_number);
        };

        struct Write : public Request
        {
            Write(Protocol& protocol, uint16_t object, uint8_t subindex, uint16_t value);
            Write(Protocol& protocol, uint16_t object, uint8_t subindex, uint8_t value);
            bool update(canbus::Message const& message);
        };

        /** Update the heartbeat period parameter on a device
         *
         * Note that the device must be reset for the setting to take effect
         */
        struct WriteHeartbeatPeriod : public Write
        {
            WriteHeartbeatPeriod(Protocol& protocol, base::Time const& period);
        };

        /** Update the update period parameter on a device
         *
         * Note that the device must be reset for the setting to take effect
         */
        struct WriteUpdatePeriod : public Write
        {
            WriteUpdatePeriod(Protocol& protocol, base::Time const& period);
        };
    }
}

#endif

