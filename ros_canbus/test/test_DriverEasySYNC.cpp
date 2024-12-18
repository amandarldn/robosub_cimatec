#include "test_Helpers.hpp"
#include <ros_canbus/DriverEasySYNC.hpp>

using namespace std;
using namespace canbus;
using ::testing::ElementsAre;
using ::testing::ContainerEq;


struct DriverTest : ::testing::Test, ros_driver_base::Fixture<DriverEasySYNC>
{
    DriverTest()
    {
        ros::Time::init();
    }

    void open(bool use_board_timestamps = false)
    {
        driver.setUseBoardTimestamps(use_board_timestamps);
        { ROS_DRIVER_BASE_MOCK();
            EXPECT_REPLY("C\r", "\r");
            EXPECT_REPLY("E\r", "E\r");
            if (use_board_timestamps)
                EXPECT_REPLY("Z1\r", "\r");
            else
                EXPECT_REPLY("Z0\r", "\r");
            EXPECT_REPLY("S2\r", "\r");
            EXPECT_REPLY("O\r", "\r");
            EXPECT_REPLY("E\r", "E\r");
            driver.open("test://:50k");
        }
    }


    void EXPECT_REPLY(const char* cmd, const char* reply)
    {
        uint8_t const* cmd_int8 = reinterpret_cast<uint8_t const*>(cmd);
        uint8_t const* reply_int8 = reinterpret_cast<uint8_t const*>(reply);
        ros_driver_base::Fixture<DriverEasySYNC>::EXPECT_REPLY(
            std::vector<uint8_t>(cmd_int8, cmd_int8 + strlen(cmd)),
            std::vector<uint8_t>(reply_int8, reply_int8 + strlen(reply))
        );
    }

    void pushDataToDriver(char const* msg)
    {
        uint8_t const* msg_int8 =
            reinterpret_cast<uint8_t const*>(msg);
        std::vector<uint8_t> packet(msg_int8, msg_int8 + strlen(msg));
        ros_driver_base::Fixture<DriverEasySYNC>::pushDataToDriver(packet);
    }
};

TEST_F(DriverTest, open_sets_a_CAN_rate_provided_in_the_URI)
{   ROS_DRIVER_BASE_MOCK();
    EXPECT_REPLY("C\r", "\r");
    EXPECT_REPLY("E\r", "E\r");
    EXPECT_REPLY("Z0\r", "\r");
    EXPECT_REPLY("S2\r", "\r");
    EXPECT_REPLY("O\r", "\r");
    EXPECT_REPLY("E\r", "E\r");
    driver.open("test://:50k");
}

TEST_F(DriverTest, open_does_not_set_an_explicit_rate_if_it_is_not_provided_in_the_URI)
{ ROS_DRIVER_BASE_MOCK();
    EXPECT_REPLY("C\r", "\r");
    EXPECT_REPLY("E\r", "E\r");
    EXPECT_REPLY("Z0\r", "\r");
    EXPECT_REPLY("O\r", "\r");
    EXPECT_REPLY("E\r", "E\r");
    driver.open("test://");
}

TEST_F(DriverTest, open_raises_if_one_of_the_messages_gets_a_0x7_reply)
{ ROS_DRIVER_BASE_MOCK();
    EXPECT_REPLY("C\r", "\x7");
    ASSERT_ANY_THROW( driver.open("test://"); );
}

canbus::Message writeTestMessage()
{
    Message msg;
    msg.can_id = 0x345;
    msg.size = 8;
    const uint8_t data[8] = {
        0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    };
    for (int i = 0; i < 8; ++i)
        msg.data[i] = data[i];
    return msg;
}

TEST_F(DriverTest, write_encodes_a_standard_frame_and_returns_when_it_is_acknowledged)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    canbus::Message msg = writeTestMessage();
    EXPECT_REPLY("t34580123456789ABCDEF\r", "z\r");
    driver.write(msg);
}

TEST_F(DriverTest, write_rejects_the_wrong_ack)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    canbus::Message msg = writeTestMessage();
    EXPECT_REPLY("t34580123456789ABCDEF\r", "\r");
    ASSERT_ANY_THROW(driver.write(msg));
}

TEST_F(DriverTest, write_throws_if_it_gets_an_error_reply)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    canbus::Message msg = writeTestMessage();
    EXPECT_REPLY("t34580123456789ABCDEF\r", "\x7");
    ASSERT_ANY_THROW(driver.write(msg));
}

TEST_F(DriverTest, it_parses_a_standard_frame_with_timestamp)
{
    open(true);

    ROS_DRIVER_BASE_MOCK();
    pushDataToDriver("t34580123456789ABCDEF2345\r");
    Message msg = driver.read();

    ASSERT_EQ(msg.can_id, 0x345);
    ASSERT_EQ(msg.can_time, base::Time::fromMilliseconds(0x2345));
    ASSERT_EQ(msg.size, 8);
    ASSERT_EQ(msg.data[0], 0x01);
    ASSERT_EQ(msg.data[1], 0x23);
    ASSERT_EQ(msg.data[2], 0x45);
    ASSERT_EQ(msg.data[3], 0x67);
    ASSERT_EQ(msg.data[4], 0x89);
    ASSERT_EQ(msg.data[5], 0xAB);
    ASSERT_EQ(msg.data[6], 0xCD);
    ASSERT_EQ(msg.data[7], 0xEF);
}

TEST_F(DriverTest, it_rejects_a_frame_with_an_invalid_character_in_the_can_ID)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    pushDataToDriver("t34@80123456789ABCDEF2345\r");
    ASSERT_ANY_THROW( driver.read() );
}

TEST_F(DriverTest, it_rejects_a_frame_with_an_invalid_character_in_the_can_length)
{
    open(true);

    ROS_DRIVER_BASE_MOCK();
    pushDataToDriver("t345901234567890ABCDEF2345\r");
    ASSERT_ANY_THROW( driver.read() );
}

TEST_F(DriverTest, it_rejects_a_frame_with_an_invalid_character_in_the_payload)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    pushDataToDriver("t34580123456@89ABCDEF2345\r");
    ASSERT_ANY_THROW( driver.read() );
}

TEST_F(DriverTest, it_rejects_a_frame_with_an_invalid_character_in_the_timestamp)
{
    open(true);

    ROS_DRIVER_BASE_MOCK();
    pushDataToDriver("t34580123456789ABCDEF2@45\r");
    ASSERT_ANY_THROW( driver.read() );
}

TEST_F(DriverTest, it_parses_a_status_message)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    EXPECT_REPLY("F\r", "C6\r");
    DriverEasySYNC::Status status = driver.getStatus();
    ASSERT_EQ(status.tx_state, DriverEasySYNC::WARNING);
    ASSERT_EQ(status.rx_state, DriverEasySYNC::WARNING);
    ASSERT_TRUE(status.rx_buffer0_overflow);
    ASSERT_TRUE(status.rx_buffer1_overflow);
}

TEST_F(DriverTest, it_determines_the_state_using_the_worst_case)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    EXPECT_REPLY("F\r", "3E\r");
    DriverEasySYNC::Status status = driver.getStatus();
    ASSERT_EQ(status.tx_state, DriverEasySYNC::OFF);
    ASSERT_EQ(status.rx_state, DriverEasySYNC::PASSIVE);
}

TEST_F(DriverTest, it_handles_an_all_OK_message)
{
    open();

    ROS_DRIVER_BASE_MOCK();
    EXPECT_REPLY("F\r", "00\r");
    DriverEasySYNC::Status status = driver.getStatus();
    ASSERT_EQ(status.tx_state, DriverEasySYNC::OK);
    ASSERT_EQ(status.rx_state, DriverEasySYNC::OK);
    ASSERT_FALSE(status.rx_buffer0_overflow);
    ASSERT_FALSE(status.rx_buffer1_overflow);
}
