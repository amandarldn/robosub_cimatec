#include <iostream>
#include <ros_canbus/Driver.hpp>
#include <iomanip>
#include <map>
#include <boost/lexical_cast.hpp>

using namespace std;

int main(int argc, char**argv)
{
    if (argc < 2 || argc > 6)
    {
        cerr
            << "usage: canbus-monitor <device> <type> [count] [id] [mask]\n"
            << "  count is the count of messages to listen to, or the nolimit keyword\n"
            << "  the id/mask combination filters the CAN IDs to the ones that match can_id & mask == id\n"
            << endl;
        return 1;
    }

    std::map<int, int> statistics;
    canbus::Driver * driver = canbus::openCanDevice(argv[1], argv[2]);
    if (!driver)
        return 1;
    if (!driver->reset())
        return 1;

    int64_t count = -1;
    if (argc >= 4)
    {
        string count_s = argv[3];
        if (count_s == "nolimit")
            count = -1;
        else
            count = boost::lexical_cast<size_t>(argv[3]);
    }

    unsigned int id   = 0;
    unsigned int mask = 0;
    if (argc >= 5)
    {
        id = strtol(argv[4], NULL, 0);
        mask = 0x7FF;
    }
    if (argc >= 6)
        mask = strtol(argv[5], NULL, 0);

    cerr << "id: " << hex << id << " mask: " << hex << mask << endl;
    cout << setw(10) << "t" << " " << setw(10) << "can_t" << " " << setw(10) << "index" << " " << setw(6) << "can_id" << " " << setw(4) << "size";
    for (int byte_i = 0; byte_i < 8; ++byte_i)
        cout << " " << setw(3) << byte_i;
    cout << endl;

    int i = 0;
    base::Time firstTime;
    base::Time firstCanTime;
    while(count == -1 || i < count)
    {
        canbus::Message msg;
        try {
            msg = driver->read();
        } catch (...) {
            continue;
        }

        if ((msg.can_id & mask) != id)
            continue;

        if (firstTime.isNull())
        {
            firstTime = msg.time;
            firstCanTime = msg.can_time;
        }
        ++i;

        uint64_t timeDeltaMs = (msg.time - firstTime).toMilliseconds();
        uint64_t canTimeDeltaMs = (msg.can_time - firstCanTime).toMilliseconds();

        cout << dec << setw(10) << timeDeltaMs << " " << setw(10) << canTimeDeltaMs << " " << setw(10) << i << " " << setw(6) << hex << msg.can_id << " " << setw(4) << std::dec << (int)msg.size;
        for (int i = 0; i < msg.size; ++i)
            cout << " " << setw(3) << std::hex << (int)msg.data[i];
        cout << endl;
        statistics[msg.can_id]++;
    }

    cerr << "message statistics:\n"
        << "ID count\n";
    for (map<int, int>::const_iterator it =
            statistics.begin(); it != statistics.end(); ++it)
    {
        cerr << hex << it->first << " " << std::dec << it->second << endl;
    }
    return 0;
}

