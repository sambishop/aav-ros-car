#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "GpsDriver.h"

using namespace ros;
using namespace sensor_msgs;

GpsDriver *openDriver(const char *devicePath)
{
    int fd = 0;

    /*
    fd = open(devicePath, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        fprintf(stderr, "'%s' attempting to open '%s'\n",
                strerror(errno), devicePath);
                */

    GpsDriver *driver = new GpsDriver(fd);
    // Shut off all sentences except GGA sentences.
    driver->writeSentence("PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
    return driver;
}

NavSatFix readFix(GpsDriver *driver)
{
    char *sentence;
    do {
        sentence = driver->readSentence();
    } while (strncmp("$GPGGA", sentence, 6) != 0);

    NavSatFix fix;
    fix.header.stamp = Time::now();
    //fix.header.frame_id = ...  the location of the receiver?
    fix.status.service = NavSatStatus::SERVICE_GPS;

    char *token = strtok(sentence, ",");
    int field = 0;
    while (token != NULL) {
        switch (field) {
            case 2:
                fix.latitude = strtod(token + 2, NULL) / 60;
                token[2] = '\0';
                fix.latitude += atoi(token);
                break;
            case 3:
                if (token[0] == 'S') {
                    fix.latitude *= -1;
                }
                break;
            case 4:
                fix.longitude = strtod(token + 3, NULL) / 60;
                token[3] = '\0';
                fix.longitude += atoi(token);
                break;
            case 5:
                if (token[0] == 'W') {
                    fix.longitude *= -1;
                }
                break;
            case 6:
                static int quality_map[] = {
                    NavSatStatus::STATUS_NO_FIX,
                    NavSatStatus::STATUS_FIX,
                    NavSatStatus::STATUS_SBAS_FIX,
                    NavSatStatus::STATUS_NO_FIX,
                    NavSatStatus::STATUS_GBAS_FIX,
                    NavSatStatus::STATUS_GBAS_FIX
                };
                fix.status.status = quality_map[atoi(token)];
                break;
            case 8: {
                double hdop = strtod(token, NULL);
                fix.position_covariance[0] = hdop * hdop;
                fix.position_covariance[4] = hdop * hdop;
                fix.position_covariance[8] = (2 * hdop) * (2 * hdop);
                fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
                break;
            }
            case 9:
                break;
            case 10:
                break;
        }
        ++field;
        token = strtok(NULL, ",");
    }

    return fix;
}

int main(int argc, char **argv)
{
    init(argc, argv, ROS_PACKAGE_NAME);
    NodeHandle node;
    Publisher publisher = node.advertise<NavSatFix>("fix", 10);
    GpsDriver *driver = openDriver("/dev/ttyAMA0");
    while (ros::ok()) {
        NavSatFix fix = readFix(driver);
        publisher.publish(fix);
        ros::spinOnce();
    }
    return 0;
}

