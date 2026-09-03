#include <fd/dhd.hpp>
#include <fd/drd.hpp>
#include <st3215/SCServo.hpp>

#include <iostream>


int main() {
    int major, minor, release, revision;
    dhdGetSDKVersion(&major, &minor, &release, &revision);
    printf("Force Dimension SDK, version %i.%i.%i", major, minor, release);

    SMS_STS sms_sts_;

    char id = 11;

    sms_sts_.begin(1000000, &id);

    sms_sts_.
}
