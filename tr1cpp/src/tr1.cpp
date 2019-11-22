#include "ros/ros.h"
#include <stdexcept>
#include <tr1cpp/tr1.h>

namespace tr1cpp {
TR1::TR1()
{
    for (int i = 0; i < 2; i++) {
        armRight.joints[i].setMotorId(i + 1);
    }

    //armRight
    armRight.joints[0].name = "JointRightShoulderPan";
    armRight.joints[0].setMotorId(1);
    armRight.joints[1].name = "JointRightShoulderTilt";
    armRight.joints[1].setMotorId(2);
}

TR1::~TR1() {}

Joint TR1::getJoint(std::string jointName)
{
    int numJointsRight = sizeof(armRight.joints) / sizeof(armRight.joints[0]);
    for (int i = 0; i < numJointsRight; i++) {
        if (armRight.joints[i].name == jointName) {
            return armRight.joints[i];
        }
    }

    throw std::runtime_error("Could not find joint with name " + jointName);
}

void TR1::setJoint(Joint joint)
{
    bool foundJoint = false;

    int numJointsRight = sizeof(armRight.joints) / sizeof(armRight.joints[0]);
    for (int i = 0; i < numJointsRight; i++) {
        if (armRight.joints[i].name == joint.name) {
            foundJoint = true;
            armRight.joints[i] = joint;
        }
    }

    if (foundJoint == false) {
        throw std::runtime_error("Could not find joint with name " + joint.name);
    }
}
} // namespace tr1cpp
