# coding=utf-8
import roslib
import rospy
from smart_battery_msgs.msg import SmartBatteryStatus  # for netbook battery
from kobuki_msgs.msg import SensorState


class netbook_battery():
    def __init__(self):
        rospy.init_node("netbook_battery")
        rospy.Subscriber("/laptop_charge", SmartBatteryStatus, self.NetbookPowerEventCallback)
        rospy.Subscriber("/mobile_base/sensors/core", SensorState, self.SensorPowerEventCallback)
        self.kobuki_base_max_charge = 160
        rospy.spin()

    def SensorPowerEventCallback(self, data):
        rospy.loginfo("Kobuki's battery is now: " + str(
            round(float(data.battery) / float(self.kobuki_base_max_charge) * 100)) + "%")
        if (int(data.charger) == 0):
            rospy.loginfo("Not charging at docking station")
        else:
            rospy.loginfo("Charging at docking station")

    def NetbookPowerEventCallback(self, data):
        print("Percent: " + str(data.percentage))
        print("Charge: " + str(data.charge))
        if (int(data.charge_state) == 1):
            print("Currently charging")
        else:
            print("Not charging")
        print("-----")

        # Tip: try print(data) for a complete list of information available in the /laptop_charge/ thread


if __name__ == '__main__':
    try:
        netbook_battery()
    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
