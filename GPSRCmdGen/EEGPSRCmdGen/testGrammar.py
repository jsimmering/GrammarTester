import sys
import rospy
from std_msgs.msg import String

class GrammarTestNode:
    state = False
    lastData = ""

    def init():
        rospy.init_node('talker', anonymous=True)
        pub = rospy.Publisher('/ps_adapter/custom_rec', String, queue_size=1)
        rospy.Subscriber(args[5], String, callback)
        rate = rospy.Rate(10) #10hz

    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "got some data: %s", data.data)
        lastData = data.data
        state = True


args = sys.argv
if len(args) < 4:
    print('Usage: python testGrammar.py <ExampleCommandsFileName> <ExceptedCommandsFileName> <DeclinedCommandsFileName> <grammarNameForTopic')
    sys.exit()

try:
    file = open(args[1], 'r')
except IOError:
    print ('error reading to' + args[1])
    sys.exit()

commands = file.readlines();
file.close

try:
    caught = open(args[2], 'w')
except IOError:
    print ('error writing to ' + args[2])
    sys.exit()

try:
    declined = open(args[3], 'w')
except IOError:
    print ('error writing to ' + args[3])
    sys.exit()

for command in commands:
    state = False
    #TODO: send command to pochetphinx and check wether it is excepted
    rospy.loginfo(command)
    pub.publish(command)
    callback(command)
    while (not state):
        rate.sleep()
    if lastData is command:
        accepted = True
    else:
        accepted = False

    if (accepted):
        caught.write(command)
    else:
        declined.write(command)

