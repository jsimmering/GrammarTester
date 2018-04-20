import sys
import rospy
from std_msgs.msg import String

class GrammarTestNode:
    def __init__(self, commands, caught, declined, partly):
        self.lastData = ""
        self.state = False
        rospy.init_node('talker', anonymous=True)
        self.pub = rospy.Publisher('/ps_adapter/custom_rec', String, queue_size=1)
        rospy.Subscriber(args[5], String, self.callback)
        rate = rospy.Rate(10) #10hz
        print('node stuff done')
        rospy.sleep(0.5)

        print(type(commands))
        for command in commands:
            self.state = False
            accepted = False
            partlyParsed = False
            s = command.strip()
            rospy.loginfo(s)
            self.pub.publish(s)
            print('command published')
            timeout = 0
            while ((not self.state) and timeout<10):
                timeout += 1
                print(str(timeout))
                rate.sleep()
            if self.lastData==s:
                accepted = True
            elif timeout>=10:
                partlyParsed = True
            else:
                accepted = False

            if (partlyParsed):
                partly.write(command)
            elif (accepted):
                caught.write(command)
            else:
                declined.write(command)
            rospy.sleep(30)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " got some data: %s", data.data)
        self.lastData = data.data
        self.state = True

if __name__ == '__main__':
    args = sys.argv
    if len(args) < 5:
        print('Usage: python testGrammar.py <ExampleCommandsFileName> <AcceptedCommandsFileName> <DeclinedCommandsFileName> <partlyParsedFileName> <grammarNameForTopic')
        sys.exit()

    try:
        file = open(args[1], 'r')
    except IOError:
        print ('error reading to' + args[1])
        sys.exit()

    commands = file.readlines();
    print('commands read: ' + str(len(commands)))
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

    try:
        partly = open(args[4], 'w')
    except IOError:
        print  ('error writing to ' + args[4])
        sys.exit()

    node = GrammarTestNode(commands, caught, declined, partly)

    caught.close
    declined.close
    partly.close
    print('DONE')
