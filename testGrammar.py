import sys
import rospy
from std_msgs.msg import String

class GrammarTestNode:
    def __init__(self, commands, caught, declined, partly):
        self.lastData = ""
        self.state = False
        rospy.init_node('talker', anonymous=True)
        self.pub = rospy.Publisher('/ps_adapter/custom_rec', String, queue_size=1)
        rospy.Subscriber(topicName, String, self.callback)
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
            #rospy.sleep(6)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " got some data: %s", data.data)
        self.lastData = data.data
        self.state = True

if __name__ == '__main__':
    args = sys.argv
    if len(args) != 6 and len(args) != 2:
        print('Usage: python testGrammar.py <ExampleCommandsFileName> <AcceptedCommandsFileName> <DeclinedCommandsFileName> <partlyParsedFileName> <grammarNameForTopic>\nShortcut: python testGrammar.py gpsr\nShortcut: python testGrammar.py eegpsr\nShortcut: python testGrammar.py spr')
        sys.exit()

    if len(args) == 6:
        exampleCommandsFileName = args[1]
        acceptedCommandsFileName = args[2]
        declinedCommandsFileName = args[3]
        partlyParsedCommandsFileName = args[4]
        topicName = args[5]
    if len(args) == 2:
        if args[1] == 'gpsr':
            exampleCommandsFileName = 'GPSRCmdGen/GPSR Cat2 Examples/GPSR Cat2 Examples.txt'
            acceptedCommandsFileName = 'gpsr_accepted.txt'
            declinedCommandsFileName = 'gpsr_declined.txt'
            partlyParsedCommandsFileName = 'gpsr_partlyparsed.txt'
            topicName = '/speechrec/psa/commands/simple'
        if args[1] == 'eegpsr':
            exampleCommandsFileName = 'GPSRCmdGen/EEGPSR Examples.txt'
            acceptedCommandsFileName = 'eegpsr_accepted.txt'
            declinedCommandsFileName = 'eegpsr_declined.txt'
            partlyParsedCommandsFileName = 'eegpsr_partlyparsed.txt'
            topicName = '/speechrec/psa/commands/simple'
        if args[1] == 'spr':
            exampleCommandsFileName = 'GPSRCmdGen/SPRTest 35000 Examples/SPRTest 3500 Examples.txt'
            acceptedCommandsFileName = 'spr_accepted.txt'
            declinedCommandsFileName = 'spr_declined.txt'
            partlyParsedCommandsFileName = 'spr_partlyparsed.txt'
            topicName = '/speechrec/psa/speechRecognition/simple'
            #print('NOT SUPPORTED YET; MISSING EXAMPLE FILE LOCATION')
            sys.exit()

    print('using following configuration:\n'+exampleCommandsFileName+'\n'+acceptedCommandsFileName+'\n'+declinedCommandsFileName+'\n'+partlyParsedCommandsFileName+'\n'+topicName)

    try:
        file = open(exampleCommandsFileName, 'r')
    except IOError:
        print ('error reading to' + exampleCommandsFileName)
        sys.exit()

    commands = file.readlines();
    print('commands read: ' + str(len(commands)))
    file.close

    try:
        caught = open(acceptedCommandsFileName, 'w')
    except IOError:
        print ('error writing to ' + acceptedCommandsFileName)
        sys.exit()

    try:
        declined = open(declinedCommandsFileName, 'w')
    except IOError:
        print ('error writing to ' + declinedCommandsFileName)
        sys.exit()

    try:
        partly = open(partlyParsedCommandsFileName, 'w')
    except IOError:
        print  ('error writing to ' + partlyParsedCommandsFileName)
        sys.exit()

    node = GrammarTestNode(commands, caught, declined, partly)

    caught.close
    declined.close
    partly.close
    print('DONE')
