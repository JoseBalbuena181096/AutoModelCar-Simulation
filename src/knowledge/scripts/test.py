#!/usr/bin/env python
import rospy
import pyswip
from std_msgs.msg import String

def callback_query(msg):
    global query_str
    print("Received query: " + msg.data)
    query_str = msg.data

def main():
    
    print("INITIALIZING TEST FOR PYSWIP")
    rospy.init_node("prolog_test")
    rospy.Subscriber("/query", String, callback_query)
    loop = rospy.Rate(10)

    pl_file = ""
    if rospy.has_param("~pl_file"):
        pl_file = rospy.get_param("~pl_file")
    if pl_file == "":
        print("PL file must be specified")
        return

    prolog = pyswip.Prolog()
    prolog.consult(pl_file)

    global query_str
    query_str = ""
    while not rospy.is_shutdown():
        if query_str != "":
            print("Query result:")
            print(list(prolog.query(query_str)))
            query_str = ""
        loop.sleep()

if __name__ == '__main__':
    main()
