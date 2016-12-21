#!/usr/bin/env python 

#Must import rospy and msgs
import rospy
from GUI.Tic_Tac_GUI import Program

class GUI_GameNode():
    # Callback for msgs        
            
    # Must have __init__(self) function for a class
    def __init__(self):
        p = Program()
        p.master.title('Tic-Tac-Drone-GUI')
        mainloop()
        # Create a publisher for commands
        
        # Set the message to publish as command.
        
        # Initialize message variables.
        
        # Create a subscriber for color msg

        
        # Main while loop.
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('GUI')
    
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        gg = GUI_GameNode()
    except rospy.ROSInterruptException:
        pass