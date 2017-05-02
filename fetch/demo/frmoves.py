#!/usr/bin/python
import json
import requests
import sys
from time import sleep

class FRGoto:
        '''System variables'''
        #base_url = 'http://fetchcore.local:9000'
        base_url = 'http://192.168.1.102:9000'
        #def __init__(self, fetchcore_url):
                #self.base_url = fetchcore_url
        robot_name = 'freight31'
        global_speed = 1.0
        current_pose = 0
        session = None
        custom_poses_names = []

        '''--------------------
         Authorization process
        --------------------'''
        def login(self):
                # Instantiate session object and authenticate
                self.session = requests.Session()
                self.session.headers = {'User-Agent': 'fetchcore.client.python', 'Accept': 'application/json'}
                auth_data = {'agentname': 'admin', 'password': 'admin'}
                self.session.post(self.base_url + '/login', data=auth_data)

        def list(self):
                '''-------------------------------------------------------------
                   Populate a list with all the available poses on the first map
                -------------------------------------------------------------'''
                self.login()
                response = self.session.get(self.base_url + "/api/v1/maps/")

                # Parse the content into JSON
                map_record = json.loads(response.content)['available_maps']

                # Show all available maps
                print "Available maps: "
                for i in range(0, len(map_record)):
                        print str(i) + "." + str(map_record[i]['mapname'])

                # Grab all the poses from the first map
                response = self.session.get(self.base_url + "/api/v1/maps/" + str(map_record[0]['mapname']))
                custom_poses = json.loads(response.content)['annotations']['custom_poses']
                print "There are " + str(len(custom_poses)) + " custom poses named:"

                custom_poses_names = []
                for i in range(0,len(custom_poses)):
                        self.custom_poses_names.append(custom_poses[i]['name'])
                        ##print self.custom_poses_names[i]

                return custom_poses

                #'''-------------------------------------------------------------
                   #Populate a list with all the available poses on the first map
                #-------------------------------------------------------------'''
        def printlist(self):
                custom_poses = self.list()
                for i in range(0,len(custom_poses)):
                        #self.custom_poses_names.append(custom_poses[i]['name'])
                        print custom_poses[i]['name']
        

        '''----------------------------
         Multiple Actions Task creation
        ----------------------------'''
        def movetoallpos(self):
                self.login()
                while (True):
                        # Create the first NAVIGATE action that will be easily overridable
                        action_navigate = {
                                'target_poses': [self.custom_poses_names[self.current_pose]],
                                'limit_velocity': True,
                                'max_velocity': self.global_speed,
                                'preemptable': 'SOFT',
                                'action_status': 'NEW',
                                'action_name': 'NAVIGATE'}

                        # Create the task with all the required actions
                        task_goto = {
                                'task_type': 'GOTO_EXAMPLE',
                                'requesting_agent': 'admin',
                                'actions': [ action_navigate ],
                                'task_status': 'NEW',
                                'assigned_agent': self.robot_name}

                        print ("Going to " + self.custom_poses_names[self.current_pose] + " at " + str(self.global_speed) + "m/s")
        
                        # POST the task and print the response
                        response = self.session.post(self.base_url + "/api/v1/tasks/", data=json.dumps(task_goto))
                        # Parse the content into JSON
                        task_record = json.loads(response.content)
                        task_url = task_record["url"]
                        task_id = task_url.split("/")[4]

                        #print "GoTo task has been created with ID " + task_id 

                        #Pull the status of the task
                        current_task_status = "none"
                        while (current_task_status != "COMPLETE" and current_task_status != "FAILED" and current_task_status != "PREEMPTED"):
                                # GET the status for a task identified with "TASK-some_task_id"
                                response = self.session.get(self.base_url + task_url)

                                # Parse the content into JSON
                                task_record = json.loads(response.content)
                                current_task_status = task_record['task_status']
                                sleep(0.5)
                
                        print "Task is now " + current_task_status

                        if self.current_pose >= len(self.custom_poses_names)-1:
                                self.current_pose = 0
                        else:
                                self.current_pose = self.current_pose + 1
        
                        sleep(2)

        def movetopos(self, pos):
                self.login()
                #while (True):
                # Create the first NAVIGATE action that will be easily overridable
                self.current_pose = pos

                action_navigate = {
                        'target_poses': [self.custom_poses_names[self.current_pose]],
                        'limit_velocity': True,
                        'max_velocity': self.global_speed,
                        'preemptable': 'SOFT',
                        'action_status': 'NEW',
                        'action_name': 'NAVIGATE'}

                #Create the task with all the required actions
                
                task_goto = {
                        'task_type': 'GOTO_EXAMPLE',
                        'requesting_agent': 'admin',
                        'actions': [ action_navigate ],
                        'task_status': 'NEW',
                        'assigned_agent': self.robot_name}

                
                print ("Going to " + self.custom_poses_names[self.current_pose] + " at " + str(self.global_speed) + "m/s")

                # POST the task and print the response
                response = self.session.post(self.base_url + "/api/v1/tasks/", data=json.dumps(task_goto))
                # Parse the content into JSON
                task_record = json.loads(response.content)
                task_url = task_record["url"]
                task_id = task_url.split("/")[4]

                #print "GoTo task has been created with ID " + task_id 

                #Pull the status of the task
                current_task_status = "none"
                while (current_task_status != "COMPLETE" and current_task_status != "FAILED" and current_task_status != "PREEMPTED"):
                        # GET the status for a task identified with "TASK-some_task_id"
                        response = self.session.get(self.base_url + task_url)

                        # Parse the content into JSON
                        task_record = json.loads(response.content)
                        current_task_status = task_record['task_status']
                        sleep(0.5)

                print "Task is now " + current_task_status

                #if self.current_pose >= len(self.custom_poses_names)
                #        self.current_pose = 0
                #else:
                #        self.current_pose = self.current_pose + 1

                #        sleep(2)
                return current_task_status
                
        def movetopoint(self, point):
                self.login()
                #while (True):
                # Create the first NAVIGATE action that will be easily overridable
                #self.current_pose = pos
#		movingpoint = '"%s"' % (point)
#		print movingpoint

                action_navigate = {
                        'target_poses': [point],
                        'limit_velocity': True,
                        'max_velocity': self.global_speed,
                        'preemptable': 'SOFT',
                        'action_status': 'NEW',
                        'action_name': 'NAVIGATE'}

                #Create the task with all the required actions
                
                task_goto = {
                        'task_type': 'GOTO_EXAMPLE',
                        'requesting_agent': 'admin',
                        'actions': [ action_navigate ],
                        'task_status': 'NEW',
                        'assigned_agent': self.robot_name}

                
                print ("Going to " + point + " at " + str(self.global_speed) + "m/s")

                # POST the task and print the response
                response = self.session.post(self.base_url + "/api/v1/tasks/", data=json.dumps(task_goto))
                # Parse the content into JSON
                task_record = json.loads(response.content)
                task_url = task_record["url"]
                task_id = task_url.split("/")[4]

                print "GoTo task has been created with ID " + task_id 

                #Pull the status of the task
                current_task_status = "none"
                while (current_task_status != "COMPLETE" and current_task_status != "FAILED" and current_task_status != "PREEMPTED"):
                        # GET the status for a task identified with "TASK-some_task_id"
                        response = self.session.get(self.base_url + task_url)

                        # Parse the content into JSON
                        task_record = json.loads(response.content)
                        current_task_status = task_record['task_status']
                        sleep(0.5)

                print "Task is now " + current_task_status

                #if self.current_pose >= len(self.custom_poses_names)
                #        self.current_pose = 0
                #else:
                #        self.current_pose = self.current_pose + 1

                #        sleep(2)
                return current_task_status
'''
        def getCurrentPosition(self):
                self.login()
				# GET the information for the given robot_name
				response = session.get(base_url + "/api/v1/agents/" + robot_name)

				# Parse the content into JSON
				agent_record = json.loads(response.content)

				#Filter the battery level and location
				battery_level = agent_record['agent_state']['battery_level']
				current_location = agent_record['agent_state']['current_pose'].split(",")

				print ("Battery level: " + str("%.2f" % (battery_level*100)) + "%")
				print ("Current Location: " )
				print ("X: "+ str(current_location[0]))
				print ("Y: "+ str(current_location[1]))
				print ("Theta: "+ str(current_location[2]))

#                return "[%s,%s,%s]" % (current_location[0], current_location[1], current_location[2])
'''
#if __name__ == "__main__":
#        f = FRGoto()
	#print f.base_url
		#f.printlist()
        #f.movetoallpos()
		#f.movetopos(3)

	#start	
	#f.movetopoint("14.08,15.57,-1.5010")

	#position 6	
	#f.movetopoint("11.55,15.63,-3.0951")

	#position 5	
	#f.movetopoint("11.55,14.80,-3.0344")

	#position 4	
	#f.movetopoint('11.55,14.12,-3.1018')

	#position 3	
	#f.movetopoint('11.55,13.35,-3.0753')

	#position 2	
	#f.movetopoint("11.55,12.34,-3.0785")

	#position 1	
#	f.movetopoint('11.55,11.91,-3.0977')

#	print f.getCurrentPosition()

