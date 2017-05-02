#!/usr/bin/python
import json
import requests

'''System variables'''
base_url = 'http://192.168.100.3:9000'
robot_name = "freight31"

'''--------------------
 Authorization process
--------------------'''
# Instantiate session object and authenticate
session = requests.Session()
session.headers = {'User-Agent': 'fetchcore.client.python', 'Accept': 'application/json'}
auth_data = {'agentname': 'admin', 'password': 'admin'}
session.post(base_url + '/login', data=auth_data)


'''---------------------------
 Get information from freight:
  - Battery Level
  - Location in X, Y, Theta
---------------------------'''
# GET the information for the given robot_name
response = session.get(base_url + "/api/v1/agents/" + robot_name)

# Parse the content into JSON
agent_record = json.loads(response.content)

#Filter the battery level and location
battery_level = agent_record['agent_state']['battery_level']
current_location = agent_record['agent_state']['current_pose'].split(",")

#print ("Battery level: " + str("%.2f" % (battery_level*100)) + "%")
print ("Current Location: " )
#print ("X: "+ str(current_location[0]))
#print ("Y: "+ str(current_location[1]))
#print ("Theta: "+ str(current_location[2]))
print str(current_location[0])+","+str(current_location[1])+","+str(current_location[2])
