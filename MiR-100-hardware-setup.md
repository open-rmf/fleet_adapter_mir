# Contents
1. Setup Instructions
   1. MIR Setup
   2. Fleet Adapter Setup
2. Running Test
   1. Running MiR Client Test
   2. Running Fleet Adapter
3. Troubleshooting

# MIR Setup Instructions
## MIR Setup
Ensure that your MIR has been [setup](https://cdn.kyklo.co/assets/W1siZiIsIjIwMTkvMTAvMTYvMTgvNTkvMTUvYTgxNDgyMDEtNTVjMy00OTY1LTljOTYtYTEyYTZmMWYyYzI1L01pUiUyMC0lMjBNaVIyMDAlMjAtJTIwVXNlciUyMGd1aWRlLnBkZiJdXQ?sha=f515f0374a4c7c5d) properly and power up the robot (Instructions are under Commissioning, section 4, page 15).

## MIR Map and Mission
Connect to the WiFi of the robot and head to www.mir.com. Follow this [tutorial](https://www.youtube.com/watch?v=_uV6zP95xM0) on how to set up a map and this [tutorial](https://www.youtube.com/watch?v=jlks6J78R-8) on creating a mission. Keep in mind these 2 tutorials have 2 parts each.
Creating a map and mission is essential in testing the fleet adapter later on.

## MIR REST API
The MIR REST API allows you to call your missions through coding and have a multitude of different functionalities as well.
Use the "Help" tab and you can find the REST API documentation there.

![MiRHelpRESTAPI](_images/MiRHelpRESTAPI.jpg)

Enter your username and password. You will see the list of REST API calls available to you. Use "GET /missions" under the "Missions" tab

![MiRRESTAPIGETmissions](_images/MiRRESTAPIGETmissions.png)

Click on the "GET /missions" and "Try it out" then "Execute". A list of missions available should appear, try to look for the custom mission you made previously.
The "GET /missions" command returns the missions with their corresponding GUID. This GUID is **important** to identify missions later on.

![MIRGUID__2_](_images/MIRGUID__2_.jpg)

<!-- ## MIR REST API with Python
Now that we have seen the output of a REST API, let's recreate this output with a python script. First, note down the Authentication key, which can be found when you click on any of the tabs in the REST API.
(Insert picture of authentication key)

The authentication key is a HASH key of your username and password. We will be using the HASH password for the Fleet Adapter later.

Fire up your favourite python editor and paste this code
```python
import requests, json
# Get Request
ip =  'mir.com' # Can be swapped out with IP Address
host = 'https://' + ip + '/api/v2.0.0/' # v2.0.0 is important for REST API versions higher than 10

# Format headers
headers = {}
headers['Content-Type'] = 'application/json'
headers['Authorization'] = "Basic QWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA==" # Change this to your own Authorization key

get_missions = requests.get(host+'missions', verify ="{Insert Verification Key}" , headers = headers) #  verify = false, ignore verifying the SSL certificate
print(get_missions.text)
``` -->

# Fleet Adapter Setup
The MiR Fleet adapter depends on the mir100_client, thus both repositories will be cloned into the workspace
```bash
# Create workspace
mkdir -p mirfleetadapter/src
cd mirfleetadapter/src
git clone https://github.com/osrf/fleet_adapter_mir.git # TODO Change to current repo
git clone https://github.com/osrf/mir100-client.git
```

## Configure
### Mir_Config
Head to fleet adapter package and edit the configuration file
```bash
cd mirfleetadapter/src/fleet_adapter_mir
# Open up mir_config.yaml
```
Once you opened up the mir_config.yaml, change the mir_config parts:
```
base_url: "{Username}"
user: "http://mir.com/api/v2.0.0/"
password: "{password in SHA256}" #Or Your Basic Authorization Key
```
Note: Username and password are the same as login.

![mir_config](_images/mir_config.png)

### RMF_Config
Set map name as the map you are running in RMF.

Waypoint_index refers to the vertex inside the map that the robot will spawn on. The index is determined by the order the vertexes were created. If the vertex is unidentifiable, the easiest way is to redo the vertex in traffic-editor by deleting and replacing the same vertex. This way the index of that vertex  will be last on the index list.

Orientation refers to the orientation the robot will spawn in. Adjust this value to match where you want the robot to face

![rmfconfig](_images/rmfconfig.png)

### Reference Coordinates
According to the official release of the [MiR Fleet Adapter](https://github.com/osrf/fleet_adapter_mir). The reference coordinate is used for the nudge Python library to compute the 2 different coordinates transform of the MiR map and the RMF map.

Inside mir.com, look up the starting position's x and y coordinates. Then in traffic editor, locate the corresponding vertex that represents where the robot is suppose to spawn and note down that vertex's xy coordinate. Plot these coordinates inside "reference coordinates".

When you launch RMF, a REST API call to the initial spawn position will be called.

![referenceCoordinateConfig](_images/referenceCoordinateConfig.png)

### RMF Fleet
RMF Fleet config is responsible for spawning the robot's profile inside RViz along with its corresponding properties like battery system, footprint, etc. Modify linear and angular velocity and acceleration to best match the settings of the real robot so the simulation and robot can sync better.

![rmf_fleet_config](_images/rmf_fleet_config.png)

## Hash
Hash password is used to store the password securely, in this case, SHA256, a has function that generates a value that is 256 bits long is used. Quickly generate the SHA256 encoded password using this link https://xorbin.com/tools/sha256-hash-calculator.

![SHA256HashExample](_images/SHA256HashExample.JPG)

The authentication token is made up of a [Base64 encoded](https://www.base64encode.org/) hash of your username and followed by a colon and your SHA256 password.  Quickly generate the BASE64 encoded password using this link https://www.base64encode.org/.

![AutenticationTokenExample](_images/AutenticationTokenExample.JPG)

Thus if your username is "Admin" and password is "admin", you would need to convert "admin" to an encoded SHA256 and convert it again into a Base64 encoded passcode, with "Admin:" in front of it.

## Running MiR Client Test
Go to Mir100_client and create a test python file
```bash
cd mirfleetadapter/src/mir100_client
touch test.py
```

Edit test.py by adding in the test codes from the [Mir100_client repository](https://github.com/osrf/mir100-client).

Note: Each try and except tests one REST API, no need to copy the entire code, example below just calls for status updates

![RESTAPITRYANDEXCEPT](_images/RESTAPITRYANDEXCEPT.jpg)

Remember to configure the script by changing username and password in **[SHA256](https://xorbin.com/tools/sha256-hash-calculator)**.

![RESTAPICONFIG](_images/RESTAPICONFIG.jpg)

Some REST API needs a body, remember to fill up the contents of the body, hover over the code to see what parameters are needed. Information like "group_id" or "session_id" can be found using other GET REST API commands. Copy the relevant "try and except" get commands to find such information.

![RESTAPIBODY](_images/RESTAPIBODY.jpg)

Note: Not all parameters are needed, check "Documentation For Models" near the end of https://github.com/osrf/mir100-client. Click on relevant parameters like "PostMissions" to see which parameters are needed

Here is a client call demo for the MiR
https://youtu.be/QOk1AOc3Y80

## Running Fleet Adapter
Build and source the workspace
```bash
cd mirfleetadapter/
source ~/opt/ros/foxy/setup.bash
colcon build
source ./install/setup.bash
```

Run the test
```bash
cd mirfleetadapter/src/fleet_adapter_mir/
python3 main.py mir_config.yaml -m
```
Note: Enter "python3 main.py mir_config.yaml -h" for information on different arguments

You should see the status of the robot, and you can confirm this by going into the REST API of the robot and clicking "Try it out" on the "GET status" tab

## Troubleshooting
### Authentication Key Not Working
There are 2 places where you can retrieve the authentication key. The first is at the main REST API page before you sign in. If you are using that, try signing in and getting the authentication key from any tab inside the REST API page.
