#!/usr/bin/env python3

# from parseCSVstring import *

description = \
'''
This script will convert a .csv file of x,y coordinates into a .world file containing a unit box located at each x,y coordinate.

USAGE:
1)  Open custom_world_coords.xls and edit the x,y coordinates.
2)  Export the spreadsheet as custom_world_coords.csv  
3)  Open a terminal and change directories to where this script (create_world.py) is saved.  Then:
	python3 create_world.py
'''

preamble = """
<sdf version='1.4'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
            <bounce/>
            <contact>
              <ode/>
            </contact>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <velocity_decay>
          <linear>0</linear>
          <angular>0</angular>
        </velocity_decay>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
    </model>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

stateOpening = """
    <state world_name='default'>
      <sim_time>39 388000000</sim_time>
      <real_time>39 522101720</real_time>
      <wall_time>1512656316 460847358</wall_time>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
"""

modelTemplate = """
    <model name='unit_box_{i}'>
      <pose>{x:f} {y:f} 0 0 -0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{lx:f} {ly:f} {h:f}</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <bounce/>
            <friction>
              <ode/>
            </friction>
            <contact>
              <ode/>
            </contact>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{lx:f} {ly:f} {h:f}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Yellow</name>
            </script>
          </material>
        </visual>
        <velocity_decay>
          <linear>0</linear>
          <angular>0</angular>
        </velocity_decay>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <gravity>1</gravity>
      </link>
      <static>1</static>
    </model>
"""

stateTemplate = """
      <model name='unit_box_{i}'>
        <pose>{x:f} {y:f} 0 0 -0 0</pose>
        <link name='link'>
          <pose>{x:f} {y:f} 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
"""

stateClosing = """
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>5 -5 2 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
  </world>
</sdf>
"""

class make_coords:
	def __init__(self, x, y):
		# Sets the value of myCoords[box_number]
		self.x 	= x
		self.y 	= y

def read_coords(filename):
  # Initialize our data structure to store our x,y coordinates for each obstacle:
  myCoords = {}
  i = 0
  # Read from the .csv file:
  import csv
  with open(filename, newline='') as csvfile:
    rawData = csv.reader(csvfile, delimiter=',', quotechar='|')
    for row in rawData:
      if (row[0][0] != '%'):
        i = i + 1
        x = int(row[0])
        y = int(row[1])
        myCoords[i+1] = make_coords(x, y)
  return myCoords

def read_buildfile(filename):
    import re
    pattern_worldname = re.compile(r'\s*<world\s+name="([^"]+)">\s*')
    pattern_endworld = re.compile(r'\s*</world>\s*')

    buildfile = {'world_name': "", 'preamble':"", 'closing':""}
    with open(filename) as infile:
        for line in infile:
            match_name = pattern_worldname.fullmatch(line)
            if match_name:
                buildfile['world_name'] = match_name.group(1)

            match_end = pattern_endworld.fullmatch(line)
            if match_end or buildfile['closing'] != "":
                buildfile['closing'] += line
            else:
                buildfile['preamble'] += line

    return buildfile


def write_world(filename, myCoords, resolution, height, buildfile=None):	
    # Open the outfile for writing:
    outFile = open(filename, 'w')

    if buildfile is not None:
        outFile.write(buildfile['preamble'])
    else:
        outFile.write(preamble)

    # Create a model for each unit obstacle on the map:
    for i in myCoords:
        thisString = modelTemplate.format(i=i, x=myCoords[i].x * resolution, y=myCoords[i].y * resolution,
                                               lx=resolution, ly=resolution, h=height)
        outFile.write(thisString)

    if buildfile is not None:
        outFile.write(f"\n  <state world_name=\"{buildfile['world_name']}\">\n")
    else:
        outFile.write(stateOpening)

    # Create a state for each model:
    for i in myCoords:
        thisString = stateTemplate.format(i=i, x=myCoords[i].x * resolution, y=myCoords[i].y * resolution)
        outFile.write(thisString)

    if buildfile is not None:
        outFile.write("  </state>\n")
        outFile.write(buildfile['closing'])
    else:
        outFile.write(stateClosing)

    outFile.close()

if __name__ == '__main__':
  import argparse

  parser = argparse.ArgumentParser(description=description)
  parser.add_argument('infile', help='Input .csv file to load coordinates from')
  parser.add_argument('outfile', help='Output .world file to create')
  parser.add_argument('-b', '--build-on', dest='buildfile',
    help='Another .world file to build on top of, instead of creating a new one.')
  parser.add_argument('-r', '--resolution', dest='resolution', type=float, default=1.0,
    help='Resolution of cell locations and sizes in meters.')
  parser.add_argument('-H', '--height', dest='height', type=float, default=1.0,
    help='Block height in meters.')
  args = parser.parse_args()

  myCoords = read_coords(args.infile)
  buildfile = None
  if args.buildfile:
      buildfile = read_buildfile(args.buildfile)
  write_world(args.outfile, myCoords, args.resolution, args.height, buildfile)

  print(f"The file '{args.outfile}' has been generated.")