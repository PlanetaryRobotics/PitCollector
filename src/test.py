#! /usr/bin/env python
import numpy as np

import json


def main():
    print('collector: loading json position sequence')
    with open('/home/pipedream/PitCollector/json_sequences/sequence_of_positions.json') as json_file:
        data = json.load(json_file)
    #for each position
    print(type(data['position_pan_tilt']))


    cnt = 0
    for position_pan_tilt in data['position_pan_tilt']:
    	#goto first position in json
    	if cnt == 0:
    		position = position_pan_tilt[0]
    		pan  = position_pan_tilt[1]
    		tilt = position_pan_tilt[2]
    		#goto first position
    		print('collector: initialize position',position)
    		self.rig.go_to_pos(position)
    		#goto pan/tilt
    		print('collector: initialize pan/tilt',pan,tilt )
    		self.set_pan_tilt(pan,tilt)

    	elif position != position_pan_tilt[0]:
    		position = position_pan_tilt[0]
    		#goto new position
    		print('collector: change position',position)
    		self.rig.go_to_pos(position)

    	if pan != position_pan_tilt[1] or tilt != position_pan_tilt[2]:
    		#goto next pan tilt
    		pan  = position_pan_tilt[1]
    		tilt = position_pan_tilt[2]
    		#goto pan/tilt
    		print('collector: change pan/tilt',pan,tilt )
    		self.set_pan_tilt(pan,tilt)
    	
    	print('set exposure 1')
    	print('take image')
    	print('set exposure 2')
    	print('take image')
    	print('set exposure 3')
    	print('takeimage')

    	cnt += 1
    	#end of loop

  
if __name__== "__main__":
  main()

