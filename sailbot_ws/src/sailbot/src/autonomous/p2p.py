import math
import json

class P2P:
	def __init__(self, curpos, dest):
		self.curpos = curpos #coordinates of starting point
		self.dest = dest #coordinates of end point 
		self.state = 1
		self.temphead = 0 #45 when on port tack, 135 when on starboard
		self.rudder_angle = 70
	

	def getAction(self, wind, cmpas, track):
		if self.getdistance() < 0.01: #within 10 meters
			return {"status" : "DONE"}
		windAng = self.getWindToNorth(wind,cmpas) #direction of wind relative to north
		boatAng = self.getHeading() #direction relative to north to get from current position to end position
		if boatAng < 0:
			boatAng += 360
		pointofsail = boatAng - windAng #angle between wind and direction to get to destination
		if pointofsail < 0:
			pointofsail += 360
		if self.state == 1:
			tt = self.state1(pointofsail, windAng, boatAng)
			return {"status" : "OK", "tt-state" : tt}
		elif self.state == 2:
			rudder = self.state2(windAng, boatAng, track)
			return {"status" : "OK", "rudder-angle" : rudder}
		elif self.state == 3:
			rudder = self.state3(boatAng, track, pointofsail)
			return {"status" : "OK", "rudder-angle" : rudder}
		
	def state1(self, pointofsail, windAng, boatAng):
		#########
		#state 1#
		#########
		print("current state: 1")
		if pointofsail >= 0 and pointofsail < 45:
			self.temphead = 50
			x = 0 #max lift port
			#json.dumps({"tack":"port","trimtab":"lift"})
			self.state = 2
		elif pointofsail >= 45 and pointofsail <= 135:
			heading = boatAng
			x = 0 #max lift port
			#json.dumps({"tack":"port","trimtab":"lift"})
			self.state = 3
		elif pointofsail > 135 and pointofsail < 180:
			heading = boatAng
			x = 2 #max drag port
			#json.dumps({"tack":"port","trimtab":"drag"})
			self.state = 3
		elif pointofsail >= 180 and pointofsail < 225:
			heading = boatAng
			x = 3 #max drag starboard
			#json.dumps({"tack":"starboard","trimtab":"drag"})
			self.state = 3
		elif pointofsail >= 225 and pointofsail <= 315:
			heading = boatAng
			x = 1 #max lift starboard
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			self.state = 3
		elif pointofsail >= 315 and pointofsail < 360:
			self.temphead = 310
			x = 1 #max lift starboard
			#json.dumps({"tack":"starboard","trimtab":"lift"})
			self.state = 2
		print(x)
		return(x)
	
	def state2(self, windAng, boatAng, track):
		#########
		#state 2#
		#########
		print("current state: 2")
		rudders = self.rudder_angle
		tempheading = windAng + self.temphead #move 45 degrees from upwind either port or starboard
		if tempheading >= 360:
			while tempheading >= 360:
				tempheading -= 360
		# get and keep the boat on course #
		variance = tempheading - track
		if variance < -180:  #ensures that if tempheading = 359 and track = 1 the boat wont try to turn the long way around
			variance += 360
		elif variance > 180:
			variance -= 360
		if variance > 7: #need to turn STBD (decrease rudder angle)
			rudders = self.rudder_angle - 10
		elif variance < -7: #need to turn PORT (increase rudder angle)
			rudders = self.rudder_angle + 10
		else:
			rudders = 70
		#handles tacking when destination is no longer in the no sail zone
		if self.temphead == 50:
			if (boatAng < 315) and (boatAng > 180):
				self.state = 1
			else:
				self.state = 2
		elif self.temphead == 310:
			if (boatAng > 45) and (boatAng < 180):
				self.state = 1
			else:
				self.state = 2
		else:
			print("wtf happened? self.temphead should only be 50 or 310!")
		self.rudder_angle = max(min(rudders, 115), 25) #constrain to 25-115
		return (self.rudder_angle)

	def state3(self, boatAng, track, pointofsail):
		#########
		#state 3#
		#########
		print("current state: 3")
		rudders = self.rudder_angle
		heading = boatAng
		# get and keep the boat on course #
		variance = heading - track
		if variance < -180:
			variance += 360  #ensures that if heading = 359 and track = 1
		elif variance > 180: #the boat wont try to turn the long way around
			variance -= 360
		if variance > 7: #need to turn STBD (decrease rudder angle)
			rudders = self.rudder_angle - 10
		elif variance < -7: #need to turn PORT (increase rudder angle)
			rudders = self.rudder_angle + 10
		else:
			rudders = 70
		
		if pointofsail <= 45 or pointofsail >= 315: #heading up wind, need to restart P2P
			self.state = 1
		else:
			self.state = 3
		self.rudder_angle = max(min(rudders, 115), 25) #constrain to 25-115
		return (self.rudder_angle)
		
			
	
	def getHeading(self):
		X = math.cos(math.radians(self.dest[0]))*math.sin(math.radians(self.difLon()))
		Y = math.cos(math.radians(self.curpos[0]))*math.sin(math.radians(self.dest[0]))-math.sin(math.radians(self.curpos[0]))*math.cos(math.radians(self.dest[0]))*math.cos(math.radians(self.difLon()))
		angle = math.degrees(math.atan2(X,Y))
		if angle < 0:
			angle += 360
		print(X,Y,angle)
		return(angle)

	def getWindToNorth(self, wind, cmpas):
		angle = wind + cmpas
		if angle >= 360:
			while angle >= 360:
				angle -= 360
			return angle
		else:
			return angle
			
	def difLon(self):
		return self.dest[1]-self.curpos[1]

	def difLat(self):
		return self.dest[0]-self.curpos[0]
		
	def getdistance(self):
		# approximate radius of earth in km
		R = 6373.0
		lat1 = math.radians(self.curpos[0])
		lon1 = math.radians(self.curpos[1])
		lat2 = math.radians(self.dest[0])
		lon2 = math.radians(self.dest[1])
		dlon = math.radians(self.difLon())
		dlat = math.radians(self.difLat())
		a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
		distance = R * c #in km
		return(distance)

		

