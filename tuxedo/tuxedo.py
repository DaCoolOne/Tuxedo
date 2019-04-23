import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from rlbot.utils.structures.game_data_struct import Vector3 as UI_Vec3
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator, GameInfoState
# from rlbot.utils.structures.quick_chats import QuickChats
# from random import randint

class Vec3:
	def __init__(self, x=0, y=0, z=0):
		self.x = float(x)
		self.y = float(y)
		self.z = float(z)
	
	def __add__(self, val):
		return Vec3(self.x + val.x, self.y + val.y, self.z + val.z)

	def __sub__(self, val):
		return Vec3(self.x - val.x, self.y - val.y, self.z - val.z)
	
	def __mul__(self, val):
		return Vec3(self.x * val, self.y * val, self.z * val)
	
	def len(self):
		return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
	
	def set(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
	
	def to_rotation(self):
		v = Vector2(self.x, self.y)
		r = math.atan2(v.y, v.x)
		r2 = math.atan(self.z / v.len())
		# r2 = math.pi * 0.3
		return Vector2(r, r2)
	
	def align_to(self, rot):
		v = Vec3(self.x, self.y, self.z)
		
		# Apply yaw
		v.set(math.cos(-rot.yaw) * v.x + math.sin(-rot.yaw) * v.y, math.cos(-rot.yaw) * v.y - math.sin(-rot.yaw) * v.x, v.z)
		
		# Apply pitch
		v.set(v.x, math.cos(rot.pitch) * v.y + math.sin(rot.pitch) * v.z, math.cos(rot.pitch) * v.z - math.sin(rot.pitch) * v.y)
		
		# Apply roll
		v.set(math.cos(rot.pitch) * v.x + math.sin(rot.pitch) * v.z, v.y, math.cos(rot.pitch) * v.z - math.sin(rot.pitch) * v.x)
		
		return v
	
	def UI_Vec3(self):
		return UI_Vec3(self.x, self.y, self.z)
	
	def copy(self):
		return Vec3(self.x, self.y, self.z)
	
	def normal(self, n = 1):
		l = self.len()
		return Vec3(self.x / l * n, self.y / l * n, self.z / l * n)


def sq(n):
	return n * n

def sign(n):
	if n >= 0:
		return 1
	else:
		return -1

def constrain_pi(n):
	while n > math.pi:
		n -= math.pi * 2
	while n < -math.pi:
		n += math.pi * 2
	return n

def constrain(n, mi, ma):
	return max(mi, min(n, ma))

def correct(target, val, mult = 1):
	return min(1, max(-1, constrain_pi(target - val) * mult))

def within(r1, r2, n):
	return abs(constrain_pi(r1-r2)) < n

def angle_between(a, b):
	return math.cos(dot(a, b) / (a.len() * b.len()))

def Make_Vect(v):
	return Vec3(v.x, v.y, v.z)

def perp(v):
	return Vector2(v.y, -v.x)

def Rotation_Vector(rot):
	pitch = float(rot.pitch)
	yaw = float(rot.yaw)
	facing_x = math.cos(pitch) * math.cos(yaw)
	facing_y = math.cos(pitch) * math.sin(yaw)
	facing_z = math.sin(pitch)
	return Vec3(facing_x, facing_y, facing_z)

def clamp(n, _min, _max):
	return max(_min, min(_max, n))

def dot_2D(v1, v2):
	return v1.x*v2.x+v1.y*v2.y

def dot(v1, v2):
	return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z

# Manuevers (Anything that requires very specific input and cannot be interupted)

# Half Flip = back flip + forward pitch as hard as possible then air roll
def Manuever_Half_Flip(self, packet, time):
	if time > 1.4:
		self.controller_state.jump = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0
		self.controller_state.roll = 0.0
		self.controller_state.boost = False
		self.controller_state.throttle = 1.0
	elif time > 1.3 and not packet.game_cars[self.index].double_jumped:
		self.controller_state.jump = True
		self.controller_state.boost = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 1.0
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0
	elif time > 1.1:
		self.controller_state.jump = False
		self.controller_state.boost = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0
	elif time > 0.7:
		self.controller_state.jump = False
		self.controller_state.boost = time < 0.9
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = -1.0
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0
	else:
		self.controller_state.roll = correct(0.0, packet.game_cars[self.index].physics.rotation.roll)
		self.controller_state.jump = False
		self.controller_state.throttle = 1.0
		self.controller_state.boost = True
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0

# Keep controls steady during flip
def Manuever_Flip(self, packet, time):
	if time > 0.9 or packet.game_cars[self.index].double_jumped or time < 0.5:
		self.controller_state.jump = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0
		self.controller_state.roll = 0.0
		self.controller_state.boost = False
		self.controller_state.throttle = self.flip_dir.y
	else:
		self.controller_state.jump = True
		self.controller_state.boost = False
		self.controller_state.yaw = self.flip_dir.x
		self.controller_state.pitch = -self.flip_dir.y
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0

# Maneuver setup
def Enter_Flip(self, packet, dir):
	self.manuever_lock = 1.0
	self.manuever = Manuever_Flip
	self.flip_dir = dir
	self.controller_state.jump = True
	self.controller_state.steer = 0.0

# Used to keep the nose from scraping the ground on front flips
def Enter_High_Flip(self, packet, dir):
	self.manuever_lock = 1.1
	self.manuever = Manuever_Flip
	self.flip_dir = dir
	self.controller_state.jump = True
	self.controller_state.steer = 0.0

def Enter_Half_Flip(self, packet):
	self.manuever_lock = 1.7
	self.manuever = Manuever_Half_Flip
	self.controller_state.jump = True
	self.controller_state.pitch = 1.0
	self.controller_state.boost = False

def Drive_To(self, packet: GameTickPacket, position, boost = False):
	
	my_car = packet.game_cars[self.index]
	
	if my_car.has_wheel_contact:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		car_to_pos = Vector2(position.x, position.y) - car_location
		
		car_to_pos_vel = car_to_pos - Vector2(my_car.physics.velocity.x * 0.1, my_car.physics.velocity.y * 0.1)
		
		car_direction = get_car_facing_vector(my_car)
		
		steer_correction_radians = car_direction.correction_to(car_to_pos)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 7.0))
		
		if car_to_pos.len() > 500:
			self.controller_state.throttle = 1.0
		else:
			self.controller_state.throttle = constrain(car_to_pos_vel.len() / 100, 0, 1)
			if dot_2D(car_to_pos_vel, get_car_facing_vector(my_car)) < 0.0 and my_car.physics.location.z < 50.0:
				self.controller_state.throttle = -self.controller_state.throttle
				self.controller_state.steer = -self.controller_state.steer
		
		self.controller_state.boost = (abs(steer_correction_radians) < 0.2 and car_to_pos.len() > 500.0) or (boost and abs(steer_correction_radians) < math.pi * 0.5)
		
		self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.5 and Make_Vect(my_car.physics.velocity).len() > 200 and my_car.physics.location.z < 50.0
		
		if self.controller_state.handbrake:
			self.controller_state.boost = False
		
		self.controller_state.jump = False
		self.controller_state.pitch = 0.0
	else:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		car_direction = get_car_facing_vector(my_car)
		
		steer_correction_radians = car_direction.correction_to(my_car.physics.velocity)
		
		car_rot = my_car.physics.rotation
		
		self.controller_state.throttle = 1.0
		
		self.controller_state.roll = correct(0.0, car_rot.roll)
		self.controller_state.yaw = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.pitch = correct(0.0, car_rot.pitch)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.handbrake = False
		self.controller_state.boost = False
		self.controller_state.jump = False

def Collect_Boost(self, packet: GameTickPacket, position, do_boost = False, allow_flips = True):
	
	target = Vector2(position.x, position.y)
	
	info = self.get_field_info()
	
	# Acts as maximum search radius
	closest_boost = 1.5
	closest_boost_real = -1
	# Units per second that we're travelling
	if do_boost:
		speed = 3000.0
	else:
		speed = 1000.0
	
	max_dot = 0.75
	
	car = packet.game_cars[self.index]
	
	car_pos = Make_Vect(car.physics.location)
	car_dir = get_car_facing_vector(car)
	
	self.renderer.begin_rendering()
	
	for i in range(info.num_boosts):
		boost = info.boost_pads[i]
		boost_info = packet.game_boosts[i]
		car_to_boost = Make_Vect(boost.location) - car_pos
		time_to_boost = car_to_boost.len() / speed
		d = max_dot
		if boost.is_full_boost: # We are allowed to turn more to get full boosts
			d = d * 0.5
		
		car_to_boost_n = car_to_boost.normal()
		boost_to_pos_n = (Make_Vect(boost.location) - Vec3(target.x, target.y, 0.0)).normal()
		
		if -dot_2D(boost_to_pos_n, car_to_boost_n) > d and boost_info.is_active: #time_to_boost > boost_info.timer:
			self.renderer.draw_line_3d(boost.location, car.physics.location, self.renderer.yellow())
			if closest_boost > time_to_boost / (1 + dot_2D(car_dir, car_to_boost_n)):
				target = Vector2(boost.location.x, boost.location.y)
				closest_boost = time_to_boost / (1 + dot_2D(car_dir, car_to_boost_n))
				closest_boost_real = boost_info
	
	if closest_boost_real != -1:
		self.renderer.draw_string_3d(target.UI_Vec3(), 2, 2, str(closest_boost_real.timer), self.renderer.white())
	self.renderer.end_rendering()
	
	Drive_To(self, packet, target, do_boost)
	self.controller_state.boost = self.controller_state.boost and do_boost
	
	car_direction = get_car_facing_vector(car)
	
	steer_correction_radians = car_direction.correction_to(target - car_pos)
	
	if allow_flips and abs(steer_correction_radians) < 0.025 and Make_Vect(car.physics.velocity).len() > 500 and (target - car_pos).len() > max(1200, Make_Vect(car.physics.velocity).len() * 1.3 + 200):
		Enter_High_Flip(self, packet, Vector2(-math.sin(steer_correction_radians), math.cos(steer_correction_radians)))
	

def Approximate_Time_To_Ball(prediction, car_index, packet, resolution, acceleration = 0, boost = True):
	
	car = packet.game_cars[car_index]
	
	ball_pos = Make_Vect(packet.game_ball.physics.location)
	car_pos = Make_Vect(car.physics.location)
	car_vel = Make_Vect(car.physics.velocity)
	
	arieal_speed = max(1000, car_vel.len()) + acceleration
	
	time_to_reach_ball = 0
	refine = 0
	
	car_to_ball = Make_Vect(prediction.slices[0].physics.location) - car_pos
	
	while refine < resolution:
		car_to_ball = Make_Vect(prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)].physics.location) - car_pos
		
		target_vel_xy2 = Vector2(car_to_ball.x, car_to_ball.y)
		
		time_to_reach_ball = car_to_ball.len() / arieal_speed
		
		target_vel_xy = target_vel_xy2 * (arieal_speed / max(0.01, target_vel_xy2.len()))
		
		z_vel = (car_to_ball.z - 0.5 * packet.game_info.world_gravity_z * time_to_reach_ball * time_to_reach_ball) / time_to_reach_ball
		
		# velocity we want to be going
		target_vel = Vec3(target_vel_xy.x, target_vel_xy.y, z_vel)
		
		refine = refine + 1
	
	if boost:
		return time_to_reach_ball * (2 - car.boost * 0.01)
	else:
		return time_to_reach_ball

def Get_Ball_At_T(prediction, time_to_reach_ball):
	return prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)].physics

def Dribble(self, packet: GameTickPacket, position: Vec3):
	
	prediction = self.get_ball_prediction_struct()
	
	my_car = packet.game_cars[self.index]
	
	car_direction = get_car_facing_vector(my_car)
	
	ball_pos = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.17
	
	ball_predict = Get_Ball_At_T(prediction, Approximate_Time_To_Ball(prediction, self.index, packet, 10, self.arieal_acceleration, True))
	
	if Vector2(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y).len() > 1750:
		ball_pos = Make_Vect(ball_predict.location)
	
	angle = car_direction.correction_to((Make_Vect(my_car.physics.location) - position).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
	
	dir = Vec3(car_direction.x * math.cos(-angle) - car_direction.y * math.sin(-angle) * 3, car_direction.y * math.cos(-angle) + car_direction.x * math.sin(-angle) * 3, 0.0)
	
	dir_2D = Vector2(dir.x * 0.2, dir.y).normal()
	
	multiplier = 1.0
	
	if Vector2(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y).len() > 150.0 and abs(ball_predict.velocity.z) < 10.0 and ball_predict.location.z < 170.0:
		multiplier = 1.5
	
	position = ball_pos + dir * (20 * (constrain((200 - packet.game_ball.physics.velocity.z) * (1 / 200), -1, 1))) * multiplier
	
	position = Vector2(position.x, position.y) # + car_direction * (10.0 * sq(dot_2D(car_direction, dir)) / (1 + Make_Vect(packet.game_ball.physics.velocity).len() * 0.06))
	
	if my_car.has_wheel_contact:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		car_vel = Make_Vect(my_car.physics.velocity) * 0.15
		
		car_to_pos = Vector2(position.x, position.y) - car_location - car_vel
		
		steer_correction_radians = car_direction.correction_to(car_to_pos)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		if dot_2D(car_to_pos, car_direction) > 0.0 or car_to_pos.len() > 250:
			self.controller_state.throttle = constrain(car_to_pos.len() / 10, -1, 1)
		else:
			self.controller_state.throttle = -constrain(car_to_pos.len() / 10, -1, 1)
			self.controller_state.steer = - self.controller_state.steer
		
		if car_to_pos.len() > 500.0:
			self.controller_state.boost = abs(steer_correction_radians) < 0.2 and (car_to_pos - car_vel * 10).len() > 200 and dot_2D(car_to_pos - car_vel * 3, car_to_pos) > 0.0
		else:
			l = car_to_pos.len()
			self.controller_state.boost = l < 30 and l > 25 and ball_pos.z > 110 and ball_pos.z < 140
		
		self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.5 and Make_Vect(my_car.physics.velocity).len() > 200 and car_to_pos.len() > 500
		
		self.controller_state.jump = False
		self.controller_state.pitch = 0.0
		
		self.renderer.begin_rendering()
		
		if ball_pos.z < 200.0 and car_to_pos.len() < 350:
			count = 0
			while count < packet.num_cars:
				car = packet.game_cars[count]
				l = Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity) * 0.4 - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.4
				self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity)).UI_Vec3(), self.renderer.white())
				if l.len() < 500.0 and self.index != count and not car.is_demolished and car.physics.location.z + car.physics.velocity.z * 0.2 < 350.0:
					if Vector2(car_to_pos.x, car_to_pos.y).len() < 100 and ball_pos.z < 130:
						Enter_Flip(self, packet, Vector2(-dir_2D.x, -dir_2D.y))
					else:
						ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
						
						Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
					self.controller_state.boost = True
				count += 1
		
		# self.renderer.draw_line_3d(position.toVector3(), UI_Vec3(my_car.physics.location.x, my_car.physics.location.y, 100.0), self.renderer.green())
		
		self.renderer.end_rendering()
		
	else:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		steer_correction_radians = car_direction.correction_to(my_car.physics.velocity)
		
		car_rot = my_car.physics.rotation
		
		self.controller_state.throttle = 1.0
		
		self.controller_state.roll = correct(0.0, car_rot.roll)
		self.controller_state.yaw = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.pitch = correct(0.0, car_rot.pitch)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.handbrake = False
		self.controller_state.boost = False
		self.controller_state.jump = False

def Get_Ball_On_Car(self, packet: GameTickPacket, direction: Vec3):
	
	prediction = self.get_ball_prediction_struct()
	
	my_car = packet.game_cars[self.index]
	
	car_direction = get_car_facing_vector(my_car)
	
	ball_pos = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.17
	
	ball_predict = Get_Ball_At_T(prediction, Approximate_Time_To_Ball(prediction, self.index, packet, 10, 200, True))
	
	if Vector2(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y).len() > 1750:
		ball_pos = Make_Vect(ball_predict.location)
	
	car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
	
	car_to_pos = Vector2(ball_pos.x, ball_pos.y) - car_location
	
	car_to_real_pos = Vector2(packet.game_ball.physics.location.x, packet.game_ball.physics.location.y) - car_location
	
	if my_car.has_wheel_contact:
		
		if car_to_real_pos.len() < 130.0:
			
			v = (Make_Vect(my_car.physics.velocity) - Make_Vect(ball_predict.velocity)) * 0.05
			
			self.controller_state.boost = False
			self.controller_state.throttle = 0.0
			self.controller_state.steer = constrain(car_direction.correction_to(car_to_real_pos) * 10, -1, 1)
			
			self.controller_state.pitch = 0.0
			self.controller_state.roll = 0.0
			self.controller_state.yaw = 0.0
			
			self.controller_state.jump = False
			self.controller_state.handbrake = False
			
		else:
			n = perp(car_to_pos).normal()
			v = (Make_Vect(my_car.physics.velocity) - Make_Vect(ball_predict.velocity)) * 0.05
			car_vel = Make_Vect(my_car.physics.velocity)
			if car_to_real_pos.len() > 1000:
				Collect_Boost(self, packet, ball_pos + Vec3(n.x * 110, n.y * 110, 0.0) * sign(my_car.physics.location.x)) #Vec3(-sign(car_to_pos.x) * 150, 0, 0))
			else:
				Drive_To(self, packet, ball_pos + Vec3(n.x * 110, n.y * 110, 0.0) * sign(my_car.physics.location.x))
			self.controller_state.boost = False
			car_ball = Vector2(ball_pos.x, ball_pos.y) - car_location - Vector2(car_vel.x, car_vel.y) * 3
			self.controller_state.boost = self.controller_state.boost and car_ball.len() > 1000
		
		if ball_pos.z < 180.0 and car_to_pos.len() < 300:
			count = 0
			while count < packet.num_cars:
				car = packet.game_cars[count]
				l = (Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity) * 0.2 - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.2).len()
				if l < 700.0 and self.index != count and not car.is_demolished:
					if car_to_pos.len() < 100:
						Enter_Flip(self, packet, Vector2(-dir_2D.x, -dir_2D.y))
					else:
						ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
						
						Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
					self.controller_state.boost = True
				count += 1
		
	else:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		steer_correction_radians = car_direction.correction_to(my_car.physics.velocity)
		
		car_rot = my_car.physics.rotation
		
		self.controller_state.throttle = 1.0
		
		self.controller_state.roll = correct(0.0, car_rot.roll)
		self.controller_state.yaw = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.pitch = correct(0.0, car_rot.pitch)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.handbrake = False
		self.controller_state.boost = False
		self.controller_state.jump = False

def Attack_Ball(self, packet: GameTickPacket, aim_pos: Vec3, ball_predict: Vec3):
	car = packet.game_cars[self.index]
	
	aim = (ball_predict - aim_pos).normal() * -50
	car_to_pos = ball_predict - Make_Vect(car.physics.location)
	
	if car_to_pos.len() < 300:
		car_direction = get_car_facing_vector(car)
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.1 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.1
		ang = car_direction.correction_to(c_p)
		Enter_Flip(self, packet, Vector2(-math.sin(ang), math.cos(ang)))
	else:
		Drive_To(self, packet, ball_predict - aim)

def Attack_Aim_Ball(self, packet: GameTickPacket, aim_pos: Vec3, ball_predict: Vec3):
	car = packet.game_cars[self.index]
	
	aim = (ball_predict - aim_pos).normal() * -250
	car_to_pos = ball_predict - Make_Vect(car.physics.location)
	
	if car_to_pos.len() < 300:
		car_direction = get_car_facing_vector(car)
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.1 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.1
		ang = car_direction.correction_to(c_p)
		Enter_Flip(self, packet, Vector2(-math.sin(ang), math.cos(ang)))
	else:
		Drive_To(self, packet, ball_predict - aim)

def Hit_Ball_To(self, packet: GameTickPacket, aim_pos: Vec3):
	exit_condition = True
	
	info = self.get_field_info()
	
	own_goal = 0
	for i in range(info.num_goals):
		goal = info.goals[i]
		if goal.team_num == packet.game_cars[self.index].team:
			own_goal = goal
			break
	
	if self.is_arieal:
		
		self.jump_timer += self.delta
		
		if self.flip_timer > 0.0:
			self.flip_timer -= self.delta
		else:
			prediction = self.get_ball_prediction_struct()
			
			car = packet.game_cars[self.index]
			
			ball_pos = Make_Vect(packet.game_ball.physics.location)
			car_pos = Make_Vect(car.physics.location)
			car_vel = Make_Vect(car.physics.velocity)
			
			self.arieal_speed = max(1500, car_vel.len()) + self.arieal_acceleration
			
			time_to_reach_ball = 0
			refine = 0
			
			car_to_ball = 1
			ball_offset = 1
			
			# Calculate impluse vector
			
			while refine < 20.0:
				ball = prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)]
				
				# Calculate an offset vector to use when steering the car
				vel = Make_Vect(ball.physics.velocity)
				ball_to_target = Make_Vect(ball.physics.location) - aim_pos
				ball_offset = (ball_to_target.normal(max(500, vel.len() * 1.5)) - vel).normal(-70)
				
				car_to_ball = Make_Vect(ball.physics.location) - car_pos + ball_offset
				
				target_vel_xy2 = Vector2(car_to_ball.x, car_to_ball.y)
				
				time_to_reach_ball = car_to_ball.len() / self.arieal_speed
				
				target_vel_xy = target_vel_xy2 * (self.arieal_speed / target_vel_xy2.len())
				
				z_vel = (car_to_ball.z - 0.5 * packet.game_info.world_gravity_z * time_to_reach_ball * time_to_reach_ball) / time_to_reach_ball
				
				# velocity we want to be going
				target_vel = Vec3(target_vel_xy.x, target_vel_xy.y, z_vel)
				
				refine = refine + 1
			
			# self.renderer.begin_rendering()
			# self.renderer.draw_line_3d(ball_pos.UI_Vec3(), (ball_pos + ball_offset).UI_Vec3(), self.renderer.red())
			# self.renderer.draw_line_3d(aim_pos.UI_Vec3(), (aim_pos + Vec3(0, 0, 100)).UI_Vec3(), self.renderer.yellow())
			# self.renderer.end_rendering()
			
			impulse = target_vel - car_vel
			
			rot = impulse.to_rotation()
			
			car_rot = car.physics.rotation
			
			car_rot_vel = Make_Vect(car.physics.angular_velocity) #.align_to(car_rot)
			
			car_to_ball_2D = Vector2(ball_pos.x, ball_pos.y) - Vector2(car_pos.x, car_pos.y)
			
			self.controller_state.yaw = correct(rot.x, car_rot.yaw, 0.5)
			self.controller_state.pitch = correct(rot.y + 0.1, car_rot.pitch, 1.0)
			
			self.controller_state.steer = correct(rot.x, car_rot.yaw, 1.0)
			
			a = angle_between(impulse, Rotation_Vector(car_rot))
			pitch_a = constrain_pi(car_rot.pitch - rot.y)
			self.controller_state.boost = impulse.len() > a * 200.0 and abs(pitch_a) < math.cos(rot.y) * car_to_ball_2D.len() / car_to_ball.z * 2.0 and a < math.pi * 0.5
			
			self.controller_state.roll = correct(0.0, car_rot.roll)
			
			self.controller_state.throttle = 1.0
			
			self.controller_state.jump = self.jump_timer > 0.3 and not car.double_jumped and (impulse.z > impulse.len() * 0.5 or impulse.z > 250.0)
			
			# self.renderer.begin_rendering()
			
			# self.renderer.draw_line_3d(car.physics.location, (car_pos + impulse).toVector3(), self.renderer.red())
			# self.renderer.draw_line_3d(car.physics.location, (car_pos + car_to_ball).toVector3(), self.renderer.white())
			
			# self.renderer.end_rendering()
			
			if self.controller_state.jump:
				self.controller_state.yaw = 0.0
				self.controller_state.pitch = 0.0
				self.controller_state.roll = 0.0
			
			# Flip into the ball
			if not car.double_jumped and car_to_ball.len() < 600.0 and abs(car_to_ball.z - 30) < 150.0 and car_pos.z > 50.0:
				self.controller_state.jump = True
				yaw = self.controller_state.yaw
				self.controller_state.yaw = math.sin(yaw)
				self.controller_state.pitch = -math.cos(yaw)
				self.controller_state.roll = 0.0
				self.flip_timer = 0.9
		
	else:
		if packet.game_cars[self.index].has_wheel_contact:
			prediction = self.get_ball_prediction_struct()
			
			car = packet.game_cars[self.index]
			
			car_pos = Make_Vect(car.physics.location)
			car_vel = Make_Vect(car.physics.velocity)
			
			self.arieal_speed = max(800, car_vel.len() + self.arieal_acceleration)
			
			time_to_reach_ball = 0
			refine = 0
			
			car_to_ball = Make_Vect(prediction.slices[0].physics.location) - car_pos
			
			# Calculate impluse vector
			
			while refine < 20.0:
				
				ball_location = Make_Vect(prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)].physics.location)
				
				car_to_ball = ball_location - car_pos
				
				time_to_reach_ball = car_to_ball.len() / self.arieal_speed
				
				refine = refine + 1
			
			if ball_location.z < 150:
				Attack_Ball(self, packet, aim_pos, ball_location)
			else:
				ball_z = ball_location.z - 100
				ball_location = Vector2(ball_location.x, ball_location.y)
				car_location = Vector2(car_pos.x, car_pos.y)
				car_direction = get_car_facing_vector(car)
				car_to_target = ball_location - car_location
				steer_correction_radians = car_direction.correction_to(car_to_target) * 3.0
				
				car_to_ball_2D = ball_location - car_location
				car_to_ball_plus_vel = car_to_ball_2D - Vector2(car_vel.x, car_vel.y) * 0.75
				car_to_ball_plus_vel_2 = car_to_ball_2D - Vector2(car_vel.x, car_vel.y) * 1.25
				
				take_off_angle = math.atan2(car_to_ball.z, car_to_ball_plus_vel_2.len())
				
				self.controller_state.throttle = min(1, max(-1, -(take_off_angle - math.pi * 0.2) * 2))
				
				if self.controller_state.throttle < 0.0:
					steer_correction_radians = -steer_correction_radians
				
				self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.8
				self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians))
				self.controller_state.boost = car_to_ball_plus_vel.len() > max(ball_z * 3.5, 1000) and abs(steer_correction_radians) < math.pi * 0.3
				
				if abs(steer_correction_radians) < 0.3 and self.controller_state.throttle > -0.5:
					self.line_up_time += self.delta
				else:
					self.line_up_time = 0
				
				if self.line_up_time > 0.2 and abs(take_off_angle - math.pi * 0.2) < 0.2 and self.controller_state.throttle >= 0.0 and time_to_reach_ball * 40 < car.boost:
					self.controller_state.jump = True
					self.is_arieal = True
			
		else:
			Drive_To(self, packet, aim_pos)
		# if abs(steer_correction_radians) > math.pi * 0.8 and car_to_ball.len() > 1000.0:
			# Enter_Half_Flip(self, packet)
		
		# self.renderer.begin_rendering()
		
		# self.renderer.draw_line_3d(car.physics.location, (car_pos + car_to_ball).toVector3(), self.renderer.white())
		# self.renderer.draw_string_3d(car.physics.location, 2, 2, str(take_off_angle / math.pi), self.renderer.white())
		
		# self.renderer.end_rendering()
	
	if not packet.game_info.is_round_active:
		self.reset()
	
	return exit_condition

def Strategy_Ones(self, packet: GameTickPacket):
	
	my_car = packet.game_cars[self.index]
	
	opponent_goal = 0
	my_goal = 0
	
	# self.renderer.begin_rendering()
	
	for i in range(self.get_field_info().num_goals):
		goal = self.get_field_info().goals[i]
		# self.renderer.draw_string_3d((Make_Vect(goal.location) + Vector3(0, 0, 500)).UI_Vec3(), 2, 2, str(goal.team_num) + ',' + str(counter), self.renderer.white())
		if goal.team_num == my_car.team:
			my_goal = goal
		else:
			opponent_goal = goal
	
	# self.renderer.end_rendering()
	
	v = Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)
	
	closest_car_dist = 10000
	
	closest_car_pos = Vec3()
	
	ball_pos = Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 10, self.arieal_acceleration, False))
	
	for i in range(packet.num_cars):
		car = packet.game_cars[i]
		d = (Make_Vect(car.physics.location) - Make_Vect(my_car.physics.location)).len()
		if car.team != my_car.team and not car.is_demolished and d < closest_car_dist:
			closest_car_dist = d
			closest_car_pos = Make_Vect(car.physics.location)
	
	if packet.game_info.is_kickoff_pause:
		if (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() > 750:
			Collect_Boost(self, packet, Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.direction) * 200, True, False)
		else:
			car_direction = get_car_facing_vector(my_car)
			ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.3).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
			
			Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
	elif (dot(Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location) + Make_Vect(my_goal.direction) * 500, my_goal.direction) < 0.0 or (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() < 3000.0) and not ((Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).len() < 200): # and closest_car_dist > 1000):
		if dot(Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location), Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * 200) > 0.0:
			Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location))
		else:
			Collect_Boost(self, packet, Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * -100, True)
	else:
		if Vector2(v.x, v.y).len() < 100.0 or packet.game_ball.physics.location.z > 120:
			Dribble(self, packet, Make_Vect(opponent_goal.location))
		# If in front of car
		elif dot(closest_car_pos - Make_Vect(my_car.physics.location), opponent_goal.direction) > 0.0:
			print("Attack Mode")
			prediction = self.get_ball_prediction_struct()
			ball_pos = Make_Vect(Get_Ball_At_T(prediction, Approximate_Time_To_Ball(prediction, self.index, packet, 10, self.arieal_acceleration, True)).location)
			Attack_Aim_Ball(self, packet, Make_Vect(opponent_goal.location), ball_pos)
		elif closest_car_dist < 2000:
			Dribble(self, packet, Make_Vect(opponent_goal.location))
		else:
			Get_Ball_On_Car(self, packet, Make_Vect(my_goal.direction))
	
	# self.renderer.begin_rendering()
	
	# self.renderer.draw_string_3d((Make_Vect(my_goal.location) + Vector3(0, 0, 1000)).UI_Vec3(), 2, 2, "My Goal", self.renderer.white())
	
	# self.renderer.draw_string_3d((Make_Vect(opponent_goal.location) + Vector3(0, 0, 1000)).UI_Vec3(), 2, 2, "Other Goal", self.renderer.white())
	
	# self.renderer.end_rendering()
	

class Tuxedo(BaseAgent):
	def reset(self):
		self.manuever_lock = -1.0
		self.flip_timer = 0.0
		self.is_arieal = False
		self.line_up_time = 0
		self.target_location = Vector2(1, 1)
		self.jump_timer = 0
		self.arieal_speed = 2000.0
		self.arieal_acceleration = 300.0
		self.delta = 0.0
	
	def initialize_agent(self):
		#This runs once before the bot starts up
		self.controller_state = SimpleControllerState()
		self.prev_time = 0.0
		self.has_endgame_quickchat = True
		self.reset()
	
	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
		
		self.controller_state = SimpleControllerState()
		
		self.delta = packet.game_info.seconds_elapsed - self.prev_time
		self.prev_time = packet.game_info.seconds_elapsed
		
		# Celebration and end of game quick chats
		if packet.game_info.is_match_ended:
			my_car = packet.game_cars[self.index]
			
			if my_car.physics.rotation.pitch > -0.75:
				self.controller_state.pitch = -1.0
				self.controller_state.boost = False
			else:
				self.controller_state.pitch = 1.0
				self.controller_state.boost = True
			
			self.controller_state.handbrake = False
			self.controller_state.jump = my_car.has_wheel_contact
			self.controller_state.yaw = 0.0
			self.controller_state.roll = 0.0
			
		# Match Logic
		else:
			if self.manuever_lock <= 0.0:
				# Get_Ball_On_Car(self, packet, Make_Vect(self.get_field_info().goals[0].direction))
				# Collect_Boost(self, packet, packet.game_ball.physics.location, True)
				Strategy_Ones(self, packet)
			else:
				self.manuever_lock = self.manuever_lock - self.delta
				self.manuever(self, packet, self.manuever_lock)
			# Drive_To(self, packet, packet.game_ball.physics.location)
		
		return self.controller_state

class Vector2:
	def __init__(self, x=0, y=0):
		self.x = float(x)
		self.y = float(y)
	
	def __add__(self, val):
		return Vector2(self.x + val.x, self.y + val.y)

	def __sub__(self, val):
		return Vector2(self.x - val.x, self.y - val.y)
		
	def __mul__(self, val):
		return Vector2(self.x * val, self.y * val)
	
	def correction_to(self, ideal):
		# The in-game axes are left handed, so use -x
		current_in_radians = math.atan2(self.y, -self.x)
		ideal_in_radians = math.atan2(ideal.y, -ideal.x)

		correction = ideal_in_radians - current_in_radians

		# Make sure we go the 'short way'
		if abs(correction) > math.pi:
			if correction < 0:
				correction += 2 * math.pi
			else:
				correction -= 2 * math.pi

		return correction
	
	def len(self):
		return math.sqrt(self.x * self.x + self.y * self.y)
	
	def normal(self, n = 1):
		l = max(0.0001, self.len())
		return Vector2(self.x / l * n, self.y / l * n)
	
	def toVector3(self):
		return Vec3(self.x, self.y, 0)
	
	def UI_Vec3(self):
		return UI_Vec3(self.x, self.y, 0)

def get_car_facing_vector(car):
	pitch = float(car.physics.rotation.pitch)
	yaw = float(car.physics.rotation.yaw)
	
	# facing_x = math.cos(pitch) * math.cos(yaw)
	# facing_y = math.cos(pitch) * math.sin(yaw)
	
	facing_x = math.cos(yaw)
	facing_y = math.sin(yaw)
	
	return Vector2(facing_x, facing_y)
