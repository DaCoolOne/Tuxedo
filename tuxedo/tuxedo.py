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
	
	def flatten(self):
		return Vec3(self.x, self.y, 0)
	
	def normal(self, n = 1):
		l = self.len()
		return Vec3(self.x / l * n, self.y / l * n, self.z / l * n)

class Quat:
	
	def __init__(self, w, x, y, z):
		self.w = w
		self.x = x
		self.y = y
		self.z = z
	
	def To_Euler(q):
		return Vec3(
			# arctan( 2(ab + cd) / (a^2-b^2-c^2+d^2) )
			math.atan((2 * (q.w * q.x + q.y * q.z)) / (q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)),
			# arcsin( 2(bd - ac) )
			-math.asin(2 * (q.x * q.z - q.w * q.y)),
			# arctan( 2(ad + bc) / (a^2+b^2-c^2-d^2) )
			math.atan((2 * (q.w * q.z + q.x * q.y)) / (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)),
		)
	
	def From_Angle_Axis(axis, angle):
		s_a = math.sin(angle)
		return Quat(math.cos(angle), s_a * axis.x, s_a * axis.y, s_a * axis.z)
	
	def From_Euler(x, y, z):
		vec = Vec3(math.cos(x) * math.cos(y), math.sin(x) * math.cos(y), math.sin(y))
		return Quat.From_Angle_Axis(vec, z)
	
	def __add__(self, val):
		return Quat(self.w + val.w, self.x + val.x, self.y + val.y, self.z + val.z)
	
	def __sub__(self, val):
		return Quat(self.w - val.w, self.x - val.x, self.y - val.y, self.z - val.z)
	
	def __mul__(a, b):
		# Quaternion multiplication
		if isinstance(b, Quat):
			return Quat(
				a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
				a.w * b.x + a.x * b.w - a.y * b.z - a.z * b.y,
				a.w * b.y - a.x * b.z - a.y * b.w - a.z * b.x,
				a.w * b.z + a.x * b.y - a.y * b.x - a.z * b.w)
		# Vector multiplication
		elif isinstance(b, Vec3):
			v_as_q = Quat(0, b.x, b.y, b.z)
			res = a * v_as_q * a.inverse()
			return Vec3(res.x, res.y, res.z)
		# Float
		else:
			return Quat(a.w * b, a.x * b, a.y * b, a.z * b)
	
	def len(self):
		return math.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
	
	def normal(self):
		l = 1 / self.len()
		return Quat(self.w*l,self.x*l,self.y*l,self.z*l)
	
	def inverse(self):
		Quat(self.w,-self.x,-self.y,-self.z)
	

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

def perp_3D(v):
	return Vec3(v.y, -v.x, v.z)

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
		self.controller_state.boost = False #time < 0.9
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = -1.0
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0
	else:
		self.controller_state.roll = correct(0.0, packet.game_cars[self.index].physics.rotation.roll)
		self.controller_state.jump = False
		self.controller_state.throttle = 1.0
		self.controller_state.boost = False #True
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0

# Keep controls steady during flip
def Manuever_Flip(self, packet, time):
	if time > 0.6 or packet.game_cars[self.index].double_jumped or time < 0.5:
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
	self.manuever_lock = 0.7
	self.manuever = Manuever_Flip
	self.flip_dir = dir
	self.controller_state.jump = True
	self.controller_state.steer = 0.0

# Used to keep the nose from scraping the ground on front flips
def Enter_High_Flip(self, packet, dir):
	self.manuever_lock = 0.8
	self.manuever = Manuever_Flip
	self.flip_dir = dir
	self.controller_state.jump = True
	self.controller_state.steer = 0.0

def Enter_Half_Flip(self, packet):
	self.manuever_lock = 1.5
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
			
			self.controller_state.boost = (abs(steer_correction_radians) < 0.2 and car_to_pos.len() > 500.0) and boost
			
			self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.5 and Make_Vect(my_car.physics.velocity).flatten().len() < 1000 and Make_Vect(my_car.physics.velocity).flatten().len() > 400 and my_car.physics.location.z < 50.0
			
		else:
			self.controller_state.boost = False
			self.controller_state.handbrake = False
			self.controller_state.throttle = constrain(car_to_pos_vel.len() / 100, 0, 1)
			if dot_2D(car_to_pos_vel, get_car_facing_vector(my_car)) < 0.0 and my_car.physics.location.z < 50.0:
				self.controller_state.throttle = -self.controller_state.throttle
				self.controller_state.steer = -self.controller_state.steer
		
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
		self.controller_state.yaw = -max(-1.0, min(1.0, steer_correction_radians * 2.0 + my_car.physics.angular_velocity.z * 0.25))
		
		self.controller_state.pitch = correct(0.0, car_rot.pitch)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.handbrake = False
		self.controller_state.boost = False
		self.controller_state.jump = False
	
	self.renderer.draw_line_3d(UI_Vec3(position.x, position.y, 100.0), my_car.physics.location, self.renderer.green())

def Collect_Boost(self, packet: GameTickPacket, position, do_boost = False, allow_flips = True):
	
	car = packet.game_cars[self.index]
	
	if car.boost > 95:
		Drive_To(self, packet, position, do_boost)
	
	end_target = Vector2(position.x, position.y)
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
	
	car_pos = Make_Vect(car.physics.location)
	car_dir = get_car_facing_vector(car)
	
	# self.renderer.begin_rendering()
	
	# Pick boost to drive to
	for i in range(info.num_boosts):
		boost = info.boost_pads[i]
		boost_info = packet.game_boosts[i]
		car_to_boost = Make_Vect(boost.location) - car_pos
		time_to_boost = car_to_boost.len() / speed
		d = max_dot
		if boost.is_full_boost: # We are allowed to turn more to get full boosts
			d = d * 0.25
		
		car_to_boost_n = car_to_boost.normal()
		boost_to_pos_n = (Make_Vect(boost.location) - Vec3(target.x, target.y, 0.0)).normal()
		
		if -dot_2D(boost_to_pos_n, car_to_boost_n) > d and boost_info.is_active: #time_to_boost > boost_info.timer:
			# self.renderer.draw_line_3d(boost.location, car.physics.location, self.renderer.yellow())
			if closest_boost > time_to_boost / (1 + d + dot_2D(car_dir, car_to_boost_n)):
				target = Vector2(boost.location.x, boost.location.y)
				closest_boost = time_to_boost / (1 + d + dot_2D(car_dir, car_to_boost_n))
				closest_boost_real = boost_info
	
	# if closest_boost_real != -1:
		# self.renderer.draw_string_3d(target.UI_Vec3(), 2, 2, str(closest_boost_real.timer), self.renderer.white())
	# self.renderer.end_rendering()
	
	Drive_To(self, packet, target, do_boost)
	self.controller_state.boost = self.controller_state.boost and do_boost
	
	car_direction = get_car_facing_vector(car)
	
	steer_correction_radians = car_direction.correction_to(target - car_pos)
	
	if allow_flips and abs(steer_correction_radians) < 0.025 and Make_Vect(car.physics.velocity).len() > 500 and (target - car_pos).len() > max(2000, Make_Vect(car.physics.velocity).len() * 2):
		Enter_High_Flip(self, packet, Vector2(-math.sin(steer_correction_radians), math.cos(steer_correction_radians)))
	

def Approximate_Time_To_Ball(prediction, car_index, packet, resolution, acceleration = 0, boost = True):
	
	car = packet.game_cars[car_index]
	
	ball_pos = Make_Vect(packet.game_ball.physics.location)
	car_pos = Make_Vect(car.physics.location)
	car_vel = Make_Vect(car.physics.velocity)
	
	arieal_speed = max(500, car_vel.len()) + acceleration
	
	time_to_reach_ball = 0
	refine = 0
	
	car_to_ball = Make_Vect(prediction.slices[0].physics.location) - car_pos
	
	while refine < resolution:
		car_to_ball = Make_Vect(prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 5)].physics.location) - car_pos
		
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
	return prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 5)].physics

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
		
		if car_to_pos.len() > 300.0:
			self.controller_state.boost = abs(steer_correction_radians) < 0.2 and (car_to_pos - car_vel * 10).len() > 200 and dot_2D(car_to_pos - car_vel * 3, car_to_pos) > 0.0
		else:
			l = car_to_pos.len()
			self.controller_state.boost = l < 30 and l > 25 and ball_pos.z > 110 and ball_pos.z < 140
		
		self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.5 and Make_Vect(my_car.physics.velocity).flatten().len() > 400 and car_to_pos.len() > 500
		
		self.controller_state.jump = False
		self.controller_state.pitch = 0.0
		
		# self.renderer.begin_rendering()
		
		if ball_pos.z < 200.0 and car_to_pos.len() < 350:
			count = 0
			while count < packet.num_cars:
				car = packet.game_cars[count]
				l = Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity) * 0.4 - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.4
				self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity)).UI_Vec3(), self.renderer.white())
				if l.len() < 300.0 and self.index != count and not car.is_demolished and car.physics.location.z + car.physics.velocity.z * 0.4 < 300.0:
					if Vector2(car_to_pos.x, car_to_pos.y).len() < 100 and ball_pos.z < 130:
						Enter_Flip(self, packet, Vector2(-dir_2D.x, -dir_2D.y))
					else:
						ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
						
						Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
					self.controller_state.boost = True
				count += 1
		
		# self.renderer.draw_line_3d(position.toVector3(), UI_Vec3(my_car.physics.location.x, my_car.physics.location.y, 100.0), self.renderer.green())
		
		# self.renderer.end_rendering()
		
	else:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		steer_correction_radians = car_direction.correction_to(my_car.physics.velocity)
		
		car_rot = my_car.physics.rotation
		
		self.controller_state.throttle = 1.0
		
		self.controller_state.roll = correct(0.0, car_rot.roll)
		self.controller_state.yaw = -max(-1.0, min(1.0, steer_correction_radians * 2.0 + my_car.physics.angular_velocity.z * 0.25))
		
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
		
		if car_to_real_pos.len() < 150.0:
			Drive_To(self, packet, ball_predict.location)
		else:
			v = Make_Vect(ball_predict.velocity)
			n = perp(v).normal() * 130
			if dot_2D(direction, n) < 0.0:
				n = n * -1
			b_p = Vector2(ball_predict.location.x, ball_predict.location.y)
			Drive_To(self, packet, b_p + n)
		
		# if car_to_real_pos.len() < 150.0:
			# v = (Make_Vect(my_car.physics.velocity) - Make_Vect(ball_predict.velocity)) * 0.05
			
			# self.controller_state.boost = False
			# self.controller_state.throttle = 0.0
			# self.controller_state.steer = constrain(car_direction.correction_to(car_to_real_pos) * 10, -1, 1)
			
			# self.controller_state.pitch = 0.0
			# self.controller_state.roll = 0.0
			# self.controller_state.yaw = 0.0
			
			# self.controller_state.jump = False
			# self.controller_state.handbrake = False
			
		# else:
			# n = perp(car_to_pos).normal()
			# v = (Make_Vect(my_car.physics.velocity) - Make_Vect(ball_predict.velocity)) * 0.05
			# car_vel = Make_Vect(my_car.physics.velocity)
			# if car_to_real_pos.len() > 1000:
				# Collect_Boost(self, packet, ball_pos + Vec3(n.x * 130, n.y * 130, 0.0) * -sign(my_car.physics.location.x), True) #Vec3(-sign(car_to_pos.x) * 150, 0, 0))
			# else:
				# Drive_To(self, packet, ball_pos + Vec3(n.x * 130, n.y * 130, 0.0) * -sign(my_car.physics.location.x))
			# self.controller_state.boost = False
			# car_ball = Vector2(ball_pos.x, ball_pos.y) - car_location - Vector2(car_vel.x, car_vel.y) * 3
			# self.controller_state.boost = self.controller_state.boost and car_ball.len() > 1000
		
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
	
	car_to_ball_real = Make_Vect(packet.game_ball.physics.location) - Make_Vect(car.physics.location)
	
	if car_to_ball_real.len() < 300:
		car_direction = get_car_facing_vector(car)
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.15 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.15
		ang = car_direction.correction_to(c_p)
		Enter_Flip(self, packet, Vector2(-math.sin(ang) * 2, math.cos(ang)).normal())
	else:
		Drive_To(self, packet, ball_predict - aim, True)

def Attack_Aim_Ball(self, packet: GameTickPacket, aim_pos: Vec3, ball_predict: Vec3):
	car = packet.game_cars[self.index]
	
	aim = (ball_predict - aim_pos).normal() * -150
	car_to_pos = ball_predict - Make_Vect(car.physics.location)
	car_to_aim = aim_pos - Make_Vect(car.physics.location)
	
	car_to_ball_real = Make_Vect(packet.game_ball.physics.location) - Make_Vect(car.physics.location)
	
	if dot(car_to_pos, car_to_aim) < 0.0:
		Drive_To(self, packet, Vector2(ball_predict.x, ball_predict.y - sign(car_to_aim.y) * 1000), True)
	elif car_to_ball_real.len() < 300:
		car_direction = get_car_facing_vector(car)
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.15 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.15
		ang = car_direction.correction_to(c_p)
		Enter_Flip(self, packet, Vector2(-math.sin(ang) * 2.0, math.cos(ang)).normal())
	elif car_to_ball_real.len() < 500:
		Drive_To(self, packet, Make_Vect(packet.game_ball.physics.location) - aim, True)
	elif car_to_ball_real.len() > 1000:
		Drive_To(self, packet, Make_Vect(packet.game_ball.physics.location) - aim, True)
	else:
		Drive_To(self, packet, ball_predict - aim, True)
	
	self.renderer.draw_line_3d(car.physics.location, (ball_predict - aim).UI_Vec3(), self.renderer.yellow())
	

def Hit_Ball_To(self, packet: GameTickPacket, aim_pos: Vec3, fallback: Vec3):
	exit_condition = True
	
	info = self.get_field_info()
	
	own_goal = 0
	for i in range(info.num_goals):
		goal = info.goals[i]
		if goal.team_num == packet.game_cars[self.index].team:
			own_goal = goal
			break
	
	target_take_off_angle = math.pi * 0.35
	
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
			
			self.arieal_speed = max(1300, car_vel.len() + self.arieal_acceleration)
			
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
			
			impulse = impulse + Vec3(0, 0, impulse.len() * 0.5)
			
			rot = impulse.to_rotation()
			
			car_rot = car.physics.rotation
			
			car_rot_vel = Make_Vect(car.physics.angular_velocity) #.align_to(car_rot)
			
			car_rot_vel.x = -car_rot_vel.x
			
			car_to_ball_2D = Vector2(ball_pos.x, ball_pos.y) - Vector2(car_pos.x, car_pos.y)
			
			quat_vel = Quat.From_Angle_Axis(car_rot_vel, car_rot_vel.len()).normal()
			quat_rot = Quat.From_Euler(car_rot.yaw, car_rot.pitch, car_rot.roll).normal()
			
			# Translate velocity to local units
			local_quat_vel = (quat_rot * quat_vel).normal()
			
			local_euler = local_quat_vel.To_Euler()
			
			# Improving this
			rot_ang_const = 0.25
			self.controller_state.yaw = correct(rot.x, car_rot.yaw + local_euler.x * rot_ang_const, 2.0)
			if car.double_jumped:
				self.controller_state.pitch = correct(rot.y, car_rot.pitch + local_euler.y * rot_ang_const, 2.0)
			else:
				self.controller_state.pitch = correct(rot.y, car_rot.pitch - math.pi * 0.1, 4.0)
				# self.controller_state.pitch = 0.8
			self.controller_state.roll = correct(0.0, car_rot.roll + local_euler.z * rot_ang_const, 2.0)
			
			self.controller_state.steer = correct(rot.x, car_rot.yaw, 1.0)
			
			a = angle_between(impulse, Rotation_Vector(car_rot))
			pitch_a = constrain_pi(car_rot.pitch - rot.y)
			self.controller_state.boost = impulse.len() > a * 200.0 and pitch_a > -0.5 and a < math.pi * 0.5
			
			self.controller_state.throttle = 1.0
			
			self.controller_state.jump = self.jump_timer > 0.2 and not car.double_jumped #  and car_to_ball.z > 300 # impulse.z > 250.0
			
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
			
			self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + impulse).UI_Vec3(), self.renderer.green())
			
			self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + car_to_ball).UI_Vec3(), self.renderer.green())
			
	else:
		self.is_arieal = False
		self.jump_timer = 0.0
		
		if packet.game_cars[self.index].has_wheel_contact:
			prediction = self.get_ball_prediction_struct()
			
			car = packet.game_cars[self.index]
			
			car_pos = Make_Vect(car.physics.location)
			car_vel = Make_Vect(car.physics.velocity)
			
			self.arieal_speed = max(1300, car_vel.len() + self.arieal_acceleration)
			
			time_to_reach_ball = 0
			refine = 0
			
			car_to_ball = Make_Vect(prediction.slices[0].physics.location) - car_pos
			
			# Calculate impluse vector
			
			while refine < 20.0:
				
				ball_location = Make_Vect(prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)].physics.location)
				
				car_to_ball = ball_location - car_pos
				
				time_to_reach_ball = car_to_ball.len() / self.arieal_speed
				
				refine = refine + 1
			
			# if ball_location.z < 150:
				# Attack_Ball(self, packet, aim_pos, ball_location)
			# else:
			ball_z = ball_location.z - 100
			ball_location = Vector2(ball_location.x, ball_location.y)
			car_location = Vector2(car_pos.x, car_pos.y)
			car_direction = get_car_facing_vector(car)
			car_to_target = ball_location - car_location
			steer_correction_radians = car_direction.correction_to(car_to_target) * 3.0
			
			car_to_ball_2D = ball_location - car_location
			car_to_ball_plus_vel = car_to_ball_2D - Vector2(car_vel.x, car_vel.y) * 2
			car_to_ball_plus_vel_2 = car_to_ball_2D - Vector2(car_vel.x, car_vel.y) * 3
			
			take_off_angle = math.atan2(car_to_ball.z, car_to_ball_plus_vel_2.len())
			
			# if take_off_angle > math.pi * 0.2:
				# Collect_Boost(self, packet, fallback, True, False)
			# else:
			# Drive_To(self, packet, ball_location, False)
			self.controller_state.throttle = min(1, max(-1, -(take_off_angle - target_take_off_angle) * 2))
			
			self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.8 and car_vel.len() > 400
			self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians))
			self.controller_state.boost = take_off_angle > target_take_off_angle and abs(steer_correction_radians) < math.pi * 0.3
			
			if self.controller_state.throttle < 0.0:
				self.controller_state.steer = -self.controller_state.steer
			
			if abs(steer_correction_radians) < 0.1:
				self.line_up_time += self.delta
			else:
				self.line_up_time = 0
			
			if self.line_up_time > 0.2 and abs(take_off_angle - target_take_off_angle) < 0.2 and time_to_reach_ball * 40 < car.boost and car_to_ball.z < 1500:
				self.controller_state.jump = True
				self.controller_state.pitch = 1
				self.is_arieal = True
			
			self.renderer.draw_string_3d(car.physics.location, 2, 2, str(take_off_angle - target_take_off_angle), self.renderer.green())
			
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
	
	for i in range(self.get_field_info().num_goals):
		goal = self.get_field_info().goals[i]
		if goal.team_num == my_car.team:
			my_goal = goal
		else:
			opponent_goal = goal
	
	v = Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)
	
	closest_car_dist = 10000
	
	closest_car_pos = Make_Vect(opponent_goal.location)
	
	ball_pos = Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 10, self.arieal_acceleration, False))
	
	for i in range(packet.num_cars):
		car = packet.game_cars[i]
		d = (Make_Vect(car.physics.location) - Make_Vect(my_car.physics.location)).len()
		if car.team != my_car.team and not car.is_demolished and d < closest_car_dist:
			closest_car_dist = d
			closest_car_pos = Make_Vect(car.physics.location)
	
	state = ""
	
	# Kickoff
	if packet.game_info.is_kickoff_pause:
		state = "Kickoff"
		if (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() > 700:
			Collect_Boost(self, packet, Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.direction) * 200, True, False)
			self.controller_state.boost = True
		elif closest_car_dist > 3500:
			self.controller_state.throttle = -1
			self.controller_state.steer = 0.0
			self.controller_state.boost = False
			self.controller_state.handbrake = False
			self.controller_state.jump = False
		else:
			car_direction = get_car_facing_vector(my_car)
			ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.25).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
			
			Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
			self.controller_state.boost = False
	# Inside goal (Prevents car from entering net too far)
	elif abs(my_car.physics.location.x) < 900 and (dot(Make_Vect(my_car.physics.location) + Make_Vect(my_car.physics.velocity) * 0.25 - Make_Vect(my_goal.location), my_goal.direction) < 0.0 and ((Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000.0 or Make_Vect(my_car.physics.velocity).flatten().len() > 1000)):
		state = "Inside Goal"
		# Driving away from ball
		car_face = get_car_facing_vector(my_car)
		if dot_2D(car_face.normal(), (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location))) < 0.0 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000.0: # (Make_Vect(my_goal.location) - Make_Vect(ball_pos.location)).normal()) > 0.5:
			self.controller_state.throttle = -1.0
			self.controller_state.steer = 0.0
			self.controller_state.boost = False
			self.controller_state.handbrake = False
			self.controller_state.jump = False
			if Make_Vect(my_car.physics.velocity).len() < 1500.0 or dot_2D(car_face, my_car.physics.velocity) < 0.0:
				Enter_Half_Flip(self, packet)
			
			self.is_arieal = False
			self.jump_timer = 0.0
		#Driving towards ball
		else:
			# Hit_Ball_To(self, packet, Make_Vect(my_goal.location))
			p1 = Make_Vect(my_goal.location) + Vec3(800, my_goal.direction.y * 100, 0.0)
			p2 = Make_Vect(my_goal.location) - Vec3(800, -my_goal.direction.y * 100, 0.0)
			b = Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)
			
			a1 = car_face.correction_to(p1)
			a2 = car_face.correction_to(p2)
			
			b_a = car_face.correction_to(b)
			
			car_pos = Make_Vect(my_car.physics.location)
			
			# Same sign:
			# angle1 * angle2 > 0.0
			
			# Opposite sign:
			# angle1 * angle2 < 0.0
			
			if (a1 - b_a) * (a1 - a2) > 0.0 and (p1 - car_pos).len() > 200:
				Drive_To(self, packet, p1)
				self.is_arieal = False
				self.jump_timer = 0.0
				self.controller_state.boost = False
			elif (a2 - b_a) * (a1 - a2) < 0.0 and (p2 - car_pos).len() > 200:
				Drive_To(self, packet, p2)
				self.is_arieal = False
				self.jump_timer = 0.0
				self.controller_state.boost = False
			else:
				Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
			
		self.renderer.draw_line_3d(my_car.physics.location, p1.UI_Vec3(), self.renderer.red())
		self.renderer.draw_line_3d(my_car.physics.location, p2.UI_Vec3(), self.renderer.red())
		
		if my_car.physics.location.z > 200.0 and my_car.has_wheel_contact:
			self.controller_state.jump = True
	# Shadow
	elif (Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).flatten().len() > 600 and (closest_car_pos - Make_Vect(ball_pos.location)).flatten().len() < 500 and (closest_car_pos - Make_Vect(ball_pos.location)).z > -300:
		state = "Shadow"
		if (Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).flatten().len() < 300:
			Attack_Ball(self, packet, Make_Vect(opponent_goal.location), ball_pos)
		else:
			v = Make_Vect(my_goal.location) + (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).flatten() * 0.8
			Collect_Boost(self, packet, v, True, False)
	# Defense
	elif (((closest_car_pos - Make_Vect(ball_pos.location)).flatten().len() < 1500 and dot_2D((Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)), my_goal.direction) < 0.0) or (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() < 4000.0) and (Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).len() > 200: # and closest_car_dist > 1000):
		state = "Defending"
		# Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
		if (Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).len() < 500.0 or (dot(Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location), Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * 300) > 0.0 or abs(packet.game_ball.physics.location.x) > constrain(abs(my_car.physics.location.x), 2000, 1000)):
			Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
		else:
			self.is_arieal = False
			self.jump_timer = 0.0
			if (Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).len() > 1000:
				Collect_Boost(self, packet, Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * 100, True) #Vec3(-sign(car_to_pos.x) * 150, 0, 0))
			else:
				Drive_To(self, packet, Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * 100, False)
	# Offense
	else:
		state = "Offense"
		if packet.game_ball.physics.location.z > 1500 or (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() > 2000:
			Collect_Boost(self, packet, Make_Vect(ball_pos.location), (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() > 1500)
			self.is_arieal = False
			self.jump_timer = 0.0
		elif Vector2(v.x, v.y).len() < 100.0 and packet.game_ball.physics.location.z > 120:
			Dribble(self, packet, Make_Vect(opponent_goal.location))
			self.is_arieal = False
			self.jump_timer = 0.0
		# If opponet isn't defending
		elif dot((closest_car_pos - Make_Vect(my_car.physics.location)).normal(), opponent_goal.direction) > -0.5:
			prediction = self.get_ball_prediction_struct()
			ball_pos = Make_Vect(Get_Ball_At_T(prediction, Approximate_Time_To_Ball(prediction, self.index, packet, 10, self.arieal_acceleration, True)).location)
			if ball_pos.z < 200:
				Attack_Aim_Ball(self, packet, Make_Vect(opponent_goal.location), ball_pos)
				self.is_arieal = False
				self.jump_timer = 0.0
			else:
				Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
				# Dribble(self, packet, Make_Vect(opponent_goal.location))
		elif closest_car_dist < 2000 or  packet.game_ball.physics.location.z > 500:
			Dribble(self, packet, Make_Vect(opponent_goal.location))
			self.is_arieal = False
			self.jump_timer = 0.0
		else:
			# Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
			Get_Ball_On_Car(self, packet, Make_Vect(my_goal.direction))
			self.is_arieal = False
			self.jump_timer = 0.0
	
	self.renderer.draw_string_3d(my_car.physics.location, 2, 2, state, self.renderer.white())
	
	if my_car.is_super_sonic:
		self.controller_state.boost = False
	

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
		
		self.renderer.begin_rendering()
		
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
				# Get_Ball_On_Car(self, packet, Vec3(0, 1, 0))
				Strategy_Ones(self, packet)
			else:
				self.manuever_lock = self.manuever_lock - self.delta
				self.manuever(self, packet, self.manuever_lock)
		
		self.renderer.end_rendering()
		
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

