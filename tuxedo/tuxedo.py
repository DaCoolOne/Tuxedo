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
		l = max(0.0001, self.len())
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
	if time > 0.4 or packet.game_cars[self.index].double_jumped:
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

def Manuever_Double_Jump(self, packet, time):
	if time > 0.2 or packet.game_cars[self.index].double_jumped:
		self.controller_state.jump = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0
		self.controller_state.roll = 0.0
		self.controller_state.boost = False
		self.controller_state.throttle = 1.0
	else:
		self.controller_state.jump = True
		self.controller_state.boost = False
		self.controller_state.yaw = 0
		self.controller_state.pitch = 0
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0

# Maneuver setup
def Enter_Flip(self, packet, dir):
	self.manuever_lock = 0.5
	self.manuever = Manuever_Flip
	self.flip_dir = dir
	self.controller_state.jump = True
	self.controller_state.steer = 0.0

# Used to keep the nose from scraping the ground on front flips
def Enter_High_Flip(self, packet, dir):
	self.manuever_lock = 0.6
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

def Enter_Double_Jump(self, packet):
	self.manuever_lock = 0.3
	self.manuever = Manuever_Double_Jump
	self.controller_state.jump = True
	self.controller_state.pitch = 0.0
	self.controller_state.boost = False

def Drive_To(self, packet: GameTickPacket, position, boost = False):
	
	position = Vector2(constrain(position.x, -4000, 4000), constrain(position.y, -5100, 5100))
	
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
			
			self.controller_state.boost = (abs(steer_correction_radians) < 0.5 and car_to_pos.len() > 500.0) and boost
			
			self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.6 and Make_Vect(my_car.physics.velocity).flatten().len() < 1000 and Make_Vect(my_car.physics.velocity).flatten().len() > 400 and my_car.physics.location.z < 50.0
			
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
		
		car_to_pos = Vector2(position.x, position.y) - car_location
		
		car_direction = get_car_facing_vector(my_car)
		
		steer_correction_radians = car_direction.correction_to(car_to_pos)
		
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
	
	end_target = Vector2(position.x, position.y)
	
	if car.boost > 75 or (Vector2(car.physics.location.x, car.physics.location.y) - end_target).len() < 2000.0:
		Drive_To(self, packet, position, do_boost)
	else:
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
	
	car_pos = Make_Vect(car.physics.location)
	car_vel = Make_Vect(car.physics.velocity)
	
	arieal_speed = max(1000, car_vel.len()) + acceleration
	
	time_to_reach_ball = 0
	refine = 0
	
	car_to_ball = Make_Vect(prediction.slices[0].physics.location) - car_pos
	
	while refine < resolution:
		car_to_ball = Make_Vect(prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 5)].physics.location) - car_pos
		
		time_to_reach_ball = car_to_ball.len() / arieal_speed
		
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
	
	ball_to_position = Make_Vect(my_car.physics.location) - position
	
	car_direction = get_car_facing_vector(my_car)
	
	ball_pos = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.17
	
	ball_predict = Get_Ball_At_T(prediction, Approximate_Time_To_Ball(prediction, self.index, packet, 20, self.arieal_acceleration, True))
	
	if Vector2(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y).len() > 700 or ball_pos.z - my_car.physics.location.z > 300:
		ball_pos = Make_Vect(ball_predict.location)
	
	angle = car_direction.correction_to((Make_Vect(my_car.physics.location) - position).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
	
	dir = Vec3(car_direction.x * math.cos(-angle) - car_direction.y * math.sin(-angle) * 3, car_direction.y * math.cos(-angle) + car_direction.x * math.sin(-angle) * 3, 0.0)
	
	dir_2D = Vector2(dir.x * 0.2, dir.y).normal()
	
	multiplier = 1.0
	
	if Vector2(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y).len() > 150.0 and abs(ball_predict.velocity.z) < 10.0 and ball_predict.location.z < 170.0:
		multiplier = 1.5
	
	position = ball_pos + dir * (20 * (constrain((200 - packet.game_ball.physics.velocity.z) * (1 / 200), -1, 1))) * multiplier
	
	position = Vector2(position.x, position.y) # + car_direction * (10.0 * sq(dot_2D(car_direction, dir)) / (1 + Make_Vect(packet.game_ball.physics.velocity).len() * 0.06))
	
	car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
	
	car_vel = Make_Vect(my_car.physics.velocity) * 0.15
	
	car_to_pos = Vector2(position.x, position.y) - car_location - car_vel
	
	if my_car.has_wheel_contact:
		
		if car_to_pos.len() > 1500:
			Collect_Boost(self, packet, Make_Vect(ball_predict.location), True, True)
		else:
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
				self.controller_state.boost = l < 40 and l > 25 and ball_pos.z > 110 and ball_pos.z < 140 and abs(steer_correction_radians) < math.pi * 0.3
			
			self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.5 and Make_Vect(my_car.physics.velocity).flatten().len() > 400 and car_to_pos.len() > 500
			
			self.controller_state.jump = False
			self.controller_state.pitch = 0.0
			
			# self.renderer.begin_rendering()
			
			if ball_pos.z < 200.0 and car_to_pos.len() < 350:
				count = 0
				while count < packet.num_cars:
					car = packet.game_cars[count]
					l = Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity) * 0.3 - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.3
					self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity)).UI_Vec3(), self.renderer.white())
					if (l.len() < 400.0 and self.index != count and not car.is_demolished and car.physics.location.z + car.physics.velocity.z * 0.4 < 300.0):
						if Vector2(car_to_pos.x, car_to_pos.y).len() < 100 and ball_pos.z < 130:
							Enter_Flip(self, packet, Vector2(-dir_2D.x, -dir_2D.y))
						else:
							ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
							
							Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
						self.controller_state.boost = True
						break
					count += 1
			
			# self.renderer.draw_line_3d(position.toVector3(), UI_Vec3(my_car.physics.location.x, my_car.physics.location.y, 100.0), self.renderer.green())
			
			# self.renderer.end_rendering()
		
	else:
		car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
		
		steer_correction_radians = car_direction.correction_to(car_to_pos)
		
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
	
	ball_pos = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.1
	
	ball_pos_2 = Make_Vect(packet.game_ball.physics.location)
	
	ball_predict = Get_Ball_At_T(prediction, 0.1)
	
	if Vector2(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y).len() > 1000:
		ball_pos = Make_Vect(ball_predict.location)
	
	car_location = Vector2(my_car.physics.location.x, my_car.physics.location.y)
	
	car_to_real_pos = Vector2(ball_pos_2.x, ball_pos_2.y) - car_location
	
	if my_car.has_wheel_contact:
		
		v = Make_Vect(ball_predict.velocity).flatten()
		n = perp(v).normal() * 150
		if dot_2D(direction, n) > 0.0:
			n = n * -1
		b_p = Vector2(ball_pos.x, ball_pos.y)
		
		car_to_pos = Vector2((b_p + n).x, (b_p + n).y) - car_location
		
		ang = car_direction.correction_to(car_to_real_pos)
		
		if car_to_real_pos.len() < 170.0 and (abs(ang) > math.pi * 0.4 or self.getting_ball_on_car):
			
			self.getting_ball_on_car = True
			
			self.controller_state.throttle = 1 # constrain(abs(ang) * 6, -1, 1)
			self.controller_state.boost = False
			self.controller_state.steer = constrain(-ang * 6, -1, 1)
			
			self.controller_state.handbrake = car_to_real_pos.len() > 240.0 and abs(ang) > math.pi * 0.4
			
		elif car_to_real_pos.len() < 500.0:
			self.getting_ball_on_car = False
			if v.len() > 900:
				Drive_To(self, packet, b_p + n + car_direction * 75, False)
			elif abs(ang) > math.pi * 0.3 and abs(ang) < math.pi * 0.7:
				Drive_To(self, packet, b_p + car_direction * 200)
			else:
				Drive_To(self, packet, b_p, False)
			# self.controller_state.throttle = 1
			
			# self.renderer.draw_string_3d(my_car.physics.location, 3, 2, "Mid-stage", self.renderer.red())
			
		else:
			self.getting_ball_on_car = False
			Collect_Boost(self, packet, b_p + n - Make_Vect(my_car.physics.velocity) * 0.25, (car_to_real_pos - Make_Vect(my_car.physics.velocity)).len() > 1000.0)
		
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
		
		# if ball_pos.z < 180.0 and car_to_pos.len() < 300:
			# count = 0
			# while count < packet.num_cars:
				# car = packet.game_cars[count]
				# l = (Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity) * 0.2 - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.2).len()
				# if l < 700.0 and self.index != count and not car.is_demolished:
					# if car_to_pos.len() < 100:
						# Enter_Flip(self, packet, Vector2(-dir_2D.x, -dir_2D.y))
					# else:
						# ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
						
						# Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
					# self.controller_state.boost = True
				# count += 1
		
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
	
	aim = (ball_predict - aim_pos).flatten().normal(-1)
	car_to_pos = ball_predict - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.3
	
	car_to_ball_real = Make_Vect(packet.game_ball.physics.location) - Make_Vect(car.physics.location)
	
	if car_to_ball_real.flatten().len() < 330 and car_to_pos.z < 230 and Make_Vect(car.physics.velocity).len() > 300:
		car_direction = get_car_facing_vector(car)
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.1 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.1
		ang = car_direction.correction_to(c_p)
		Enter_Flip(self, packet, Vector2(-math.sin(ang), math.cos(ang)))
	elif car_to_ball_real.flatten().len() > 1500:
		Collect_Boost(self, packet, ball_predict - aim * (car_to_pos.len() * 0.04 + 100), True, True)
	else:
		Drive_To(self, packet, ball_predict - aim * (car_to_pos.len() * 0.04 + 100), True)
	
	self.renderer.draw_line_3d(car.physics.location, ball_predict.UI_Vec3(), self.renderer.white())
	self.renderer.draw_line_3d((ball_predict - aim * (car_to_pos.len() * 0.02 + 100)).UI_Vec3(), ball_predict.UI_Vec3(), self.renderer.red())
	

def Attack_Aim_Ball(self, packet: GameTickPacket, aim_pos: Vec3, ball_predict: Vec3):
	car = packet.game_cars[self.index]
	
	car_direction = get_car_facing_vector(car)
	
	a = (ball_predict - aim_pos).flatten().normal(-125)
	aim = (Vector2(a.x, a.y) + car_direction * 150).toVector3()
	aim_2 = (Vector2(a.x, a.y)).toVector3()
	car_to_pos = ball_predict - Make_Vect(car.physics.location)
	
	car_to_ball_real = Make_Vect(packet.game_ball.physics.location) - Make_Vect(car.physics.location)
	
	if car_to_ball_real.len() < 350 and abs(car_to_ball_real.z) < 200:
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.15 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.15
		ang = car_direction.correction_to(c_p)
		Enter_Flip(self, packet, Vector2(-math.sin(ang) * 2, math.cos(ang)).normal())
	else:
		Collect_Boost(self, packet, (ball_predict - aim_2 * (car_to_ball_real.len() * 0.003 + 0.1)), True)
	
	if abs(car_to_pos.z) > 300:
		self.controller_state.boost = False
	
	self.renderer.draw_line_3d(ball_predict.UI_Vec3(), (ball_predict - aim_2 * (car_to_pos.len() * 0.003 + 0.1)).UI_Vec3(), self.renderer.red())
	
	self.renderer.draw_line_3d(packet.game_ball.physics.location, aim_pos.UI_Vec3(), self.renderer.white())
	

def Hit_Ball_To(self, packet: GameTickPacket, aim_pos: Vec3, fallback: Vec3):
	exit_condition = True
	
	info = self.get_field_info()
	
	own_goal = 0
	for i in range(info.num_goals):
		goal = info.goals[i]
		if goal.team_num == packet.game_cars[self.index].team:
			own_goal = goal
			break
	
	target_take_off_angle = math.pi * 0.25
	
	min_take_off_angle = math.pi * 0.1
	
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
			
			self.arieal_speed = max(1500, car_vel.len() + self.arieal_acceleration)
			
			time_to_reach_ball = 0
			refine = 0
			
			car_to_ball = 1
			ball_offset = 1
			
			# Calculate impluse vector
			
			while refine < 20.0:
				ball = prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)]
				
				# Calculate an offset vector to use when steering the car
				# vel = Make_Vect(ball.physics.velocity)
				ball_to_target = Make_Vect(ball.physics.location) - aim_pos
				ball_offset = ball_to_target.normal(50)
				
				car_to_ball = Make_Vect(ball.physics.location) - car_pos + ball_offset
				
				target_vel_xy2 = Vector2(car_to_ball.x, car_to_ball.y)
				
				time_to_reach_ball = car_to_ball.len() / self.arieal_speed
				
				target_vel_xy = target_vel_xy2 * (self.arieal_speed / target_vel_xy2.len())
				
				z_vel = (car_to_ball.z - 0.5 * packet.game_info.world_gravity_z * time_to_reach_ball * time_to_reach_ball) / time_to_reach_ball
				
				# velocity we want to be going
				target_vel = Vec3(target_vel_xy.x, target_vel_xy.y, z_vel)
				
				refine = refine + 1
			
			self.renderer.draw_line_3d(car.physics.location, (car_pos + car_to_ball).UI_Vec3(), self.renderer.yellow())
			
			impulse = target_vel - car_vel
			
			impulse = impulse + Vec3(0, 0, impulse.z * 0.1)
			
			rot = impulse.to_rotation()
			
			car_rot = car.physics.rotation
			
			car_rot_vel = Make_Vect(car.physics.angular_velocity) #.align_to(car_rot)
			
			# car_rot_vel.x = -car_rot_vel.x
			
			car_rot_vel.y = -car_rot_vel.y
			car_rot_vel.z = -car_rot_vel.z
			
			car_to_ball_2D = Vector2(ball_pos.x, ball_pos.y) - Vector2(car_pos.x, car_pos.y)
			
			quat_vel = Quat.From_Angle_Axis(car_rot_vel, car_rot_vel.len()).normal()
			quat_rot = Quat.From_Euler(car_rot.yaw, car_rot.pitch, car_rot.roll).normal()
			
			# Translate velocity to local units
			local_quat_vel = (quat_rot * quat_vel).normal()
			
			local_euler = local_quat_vel.To_Euler()
			
			# Improving this
			rot_ang_const = 0.25
			self.controller_state.yaw = correct(rot.x, car_rot.yaw + local_euler.x * rot_ang_const, 4.0)
			if car.double_jumped:
				self.controller_state.pitch = correct(rot.y, car_rot.pitch + local_euler.y * rot_ang_const, 4.0)
			else:
				self.controller_state.pitch = correct(rot.y, car_rot.pitch - math.pi * 0.1, 4.0)
				# self.controller_state.pitch = 0.8
			self.controller_state.roll = correct(0.0, car_rot.roll + local_euler.z * rot_ang_const, 4.0)
			
			self.controller_state.steer = correct(rot.x, car_rot.yaw, 1.0)
			
			a = angle_between(impulse, Rotation_Vector(car_rot))
			pitch_a = constrain_pi(car_rot.pitch - rot.y)
			self.controller_state.boost = impulse.len() > a * 200.0 and pitch_a > -0.5 and a < math.pi * 0.5
			
			self.controller_state.throttle = 1.0
			
			self.controller_state.jump = self.jump_timer > 0.2 and not car.double_jumped and impulse.z > 300 # impulse.z > 250.0
			
			if self.controller_state.jump:
				self.controller_state.yaw = 0.0
				self.controller_state.pitch = 0.0
				self.controller_state.roll = 0.0
			
			# Flip into the ball
			if not car.double_jumped and car_to_ball.len() < 300.0 and abs(car_to_ball.z - 30) < 60.0 and car_pos.z > 50.0:
				self.controller_state.jump = True
				yaw = self.controller_state.yaw
				self.controller_state.yaw = math.sin(yaw) * 0.3
				self.controller_state.pitch = -math.cos(yaw)
				self.controller_state.roll = 0.0
				self.flip_timer = 0.9
			
			if dot(car.physics.velocity, car_to_ball) < 0.0:
				self.is_arieal = False
				self.jump_timer = 0.0
			
			self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + impulse).UI_Vec3(), self.renderer.green())
			
	else:
		self.is_arieal = False
		self.jump_timer = 0.0
		
		if packet.game_cars[self.index].has_wheel_contact:
			
			prediction = self.get_ball_prediction_struct()
			
			car = packet.game_cars[self.index]
			
			car_pos = Make_Vect(car.physics.location)
			car_vel = Make_Vect(car.physics.velocity)
			
			self.arieal_speed = max(1500, car_vel.len() + self.arieal_acceleration)
			
			time_to_reach_ball = 0
			refine = 0
			
			car_to_ball = Make_Vect(prediction.slices[0].physics.location) - car_pos
			ball_location = Vec3()
			
			# Calculate impluse vector
			
			while refine < 10.0:
				
				ball_location = Make_Vect(prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 1)].physics.location)
				
				car_to_ball = ball_location - car_pos
				
				time_to_reach_ball = car_to_ball.len() / self.arieal_speed
				
				refine = refine + 1
			
			self.renderer.draw_line_3d(car.physics.location, ball_location.UI_Vec3(), self.renderer.yellow())
			
			self.renderer.draw_line_3d(ball_location.UI_Vec3(), aim_pos.UI_Vec3(), self.renderer.white())
			
			if ball_location.z < 250 or car.boost + 15 < time_to_reach_ball * 40:
				Attack_Ball(self, packet, aim_pos, ball_location)
			elif ball_location.z > 1000 or car_to_ball.len() > 3000:
				Collect_Boost(self, packet, fallback, True, True)
			else:
				ball_location = Vector2(ball_location.x, ball_location.y)
				car_location = Vector2(car_pos.x, car_pos.y)
				car_direction = get_car_facing_vector(car)
				car_to_target = ball_location - car_location
				steer_correction_radians = car_direction.correction_to(car_to_target)
				
				car_to_ball_2D = ball_location - car_location
				car_to_ball_plus_vel = car_to_ball_2D - Vector2(car_vel.x, car_vel.y)
				car_to_ball_plus_vel_2 = car_to_ball_2D - Vector2(car_vel.x, car_vel.y) - car_direction * 1000
				
				take_off_angle = math.atan2(car_to_ball_plus_vel_2.len(), car_to_ball.z)
				
				# if take_off_angle > math.pi * 0.2:
					# Collect_Boost(self, packet, fallback, True, False)
				# else:
				# Drive_To(self, packet, ball_location, False)
				self.controller_state.throttle = min(1, max(-1, (take_off_angle - target_take_off_angle) * 5))
				
				self.controller_state.handbrake = abs(steer_correction_radians) > math.pi * 0.4 and car_vel.len() > 400
				self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 6.0))
				self.controller_state.boost = (take_off_angle > target_take_off_angle + 0.5) and abs(steer_correction_radians) < 0.3
				
				if dot_2D(car_direction, car_vel) < 0.0:
					self.controller_state.steer = -self.controller_state.steer
				
				if abs(steer_correction_radians) < 0.1:
					self.line_up_time += self.delta
				else:
					self.line_up_time = 0
				
				if self.controller_state.throttle > 0.0 and self.line_up_time > 0.2 and take_off_angle - min_take_off_angle > 0.0 and time_to_reach_ball * 40 < car.boost + 15: # and car_to_ball.z < 1500:
					self.controller_state.jump = True
					self.controller_state.pitch = 1
					self.is_arieal = True
				
				self.renderer.draw_string_3d((car_pos + Vec3(0, 0, 500)).UI_Vec3(), 2, 2, str(take_off_angle - min_take_off_angle), self.renderer.green())
			
		else:
			Drive_To(self, packet, Make_Vect(packet.game_ball.physics.location), False)
			self.line_up_time = 0
		# if abs(steer_correction_radians) > math.pi * 0.8 and car_to_ball.len() > 1000.0:
			# Enter_Half_Flip(self, packet)
		
		# self.renderer.begin_rendering()
		
		# self.renderer.draw_line_3d(car.physics.location, (car_pos + car_to_ball).toVector3(), self.renderer.white())
		# self.renderer.draw_string_3d(car.physics.location, 2, 2, str(take_off_angle / math.pi), self.renderer.white())
		
		# self.renderer.end_rendering()
	
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
	
	closest_car_pos = Vec3(0, 0, 10000)
	
	ball_pos = Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 20, self.arieal_acceleration, False))
	
	for i in range(packet.num_cars):
		car = packet.game_cars[i]
		d = (Make_Vect(car.physics.location) - Make_Vect(packet.game_ball.physics.location)).len()
		if car.team != my_car.team and not car.is_demolished and d < closest_car_dist:
			closest_car_dist = d
			closest_car_pos = Make_Vect(car.physics.location)
	
	state = ""
	sub_state = ""
	
	car_direction = get_car_facing_vector(my_car)
	
	if self.is_arieal and not my_car.has_wheel_contact:
		Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
	# Kickoff
	elif packet.game_info.is_kickoff_pause:
		self.is_arieal = False
		self.line_up_time = 0.0
		state = "Kickoff"
		if (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() > 850:
			Collect_Boost(self, packet, Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.direction) * 200, True, False)
			self.controller_state.boost = True
			sub_state = "Charging Ball"
		elif closest_car_dist > 3500:
			self.controller_state.throttle = -1
			self.controller_state.steer = 0.0
			self.controller_state.boost = False
			self.controller_state.handbrake = False
			self.controller_state.jump = False
			sub_state = "Countering Fake"
		else:
			ang = car_direction.correction_to((Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.2).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
			
			Enter_Flip(self, packet, Vector2(math.sin(-ang), math.cos(-ang)))
			self.controller_state.boost = False
			sub_state = "Flip"
	elif abs(my_car.physics.location.x) < 900 and (dot(Make_Vect(my_car.physics.location) + Make_Vect(my_car.physics.velocity) * 0.25 - Make_Vect(my_goal.location), my_goal.direction) < 0.0 and ((Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000.0 or Make_Vect(my_car.physics.velocity).flatten().len() > 1000)):
		state = "Inside Goal"
		# Driving away from ball
		car_face = get_car_facing_vector(my_car)
		if dot_2D(car_face.normal(), my_goal.direction) < 0.0 and dot_2D(car_face.normal(), (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location))) < 0.0 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000.0: # (Make_Vect(my_goal.location) - Make_Vect(ball_pos.location)).normal()) > 0.5:
			self.controller_state.throttle = -1.0
			self.controller_state.steer = 0.0
			self.controller_state.boost = False
			self.controller_state.handbrake = False
			self.controller_state.jump = False
			if Make_Vect(my_car.physics.velocity).len() < 1500.0 or dot_2D(car_face, my_car.physics.velocity) < 0.0:
				Enter_Half_Flip(self, packet)
			
			self.is_arieal = False
			self.jump_timer = 0.0
			
			sub_state = "Half Flip"
		#Driving towards ball
		else:
			
			car_pos = Make_Vect(my_car.physics.location)
			
			# Hit_Ball_To(self, packet, Make_Vect(my_goal.location))
			p1 = Make_Vect(my_goal.location) + Vec3(800, my_goal.direction.y * 100, 0.0)
			p2 = Make_Vect(my_goal.location) - Vec3(800, -my_goal.direction.y * 100, 0.0)
			b = Make_Vect(ball_pos.location) - car_pos
			
			a1 = car_face.correction_to(p1 - car_pos)
			a2 = car_face.correction_to(p2 - car_pos)
			
			b_a = car_face.correction_to(b)
			
			if (a1 - b_a) * (a1 - a2) < 0.0 and (p1 - car_pos).len() > 100:
				Drive_To(self, packet, p1)
				self.is_arieal = False
				self.jump_timer = 0.0
				self.controller_state.boost = False
				sub_state = "Drive Around Post"
			elif (a2 - b_a) * (a1 - a2) > 0.0 and (p2 - car_pos).len() > 100:
				Drive_To(self, packet, p2)
				self.is_arieal = False
				self.jump_timer = 0.0
				self.controller_state.boost = False
				sub_state = "Drive Around Post"
			else:
				Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location), Make_Vect(my_goal.location))
				sub_state = "Hit Ball"
			
			self.renderer.draw_line_3d(my_car.physics.location, p1.UI_Vec3(), self.renderer.red())
			self.renderer.draw_line_3d(my_car.physics.location, p2.UI_Vec3(), self.renderer.red())
		
		if my_car.physics.location.z > 200.0 and my_car.has_wheel_contact:
			self.controller_state.jump = True
			sub_state = "Jump Off Wall"
	elif closest_car_dist < 1000 and ((closest_car_pos - Make_Vect(my_car.physics.location)).len() < 500) and closest_car_pos.z - my_car.physics.location.z > 100 and packet.game_ball.physics.location.z - my_car.physics.location.z > 200:
		state = "Tactical Retreat"
		Drive_To(self, packet, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300, False)
	elif dot((closest_car_pos - Make_Vect(ball_pos.location)).normal(), (Make_Vect(ball_pos.location) - Make_Vect(opponent_goal.location)).normal()) > -0.4 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000:
		state = "Powershot"
		self.is_arieal = False
		self.line_up_time = 0.0
		if dot((Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).normal(), (Make_Vect(ball_pos.location) - Make_Vect(opponent_goal.location)).normal()) > 0.3:
			sub_state = "Shooting"
			Attack_Aim_Ball(self, packet, Make_Vect(opponent_goal.location) - Make_Vect(opponent_goal.direction) * 150, Make_Vect(Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 20, self.arieal_acceleration, False)).location))
			# Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location) - Make_Vect(opponent_goal.direction) * 100, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300)
		else:
			Collect_Boost(self, packet, Make_Vect(ball_pos.location) + (Make_Vect(ball_pos.location) - Make_Vect(opponent_goal.location)).normal(1000), True, True)
			sub_state = "Lining up"
	elif (closest_car_dist < 2000 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() < 4000) or (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() < 2000:
		state = "Defending"
		v = Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)
		v2 = Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.location)
		v3 = Make_Vect(my_car.physics.location) - Make_Vect(my_goal.location)
		if dot(v, my_goal.direction) > 0.0 or dot(Make_Vect(packet.game_ball.physics.velocity).normal(), v2.normal()) < -0.5:
			Hit_Ball_To(self, packet, Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * 1000 + Vec3(sign(v.x) * 5000, 0, 0), Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300)
			sub_state = "Clear Ball"
		else:
			self.is_arieal = False
			self.jump_timer = 0.0
			if (Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).len() > 1000:
				Collect_Boost(self, packet, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300, True) #Vec3(-sign(car_to_pos.x) * 150, 0, 0))
				sub_state = "Collect Boost to Goal"
			else:
				Drive_To(self, packet, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300, False)
				sub_state = "Head to Goal"
			
			self.controller_state.jump = v3.len() < Make_Vect(my_car.physics.velocity).len() * dot(Make_Vect(my_car.physics.velocity).normal(), Make_Vect(v3.normal()))
	else:
		state = "Dribble"
		self.is_arieal = False
		self.line_up_time = 0.0
		if packet.game_ball.physics.location.z < 130 and closest_car_dist > 1500:
			Get_Ball_On_Car(self, packet, Make_Vect(my_goal.direction))
			sub_state = "Get Ball on Car"
		else:
			Dribble(self, packet, Make_Vect(opponent_goal.location))
	
	self.renderer.draw_string_3d(my_car.physics.location, 2, 2, state+"\n "+sub_state, self.renderer.white())
	
	if not packet.game_info.is_round_active:
		self.reset()
	
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
		self.arieal_acceleration = 350.0
		self.delta = 0.0
		self.getting_ball_on_car = False
	
	def initialize_agent(self):
		#This runs once before the bot starts up
		self.controller_state = SimpleControllerState()
		self.prev_time = 0.0
		self.has_endgame_quickchat = True
		self.reset()
	
	def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
		
		self.renderer.begin_rendering()
		
		self.controller_state = SimpleControllerState()
		
		time = packet.game_info.seconds_elapsed
		self.delta = time - self.prev_time
		self.prev_time = time
		
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
			
			# print("Celebrate")
			
		# Match Logic
		else:
			if self.manuever_lock <= 0.0:
				
				# opponent_goal = 0
				# my_goal = 0
				
				# for i in range(self.get_field_info().num_goals):
					# goal = self.get_field_info().goals[i]
					# if goal.team_num == packet.game_cars[self.index].team:
						# my_goal = goal
					# else:
						# opponent_goal = goal
				
				# if packet.game_ball.physics.location.z < 130:
					# Get_Ball_On_Car(self, packet, Make_Vect(my_goal.direction))
					# self.renderer.draw_string_3d(packet.game_cars[self.index].physics.location, 2, 2, "Ball On Car", self.renderer.white())
				# else:
					# Dribble(self, packet, Make_Vect(opponent_goal.location))
					# self.renderer.draw_string_3d(packet.game_cars[self.index].physics.location, 2, 2, "Dribble", self.renderer.white())
				
				# my_car = packet.game_cars[self.index]
				
				# if my_car.has_wheel_contact:
					# self.is_arieal = False
					# self.jump_timer = 0.0
					
				
				# Hit_Ball_To(self, packet, Make_Vect(self.get_field_info().goals[0].location), Make_Vect(self.get_field_info().goals[1].location))
				
				Strategy_Ones(self, packet)
				
				# Attack_Aim_Ball(self, packet, Make_Vect(self.get_field_info().goals[0].location), Make_Vect(Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 20, self.arieal_acceleration, False)).location))
				
				# Attack_Ball(self, packet, Make_Vect(self.get_field_info().goals[0].location), Make_Vect(Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 20, self.arieal_acceleration, False)).location))
			else:
				self.renderer.draw_string_3d(packet.game_cars[self.index].physics.location, 2, 2, "Manuever Lock", self.renderer.white())
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

