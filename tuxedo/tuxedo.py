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
	
	def align_to(self, rot):
		v = Vec3(self.x, self.y, self.z)
		
		# Apply roll
		v.set(v.x, math.cos(rot.roll) * v.y + math.sin(rot.roll) * v.z, math.cos(rot.roll) * v.z - math.sin(rot.roll) * v.y)
		
		# Apply pitch
		v.set(math.cos(-rot.pitch) * v.x + math.sin(-rot.pitch) * v.z, v.y, math.cos(-rot.pitch) * v.z - math.sin(-rot.pitch) * v.x)
		
		# Apply yaw
		v.set(math.cos(-rot.yaw) * v.x + math.sin(-rot.yaw) * v.y, math.cos(-rot.yaw) * v.y - math.sin(-rot.yaw) * v.x, v.z)
		
		return v
	
	# Inverse of align_to
	def align_from(self, rot):
		v = Vec3(self.x, self.y, self.z)
		
		# Apply yaw
		v.set(math.cos(rot.yaw) * v.x + math.sin(rot.yaw) * v.y, math.cos(rot.yaw) * v.y - math.sin(rot.yaw) * v.x, v.z)
		
		# Apply pitch
		v.set(math.cos(rot.pitch) * v.x + math.sin(rot.pitch) * v.z, v.y, math.cos(rot.pitch) * v.z - math.sin(rot.pitch) * v.x)
		
		# Apply roll
		v.set(v.x, math.cos(-rot.roll) * v.y + math.sin(-rot.roll) * v.z, math.cos(-rot.roll) * v.z - math.sin(-rot.roll) * v.y)
		
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
	

# Pretty sure my quaternion class is borked. Please do not use.
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
	

def render_star(self, position: Vec3, color, size = 100):
	
	v = Vec3(1, 1, 1).normal(size)
	
	self.renderer.draw_line_3d((position - Vec3(size, 0, 0)).UI_Vec3(), (position + Vec3(size, 0, 0)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(0, size, 0)).UI_Vec3(), (position + Vec3(0, size, 0)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(0, 0, size)).UI_Vec3(), (position + Vec3(0, 0, size)).UI_Vec3(), color)
	
	self.renderer.draw_line_3d((position - Vec3(-v.x, v.y, v.z)).UI_Vec3(), (position + Vec3(-v.x, v.y, v.z)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(v.x, -v.y, v.z)).UI_Vec3(), (position + Vec3(v.x, -v.y, v.z)).UI_Vec3(), color)
	self.renderer.draw_line_3d((position - Vec3(v.x, v.y, -v.z)).UI_Vec3(), (position + Vec3(v.x, v.y, -v.z)).UI_Vec3(), color)
	

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
	return math.acos(dot(a.normal(), b.normal()))

def Make_Vect(v):
	return Vec3(v.x, v.y, v.z)

def perp(v):
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

def correction(car, ideal):
	v = Make_Vect(ideal).align_from(car.physics.rotation)
	return constrain_pi(math.atan2(-v.y, v.x))

# Manuevers (Anything that requires very specific input and cannot be interupted)

# Half Flip = back flip + forward pitch as hard as possible then air roll
def Manuever_Half_Flip(self, packet, time):
	if time > 0.6:
		self.controller_state.jump = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0
		self.controller_state.roll = 0.0
		self.controller_state.boost = False
		self.controller_state.throttle = 1.0
	elif not packet.game_cars[self.index].double_jumped:
		self.controller_state.jump = True
		self.controller_state.boost = False
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 1.0
		self.controller_state.roll = 0.0
		self.controller_state.throttle = 1.0
	else:
		Align_Car_To(self, packet, self.flip_dir)

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
	self.manuever_lock = 0.7
	self.manuever = Manuever_Half_Flip
	self.flip_dir = get_car_facing_vector(packet.game_cars[self.index]).flatten() * -1
	self.controller_state.jump = True
	self.controller_state.pitch = 1.0
	self.controller_state.boost = False

def Enter_Double_Jump(self, packet):
	self.manuever_lock = 0.3
	self.manuever = Manuever_Double_Jump
	self.controller_state.jump = True
	self.controller_state.pitch = 0.0
	self.controller_state.boost = False

boost_accel = 991.666

def Drive_To(self, packet: GameTickPacket, position, boost = False, no_overshoot = False):
	
	position = Vec3(constrain(position.x, -4100, 4100), constrain(position.y, -5100, 5100), position.z)
	
	if abs(position.x) < 3500 and abs(position.y) < 4500:
		position.z = 0
	
	render_star(self, position, self.renderer.green())
	
	my_car = packet.game_cars[self.index]
	
	if my_car.has_wheel_contact:
		car_location = Make_Vect(my_car.physics.location)
		
		car_to_pos = position - car_location
		
		car_to_pos_vel = car_to_pos - Make_Vect(my_car.physics.velocity) * 0.1
		car_to_pos_vel_2 = car_to_pos - Make_Vect(my_car.physics.velocity) * 0.4
		
		if no_overshoot:
			car_to_pos_vel_2 = car_to_pos - Make_Vect(my_car.physics.velocity) * 2
		
		steer_correction_radians = correction(my_car, car_to_pos)
		
		self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 3.0))
		
		if car_to_pos.len() > 1000:
			self.controller_state.throttle = 1.0
			
			self.controller_state.boost = (abs(steer_correction_radians) < 0.5 and car_to_pos_vel_2.len() > 500.0) and boost
			
			self.controller_state.handbrake = False
			
		else:
			self.controller_state.boost = False
			self.controller_state.handbrake = (abs(steer_correction_radians) < math.pi * 0.7) and (abs(steer_correction_radians) > math.pi * 0.3) and my_car.physics.location.z < 100.0
			self.controller_state.throttle = constrain(car_to_pos_vel_2.len() / 100, 0, 1)
			if dot_2D(car_to_pos_vel.flatten().normal(), get_car_facing_vector(my_car).flatten().normal()) < -0.7 and my_car.physics.location.z < 50.0:
				self.controller_state.throttle = -self.controller_state.throttle
				# May need to update
				self.controller_state.steer = constrain(constrain_pi(math.pi - steer_correction_radians), -1, 1)
		
		if self.controller_state.handbrake:
			self.controller_state.boost = False
		
		self.controller_state.jump = False
		self.controller_state.pitch = 0.0
	else:
		
		car_to_pos = Make_Vect(position) - Make_Vect(my_car.physics.location)
		vel = Make_Vect(my_car.physics.velocity)
		
		if my_car.physics.location.z > -my_car.physics.velocity.z * 1.2 and my_car.physics.location.z > 200:
			Align_Car_To(self, packet, vel.flatten().normal() - Vec3(0, 0, 2))
			self.controller_state.throttle = 1.0
			self.controller_state.boost = 0.3 > angle_between(Vec3(1, 0, 0).align_to(my_car.physics.rotation), Make_Vect(my_car.physics.velocity).flatten().normal() - Vec3(0, 0, 2))
		else:
			Align_Car_To(self, packet, vel.flatten(), Vec3(0, 0, 1))
			self.controller_state.throttle = 1.0
			self.controller_state.boost = False
		
		# steer_correction_radians = correction(my_car, car_to_pos)
		
		# car_rot = my_car.physics.rotation
		
		# self.controller_state.throttle = 1.0
		
		# self.controller_state.roll = correct(0.0, car_rot.roll)
		# self.controller_state.yaw = -max(-1.0, min(1.0, steer_correction_radians * 2.0 - my_car.physics.angular_velocity.z * 0.25))
		
		# self.controller_state.pitch = correct(0.0, car_rot.pitch)
		
		# self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
		
		self.controller_state.handbrake = False
		self.controller_state.jump = False
	
	self.renderer.draw_line_3d(UI_Vec3(position.x, position.y, 100.0), my_car.physics.location, self.renderer.green())

# Defensive approach from the back where we just try to turn the ball away
def Hook_Ball(self, packet, away_target):
	my_car = packet.game_cars[self.index]
	
	ball_pos = Make_Vect(packet.game_ball.physics.location)
	
	car_to_ball = ball_pos - Make_Vect(my_car.physics.location)
	ball_to_away = away_target - ball_pos
	
	offset = perp(car_to_ball).normal(170)
	
	if sign(offset.x) != sign(ball_to_away.x):
		offset = offset * 1
	
	Collect_Boost(self, packet, ball_pos + offset, True, True)
	
	self.controller_state.boost = car_to_ball.len() > 500
	
	self.controller_state.throttle = constrain(self.controller_state.throttle + 0.2, -1, 1)

def Collect_Boost(self, packet: GameTickPacket, position, do_boost = False, allow_flips = True, no_overshoot = False):
	
	car = packet.game_cars[self.index]
	
	end_target = Make_Vect(position)
	
	if car.boost > 75 or (Make_Vect(car.physics.location) - end_target).flatten().len() < 2000.0:
		Drive_To(self, packet, position, do_boost, no_overshoot)
	else:
		target = Make_Vect(position)
		
		info = self.get_field_info()
		
		# Acts as maximum search radius
		closest_boost = 1500
		closest_boost_real = -1
		# Units per second that we're travelling
		if do_boost:
			speed = 2000.0
		else:
			speed = 1000.0
		
		max_dot = 0.75
		max_boost_dot = 0.4
		
		car_pos = Make_Vect(car.physics.location)
		car_dir = get_car_facing_vector(car)
		
		# self.renderer.begin_rendering()
		
		# Pick boost to drive to
		for i in range(info.num_boosts):
			boost = info.boost_pads[i]
			boost_info = packet.game_boosts[i]
			car_to_boost = Make_Vect(boost.location) - car_pos
			time_to_boost = car_to_boost.len()
			d = max_dot
			
			if boost.is_full_boost: # We are allowed to turn more to get full boosts
				d = max_boost_dot
			
			car_to_boost_n = car_to_boost.normal()
			pos_to_boost_n = (Make_Vect(boost.location) - Vec3(target.x, target.y, 0.0)).normal()
			
			if -dot_2D(pos_to_boost_n, car_to_boost_n) > d and dot(get_car_facing_vector(car).normal(), car_to_boost_n) > d and boost_info.is_active: #time_to_boost > boost_info.timer:
				self.renderer.draw_line_3d(boost.location, car.physics.location, self.renderer.yellow())
				if closest_boost > time_to_boost:
					closest_boost = time_to_boost
					closest_boost_real = boost
		
		# if closest_boost_real != -1:
			# self.renderer.draw_string_3d(target.UI_Vec3(), 2, 2, str(closest_boost_real.timer), self.renderer.white())
		# self.renderer.end_rendering()
		
		p = target
		
		if closest_boost_real != -1:
			p = Make_Vect(closest_boost_real.location)
		
		Drive_To(self, packet, p, do_boost, no_overshoot)
		self.controller_state.boost = self.controller_state.boost and do_boost
		
		car_direction = get_car_facing_vector(car)
		
		steer_correction_radians = correction(car, p - car_pos)
		
		if allow_flips and abs(steer_correction_radians) < 0.1 and Make_Vect(car.physics.velocity).len() > 700 and Make_Vect(car.physics.velocity).len() < 1500 and (target - car_pos).len() > Make_Vect(car.physics.velocity).len() + 1000:
			Enter_High_Flip(self, packet, Vec3(-math.sin(steer_correction_radians), math.cos(steer_correction_radians), 0))
	

def Approximate_Time_To_Ball(prediction, car_index, packet, resolution, acceleration = 0, boost = True):
	
	car = packet.game_cars[car_index]
	
	car_pos = Make_Vect(car.physics.location)
	car_vel = Make_Vect(car.physics.velocity)
	
	arieal_speed = max(1000, car_vel.len()) + acceleration
	
	time_to_reach_ball = 0
	refine = 0
	
	slice = 0
	ball_pos = Make_Vect(prediction.slices[0].physics.location)
	car_to_ball = ball_pos - car_pos
	
	while refine < resolution:
		slice = clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 5)
		
		ball_pos = Make_Vect(prediction.slices[slice].physics.location)
		
		car_to_ball = ball_pos - car_pos
		
		time_to_reach_ball = car_to_ball.len() / arieal_speed
		
		refine = refine + 1
	
	# while ball_pos.z > 250 and slice < len(prediction.slices) - 5:
		# slice += 1
		# ball_pos = Make_Vect(prediction.slices[slice].physics.location)
	
	# car_to_ball = ball_pos - car_pos
	
	# time_to_reach_ball = car_to_ball.len() / arieal_speed
	
	if boost:
		return time_to_reach_ball * (2 - car.boost * 0.01)
	else:
		return time_to_reach_ball

def Get_Ball_At_T(prediction, time_to_reach_ball):
	return prediction.slices[clamp(math.ceil(time_to_reach_ball * 60), 0, len(prediction.slices) - 5)].physics

def Dribble(self, packet: GameTickPacket, position: Vec3):
	
	position_2 = position
	position_2.z = 100
	
	prediction = self.get_ball_prediction_struct()
	
	my_car = packet.game_cars[self.index]
	
	ball_to_position = (Make_Vect(my_car.physics.location) - position).flatten()
	
	car_direction = get_car_facing_vector(my_car)
	
	ball_pos = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.17
	
	ball_predict = Get_Ball_At_T(prediction, Approximate_Time_To_Ball(prediction, self.index, packet, 20, self.arieal_acceleration, True))
	
	if Vec3(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y, 0).flatten().len() > 500 or ball_pos.z - my_car.physics.location.z > 400:
		ball_pos = Make_Vect(ball_predict.location)
	
	angle = correction(my_car, (Make_Vect(my_car.physics.location) - position_2).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
	
	dir = Vec3(car_direction.x * math.cos(-angle) - car_direction.y * math.sin(-angle) * 3, car_direction.y * math.cos(-angle) + car_direction.x * math.sin(-angle) * 3, 0.0)
	
	dir_2D = Vec3(dir.x * 0.2, dir.y, 0).normal()
	
	multiplier = 0.5
	
	if Vec3(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y, 0.0).len() < 250.0 and abs(packet.game_ball.physics.velocity.z) < 400.0 and ball_predict.location.z < 250.0:
		multiplier = 1
	
	position = ball_pos + dir * (20 * (constrain((200 - packet.game_ball.physics.velocity.z) * (1 / 200), -1, 1))) * multiplier
	
	position.z *= 0.2
	
	# position = position.flatten() # + car_direction * (10.0 * sq(dot_2D(car_direction, dir)) / (1 + Make_Vect(packet.game_ball.physics.velocity).len() * 0.06))
	
	car_location = Make_Vect(my_car.physics.location)
	
	car_vel = Make_Vect(my_car.physics.velocity) * 0.15
	
	car_to_pos = position - car_location - car_vel
	
	if my_car.has_wheel_contact:
		
		if car_to_pos.len() > 200:
			Collect_Boost(self, packet, Make_Vect(ball_predict.location), True, True, True)
		else:
			steer_correction_radians = correction(my_car, car_to_pos)
			
			self.controller_state.steer = -max(-1.0, min(1.0, steer_correction_radians * 2.0))
			
			if dot_2D(car_to_pos, car_direction) > 0.0 or car_to_pos.len() > 250:
				self.controller_state.throttle = constrain(car_to_pos.len() / 20, -1, 1)
			else:
				self.controller_state.throttle = -constrain(car_to_pos.len() / 20, -1, 1)
				self.controller_state.steer = - self.controller_state.steer
			
			if abs(self.controller_state.throttle) < 0.02:
				self.controller_state.throttle = sign(self.controller_state.throttle) * 0.02
			
			if car_to_pos.len() > 300.0:
				self.controller_state.boost = abs(steer_correction_radians) < 0.2 and (car_to_pos - car_vel * 10).len() > 200 and dot_2D(car_to_pos - car_vel * 3, car_to_pos) > 0.0
			else:
				l = car_to_pos.len()
				self.controller_state.boost = l > 25 and ball_pos.z > 110 and ball_pos.z < 140 and abs(steer_correction_radians) < math.pi * 0.25
			
			self.controller_state.jump = False
			self.controller_state.pitch = 0.0
			
			self.controller_state.handbrake = car_to_pos.len() > 100 and abs(steer_correction_radians) > math.pi * 0.5
			
			# if Make_Vect(my_car.physics.velocity).len() > 1500:
				# self.controller_state.throttle = -1.0
			
			# self.renderer.begin_rendering()
			
			if ball_pos.z < 230.0 and car_to_pos.len() < 400:
				count = 0
				while count < packet.num_cars:
					car = packet.game_cars[count]
					l = Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity) * 0.3 - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.3
					self.renderer.draw_line_3d(car.physics.location, (Make_Vect(car.physics.location) + Make_Vect(car.physics.velocity)).UI_Vec3(), self.renderer.white())
					if (l.len() < 400.0 and self.index != count and not car.is_demolished and car.physics.location.z + car.physics.velocity.z * 0.4 < 300.0):
						if car_to_pos.flatten().len() < 50 and ball_pos.z < 160:
							Enter_Flip(self, packet, Vec3(-dir_2D.x, -dir_2D.y, 0))
						else:
							v = (Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)).normal()
							
							ang = correction(my_car, v)
							
							Enter_Flip(self, packet, Vec3(-math.sin(ang), math.cos(ang), 0))
						self.controller_state.boost = True
						break
					count += 1
			
			# self.renderer.draw_line_3d(position.toVector3(), UI_Vec3(my_car.physics.location.x, my_car.physics.location.y, 100.0), self.renderer.green())
			
			# self.renderer.end_rendering()
		
	else:
		Drive_To(self, packet, Make_Vect(my_car.physics.location))

def Get_Ball_On_Car(self, packet: GameTickPacket, direction: Vec3):
	
	prediction = self.get_ball_prediction_struct()
	
	my_car = packet.game_cars[self.index]
	
	car_direction = get_car_facing_vector(my_car)
	
	ball_pos = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.1
	
	ball_pos_2 = Make_Vect(packet.game_ball.physics.location)
	
	ball_predict = Get_Ball_At_T(prediction, 0.1)
	
	if Vec3(my_car.physics.location.x - ball_pos.x, my_car.physics.location.y - ball_pos.y, 0.0).len() > 1000:
		ball_pos = Make_Vect(ball_predict.location)
	
	car_location = Make_Vect(my_car.physics.location)
	
	car_to_real_pos = ball_pos_2 - car_location
	
	if my_car.has_wheel_contact:
		
		v = Make_Vect(ball_predict.velocity).flatten()
		n = perp(v).normal() * 135
		if dot_2D(direction, n) > 0.0:
			n = n * -1
		
		car_to_pos = (ball_pos + n) - car_location
		
		ang = correction(my_car, car_to_real_pos) # car_direction.correction_to(car_to_real_pos)
		
		if car_to_real_pos.len() < 170.0 and (abs(ang) > math.pi * 0.4 or self.getting_ball_on_car):
			
			self.getting_ball_on_car = True
			
			self.controller_state.throttle = 0.3 # constrain(abs(ang) * 6, -1, 1)
			self.controller_state.boost = False
			self.controller_state.steer = constrain(-ang * 6, -1, 1)
			
			self.controller_state.handbrake = car_to_real_pos.len() > 240.0 and abs(ang) > math.pi * 0.6 and Make_Vect(my_car.physics.velocity).len() > 300
			
		elif car_to_real_pos.len() < 500.0:
			self.getting_ball_on_car = False
			if v.len() > 900:
				Drive_To(self, packet, ball_pos + n + car_direction * 75, True, True)
			elif abs(ang) > math.pi * 0.3 and abs(ang) < math.pi * 0.7:
				Drive_To(self, packet, ball_pos + car_direction * 200, False)
			else:
				Drive_To(self, packet, ball_pos, False)
			# self.controller_state.throttle = 1
			
			# self.renderer.draw_string_3d(my_car.physics.location, 3, 2, "Mid-stage", self.renderer.red())
			
		else:
			self.getting_ball_on_car = False
			Collect_Boost(self, packet, ball_pos + n - Make_Vect(my_car.physics.velocity) * 0.25, (car_to_real_pos - Make_Vect(my_car.physics.velocity)).len() > 1000.0)
		
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
		car_location = Make_Vect(my_car.physics.location)
		
		steer_correction_radians = correction(my_car, my_car.physics.velocity) # car_direction.correction_to(my_car.physics.velocity)
		
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
	
	car_rot = car.physics.rotation
	
	aim = aim.align_from(car_rot)
	aim.y = aim.y * 0.5
	aim = aim.align_to(car_rot)
	
	car_to_ball_real = Make_Vect(packet.game_ball.physics.location) - Make_Vect(car.physics.location)
	
	if car_to_ball_real.flatten().len() < 330 and car_to_pos.z < 230 and Make_Vect(car.physics.velocity).len() > 300:
		car_direction = get_car_facing_vector(car)
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.1 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.1
		ang = correction(car, c_p)
		Enter_Flip(self, packet, Vec3(-math.sin(ang), math.cos(ang), 0.0))
	elif car_to_ball_real.flatten().len() > 1500:
		Collect_Boost(self, packet, ball_predict - aim * (car_to_pos.len() * 0.04 + 100), True, True)
	else:
		Drive_To(self, packet, ball_predict - aim * (car_to_pos.len() * 0.04 + 100), True, True)
	
	self.renderer.draw_line_3d(car.physics.location, ball_predict.UI_Vec3(), self.renderer.white())
	self.renderer.draw_line_3d((ball_predict - aim * (car_to_pos.len() * 0.02 + 100)).UI_Vec3(), ball_predict.UI_Vec3(), self.renderer.red())
	

def Attack_Aim_Ball(self, packet: GameTickPacket, aim_pos: Vec3, ball_predict: Vec3):
	car = packet.game_cars[self.index]
	
	render_star(self, ball_predict, self.renderer.purple())
	
	render_star(self, aim_pos, self.renderer.green())
	
	car_rot = car.physics.rotation
	
	car_direction = get_car_facing_vector(car)
	
	a = (ball_predict - aim_pos).flatten().normal(-125)
	aim = (Vec3(a.x, a.y, 0) + car_direction * 150)
	aim_2 = Vec3(a.x, a.y, 0)
	
	aim = aim.align_from(car_rot)
	aim.y = aim.y * 0.5
	aim = aim.align_to(car_rot)
	
	aim_2 = aim.align_from(car_rot)
	aim_2.y = aim.y * 0.5
	aim_2 = aim.align_to(car_rot)
	
	car_to_pos = ball_predict - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.1
	
	car_to_pos_local = car_to_pos.align_from(car.physics.rotation)
	
	car_to_ball_real = Make_Vect(packet.game_ball.physics.location) - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.1
	
	# if ball_predict.z > 250:
		# Aerial_Hit_Ball(self, packet, aim_pos)
	if car_to_ball_real.len() < 350 and abs(car_to_ball_real.z) < 180:
		c_p = Make_Vect(packet.game_ball.physics.location) + Make_Vect(packet.game_ball.physics.velocity) * 0.15 - Make_Vect(car.physics.location) - Make_Vect(car.physics.velocity) * 0.15
		ang = correction(car, c_p)
		Enter_Flip(self, packet, Vec3(-math.sin(ang) * 2, math.cos(ang), 0.0).normal())
	elif abs(car_to_ball_real.z) > 250:
		Collect_Boost(self, packet, (ball_predict - aim_2 * 2), True, True, True)
	else:
		Collect_Boost(self, packet, (ball_predict - aim_2 * (car_to_ball_real.len() * 0.005 + 0.1)), True, True, True)
	
	self.renderer.draw_line_3d(ball_predict.UI_Vec3(), (ball_predict - aim_2 * (car_to_pos.len() * 0.005 + 0.1)).UI_Vec3(), self.renderer.red())
	
	self.renderer.draw_line_3d(packet.game_ball.physics.location, aim_pos.UI_Vec3(), self.renderer.white())
	


def Get_Impulse(packet, phys, point, time):
	
	# Ensure using our type of vector
	point = Make_Vect(point)
	
	phys_pos = Make_Vect(phys.location)
	
	phys_to_ball = point - phys_pos
	
	impulse_2D = phys_to_ball.flatten()
	
	impulse_2D *= (1 / max(0.0001, time))
	
	# Worked this out a while ago
	z_vel = -(0.5 * packet.game_info.world_gravity_z * time * time - phys_to_ball.z) / max(0.0001, time)
	
	return Vec3(impulse_2D.x, impulse_2D.y, z_vel)
	

def Align_Car_To(self, packet, vector: Vec3, up = Vec3(0, 0, 0)):
	
	my_car = packet.game_cars[self.index]
	
	self.renderer.draw_line_3d(my_car.physics.location, (Make_Vect(my_car.physics.location) + vector.normal(200)).UI_Vec3(), self.renderer.red())
	
	car_rot = my_car.physics.rotation
	
	car_rot_vel = Make_Vect(my_car.physics.angular_velocity)
	
	local_euler = car_rot_vel.align_from(car_rot)
	
	align_local = vector.align_from(car_rot)
	
	local_up = up.align_from(car_rot)
	
	# Improving this
	rot_ang_const = 0.25
	stick_correct = 6.0
	
	a1 = math.atan2(align_local.y, align_local.x)
	a2 = math.atan2(align_local.z, align_local.x)
	
	if local_up.y == 0 and local_up.z == 0:
		a3 = 0.0
	else:
		a3 = math.atan2(local_up.y, local_up.z)
	
	yaw = correct(0.0, -a1 + local_euler.z * rot_ang_const, stick_correct)
	pitch = correct(0.0, -a2 - local_euler.y * rot_ang_const, stick_correct)
	roll = correct(0.0, -a3 - local_euler.x * rot_ang_const, stick_correct)
	
	max_input = max(abs(pitch), max(abs(roll), abs(yaw)))
	
	# yaw /= max_input
	# roll /= max_input
	# pitch /= max_input
	
	self.controller_state.yaw = constrain(yaw, -1, 1)
	self.controller_state.pitch = constrain(pitch, -1, 1)
	self.controller_state.roll = constrain(roll, -1, 1)
	
	self.controller_state.steer = constrain(yaw, -1, 1)
	
	self.renderer.draw_line_3d(my_car.physics.location, (Make_Vect(my_car.physics.location) + align_local.align_to(car_rot).normal(100)).UI_Vec3(), self.renderer.yellow())
	

def Aerial_To(self, packet, point, time):
	
	my_car = packet.game_cars[self.index]
	
	impulse = Get_Impulse(packet, packet.game_cars[self.index].physics, point, time) - Make_Vect(my_car.physics.velocity)
	
	impulse_2 = impulse + Vec3(0, 0, 0.2) * impulse.len()
	
	forward = Vec3(1, 0, 0).align_to(my_car.physics.rotation)
	
	if dot(impulse_2.normal(), forward) > 0.8 and my_car.physics.location.z > 200:
		Align_Car_To(self, packet, impulse_2, Make_Vect(point) - Make_Vect(my_car.physics.location))
	else:
		Align_Car_To(self, packet, impulse_2)
	
	forward = Vec3(1, 0, 0).align_to(my_car.physics.rotation)
	
	self.controller_state.boost = impulse_2.len() > max(30, angle_between(impulse_2, forward) * 800) and angle_between(impulse_2, forward) < math.pi * 0.4
	
	return impulse
	

def Maneuver_Align(self, packet, time):
	
	my_car = packet.game_cars[self.index]
	
	self.jump_timer = self.jump_timer + self.delta
	
	self.aerial_hit_time -= self.delta
	
	ball_pos = Get_Ball_At_T(self.get_ball_prediction_struct(), self.aerial_hit_time).location
	
	self.renderer.draw_line_3d(ball_pos, (Make_Vect(ball_pos) - self.aerial_hit_position).UI_Vec3(), self.renderer.green())
	
	impulse = Aerial_To(self, packet, Make_Vect(ball_pos) - self.aerial_hit_position, self.aerial_hit_time) # self.aerial_hit_position, self.aerial_hit_time)
	
	local_impulse = impulse.align_from(my_car.physics.rotation)
	
	self.controller_state.jump = self.jump_timer < 0.2 and local_impulse.z > 50
	
	render_star(self, Make_Vect(Get_Ball_At_T(self.get_ball_prediction_struct(), self.aerial_hit_time).location), self.renderer.red())
	
	if self.jump_timer > 0.3 and not my_car.double_jumped and local_impulse.z > 400:
		self.controller_state.jump = True
		self.controller_state.yaw = 0.0
		self.controller_state.pitch = 0.0
		self.controller_state.roll = 0.0
		self.controller_state.steer = 0.0
	
	if (impulse.len() / 900 * 40 > (my_car.boost + 10) or impulse.len() / 900 > self.aerial_hit_time) and self.manuever_lock < -0.5:
		self.manuever_lock = 0.0
	

def Enter_Aerial(self, packet, time, aim):
	
	self.aerial_hit_time = time
	self.aerial_hit_position = aim
	
	self.jump_timer = 0.0
	
	self.has_jump = 0.6
	
	self.controller_state.jump = True
	
	self.controller_state.pitch = 0.0
	self.controller_state.roll = 0.0
	self.controller_state.yaw = 0.0
	
	self.manuever = Maneuver_Align
	self.manuever_lock = time + 0.2
	

def Maneuver_Align_Jump(self, packet, time):
	my_car = packet.game_cars[self.index]
	
	Align_Car_To(self, packet, Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location), Vec3(0, 0, 1))

def Enter_Align_Jump(self, packet):
	self.controller_state.jump = True
	
	self.manuever = Maneuver_Align_Jump
	self.manuever_lock = 0.5
	

# State setting awesomeness. This function is used to test stuff.
def Yeet_At_Ball(self, packet: GameTickPacket):
	
	if packet.game_cars[self.index].physics.location.z < 150:
		car_state = CarState(physics=Physics(location = Vector3(z=200), velocity=Vector3(z=0)))
		
		game_state = GameState(cars={self.index: car_state})
		self.set_game_state(game_state)
	else:
		
		# Attack Ball in exactly 3 seconds
		time = 3
		
		ball = Get_Ball_At_T(self.get_ball_prediction_struct(), time)
		
		impulse = Get_Impulse(packet, packet.game_cars[self.index].physics, ball.location, time)
		
		r = 300
		
		car_state = CarState(physics=Physics(velocity=Vector3(impulse.x + (random.random() - 0.5) * r, impulse.y + (random.random() - 0.5) * r, impulse.z + (random.random() - 0.5) * r)))
		
		game_state = GameState(cars={self.index: car_state})
		self.set_game_state(game_state)
		
		self.manuever = Maneuver_Align
		self.manuever_lock = 5.0
		
		self.aerial_hit_position = Make_Vect(ball.location)
		self.aerial_hit_time = 3.0
		
		return impulse
	

def Time_to_Pos(car, position, velocity):
	car_to_pos = Make_Vect(position) - Make_Vect(car.physics.location)
	
	# Subtract 100 from the length because we contact the ball slightly sooner than we reach the point
	len = max(0, car_to_pos.len() - 120)
	vel = Make_Vect(velocity)
	v_len = vel.len() * dot(car_to_pos.normal(), vel.normal())
	
	# curve:
	# f(t) = 0.5 * boost_accel * t ^ 2 + velocity * t
	
	# Analysis of t:
	# Solve for t when f(t) = len
	# Zeros of t: let c = len
	# 0.5 * boost_accel * t ^ 2 + velocity * t - len = 0
	
	# t = ( -velocity + sqrt(velocity^2 - 4(boost_accel)(-len)) ) / ( 2 * (boost_accel) )
	
	accel_time = (-v_len + math.sqrt(v_len * v_len + 4 * boost_accel * len)) / (2 * boost_accel)
	
	# However, the car speed maxes out at 2300 uu, so we need to account for that by stopping acceleration at 2300 uu. To do this we
	# calculate when we hit 2300 uu and cancel out any acceleration that happens after that
	
	# f(t) = 0.5 * boost_accel * t ^ 2 + velocity * t
	# Derivative:
	# v(t) = boost_accel * t + velocity
	# Solve for t when v(t) = 2300
	# 2300 = boost_accel * t + velocity
	# 2300 - velocity = boost_accel * t
	# ( 2300 - velocity ) / boost_accel = t
	
	max_vel_time = (2300 - v_len) / boost_accel
	
	a = 0
	
	if max_vel_time < accel_time:
		
		# plug time into position function
		pos = 0.5 * boost_accel * max_vel_time * max_vel_time + v_len * max_vel_time
		
		# Calculate additional distance that needs to be traveled
		extra_vel = len - pos
		
		# Add additional time onto velocity
		a = max_vel_time + extra_vel / 2300
		
	else:
		a = accel_time
	
	return a
	

def Aerial_Hit_Ball(self, packet: GameTickPacket, target: Vec3):
	
	my_car = packet.game_cars[self.index]
	
	time = 0
	ball_pos = packet.game_ball.physics.location
	car_pos = Make_Vect(my_car.physics.location) # + Make_Vect(my_car.physics.velocity) * 0.25
	
	aim = Vec3()
	
	up_vect = Vec3(0, 0, 0)
	
	# Car gets instantaneous velocity increase of 300 uu from first jump, plus some for second jump
	if not my_car.double_jumped:
		up_vect = Vec3(0, 0, 300).align_to(my_car.physics.rotation)
	
	for i in range(20):
		
		ball_pos = Make_Vect(Get_Ball_At_T(self.get_ball_prediction_struct(), time).location)
		
		aim = (target - ball_pos).normal(50)
		
		time = Time_to_Pos(my_car, ball_pos - aim, Make_Vect(my_car.physics.velocity) + up_vect) * 1.25
		
	
	self.controller_state.handbrake = False
	
	if ball_pos.z < 200:
		Attack_Aim_Ball(self, packet, target, ball_pos)
	elif time > 1:
		Collect_Boost(self, packet, ball_pos, True, True)
	else:
		
		impulse_raw = Get_Impulse(packet, my_car.physics, ball_pos, time)
		
		impulse = impulse_raw - Make_Vect(my_car.physics.velocity) - up_vect
		
		impulse_local = impulse.align_from(my_car.physics.rotation)
		
		forward_vect = Vec3(1, 0, 0).align_to(my_car.physics.rotation)
		
		if my_car.has_wheel_contact:
			a = correction(my_car, Get_Ball_At_T(self.get_ball_prediction_struct(), time).location)
			
			if abs(a) < math.pi * 0.1 or impulse_local.flatten().len() < 100:
				self.controller_state.throttle = constrain(impulse_local.x * 0.01, -1, 1)
				self.controller_state.steer = constrain(a, -1, 1)
			else:
				Drive_To(self, packet, ball_pos)
				# self.controller_state.throttle = sign(dot(my_car.physics.velocity, forward_vect))
			
			if dot(my_car.physics.velocity, get_car_facing_vector(my_car)) < 0.0:
				self.controller_state.steer = constrain(-self.controller_state.steer, -1, 1)
			
			# if abs(dot(Vec3(1, 0, 0), impulse_local.flatten().normal())) < 0.2 or impulse_local.flatten().len() < impulse_local.z * 0.5:
				# self.controller_state.handbrake = True
				# self.controller_state.throttle = 1.0
			
			self.controller_state.boost = self.controller_state.throttle > 0.99
			
			self.renderer.draw_line_3d(my_car.physics.location, (car_pos + impulse_raw).UI_Vec3(), self.renderer.red())
			self.renderer.draw_line_3d(my_car.physics.location, (car_pos + Make_Vect(my_car.physics.velocity) + up_vect).UI_Vec3(), self.renderer.yellow())
			
			render_star(self, ball_pos, self.renderer.yellow())
		else:
			Drive_To(self, packet, ball_pos)
		
		# Drive_To(self, packet, ball_pos, True)
		
		if dot(impulse_raw.normal(), (Make_Vect(my_car.physics.velocity) + up_vect).normal()) > 0.7 and impulse.len() / 900 * 40 < (my_car.boost + 10) and impulse.len() / 900 < time:
			self.jump_timer += self.delta
		else:
			self.jump_timer = 0.0
		
		# self.controller_state.jump = my_car.has_wheel_contact and self.jump_timer > 0.2
		
		# ((car_pos - Make_Vect(packet.game_ball.physics.location)).len() < 300 and not my_car.has_wheel_contact)
		if self.jump_timer > 0.1: # impulse_local.flatten().len() < max(75, time * 400)
			Enter_Aerial(self, packet, time, aim)
		

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
	
	if packet.game_info.is_kickoff_pause:
		
		self.jump_timer = 0.0
		self.line_up_time = 0.0
		state = "Kickoff"
		if (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() > 850:
			Collect_Boost(self, packet, Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.direction) * 150, True, False)
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
			ang = correction(my_car, (Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location) - Make_Vect(my_car.physics.velocity) * 0.2).normal()) # constrain(car_direction.correction_to((Make_Vect(my_car.physics.location) - direction).normal()), -math.pi * 0.4, math.pi * 0.4)
			
			Enter_Flip(self, packet, Vec3(math.sin(-ang) * 2, math.cos(-ang), 0.0).normal())
			self.controller_state.boost = False
			sub_state = "Flip"
	elif abs(my_car.physics.location.x) < 900 and (dot(Make_Vect(my_car.physics.location) + Make_Vect(my_car.physics.velocity) * 0.25 - Make_Vect(my_goal.location), my_goal.direction) < 0.0 and ((Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000.0 or Make_Vect(my_car.physics.velocity).flatten().len() > 1000)):
		state = "Inside Goal"
		# Driving away from ball
		car_face = get_car_facing_vector(my_car)
		if dot_2D(car_face.flatten().normal(), my_goal.direction) < 0.0 and dot_2D(car_face.flatten().normal(), (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location))) < 0.0 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000.0: # (Make_Vect(my_goal.location) - Make_Vect(ball_pos.location)).normal()) > 0.5:
			self.controller_state.throttle = -1.0
			self.controller_state.steer = 0.0
			self.controller_state.boost = False
			self.controller_state.handbrake = False
			self.controller_state.jump = False
			if Make_Vect(my_car.physics.velocity).len() < 1500.0 or dot_2D(car_face, my_car.physics.velocity) < 0.0:
				Enter_Half_Flip(self, packet)
			
			self.jump_timer = 0.0
			self.line_up_time = 0.0
			
			sub_state = "Half Flip"
		#Driving towards ball
		else:
			
			car_pos = Make_Vect(my_car.physics.location)
			
			# Hit_Ball_To(self, packet, Make_Vect(my_goal.location))
			p1 = Make_Vect(my_goal.location) + Vec3(900, my_goal.direction.y * 100, 0.0)
			p2 = Make_Vect(my_goal.location) - Vec3(900, -my_goal.direction.y * 100, 0.0)
			b = Make_Vect(ball_pos.location) - car_pos
			
			p1.z = 0.0
			p2.z = 0.0
			
			a1 = correction(my_car, p1 - car_pos)
			a2 = correction(my_car, p2 - car_pos)
			
			b_a = correction(my_car, b)
			
			if (a1 - b_a) * (a1 - a2) < 0.0 and (p1 - car_pos).len() > 100:
				self.jump_timer = 0.0
				self.line_up_time = 0.0
				Drive_To(self, packet, p1)
				self.is_arieal = False
				self.jump_timer = 0.0
				self.controller_state.boost = False
				sub_state = "Drive Around Post"
			elif (a2 - b_a) * (a1 - a2) > 0.0 and (p2 - car_pos).len() > 100:
				self.jump_timer = 0.0
				self.line_up_time = 0.0
				Drive_To(self, packet, p2)
				self.is_arieal = False
				self.jump_timer = 0.0
				self.controller_state.boost = False
				sub_state = "Drive Around Post"
			else:
				Aerial_Hit_Ball(self, packet, Make_Vect(opponent_goal.location))
				sub_state = "Hit Ball"
			
			self.renderer.draw_line_3d(my_car.physics.location, p1.UI_Vec3(), self.renderer.red())
			self.renderer.draw_line_3d(my_car.physics.location, p2.UI_Vec3(), self.renderer.red())
		
		# if my_car.physics.location.z > 200.0 and my_car.has_wheel_contact:
			# self.controller_state.jump = True
			# sub_state = "Jump Off Wall"
	elif closest_car_dist < 1000 and ((closest_car_pos - Make_Vect(my_car.physics.location)).len() < 500) and closest_car_pos.z - my_car.physics.location.z > 100 and packet.game_ball.physics.location.z - my_car.physics.location.z > 200:
		state = "Tactical Retreat"
		self.jump_timer = 0.0
		self.line_up_time = 0.0
		Drive_To(self, packet, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300, False, True)
	elif packet.game_ball.physics.location.z > 130 and (Make_Vect(ball_pos.location) - Make_Vect(my_car.physics.location)).len() < 200:
		self.jump_timer = 0.0
		self.line_up_time = 0.0
		state = "Maintain Dribble"
		Dribble(self, packet, Make_Vect(opponent_goal.location))
	elif dot((closest_car_pos - Make_Vect(ball_pos.location)).normal(), (Make_Vect(ball_pos.location) - Make_Vect(opponent_goal.location)).normal()) > -0.4 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() > 1000:
		state = "Powershot"
		self.jump_timer = 0.0
		self.line_up_time = 0.0
		if dot((Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).normal(), (Make_Vect(ball_pos.location) - Make_Vect(opponent_goal.location)).normal()) > 0.3:
			sub_state = "Shooting"
			Attack_Aim_Ball(self, packet, Make_Vect(opponent_goal.location) - Make_Vect(opponent_goal.direction) * 150, Make_Vect(Get_Ball_At_T(self.get_ball_prediction_struct(), Approximate_Time_To_Ball(self.get_ball_prediction_struct(), self.index, packet, 20, self.arieal_acceleration, False)).location))
			# Hit_Ball_To(self, packet, Make_Vect(opponent_goal.location) - Make_Vect(opponent_goal.direction) * 100, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 300)
		else:
			Collect_Boost(self, packet, Make_Vect(ball_pos.location) + Make_Vect(opponent_goal.direction) * 500, True, False)
			sub_state = "Lining up"
	elif (sign(ball_pos.velocity.y) != sign(my_goal.direction.y) and abs(ball_pos.velocity.y) > 500) or (closest_car_dist < 2000 and (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() < 4000) or (Make_Vect(ball_pos.location) - Make_Vect(my_goal.location)).len() < 1000:
		state = "Defending"
		v = Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_car.physics.location)
		v2 = Make_Vect(packet.game_ball.physics.location) - Make_Vect(my_goal.location)
		v3 = Make_Vect(my_car.physics.location) - Make_Vect(my_goal.location)
		if dot(v.normal(), my_goal.direction) > -0.3: # or dot(Make_Vect(packet.game_ball.physics.velocity).normal(), v2.normal()) < -0.5:
			c = Make_Vect(my_goal.location) + Make_Vect(my_goal.direction) * 1000 + Vec3(sign(v.x) * 5000, 0, 0)
			c.y = packet.game_ball.physics.location.y
			Aerial_Hit_Ball(self, packet, c)
			sub_state = "Clear Ball"
		elif my_car.has_wheel_contact:
			self.jump_timer = 0.0
			self.line_up_time = 0.0
			
			if dot(v3.normal(), v2.normal()) > 0.5 and v3.len() > v2.len():
				Hook_Ball(self, packet, Make_Vect(my_goal.location))
				sub_state = "Push to corner"
			elif (Make_Vect(my_car.physics.location) - Make_Vect(ball_pos.location)).len() > 1000:
				Collect_Boost(self, packet, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 200, True) #Vec3(-sign(car_to_pos.x) * 150, 0, 0))
				sub_state = "Collect Boost to Goal"
			else:
				Drive_To(self, packet, Make_Vect(my_goal.location) - Make_Vect(my_goal.direction) * 200, True)
				sub_state = "Head to Goal"
			
			if v3.len() < Make_Vect(my_car.physics.velocity).len() * -dot(Make_Vect(my_car.physics.velocity).normal(), Make_Vect(v3.normal())):
				Enter_Align_Jump(self, packet)
			
		else:
			
			self.jump_timer = 0.0
			self.line_up_time = 0.0
			
			if my_car.physics.location.z > -my_car.physics.velocity.z * 1.2 and my_car.physics.location.z > 200:
				Align_Car_To(self, packet, Make_Vect(my_car.physics.velocity).flatten().normal() - Vec3(0, 0, 2))
				self.controller_state.throttle = 1.0
				self.controller_state.boost = 0.3 > angle_between(Vec3(1, 0, 0).align_to(my_car.physics.rotation), Make_Vect(my_car.physics.velocity).flatten().normal() - Vec3(0, 0, 2))
			else:
				Align_Car_To(self, packet, Make_Vect(my_car.physics.velocity).flatten(), Vec3(0, 0, 1))
				self.controller_state.throttle = 1.0
			
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
	
	c_f = get_car_facing_vector(my_car)
	if abs(c_f.z) < 0.001 and abs(my_car.physics.location.y - opponent_goal.location.y) < 100 and abs(my_car.physics.location.x - opponent_goal.location.x) < 1500:
		self.controller_state.jump = True
	

class Tuxedo(BaseAgent):
	def reset(self):
		self.manuever_lock = -1.0
		self.flip_timer = 0.0
		self.is_arieal = False
		self.line_up_time = 0
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
		
		self.controller_state.throttle = 0.0
		self.controller_state.handbrake = False
		self.controller_state.boost = False
		self.controller_state.jump = False
		
		time = packet.game_info.seconds_elapsed
		self.delta = time - self.prev_time
		self.prev_time = time
		
		# Celebration and end of game quick chats
		if packet.game_info.is_match_ended:
			
			# self.controller_state.throttle = 1.0
			# self.controller_state.steer = -1.0
			# self.controller_state.handbrake = False
			# self.controller_state.boost = True
			
			if self.c_time > 10:
				my_car = packet.game_cars[self.index]
				
				Align_Car_To(self, packet, Vec3(0, 0, -1))
				
				self.controller_state.handbrake = False
				self.controller_state.jump = my_car.has_wheel_contact
				
				if my_car.has_wheel_contact:
					self.controller_state.yaw = 0.0
					self.controller_state.roll = 0.0
					self.controller_state.pitch = 0.0
			
			self.c_time += 1
			
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

def get_car_facing_vector(car):
	return Vec3(1, 0, 0).align_to(car.physics.rotation)

