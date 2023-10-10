mod utils;

use std::f64;
use web_sys;
use wasm_bindgen::prelude::*;

use rand::prelude::*;

use min_max::*;

use nalgebra;
use nalgebra::{Rotation2};

pub type RealType = f32;
pub type Vec2f = nalgebra::SVector<RealType,2>;

pub type Tick = usize;



const SPAWNED_AGENTS_VELOCITY: RealType = 2.2;

// (Radius, Weight), depending on tick
const PHEROMONE_STATES: &[(RealType,RealType)] = &[
	(1.0,1.0),
	(1.5,0.4),
	(2.25,0.15),
	(3.0,0.05),
	(3.5,0.0),
];

const PHEROMONE_LIFETIME: usize = 100;

const PHEROMONE_DROP_PERIOD: usize = 4;




struct Pheromone {
	position: Vec2f, // chunk local position
	created: Tick,
}

impl Pheromone {
	pub fn new(position: Vec2f,created: Tick)->Self {
		Self{position,created}
	}
	
	pub fn radius_and_weight(&self,now: Tick)->(RealType,RealType) {
		let time = now - self.created;
		
		if time <= PHEROMONE_LIFETIME {
			let pairs_number = PHEROMONE_STATES.len() - 1;
			
			let pair_index = min!(((time as f32/PHEROMONE_LIFETIME as f32)*pairs_number as f32) as usize,pairs_number - 1);
			
			let pair_duration = PHEROMONE_LIFETIME/pairs_number;
			
			// If this is last pair, let duration be the rest of time
			let local_factor = if pair_index == pairs_number - 1 {
				let head = pair_duration*(pairs_number - 1);
				
				let duration = PHEROMONE_LIFETIME - head;
				let local_time = duration%head;
				
				(local_time) as f32/duration as f32
			}
			else {
				(time%pair_duration) as f32/pair_duration as f32
			};
			
			let ps0 = PHEROMONE_STATES[pair_index];
			let ps1 = PHEROMONE_STATES[pair_index + 1];
			
			// Linear interpolation
			(ps0.0*(1.0 - local_factor) + ps1.0*local_factor,ps0.1*(1.0 - local_factor) + ps1.1*local_factor)
		}
		else {
			panic!("Error! Pheromone lifetime exceeded limit");
		}
	}
}



struct Agent {
	position: Vec2f, // chunk local position
	velocity: Vec2f,
}

impl Agent {
	pub fn new(position: Vec2f,velocity: Vec2f)->Self {
		Self{position,velocity}
	}
}



struct Chunk {
	x: usize,
	y: usize,
	
	agents: Vec<Agent>,
	pheromones: Vec<Pheromone>,
}

impl Chunk {
	pub fn new(x: usize,y: usize)->Self {
		Self {
			x,
			y,
			agents: Vec::new(),
			pheromones: Vec::new(),
		}
	}
}



enum AgentCommand {
	LEFT,
	RIGHT,
	FORWARD,
}



pub fn chunk_coordinate(size: usize,c: usize,pc: isize)->usize {
	((size as isize + c as isize + pc) as usize)%size
}

pub fn global_position(position: Vec2f,chunk_size: RealType,x: usize,y: usize)->Vec2f {
	Vec2f::new(x as RealType*chunk_size + position.x,y as RealType*chunk_size + position.y)
}



// Default size_w and size_h is about 12, chunk_size is 16.0
#[wasm_bindgen]
pub struct World {
	pub size_w: usize, // in chunks
	pub size_h: usize, // in chunks
	
	pub chunk_size: RealType, // in "pixels" or ones
	
	pub sensor_angle: RealType, // in radians
	pub sensor_range: RealType, // in "pixels" or ones
	pub angle_change: RealType, // in radians
	pub agent_visiblity: bool,
	
	chunks: Vec<Vec<Chunk>>, // containers for agents and pheromone
	
	pub tick: Tick, // current world tick
}

#[wasm_bindgen]
impl World {
	pub fn new(size_w: usize,size_h: usize,chunk_size: RealType,sensor_angle: RealType,sensor_range: RealType,angle_change: RealType)->Self {
		assert!(sensor_range <= chunk_size/2.0);
		
		Self {
			size_w,
			size_h,
			chunk_size,
			sensor_angle,
			sensor_range,
			angle_change,
			agent_visiblity: true,
			
			chunks: (0..size_h).map(|y| (0..size_w).map(|x| Chunk::new(x,y)).collect()).collect(),
			
			tick: 0,
		}
	}
	
	
	pub fn update(&mut self) {
		// Transfering agents between chunks
		
		// Vector to store (Agent, new chunk x, new chunk y)
		let mut new_agents: Vec<(Agent,usize,usize)> = Vec::new();
		
		for y in 0..self.size_h {
			for x in 0..self.size_w {
				let chunk = &mut self.chunks[y][x];
				
				let mut i = 0;
				while i < chunk.agents.len() {
					let agent = &chunk.agents[i];
					
					let axl = agent.position.x < 0.0;
					let axg = agent.position.x >= self.chunk_size;
					let ayl = agent.position.y < 0.0;
					let ayg = agent.position.y >= self.chunk_size;
					
					if axl || axg || ayl || ayg {
						let mut agent = chunk.agents.remove(i);
						
						let px = if axl {
							-1
						}
						else if axg {
							1
						}
						else {
							0
						};
						let py = if ayl {
							-1
						}
						else if ayg {
							1
						}
						else {
							0
						};
						
						agent.position.x -= px as RealType*self.chunk_size;
						agent.position.y -= py as RealType*self.chunk_size;
						
						new_agents.push((
							agent,
							chunk_coordinate(self.size_w,x,px),
							chunk_coordinate(self.size_h,y,py),
						));
					}
					else {
						i += 1;
					}
				}
			}
		}
		
		for (a,cx,cy) in new_agents {
			let another_chunk = &mut self.chunks[cy][cx];
			another_chunk.agents.push(a);
		}
		//
		
		// Compute agent commands, depending on surrounding pheromone
		let mut agent_commands: Vec<AgentCommand> = Vec::new();
		
		for y in 0..self.size_h {
			for x in 0..self.size_w {
				let chunk = &self.chunks[y][x];
				
				let neighbor_n  = &self.chunks[chunk_coordinate(self.size_h,y, 1)][chunk_coordinate(self.size_w,x, 0)];
				let neighbor_ne = &self.chunks[chunk_coordinate(self.size_h,y, 1)][chunk_coordinate(self.size_w,x, 1)];
				let neighbor_e  = &self.chunks[chunk_coordinate(self.size_h,y, 0)][chunk_coordinate(self.size_w,x, 1)];
				let neighbor_se = &self.chunks[chunk_coordinate(self.size_h,y,-1)][chunk_coordinate(self.size_w,x, 1)];
				let neighbor_s  = &self.chunks[chunk_coordinate(self.size_h,y,-1)][chunk_coordinate(self.size_w,x, 0)];
				let neighbor_sw = &self.chunks[chunk_coordinate(self.size_h,y,-1)][chunk_coordinate(self.size_w,x,-1)];
				let neighbor_w  = &self.chunks[chunk_coordinate(self.size_h,y, 0)][chunk_coordinate(self.size_w,x,-1)];
				let neighbor_nw = &self.chunks[chunk_coordinate(self.size_h,y, 1)][chunk_coordinate(self.size_w,x,-1)];
				
				for agent in chunk.agents.iter() {
					let mut probing: Vec<&Chunk> = vec![chunk];
					
					let nx = if agent.position.x < self.chunk_size/2.0 {
						-1
					}
					else {
						1
					};
					let ny = if agent.position.y < self.chunk_size/2.0 {
						-1
					}
					else {
						1
					};
					
					if nx == 1 {
						probing.push(neighbor_e);
					}
					else {
						probing.push(neighbor_w);
					}
					if ny == 1 {
						probing.push(neighbor_n);
					}
					else {
						probing.push(neighbor_s);
					}
					probing.push(
						match (nx,ny) {
							( 1, 1) => neighbor_ne,
							(-1, 1) => neighbor_nw,
							(-1,-1) => neighbor_sw,
							( 1,-1) => neighbor_se,
							
							default => panic!("(nx,ny) doesn't comply"),
						}
					);
					
					let rotation_left  = Rotation2::new(self.sensor_angle);
					let rotation_right = Rotation2::new(-self.sensor_angle);
					
					let dir_forward = agent.velocity.normalize()*self.sensor_range;
					let dir_left = rotation_left*dir_forward;
					let dir_right = rotation_right*dir_forward;
					
					let agent_position = global_position(agent.position,self.chunk_size,chunk.x,chunk.y);
					
					let probe_forward = agent_position + dir_forward;
					let probe_left = agent_position + dir_left;
					let probe_right = agent_position + dir_right;
					
					let mut weight_forward = 0.0;
					let mut weight_left = 0.0;
					let mut weight_right = 0.0;
					
					for chunk_probe in probing.iter() {
						// Here we will check if probes points are in pheromones circles
						// Note, that this code isn't ideal and checking pheromones on wrapped sides aren't implemented here
						
						for pheromone in chunk_probe.pheromones.iter() {
							let pheromone_position = global_position(pheromone.position,self.chunk_size,chunk_probe.x,chunk_probe.y);
							
							let (radius,weight) = pheromone.radius_and_weight(self.tick);
							
							let radius_squared = radius*radius;
							
							if (pheromone_position - probe_forward).magnitude_squared() <= radius_squared {
								weight_forward += weight;
							}
							if (pheromone_position - probe_left).magnitude_squared() <= radius_squared {
								weight_left += weight;
							}
							if (pheromone_position - probe_right).magnitude_squared() <= radius_squared {
								weight_right += weight;
							}
						}
					}
					
					if weight_left > weight_forward && weight_left > weight_right {
						agent_commands.push(AgentCommand::LEFT);
					}
					else if weight_right > weight_forward && weight_right > weight_left {
						agent_commands.push(AgentCommand::RIGHT);
					}
					else {
						agent_commands.push(AgentCommand::FORWARD);
					}
				}
			}
		}
		//
		
		// Apply commands to agents
		let mut i = 0;
		
		for y in 0..self.size_h {
			for x in 0..self.size_w {
				let chunk = &mut self.chunks[y][x];
				
				for agent in chunk.agents.iter_mut() {
					match agent_commands[i] {
						AgentCommand::LEFT => {
							let rotation  = Rotation2::new(self.angle_change);
							
							agent.velocity = rotation*agent.velocity;
						}
						AgentCommand::RIGHT => {
							let rotation  = Rotation2::new(-self.angle_change);
							
							agent.velocity = rotation*agent.velocity;
						}
						AgentCommand::FORWARD => {
							// do nothing
						}
					}
					
					i += 1;
				}
			}
		}
		//
		
		// Final stuff
		for y in 0..self.size_h {
			for x in 0..self.size_w {
				let chunk = &mut self.chunks[y][x];
				
				// Moving agent
				for agent in chunk.agents.iter_mut() {
					agent.position += agent.velocity;
				}
				
				// Dropping pheromone by agents
				if self.tick%PHEROMONE_DROP_PERIOD == 0 {
					for agent in chunk.agents.iter() {
						chunk.pheromones.push(
							Pheromone::new(agent.position,self.tick)
						);
					}
				}
				
				// Remove pheromones which lifetime is exceeded
				let self_tick = self.tick;
				chunk.pheromones.retain(|p| {
					self_tick - p.created < PHEROMONE_LIFETIME
				})
			}
		}
		//
		
		// Increment world tick
		self.tick += 1;
	}
	
	pub fn draw(&self) {
		let document = web_sys::window().unwrap().document().unwrap();
		let canvas = document.get_element_by_id("canvas").unwrap();
		let canvas: web_sys::HtmlCanvasElement = canvas
			.dyn_into::<web_sys::HtmlCanvasElement>()
			.map_err(|_| ())
			.unwrap();
		
		let context = canvas
			.get_context("2d")
			.unwrap()
			.unwrap()
			.dyn_into::<web_sys::CanvasRenderingContext2d>()
			.unwrap();
		
		
		context.clear_rect(0.0,0.0,(self.size_w as RealType*self.chunk_size).into(),(self.size_h as RealType*self.chunk_size).into());
		
		
		
		context.set_fill_style(&JsValue::from_str("rgba(0,255,0,0.15)"));
		
		for y in 0..self.size_h {
			for x in 0..self.size_w {
				let chunk = &self.chunks[y][x];
				
				for pheromone in chunk.pheromones.iter() {
					let (radius,_) = pheromone.radius_and_weight(self.tick);
					
					context.begin_path();
					
					let pheromone_position = global_position(pheromone.position,self.chunk_size,x,y);
					
					context.arc(
						pheromone_position.x.into(),
						pheromone_position.y.into(),
						radius.into(),
						0.0,
						f64::consts::PI*2.0
					).unwrap();
					
					context.fill();
					context.close_path();
				}
			}
		}
		
		
		
		if self.agent_visiblity {
			for y in 0..self.size_h {
				for x in 0..self.size_w {
					let chunk = &self.chunks[y][x];
					
					
					for agent in chunk.agents.iter() {
						context.begin_path();
						
						let agent_position = global_position(agent.position,self.chunk_size,x,y);
						
						context.arc(
							agent_position.x.into(),
							agent_position.y.into(),
							5.0,
							0.0,
							f64::consts::PI*2.0
						).unwrap();
						
						context.stroke();
						context.close_path();
					}
				}
			}
		}
	}
	
	
	pub fn set_sensor_angle(&mut self,sensor_angle: RealType) {
		self.sensor_angle = sensor_angle;
	}
	
	pub fn set_sensor_range(&mut self,sensor_range: RealType) {
		assert!(sensor_range <= self.chunk_size/2.0);
		
		self.sensor_range = sensor_range;
	}
	
	pub fn set_angle_change(&mut self,angle_change: RealType) {
		self.angle_change = angle_change;
	}
	
	pub fn set_agent_visiblity(&mut self,agent_visiblity: bool) {
		self.agent_visiblity = agent_visiblity;
	}
	
	
	pub fn spawn_random_agents(&mut self,number: usize) {
		let mut rng = rand::thread_rng();
		
		let mut full: f32 = 0.0;
		let delta = number as f32/(self.size_w*self.size_h) as f32;
		
		for y in 0..self.size_h {
			for x in 0..self.size_w {
				let full_old = full;
				full += delta;
				
				let n = (full - full_old.floor()) as usize;
				
				let mut agents: Vec<Agent> = 
					(0..n)
					.map(|_| {
						let (r0,r1,r2,r3) : (RealType,RealType,RealType,RealType) = (rng.gen(),rng.gen(),rng.gen(),rng.gen());
						
						Agent::new(
							Vec2f::new(r0*self.chunk_size,r1*self.chunk_size),
							Vec2f::new(r2*SPAWNED_AGENTS_VELOCITY,r3*SPAWNED_AGENTS_VELOCITY)
						)
					})
					.collect();
				
				self.chunks[y][x].agents.append(&mut agents);
			}
		}
	}
}


























