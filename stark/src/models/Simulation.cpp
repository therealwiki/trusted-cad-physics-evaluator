#include "Simulation.h"

using namespace stark;


double Simulation::get_time() const
{
	return this->stark.current_time;
}

double Simulation::get_time_step_size() const
{
	return this->stark.dt;
}

int Simulation::get_frame() const
{
	return this->stark.current_frame;
}

Eigen::Vector3d Simulation::get_gravity() const
{
	return this->stark.gravity;
}

void Simulation::set_gravity(const Eigen::Vector3d& gravity)
{
	this->stark.gravity = gravity;
}

symx::Logger& Simulation::get_logger()
{
	return *this->stark.context->logger;
}

const Settings& Simulation::get_settings() const
{
	return this->stark.settings;
}

void Simulation::add_time_event(double t0, double t1, std::function<void(double)> action)
{
	this->add_time_event(t0, t1, [action](double t, EventInfo& event_info) { action(t); });
}
void Simulation::add_time_event(double t0, double t1, std::function<void(double, EventInfo&)> action)
{
	this->stark.script.add_event(
		/* action = */ [action, this](EventInfo& event_info) { action(this->get_time(), event_info); },
		/* run_when = */ [t0, t1, this](EventInfo& event_info) { return this->get_time() >= t0 && this->get_time() < t1; },
		/* delete_when = */ [t1, this](EventInfo& event_info) { return this->get_time() >= t1; }
	);
}

void Simulation::run(std::function<void()> callback)
{
	this->run(std::numeric_limits<double>::max(), callback);
}

EventDrivenScript& Simulation::get_script()
{
	return this->stark.script;
}

spCallbacks &Simulation::get_callbacks()
{
    return this->stark.callbacks;
}

void Simulation::run(double duration, std::function<void()> user_callback)
{
	this->stark.run(duration, 
		[user_callback, this]()
		{
			this->stark.script.run_a_cycle(this->get_time());
			if (user_callback != nullptr) user_callback();
		}
	);
}

bool Simulation::run_one_time_step()
{
	this->stark.script.run_a_cycle(this->get_time());
	return this->stark.run_one_step();
}

Simulation::Simulation(const Settings& settings)
	: stark(settings)
{
	// Base dynamics
	spPointDynamics point_dynamics = std::make_shared<PointDynamics>(this->stark);
	spRigidBodyDynamics rb_dynamics = std::make_shared<RigidBodyDynamics>(this->stark);

	// Physical Systems
	this->deformables = std::make_shared<Deformables>(this->stark, point_dynamics);
	this->rigidbodies = std::make_shared<RigidBodies>(this->stark, rb_dynamics);

	// Interactions
	this->interactions = std::make_shared<Interactions>(this->stark, point_dynamics, rb_dynamics);

	// Presets
	this->presets = std::make_shared<Presets>(this->stark, this->deformables, this->rigidbodies, this->interactions);
}
