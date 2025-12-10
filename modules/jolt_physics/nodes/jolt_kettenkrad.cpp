
#include "jolt_kettenkrad.h"
#include "../jolt_physics_server_3d.h"

#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_body_3d.h"
#include "../spaces/jolt_space_3d.h"
#include "Jolt/Physics/Vehicle/TrackedVehicleController.h"
#include "../spaces/jolt_broad_phase_layer.h"

JoltKettenkrad::JoltKettenkrad() : RigidBody3D() {
}

void JoltKettenkrad::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_driver_input"), &JoltKettenkrad::set_driver_input);
	ClassDB::bind_method(D_METHOD("get_wheel_positions"), &JoltKettenkrad::get_wheel_positions);
	ClassDB::bind_method(D_METHOD("init_vehicle"), &JoltKettenkrad::init_vehicle);
	ClassDB::bind_method(D_METHOD("deinit_vehicle"), &JoltKettenkrad::deinit_vehicle);

	ClassDB::bind_method(D_METHOD("get_engine_rpm_ratio"), &JoltKettenkrad::get_engine_rpm_ratio);
	ClassDB::bind_method(D_METHOD("get_trans_gear"), &JoltKettenkrad::get_trans_gear);
	ClassDB::bind_method(D_METHOD("is_switching_gear"), &JoltKettenkrad::is_switching_gear);

	ClassDB::bind_method(D_METHOD("get_wheel_count"), &JoltKettenkrad::get_wheel_count);
	ClassDB::bind_method(D_METHOD("get_wheel_transform"), &JoltKettenkrad::get_wheel_transform);

	ClassDB::bind_method(D_METHOD("get_wheel_contacts"), &JoltKettenkrad::get_wheel_contacts);
	ClassDB::bind_method(D_METHOD("get_wheel_rotation_angles"), &JoltKettenkrad::get_wheel_rotation_angles);
	ClassDB::bind_method(D_METHOD("get_wheel_angular_velocities"), &JoltKettenkrad::get_wheel_angular_velocities);
	ClassDB::bind_method(D_METHOD("get_wheel_lateral_relative_velocities"), &JoltKettenkrad::get_wheel_lateral_relative_velocities);

	ClassDB::bind_method(D_METHOD("set_front_steer_angle"), &JoltKettenkrad::set_front_steer_angle);
	ClassDB::bind_method(D_METHOD("set_friction_factor"), &JoltKettenkrad::set_friction_factor);

	ClassDB::bind_method(D_METHOD("set_use_low_gear"), &JoltKettenkrad::set_use_low_gear);
	ClassDB::bind_method(D_METHOD("get_use_low_gear"), &JoltKettenkrad::get_use_low_gear);

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_low_gear"), "set_use_low_gear", "get_use_low_gear");
}


void JoltKettenkrad::set_use_low_gear(bool p_use_low_gear) {
	if (m_vehicle_constraint == nullptr) {return;}

	auto* controller = static_cast<JPH::TrackedVehicleController*>(m_vehicle_constraint->GetController());

	auto& trans = controller->GetTransmission();

	is_low_gear = p_use_low_gear;
	if (p_use_low_gear) {
		trans.mGearRatios = {14.8f, 9.7f, 5.95f};
		trans.mReverseGearRatios = {-11.9f};
	} else {
		trans.mGearRatios = {2.57f, 1.60f, 1.03f};
		trans.mReverseGearRatios = {-2.07f};
	}

}

bool JoltKettenkrad::get_use_low_gear() {
	return is_low_gear;
}


void JoltKettenkrad::set_driver_input(float forward, float left_ratio, float right_ratio, float brake) {
	if (m_vehicle_constraint == nullptr) {
		return;
	}

	static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController())->SetDriverInput(forward, left_ratio, right_ratio, brake);
}


void JoltKettenkrad::set_front_steer_angle(float in_angle) {
	if (m_vehicle_constraint == nullptr) {
		return;
	}

	auto* wheel = m_vehicle_constraint->GetWheel(12);
	wheel->SetSteerAngle(in_angle);

	// Do something AWFUL here
	JPH::WheelSettings* wheel_settings = const_cast<JPH::WheelSettings*>(wheel->GetSettings());
	wheel_settings->mPosition.SetX(in_angle * 0.05);
}

void JoltKettenkrad::set_friction_factor(float factor) {
	this->friction_factor = factor;
}

double JoltKettenkrad::get_engine_rpm_ratio() {
	if (m_vehicle_constraint == nullptr) {
		return 0.0;
	}

	auto* c = static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController());
	const auto& engine = c->GetEngine();

	return (engine.GetCurrentRPM() - engine.mMinRPM) / (engine.mMaxRPM - engine.mMinRPM);
}

int JoltKettenkrad::get_trans_gear() {
	if (m_vehicle_constraint == nullptr) {
		return 0;
	}

	auto* c = static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController());

	return c->GetTransmission().GetCurrentGear();
}

bool JoltKettenkrad::is_switching_gear() {
	if (m_vehicle_constraint == nullptr) {
		return false;
	}

	auto* c = static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController());

	return c->GetTransmission().IsSwitchingGear();
}

PackedVector3Array JoltKettenkrad::get_wheel_positions() {
	if (Engine::get_singleton()->is_editor_hint() || m_vehicle_constraint == nullptr) {
		return PackedVector3Array();
	}

	auto array = PackedVector3Array();
	for (uint32_t w = 0; w < m_vehicle_constraint->GetWheels().size(); ++w)
	{
		JPH::RMat44 wheel_transform = m_vehicle_constraint->GetWheelLocalTransform(w, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX()); // The cylinder we draw is aligned with Y so we specify that as rotational axis
		array.append(to_godot(wheel_transform.GetTranslation()));
	}

	return array;
}

PackedFloat32Array JoltKettenkrad::get_wheel_rotation_angles() {
	if (Engine::get_singleton()->is_editor_hint() || m_vehicle_constraint == nullptr) {
		return PackedFloat32Array();
	}

	auto array = PackedFloat32Array();
	for (uint32_t w = 0; w < m_vehicle_constraint->GetWheels().size(); ++w)
	{
		const JPH::Wheel* wheel = m_vehicle_constraint->GetWheel(w);
		array.append(wheel->GetRotationAngle());
	}

	return array;
}

PackedFloat32Array JoltKettenkrad::get_wheel_angular_velocities() {
	if (Engine::get_singleton()->is_editor_hint() || m_vehicle_constraint == nullptr) {
		return PackedFloat32Array();
	}

	auto array = PackedFloat32Array();
	JPH::Wheels& wheels = m_vehicle_constraint->GetWheels();
	array.resize(wheels.size());
	for (uint32_t w = 0; w < wheels.size(); ++w) {
		array.set(w, wheels[w]->GetAngularVelocity());
	}


	return array;
}

PackedFloat32Array JoltKettenkrad::get_wheel_lateral_relative_velocities() {
	if (Engine::get_singleton()->is_editor_hint() || m_vehicle_constraint == nullptr) {
		return PackedFloat32Array();
	}

	auto array = PackedFloat32Array();
	JPH::Wheels& wheels = m_vehicle_constraint->GetWheels();
	array.resize(wheels.size());
	for (uint32_t w = 0; w < wheels.size(); ++w) {
		auto* wheel = wheels[w];
		JPH::Vec3 relative_velocity = m_vehicle_constraint->GetVehicleBody()->GetPointVelocity(wheel->GetContactPosition()) - wheel->GetContactPointVelocity();
		float relative_lateral_velocity = relative_velocity.Dot(wheel->GetContactLateral());

		array.set(w, relative_lateral_velocity);
	}

	return array;
}

PackedInt32Array JoltKettenkrad::get_wheel_contacts() {
	auto array = PackedInt32Array();

	if (m_vehicle_constraint == nullptr) {
		return array;
	}

	for (JPH::Wheel *w_base : m_vehicle_constraint->GetWheels()) {
		array.append(w_base->HasContact());
	}

	return array;
}

Transform3D JoltKettenkrad::get_wheel_transform(uint32_t wheel_idx) {
	if (Engine::get_singleton()->is_editor_hint() || m_vehicle_constraint == nullptr || wheel_idx >= m_vehicle_constraint->GetWheels().size()) {
		return Transform3D();
	}

	return to_godot(m_vehicle_constraint->GetWheelLocalTransform(wheel_idx, JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY()));
}

void JoltKettenkrad::deinit_vehicle() {
	// auto* server = dynamic_cast<JoltPhysicsServer3D*>(PhysicsServer3D::get_singleton());
	JoltPhysicsServer3D* server = JoltPhysicsServer3D::get_singleton();

	// Only supported with JoltPhysics3D server, exit early in that case
	if (server == nullptr) {
		return;
	}

	auto* body_impl = server->get_body(get_rid());

	if (body_impl == nullptr) {
		return;
	}

	auto* space = body_impl->get_space();

	if (space == nullptr) {
		return;
	}

	auto& system = space->get_physics_system();

	if (m_vehicle_constraint != nullptr) {
		system.RemoveStepListener(m_vehicle_constraint);
		system.RemoveConstraint(m_vehicle_constraint);
		delete m_vehicle_constraint;
		m_vehicle_constraint = nullptr;
	}
}

void JoltKettenkrad::init_vehicle() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	deinit_vehicle(); // Allow implicit reinitialization (e.g. space changed)
	JoltPhysicsServer3D* server = JoltPhysicsServer3D::get_singleton();

	if (server == nullptr) { return; }

	auto* body_impl = server->get_body(get_rid());
	auto* space = body_impl->get_space();
	auto& system = space->get_physics_system();

	JPH::VehicleConstraintSettings vehicle;
	vehicle.mMaxPitchRollAngle = 3.1415f / 2.5;
	vehicle.mDrawConstraintSize = 0.05f;

	auto *controller = new JPH::TrackedVehicleControllerSettings();
	vehicle.mController = controller;

	auto& engine = controller->mEngine;
	engine.mMaxTorque = 80.0f; // 36 bhp at 3400 rpm gives 75nM. We can assume 3400 is a bit past peak torque (it's pretty high revs for that engine).
	engine.mMaxRPM = 3800.f + 200.f; // 36 bhp at 3800 or 3200? But apparently top cruise is at 2800rpm for ~60kph
	engine.mMinRPM = 600.f; // Idle RPM from that one guy in Japan, trust that over the specs (which say 10k)
	engine.mAngularDamping = 0.1f; // This is probably too high, but it's an old vehicle and it's running treads, so you'd expect a fair amount of friction.
	engine.mInertia = 0.4f;

	engine.mNormalizedTorque.Clear();
	engine.mNormalizedTorque.Reserve(5);
	engine.mNormalizedTorque.AddPoint( 0.0f, 0.7f);
	engine.mNormalizedTorque.AddPoint(0.25f, 0.9f);
	engine.mNormalizedTorque.AddPoint( 0.5f, 1.0f);
	engine.mNormalizedTorque.AddPoint(0.75f, 1.0f);
	engine.mNormalizedTorque.AddPoint( 1.0f, 0.9f);

	auto& trans = controller->mTransmission;

	// Ratios taken from "[Military History 038] - Kettenkrad Sd Kfz 2 Type HK-101 (Schiffer Publishing)" :)
	trans.mGearRatios = {2.57f, 1.60f, 1.03f}; // High gear ratios
//	trans.mGearRatios = {14.8f, 9.7f, 5.95f}; // Low gear ratios

	trans.mReverseGearRatios = {-2.07f}; // High gear ratios
//	trans.mReverseGearRatios = {-11.9f}; // Low gear ratios


	trans.mShiftUpRPM = 3100.f;
	trans.mShiftDownRPM = 1000.f;
	trans.mClutchStrength = 20.0f;

	// Wheel just hardcode the we'll positions for now
	static JPH::Vec3 wheel_pos[] = {
		JPH::Vec3(0.0f, 0.05f, 0.52f),
		JPH::Vec3(0.0f, 0.05f, 0.0f),
		JPH::Vec3(0.0f, 0.05f, -0.27f),
		JPH::Vec3(0.0f, 0.05f, -0.54f),
		JPH::Vec3(0.0f, 0.05f, -0.82f),
		JPH::Vec3(0.0f, 0.05f, -1.14f),
	};

	const uint32_t NUM_WHEELS_PER_TRACK = 6;

	float wheel_radius = 0.25f;
	float wheel_width = 0.15f;
	float suspension_min_length = 0.0f;
	float suspension_max_length = 0.2f;
	float suspension_frequency = 1.0f;
	float half_body_width = 0.85f / 2.0f;


	for (int track_idx = 0; track_idx < 2; track_idx++) {
		JPH::VehicleTrackSettings &track = controller->mTracks[track_idx];
		track.mMaxBrakeTorque = 500.0f;
		track.mDifferentialRatio = /* Differential Gear = */ 1.91 * /* Reduction Drive */ 2.143;

		track.mDrivenWheel = (uint32_t)(vehicle.mWheels.size() + NUM_WHEELS_PER_TRACK - 1);
		track.mInertia = 2.0f;

		for (uint32_t wheel = 0; wheel < NUM_WHEELS_PER_TRACK; wheel++) {
			JPH::WheelSettingsTV* w = new JPH::WheelSettingsTV();

			w->mPosition = wheel_pos[wheel];
			w->mPosition.SetX(track_idx == 0? half_body_width : -half_body_width);
			w->mRadius = wheel_radius;
			w->mWidth = wheel_width;
			w->mSuspensionMinLength = suspension_min_length;
			w->mSuspensionMaxLength = suspension_max_length;

			if (wheel == 0) {
				// Front drive wheel, give it a tiny suspension for track compression
				w->mSuspensionMaxLength = suspension_min_length + 0.02f;
				w->mRadius = 0.225;
			} else if (wheel == NUM_WHEELS_PER_TRACK - 1) {
				// Rear road wheel, ditto
				w->mSuspensionMaxLength = suspension_min_length + 0.02f;
				w->mRadius = 0.225;
			}
			w->mSuspensionSpring.mFrequency = suspension_frequency;

			// Add the wheel to the vehicle
			track.mWheels.push_back((uint32_t)vehicle.mWheels.size());
			vehicle.mWheels.push_back(w);

			// Add anti-roll bars
			if (wheel != 0 && wheel != NUM_WHEELS_PER_TRACK - 1) {
				JPH::VehicleAntiRollBar rollbar;
				rollbar.mLeftWheel = wheel;
				rollbar.mRightWheel = wheel + NUM_WHEELS_PER_TRACK;
				rollbar.mStiffness = 100.f;
				vehicle.mAntiRollBars.push_back(rollbar);
			}
		}
	}

	// Extra front wheel
	auto *w = new JPH::WheelSettingsTV();
	w->mPosition = JPH::Vec3(0.0f, 0.0f, 1.0f);
	w->mRadius = 0.35f;
	w->mWidth = wheel_width;
	w->mSuspensionMinLength = 0.0f;
	w->mSuspensionMaxLength = 0.125f;
	w->mSuspensionSpring.mFrequency = 1.5f;
	w->mSuspensionDirection = JPH::Quat::sEulerAngles(JPH::Vec3(-0.34, 0.0, 0.0)) * JPH::Vec3(0.0f, -1.0f, 0.0f);
	w->mSteeringAxis = -w->mSuspensionDirection;
	vehicle.mWheels.push_back(w);

	auto body = space->try_get_jolt_body(body_impl->get_jolt_id());

	m_vehicle_constraint = new JPH::VehicleConstraint(*body, vehicle);

	JPH::VehicleConstraint::CombineFunction combine_function = [this](uint32_t, float &ioLongitudinalFriction, float &ioLateralFriction, const JPH::Body &inBody2, const JPH::SubShapeID &)
	{
		float body_friction = inBody2.GetFriction();
		ioLongitudinalFriction = sqrt(ioLongitudinalFriction * body_friction * this->friction_factor);
		ioLateralFriction = sqrt(ioLateralFriction * body_friction * this->friction_factor);
	};

	m_vehicle_constraint->SetVehicleCollisionTester(new JPH::VehicleCollisionTesterCastCylinder(body->GetObjectLayer(), 0.2));
	m_vehicle_constraint->SetCombineFriction(combine_function);

#ifdef JPH_DEBUG_RENDERER
	static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController())->SetRPMMeter(JPH::Vec3(0, 2, 0), 0.5f);
#endif // JPH_DEBUG_RENDERER

	m_vehicle_constraint->SetNumPositionStepsOverride(20);
	m_vehicle_constraint->SetNumVelocityStepsOverride(6);

	system.AddConstraint(m_vehicle_constraint);
	system.AddStepListener(m_vehicle_constraint);

}

void JoltKettenkrad::_space_changed(const RID &p_new_space) {
	if (p_new_space.is_valid()) {
		init_vehicle();
	} else {
		deinit_vehicle();
	}
}

JoltKettenkrad::~JoltKettenkrad() {
	deinit_vehicle();
}
