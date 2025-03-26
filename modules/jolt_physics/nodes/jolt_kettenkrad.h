#pragma once

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/TrackedVehicleController.h"
#include "Jolt/Physics/Vehicle/VehicleConstraint.h"
#include "scene/3d/physics/rigid_body_3d.h"

class JoltKettenkrad : public RigidBody3D {
	GDCLASS(JoltKettenkrad, RigidBody3D)
private:
	static void _bind_methods();
	JPH::VehicleConstraint* m_vehicle_constraint = nullptr;
	float friction_factor = 1.0;
	bool is_low_gear = false;
public:
	void init_vehicle();
	void deinit_vehicle();
	void set_driver_input(float forward, float left_ratio, float right_ratio, float brake);
	void set_front_steer_angle(float angle);
	void set_friction_factor(float factor);

	void set_use_low_gear(bool p_use_low_gear);
	bool get_use_low_gear();

	double get_engine_rpm_ratio();
	int get_trans_gear();
	bool is_switching_gear();

protected:
	void _space_changed(const RID &p_new_space) override;

public:
	int get_wheel_count() {
		if (m_vehicle_constraint == nullptr) {
			return 0;
		}

		return m_vehicle_constraint->GetWheels().size();
	}

	PackedVector3Array get_wheel_positions();
	PackedInt32Array get_wheel_contacts();
	PackedFloat32Array get_wheel_rotation_angles();
	PackedFloat32Array get_wheel_angular_velocities();
	PackedFloat32Array get_wheel_lateral_relative_velocities();

	Transform3D get_wheel_transform(uint wheel_idx);

	JoltKettenkrad();
	~JoltKettenkrad();
};
