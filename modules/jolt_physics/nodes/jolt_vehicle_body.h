#pragma once

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Vehicle/MotorcycleController.h"
#include "Jolt/Physics/Vehicle/TrackedVehicleController.h"
#include "Jolt/Physics/Vehicle/VehicleConstraint.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"
#include "scene/3d/physics/rigid_body_3d.h"
#include "scene/property_list_helper.h"
#include "scene/resources/mesh.h"

class JoltVehicleBodyTracked;
class JoltVehicleBodyWheeled;

#define SETTINGS_SETGET_F(mPropertyName, m_godot_name) \
	void set_##m_godot_name(float m_godot_name) {      \
		settings.mPropertyName = m_godot_name;         \
		emit_changed();                                \
	}                                                  \
	float get_##m_godot_name() const {                 \
		return settings.mPropertyName;                 \
	}

void bake_linear_curve(const Ref<Curve> &godot_curve,
		JPH::LinearCurve &jolt_curve,
		const int bake_resolution = 16,
		const float default_value = 1.0f);

class JoltVehicleEngineSettings : public Resource {
	GDCLASS(JoltVehicleEngineSettings, Resource);

private:
	JPH::VehicleEngineSettings settings;
	Ref<Curve> torque_curve;
	bool _torque_curve_dirty = true;
	void _on_torque_curve_changed() {
		_torque_curve_dirty = true;
		emit_changed();
	}

protected:
	static void _bind_methods();

public:
	SETTINGS_SETGET_F(mMaxTorque, max_torque);
	SETTINGS_SETGET_F(mMinRPM, min_rpm);
	SETTINGS_SETGET_F(mMaxRPM, max_rpm);
	SETTINGS_SETGET_F(mInertia, inertia);
	SETTINGS_SETGET_F(mAngularDamping, angular_damping);

	void set_torque_curve(const Ref<Curve> &p_curve) {
		if (torque_curve.is_valid()) {
			torque_curve->disconnect_changed(callable_mp(this, &JoltVehicleEngineSettings::_on_torque_curve_changed));
		}
		torque_curve = p_curve;
		if (p_curve.is_valid()) {
			p_curve->connect_changed(callable_mp(this, &JoltVehicleEngineSettings::_on_torque_curve_changed));
		}
		_torque_curve_dirty = true;
		emit_changed();
	}
	Ref<Curve> get_torque_curve() const { return torque_curve; }

	void write_settings(JPH::VehicleEngineSettings *r_settings) {
		if (r_settings == nullptr) {
			return;
		}
		if (_torque_curve_dirty) {
			bake_linear_curve(torque_curve, settings.mNormalizedTorque);
		}

		*r_settings = settings;
	}
};

class JoltVehicleTransmissionSettings : public Resource {
	GDCLASS(JoltVehicleTransmissionSettings, Resource);

private:
	JPH::VehicleTransmissionSettings settings;

protected:
	static void _bind_methods();
	SETTINGS_SETGET_F(mSwitchTime, switch_time);
	SETTINGS_SETGET_F(mClutchReleaseTime, clutch_release_time);
	SETTINGS_SETGET_F(mSwitchLatency, switch_latency);
	SETTINGS_SETGET_F(mShiftUpRPM, shift_up_rpm);
	SETTINGS_SETGET_F(mShiftDownRPM, shift_down_rpm);
	SETTINGS_SETGET_F(mClutchStrength, clutch_strength);

	void set_manual(bool p_manual) {
		if (p_manual) {
			settings.mMode = JPH::ETransmissionMode::Manual;
		} else {
			settings.mMode = JPH::ETransmissionMode::Auto;
		}
	}
	bool get_manual() {
		return settings.mMode == JPH::ETransmissionMode::Manual;
	}

	void set_gear_ratios(const TypedArray<float> &gears) {
		settings.mGearRatios.resize(gears.size());
		for (int i = 0; i < gears.size(); i++) {
			settings.mGearRatios[i] = gears[i];
		}
	}

	TypedArray<float> get_gear_ratios() const {
		TypedArray<float> gears;
		gears.resize(settings.mGearRatios.size());
		for (uint32_t i = 0; i < settings.mGearRatios.size(); i++) {
			gears[i] = settings.mGearRatios[i];
		}
		return gears;
	}

	void set_reverse_gear_ratios(const TypedArray<float> &gears) {
		settings.mReverseGearRatios.resize(gears.size());
		for (int i = 0; i < gears.size(); i++) {
			settings.mReverseGearRatios[i] = gears[i];
		}
	}

	TypedArray<float> get_reverse_gear_ratios() const {
		TypedArray<float> gears;
		gears.resize(settings.mReverseGearRatios.size());
		for (uint32_t i = 0; i < settings.mReverseGearRatios.size(); i++) {
			gears[i] = settings.mReverseGearRatios[i];
		}
		return gears;
	}

public:
	void write_settings(JPH::VehicleTransmissionSettings *r_settings) {
		if (r_settings == nullptr) {
			return;
		}
		*r_settings = settings;
	}
};

class JoltVehicleWheelBase : public Node3D {
	GDCLASS(JoltVehicleWheelBase, Node3D)
	friend class JoltVehicleBody;
	friend class JoltVehicleBodyWheeled;

private:
	RID debug_draw_instance;
	Ref<ArrayMesh> debug_draw_mesh;
	Ref<Material> debug_draw_material;
	Ref<ArrayMesh> debug_get_mesh();
	Ref<Material> debug_get_material();

	void _update_debug_mesh() {
		_settings_changed = false;
		debug_draw_mesh.unref();
		if (debug_draw_instance.is_valid()) {
			RS::get_singleton()->instance_set_base(debug_draw_instance, debug_get_mesh()->get_rid());
		}
	}

	bool _settings_changed = false;

	JPH::WheelSettings *get_settings() const { return settings; }

protected:
	static void _bind_methods();
	void _notification(int p_what);

	int wheel_index = -1;
	RID wheel_physics_space;

	friend class JoltVehicleBodyTracked;
	JPH::Ref<JPH::VehicleConstraint> parent_constraint;

	// May be JPH::WheelSettingsTV or JPH::WheelSettingsWV
	JPH::Ref<JPH::WheelSettings> settings;

	Transform3D m_wheel_transform;
	float m_angular_velocity = 0.0f;
	float m_rotation_angle = 0.0f;

	bool m_has_contact = false;
	Vector3 m_contact_position;
	Vector3 m_contact_normal;
	Vector3 m_contact_slip_velocity;
	RID m_contact_rid;
	int m_contact_shape_index = 0;

public:
	JoltVehicleWheelBase() {
		// TODO only draw debug if debug draw is enabled.
		set_notify_transform(true);

		// This we keep enabled so the wheel setting updates if we move the node.
		set_notify_local_transform(true);
	}

	void update_settings() {
		if (_settings_changed) {
			return;
		}
		_settings_changed = true;
		if (Engine::get_singleton()->is_editor_hint()) {
			_update_debug_mesh();
		} else if (is_inside_tree()) {
			callable_mp(this, &JoltVehicleWheelBase::_update_debug_mesh).call_deferred();
		}
	}

#define SETGET_VEHICLEWHEEL_F(jolt_property, godot_property) \
	void set_##godot_property(float p_##godot_property) {    \
		get_settings()->jolt_property = p_##godot_property;  \
		update_settings();                                   \
	}                                                        \
	float get_##godot_property() const {                     \
		return get_settings()->jolt_property;                \
	}

	SETGET_VEHICLEWHEEL_F(mSuspensionMinLength, suspension_min_length);
	SETGET_VEHICLEWHEEL_F(mSuspensionMaxLength, suspension_max_length);

	void set_suspension_damping(float damping) {
		settings->mSuspensionSpring.mDamping = damping;
		update_settings();
	}
	float get_suspension_damping() { return settings->mSuspensionSpring.mDamping; }

	void set_suspension_frequency(float frequency) {
		settings->mSuspensionSpring.mFrequency = frequency;
		update_settings();
	}
	float get_suspension_frequency() { return settings->mSuspensionSpring.mFrequency; }

	SETGET_VEHICLEWHEEL_F(mRadius, radius);
	SETGET_VEHICLEWHEEL_F(mWidth, width);

	Transform3D get_wheel_transform() {
		return m_wheel_transform;
	}
	float get_wheel_angular_velocity() {
		return m_angular_velocity;
	}
	float get_wheel_rotation_angle() {
		return m_rotation_angle;
	}
	bool get_wheel_has_contact() {
		return m_has_contact;
	}
	Vector3 get_wheel_contact_position() {
		return m_contact_position;
	}
	Vector3 get_wheel_contact_normal() {
		return m_contact_normal;
	}
	Vector3 get_wheel_contact_slip_velocity() {
		return m_contact_slip_velocity;
	}
	Object *get_wheel_contact_object() {
		ObjectID id = PhysicsServer3D::get_singleton()->body_get_object_instance_id(m_contact_rid);
		return ObjectDB::get_instance(id);
	}
	RID get_wheel_contact_rid() {
		return m_contact_rid;
	}
	int get_wheel_contact_shape() {
		return m_contact_shape_index;
	}

	void set_steer_angle(float p_angle) {
		ERR_FAIL_NULL(parent_constraint);
		ERR_FAIL_COND(wheel_index == -1);
		parent_constraint->GetWheel(wheel_index)->SetSteerAngle(p_angle);
	}

	float get_steer_angle() {
		ERR_FAIL_NULL_V(parent_constraint, 0.0);
		ERR_FAIL_COND_V(wheel_index == -1, 0.0);

		return parent_constraint->GetWheel(wheel_index)->GetSteerAngle();
	}

	void update_wheel_runtime(JPH::VehicleConstraint *p_constraint, PhysicsDirectSpaceState3D *p_space_state);
};

class JoltVehicleWheel : public JoltVehicleWheelBase {
	GDCLASS(JoltVehicleWheel, JoltVehicleWheelBase)
	friend class JoltVehicleBodyWheeled;
	JoltVehicleBodyWheeled *body = nullptr;
	inline JPH::WheelSettingsWV *get_settings() const { return static_cast<JPH::WheelSettingsWV *>(settings.GetPtr()); }

private:
	Ref<Curve> longitudinal_friction;
	Ref<Curve> lateral_friction;
	void _on_longitudinal_friction_changed() {
		JPH::LinearCurve &curve = get_settings()->mLongitudinalFriction;
		bake_linear_curve(longitudinal_friction, curve);
		update_settings();
	}
	void _on_lateral_friction_changed() {
		JPH::LinearCurve &curve = get_settings()->mLateralFriction;
		bake_linear_curve(lateral_friction, curve);
		update_settings();
	}

protected:
	static void _bind_methods();
	JoltVehicleWheel() { settings = new JPH::WheelSettingsWV; }

	void _notification(int p_what);

	SETGET_VEHICLEWHEEL_F(mInertia, inertia);
	SETGET_VEHICLEWHEEL_F(mAngularDamping, angular_damping);
	SETGET_VEHICLEWHEEL_F(mMaxSteerAngle, max_steer_angle);
	SETGET_VEHICLEWHEEL_F(mMaxBrakeTorque, max_brake_torque);
	SETGET_VEHICLEWHEEL_F(mMaxHandBrakeTorque, max_handbrake_torque);

	void set_longitudinal_friction(const Ref<Curve> &p_curve) {
		if (longitudinal_friction.is_valid()) {
			longitudinal_friction->disconnect_changed(callable_mp(this, &JoltVehicleWheel::_on_longitudinal_friction_changed));
		}
		longitudinal_friction = p_curve;
		if (p_curve.is_valid()) {
			p_curve->connect_changed(callable_mp(this, &JoltVehicleWheel::_on_longitudinal_friction_changed));
		}
		_on_longitudinal_friction_changed();
	}
	Ref<Curve> get_longitudinal_friction() const { return longitudinal_friction; }

	void set_lateral_friction(const Ref<Curve> &p_curve) {
		if (lateral_friction.is_valid()) {
			lateral_friction->disconnect_changed(callable_mp(this, &JoltVehicleWheel::_on_lateral_friction_changed));
		}
		lateral_friction = p_curve;
		if (p_curve.is_valid()) {
			p_curve->connect_changed(callable_mp(this, &JoltVehicleWheel::_on_lateral_friction_changed));
		}
		_on_lateral_friction_changed();
	}
	Ref<Curve> get_lateral_friction() const { return lateral_friction; }
};

class JoltVehicleWheelTracked : public JoltVehicleWheelBase {
	GDCLASS(JoltVehicleWheelTracked, JoltVehicleWheelBase)
	friend class JoltVehicleBodyTracked;
	JoltVehicleBodyTracked *body;

public:
	enum TrackSide : int32_t {
		TRACK_SIDE_NONE = -1,
		TRACK_SIDE_LEFT = 0,
		TRACK_SIDE_RIGHT = 1,
	};

private:
	TrackSide track_side = TRACK_SIDE_NONE;
	bool is_driven_wheel = false;
	inline JPH::WheelSettingsTV *get_settings() const { return static_cast<JPH::WheelSettingsTV *>(settings.GetPtr()); }

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	JoltVehicleWheelTracked();
	PackedStringArray get_configuration_warnings() const override;
	void set_longitudinal_friction(float friction) {
		get_settings()->mLongitudinalFriction = friction;
		update_settings();
	}
	float get_longitudinal_friction() { return get_settings()->mLongitudinalFriction; }

	void set_lateral_friction(float friction) {
		get_settings()->mLateralFriction = friction;
		update_settings();
	}
	float get_lateral_friction() { return get_settings()->mLateralFriction; }

	void set_track_side(TrackSide side) {
		track_side = side;
		update_settings();
	}
	TrackSide get_track_side() { return track_side; }
};

struct AntiRollBarNodeSettings {
	mutable NodePath left;
	mutable NodePath right;
	mutable float stiffness = 1000.0f;
};

class JoltVehicleBody : public RigidBody3D {
	GDCLASS(JoltVehicleBody, RigidBody3D)
private:

	Ref<JoltVehicleEngineSettings> engine;
	Ref<JoltVehicleTransmissionSettings> transmission;
	bool _is_dirty = false;

protected:
	static inline PropertyListHelper base_anti_rollbar_property_helper;
	PropertyListHelper anti_rollbar_property_helper;

	static void _bind_methods();
	void _notification(int p_what);

	virtual JPH::VehicleEngine *runtime_get_engine() = 0;
	virtual JPH::VehicleTransmission *runtime_get_transmission() = 0;

	void _init_wheels();
	void _finalize_constraint();

	void _wheels_changed() {
		_is_dirty = true;
	}

	Vector<AntiRollBarNodeSettings> anti_roll_bars;
	JPH::Ref<JPH::VehicleConstraint> m_vehicle_constraint = nullptr;
	JPH::VehicleConstraintSettings m_constraint_settings;
	Vector<JoltVehicleWheelBase *> child_wheels;

	void set_engine(const Ref<JoltVehicleEngineSettings> &p_engine) {
		if (engine.is_valid()) {
			engine->disconnect_changed(callable_mp(this, &JoltVehicleBody::_engine_changed));
		}
		engine = p_engine;
		p_engine->connect_changed(callable_mp(this, &JoltVehicleBody::_engine_changed));
		_engine_changed();
	}
	Ref<JoltVehicleEngineSettings> get_engine() { return engine; }

	void set_transmission(const Ref<JoltVehicleTransmissionSettings> &p_transmission) {
		transmission = p_transmission;
		_transmission_changed();
	}
	Ref<JoltVehicleTransmissionSettings> get_transmission() { return transmission; }

	float get_engine_rpm() {
		ERR_FAIL_NULL_V(m_vehicle_constraint, 0.0);
		JPH::VehicleEngine *r_engine = runtime_get_engine();
		return r_engine->GetCurrentRPM();
	}

	void set_transmission_gear(int p_gear, float p_clutch) {
		ERR_FAIL_NULL(m_vehicle_constraint);
		JPH::VehicleTransmission *r_transmission = runtime_get_transmission();
		CLAMP(p_gear, -(int(r_transmission->mReverseGearRatios.size()) - 1), int(r_transmission->mGearRatios.size()) - 1);
		return r_transmission->Set(p_gear, p_clutch);
	}

	int get_transmission_gear() {
		ERR_FAIL_NULL_V(m_vehicle_constraint, 0);
		JPH::VehicleTransmission *r_transmission = runtime_get_transmission();

		if (r_transmission->IsSwitchingGear()) {
			return 0;
		} else {
			return r_transmission->GetCurrentGear();
		}
	}

	float get_transmission_clutch() {
		ERR_FAIL_NULL_V(m_vehicle_constraint, 0.0);
		return runtime_get_transmission()->GetClutchFriction();
	}

	bool _set(const StringName &p_name, const Variant &p_value) { return anti_rollbar_property_helper.property_set_value(p_name, p_value); }
	bool _get(const StringName &p_name, Variant &r_ret) const { return anti_rollbar_property_helper.property_get_value(p_name, r_ret); }
	void _get_property_list(List<PropertyInfo> *p_list) const { anti_rollbar_property_helper.get_property_list(p_list); }
	bool _property_can_revert(const StringName &p_name) const { return anti_rollbar_property_helper.property_can_revert(p_name); }
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const { return anti_rollbar_property_helper.property_get_revert(p_name, r_property); }

	void _space_changed(const RID &p_new_space) override;
	void _body_state_changed(PhysicsDirectBodyState3D *p_state) override;

	virtual void init_constraint(bool keep_state = false) = 0;
	virtual void deinit_constraint();

	void set_max_pitch_roll_angle(float p_pitch_roll_angle) {
		m_constraint_settings.mMaxPitchRollAngle = p_pitch_roll_angle;
		if (m_vehicle_constraint) {
			m_vehicle_constraint->SetMaxPitchRollAngle(p_pitch_roll_angle);
		}
	}

	float get_max_pitch_roll_angle() {
		return m_constraint_settings.mMaxPitchRollAngle;
	}

	int add_anti_roll_bar();

	void set_anti_roll_bar_count(int p_count);
	int get_anti_roll_bar_count() const;
	void remove_anti_roll_bar(int p_idx);

	void move_anti_roll_bar(int p_from_idx, int p_to_idx);

	void set_anti_roll_bar_left_wheel(int p_idx, NodePath left_wheel);
	NodePath get_anti_roll_bar_left_wheel(int p_idx);

	void set_anti_roll_bar_right_wheel(int p_idx, NodePath right_wheel);
	NodePath get_anti_roll_bar_right_wheel(int p_idx);

	void set_anti_roll_bar_stiffness(int p_idx, float stiffness);
	float get_anti_roll_bar_stiffness(int p_idx);

	void init_anti_roll_bars();

public:
	void _engine_changed() {
		if (unlikely(get_engine().is_null())) {
			// Resource was just cleared - revert to the engine settings resource defaults?
			JoltVehicleEngineSettings().write_settings(runtime_get_engine());
		} else {
			get_engine()->write_settings(runtime_get_engine());
		}
	}
	void _transmission_changed() {
		if (unlikely(get_transmission().is_null())) {
			// Resource was just cleared - revert to the transmission settings resource defaults?
			JoltVehicleTransmissionSettings().write_settings(runtime_get_transmission());
		} else {
			get_transmission()->write_settings(runtime_get_transmission());
		}
	}
	JoltVehicleBody();
};

struct DifferentialSettings {
	mutable NodePath left_wheel;
	mutable NodePath right_wheel;
	mutable float differential_ratio = 3.42f;
	mutable float left_right_split = 0.5f;
	mutable float limited_slip_ratio = 1.4f;
	mutable float engine_torque_ratio = 1.0f;
};

class JoltVehicleBodyWheeled : public JoltVehicleBody {
	GDCLASS(JoltVehicleBodyWheeled, JoltVehicleBody)
	friend class JoltVehicleWheel;
	friend class JoltVehicleWheelBase;

	Vector<DifferentialSettings> diffs;

	inline JPH::WheeledVehicleController *get_controller() {
		if (m_vehicle_constraint == nullptr) {
			return nullptr;
		}

		return static_cast<JPH::WheeledVehicleController *>(m_vehicle_constraint->GetController());
	}

	inline JPH::WheeledVehicleControllerSettings *get_controller_settings() {
		return static_cast<JPH::WheeledVehicleControllerSettings *>(m_constraint_settings.mController.GetPtr());
	}

	inline JPH::Array<JPH::VehicleDifferentialSettings> &get_differential_settings() {
		return get_controller_settings()->mDifferentials;
	}

protected:
	static inline PropertyListHelper base_differential_helper;
	PropertyListHelper differential_helper;

	static void _bind_methods();
	bool _set(const StringName &p_name, const Variant &p_value) {
		return differential_helper.property_set_value(p_name, p_value); // || anti_rollbar_property_helper.property_set_value(p_name, p_value);
	}
	bool _get(const StringName &p_name, Variant &r_ret) const {
		return differential_helper.property_get_value(p_name, r_ret); // || anti_rollbar_property_helper.property_get_value(p_name, r_ret);
	}
	void _get_property_list(List<PropertyInfo> *p_list) const {
		differential_helper.get_property_list(p_list);
//		anti_rollbar_property_helper.get_property_list(p_list);
	}
	bool _property_can_revert(const StringName &p_name) const {
		return differential_helper.property_can_revert(p_name); // || anti_rollbar_property_helper.property_can_revert(p_name);
	}
	bool _property_get_revert(const StringName &p_name, Variant &r_property) const {
		return differential_helper.property_get_revert(p_name, r_property); // || anti_rollbar_property_helper.property_get_revert(p_name, r_property);
	}

	JPH::VehicleEngine *runtime_get_engine() override {
		auto *controller = get_controller();
		if (controller == nullptr) {
			return nullptr;
		}
		return &controller->GetEngine();
	}

	JPH::VehicleTransmission *runtime_get_transmission() override {
		auto *controller = get_controller();
		if (controller == nullptr) {
			return nullptr;
		}
		return &controller->GetTransmission();
	}

	void init_constraint(bool keep_state = false) override;

	JoltVehicleBodyWheeled() {
		m_constraint_settings.mController = new JPH::WheeledVehicleControllerSettings;
		differential_helper.setup_for_instance(base_differential_helper, this);
	}

	void set_driver_input(float p_forward, float p_right, float p_brake, float p_handbrake = 0.0) {
		get_controller()->SetDriverInput(p_forward, p_right, p_brake, p_handbrake);
	}

	int add_differential();
	void remove_differential(int p_idx);

	void set_differential_count(int p_count);
	int get_differential_count() const;

	void move_differential(int p_from_idx, int p_to_idx);

	void set_differential_left_wheel(int p_idx, NodePath left_wheel);
	NodePath get_differential_left_wheel(int p_idx);

	void set_differential_right_wheel(int p_idx, NodePath right_wheel);
	NodePath get_differential_right_wheel(int p_idx);

	void set_differential_ratio(int p_idx, float p_differential_ratio);
	float get_differential_ratio(int p_idx);

	void set_differential_left_right_split(int p_idx, float p_left_right_split);
	float get_differential_left_right_split(int p_idx);

	void set_differential_limited_slip_ratio(int p_idx, float p_limited_slip_ratio);
	float get_differential_limited_slip_ratio(int p_idx);

	void set_differential_engine_torque_ratio(int p_idx, float p_engine_torque_ratio);
	float get_differential_engine_torque_ratio(int p_idx);
};

class JoltVehicleBodyMotorcycle : public JoltVehicleBodyWheeled {
	GDCLASS(JoltVehicleBodyMotorcycle, JoltVehicleBodyWheeled)

	JoltVehicleBodyMotorcycle() {
		m_constraint_settings.mController = new JPH::MotorcycleControllerSettings;

		auto *controller = static_cast<JPH::MotorcycleControllerSettings *>(m_constraint_settings.mController.GetPtr());

		auto diff = JPH::VehicleDifferentialSettings();
		diff.mLeftWheel = 1;
		diff.mLeftRightSplit = 0;
		controller->mDifferentials.push_back(diff);
	}
};

class JoltVehicleBodyTracked : public JoltVehicleBody {
	GDCLASS(JoltVehicleBodyTracked, JoltVehicleBody)
	friend class JoltVehicleWheelTracked;

private:
	static void _bind_methods();
	JPH::TrackedVehicleController *get_controller() {
		if (m_vehicle_constraint == nullptr) {
			return nullptr;
		}
		return static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController());
	}
	NodePath driven_wheels[2];

protected:
	JPH::VehicleEngine *runtime_get_engine() override {
		auto *controller = get_controller();
		if (controller == nullptr) {
			return nullptr;
		}
		return &controller->GetEngine();
	}

	JPH::VehicleTransmission *runtime_get_transmission() override {
		auto *controller = get_controller();
		if (controller == nullptr) {
			return nullptr;
		}
		return &controller->GetTransmission();
	}

	JoltVehicleBodyTracked();
	void init_constraint(bool keep_state = false) override;

	void set_track_driven_wheel(int p_track_side, NodePath p_wheel) {
		driven_wheels[p_track_side] = p_wheel;

		if (auto *controller = get_controller()) {
			auto *wheel_node = dynamic_cast<JoltVehicleWheelTracked *>(get_node_or_null(driven_wheels[p_track_side]));
			controller->GetTracks()[p_track_side].mDrivenWheel = wheel_node->wheel_index;
		}
	}
	NodePath get_track_driven_wheel(int p_track_side) {
		return driven_wheels[p_track_side];
	}

#define SETGET_TRACK_F(godot_property, jolt_property)                                                                      \
	void set_##godot_property(float p_##godot_property) {                                                                  \
		auto *settings = static_cast<JPH::TrackedVehicleControllerSettings *>(m_constraint_settings.mController.GetPtr()); \
		settings->mTracks[0].jolt_property = p_##godot_property;                                                           \
		settings->mTracks[1].jolt_property = p_##godot_property;                                                           \
                                                                                                                           \
		if (auto *controller = get_controller()) {                                                                         \
			controller->GetTracks()[0].jolt_property = p_##godot_property;                                                 \
			controller->GetTracks()[1].jolt_property = p_##godot_property;                                                 \
		}                                                                                                                  \
	}                                                                                                                      \
                                                                                                                           \
	float get_##godot_property() {                                                                                         \
		auto *settings = static_cast<JPH::TrackedVehicleControllerSettings *>(m_constraint_settings.mController.GetPtr()); \
		return settings->mTracks[0].jolt_property;                                                                         \
	}

	SETGET_TRACK_F(track_inertia, mInertia)
	SETGET_TRACK_F(track_angular_damping, mAngularDamping)
	SETGET_TRACK_F(track_max_brake_torque, mMaxBrakeTorque)
	SETGET_TRACK_F(track_differential_ratio, mDifferentialRatio)
public:
	void set_driver_input(float forward, float left, float right, float brake);
};

VARIANT_ENUM_CAST(JoltVehicleWheelTracked::TrackSide);
