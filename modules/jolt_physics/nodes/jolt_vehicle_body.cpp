#include "jolt_vehicle_body.h"

#include "../jolt_physics_server_3d.h"
#include "../misc/jolt_type_conversions.h"
#include "../objects/jolt_body_3d.h"
#include "../spaces/jolt_space_3d.h"
#include "Jolt/Physics/Vehicle/TrackedVehicleController.h"
#include "Jolt/Physics/Vehicle/WheeledVehicleController.h"

#include "core/object/class_db.h"

void bake_linear_curve(const Ref<Curve> &godot_curve,
		JPH::LinearCurve &jolt_curve,
		const int bake_resolution,
		const float default_value
) {
	jolt_curve.Clear();
	if (godot_curve.is_valid()) {
		jolt_curve.Reserve(bake_resolution);

		for (int i = 0; i < bake_resolution; i++) {
			float x = float(i) / float(bake_resolution - 1);
			float y = godot_curve->sample(x);
			jolt_curve.AddPoint(x, y);
		}

	} else {
		jolt_curve.AddPoint(0.0f, default_value);
	}
}

Ref<ArrayMesh> JoltVehicleWheelBase::debug_get_mesh() {
	if (debug_draw_mesh.is_valid()) {
		return debug_draw_mesh;
	}

	Color debug_color = Color::named("LIGHT_BLUE");

	debug_draw_mesh.instantiate();

	// Get the whole ass debug mesh with all the bits attached :)
	Vector<Vector3> line_points;
	Vector<Color> line_colors;

	const int CIRCLE_RESOLUTION = 36;
	const int SPRING_RESOLUTION = 12;
	const int SPRING_TURNS = 4;

	Vector3 wheel_center = Vector3(0.0f, -settings->mSuspensionMaxLength, 0.0f);

	// Draw a line along the suspension axis to the wheel axle
	{
		line_points.push_back(Vector3());
		line_colors.push_back(debug_color);

		line_points.push_back(wheel_center);
		line_colors.push_back(debug_color);
	}

	// Draw a spring for fun :)
	float spring_radius = settings->mRadius * 0.2;
	for (int i = 0; i < (SPRING_RESOLUTION * SPRING_TURNS); i++) {
		float a1 = (float(i) / float(SPRING_RESOLUTION)) * Math::TAU;
		float a2 = (float(i+1) / float(SPRING_RESOLUTION)) * Math::TAU;

		float t1 = (float(i) / float((SPRING_RESOLUTION * SPRING_TURNS) + 1)) * -settings->mSuspensionMaxLength;
		float t2 = (float(i+1) / float((SPRING_RESOLUTION * SPRING_TURNS) + 1)) * -settings->mSuspensionMaxLength;

		line_points.push_back(Vector3(Math::sin(a1) * spring_radius, t1, Math::cos(a1) * spring_radius));
		line_colors.push_back(debug_color);

		line_points.push_back(Vector3(Math::sin(a2) * spring_radius, t2, Math::cos(a2) * spring_radius));
		line_colors.push_back(debug_color);
	}

	// Draw a wheel at the maximum extension
	for (int i = 0; i < CIRCLE_RESOLUTION; i++) {
		float ra = (float(i) / float(CIRCLE_RESOLUTION)) * Math::TAU;
		float rb = (float(i+1) / float(CIRCLE_RESOLUTION)) * Math::TAU;

		line_points.push_back(wheel_center + Vector3(0.0f, Math::sin(ra), Math::cos(ra)) * settings->mRadius);
		line_colors.push_back(debug_color);

		line_points.push_back(wheel_center + Vector3(0.0f, Math::sin(rb), Math::cos(rb)) * settings->mRadius);
		line_colors.push_back(debug_color);
	}

	Array lines_array;
	lines_array.resize(Mesh::ARRAY_MAX);
	lines_array[Mesh::ARRAY_VERTEX] = line_points;
	lines_array[Mesh::ARRAY_COLOR] = line_colors;

	debug_draw_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, lines_array);
	debug_draw_mesh->surface_set_material(0, debug_get_material());

	return debug_draw_mesh;
}

void JoltVehicleWheelBase::_notification(int p_what) {
	ERR_FAIL_NULL(RenderingServer::get_singleton());
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			if (Engine::get_singleton()->is_editor_hint()) {
				debug_draw_instance = RS::get_singleton()->instance_create();
				RS::get_singleton()->instance_set_scenario(debug_draw_instance, get_world_3d()->get_scenario());
				RS::get_singleton()->instance_set_base(debug_draw_instance, debug_get_mesh()->get_rid());
				RS::get_singleton()->instance_set_transform(debug_draw_instance, get_global_transform());
			}
		} break;
		case NOTIFICATION_EXIT_TREE: {
			if (Engine::get_singleton()->is_editor_hint()) {
				RS::get_singleton()->free_rid(debug_draw_instance);
			}
		} break;
		case NOTIFICATION_TRANSFORM_CHANGED: {
			if (Engine::get_singleton()->is_editor_hint()) {
				RS::get_singleton()->instance_set_transform(debug_draw_instance, get_global_transform());
			}
		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {
			settings->mPosition = to_jolt(get_position());
			settings->mWheelUp = to_jolt(get_basis().inverse()[1].normalized());

			// TODO allow alternate steering and suspension axes (e.g. wheel camber)
			settings->mSteeringAxis = settings->mWheelUp;
			settings->mSuspensionDirection = -settings->mWheelUp;

			settings->mWheelForward = -to_jolt(get_basis()[2]);
		} break;
	}
}

Ref<Material> JoltVehicleWheelBase::debug_get_material() {
	if (debug_draw_material.is_valid()) {
		return debug_draw_material;
	}

	Ref<StandardMaterial3D> m = memnew(StandardMaterial3D);
	m->set_albedo(Color(1.0, 1.0, 1.0));
	m->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
	m->set_transparency(BaseMaterial3D::TRANSPARENCY_ALPHA);
	m->set_render_priority(StandardMaterial3D::RENDER_PRIORITY_MIN + 1);
	m->set_cull_mode(StandardMaterial3D::CULL_BACK);
	m->set_flag(StandardMaterial3D::FLAG_DISABLE_FOG, true);
	m->set_flag(StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	m->set_flag(StandardMaterial3D::FLAG_SRGB_VERTEX_COLOR, true);
	m->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);

	debug_draw_material = m;

	return debug_draw_material;
}

#define STRINGIFY(a) #a
#define BIND_SETGET(cls, name)                                                      \
	ClassDB::bind_method(D_METHOD(STRINGIFY(set_##name), #name), &cls::set_##name); \
	ClassDB::bind_method(D_METHOD(STRINGIFY(get_##name)), &cls::get_##name)

void JoltVehicleEngineSettings::_bind_methods() {
	BIND_SETGET(JoltVehicleEngineSettings, max_torque);
	BIND_SETGET(JoltVehicleEngineSettings, min_rpm);
	BIND_SETGET(JoltVehicleEngineSettings, max_rpm);
	BIND_SETGET(JoltVehicleEngineSettings, inertia);
	BIND_SETGET(JoltVehicleEngineSettings, angular_damping);

	BIND_SETGET(JoltVehicleEngineSettings, torque_curve);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_torque", PROPERTY_HINT_RANGE, "0,500,or_greater,suffix:Nm"), "set_max_torque", "get_max_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_rpm", PROPERTY_HINT_RANGE, "0,10000,or_greater,suffix:rpm"), "set_min_rpm", "get_min_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_rpm", PROPERTY_HINT_RANGE, "0,10000,or_greater,suffix:rpm"), "set_max_rpm", "get_max_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "torque_curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_torque_curve", "get_torque_curve");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,1,or_greater,suffix:kgm^2"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damping", PROPERTY_HINT_RANGE, "0.0,1.0"), "set_angular_damping", "get_angular_damping");
}

void JoltVehicleTransmissionSettings::_bind_methods() {
	BIND_SETGET(JoltVehicleTransmissionSettings, switch_time);
	BIND_SETGET(JoltVehicleTransmissionSettings, clutch_release_time);
	BIND_SETGET(JoltVehicleTransmissionSettings, switch_latency);
	BIND_SETGET(JoltVehicleTransmissionSettings, shift_up_rpm);
	BIND_SETGET(JoltVehicleTransmissionSettings, shift_down_rpm);
	BIND_SETGET(JoltVehicleTransmissionSettings, clutch_strength);
	BIND_SETGET(JoltVehicleTransmissionSettings, manual);

	BIND_SETGET(JoltVehicleTransmissionSettings, gear_ratios);
	BIND_SETGET(JoltVehicleTransmissionSettings, reverse_gear_ratios);

	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "gear_ratios"), "set_gear_ratios", "get_gear_ratios");
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_FLOAT32_ARRAY, "reverse_gear_ratios"), "set_reverse_gear_ratios", "get_reverse_gear_ratios");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "clutch_strength", PROPERTY_HINT_RANGE, "0,10,or_greater,suffix:k m^2/s"), "set_clutch_strength", "get_clutch_strength");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "manual", PROPERTY_HINT_NONE), "set_manual", "get_manual");

	ADD_GROUP("Automatic Transmission", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "switch_time", PROPERTY_HINT_RANGE, "0.0,1.0,or_greater,suffix:s"), "set_switch_time", "get_switch_time");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "clutch_release_time", PROPERTY_HINT_RANGE, "0.0,1.0,or_greater,suffix:s"), "set_clutch_release_time", "get_clutch_release_time");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "switch_latency", PROPERTY_HINT_RANGE, "0.0,1.0,or_greater,suffix:s"), "set_switch_latency", "get_switch_latency");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shift_up_rpm", PROPERTY_HINT_RANGE, "0.0,10000,or_greater,suffix:rpm"), "set_shift_up_rpm", "get_shift_up_rpm");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "shift_down_rpm", PROPERTY_HINT_RANGE, "0.0,10000,or_greater,suffix:rpm"), "set_shift_down_rpm", "get_shift_down_rpm");
}

void JoltVehicleWheelBase::_bind_methods() {
	BIND_SETGET(JoltVehicleWheelBase, suspension_min_length);
	BIND_SETGET(JoltVehicleWheelBase, suspension_max_length);

	BIND_SETGET(JoltVehicleWheelBase, suspension_frequency);
	BIND_SETGET(JoltVehicleWheelBase, suspension_damping);

	BIND_SETGET(JoltVehicleWheelBase, radius);
	BIND_SETGET(JoltVehicleWheelBase, width);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, "0.001,10,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "width", PROPERTY_HINT_RANGE, "0.001,1,or_greater,suffix:m"), "set_width", "get_width");

	ADD_GROUP("Suspension", "suspension_");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_min_length", PROPERTY_HINT_RANGE, "0.0,10,or_greater,suffix:m"), "set_suspension_min_length", "get_suspension_min_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_max_length", PROPERTY_HINT_RANGE, "0.0,10,or_greater,suffix:m"), "set_suspension_max_length", "get_suspension_max_length");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_frequency", PROPERTY_HINT_NONE, "suffix:Hz"), "set_suspension_frequency", "get_suspension_frequency");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "suspension_damping", PROPERTY_HINT_RANGE, "0.0,1.0,or_greater"), "set_suspension_damping", "get_suspension_damping");

	ClassDB::bind_method(D_METHOD("get_wheel_transform"), &JoltVehicleWheelBase::get_wheel_transform);
	ClassDB::bind_method(D_METHOD("get_wheel_angular_velocity"), &JoltVehicleWheelBase::get_wheel_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_wheel_rotation_angle"), &JoltVehicleWheelBase::get_wheel_rotation_angle);

	ClassDB::bind_method(D_METHOD("get_wheel_has_contact"), &JoltVehicleWheelBase::get_wheel_has_contact);
	ClassDB::bind_method(D_METHOD("get_wheel_contact_position"), &JoltVehicleWheelBase::get_wheel_contact_position);
	ClassDB::bind_method(D_METHOD("get_wheel_contact_normal"), &JoltVehicleWheelBase::get_wheel_contact_normal);

	ClassDB::bind_method(D_METHOD("get_wheel_contact_slip_velocity"), &JoltVehicleWheelBase::get_wheel_contact_slip_velocity);

	ClassDB::bind_method(D_METHOD("get_wheel_contact_object"), &JoltVehicleWheelBase::get_wheel_contact_object);
	ClassDB::bind_method(D_METHOD("get_wheel_contact_rid"), &JoltVehicleWheelBase::get_wheel_contact_rid);
	ClassDB::bind_method(D_METHOD("get_wheel_contact_shape"), &JoltVehicleWheelBase::get_wheel_contact_shape);

	BIND_SETGET(JoltVehicleWheelBase, steer_angle);
}

void JoltVehicleWheelBase::update_wheel_runtime(JPH::VehicleConstraint *p_constraint, PhysicsDirectSpaceState3D * /* p_space_state */) {
	ERR_FAIL_COND_MSG(wheel_index < 0, "wheel_index invalid");

	JPH::Wheel *wheel = static_cast<JPH::Wheel *>(p_constraint->GetWheel(wheel_index));

	JPH::RMat44 wheel_tf = p_constraint->GetWheelLocalTransform(wheel_index, JPH::Vec3::sAxisX(), JPH::Vec3::sAxisY());
	m_wheel_transform = to_godot(wheel_tf);
	m_angular_velocity = wheel->GetAngularVelocity();
	m_rotation_angle = wheel->GetRotationAngle();

	m_has_contact = wheel->HasContact();
	if (m_has_contact) {
		m_contact_position = to_godot(wheel->GetContactPosition());
		m_contact_normal = to_godot(wheel->GetContactNormal());

		// Velocity of the contact point as though the wheel was not spinning.
		JPH::Vec3 contact_relative_velocity = p_constraint->GetVehicleBody()->GetPointVelocity(wheel->GetContactPosition()) - wheel->GetContactPointVelocity();
		// Relative velocity of the rim of the wheel, in world space, not including the motion of the body.
		JPH::Vec3 wheel_contact_velocity = wheel->GetAngularVelocity() * wheel->GetSettings()->mRadius * wheel->GetContactLongitudinal();

		// Subtract the two velocities to get the relative motion of the surface of the wheel against the contact surface ("slip").
		m_contact_slip_velocity = to_godot(contact_relative_velocity - wheel_contact_velocity);

		JPH::BodyID body_id = wheel->GetContactBodyID();
		auto *server = JoltPhysicsServer3D::get_singleton();
		ERR_FAIL_NULL(server);
		auto *space = server->get_space(wheel_physics_space);
		auto body = space->try_get_body(body_id);
		ERR_FAIL_NULL(body);
		auto jolt_object = space->try_get_object(body_id);
		ERR_FAIL_NULL(jolt_object);

		m_contact_rid = body->get_rid();
		m_contact_shape_index = 0;
		if (const JoltShapedObject3D *shaped_object = jolt_object->as_shaped()) {
			const int shape_index = shaped_object->find_shape_index(wheel->GetContactSubShapeID());
			ERR_FAIL_COND(shape_index == -1);
			m_contact_shape_index = shape_index;
			// TODO copy JoltPhysicsDirectSpaceState3D::_try_get_face_index to get triangle index if needed.
		}
	} else {
		m_contact_position = Vector3();
		m_contact_normal = Vector3();
		m_contact_slip_velocity = Vector3();
		m_contact_rid = RID();
		m_contact_shape_index = 0;
	}
}

void JoltVehicleWheel::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			JoltVehicleBodyWheeled *_body = Object::cast_to<JoltVehicleBodyWheeled>(get_parent());
			if (!_body) { return; }

			body = _body;
			wheel_index = -1;
			settings->mPosition = to_jolt(get_position());
			body->child_wheels.push_back(this);
			body->_wheels_changed();
		} break;
		case NOTIFICATION_EXIT_TREE: {
			if (!body) {return;}

			body->child_wheels.erase(this);
			wheel_index = -1;
			body->_wheels_changed();
			body = nullptr;
		} break;
	}
}

void JoltVehicleWheel::_bind_methods() {
	BIND_SETGET(JoltVehicleWheel, inertia);
	BIND_SETGET(JoltVehicleWheel, angular_damping);
	BIND_SETGET(JoltVehicleWheel, max_steer_angle);
	BIND_SETGET(JoltVehicleWheel, max_brake_torque);
	BIND_SETGET(JoltVehicleWheel, max_handbrake_torque);

//	BIND_SETGET(JoltVehicleWheel, longitudinal_friction);
//	BIND_SETGET(JoltVehicleWheel, lateral_friction);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "inertia", PROPERTY_HINT_RANGE, "0,10,or_greater,hide_slider,suffix:kgm^2"), "set_inertia", "get_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "angular_damping", PROPERTY_HINT_RANGE, "0,1"), "set_angular_damping", "get_angular_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_steer_angle", PROPERTY_HINT_RANGE, "0,90,or_greater,radians_as_degrees"), "set_max_steer_angle", "get_max_steer_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_brake_torque", PROPERTY_HINT_RANGE, "0,500,or_greater,suffix:Nm"), "set_max_brake_torque", "get_max_brake_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_handbrake_torque", PROPERTY_HINT_RANGE, "0,500,or_greater,suffix:Nm"), "set_max_handbrake_torque", "get_max_handbrake_torque");

//	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "longitudinal_friction", PROPERTY_HINT_RESOURCE_TYPE, "Curve", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_longitudinal_friction", "get_longitudinal_friction");
//	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "lateral_friction", PROPERTY_HINT_RESOURCE_TYPE, "Curve", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_lateral_friction", "get_lateral_friction");
}

void JoltVehicleWheelTracked::_bind_methods() {
	BIND_SETGET(JoltVehicleWheelTracked, longitudinal_friction);
	BIND_SETGET(JoltVehicleWheelTracked, lateral_friction);
	BIND_SETGET(JoltVehicleWheelTracked, track_side);

	BIND_ENUM_CONSTANT(TRACK_SIDE_NONE)
	BIND_ENUM_CONSTANT(TRACK_SIDE_LEFT)
	BIND_ENUM_CONSTANT(TRACK_SIDE_RIGHT)

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "longitudinal_friction"), "set_longitudinal_friction", "get_longitudinal_friction");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lateral_friction"), "set_lateral_friction", "get_lateral_friction");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "track_side", PROPERTY_HINT_ENUM, "None:-1,Left:0,Right:1"), "set_track_side", "get_track_side");
}

JoltVehicleWheelTracked::JoltVehicleWheelTracked() {
	body = nullptr;
	settings = new JPH::WheelSettingsTV;
}

void JoltVehicleWheelTracked::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			JoltVehicleBodyTracked *_body = Object::cast_to<JoltVehicleBodyTracked>(get_parent());
			if (!_body) {
				return;
			}

			body = _body;
			wheel_index = -1;
			settings->mPosition = to_jolt(get_position());
			body->child_wheels.push_back(this);
			body->_wheels_changed();
		} break;
		case NOTIFICATION_EXIT_TREE: {
			if (!body) {return;}

			body->child_wheels.erase(this);
			body->_wheels_changed();
			wheel_index = -1;
			body = nullptr;
		} break;
		case NOTIFICATION_TRANSFORM_CHANGED: {
		} break;
	}
}
PackedStringArray JoltVehicleWheelTracked::get_configuration_warnings() const {
	auto warnings = Node::get_configuration_warnings();

	JoltVehicleBodyTracked *_body = Object::cast_to<JoltVehicleBodyTracked>(get_parent());

	if (!_body) {
		warnings.push_back("Node must be a child of JoltVehicleBodyTracked. Wheel will be disabled.");
	}

	return warnings;
}

JoltVehicleBody::JoltVehicleBody() {
	// -Z is forward in godot land.
	m_constraint_settings.mForward = JPH::Vec3(0.0f, 0.0f, -1.0f);

	anti_rollbar_property_helper.setup_for_instance(base_anti_rollbar_property_helper, this);

	PhysicsServer3D::get_singleton()->body_set_state_sync_callback(get_rid(), callable_mp(this, &JoltVehicleBodyTracked::_body_state_changed));
}

void JoltVehicleBody::_bind_methods() {
	BIND_SETGET(JoltVehicleBody, max_pitch_roll_angle);

	BIND_SETGET(JoltVehicleBody, engine);
	BIND_SETGET(JoltVehicleBody, transmission);

	ClassDB::bind_method(D_METHOD("add_anti_roll_bar"), &JoltVehicleBody::add_anti_roll_bar);

	ClassDB::bind_method(D_METHOD("set_anti_roll_bar_count", "count"), &JoltVehicleBody::set_anti_roll_bar_count);
	ClassDB::bind_method(D_METHOD("get_anti_roll_bar_count"), &JoltVehicleBody::get_anti_roll_bar_count);
	ClassDB::bind_method(D_METHOD("remove_anti_roll_bar", "index"), &JoltVehicleBody::remove_anti_roll_bar);

	ClassDB::bind_method(D_METHOD("move_anti_roll_bar", "from_index", "to_index"), &JoltVehicleBody::move_anti_roll_bar);

	ClassDB::bind_method(D_METHOD("set_anti_roll_bar_left_wheel", "index", "wheel"), &JoltVehicleBody::set_anti_roll_bar_left_wheel);
	ClassDB::bind_method(D_METHOD("get_anti_roll_bar_left_wheel", "index"), &JoltVehicleBody::get_anti_roll_bar_left_wheel);

	ClassDB::bind_method(D_METHOD("set_anti_roll_bar_right_wheel", "index", "wheel"), &JoltVehicleBody::set_anti_roll_bar_right_wheel);
	ClassDB::bind_method(D_METHOD("get_anti_roll_bar_right_wheel", "index"), &JoltVehicleBody::get_anti_roll_bar_right_wheel);

	ClassDB::bind_method(D_METHOD("set_anti_roll_bar_stiffness", "index", "stiffness"), &JoltVehicleBody::set_anti_roll_bar_stiffness);
	ClassDB::bind_method(D_METHOD("get_anti_roll_bar_stiffness", "index"), &JoltVehicleBody::get_anti_roll_bar_stiffness);

	ClassDB::bind_method(D_METHOD("get_engine_rpm"), &JoltVehicleBody::get_engine_rpm);
	ClassDB::bind_method(D_METHOD("set_transmission_gear", "gear", "clutch"), &JoltVehicleBody::set_transmission_gear);
	ClassDB::bind_method(D_METHOD("get_transmission_gear"), &JoltVehicleBody::get_transmission_gear);
	ClassDB::bind_method(D_METHOD("get_transmission_clutch"), &JoltVehicleBody::get_transmission_clutch);

	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "engine", PROPERTY_HINT_RESOURCE_TYPE, "JoltVehicleEngineSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_engine", "get_engine");
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "transmission", PROPERTY_HINT_RESOURCE_TYPE, "JoltVehicleTransmissionSettings", PROPERTY_USAGE_DEFAULT | PROPERTY_USAGE_EDITOR_INSTANTIATE_OBJECT), "set_transmission", "get_transmission");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_pitch_roll_angle", PROPERTY_HINT_RANGE, "0,180,radians_as_degrees"), "set_max_pitch_roll_angle", "get_max_pitch_roll_angle");

	ADD_ARRAY_COUNT("Anti-Roll Bars", "anti_roll_bar_count", "set_anti_roll_bar_count", "get_anti_roll_bar_count", "anti_roll_bar_");

	AntiRollBarNodeSettings defaults;

	base_anti_rollbar_property_helper.set_prefix("anti_roll_bar_");
	base_anti_rollbar_property_helper.set_array_length_getter(&JoltVehicleBody::get_anti_roll_bar_count);
	base_anti_rollbar_property_helper.register_property(PropertyInfo(Variant::NODE_PATH, "left_wheel", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "JoltVehicleWheelBase"), defaults.left, &JoltVehicleBody::set_anti_roll_bar_left_wheel, &JoltVehicleBody::get_anti_roll_bar_left_wheel);
	base_anti_rollbar_property_helper.register_property(PropertyInfo(Variant::NODE_PATH, "right_wheel", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "JoltVehicleWheelBase"), defaults.right, &JoltVehicleBody::set_anti_roll_bar_right_wheel, &JoltVehicleBody::get_anti_roll_bar_right_wheel);
	base_anti_rollbar_property_helper.register_property(PropertyInfo(Variant::FLOAT, "stiffness"), defaults.stiffness, &JoltVehicleBody::set_anti_roll_bar_stiffness, &JoltVehicleBody::get_anti_roll_bar_stiffness);
	PropertyListHelper::register_base_helper(get_class_static(), &base_anti_rollbar_property_helper);
}

void JoltVehicleBody::_space_changed(const RID &p_new_space) {
	if (p_new_space.is_valid()) {
		init_constraint();
	} else {
		deinit_constraint();
	}
}

void JoltVehicleBody::_body_state_changed(PhysicsDirectBodyState3D *p_state) {
	RigidBody3D::_body_state_changed(p_state);

	PhysicsDirectSpaceState3D *space_state = p_state->get_space_state();
	for (auto *wheel_node : child_wheels) {
		wheel_node->update_wheel_runtime(m_vehicle_constraint, space_state);
	}
}

int JoltVehicleBody::add_anti_roll_bar() {
	anti_roll_bars.push_back(AntiRollBarNodeSettings{ NodePath(), NodePath(), 1000.f });

	notify_property_list_changed();

	return anti_roll_bars.size() - 1;
}

void JoltVehicleBody::set_anti_roll_bar_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);
	anti_roll_bars.resize(p_count);

	notify_property_list_changed();
}

int JoltVehicleBody::get_anti_roll_bar_count() const {
	return anti_roll_bars.size();
}

void JoltVehicleBody::remove_anti_roll_bar(int p_idx) {
	ERR_FAIL_INDEX(p_idx, anti_roll_bars.size());
	anti_roll_bars.remove_at(p_idx);

	notify_property_list_changed();
}

void JoltVehicleBody::move_anti_roll_bar(int p_from_idx, int p_to_idx) {
	ERR_FAIL_INDEX(p_from_idx, anti_roll_bars.size());
	ERR_FAIL_INDEX(p_to_idx, anti_roll_bars.size());

	AntiRollBarNodeSettings temp = anti_roll_bars[p_from_idx];
	anti_roll_bars.remove_at(p_from_idx);
	anti_roll_bars.insert(p_to_idx, temp);

	notify_property_list_changed();
}
void JoltVehicleBody::set_anti_roll_bar_left_wheel(int p_idx, NodePath left_wheel) {
	ERR_FAIL_INDEX(p_idx, anti_roll_bars.size());
	anti_roll_bars[p_idx].left = left_wheel;
}
NodePath JoltVehicleBody::get_anti_roll_bar_left_wheel(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, anti_roll_bars.size(), NodePath());
	return anti_roll_bars[p_idx].left;
}
void JoltVehicleBody::set_anti_roll_bar_right_wheel(int p_idx, NodePath right_wheel) {
	ERR_FAIL_INDEX(p_idx, anti_roll_bars.size());
	anti_roll_bars[p_idx].right = right_wheel;
}
NodePath JoltVehicleBody::get_anti_roll_bar_right_wheel(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, anti_roll_bars.size(), NodePath());
	return anti_roll_bars[p_idx].right;
}
void JoltVehicleBody::set_anti_roll_bar_stiffness(int p_idx, float stiffness) {
	ERR_FAIL_INDEX(p_idx, anti_roll_bars.size());
	anti_roll_bars[p_idx].stiffness = stiffness;
}
float JoltVehicleBody::get_anti_roll_bar_stiffness(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, anti_roll_bars.size(), 0.0f);
	return anti_roll_bars[p_idx].stiffness;
}

void JoltVehicleBody::_notification(int p_what) {
	RigidBody3D::_notification(p_what);

	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {
			set_physics_process_internal(true);
		} break;
		case NOTIFICATION_INTERNAL_PHYSICS_PROCESS: {
			if (_is_dirty) {
				_is_dirty = false;
				init_constraint();
			}
		} break;
	}
}

void JoltVehicleBody::_init_wheels() {
	ERR_FAIL_COND(!is_inside_tree());

	m_constraint_settings.mWheels.clear();

	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	ERR_FAIL_NULL(server);

	RID physics_space =	server->body_get_space(get_rid());
	ERR_FAIL_COND(physics_space.is_null());
	for (JoltVehicleWheelBase *wheel_c : child_wheels) {
		auto *wheel = static_cast<JoltVehicleWheelBase *>(wheel_c);
		wheel->wheel_index = m_constraint_settings.mWheels.size();
		wheel->wheel_physics_space = physics_space;
		m_constraint_settings.mWheels.push_back(wheel->settings);
	}
}
void JoltVehicleBody::_finalize_constraint() {
	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	ERR_FAIL_NULL(server);
	
	auto *body_impl = server->get_body(get_rid());
	auto *space = body_impl->get_space();
	auto &system = space->get_physics_system();

	auto body = space->try_get_jolt_body(body_impl->get_jolt_id());
	ERR_FAIL_NULL(body);

	m_vehicle_constraint = new JPH::VehicleConstraint(*body, m_constraint_settings);
	m_vehicle_constraint->SetVehicleCollisionTester(new JPH::VehicleCollisionTesterCastCylinder(body->GetObjectLayer(), 0.2f));

	for (auto wheel_node : child_wheels) {
		wheel_node->parent_constraint = m_vehicle_constraint;
	}

	system.AddConstraint(m_vehicle_constraint);
	system.AddStepListener(m_vehicle_constraint);
}

void JoltVehicleBody::deinit_constraint() {
	if (m_vehicle_constraint == nullptr) {
		return;
	}

	JoltPhysicsServer3D *server = JoltPhysicsServer3D::get_singleton();
	ERR_FAIL_NULL(server);

	auto *body_impl = server->get_body(get_rid());
	if (!body_impl) {
		return;
	}

	auto *space = body_impl->get_space();
	if (!space) {
		return;
	}

	auto &system = space->get_physics_system();

	system.RemoveStepListener(m_vehicle_constraint);
	system.RemoveConstraint(m_vehicle_constraint);
	m_vehicle_constraint = nullptr;
}

void JoltVehicleBody::init_anti_roll_bars() {
	// Add Anti-Roll bars
	m_constraint_settings.mAntiRollBars.clear();
	for (AntiRollBarNodeSettings &roll_bar : anti_roll_bars) {
		auto *left_wheel = Object::cast_to<JoltVehicleWheelBase>(get_node(roll_bar.left));
		auto *right_wheel = Object::cast_to<JoltVehicleWheelBase>(get_node(roll_bar.right));

		if (!left_wheel) {
			WARN_PRINT("left_wheel is not assigned to a JoltVehicleWheelBase");
		}
		if (!right_wheel) {
			WARN_PRINT("right_wheel is not assigned to a JoltVehicleWheelBase");
		}

		if (!left_wheel || !right_wheel) {
			continue;
		}

		auto jolt_roll_bar = JPH::VehicleAntiRollBar();

		jolt_roll_bar.mLeftWheel = left_wheel->wheel_index;
		jolt_roll_bar.mRightWheel = right_wheel->wheel_index;
		jolt_roll_bar.mStiffness = roll_bar.stiffness;

		m_constraint_settings.mAntiRollBars.push_back(jolt_roll_bar);
	}
}

void JoltVehicleBodyWheeled::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_driver_input", "forward", "right", "brake", "handbrake"), &JoltVehicleBodyWheeled::set_driver_input);

	ClassDB::bind_method(D_METHOD("add_differential"), &JoltVehicleBodyWheeled::add_differential);
	ClassDB::bind_method(D_METHOD("remove_differential", "index"), &JoltVehicleBodyWheeled::remove_differential);

	ClassDB::bind_method(D_METHOD("set_differential_count", "count"), &JoltVehicleBodyWheeled::set_differential_count);
	ClassDB::bind_method(D_METHOD("get_differential_count"), &JoltVehicleBodyWheeled::get_differential_count);

	ClassDB::bind_method(D_METHOD("move_differential", "from", "to"), &JoltVehicleBodyWheeled::move_differential);


	ClassDB::bind_method(D_METHOD("set_differential_left_wheel", "index", "left_wheel"), &JoltVehicleBodyWheeled::set_differential_left_wheel);
	ClassDB::bind_method(D_METHOD("get_differential_left_wheel", "index"), &JoltVehicleBodyWheeled::get_differential_left_wheel);

	ClassDB::bind_method(D_METHOD("set_differential_right_wheel", "index", "right_wheel"), &JoltVehicleBodyWheeled::set_differential_right_wheel);
	ClassDB::bind_method(D_METHOD("get_differential_right_wheel", "index"), &JoltVehicleBodyWheeled::get_differential_right_wheel);

	ClassDB::bind_method(D_METHOD("set_differential_ratio", "index", "ratio"), &JoltVehicleBodyWheeled::set_differential_ratio);
	ClassDB::bind_method(D_METHOD("get_differential_ratio", "index"), &JoltVehicleBodyWheeled::get_differential_ratio);

	ClassDB::bind_method(D_METHOD("set_differential_left_right_split", "index", "ratio"), &JoltVehicleBodyWheeled::set_differential_left_right_split);
	ClassDB::bind_method(D_METHOD("get_differential_left_right_split", "index"), &JoltVehicleBodyWheeled::get_differential_left_right_split);

	ClassDB::bind_method(D_METHOD("set_differential_limited_slip_ratio", "index", "ratio"), &JoltVehicleBodyWheeled::set_differential_limited_slip_ratio);
	ClassDB::bind_method(D_METHOD("get_differential_limited_slip_ratio", "index"), &JoltVehicleBodyWheeled::get_differential_limited_slip_ratio);

	ClassDB::bind_method(D_METHOD("set_differential_engine_torque_ratio", "index", "ratio"), &JoltVehicleBodyWheeled::set_differential_engine_torque_ratio);
	ClassDB::bind_method(D_METHOD("get_differential_engine_torque_ratio", "index"), &JoltVehicleBodyWheeled::get_differential_engine_torque_ratio);

	ADD_ARRAY_COUNT("Differentials", "differential_count", "set_differential_count", "get_differential_count", "differential_");

	DifferentialSettings defaults;

	base_differential_helper.set_prefix("differential_");
	base_differential_helper.set_array_length_getter(&JoltVehicleBodyWheeled::get_differential_count);
	base_differential_helper.register_property(PropertyInfo(Variant::NODE_PATH, "left_wheel", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "JoltVehicleWheelBase"),
			defaults.left_wheel,
			&JoltVehicleBodyWheeled::set_differential_left_wheel,
			&JoltVehicleBodyWheeled::get_differential_left_wheel);
	base_differential_helper.register_property(PropertyInfo(Variant::NODE_PATH, "right_wheel", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "JoltVehicleWheelBase"),
			defaults.right_wheel,
			&JoltVehicleBodyWheeled::set_differential_right_wheel,
			&JoltVehicleBodyWheeled::get_differential_right_wheel);
	base_differential_helper.register_property(PropertyInfo(Variant::FLOAT, "differential_ratio", PROPERTY_HINT_RANGE, "0,10.0,or_greater"),
			defaults.differential_ratio,
			&JoltVehicleBodyWheeled::set_differential_ratio,
			&JoltVehicleBodyWheeled::get_differential_ratio);
	base_differential_helper.register_property(PropertyInfo(Variant::FLOAT, "left_right_split", PROPERTY_HINT_RANGE, "0,1.0"),
			defaults.left_right_split,
			&JoltVehicleBodyWheeled::set_differential_left_right_split,
			&JoltVehicleBodyWheeled::get_differential_left_right_split);
	base_differential_helper.register_property(PropertyInfo(Variant::FLOAT, "limited_slip_ratio", PROPERTY_HINT_RANGE, "1.0,2.0,or_greater"),
			defaults.limited_slip_ratio,
			&JoltVehicleBodyWheeled::set_differential_limited_slip_ratio,
			&JoltVehicleBodyWheeled::get_differential_limited_slip_ratio);
	base_differential_helper.register_property(PropertyInfo(Variant::FLOAT, "engine_torque_ratio", PROPERTY_HINT_RANGE, "0.0,1.0,or_greater"),
			defaults.engine_torque_ratio,
			&JoltVehicleBodyWheeled::set_differential_engine_torque_ratio,
			&JoltVehicleBodyWheeled::get_differential_engine_torque_ratio);

	PropertyListHelper::register_base_helper(get_class_static(), &base_differential_helper);
}

void JoltVehicleBodyWheeled::init_constraint(bool keep_state) {
	deinit_constraint();

	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	auto *controller = static_cast<JPH::WheeledVehicleControllerSettings *>(m_constraint_settings.mController.GetPtr());

	if (get_transmission().is_valid()) {
		get_transmission()->write_settings(&controller->mTransmission);
	}

	if (get_engine().is_valid()) {
		get_engine()->write_settings(&controller->mEngine);
	}

	// Base wheel initialization
	_init_wheels();

	controller->mDifferentials.clear();
	for (DifferentialSettings &in_diff : diffs) {

		auto *left_wheel = Object::cast_to<JoltVehicleWheelBase>(get_node(in_diff.left_wheel));
		auto *right_wheel = Object::cast_to<JoltVehicleWheelBase>(get_node(in_diff.right_wheel));

		if (!left_wheel) {
			WARN_PRINT("left_wheel is not assigned to a JoltVehicleWheelBase");
		}
		if (!right_wheel) {
			WARN_PRINT("right_wheel is not assigned to a JoltVehicleWheelBase");
		}

		if (!left_wheel || !right_wheel) {
			continue;
		}

		JPH::VehicleDifferentialSettings diff;

		diff.mLeftWheel = left_wheel->wheel_index;
		diff.mRightWheel = right_wheel->wheel_index;
		diff.mDifferentialRatio = in_diff.differential_ratio;
		diff.mLeftRightSplit = in_diff.left_right_split;
		diff.mLimitedSlipRatio = in_diff.limited_slip_ratio;
		diff.mEngineTorqueRatio = in_diff.engine_torque_ratio;

		controller->mDifferentials.push_back(diff);
	}

	init_anti_roll_bars();

	_finalize_constraint();
}

int JoltVehicleBodyWheeled::add_differential() {
	diffs.push_back( DifferentialSettings());

	notify_property_list_changed();

	return diffs.size() - 1;
}

void JoltVehicleBodyWheeled::remove_differential(int p_idx) {
	ERR_FAIL_INDEX(p_idx, diffs.size());

	diffs.remove_at(p_idx);

	notify_property_list_changed();
}

void JoltVehicleBodyWheeled::set_differential_count(int p_count) {
	ERR_FAIL_COND(p_count < 0);

	diffs.resize(p_count);

	notify_property_list_changed();
}

int JoltVehicleBodyWheeled::get_differential_count() const {
	return diffs.size();
}

void JoltVehicleBodyWheeled::move_differential(int p_from_idx, int p_to_idx) {
	ERR_FAIL_INDEX(p_from_idx, diffs.size());
	ERR_FAIL_INDEX(p_to_idx, diffs.size());

	DifferentialSettings temp = diffs[p_from_idx];
	diffs.remove_at(p_from_idx);
	diffs.insert(p_to_idx, temp);

	notify_property_list_changed();
}

void JoltVehicleBodyWheeled::set_differential_left_wheel(int p_idx, NodePath left_wheel) {
	ERR_FAIL_INDEX(p_idx, diffs.size());
	diffs[p_idx].left_wheel = left_wheel;
}
NodePath JoltVehicleBodyWheeled::get_differential_left_wheel(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, diffs.size(), NodePath());
	return diffs[p_idx].left_wheel;
}

void JoltVehicleBodyWheeled::set_differential_right_wheel(int p_idx, NodePath right_wheel) {
	ERR_FAIL_INDEX(p_idx, diffs.size());
	diffs[p_idx].right_wheel = right_wheel;
}
NodePath JoltVehicleBodyWheeled::get_differential_right_wheel(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, diffs.size(), NodePath());
	return diffs[p_idx].right_wheel;
}

void JoltVehicleBodyWheeled::set_differential_ratio(int p_idx, float p_differential_ratio) {
	ERR_FAIL_INDEX(p_idx, diffs.size());
	diffs[p_idx].differential_ratio = p_differential_ratio;
}
float JoltVehicleBodyWheeled::get_differential_ratio(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, diffs.size(), 0.0);
	return diffs[p_idx].differential_ratio;
}

void JoltVehicleBodyWheeled::set_differential_left_right_split(int p_idx, float p_left_right_split) {
	ERR_FAIL_INDEX(p_idx, diffs.size());
	diffs[p_idx].left_right_split = p_left_right_split;
}
float JoltVehicleBodyWheeled::get_differential_left_right_split(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, diffs.size(), 0.0);
	return diffs[p_idx].left_right_split;
}

void JoltVehicleBodyWheeled::set_differential_limited_slip_ratio(int p_idx, float p_limited_slip_ratio) {
	ERR_FAIL_INDEX(p_idx, diffs.size());
	diffs[p_idx].limited_slip_ratio = p_limited_slip_ratio;
}
float JoltVehicleBodyWheeled::get_differential_limited_slip_ratio(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, diffs.size(), 0.0);
	return diffs[p_idx].limited_slip_ratio;
}

void JoltVehicleBodyWheeled::set_differential_engine_torque_ratio(int p_idx, float p_engine_torque_ratio) {
	ERR_FAIL_INDEX(p_idx, diffs.size());
	diffs[p_idx].engine_torque_ratio = p_engine_torque_ratio;

}
float JoltVehicleBodyWheeled::get_differential_engine_torque_ratio(int p_idx) {
	ERR_FAIL_INDEX_V(p_idx, diffs.size(), 0.0);
	return diffs[p_idx].engine_torque_ratio;
}

void JoltVehicleBodyTracked::init_constraint(bool keep_state) {
	if (keep_state && m_vehicle_constraint) {
		// Save whatever state the constraint has that'll be reset when we delete it.
		// TODO determine what will break here, and what's fine to break (e.g. when adding/removing wheels)
	}
	deinit_constraint();

	// This can't be done in JoltVehicleBody* because the engine and transmission belong to the controller subclass.
	auto *controller = static_cast<JPH::TrackedVehicleControllerSettings *>(m_constraint_settings.mController.GetPtr());

	if (get_transmission().is_valid()) {
		get_transmission()->write_settings(&controller->mTransmission);
	}

	if (get_engine().is_valid()) {
		get_engine()->write_settings(&controller->mEngine);
	}

	// Base wheel initialization
	_init_wheels();

	// Add tracks
	for (JPH::VehicleTrackSettings &track : controller->mTracks) { track.mWheels.clear(); }
	for (JoltVehicleWheelBase *wheel_c : child_wheels) {
		auto* wheel = dynamic_cast<JoltVehicleWheelTracked *>(wheel_c);
		ERR_CONTINUE_MSG(wheel == nullptr, "TrackedVehicleBody wheel must be VehicleWheelTracked");

		if (wheel->track_side != JoltVehicleWheelTracked::TRACK_SIDE_NONE) {
			if (wheel->track_side != JoltVehicleWheelTracked::TRACK_SIDE_LEFT && wheel->track_side != JoltVehicleWheelTracked::TRACK_SIDE_RIGHT) {
				ERR_PRINT("Invalid track side in JoltVehicleWheelTracked");
				print_line(wheel->track_side);
			} else {
				controller->mTracks[wheel->track_side].mWheels.push_back(wheel->wheel_index);
				controller->mTracks[wheel->track_side].mDrivenWheel = wheel->wheel_index;
			}
		}
	}

	// Set driven wheel for each track.
	{
		for (int i = 0; i < 2; i++) {
			auto* wheel_node = dynamic_cast<JoltVehicleWheelTracked*>(get_node_or_null(driven_wheels[i]));
			// Check the wheel is actually a child of ours, otherwise wheel_index will be wrong.
			if (wheel_node && child_wheels.has(wheel_node)) {
				controller->mTracks[i].mDrivenWheel = wheel_node->wheel_index;
			}
		}
	}

	init_anti_roll_bars();

	_finalize_constraint();

	// TODO Try rendering constraints to the rendering server for funsies
}

JoltVehicleBodyTracked::JoltVehicleBodyTracked() {
	m_constraint_settings.mController = new JPH::TrackedVehicleControllerSettings;
}

void JoltVehicleBodyTracked::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_track_driven_wheel", "track_side", "wheel"), &JoltVehicleBodyTracked::set_track_driven_wheel);
	ClassDB::bind_method(D_METHOD("get_track_driven_wheel", "track_side"), &JoltVehicleBodyTracked::get_track_driven_wheel);

	BIND_SETGET(JoltVehicleBodyTracked, track_inertia);
	BIND_SETGET(JoltVehicleBodyTracked, track_angular_damping);
	BIND_SETGET(JoltVehicleBodyTracked, track_max_brake_torque);
	BIND_SETGET(JoltVehicleBodyTracked, track_differential_ratio);

	ADD_PROPERTYI(PropertyInfo(Variant::NODE_PATH, "left_track/driven_wheel", PROPERTY_HINT_NODE_TYPE, "JoltVehicleWheelTracked"), "set_track_driven_wheel", "get_track_driven_wheel", JoltVehicleWheelTracked::TrackSide::TRACK_SIDE_LEFT);
	ADD_PROPERTYI(PropertyInfo(Variant::NODE_PATH, "right_track/driven_wheel", PROPERTY_HINT_NODE_TYPE, "JoltVehicleWheelTracked"), "set_track_driven_wheel", "get_track_driven_wheel", JoltVehicleWheelTracked::TrackSide::TRACK_SIDE_RIGHT);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "track_inertia", PROPERTY_HINT_RANGE, "0,10,or_greater,hide_slider,suffix:kgm^2"), "set_track_inertia", "get_track_inertia");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "track_angular_damping", PROPERTY_HINT_RANGE, "0,1"), "set_track_angular_damping", "get_track_angular_damping");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "track_max_brake_torque", PROPERTY_HINT_RANGE, "0,500,or_greater,suffix:Nm"), "set_track_max_brake_torque", "get_track_max_brake_torque");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "track_differential_ratio", PROPERTY_HINT_RANGE, "0,10.0,or_greater"), "set_track_differential_ratio", "get_track_differential_ratio");

	ClassDB::bind_method(D_METHOD("set_driver_input", "forward_axis", "left_ratio", "right_ratio", "brake"), &JoltVehicleBodyTracked::set_driver_input);
}

void JoltVehicleBodyTracked::set_driver_input(float forward, float left, float right, float brake) {
	ERR_FAIL_NULL(m_vehicle_constraint);

	set_sleeping(false);

	static_cast<JPH::TrackedVehicleController *>(m_vehicle_constraint->GetController())->SetDriverInput(forward, left, right, brake);
}
