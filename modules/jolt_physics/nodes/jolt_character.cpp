#include "jolt_character.h"

#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
#include "Jolt/Physics/Collision/Shape/CylinderShape.h"
#include "Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"

#include "modules/jolt_physics/jolt_physics_server_3d.h"
#include "modules/jolt_physics/spaces/jolt_broad_phase_layer.h"

#define STRINGIFY(a) #a
#define BIND_SETGET(cls, name)                                                      \
	ClassDB::bind_method(D_METHOD(STRINGIFY(set_##name), #name), &cls::set_##name); \
	ClassDB::bind_method(D_METHOD(STRINGIFY(get_##name)), &cls::get_##name)

void JoltCharacter::_bind_methods() {
	BIND_SETGET(JoltCharacter, linear_velocity);
	BIND_SETGET(JoltCharacter, max_slope_angle);
	BIND_SETGET(JoltCharacter, mass);
	BIND_SETGET(JoltCharacter, max_strength);
	BIND_SETGET(JoltCharacter, collision_layer);
	BIND_SETGET(JoltCharacter, collision_mask);

	BIND_SETGET(JoltCharacter, stick_to_floor_step_down);
	BIND_SETGET(JoltCharacter, stairs_step_up);
	BIND_SETGET(JoltCharacter, stairs_min_step_forward);
	BIND_SETGET(JoltCharacter, stairs_step_forward_test);
	BIND_SETGET(JoltCharacter, stairs_step_down_extra);

	BIND_SETGET(JoltCharacter, radius);
	BIND_SETGET(JoltCharacter, height);
	ClassDB::bind_method(D_METHOD("try_set_radius"), &JoltCharacter::try_set_radius);
	ClassDB::bind_method(D_METHOD("try_set_height"), &JoltCharacter::try_set_height);

	ClassDB::bind_method(D_METHOD("is_supported"), &JoltCharacter::is_supported);
	ClassDB::bind_method(D_METHOD("move_and_step"), &JoltCharacter::move_and_step);
	ClassDB::bind_method(D_METHOD("get_ground_normal"), &JoltCharacter::get_ground_normal);
	ClassDB::bind_method(D_METHOD("get_ground_velocity"), &JoltCharacter::get_ground_velocity);
	ClassDB::bind_method(D_METHOD("get_ground_state"), &JoltCharacter::get_ground_state);

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity", PROPERTY_HINT_NONE, U"suffix:m/s", PROPERTY_USAGE_NO_EDITOR), "set_linear_velocity", "get_linear_velocity");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_slope_angle", PROPERTY_HINT_RANGE, U"0,90,0.01,radians_as_degrees"), "set_max_slope_angle", "get_max_slope_angle");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mass", PROPERTY_HINT_RANGE, U"0,100,or_greater,suffix:kg"), "set_mass", "get_mass");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_strength", PROPERTY_HINT_RANGE, U"0,1000,or_greater,suffix:N"), "set_max_strength", "get_max_strength");

	ADD_GROUP("Shape", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius", PROPERTY_HINT_RANGE, U"0,0.5,or_greater,suffix:m"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height", PROPERTY_HINT_RANGE, U"0,2.0,or_greater,suffix:m"), "set_height", "get_height");


	ADD_GROUP("Collision", "");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ADD_GROUP("Stair Stepping", "stairs");

	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "stairs_step_up", PROPERTY_HINT_NONE, U"suffix:m"), "set_stairs_step_up", "get_stairs_step_up");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stairs_min_step_forward", PROPERTY_HINT_NONE, U"suffix:m"), "set_stairs_min_step_forward", "get_stairs_min_step_forward");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stairs_step_forward_test", PROPERTY_HINT_NONE, U"suffix:m"), "set_stairs_step_forward_test", "get_stairs_step_forward_test");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "stairs_step_down_extra", PROPERTY_HINT_NONE, U"suffix:m"), "set_stairs_step_down_extra", "get_stairs_step_down_extra");

	ADD_GROUP("Stick to Floor", "stick_to_floor");
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "stick_to_floor_step_down", PROPERTY_HINT_NONE, U"suffix:m"), "set_stick_to_floor_step_down", "get_stick_to_floor_step_down");


	BIND_ENUM_CONSTANT(ON_GROUND);
	BIND_ENUM_CONSTANT(ON_STEEP_GROUND);
	BIND_ENUM_CONSTANT(ON_WALL);
	BIND_ENUM_CONSTANT(NONE);
}

void JoltCharacter::_notification(int p_what) {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}

	switch (p_what) {
		case NOTIFICATION_ENTER_WORLD: {
			init_character();
			break;
		}
		case NOTIFICATION_EXIT_WORLD: {
			deinit_character();
			break;
		}
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED:
		case NOTIFICATION_TRANSFORM_CHANGED: {
			character_->SetPosition(to_jolt(get_global_position()));
		}
		default:
			break;
	}
}

void JoltCharacter::init_character() {
	JoltPhysicsServer3D *jolt_server = JoltPhysicsServer3D::get_singleton();
	ERR_FAIL_NULL(jolt_server);

	Ref<World3D> world_ref = get_world_3d();
	ERR_FAIL_COND(world_ref.is_null());

	RID space = world_ref->get_space();
	space_3d_ = jolt_server->get_space(space);
	physics_system_ = &space_3d_->get_physics_system();

	JPH::Ref settings = new JPH::CharacterVirtualSettings();
	settings->mShape = new JPH::RotatedTranslatedShape(JPH::RVec3(0.0, height_ / 2.0, 0.0), JPH::QuatArg::sIdentity(), new JPH::CylinderShape(height_ / 2.0, radius_));
	settings->mEnhancedInternalEdgeRemoval = true;
	settings->mSupportingVolume = { JPH::Vec3::sAxisY(), -0.25};

	character_ = new JPH::CharacterVirtual(settings, to_jolt(get_position()), to_jolt(Quaternion()), physics_system_);
	character_->SetListener(this);
}

void JoltCharacter::deinit_character() {
	character_ = nullptr;
}
bool JoltCharacter::try_set_shape(float radius, float height) {
	if (character_ == nullptr) { return true; }

	float capsule_half_height = (height / 2.0) - radius;

	// JPH::RefConst<JPH::Shape> new_shape = new JPH::RotatedTranslatedShape(JPH::RVec3(0.0, height / 2.0, 0.0), JPH::QuatArg::sIdentity(), new JPH::CylinderShape(height / 2.0, radius));
	JPH::RefConst<JPH::Shape> new_shape = new JPH::RotatedTranslatedShape(JPH::RVec3(0.0, height / 2.0, 0.0), JPH::QuatArg::sIdentity(), new JPH::CapsuleShape(capsule_half_height, radius));
	JPH::ObjectLayer collision_layer = space_3d_->map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, 0, collision_mask_);

	// TODO try_shape penetration depth needs to be a property
	return character_->SetShape(new_shape, 0.05, physics_system_->GetDefaultBroadPhaseLayerFilter(collision_layer), physics_system_->GetDefaultLayerFilter(collision_layer), {}, {}, *allocator_);
}

void JoltCharacter::move_and_step() {
	auto gravity = -JPH::Vec3Arg(0.0, 9.81, 0.0);
	float delta = get_physics_process_delta_time();

	character_->SetPosition(to_jolt(get_global_position()));

	JPH::ObjectLayer collision_layer = space_3d_->map_to_object_layer(JoltBroadPhaseLayer::BODY_DYNAMIC, 0, collision_mask_);

	character_->ExtendedUpdate(delta,
			gravity,
			stair_settings_,
			physics_system_->GetDefaultBroadPhaseLayerFilter(collision_layer),
			physics_system_->GetDefaultLayerFilter(collision_layer),
			{ },
			{ },
			*allocator_);

	set_ignore_transform_notification(true);
	set_global_position(to_godot(character_->GetPosition()));
	set_ignore_transform_notification(false);
}
