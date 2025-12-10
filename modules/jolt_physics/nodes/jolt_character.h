#pragma once

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Character/CharacterVirtual.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "modules/jolt_physics/misc/jolt_type_conversions.h"
#include "modules/jolt_physics/spaces/jolt_space_3d.h"
#include "scene/3d/node_3d.h"

class JoltCharacter : public Node3D, public JPH::CharacterContactListener {
	GDCLASS(JoltCharacter, Node3D);

private:
	JoltCharacter() {
		allocator_ = new JPH::TempAllocatorMalloc();
		settings_.mEnhancedInternalEdgeRemoval = true;
		settings_.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	}
	~JoltCharacter() {
		delete allocator_;
	}

	JPH::Ref<JPH::CharacterVirtual> character_ = nullptr;
	JPH::PhysicsSystem *physics_system_ = nullptr;
	JoltSpace3D *space_3d_ = nullptr;
	JPH::TempAllocator *allocator_ = nullptr;

	JPH::CharacterVirtual::ExtendedUpdateSettings stair_settings_;
	JPH::CharacterVirtualSettings settings_;

	int collision_layer_ = 1;
	int collision_mask_ = 1;

	float height_ = 1.5;
	float radius_ = 0.25;

	void reinitialize() {
		if (character_ != nullptr) {
			auto velocity = character_->GetLinearVelocity();
			auto position = character_->GetPosition();
			auto rotation = character_->GetRotation();

			init_character();

			character_->SetLinearVelocity(velocity);
			character_->SetPosition(position);
			character_->SetRotation(rotation);
		}
	}

protected:
	static void _bind_methods();
	void _notification(int p_what);

	void init_character();
	void deinit_character();
	void sync_character();

	int get_collision_layer() {
		return collision_layer_;
	}
	void set_collision_layer(int collision_layer) {
		collision_layer_ = collision_layer;
	}

	int get_collision_mask() {
		return collision_mask_;
	}
	void set_collision_mask(int collision_mask) {
		collision_mask_ = collision_mask;
	}

	bool try_set_shape(float radius, float height);

	float get_radius() {
		return radius_;
	}
	void set_radius(float radius) {
		radius_ = radius;
	}
	bool try_set_radius(float radius) {
		if (try_set_shape(radius, height_)) {
			set_radius(radius);
			return true;
		}
		return false;
	}

	float get_height() {
		return height_;
	}
	void set_height(float height) {
		height_ = height;
	}
	bool try_set_height(float height) {
		if (try_set_shape(radius_, height)) {
			set_height(height);
			return true;
		}
		return false;
	}

	void set_linear_velocity(Vector3 arg) {
		if (character_ == nullptr) {
			return;
		}

		character_->SetLinearVelocity(to_jolt(arg));
	}
	Vector3 get_linear_velocity() {
		if (character_ == nullptr) {
			return Vector3();
		}

		return to_godot(character_->GetLinearVelocity());
	}

	bool is_supported() {
		ERR_FAIL_NULL_V(character_, false);

		return character_->IsSupported();
	}

	Vector3 get_ground_normal() {
		ERR_FAIL_NULL_V(character_, Vector3());

		return to_godot(character_->GetGroundNormal());
	}

	Vector3 get_ground_velocity() {
		ERR_FAIL_NULL_V(character_, Vector3());

		return to_godot(character_->GetGroundVelocity());
	}

	enum GroundState {
		ON_GROUND,
		ON_STEEP_GROUND,
		ON_WALL,
		NONE
	};

	GroundState get_ground_state() {
		ERR_FAIL_NULL_V(character_, NONE);

		switch (character_->GetGroundState()) {
			case JPH::CharacterVirtual::EGroundState::OnGround:
				return ON_GROUND;
			case JPH::CharacterVirtual::EGroundState::OnSteepGround:
				return ON_STEEP_GROUND;
			case JPH::CharacterVirtual::EGroundState::NotSupported:
				return ON_WALL;
			case JPH::CharacterVirtual::EGroundState::InAir:
			default:
				return NONE;
		}
	}

	/// Like move_and_slide() but with stair stepping
	void move_and_step();


#define SETTINGS_SETGET_F(PropertyName, m_godot_name)                 \
	void set_##m_godot_name(float m_godot_name) {                     \
		settings_.m##PropertyName = m_godot_name;                     \
		if (character_) {                                             \
			character_->Set##PropertyName(settings_.m##PropertyName); \
		}                                                             \
	}                                                                 \
	float get_##m_godot_name() const {                                \
		return settings_.m##PropertyName;                             \
	}

#define STAIR_SETTINGS_SETGET(mPropertyName, m_godot_name, T) \
	void set_##m_godot_name(T m_godot_name) { \
		stair_settings_.mPropertyName = to_jolt(m_godot_name); \
	} \
	T get_##m_godot_name() { \
		return to_godot(stair_settings_.mPropertyName); \
	}
#define STAIR_SETTINGS_SETGET_F(mPropertyName, m_godot_name) \
	void set_##m_godot_name(float m_godot_name) { \
	stair_settings_.mPropertyName = m_godot_name; \
	} \
	float get_##m_godot_name() { \
	return stair_settings_.mPropertyName; \
	}

	STAIR_SETTINGS_SETGET(mStickToFloorStepDown, stick_to_floor_step_down, Vector3);
	STAIR_SETTINGS_SETGET(mWalkStairsStepUp, stairs_step_up, Vector3);
	STAIR_SETTINGS_SETGET_F(mWalkStairsMinStepForward, stairs_min_step_forward);
	STAIR_SETTINGS_SETGET_F(mWalkStairsStepForwardTest, stairs_step_forward_test);
	// TODO custom setter/getter for this
	// STAIR_SETTINGS_SETGET_F(mWalkStairsCosAngleForwardContact, stairs_cos_angle_forward_contact);
	STAIR_SETTINGS_SETGET(mWalkStairsStepDownExtra, stairs_step_down_extra, Vector3);

	SETTINGS_SETGET_F(MaxSlopeAngle, max_slope_angle);
	SETTINGS_SETGET_F(Mass, mass);
	SETTINGS_SETGET_F(MaxStrength, max_strength);

	void set_supporting_volume(Plane arg) {
		settings_.mSupportingVolume = to_jolt(arg);
		reinitialize();
	}
	Plane get_supporting_volume() {
		return to_godot(settings_.mSupportingVolume);
	}
};

VARIANT_ENUM_CAST(JoltCharacter::GroundState);

#undef SETTINGS_SETGET_F
#undef STAIR_SETTINGS_SETGET
#undef STAIR_SETTINGS_SETGET_F