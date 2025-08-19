/**************************************************************************/
/*  register_types.cpp                                                    */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "register_types.h"

#include "jolt_globals.h"
#include "jolt_physics_server_3d.h"
#include "jolt_project_settings.h"

#include "core/config/project_settings.h"
#include "core/object/callable_mp.h"

#include "modules/jolt_physics/nodes/jolt_vehicle_body.h"

#include "modules/jolt_physics/spaces/jolt_physics_direct_space_state_3d.h"
#include "servers/physics_3d/physics_server_3d_wrap_mt.h"

PhysicsServer3D *create_jolt_physics_server() {
#ifdef THREADS_ENABLED
	bool run_on_separate_thread = GLOBAL_GET("physics/3d/run_on_separate_thread");
#else
	bool run_on_separate_thread = false;
#endif

	JoltPhysicsServer3D *physics_server = memnew(JoltPhysicsServer3D(run_on_separate_thread));

	return memnew(PhysicsServer3DWrapMT(physics_server, run_on_separate_thread));
}

void initialize_jolt_physics_module(ModuleInitializationLevel p_level) {
	switch (p_level) {
		case MODULE_INITIALIZATION_LEVEL_CORE: {
		} break;
		case MODULE_INITIALIZATION_LEVEL_SERVERS: {
			jolt_initialize();
			PhysicsServer3DManager::get_singleton()->register_server("Jolt Physics", callable_mp_static(&create_jolt_physics_server));
			JoltProjectSettings::register_settings();
			GDREGISTER_CLASS(JoltPhysicsDirectSpaceState3D)
		} break;
		case MODULE_INITIALIZATION_LEVEL_SCENE: {
			GDREGISTER_CLASS(JoltVehicleEngineSettings)
			GDREGISTER_CLASS(JoltVehicleTransmissionSettings)

			GDREGISTER_ABSTRACT_CLASS(JoltVehicleWheelBase)
			GDREGISTER_CLASS(JoltVehicleWheel)
			GDREGISTER_CLASS(JoltVehicleWheelTracked)

			GDREGISTER_ABSTRACT_CLASS(JoltVehicleBody)
			GDREGISTER_CLASS(JoltVehicleBodyWheeled)
			GDREGISTER_CLASS(JoltVehicleBodyMotorcycle);
			GDREGISTER_CLASS(JoltVehicleBodyTracked)
		} break;
		case MODULE_INITIALIZATION_LEVEL_EDITOR: {
		} break;
	}
}

void uninitialize_jolt_physics_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SERVERS) {
		return;
	}

	jolt_deinitialize();
}
