#include "cg_cleanup.hpp"
#include "cg_local.hpp"
#include "cg_offsets.hpp"
#include "cm/cm_brush.hpp"
#include "cod4x/cod4x.hpp"
#include "dvar/dvar.hpp"
#include "r/r_drawactive.hpp"
#include "r/r_endscene.hpp"
#include "utils/hook.hpp"
#include "com/com_channel.hpp"

#include <cmath>
#include "g/g_trigger.hpp"
#include "menu/menu_parse.hpp"
#include "cl/cl_main.hpp"
#include "g/g_spawn.hpp"

static void CG_TraceBounceDebug(
	trace_t* result,
	const float* start,
	const float* mins,
	const float* maxs,
	const float* end,
	int skipEntity,
	int tracemask)
{
	auto* traceHook = hooktable::find<void, trace_t*, const float*, const float*, const float*, const float*, int, int>(
		HOOK_PREFIX("CG_TraceBounceDebug"));
	traceHook->call(result, start, mins, maxs, end, skipEntity, tracemask);

	static dvar_s* debug = Dvar_FindMalleableVar("pm_bounceTraceDebug");
	if (!debug || !debug->current.enabled || !result || !start || !mins || !maxs || !end)
		return;

	const float traceDx = end[0] - start[0];
	const float traceDy = end[1] - start[1];
	const float traceDz = end[2] - start[2];

	// PM_StepSlideMove's relevant down trace is vertical, short, and uses
	// the player's capsule bounds. Ignore ordinary horizontal movement traces.
	if (std::fabs(traceDx) > 0.01f || std::fabs(traceDy) > 0.01f)
		return;
	if (traceDz >= -0.01f || traceDz < -40.0f)
		return;
	if (std::fabs(mins[0] + maxs[0]) > 0.01f || std::fabs(mins[1] + maxs[1]) > 0.01f)
		return;
	if ((maxs[2] - mins[2]) < 20.0f)
		return;
	if (result->fraction >= 1.0f || result->allsolid)
		return;

	Com_Printf(
		"[bounce-trace] dz=%.3f frac=%.6f normal=(%.6f %.6f %.6f) walkable=%d "
		"startsolid=%d hitType=%d hitId=%u material=%s contents=0x%X\n",
		traceDz,
		result->fraction,
		result->normal[0], result->normal[1], result->normal[2],
		result->walkable ? 1 : 0,
		result->startsolid ? 1 : 0,
		static_cast<int>(result->hitType),
		static_cast<unsigned>(result->hitId),
		result->material ? result->material : "<null>",
		result->contents);
}

static void CG_CreateHooksInternal();
void CG_CreateHooks()
{
	hooktable::initialize();
	CG_CreateHooksInternal();
}

void CG_CreateHooksInternal()
{
	hooktable::preserver<void>(HOOK_PREFIX("CG_DrawActive"), CoD4X::Get() ? CoD4X::Get() + 0x5464 : 0x42F7F0, CG_DrawActive);
	hooktable::preserver<void, GfxViewParms*>(HOOK_PREFIX("RB_DrawDebug"), 0x658860, RB_DrawDebug);
	hooktable::preserver<void, trace_t*, const float*, const float*, const float*, const float*, int, int>(
		HOOK_PREFIX("CG_TraceBounceDebug"), 0x45A230, CG_TraceBounceDebug);
	hooktable::preserver<void, int>(HOOK_PREFIX("CL_Disconnect"), 0x4696B0, CL_Disconnect);


	hooktable::preserver<void>(HOOK_PREFIX("CG_CleanupHook"), 0x477210, CG_CleanupHook);

	hooktable::overwriter<void>(HOOK_PREFIX("__asm_adjacency_winding"), 0x57D86C, __brush::__asm_adjacency_winding);

	hooktable::overwriter<void>(HOOK_PREFIX("G_Trigger"), 0x4E1130, G_Trigger);
	hooktable::overwriter<void>(HOOK_PREFIX("G_ParseEntityFields"), 0x4DFD88, G_ParseEntityFields);

	hooktable::preserver<void>(HOOK_PREFIX("CG_SetClientDvarFromServerASM"), 0x44BD90, CG_SetClientDvarFromServerASM);
	hooktable::preserver<void>(HOOK_PREFIX("Script_AddTextWrapper"), 0x54D5D0, Script_AddTextWrapper);
	hooktable::preserver<void>(HOOK_PREFIX("Script_ExecHandler"), 0x54D570, Script_ExecHandler);
	hooktable::preserver<void>(HOOK_PREFIX("Script_ConditionalExecHandler"), 0x54D620, Script_ConditionalExecHandler);
	
	//if (dx && dx->device)
	//	hooktable::preserver<long, IDirect3DDevice9*>(HOOK_PREFIX("R_EndScene"),
	//		reinterpret_cast<size_t>((*reinterpret_cast<PVOID**>(dx->device))[42]), R_EndScene);

}
