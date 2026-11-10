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
#include <array>
#include "g/g_trigger.hpp"
#include "menu/menu_parse.hpp"
#include "cl/cl_main.hpp"
#include "g/g_spawn.hpp"

struct BounceTraceSample
{
	float traceDz = 0.0f;
	float fraction = 0.0f;
	float normal[3] = {};
	bool walkable = false;
	bool startsolid = false;
	TraceHitType hitType = TRACE_HITTYPE_NONE;
	unsigned short hitId = 0;
	const char* material = nullptr;
	int contents = 0;
};

static constexpr std::size_t kBounceTraceHistory = 12;
static std::array<BounceTraceSample, kBounceTraceHistory> bounceTraceHistory{};
static std::size_t bounceTraceWriteIndex = 0;
static std::size_t bounceTraceCount = 0;

void CG_DumpBounceTraceDebug()
{
	if (bounceTraceCount == 0) {
		Com_Printf("[bounce-trace] no buffered step traces\n");
		return;
	}

	Com_Printf("[bounce-trace] dumping %u most recent step traces\n",
		static_cast<unsigned>(bounceTraceCount));

	const std::size_t first =
		(bounceTraceWriteIndex + kBounceTraceHistory - bounceTraceCount) % kBounceTraceHistory;

	for (std::size_t i = 0; i < bounceTraceCount; ++i) {
		const auto& sample = bounceTraceHistory[(first + i) % kBounceTraceHistory];
		Com_Printf(
			"[bounce-trace %02u] dz=%.3f frac=%.6f normal=(%.6f %.6f %.6f) walkable=%d "
			"startsolid=%d hitType=%d hitId=%u material=%s contents=0x%X\n",
			static_cast<unsigned>(i),
			sample.traceDz,
			sample.fraction,
			sample.normal[0], sample.normal[1], sample.normal[2],
			sample.walkable ? 1 : 0,
			sample.startsolid ? 1 : 0,
			static_cast<int>(sample.hitType),
			static_cast<unsigned>(sample.hitId),
			sample.material ? sample.material : "<null>",
			sample.contents);
	}

	bounceTraceCount = 0;
}

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

	auto& sample = bounceTraceHistory[bounceTraceWriteIndex];
	sample.traceDz = traceDz;
	sample.fraction = result->fraction;
	sample.normal[0] = result->normal[0];
	sample.normal[1] = result->normal[1];
	sample.normal[2] = result->normal[2];
	sample.walkable = result->walkable;
	sample.startsolid = result->startsolid;
	sample.hitType = result->hitType;
	sample.hitId = result->hitId;
	sample.material = result->material;
	sample.contents = result->contents;

	bounceTraceWriteIndex = (bounceTraceWriteIndex + 1) % kBounceTraceHistory;
	if (bounceTraceCount < kBounceTraceHistory)
		++bounceTraceCount;
}

static void CG_ProjectVelocityBounceDebug()
{
	float* normal = nullptr;
	float* velocity = nullptr;

	__asm mov normal, edi
	__asm mov velocity, esi

	const float inputZ = velocity ? velocity[2] : 0.0f;

	auto* projectHook = hooktable::find<void>(HOOK_PREFIX("PM_ProjectVelocityBounceDebug"));
	projectHook->call();

	static dvar_s* debug = Dvar_FindMalleableVar("pm_bounceTraceDebug");
	if (!debug || !debug->current.enabled || !normal || !velocity)
		return;

	const float outputZ = velocity[2];

	// A real CoD4 bounce is produced when PM_ProjectVelocity turns the
	// incoming movement into upward velocity along the contact plane.
	if (inputZ > 0.0f || outputZ <= 0.0f)
		return;

	Com_Printf(
		"[bounce-detected] velocityZ %.6f -> %.6f normal=(%.6f %.6f %.6f)\n",
		inputZ,
		outputZ,
		normal[0], normal[1], normal[2]);

	CG_DumpBounceTraceDebug();
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
	hooktable::preserver<void>(
		HOOK_PREFIX("PM_ProjectVelocityBounceDebug"), 0x40E330, CG_ProjectVelocityBounceDebug);
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
