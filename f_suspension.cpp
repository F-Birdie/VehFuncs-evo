#include "plugin.h"
#include "CVehicle.h"
#include "CAutomobile.h"
#include "RenderWare.h"
#include "NodeName.h"
#include <unordered_map>
#include <cstring>

using namespace plugin;

// ===================== TUNABLE =====================
const float ROTATION_PER_UNIT = 120.0f;
const float STRETCH_PER_UNIT = -1.5f;
const float STRETCH_MIN = 0.70f;
const float STRETCH_MAX = 1.35f;
// ===================================================

struct SuspOriginalMatrix
{
    RwMatrix mat;
    bool stored = false;
};

struct WheelRestZ
{
    float z[4] = {};
    bool initialized = false;
};

static std::unordered_map<RwFrame*, SuspOriginalMatrix> g_suspMatrices;
static std::unordered_map<CVehicle*, WheelRestZ> g_wheelRestZ;

void ApplySuspensionTransform(RwFrame* frame, float angle, float scale)
{
    if (!frame) return;

    auto& entry = g_suspMatrices[frame];
    if (!entry.stored)
    {
        entry.mat = *RwFrameGetMatrix(frame);
        entry.stored = true;
    }

    RwMatrix* mat = RwFrameGetMatrix(frame);
    *mat = entry.mat;

    // Rotate
    RwV3d axis = { 0.0f, 1.0f, 0.0f };
    RwMatrixRotate(mat, &axis, angle, rwCOMBINEPRECONCAT);

    // Stretch
    mat->at.x *= scale;
    mat->at.y *= scale;
    mat->at.z *= scale;

    RwMatrixUpdate(mat);
    RwFrameUpdateObjects(frame);
}

struct SuspInfo
{
    int wheel;
    float sign;
};

SuspInfo GetSuspensionInfo(const char* name)
{
    if (!name) return { -1, 1.0f };

    if (_strnicmp(name, "suspension_lf", 13) == 0) return { 0,  1.0f };
    if (_strnicmp(name, "suspension_rf", 13) == 0) return { 2, -1.0f };
    if (_strnicmp(name, "suspension_lr", 13) == 0) return { 1,  1.0f };
    if (_strnicmp(name, "suspension_lb", 13) == 0) return { 1,  1.0f };
    if (_strnicmp(name, "suspension_rr", 13) == 0) return { 3, -1.0f };
    if (_strnicmp(name, "suspension_rb", 13) == 0) return { 3, -1.0f };

    return { -1, 1.0f };
}

eCarNodes WheelToNode(int wheel)
{
    switch (wheel)
    {
    case 0: return CAR_WHEEL_LF;
    case 1: return CAR_WHEEL_LB;
    case 2: return CAR_WHEEL_RF;
    case 3: return CAR_WHEEL_RB;
    default: return CAR_NODE_NONE;
    }
}

class SuspensionTransform
{
public:
    SuspensionTransform()
    {
        Events::vehicleDtorEvent.before += [](CVehicle* vehicle)
            {
                g_wheelRestZ.erase(vehicle);
            };

        Events::vehicleRenderEvent.before += [](CVehicle* vehicle)
            {
                if (!vehicle || !vehicle->m_pRwClump)
                    return;

                if (vehicle->m_nVehicleClass != VEHICLE_AUTOMOBILE &&
                    vehicle->m_nVehicleClass != VEHICLE_MTRUCK &&
                    vehicle->m_nVehicleClass != VEHICLE_QUAD)
                    return;

                CAutomobile* autoMobile = static_cast<CAutomobile*>(vehicle);

                WheelRestZ& rest = g_wheelRestZ[vehicle];
                if (!rest.initialized)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        RwFrame* wheelFrame = autoMobile->m_aCarNodes[WheelToNode(i)];
                        if (wheelFrame)
                            rest.z[i] = RwFrameGetMatrix(wheelFrame)->pos.z;
                        else
                            rest.z[i] = 0.0f;
                    }
                    rest.initialized = true;
                }

                struct Data {
                    CAutomobile* autoMobile;
                    WheelRestZ* rest;
                } data = { autoMobile, &rest };

                RpClumpForAllAtomics(vehicle->m_pRwClump,
                    [](RpAtomic* atomic, void* userdata) -> RpAtomic*
                    {
                        Data* d = static_cast<Data*>(userdata);
                        if (!atomic) return atomic;

                        RwFrame* frame = RpAtomicGetFrame(atomic);
                        if (!frame) return atomic;

                        char* name = GetFrameNodeName(frame);
                        if (!name || _strnicmp(name, "suspension_", 11) != 0)
                            return atomic;

                        SuspInfo info = GetSuspensionInfo(name);
                        if (info.wheel < 0) return atomic;

                        RwFrame* wheelFrame = d->autoMobile->m_aCarNodes[WheelToNode(info.wheel)];
                        if (!wheelFrame) return atomic;

                        float currentZ = RwFrameGetMatrix(wheelFrame)->pos.z;
                        float restZ = d->rest->z[info.wheel];
                        float delta = currentZ - restZ;

                        // Rotation (already has correct sign)
                        float angle = delta * ROTATION_PER_UNIT * info.sign;

                        // Stretch – now also uses the same sign
                        float scale = 1.0f + (delta * STRETCH_PER_UNIT * info.sign);
                        if (scale < STRETCH_MIN) scale = STRETCH_MIN;
                        if (scale > STRETCH_MAX) scale = STRETCH_MAX;

                        ApplySuspensionTransform(frame, angle, scale);

                        return atomic;
                    },
                    &data);
            };
    }
} suspensionTransform;
